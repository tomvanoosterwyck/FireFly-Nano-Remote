#include "VescUart.h"
#include <HardwareSerial.h>

VescUart::VescUart(void){
	nunchuck.valueX         = 127;
	nunchuck.valueY         = 127;
	nunchuck.lowerButton  	= false;
	nunchuck.upperButton  	= false;
}

void VescUart::setSerialPort(HardwareSerial* port)
{
	serialPort = port;
}

void VescUart::setDebugPort(Stream* port)
{
	debugPort = port;
}

void VescUart::setTimeout(int timeout)
{
	maxTimeout = timeout;
}

// no unpacking
int VescUart::receiveUartMessageRaw(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];

	uint32_t timeout = millis() + maxTimeout; // Defining the timestamp for timeout (100ms before timeout)

	while ( millis() < timeout && messageRead == false) {

		while (serialPort->available()) {

			payloadReceived[counter++] = serialPort->read();

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = payloadReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3)
						if( debugPort != NULL ){
							debugPort->println("Message is larger than 256 bytes - not supported");
						}
					break;

					default:
						if( debugPort != NULL ){
							debugPort->println("Unvalid start bit");
						}
					break;
				}
			}

			if (counter >= sizeof(payloadReceived)) {
				break;
			}

			if (counter == endMessage && payloadReceived[endMessage - 1] == 3) {
				payloadReceived[endMessage] = 0;
				if (debugPort != NULL) {
					debugPort->println("End of message reached!");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false && debugPort != NULL ) {
		debugPort->println("Timeout");
	}

	return counter;
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	uint16_t counter = 0;
	uint16_t endMessage = 512;
	bool messageRead = false;
	uint8_t messageReceived[512];
	uint16_t lenPayload = 0;

	uint32_t timeout = millis() + maxTimeout; // Defining the timestamp for timeout (100ms before timeout)

	while ( millis() < timeout && messageRead == false) {

		while (serialPort->available()) {

			messageReceived[counter++] = serialPort->read();

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						lenPayload = messageReceived[1];
						endMessage = lenPayload + 5; //Payload size + 2 for size + 3 for SRC and End.
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3) // SHOULD WORK NOW (JAMIE4224)
						lenPayload = messageReceived[1] << 8;
						lenPayload |= messageReceived[2];
						endMessage = lenPayload + 6;
					break;

					default:
						if( debugPort != NULL ){
							debugPort->println("Unvalid start bit");
						}
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				if (debugPort != NULL) {
					debugPort->println("End of message reached!");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false && debugPort != NULL ) {
		debugPort->println("Timeout");
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived, lenPayload);
	}

	if (unpacked) {
		// Message was read
		return lenPayload; 
	}
	else {
		// No Message Read
		return 0;
	}
}


bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload, uint16_t lenPay) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if(debugPort!=NULL){
		debugPort->print("SRC received: "); debugPort->println(crcMessage);
	}

	switch (message[0])
	{
		case 2:
			// Extract payload: 
			memcpy(payload, &message[2], message[1]);

			crcPayload = crc16(payload, message[1]);
		break;

		case 3:
			memcpy(payload, &message[3], lenPay);

			crcPayload = crc16(payload, lenPay);
		break;
	}

	if( debugPort != NULL ){
		debugPort->print("SRC calc: "); debugPort->println(crcPayload);
	}
	
	if (crcPayload == crcMessage) {
		if( debugPort != NULL ) {
			debugPort->print("Received: "); 
			serialPrint(message, lenMes); debugPort->println();

			debugPort->print("Payload :      ");
			serialPrint(payload, lenPay - 1); debugPort->println();
		}

		return true;
	}else{
		return false;
	}
}


int VescUart::packSendPayload(uint8_t * payload, int lenPay) {

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[512];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = '\0';

	if(debugPort!=NULL){
		debugPort->print("UART package send: "); serialPrint(messageSend, count);
	}

	// Sending package
	serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}


bool VescUart::processReadPacket(uint8_t * message) {

	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			data.tempFET   = buffer_get_float16(message, 10.0, &ind); // buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
			data.tempMotor = buffer_get_float16(message, 10.0, &ind); // buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);

			data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);
			data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);

			ind += 8; // Skip the next 8 bytes
			// buffer_append_float32(send_buffer, mc_interface_read_reset_avg_id(), 1e2, &ind);
			// buffer_append_float32(send_buffer, mc_interface_read_reset_avg_iq(), 1e2, &ind);

			data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);
			data.rpm 				= buffer_get_int32(message, &ind);
			data.inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
			data.ampHours 			= buffer_get_float32(message, 10000.0, &ind);
			data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);

			ind += 8; // Skip the next 8 bytes
			// buffer_append_float32(send_buffer, mc_interface_get_watt_hours(false), 1e4, &ind);
			// buffer_append_float32(send_buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind);

			data.tachometer 		= buffer_get_int32(message, &ind);
			data.tachometerAbs 		= buffer_get_int32(message, &ind);
			return true;
			

		default:
			return false;
		break;
	}
}

bool VescUart::getVescValues(uint8_t comm) {

	uint8_t command[1] = { comm }; // COMM_GET_VALUES or COMM_GET_UNITY_VALUE

	uint8_t payload[256];

	packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 55) {
		bool read = processReadPacket(payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

void VescUart::setNunchuckValues() {
	int32_t ind = 0;
	uint8_t payload[11];

	payload[ind++] = COMM_SET_CHUCK_DATA;
	payload[ind++] = nunchuck.valueX;
	payload[ind++] = nunchuck.valueY;
	buffer_append_bool(payload, nunchuck.lowerButton, &ind);
	buffer_append_bool(payload, nunchuck.upperButton, &ind);

	// Acceleration Data. Not used, Int16 (2 byte)
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;

	if(debugPort != NULL){
		debugPort->println("Data reached at setNunchuckValues:");
		debugPort->print("valueX = "); debugPort->print(nunchuck.valueX); debugPort->print(" valueY = "); debugPort->println(nunchuck.valueY);
		debugPort->print("LowerButton = "); debugPort->print(nunchuck.lowerButton); debugPort->print(" UpperButton = "); debugPort->println(nunchuck.upperButton);
	}

	packSendPayload(payload, 11);

	// second VESC
	// payload[ind++] = COMM_FORWARD_CAN;
	// payload[ind++] = 0; // Second VESC ID
	// payload[ind++] = COMM_SET_CHUCK_DATA;
	// payload[ind++] = nunchuck.valueX;
	// payload[ind++] = nunchuck.valueY;
	// buffer_append_bool(payload, nunchuck.lowerButton, &ind);
	// buffer_append_bool(payload, nunchuck.upperButton, &ind);
	//
	// // Acceleration Data. Not used, Int16 (2 byte)
	// payload[ind++] = 0;
	// payload[ind++] = 0;
	// payload[ind++] = 0;
	// payload[ind++] = 0;
	// payload[ind++] = 0;
	// payload[ind++] = 0;
	//
	// packSendPayload(payload, 11+2);

}

void VescUart::setCurrent(float current) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setRPM(float rpm) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM ;
	buffer_append_int32(payload, (int32_t)(rpm), &index);

	packSendPayload(payload, 5);
}

void VescUart::setDuty(float duty) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, 5);
}

void VescUart::serialPrint(uint8_t * data, int len) {
	if(debugPort != NULL){
		for (int i = 0; i <= len; i++)
		{
			debugPort->print(data[i]);
			debugPort->print(" ");
		}

		debugPort->println("");
	}
}

void VescUart::printVescValues() {
	if(debugPort != NULL){
		debugPort->print("avgMotorCurrent: "); 	debugPort->println(data.avgMotorCurrent);
		debugPort->print("avgInputCurrent: "); 	debugPort->println(data.avgInputCurrent);
		debugPort->print("dutyCycleNow: "); 	debugPort->println(data.dutyCycleNow);
		debugPort->print("rpm: "); 				debugPort->println(data.rpm);
		debugPort->print("inputVoltage: "); 	debugPort->println(data.inpVoltage);
		debugPort->print("ampHours: "); 		debugPort->println(data.ampHours);
		debugPort->print("ampHoursCharges: "); 	debugPort->println(data.ampHoursCharged);
		debugPort->print("tachometer: "); 		debugPort->println(data.tachometer);
		debugPort->print("tachometerAbs: "); 	debugPort->println(data.tachometerAbs);
	}
}


bool VescUart::getMotorConfiguration() {
	
	uint8_t command[1] = { COMM_GET_MCCONF }; // COMM_GET_VALUES or COMM_GET_UNITY_VALUE

	uint8_t payload[512];

	packSendPayload(command, 1);
	delay(10); 

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 50) {
		const uint8_t* buffer = (uint8_t*) payload;

		COMM_PACKET_ID packetId;
		int32_t ind = 0;

		packetId = (COMM_PACKET_ID)buffer[0];
		buffer++; // Removes the packetId from the actual message (payload)



		
		

		motorconfig.mcconf_signature = buffer_get_uint32(buffer, &ind);

		motorconfig.pwm_mode = buffer[ind++];
		motorconfig.comm_mode = buffer[ind++];
		motorconfig.motor_type = buffer[ind++];
		motorconfig.sensor_mode = buffer[ind++];
		motorconfig.l_current_max = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_current_min = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_in_current_max = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_in_current_min = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_abs_current_max = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_min_erpm = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_max_erpm = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_erpm_start = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_max_erpm_fbrake = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_max_erpm_fbrake_cc = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_min_vin = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_max_vin = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_battery_cut_start = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_battery_cut_end = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_slow_abs_current = buffer[ind++];
		motorconfig.l_temp_fet_start = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_temp_fet_end = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_temp_motor_start = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_temp_motor_end = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_temp_accel_dec = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_min_duty = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_max_duty = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_watt_max = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_watt_min = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_current_max_scale = buffer_get_float32_auto(buffer, &ind);
		motorconfig.l_current_min_scale = buffer_get_float32_auto(buffer, &ind);
		motorconfig.sl_min_erpm = buffer_get_float32_auto(buffer, &ind);
		motorconfig.sl_min_erpm_cycle_int_limit = buffer_get_float32_auto(buffer, &ind);
		motorconfig.sl_max_fullbreak_current_dir_change = buffer_get_float32_auto(buffer, &ind);
		motorconfig.sl_cycle_int_limit = buffer_get_float32_auto(buffer, &ind);
		motorconfig.sl_phase_advance_at_br = buffer_get_float32_auto(buffer, &ind);
		motorconfig.sl_cycle_int_rpm_br = buffer_get_float32_auto(buffer, &ind);
		motorconfig.sl_bemf_coupling_k = buffer_get_float32_auto(buffer, &ind);
		motorconfig.hall_table[0] = (int8_t)buffer[ind++];
		motorconfig.hall_table[1] = (int8_t)buffer[ind++];
		motorconfig.hall_table[2] = (int8_t)buffer[ind++];
		motorconfig.hall_table[3] = (int8_t)buffer[ind++];
		motorconfig.hall_table[4] = (int8_t)buffer[ind++];
		motorconfig.hall_table[5] = (int8_t)buffer[ind++];
		motorconfig.hall_table[6] = (int8_t)buffer[ind++];
		motorconfig.hall_table[7] = (int8_t)buffer[ind++];
		motorconfig.hall_sl_erpm = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_current_kp = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_current_ki = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_f_sw = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_dt_us = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_encoder_inverted = buffer[ind++];
		motorconfig.foc_encoder_offset = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_encoder_ratio = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_encoder_sin_gain = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_encoder_cos_gain = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_encoder_sin_offset = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_encoder_cos_offset = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_encoder_sincos_filter_constant = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_sensor_mode = buffer[ind++];
		motorconfig.foc_pll_kp = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_pll_ki = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_motor_l = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_motor_r = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_motor_flux_linkage = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_observer_gain = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_observer_gain_slow = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_duty_dowmramp_kp = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_duty_dowmramp_ki = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_openloop_rpm = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_sl_openloop_hyst = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_sl_openloop_time = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_sl_d_current_duty = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_sl_d_current_factor = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_hall_table[0] = buffer[ind++];
		motorconfig.foc_hall_table[1] = buffer[ind++];
		motorconfig.foc_hall_table[2] = buffer[ind++];
		motorconfig.foc_hall_table[3] = buffer[ind++];
		motorconfig.foc_hall_table[4] = buffer[ind++];
		motorconfig.foc_hall_table[5] = buffer[ind++];
		motorconfig.foc_hall_table[6] = buffer[ind++];
		motorconfig.foc_hall_table[7] = buffer[ind++];
		motorconfig.foc_sl_erpm = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_sample_v0_v7 = buffer[ind++];
		motorconfig.foc_sample_high_current = buffer[ind++];
		motorconfig.foc_sat_comp = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_temp_comp = buffer[ind++];
		motorconfig.foc_temp_comp_base_temp = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_current_filter_const = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_cc_decoupling = buffer[ind++];
		motorconfig.foc_observer_type = buffer[ind++];
		motorconfig.foc_hfi_voltage_start = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_hfi_voltage_run = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_hfi_voltage_max = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_sl_erpm_hfi = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_hfi_start_samples = buffer_get_uint16(buffer, &ind);
		motorconfig.foc_hfi_obs_ovr_sec = buffer_get_float32_auto(buffer, &ind);
		motorconfig.foc_hfi_samples = buffer[ind++];
		motorconfig.gpd_buffer_notify_left = buffer_get_int16(buffer, &ind);
		motorconfig.gpd_buffer_interpol = buffer_get_int16(buffer, &ind);
		motorconfig.gpd_current_filter_const = buffer_get_float32_auto(buffer, &ind);
		motorconfig.gpd_current_kp = buffer_get_float32_auto(buffer, &ind);
		motorconfig.gpd_current_ki = buffer_get_float32_auto(buffer, &ind);
		motorconfig.s_pid_kp = buffer_get_float32_auto(buffer, &ind);
		motorconfig.s_pid_ki = buffer_get_float32_auto(buffer, &ind);
		motorconfig.s_pid_kd = buffer_get_float32_auto(buffer, &ind);
		motorconfig.s_pid_kd_filter = buffer_get_float32_auto(buffer, &ind);
		motorconfig.s_pid_min_erpm = buffer_get_float32_auto(buffer, &ind);
		motorconfig.s_pid_allow_braking = buffer[ind++];
		motorconfig.p_pid_kp = buffer_get_float32_auto(buffer, &ind);
		motorconfig.p_pid_ki = buffer_get_float32_auto(buffer, &ind);
		motorconfig.p_pid_kd = buffer_get_float32_auto(buffer, &ind);
		motorconfig.p_pid_kd_filter = buffer_get_float32_auto(buffer, &ind);
		motorconfig.p_pid_ang_div = buffer_get_float32_auto(buffer, &ind);
		motorconfig.cc_startup_boost_duty = buffer_get_float32_auto(buffer, &ind);
		motorconfig.cc_min_current = buffer_get_float32_auto(buffer, &ind);
		motorconfig.cc_gain = buffer_get_float32_auto(buffer, &ind);
		motorconfig.cc_ramp_step_max = buffer_get_float32_auto(buffer, &ind);
		motorconfig.m_fault_stop_time_ms = buffer_get_int32(buffer, &ind);
		motorconfig.m_duty_ramp_step = buffer_get_float32_auto(buffer, &ind);
		motorconfig.m_current_backoff_gain = buffer_get_float32_auto(buffer, &ind);
		motorconfig.m_encoder_counts = buffer_get_uint32(buffer, &ind);
		motorconfig.m_sensor_port_mode = buffer[ind++];
		motorconfig.m_invert_direction = buffer[ind++];
		motorconfig.m_drv8301_oc_mode = buffer[ind++];
		motorconfig.m_drv8301_oc_adj = buffer[ind++];
		motorconfig.m_bldc_f_sw_min = buffer_get_float32_auto(buffer, &ind);
		motorconfig.m_bldc_f_sw_max = buffer_get_float32_auto(buffer, &ind);
		motorconfig.m_dc_f_sw = buffer_get_float32_auto(buffer, &ind);
		motorconfig.m_ntc_motor_beta = buffer_get_float32_auto(buffer, &ind);
		motorconfig.m_out_aux_mode = buffer[ind++];
		motorconfig.m_motor_temp_sens_type = buffer[ind++];
		motorconfig.m_ptc_motor_coeff = buffer_get_float32_auto(buffer, &ind);
		motorconfig.si_motor_poles = buffer[ind++];
		motorconfig.si_gear_ratio = buffer_get_float32_auto(buffer, &ind);
		motorconfig.si_wheel_diameter = buffer_get_float32_auto(buffer, &ind);
		motorconfig.si_battery_type = buffer[ind++];
		motorconfig.si_battery_cells = buffer[ind++];
		motorconfig.si_battery_ah = buffer_get_float32_auto(buffer, &ind);

		return true;
	}
	else
	{
		return false;
	}
}

bool VescUart::setMotorConfiguration() {


	int32_t ind = 0;
	uint8_t buffer[512];

	buffer[0] = COMM_SET_MCCONF;

	ind = confgenerator_serialize_mcconf(buffer + 1, &motorconfig);


	packSendPayload(buffer, ind + 1);
}

