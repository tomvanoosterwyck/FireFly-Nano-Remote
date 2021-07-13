/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

#include "VescUart.h"
#include "buffer.h"
#include "crc.h"

static HardwareSerial *serialPort1;
static HardwareSerial *serialPort2;
static HardwareSerial *serialPort3;
static HardwareSerial *serialPort4;
static HardwareSerial *debugSerialPort = NULL;

bool UnpackPayload(uint8_t *message, int lenMes, uint8_t *payload, int lenPa);
bool ProcessReadPacket(uint8_t *message, bldcMeasure &values, int len);

void SetSerialPort(HardwareSerial *_serialPort1, HardwareSerial *_serialPort2, HardwareSerial *_serialPort3, HardwareSerial *_serialPort4)
{
  serialPort1 = _serialPort1;
  serialPort2 = _serialPort2;
  serialPort3 = _serialPort3;
  serialPort4 = _serialPort4;
}

void SetSerialPort(HardwareSerial *_serialPort)
{
  SetSerialPort(_serialPort, _serialPort, _serialPort, _serialPort);
}

void SetDebugSerialPort(HardwareSerial *_debugSerialPort)
{
  debugSerialPort = _debugSerialPort;
}

enum REC_State
{
  REC_IDLE = 0,
  REC_LEN,
  REC_PAYLOAD,
};

uint8_t payloadGlobal[440];
uint8_t messageGlobal[440];

//HardwareSerial *serial; ///@param num as integer with the serial port in use (0=Serial; 1=Serial1; 2=Serial2; 3=Serial3;)
uint16_t ReceiveUartMessage(uint8_t *payloadReceived, int num)
{

  //Messages <= 255 start with 2. 2nd byte is length
  //Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

  int counter = 0;
  //uint16_t endMessage = 256;
  uint16_t endMessage = 440;
  bool messageRead = false;

  //for(int i = 0; i < 420; i++)
  //  messageReceived[i]=0;
  
  uint16_t lenPayload = 0;
  HardwareSerial *serial = serialPort1;

  REC_State state = REC_IDLE;
  bool longMessage = false;

  int timeout = 0;
  while (!serial->available())
  {
    timeout += 10;
    delay(10);
    if (timeout > 1000)
    {
      return 0; //Timeout, cant communicate with vesc.
    }
  }
  timeout = 0;

  bool done = false;
  while (!done)
  {
    delay(0);
    while (!serial->available())
    {
      timeout += 10;
      delay(10);
      if (timeout > 1000)
      {
        return 0; //Timeout, cant communicate with vesc.
      }
    }
    timeout = 0;

    uint8_t c = serial->read();
    switch (state)
    {
      case REC_IDLE:
        if(c == 2 || c == 3)
        {
          
          counter = 0;
          messageGlobal[counter] = c;
          state = REC_LEN;
          if(c == 3)
          {
            longMessage = true;
          }
          else 
          {
            longMessage = false;
          }
          counter = 1;
        }
        if(c == 3)
        {
          longMessage = true;
          state = REC_LEN;
        }
      break;
      case REC_LEN:
        
        if(!longMessage)
        {
          messageGlobal[counter] = c;
          lenPayload = messageGlobal[1];
          endMessage = messageGlobal[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
          state = REC_PAYLOAD;
          counter = 2; //++
        }
        else
        {
          messageGlobal[counter] = c;
          if(counter == 2)
          {
            lenPayload = messageGlobal[1] << 8 | messageGlobal[2];
            endMessage = lenPayload + 6; //Payload size + 3 for sice + 3 for SRC and End.
            state = REC_PAYLOAD;
          }
          counter++; //first 2, then 3
        }
      break;
    case REC_PAYLOAD:

      messageGlobal[counter] = c;
      counter++;
      if (counter >= endMessage || counter >= 440)
      {
        done = true;
      }
      break;
    }
  }  
  
  /*
  debugSerialPort->print("done. len = "); debugSerialPort->print(lenPayload);
  debugSerialPort->print(" endmessage = "); debugSerialPort->print(endMessage);
  debugSerialPort->print(" done = "); debugSerialPort->print(done);
  debugSerialPort->print(" counter = "); debugSerialPort->println(counter);
  for(int i = 0; i <= endMessage; i++)
  {
    debugSerialPort->print(i);debugSerialPort->print(": "); debugSerialPort->println(String(messageReceived[i]));
  }*/
  
  bool unpacked = false;
  if (done)
  {
    unpacked = UnpackPayload(messageGlobal, endMessage, payloadReceived, messageGlobal[1]);
  }
  if (unpacked)
  {
    return lenPayload; //Message was read

  }
  else {
    return 0; //No Message Read
  }
}



bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay)  //FOR ALL MSG
{
  uint16_t crcMessage = 0;
  uint16_t crcPayload = 0;
  //Rebuild src:
  crcMessage = message[lenMes - 3] << 8;
  crcMessage &= 0xFF00;
  crcMessage += message[lenMes - 2];
  //if(debugSerialPort!=NULL){
  //  debugSerialPort->print("SRC received: "); debugSerialPort->println(crcMessage);
  //} // DEBUG
  
  int lenPayload;
  //Extract payload:
  if(message[0] == 2)
  {
    lenPayload = message[1];
    memcpy(payload, &message[2], lenPayload);
  }
  else if(message[0] == 3)
  {
    lenPayload = message[1] << 8 | message[2];
    
    memcpy(payload, &message[3], lenPayload);
  }
  else 
    if(debugSerialPort!=NULL)debugSerialPort->print("ERROR: wrong start byte");
  
  crcPayload = crc16(payload, lenPayload);

  
  //if(debugSerialPort!=NULL){
  //  debugSerialPort->print("SRC calc: "); debugSerialPort->println(crcPayload);
  //}
  if (crcPayload == crcMessage)
  {
    if(debugSerialPort!=NULL){
        //debugSerialPort->print("Received: "); SerialPrint(message, lenMes); debugSerialPort->println();
        //debugSerialPort->print("Payload :      "); SerialPrint(payload, lenPayload - 1); debugSerialPort->println();
    } // DEBUG

    return true;
  }
  else
  {
    return false;
  }
}



int PackSendPayload(uint8_t* payload, int lenPay, int num) {
  uint16_t crcPayload = crc16(payload, lenPay);
  int count = 0;
  //uint8_t messageSend[420]; glob

  if (lenPay <= 256)
  {
    messageGlobal[count++] = 2;
    messageGlobal[count++] = lenPay;
  }
  else
  {
    messageGlobal[count++] = 3;
    messageGlobal[count++] = (uint8_t)(lenPay >> 8);
    messageGlobal[count++] = (uint8_t)(lenPay & 0xFF);
  }
  memcpy(&messageGlobal[count], payload, lenPay);

  count += lenPay;
  messageGlobal[count++] = (uint8_t)(crcPayload >> 8);
  messageGlobal[count++] = (uint8_t)(crcPayload & 0xFF);
  messageGlobal[count++] = 3;
  messageGlobal[count] = 0;

//if(debugSerialPort!=NULL){
//  debugSerialPort->print("UART package send: "); SerialPrint(messageGlobal, count);

//} // DEBUG


  HardwareSerial *serial;

  switch (num) {
    case 0:
      serial=serialPort1;
      break;
    case 1:
      serial=serialPort2;
      break;
    case 2:
      serial=serialPort3;
      break;
    case 3:
      serial=serialPort4;
      break;
    default:
      break;
  }

  //Sending package
  
  serial->write(messageGlobal, count);


  //Returns number of send bytes
  return count;
}



int BuildPacket(uint8_t* send_buffer, mc_configuration& mcconf)
{
  int32_t ind = 0;

  send_buffer[ind++] = COMM_SET_MCCONF;  /////////////////////////////////////////////////////////////////////////Cahnge BACK
  
  buffer_append_uint32(send_buffer, mcconf.MCCONF_SIGNATURE, &ind);
  
  send_buffer[ind++] = mcconf.pwm_mode;
  send_buffer[ind++] = mcconf.comm_mode;
  send_buffer[ind++] = mcconf.motor_type;
  send_buffer[ind++] = mcconf.sensor_mode;

  buffer_append_float32_auto(send_buffer, mcconf.l_current_max, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_current_min, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_in_current_max, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_in_current_min, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_abs_current_max, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_min_erpm, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_max_erpm, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_erpm_start, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_max_erpm_fbrake, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_max_erpm_fbrake_cc, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_min_vin, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_max_vin, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_battery_cut_start, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_battery_cut_end, &ind);
  send_buffer[ind++] = mcconf.l_slow_abs_current;
  buffer_append_float32_auto(send_buffer, mcconf.l_temp_fet_start, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_temp_fet_end, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_temp_motor_start, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_temp_motor_end, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_temp_accel_dec, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_min_duty, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_max_duty, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_watt_max, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_watt_min, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.l_current_max_scale, &ind); //new
  buffer_append_float32_auto(send_buffer, mcconf.l_current_min_scale, &ind); //new
  
  buffer_append_float32_auto(send_buffer, mcconf.sl_min_erpm, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.sl_min_erpm_cycle_int_limit, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.sl_max_fullbreak_current_dir_change, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.sl_cycle_int_limit, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.sl_phase_advance_at_br, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.sl_cycle_int_rpm_br, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.sl_bemf_coupling_k, &ind);

  memcpy(send_buffer + ind, mcconf.hall_table, 8);
  ind += 8;
  buffer_append_float32_auto(send_buffer, mcconf.hall_sl_erpm, &ind);

  buffer_append_float32_auto(send_buffer, mcconf.foc_current_kp, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_current_ki, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_f_sw, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_dt_us, &ind);
  send_buffer[ind++] = mcconf.foc_encoder_inverted;
  buffer_append_float32_auto(send_buffer, mcconf.foc_encoder_offset, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_encoder_ratio, &ind); /////insert!
  send_buffer[ind++] = mcconf.foc_sensor_mode;
  buffer_append_float32_auto(send_buffer, mcconf.foc_pll_kp, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_pll_ki, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_motor_l, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_motor_r, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_motor_flux_linkage, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_observer_gain, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_observer_gain_slow, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_duty_dowmramp_kp, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_duty_dowmramp_ki, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_openloop_rpm, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_sl_openloop_hyst, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_sl_openloop_time, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_sl_d_current_duty, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_sl_d_current_factor, &ind);
  memcpy(send_buffer + ind, mcconf.foc_hall_table, 8);
  ind += 8;
  buffer_append_float32_auto(send_buffer, mcconf.foc_sl_erpm, &ind);
  send_buffer[ind++] = mcconf.foc_sample_v0_v7;
  send_buffer[ind++] = mcconf.foc_sample_high_current;
  buffer_append_float32_auto(send_buffer, mcconf.foc_sat_comp, &ind);
  send_buffer[ind++] = mcconf.foc_temp_comp;
  buffer_append_float32_auto(send_buffer, mcconf.foc_temp_comp_base_temp, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.foc_current_filter_const, &ind);
  //NEW 
  buffer_append_int16(send_buffer, mcconf.gpd_buffer_notify_left, &ind);
	buffer_append_int16(send_buffer, mcconf.gpd_buffer_interpol, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.gpd_current_filter_const, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.gpd_current_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.gpd_current_ki, &ind);
  
  buffer_append_float32_auto(send_buffer, mcconf.s_pid_kp, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.s_pid_ki, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.s_pid_kd, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.s_pid_kd_filter, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.s_pid_min_erpm, &ind);
  send_buffer[ind++] = mcconf.s_pid_allow_braking;

  buffer_append_float32_auto(send_buffer, mcconf.p_pid_kp, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.p_pid_ki, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.p_pid_kd, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.p_pid_kd_filter, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.p_pid_ang_div, &ind);

  buffer_append_float32_auto(send_buffer, mcconf.cc_startup_boost_duty, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.cc_min_current, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.cc_gain, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.cc_ramp_step_max, &ind);

  buffer_append_int32(send_buffer, mcconf.m_fault_stop_time_ms, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.m_duty_ramp_step, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.m_current_backoff_gain, &ind);
  buffer_append_uint32(send_buffer, mcconf.m_encoder_counts, &ind);
  send_buffer[ind++] = mcconf.m_sensor_port_mode;
  send_buffer[ind++] = mcconf.m_invert_direction;
  send_buffer[ind++] = mcconf.m_drv8301_oc_mode;
  send_buffer[ind++] = mcconf.m_drv8301_oc_adj;
  buffer_append_float32_auto(send_buffer, mcconf.m_bldc_f_sw_min, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.m_bldc_f_sw_max, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.m_dc_f_sw, &ind);
  buffer_append_float32_auto(send_buffer, mcconf.m_ntc_motor_beta, &ind);
  send_buffer[ind++] = mcconf.m_out_aux_mode;  
  //NEW
  send_buffer[ind++] = (uint8_t)mcconf.si_motor_poles;
	buffer_append_float32_auto(send_buffer, mcconf.si_gear_ratio, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.si_wheel_diameter, &ind);
	send_buffer[ind++] = mcconf.si_battery_type;
	send_buffer[ind++] = (uint8_t)mcconf.si_battery_cells;
	buffer_append_float32_auto(send_buffer, mcconf.si_battery_ah, &ind);
  
  return ind;
}

bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len) {
  COMM_PACKET_ID packetId;
  int32_t ind = 0;

  packetId = (COMM_PACKET_ID)message[0];
  message++;//Eliminates the message id
  len--;

  switch (packetId)
  {

  case COMM_GET_VALUES:
    values.tempFetFiltered    = buffer_get_float16(message, 1e1, &ind);
    values.tempMotorFiltered  = buffer_get_float16(message, 1e1, &ind);
    values.avgMotorCurrent    = buffer_get_float32(message, 100.0, &ind);
    values.avgInputCurrent    = buffer_get_float32(message, 100.0, &ind);
    values.avgId              = buffer_get_float32(message, 1e2, &ind);
    values.avgIq              = buffer_get_float32(message, 1e2, &ind);
    values.dutyNow            = buffer_get_float16(message, 1000.0, &ind);
    values.rpm                = buffer_get_float32(message, 1.0, &ind);
    values.inpVoltage         = buffer_get_float16(message, 10.0, &ind);
    values.ampHours           = buffer_get_float32(message, 10000.0, &ind);
    values.ampHoursCharged    = buffer_get_float32(message, 10000.0, &ind);
    values.wattHours          = buffer_get_float32(message, 1e4, &ind);
    values.watthoursCharged   = buffer_get_float32(message, 1e4, &ind);
    values.tachometer         = buffer_get_int32(message, &ind);
    values.tachometerAbs      = buffer_get_int32(message, &ind);
    values.faultCode          = message[ind];
    return true;
    break;
  default:
    return false;
    break;
  }
}

bool ProcessReadPacket(uint8_t* data, mc_configuration& mcconf, int len) {
  COMM_PACKET_ID packetId;
  int32_t ind = 0;

  packetId = (COMM_PACKET_ID)data[0];
  data++;//Eliminates the message id
  len--;

  switch (packetId)
  {

  case COMM_GET_MCCONF:
    ind = 0;
    
    //buffer_append_uint32(buffer, MCCONF_SIGNATURE, &ind);
    /*MCCONF_SIGNATURE*/
    mcconf.MCCONF_SIGNATURE = buffer_get_uint32(data, &ind); 

		mcconf.pwm_mode = (mc_pwm_mode)data[ind++];
		mcconf.comm_mode = (mc_comm_mode)data[ind++];
		mcconf.motor_type = (mc_motor_type)data[ind++];
		mcconf.sensor_mode = (mc_sensor_mode)data[ind++];

		mcconf.l_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_current_min = buffer_get_float32_auto(data, &ind);
		mcconf.l_in_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_in_current_min = buffer_get_float32_auto(data, &ind);
		mcconf.l_abs_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.l_erpm_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm_fbrake = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm_fbrake_cc = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_vin = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_vin = buffer_get_float32_auto(data, &ind);
		mcconf.l_battery_cut_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_battery_cut_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_slow_abs_current = data[ind++];
		mcconf.l_temp_fet_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_fet_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_motor_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_motor_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_accel_dec = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_duty = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_duty = buffer_get_float32_auto(data, &ind);
		mcconf.l_watt_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_watt_min = buffer_get_float32_auto(data, &ind);

    //new
	mcconf.l_current_max_scale = buffer_get_float32_auto(data, &ind);
	mcconf.l_current_min_scale = buffer_get_float32_auto(data, &ind);
		
    mcconf.lo_current_max = mcconf.l_current_max;
		mcconf.lo_current_min = mcconf.l_current_min;
		mcconf.lo_in_current_max = mcconf.l_in_current_max;
		mcconf.lo_in_current_min = mcconf.l_in_current_min;
		mcconf.lo_current_motor_max_now = mcconf.l_current_max;
		mcconf.lo_current_motor_min_now = mcconf.l_current_min;

		mcconf.sl_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.sl_min_erpm_cycle_int_limit = buffer_get_float32_auto(data, &ind);
		mcconf.sl_max_fullbreak_current_dir_change = buffer_get_float32_auto(data, &ind);
		mcconf.sl_cycle_int_limit = buffer_get_float32_auto(data, &ind);
		mcconf.sl_phase_advance_at_br = buffer_get_float32_auto(data, &ind);
		mcconf.sl_cycle_int_rpm_br = buffer_get_float32_auto(data, &ind);
		mcconf.sl_bemf_coupling_k = buffer_get_float32_auto(data, &ind);

		memcpy(mcconf.hall_table, data + ind, 8);
		ind += 8;
		mcconf.hall_sl_erpm = buffer_get_float32_auto(data, &ind);

		mcconf.foc_current_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_current_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_f_sw = buffer_get_float32_auto(data, &ind);
		mcconf.foc_dt_us = buffer_get_float32_auto(data, &ind);
		mcconf.foc_encoder_inverted = data[ind++];
		mcconf.foc_encoder_offset = buffer_get_float32_auto(data, &ind);
		mcconf.foc_encoder_ratio = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sensor_mode = (mc_foc_sensor_mode) data[ind++];
		mcconf.foc_pll_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_pll_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_l = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_r = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_flux_linkage = buffer_get_float32_auto(data, &ind);
		mcconf.foc_observer_gain = buffer_get_float32_auto(data, &ind);
		mcconf.foc_observer_gain_slow = buffer_get_float32_auto(data, &ind);
		mcconf.foc_duty_dowmramp_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_duty_dowmramp_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_openloop_rpm = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_openloop_hyst = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_openloop_time = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_d_current_duty = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_d_current_factor = buffer_get_float32_auto(data, &ind);
		memcpy(mcconf.foc_hall_table, data + ind, 8);
		ind += 8;
		mcconf.foc_sl_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sample_v0_v7 = data[ind++];
		mcconf.foc_sample_high_current = data[ind++];
		mcconf.foc_sat_comp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_temp_comp = data[ind++];
		mcconf.foc_temp_comp_base_temp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_current_filter_const = buffer_get_float32_auto(data, &ind);
//NEW
  mcconf.gpd_buffer_notify_left = buffer_get_int16(data, &ind);
	mcconf.gpd_buffer_interpol = buffer_get_int16(data, &ind);
	mcconf.gpd_current_filter_const = buffer_get_float32_auto(data, &ind);
	mcconf.gpd_current_kp = buffer_get_float32_auto(data, &ind);
	mcconf.gpd_current_ki = buffer_get_float32_auto(data, &ind);
    
		mcconf.s_pid_kp = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_ki = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_kd = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_kd_filter = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_allow_braking = data[ind++];

		mcconf.p_pid_kp = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_ki = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_kd = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_kd_filter = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_ang_div = buffer_get_float32_auto(data, &ind);

		mcconf.cc_startup_boost_duty = buffer_get_float32_auto(data, &ind);
		mcconf.cc_min_current = buffer_get_float32_auto(data, &ind);
		mcconf.cc_gain = buffer_get_float32_auto(data, &ind);
		mcconf.cc_ramp_step_max = buffer_get_float32_auto(data, &ind);

		mcconf.m_fault_stop_time_ms = buffer_get_int32(data, &ind);
		mcconf.m_duty_ramp_step = buffer_get_float32_auto(data, &ind);
		mcconf.m_current_backoff_gain = buffer_get_float32_auto(data, &ind);
		mcconf.m_encoder_counts = buffer_get_uint32(data, &ind);
		mcconf.m_sensor_port_mode = (sensor_port_mode)data[ind++];
		mcconf.m_invert_direction = data[ind++];
		mcconf.m_drv8301_oc_mode = (drv8301_oc_mode)data[ind++];
		mcconf.m_drv8301_oc_adj = data[ind++];
		mcconf.m_bldc_f_sw_min = buffer_get_float32_auto(data, &ind);
		mcconf.m_bldc_f_sw_max = buffer_get_float32_auto(data, &ind);
		mcconf.m_dc_f_sw = buffer_get_float32_auto(data, &ind);
		mcconf.m_ntc_motor_beta = buffer_get_float32_auto(data, &ind);
		mcconf.m_out_aux_mode = (out_aux_mode)data[ind++];
//NEW
  mcconf.si_motor_poles = data[ind++];
	mcconf.si_gear_ratio = buffer_get_float32_auto(data, &ind);
	mcconf.si_wheel_diameter = buffer_get_float32_auto(data, &ind);
	mcconf.si_battery_type = (BATTERY_TYPE)data[ind++];
	mcconf.si_battery_cells = data[ind++];
	mcconf.si_battery_ah = buffer_get_float32_auto(data, &ind);
    
    return true;
    break;
  default:
    return false;
    break;
  }
} 

bool VescUartGet(bldcMeasure& values, int num) {
  uint8_t command[1] = { COMM_GET_VALUES };
  //uint8_t payload[256];glob
  PackSendPayload(command, 1, num);
  delay(10); //needed, otherwise data is not read
  int lenPayload = ReceiveUartMessage(payloadGlobal, num);
  if (lenPayload > 1) {
    bool return_val = ProcessReadPacket(payloadGlobal, values, lenPayload); //returns true if sucessful
    return return_val;
  }
  else
  {
    return false;
  }
}
bool VescUartGet(bldcMeasure& values) {
  return VescUartGet(values, 0);
} 

bool VescUartGet(mc_configuration& config, int num) {
  uint8_t command[1] = { COMM_GET_MCCONF };  //COMM_GET_MCCONF  COMM_GET_VALUES
  //uint8_t payload[400];glob
  PackSendPayload(command, 1, num);
  //delay(10); //needed, otherwise data is not read
  int lenPayload = ReceiveUartMessage(payloadGlobal, num);/* MC */
  if (lenPayload > 1 && payloadGlobal[0] == COMM_GET_MCCONF) {
    bool return_val = ProcessReadPacket(payloadGlobal, config, lenPayload); //returns true if sucessful
    return return_val;
  }
  else
  {
    return false; //no comm with vesc or too short message got.
  }
}
bool VescUartGet(mc_configuration& config) {
  return VescUartGet(config, 0);
}

bool VescUartSet(mc_configuration& config, int num) {
  
  //uint8_t payload[360];glob //= {3, 1, 84, 14, 1, 0, 2, 2, 66, 92, 0, 0, 194, 72, 0, 0, 66, 180, 0, 0, 194, 32, 0, 0, 67, 22, 0, 0, 199, 195, 80, 0, 71, 5, 252, 0, 63, 76, 204, 205, 67, 150, 0, 0, 68, 187, 128, 0, 65, 0, 0, 0, 66, 100, 0, 0, 66, 35, 51, 51, 66, 20, 204, 205, 1, 66, 170, 0, 0, 66, 200, 0, 0, 66, 170, 0, 0, 66, 200, 0, 0, 62, 25, 153, 154, 59, 163, 215, 10, 63, 115, 51, 51, 70, 106, 96, 0, 198, 106, 96, 0, 67, 22, 0, 0, 68, 137, 128, 0, 65, 32, 0, 0, 66, 120, 0, 0, 63, 76, 204, 205, 71, 156, 64, 0, 68, 22, 0, 0, 255, 1, 3, 2, 5, 6, 4, 255, 68, 250, 0, 0, 61, 35, 215, 10, 66, 4, 225, 72, 70, 156, 64, 0, 61, 163, 215, 10, 0, 67, 52, 0, 0, 64, 224, 0, 0, 2, 68, 250, 0, 0, 71, 28, 64, 0, 56, 39, 143, 252, 61, 7, 252, 185, 60, 133, 37, 3, 74, 103, 82, 192, 62, 153, 153, 154, 65, 32, 0, 0, 67, 72, 0, 0, 67, 200, 0, 0, 61, 204, 204, 205, 61, 204, 204, 205, 0, 0, 0, 0, 0, 0, 0, 0, 255, 95, 27, 62, 163, 127, 195, 255, 69, 28, 64, 0, 1, 0, 0, 0, 0, 0, 0, 65, 200, 0, 0, 61, 204, 204, 205, 59, 131, 18, 111, 59, 131, 18, 111, 56, 209, 183, 23, 62, 76, 204, 205, 68, 97, 0, 0, 1, 60, 245, 194, 143, 0, 0, 0, 0, 57, 209, 183, 23, 62, 76, 204, 205, 63, 128, 0, 0, 60, 35, 215, 10, 61, 204, 204, 205, 59, 150, 187, 153, 61, 35, 215, 10, 0, 0, 1, 244, 60, 163, 215, 10, 63, 0, 0, 0, 0, 0, 32, 0, 0, 0, 0, 16, 69, 59, 128, 0, 71, 28, 64, 0, 71, 8, 184, 0, 69, 83, 64, 0, 93, 32, 3, 0 };
  //debugSerialPort->print("comp  ");
  //SerialPrint(payload, 360);
  
  for(int i = 0; i<360; i++)
  {
    payloadGlobal[i] = 66;
  }
  
  int lenPayload = BuildPacket(payloadGlobal, config);
  //should be 340 debugSerialPort->print("lenPayload");debugSerialPort->println(lenPayload);
  //debugSerialPort->print("self build      ");
  //SerialPrint(payloadGlobal, lenPayload);
  
  int ret_pack = PackSendPayload(payloadGlobal, lenPayload, num);
  
  if(debugSerialPort!=NULL) debugSerialPort->println("Packed and sent!");
  
  lenPayload = ReceiveUartMessage(payloadGlobal, num);
  if (lenPayload > 1 || lenPayload == 0 || payloadGlobal[0] != 13) {
    //wrong answer from Vesc. should be lenPayload = 1 and payload[0] == 13 (COMM_SET_MCCONF)
    return false;
  }
  else
  {
    return true; //setting of mc-config was sucessful
  }
}
bool VescUartSet(mc_configuration& config) {
  return VescUartSet(config, 0);
}

void VescUartSetCurrent(float current, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT ;
  buffer_append_int32(payload, (int32_t)(current * 1000), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetCurrent(float current){
  VescUartSetCurrent(current, 0);
}

void VescUartSetPosition(float position, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_POS;
  buffer_append_int32(payload, (int32_t)(position * 1000000.0), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetPosition(float position) {
  VescUartSetPosition(position, 0);
}

void VescUartSetDuty(float duty, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_DUTY ;
  buffer_append_int32(payload, (int32_t)(duty * 100000), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetDuty(float duty) {
  VescUartSetDuty(duty, 0);
}


void VescUartSetRPM(float rpm, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_RPM ;
  buffer_append_int32(payload, (int32_t)(rpm), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetRPM(float rpm) {
  VescUartSetRPM(rpm, 0);
}

void VescUartSetCurrentBrake(float brakeCurrent, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT_BRAKE;
  buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetCurrentBrake(float brakeCurrent) {
  VescUartSetCurrentBrake(brakeCurrent, 0);
}


void VescUartSetNunchukValues(nunchuckPackage& data, int num) {
  int32_t ind = 0;
  uint8_t payload[11];
  payload[ind++] = COMM_SET_CHUCK_DATA;
  payload[ind++] = data.valXJoy;
  payload[ind++] = data.valYJoy;
  buffer_append_bool(payload, data.valLowerButton, &ind);
  buffer_append_bool(payload, data.valUpperButton, &ind);
  //Acceleration Data. Not used, Int16 (2 byte)
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;

if(debugSerialPort!=NULL){
  debugSerialPort->println("Data reached at VescUartSetNunchuckValues:");
  debugSerialPort->print("valXJoy = "); debugSerialPort->print(data.valXJoy); debugSerialPort->print(" valYJoy = "); debugSerialPort->println(data.valYJoy);
  debugSerialPort->print("LowerButton = "); debugSerialPort->print(data.valLowerButton); debugSerialPort->print(" UpperButton = "); debugSerialPort->println(data.valUpperButton);
}

  PackSendPayload(payload, 11, num);
}
void VescUartSetNunchukValues(nunchuckPackage& data) {
  VescUartSetNunchukValues(data, 0);
}


void SerialPrint(uint8_t* data, int len) 
{

  //  debugSerialPort->print("Data to display: "); debugSerialPort->println(sizeof(data));

  for (int i = 0; i <= len; i++)
  {
    if(debugSerialPort!=NULL) debugSerialPort->print(data[i]);
    if(debugSerialPort!=NULL) debugSerialPort->print(", ");
  }
  if(debugSerialPort!=NULL) debugSerialPort->println("");
}


void SerialPrint(const bldcMeasure& values) 
{
  if(debugSerialPort!=NULL){
    debugSerialPort->print("tempFetFiltered:  "); debugSerialPort->println(values.tempFetFiltered);
    debugSerialPort->print("tempMotorFiltered:"); debugSerialPort->println(values.tempMotorFiltered);
    debugSerialPort->print("avgMotorCurrent:  "); debugSerialPort->println(values.avgMotorCurrent);
    debugSerialPort->print("avgInputCurrent:  "); debugSerialPort->println(values.avgInputCurrent);
    debugSerialPort->print("avgId:      "); debugSerialPort->println(values.avgId);
    debugSerialPort->print("avgIq:      "); debugSerialPort->println(values.avgIq);
    debugSerialPort->print("dutyNow:      "); debugSerialPort->println(values.dutyNow);
    debugSerialPort->print("rpm:        "); debugSerialPort->println(values.rpm);
    debugSerialPort->print("inpVoltage:    "); debugSerialPort->println(values.inpVoltage);
    debugSerialPort->print("ampHours:    "); debugSerialPort->println(values.ampHours);
    debugSerialPort->print("ampHoursCharged:  "); debugSerialPort->println(values.ampHoursCharged);
    debugSerialPort->print("tachometer:    "); debugSerialPort->println(values.tachometer);
    debugSerialPort->print("tachometerAbs:  "); debugSerialPort->println(values.tachometerAbs);
    debugSerialPort->print("faultCode:    "); debugSerialPort->println(values.faultCode);
  }
}


void SerialPrint(const mc_configuration& config) 
{
  if(debugSerialPort!=NULL) 
  {
    debugSerialPort->print("MCCONF_SIGNATURE: "); debugSerialPort->println(config.MCCONF_SIGNATURE);
    debugSerialPort->print("pwm_mode: "); debugSerialPort->println(config.pwm_mode);
    debugSerialPort->print("comm_mode: "); debugSerialPort->println(config.comm_mode);
    debugSerialPort->print("motor_type: "); debugSerialPort->println(config.motor_type);
    debugSerialPort->print("sensor_mode: "); debugSerialPort->println(config.sensor_mode); 
    debugSerialPort->print("l_current_max: "); debugSerialPort->println(config.l_current_max);
    debugSerialPort->print("l_current_min: "); debugSerialPort->println(config.l_current_min);
    debugSerialPort->print("l_in_current_max: "); debugSerialPort->println(config.l_in_current_max);
    debugSerialPort->print("l_in_current_min: "); debugSerialPort->println(config.l_in_current_min);
    debugSerialPort->print("l_abs_current_max: "); debugSerialPort->println(config.l_abs_current_max);
    debugSerialPort->print("l_min_erpm: "); debugSerialPort->println(config.l_min_erpm);
    debugSerialPort->print("l_max_erpm: "); debugSerialPort->println(config.l_max_erpm);
    debugSerialPort->print("l_max_erpm_fbrake: "); debugSerialPort->println(config.l_max_erpm_fbrake);
    debugSerialPort->print("l_max_erpm_fbrake_cc: "); debugSerialPort->println(config.l_max_erpm_fbrake_cc);
    debugSerialPort->print("l_min_vin: "); debugSerialPort->println(config.l_min_vin);
    debugSerialPort->print("l_max_vin: "); debugSerialPort->println(config.l_max_vin);
    debugSerialPort->print("l_battery_cut_start: "); debugSerialPort->println(config.l_battery_cut_start);
    debugSerialPort->print("l_battery_cut_end:      "); debugSerialPort->println(config.l_battery_cut_end);
    debugSerialPort->print("l_slow_abs_current:      "); debugSerialPort->println(config.l_slow_abs_current);
    
    debugSerialPort->print("l_temp_fet_start:      "); debugSerialPort->println(config.l_temp_fet_start);
    debugSerialPort->print("l_temp_fet_end:      "); debugSerialPort->println(config.l_temp_fet_end);
    debugSerialPort->print("l_temp_motor_start:      "); debugSerialPort->println(config.l_temp_motor_start);
    debugSerialPort->print("l_temp_motor_end:      "); debugSerialPort->println(config.l_temp_motor_end);
    debugSerialPort->print("l_temp_accel_dec:      "); debugSerialPort->println(config.l_temp_accel_dec);
    debugSerialPort->print("l_min_duty:      "); debugSerialPort->println(config.l_min_duty);
    debugSerialPort->print("l_max_duty:      "); debugSerialPort->println(config.l_max_duty);
    debugSerialPort->print("l_watt_max:      "); debugSerialPort->println(config.l_watt_max);
    debugSerialPort->print("l_watt_min:      "); debugSerialPort->println(config.l_watt_min);
    
    debugSerialPort->print("l_current_max_scale:      "); debugSerialPort->println(config.l_current_max_scale);
    debugSerialPort->print("l_current_min_scale:      "); debugSerialPort->println(config.l_current_min_scale);

    debugSerialPort->print("lo_current_max:      "); debugSerialPort->println(config.lo_current_max);
    debugSerialPort->print("lo_current_min:      "); debugSerialPort->println(config.lo_current_min);
    debugSerialPort->print("lo_in_current_max:      "); debugSerialPort->println(config.lo_in_current_max);
    debugSerialPort->print("lo_in_current_min:      "); debugSerialPort->println(config.lo_in_current_min);
    debugSerialPort->print("lo_current_motor_max_now:      "); debugSerialPort->println(config.lo_current_motor_max_now);
    debugSerialPort->print("lo_current_motor_min_now:      "); debugSerialPort->println(config.lo_current_motor_min_now);
    debugSerialPort->print("sl_min_erpm:      "); debugSerialPort->println(config.sl_min_erpm);
    debugSerialPort->print("sl_min_erpm_cycle_int_limit:      "); debugSerialPort->println(config.sl_min_erpm_cycle_int_limit);
    debugSerialPort->print("sl_max_fullbreak_current_dir_change:      "); debugSerialPort->println(config.sl_max_fullbreak_current_dir_change);
    debugSerialPort->print("sl_cycle_int_limit:      "); debugSerialPort->println(config.sl_cycle_int_limit);
    debugSerialPort->print("sl_phase_advance_at_br:      "); debugSerialPort->println(config.sl_phase_advance_at_br);
    debugSerialPort->print("sl_cycle_int_rpm_br:      "); debugSerialPort->println(config.sl_cycle_int_rpm_br);
    debugSerialPort->print("sl_bemf_coupling_k:      "); debugSerialPort->println(config.sl_bemf_coupling_k);
    //debugSerialPort->print("hall_table:      "); debugSerialPort->println(config.hall_table);
    debugSerialPort->print("hall_sl_erpm:      "); debugSerialPort->println(config.hall_sl_erpm);
    debugSerialPort->print("foc_current_kp:      "); debugSerialPort->println(config.foc_current_kp);
    debugSerialPort->print("foc_current_ki:      "); debugSerialPort->println(config.foc_current_ki);
    debugSerialPort->print("foc_f_sw:      "); debugSerialPort->println(config.foc_f_sw);
    debugSerialPort->print("foc_dt_us:      "); debugSerialPort->println(config.foc_dt_us);
    debugSerialPort->print("foc_encoder_inverted:      "); debugSerialPort->println(config.foc_encoder_inverted);
    debugSerialPort->print("foc_encoder_offset:      "); debugSerialPort->println(config.foc_encoder_offset);
    debugSerialPort->print("foc_encoder_ratio:      "); debugSerialPort->println(config.foc_encoder_ratio);
    debugSerialPort->print("foc_sensor_mode:      "); debugSerialPort->println(config.foc_sensor_mode);
    debugSerialPort->print("foc_pll_kp:      "); debugSerialPort->println(config.foc_pll_kp);
    debugSerialPort->print("foc_pll_ki:      "); debugSerialPort->println(config.foc_pll_ki);
    debugSerialPort->print("foc_motor_l:      "); debugSerialPort->println(config.foc_motor_l);
    debugSerialPort->print("foc_motor_r:      "); debugSerialPort->println(config.foc_motor_r);
    debugSerialPort->print("foc_motor_flux_linkage:      "); debugSerialPort->println(config.foc_motor_flux_linkage);
    debugSerialPort->print("foc_observer_gain:      "); debugSerialPort->println(config.foc_observer_gain);
    debugSerialPort->print("foc_observer_gain_slow:      "); debugSerialPort->println(config.foc_observer_gain_slow);
    debugSerialPort->print("foc_duty_dowmramp_kp:      "); debugSerialPort->println(config.foc_duty_dowmramp_kp);
    debugSerialPort->print("foc_duty_dowmramp_ki:      "); debugSerialPort->println(config.foc_duty_dowmramp_ki);
    debugSerialPort->print("foc_openloop_rpm:      "); debugSerialPort->println(config.foc_openloop_rpm);
    debugSerialPort->print("foc_sl_openloop_hyst:      "); debugSerialPort->println(config.foc_sl_openloop_hyst);
    debugSerialPort->print("foc_sl_openloop_time:      "); debugSerialPort->println(config.foc_sl_openloop_time);
    debugSerialPort->print("foc_sl_d_current_duty:      "); debugSerialPort->println(config.foc_sl_d_current_duty);
    debugSerialPort->print("foc_sl_d_current_factor:      "); debugSerialPort->println(config.foc_sl_d_current_factor);
    debugSerialPort->print("foc_sl_erpm:      "); debugSerialPort->println(config.foc_sl_erpm);
    debugSerialPort->print("foc_sample_v0_v7:      "); debugSerialPort->println(config.foc_sample_v0_v7);
    debugSerialPort->print("foc_sample_high_current:      "); debugSerialPort->println(config.foc_sample_high_current);
    debugSerialPort->print("foc_sat_comp:      "); debugSerialPort->println(config.foc_sat_comp);
    debugSerialPort->print("foc_temp_comp:      "); debugSerialPort->println(config.foc_temp_comp);
    debugSerialPort->print("foc_temp_comp_base_temp:      "); debugSerialPort->println(config.foc_temp_comp_base_temp);
    debugSerialPort->print("foc_current_filter_const:      "); debugSerialPort->println(config.foc_current_filter_const);
    
    // GPDrive
	/*int gpd_buffer_notify_left;
	int gpd_buffer_interpol;
	float gpd_current_filter_const;
	float gpd_current_kp;
	float gpd_current_ki;*/
    
    debugSerialPort->print("s_pid_kp:      "); debugSerialPort->println(config.s_pid_kp);
    debugSerialPort->print("s_pid_ki:      "); debugSerialPort->println(config.s_pid_ki);
    debugSerialPort->print("s_pid_kd:      "); debugSerialPort->println(config.s_pid_kd);
    debugSerialPort->print("s_pid_kd_filter:      "); debugSerialPort->println(config.s_pid_kd_filter);
    debugSerialPort->print("s_pid_min_erpm:      "); debugSerialPort->println(config.s_pid_min_erpm);
    debugSerialPort->print("s_pid_allow_braking:      "); debugSerialPort->println(config.s_pid_allow_braking);
    debugSerialPort->print("p_pid_kp:      "); debugSerialPort->println(config.p_pid_kp);
    debugSerialPort->print("p_pid_ki:      "); debugSerialPort->println(config.p_pid_ki);
    debugSerialPort->print("p_pid_kd:      "); debugSerialPort->println(config.p_pid_kd);
    debugSerialPort->print("p_pid_kd_filter:      "); debugSerialPort->println(config.p_pid_kd_filter);
    debugSerialPort->print("p_pid_ang_div:      "); debugSerialPort->println(config.p_pid_ang_div);
    debugSerialPort->print("cc_startup_boost_duty:      "); debugSerialPort->println(config.cc_startup_boost_duty);
    debugSerialPort->print("cc_min_current:      "); debugSerialPort->println(config.cc_min_current);
    debugSerialPort->print("cc_gain:      "); debugSerialPort->println(config.cc_gain);
    debugSerialPort->print("cc_ramp_step_max:      "); debugSerialPort->println(config.cc_ramp_step_max);
    debugSerialPort->print("m_fault_stop_time_ms:      "); debugSerialPort->println(config.m_fault_stop_time_ms);
    debugSerialPort->print("m_duty_ramp_step:      "); debugSerialPort->println(config.m_duty_ramp_step);
    debugSerialPort->print("m_current_backoff_gain:      "); debugSerialPort->println(config.m_current_backoff_gain);
    debugSerialPort->print("m_encoder_counts:      "); debugSerialPort->println(config.m_encoder_counts);
    debugSerialPort->print("m_sensor_port_mode:      "); debugSerialPort->println(config.m_sensor_port_mode);
    debugSerialPort->print("m_invert_direction:      "); debugSerialPort->println(config.m_invert_direction);
    debugSerialPort->print("m_drv8301_oc_mode:      "); debugSerialPort->println(config.m_drv8301_oc_mode);
    debugSerialPort->print("m_drv8301_oc_adj:      "); debugSerialPort->println(config.m_drv8301_oc_adj);
    debugSerialPort->print("m_bldc_f_sw_min:      "); debugSerialPort->println(config.m_bldc_f_sw_min);
    debugSerialPort->print("m_bldc_f_sw_max:      "); debugSerialPort->println(config.m_bldc_f_sw_max);
    debugSerialPort->print("m_dc_f_sw:      "); debugSerialPort->println(config.m_dc_f_sw);
    debugSerialPort->print("m_ntc_motor_beta:      "); debugSerialPort->println(config.m_ntc_motor_beta);
    debugSerialPort->print("m_out_aux_mode:      "); debugSerialPort->println(config.m_out_aux_mode);
    
    debugSerialPort->print("si_motor_poles:      "); debugSerialPort->println(config.si_motor_poles);
    debugSerialPort->print("si_gear_ratio:      "); debugSerialPort->println(config.si_gear_ratio);
    debugSerialPort->print("si_wheel_diameter:      "); debugSerialPort->println(config.si_wheel_diameter);
    debugSerialPort->print("si_battery_type:      "); debugSerialPort->println(config.si_battery_type);
    debugSerialPort->print("si_battery_cells:      "); debugSerialPort->println(config.si_battery_cells);
    debugSerialPort->print("si_battery_ah:      "); debugSerialPort->println(config.si_battery_ah);  
  }
}
