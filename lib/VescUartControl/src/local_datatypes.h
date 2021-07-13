#ifndef LOCAL_DATATYPES_H_
#define LOCAL_DATATYPES_H_

// Added by AC to store measured values
typedef struct {
	float tempFetFiltered;
	float tempMotorFiltered;
	float avgMotorCurrent;
	float avgInputCurrent;
	float avgId;
	float avgIq;
	float dutyNow;
	long rpm;
	float inpVoltage;
	float ampHours;
	float ampHoursCharged;
	float wattHours;
	float watthoursCharged;
	long tachometer;
	long tachometerAbs;
	int faultCode;
} bldcMeasure;


//Define remote Package



struct nunchuckPackage {

	int		valXJoy;
	int		valYJoy;
	boolean	valUpperButton;
	boolean	valLowerButton;

};

#endif
