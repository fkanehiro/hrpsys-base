#ifndef YSMACOSXJOYSTICK_IS_INCLUDED
#define YSMACOSXJOYSTICK_IS_INCLUDED
/* { */

const int YsJoyReaderMaxNumAxis=6;
const int YsJoyReaderMaxNumButton=32;
const int YsJoyReaderMaxNumHatSwitch=4;

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/errno.h>
#include <sysexits.h>
#include <IOKit/hid/IOHIDLib.h>



class YsJoyReaderElement
{
public:
	int exist;
	IOHIDElementRef elem;
	int value;

	YsJoyReaderElement();
};

class YsJoyReaderAxis : public YsJoyReaderElement
{
public:
	int min,max;
	int scaledMin,scaledMax;
	int calibCenter,calibMin,calibMax;

	YsJoyReaderAxis();
	double GetCalibratedValue(void) const;

	void CaptureCenter(void);
	void BeginCaptureMinMax(void);
	void CaptureMinMax(void);
	void CenterFromMinMax(void);
};

class YsJoyReaderButton : public YsJoyReaderElement
{
public:
	YsJoyReaderButton();
};

class YsJoyReaderHatSwitch : public YsJoyReaderElement
{
public:
	YsJoyReaderHatSwitch();
	int valueNeutral;
	int value0Deg;
	int value90Deg;
	int value180Deg;
	int value270Deg;
	int GetDiscreteValue(void) const;
};

class  YsJoyReader
{
public:
	static IOHIDManagerRef hidManager;
	static CFMutableArrayRef devArray;

	int joyId;
	IOHIDDeviceRef hidDev;
	char regPath[512];

	YsJoyReaderAxis axis[YsJoyReaderMaxNumAxis];
	YsJoyReaderButton button[YsJoyReaderMaxNumButton];
	YsJoyReaderHatSwitch hatSwitch[YsJoyReaderMaxNumHatSwitch];

	YsJoyReader();
	int SetUpInterface(int joyId,IOHIDDeviceRef hidDev);
	void Read(void);
	void ReleaseInterface(void);

	int WriteCalibInfoFile(FILE *fp) const;
	int ReadCalibInfoFile(FILE *fp);

protected:
	void AddAxis(int axisId,IOHIDElementRef elem,int min,int max,int scaledMin,int scaledMax);

public:
	static int SetUpJoystick(int &nJoystick,YsJoyReader joystick[],int maxNumJoystick);
};




int YsJoyReaderSetUpJoystick(int &nJoystick,YsJoyReader joystick[],int maxNumJoystick);

FILE *YsJoyReaderOpenJoystickCalibrationFile(const char mode[]);

int YsJoyReaderSaveJoystickCalibrationInfo(int nJoystick,YsJoyReader joystick[]);
int YsJoyReaderLoadJoystickCalibrationInfo(int nJoystick,YsJoyReader joystick[]);

/* } */
#endif
