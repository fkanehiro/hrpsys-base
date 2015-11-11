// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/errno.h>
#include <sysexits.h>
#include <IOKit/hid/IOHIDLib.h>




#include "ysjoyreader.h"


YsJoyReaderElement::YsJoyReaderElement()
{
	exist=0;
	elem=NULL;
	value=0;
}

YsJoyReaderAxis::YsJoyReaderAxis()
{
	min=0;
	max=0;
	scaledMin=0;
	scaledMax=0;
	calibCenter=0;
	calibMin=0;
	calibMax=0;
}

double YsJoyReaderAxis::GetCalibratedValue(void) const
{
	double calib;

	if(calibCenter<value && calibMax!=calibCenter)
	{
		calib=(double)(value-calibCenter)/(double)(calibMax-calibCenter);
	}
	else if(value<calibCenter && calibMin!=calibCenter)
	{
		calib=(double)(value-calibCenter)/(double)(calibCenter-calibMin);
	}
	else
	{
		return 0.0;
	}

	if(calib>1.0)
	{
		calib=1.0;
	}
	if(calib<-1.0)
	{
		calib=-1.0;
	}

	return calib;
}

void YsJoyReaderAxis::CaptureCenter(void)
{
	calibCenter=value;
}

void YsJoyReaderAxis::BeginCaptureMinMax(void)
{
	calibMin=calibCenter+1000;
	calibMax=calibCenter-1000;
}

void YsJoyReaderAxis::CaptureMinMax(void)
{
	if(value<calibMin)
	{
		calibMin=value;
	}
	if(value>calibMax)
	{
		calibMax=value;
	}
}

void YsJoyReaderAxis::CenterFromMinMax(void)
{
	calibCenter=(calibMin+calibMax)/2;
}

YsJoyReaderButton::YsJoyReaderButton()
{
}

YsJoyReaderHatSwitch::YsJoyReaderHatSwitch()
{
	valueNeutral=0;
	value0Deg=1;
	value90Deg=3;
	value180Deg=5;
	value270Deg=7;
}

int YsJoyReaderHatSwitch::GetDiscreteValue(void) const
{
	if(value==valueNeutral)
	{
		return 0;
	}
	else if(value==value0Deg)
	{
		return 1;
	}
	else if(value==value90Deg)
	{
		return 3;
	}
	else if(value==value180Deg)
	{
		return 5;
	}
	else if(value270Deg==value)
	{
		return 7;
	}
	else if(value0Deg<value && value<value90Deg)
	{
		return 2;
	}
	else if(value90Deg<value && value<value180Deg)
	{
		return 4;
	}
	else if(value180Deg<value && value<value270Deg)
	{
		return 6;
	}
	else if(value270Deg<value)
	{
		return 8;
	}
	return 0;
}


IOHIDManagerRef YsJoyReader::hidManager=NULL;
CFMutableArrayRef YsJoyReader::devArray=NULL;

YsJoyReader::YsJoyReader()
{
	hidDev=NULL;
}

int YsJoyReader::SetUpInterface(int joyId,IOHIDDeviceRef hidDev)
{
	this->joyId=joyId;

	if(hidDev!=NULL)
	{
		CFArrayRef elemAry=IOHIDDeviceCopyMatchingElements(hidDev,NULL,0);
		int nElem=(int)CFArrayGetCount(elemAry);
		int isMouse=0,isJoystick=0,isKeyboard=0,isGamePad=0;

		printf("This HID Device has %d elements.\n",nElem);

		int j;
		for(j=0; j<nElem; j++)
		{
			IOHIDElementRef elem=(IOHIDElementRef)CFArrayGetValueAtIndex(elemAry,j);
			IOHIDElementType elemType=IOHIDElementGetType(elem);
			unsigned int usage=IOHIDElementGetUsage(elem);
			unsigned int usagePage=IOHIDElementGetUsagePage(elem);

			printf("Element %3d",j);
			switch(elemType)
			{
			case kIOHIDElementTypeInput_ScanCodes:
				printf(" ScanCode  ");
				break;
			case kIOHIDElementTypeInput_Misc:
				printf(" Misc      ");
				break;
			case kIOHIDElementTypeInput_Button:
				printf(" Button    ");
				break;
			case kIOHIDElementTypeInput_Axis:
				printf(" Axis      ");
				break;
			case kIOHIDElementTypeOutput:
				printf(" Output    ");
				break;
			case kIOHIDElementTypeFeature:
				printf(" Feature   ");
				break;
			case kIOHIDElementTypeCollection:
				printf(" Collection");
				break;
			}

			printf("  Usage %3d  UsagePage %3d\n",usage,usagePage);

			if(kHIDPage_GenericDesktop==usagePage)
			{
				switch(usage)
				{
				case kHIDUsage_GD_Mouse:
					printf("    Can function as mouse\n");
					isMouse=1;
					break;
				case kHIDUsage_GD_Keyboard:
					printf("    Can function as Keyboard\n");
					isKeyboard=1;
					break;
				case kHIDUsage_GD_Joystick:
					printf("    Can function as Joystick\n");
					isJoystick=1;
					break;
				case kHIDUsage_GD_GamePad:
					printf("    Can function as GamePad\n");
					isGamePad=1;
					break;
				}
			}
		}

		if(0!=isJoystick)
		{
			int nAxis=0;
			int nHat=0;

			int j;
			for(j=0; j<nElem; j++)
			{
				IOHIDElementRef elem=(IOHIDElementRef)CFArrayGetValueAtIndex(elemAry,j);
				IOHIDElementType elemType=IOHIDElementGetType(elem);
				unsigned int usage=IOHIDElementGetUsage(elem);
				unsigned int usagePage=IOHIDElementGetUsagePage(elem);
				// The following two returned 0 and 255
				// IOHIDElementGetPhysicalMin(elem);
				// IOHIDElementGetPhysicalMax(elem);
				int min=IOHIDElementGetLogicalMin(elem);
				int max=IOHIDElementGetLogicalMax(elem);
				int scaledMin=min;
				int scaledMax=max;

				if(elemType==kIOHIDElementTypeInput_Misc ||
				   elemType==kIOHIDElementTypeInput_Button ||
				   elemType==kIOHIDElementTypeInput_Axis ||
				   elemType==kIOHIDElementTypeInput_ScanCodes)
				{
					switch(usagePage)
					{
					case kHIDPage_GenericDesktop:
						switch(usage)
						{
						case kHIDUsage_GD_Mouse:
							break;
						case kHIDUsage_GD_Keyboard:
							break;
						case kHIDUsage_GD_Joystick:
							break;
						case kHIDUsage_GD_GamePad:
							break;
						case kHIDUsage_GD_X:
							printf("    This element is for X-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
							break;
						case kHIDUsage_GD_Y:
							printf("    This element is for Y-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
							break;
						case kHIDUsage_GD_Z:
							printf("    This element is for Z-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
							break;
						case kHIDUsage_GD_Rx:
							printf("    This element is for Rx-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
							break;
						case kHIDUsage_GD_Ry:
							printf("    This element is for Ry-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
							break;
						case kHIDUsage_GD_Rz:
							printf("    This element is for Rz-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
							break;
						case kHIDUsage_GD_Slider:
							printf("    This element is for Slider (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
							break;
						case kHIDUsage_GD_Wheel:
							printf("    This element is for Wheel (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							break;
						case kHIDUsage_GD_Hatswitch:
							printf("    This element is for Hatswitch (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
							if(nHat<YsJoyReaderMaxNumHatSwitch)
							{
								hatSwitch[nHat].exist=1;
								hatSwitch[nHat].elem=elem;
								CFRetain(elem);
								nHat++;
							}
							break;
						}
						break;
					case kHIDPage_Button:
						printf("    This element is for Button %d\n",usage-1);
						usage--;
						if(0<=usage && usage<YsJoyReaderMaxNumButton)
						{
							button[usage].exist=1;
							button[usage].elem=elem;
							CFRetain(elem);
						}
						break;
					}
				}
			}
			CFRelease(elemAry);
			this->hidDev=hidDev;
			return 1;

		}
		CFRelease(elemAry);
	}

	return 0;
}

void YsJoyReader::Read(void)
{
	int i;
	IOHIDValueRef valueRef;
	for(i=0; i<YsJoyReaderMaxNumAxis; i++)
	{
		if(axis[i].exist!=0)
		{
			IOHIDDeviceGetValue(hidDev,axis[i].elem,&valueRef);
			axis[i].value=IOHIDValueGetIntegerValue(valueRef);
		}
	}
	for(i=0; i<YsJoyReaderMaxNumButton; i++)
	{
		if(button[i].exist!=0)
		{
			IOHIDDeviceGetValue(hidDev,button[i].elem,&valueRef);
			button[i].value=IOHIDValueGetIntegerValue(valueRef);
		}
	}
	for(i=0; i<YsJoyReaderMaxNumHatSwitch; i++)
	{
		if(hatSwitch[i].exist!=0)
		{
			IOHIDDeviceGetValue(hidDev,hatSwitch[i].elem,&valueRef);

			double scaled=IOHIDValueGetScaledValue(valueRef,kIOHIDValueScaleTypePhysical);
			if(scaled<-0.001 || 359.999<scaled)
			{
				hatSwitch[i].value=0;
			}
			else
			{
				hatSwitch[i].value=1+(int)((scaled+22.5)/45.0);
			}
		}
	}
}

void YsJoyReader::ReleaseInterface(void)
{
	if(hidDev!=NULL)
	{
		// Honestly, I don't know what to do.
		//
		// Should I do
		//   CFRelease(hidDev);
		// ?
		//
		// This hidDev was copied from a copy of IOHIDManager's device list.
		// Who owns it?  Why did I have to make a copy?
		// 
		// The Creare Rule implies that I have the ownership.
		// http://developer.apple.com/mac/library/documentation/CoreFoundation/Conceptual/CFMemoryMgmt/Concepts/Ownership.html#//apple_ref/doc/uid/20001148-SW1
		//
		// Then, I suppose I should release it.  Am I right?
		CFRelease(hidDev);
		hidDev=NULL;
	}
}

void YsJoyReader::AddAxis(int axisId,IOHIDElementRef elem,int min,int max,int scaledMin,int scaledMax)
{
	if(0<=axisId && axisId<YsJoyReaderMaxNumAxis)
	{
		axis[axisId].exist=1;
		axis[axisId].elem=elem;
		axis[axisId].min=min;
		axis[axisId].max=max;
		axis[axisId].scaledMin=scaledMin;
		axis[axisId].scaledMax=scaledMax;

		axis[axisId].calibCenter=(min+max)/2;
		axis[axisId].calibMin=min;
		axis[axisId].calibMax=max;

		CFRetain(elem);
	}
}

void CFSetCopyCallBack(const void *value,void *context)
{
	CFArrayAppendValue((CFMutableArrayRef)context,value);
}

int YsJoyReader::SetUpJoystick(int &nJoystick,YsJoyReader joystick[],int maxNumJoystick)
{
	nJoystick=0;

	if(NULL==hidManager)
	{
		hidManager=IOHIDManagerCreate(kCFAllocatorDefault,kIOHIDOptionsTypeNone);
	}

	if(NULL!=hidManager)
	{
		IOHIDManagerSetDeviceMatching(hidManager,NULL);  // Just enumrate all devices
		IOHIDManagerOpen(hidManager,kIOHIDOptionsTypeNone);

		CFSetRef copyOfDevices=IOHIDManagerCopyDevices(hidManager);
		if(NULL!=devArray)
		{
			CFRelease(devArray);
			devArray=NULL;
		}
		devArray=CFArrayCreateMutable(kCFAllocatorDefault,0,&kCFTypeArrayCallBacks);
		CFSetApplyFunction(copyOfDevices,CFSetCopyCallBack,(void *)devArray);

		CFIndex nDev=CFArrayGetCount(devArray);

		printf("%d devices found\n",(int)nDev);

		CFRelease(copyOfDevices);



		int i;
		for(i=0; i<nDev && nJoystick<maxNumJoystick; i++)
		{
			IOHIDDeviceRef hidDev=(IOHIDDeviceRef)CFArrayGetValueAtIndex(devArray,i);
			if(joystick[nJoystick].SetUpInterface(nJoystick,hidDev)!=0)
			{
				nJoystick++;
				// CFRelease(hidDev);  // Doesn't it destroy integrity of devArray?
			}
		}
	}

	return nJoystick;
}

int YsJoyReader::WriteCalibInfoFile(FILE *fp) const
{
	int i;
	fprintf(fp,"BGNJOY %d\n",joyId);
	for(i=0; i<YsJoyReaderMaxNumAxis; i++)
	{
		if(0!=axis[i].exist)
		{
			fprintf(fp,"AXSINF %d %d %d %d\n",i,axis[i].calibCenter,axis[i].calibMin,axis[i].calibMax);
		}
	}
#ifdef YSJOYREADER_USE_HAT_CALIBRATION
	for(i=0; i<YsJoyReaderMaxNumHatSwitch; i++)
	{
		if(0!=hatSwitch[i].exist)
		{
			fprintf(fp,"HATINF %d %d %d %d %d %d\n",
			    i,
			    hatSwitch[i].valueNeutral,
			    hatSwitch[i].value0Deg,
			    hatSwitch[i].value90Deg,
			    hatSwitch[i].value180Deg,
			    hatSwitch[i].value270Deg);
		}
	}
#endif
	fprintf(fp,"ENDJOY\n");
	return 1;
}

int YsJoyReader::ReadCalibInfoFile(FILE *fp)
{
	char str[256];
	while(fgets(str,255,fp)!=NULL)
	{
		if(strncmp(str,"AXSINF",6)==0)
		{
			int axisId,cen,min,max;
			sscanf(str,"%*s %d %d %d %d",&axisId,&cen,&min,&max);
			if(0<=axisId && axisId<YsJoyReaderMaxNumAxis)
			{
				axis[axisId].calibCenter=cen;
				axis[axisId].calibMin=min;
				axis[axisId].calibMax=max;
			}
		}
#ifdef YSJOYREADER_USE_HAT_CALIBRATION
		else if(strncmp(str,"HATINF",6)==0)
		{
			int hatId;
			int valueNeutral=0,value0Deg=1,value90Deg=3,value180Deg=5,value270Deg=7;
			sscanf(str,"%*s %d %d %d %d %d %d",&hatId,&valueNeutral,&value0Deg,&value90Deg,&value180Deg,&value270Deg);
			if(0<=hatId && hatId<YsJoyReaderMaxNumHatSwitch)
			{
				hatSwitch[hatId].valueNeutral=valueNeutral;
				hatSwitch[hatId].value0Deg=value0Deg;
				hatSwitch[hatId].value90Deg=value90Deg;
				hatSwitch[hatId].value180Deg=value180Deg;
				hatSwitch[hatId].value270Deg=value270Deg;
			}
		}
#endif
		else if(strncmp(str,"ENDJOY",6)==0)
		{
			return 1;
		}
	}
	return 0;
}

int YsJoyReaderSetUpJoystick(int &nJoystick,YsJoyReader joystick[],int maxNumJoystick)
{
	return YsJoyReader::SetUpJoystick(nJoystick,joystick,maxNumJoystick);
}


extern "C" FILE *YsJoyReaderOpenJoystickCalibrationFileC(const char mode[]);

FILE *YsJoyReaderOpenJoystickCalibrationFile(const char mode[])
{
  return YsJoyReaderOpenJoystickCalibrationFileC(mode);
}

int YsJoyReaderSaveJoystickCalibrationInfo(int nJoystick,YsJoyReader joystick[])
{
	FILE *fp;
	fp=YsJoyReaderOpenJoystickCalibrationFile("w");

	if(fp!=NULL)
	{
		int i;
		for(i=0; i<nJoystick; i++)
		{
			joystick[i].WriteCalibInfoFile(fp);
		}

		fclose(fp);
		return 1;
	}
	return 0;
}

int YsJoyReaderLoadJoystickCalibrationInfo(int nJoystick,YsJoyReader joystick[])
{
	FILE *fp;
	fp=YsJoyReaderOpenJoystickCalibrationFile("r");

	if(fp!=NULL)
	{
		char str[256];
		while(fgets(str,255,fp)!=NULL)
		{
			if(strncmp(str,"BGNJOY",6)==0)
			{
				int joyId;
				sscanf(str,"%*s %d",&joyId);
				if(0<=joyId && joyId<nJoystick)
				{
					joystick[joyId].ReadCalibInfoFile(fp);
				}
			}
		}
		fclose(fp);
		return 1;
	}
	return 0;
}
