// Copyright (c) 2018 Cirque Corp. Restrictions apply. See: www.cirque.com/sw-license

// In order to get this to build, you have to run the moveFiles.bat (located in this directory) once before compile time. 
// This batch file copies the necessary includes from their source directories to this directory to satisfy Arduino's compiler (all files need to be in this directory). 
// If your local directory structure doesn't match the branch, the relative paths won't evaluate properly.
#include <EEPROM.h>
#include "API_C2.h"         /** < Provides API calls to interact with API_C2 firmware */
#include "API_Hardware.h"
#include "API_HostBus.h"    /** < Provides I2C connection to module */
#include "Adafruit_MPRLS_AllThumbs.h"

#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

const char* initCommand = "RQ_INIT_";
const char* sipSetThresholdCommand = "RQ_SETSIPTHRESH_";
const char* puffSetThresholdCommand = "RQ_SETPUFFTHRESH_";
const char* tapSetTimeoutCommand = "RQ_SETTAPTMOUT_";
const char* tapSetDistanceCommand = "RQ_SETTAPDSTTHRESH_";
const char* setZMinimumCommand = "RQ_SETMINZ_";
const char* setZMaxDeltaCommand = "RQ_SETMAXZDELTA_";
const char* setTouchThresholdCommand = "RQ_SETTOUCHTHRESHOLD_";
const char* setLiftThresholdCommand = "RQ_SETLIFTTHRESHOLD_";
const char* setLogicalScaleXMinCommand = "RQ_SETSCALEXMIN_";
const char* setLogicalScaleXMaxCommand = "RQ_SETSCALEXMAX_";
const char* setLogicalScaleYMinCommand = "RQ_SETSCALEYMIN_";
const char* setLogicalScaleYMaxCommand = "RQ_SETSCALEYMAX_";
const char* setADCGainCommand = "RQ_SETADCGAIN_";
const int sipThresholdAddr = 0; //Sip threshold setting EEPROM address
const int puffThresholdAddr = 1; //Puff threshold setting EEPROM address
const int tapDistanceAddr = 2; //Tap distance threshold setting EEPROM address
const int tapTimeoutAddr = 3; //Tap distance threshold setting EEPROM address
const int adcGainAddr = 4; //ADC Gain setting EEPROM address
const int minZAddr = 5; //Minimum Z threshold setting EEPROM address
const int maxZDeltaAddr = 6; //Maximum Z delta setting EEPROM address
const int zMinDefaultThreshold = 40;
int zMinThreshold = 40;
const int zMaxDefaultDelta = 10;
int zMaxDelta = 10;
const int sipDefaultThreshold = 10;
const int puffDefaultThreshold = 5;
int sipThreshold = 10; //Amount of pressure (hPa) below spBaseValue needed to trigger sending a sip event
int puffThreshold = 5; //Amount of pressure (hPa) above spBaseValue needed to trigger sending a puff event
float spBaseValue = 0; //Set upon init, used to determine if the current event is a sip or a puff
bool spPresent = false;
bool outputRaw = false;
bool outputRawOnlyChanges = false;

int startX = 0;
int startY = 0;
int startZX = 0;
int lastX = 0;
int lastY = 0;
int lastZX = 0;
int lastRawX = 0;
int lastRawY = 0;
int lastRawZX = 0;
bool lastRawFingerValid = false;
bool lastFingerValid = false;
ulong startMillis = 0;
ulong gestureStartMillis = 0;
bool gesture = false;
uint tapTimeoutDefaultThreshold = 12; //Multiplies by 8
uint tapMillisThreshold = 100;
float tapDistanceDefaultThreshold = 25;  //Multiplies by 2
float distanceThreshold = 50;
ulong lastMPRLSBaselineMillis = 0;
uint16_t gyScaleMin = 0;
uint16_t gyScaleMax = 2830;

report_t prevAbsReport_g;       /**< Most recent past CRQ_ABSOLUTE report */

/** Initializes reports used for keeping track of state.
    Each report will represent the most recent past report of
    its type. */
void initialize_saved_reports()
{
    prevAbsReport_g.reportID = CRQ_ABSOLUTE_REPORT_ID; //should be initialized... is there a conveninet way?
    prevAbsReport_g.abs.buttons = 0x0;
    prevAbsReport_g.abs.contactFlags = 0x0;
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);
  
  API_Hardware_init();       //Initialize board hardware
  
  API_Hardware_PowerOn();    //Power up the board
  delay(2);                  //delay for power up

  Serial.println("Initilizing Cirque Gen 4 device...");

  // initialize i2c connection at 400khz 
  API_C2_init(400000, CIRQUE_SLAVE_ADDR); 
  delay(50);                 //delay before reading registers after startup
  
  initialize_saved_reports(); //initialize state for determining touch events
  delay(50);

  Serial.println("Enabling Absolute mode");
  API_C2_setCRQ_AbsoluteMode();
  delay(50);

  Serial.println("Enabling Single Finger mode");
  API_C2_enableSingleFingerMode();
  delay(50);

  Serial.println("Enabling Advanced Absolute mode");
  API_C2_enableAdvancedAbs();
  delay(50);

  Serial.println("Locating MPRLS (pressure) sensor...");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS (pressure) sensor");
    Serial.println("Sip and Puff functionality not available");
    spPresent = false;
  } 
  else
  {
    Serial.println("Found MPRLS (pressure) sensor");
    spPresent = true;
    spBaseValue = GetSipPuffBaseValue();
    sipThreshold = GetSipThreshold();
    puffThreshold = GetPuffThreshold();
  }   

  tapMillisThreshold = GetTapTimeout();
  distanceThreshold = GetTapDistanceThreshold();
  zMinThreshold = GetZMinThreshold();
  zMaxDelta = GetZMazDelta();

  SetScalingValues();

  Serial.println("Ready");
}

/** The main structure of the loop is: 
    Wait for the Data Ready line to assert. When it does, read the data (which clears DR) and analyze the data.
    The rest is just a user interface to change various settings.
    */
void loop()
{
  /* Handle incoming messages from module */
  if(API_C2_DR_Asserted())          // When Data is ready
  {
    report_t report;
    API_C2_getReport(&report);    // read the report
    printDataReport(&report);
  }

  //Checks to see if the sensor is reporting the current pressure is +/- the corresponding threshold
  CheckForSipPuffEvent();

  //Only parses one pipe terminated command per call
  CheckForReceiveData();
}

void SetScalingValues()
{
  if(API_C2_logicalScalingEnabled())
  {
    gyScaleMin = API_C2_GetLogicalScalingYMin();
    gyScaleMax = API_C2_GetLogicalScalingYMax();
  }
  else
  {
    gyScaleMin = API_C2_GetActualYMin();
    gyScaleMax = API_C2_GetActualYMax();
  }  
}

float ReadPressure()
{
  if(!spPresent) return 0.00;
  float pressure_hPa = mpr.readPressure();
  return pressure_hPa;
}

void CheckForSipPuffEvent()
{
  float currentPressure = ReadPressure();
  if(currentPressure == 0.00) return;

  //Serial.println(currentPressure, 2);

  //Once per hour, refresh base pressure - this is to mitigate normal atmospheric pressure changes
  ulong currentMillis = millis();
  if( (abs(currentPressure - spBaseValue) < 30)             //Only if current reading is close to base value
      && (currentMillis > lastMPRLSBaselineMillis + 3600000 //Once per hour
          || currentMillis < lastMPRLSBaselineMillis        //In case of millis rollover
         )
    )
  {
    spBaseValue = currentPressure;
    lastMPRLSBaselineMillis = millis();
    Serial.print("Baseline pressure refreshed: ");
    Serial.print(currentPressure, 2);
    Serial.println(" (hPa)");    
  }

  if(currentPressure >= (puffThreshold + spBaseValue))
  {
    Serial.print("P|");
    Serial.println(currentPressure, 2);
  }
  else if (currentPressure <= (spBaseValue - sipThreshold))
  {
    Serial.print("S|");
    Serial.println(currentPressure, 2);
  }
}

float GetSipPuffBaseValue()
{
  //Read sensor for current pressure
  float currentPressure = ReadPressure();
  Serial.print("Baseline pressure: ");
  Serial.print(currentPressure, 2);
  Serial.println(" (hPa)");
  lastMPRLSBaselineMillis = millis();
  return currentPressure;
}

int GetSipThreshold()
{
  int tempSipThreshold = EEPROM.read(sipThresholdAddr);
  if(tempSipThreshold > 254 || tempSipThreshold <= 0)
  {
    EEPROM.update(sipThresholdAddr,sipDefaultThreshold); 
    tempSipThreshold = sipDefaultThreshold;
  }
  Serial.print("Current Sip pressure threshold: ");
  Serial.print(tempSipThreshold, DEC);
  Serial.println(" (hPa)");
  return tempSipThreshold;
}

int GetPuffThreshold()
{
  int tempPuffThreshold = EEPROM.read(puffThresholdAddr);
  if(tempPuffThreshold > 254 || tempPuffThreshold <= 0)
  {
    EEPROM.update(puffThresholdAddr,puffDefaultThreshold); 
    tempPuffThreshold = puffDefaultThreshold;
  }
  Serial.print("Current Puff pressure threshold: ");
  Serial.print(tempPuffThreshold, DEC);
  Serial.println(" (hPa)");
  return tempPuffThreshold;
}

int GetTapDistanceThreshold()
{
  int tempTapDistanceThreshold = EEPROM.read(tapDistanceAddr);
  if(tempTapDistanceThreshold > 254 || tempTapDistanceThreshold <= 0)
  {
    EEPROM.update(tapDistanceAddr,tapTimeoutDefaultThreshold); 
    tempTapDistanceThreshold = tapDistanceDefaultThreshold;
  }
  tempTapDistanceThreshold = tempTapDistanceThreshold * 2;
  Serial.print("Current Tap distance threshold: ");
  Serial.print(tempTapDistanceThreshold, DEC);
  Serial.println(" (touchpad points)");
  return tempTapDistanceThreshold;
}

int GetTapTimeout()
{
  int tempTapTimeoutThreshold = EEPROM.read(tapTimeoutAddr);
  if(tempTapTimeoutThreshold > 254 || tempTapTimeoutThreshold <= 0)
  {
    EEPROM.update(tapTimeoutAddr,tapTimeoutDefaultThreshold); 
    tempTapTimeoutThreshold = tapTimeoutDefaultThreshold;
  }
  tempTapTimeoutThreshold = tempTapTimeoutThreshold * 8;
  Serial.print("Current Tap timeout: ");
  Serial.print(tempTapTimeoutThreshold, DEC);
  Serial.println(" (ms)");
  return tempTapTimeoutThreshold;  
}

int GetZMinThreshold()
{
  int tempZMinThreshold = EEPROM.read(minZAddr);
  if(tempZMinThreshold > 254 || tempZMinThreshold <= 0)
  {
    EEPROM.update(minZAddr,zMinDefaultThreshold); 
    tempZMinThreshold = zMinDefaultThreshold;
  }
  Serial.print("Current Minimum Z Threshold: ");
  Serial.println(tempZMinThreshold, DEC);
  return tempZMinThreshold;    
}

int GetZMazDelta()
{
  int tempZMazDelta = EEPROM.read(maxZDeltaAddr);
  if(tempZMazDelta > 254 || tempZMazDelta <= 0)
  {
    EEPROM.update(maxZDeltaAddr,zMaxDefaultDelta); 
    tempZMazDelta = zMaxDefaultDelta;
  }
  Serial.print("Current Maximum Z Delta: ");
  Serial.println(tempZMazDelta, DEC);
  return tempZMazDelta;    
}

void CheckForReceiveData()
{
  //This function will only parse one command token then return
  //Make sure responses do not contain commas as client will encounter 
  //an exception attempting to parse x,y,z
  
  //If there is data waiting to be received
  if(Serial.available() > 0)
  {
    delay(50);
    const int maxBytes = 110; //Safeguard to break out of the loop
    int totalBytes = 0;
    int makeSerialStringPosition = 0;
    int inByte;
    char serialReadString[50];
    const int terminatingChar = 124; //pipe (|)    
    inByte = Serial.read();

    //
    if (inByte > 0 && inByte != terminatingChar)
    {
      while (inByte != terminatingChar && Serial.available() > 0)
      {
        serialReadString[makeSerialStringPosition] = inByte; 
        makeSerialStringPosition++; 
        
        //If the incoming string exceeds the limit before finding the terminating char
        //reset the char array and send an error message
        if(makeSerialStringPosition > 49)
        {
          memset(serialReadString, 0, sizeof(serialReadString));
          makeSerialStringPosition = 0;  
          Serial.print("RQE|LINE_LENGTH_EXCEEDED: Reset and continue reading from postition ");        
          Serial.println(totalBytes, DEC);
        }

        //Get the next byte
        inByte = Serial.read(); 
        totalBytes++;
        
        //Too many bytes without finding a terminating char, exit
        if(totalBytes >= maxBytes)
        {
          Serial.print("RQE|MAX_BYTES_EXCEEDED: Discarding bytes 0-");
          Serial.println(totalBytes, DEC);
          return;
        }
      }

      //If the last char is the terminating, else all available bytes have been read
      if (inByte == terminatingChar) 
      {
        serialReadString[makeSerialStringPosition] = 0; 

        //Check for known commands
        if (strcmp(serialReadString, "RQ_ECHO") == 0)
        {
          Serial.println("RQS|HI");
        }
        else if (strcmp(serialReadString, "RQ_OUTPUTFMTRAW") == 0)
        {
          outputRaw = true;
          outputRawOnlyChanges = false;
          Serial.println("RQS|OK");
        }  
        else if (strcmp(serialReadString, "RQ_OUTPUTFMTRAWONLYCHG") == 0)
        {
          outputRaw = true;
          outputRawOnlyChanges = true;
          Serial.println("RQS|OK");
        }   
        else if (strcmp(serialReadString, "RQ_OUTPUTFMTLOGIC") == 0)
        {
          outputRaw = false;
          Serial.println("RQS|OK");
        } 
        else if (strcmp(serialReadString, "RQ_SIPPUFFREFRESHBASE") == 0)
        {
          spBaseValue = GetSipPuffBaseValue();
        }        
        else if (strcmp(serialReadString, "RQ_REBOOT") == 0)
        {
          WRITE_RESTART(0x5FA0004);
        }
        else if (strstr(serialReadString, tapSetDistanceCommand))
        {
          String tapDistanceData = serialReadString;
          tapDistanceData.replace("RQ_SETTAPDSTTHRESH_", "");
          int tempTapDistance = tapDistanceData.toInt();
          if (tempTapDistance <= 0 || tempTapDistance > 254)
          {
            Serial.println("RQE|Invalid value - must be 1-254");
          }
          else
          {
            distanceThreshold = tempTapDistance;
            EEPROM.update(tapDistanceAddr,distanceThreshold); 
            distanceThreshold = distanceThreshold * 2;
            Serial.println("RQS|OK");
          }
        }   
        else if (strstr(serialReadString, tapSetTimeoutCommand))
        {
          String tapTimeouteData = serialReadString;
          tapTimeouteData.replace("RQ_SETTAPTMOUT_", "");
          int tempTapTimeout = tapTimeouteData.toInt();
          if (tempTapTimeout <= 0 || tempTapTimeout > 254)
          {
            Serial.println("RQE|Invalid value - must be 1-254");
          }
          else
          {
            tapMillisThreshold = tempTapTimeout;
            EEPROM.update(tapTimeoutAddr,tapMillisThreshold); 
            tapMillisThreshold = tapMillisThreshold * 8;
            Serial.println("RQS|OK");
          }
        } 
        else if (strstr(serialReadString, sipSetThresholdCommand))
        {
          String sipThreshData = serialReadString;
          sipThreshData.replace("RQ_SETSIPTHRESH_", "");
          int tempSipThreshold = sipThreshData.toInt();
          if (tempSipThreshold <= 0 || tempSipThreshold > 254)
          {
            Serial.println("RQE|Invalid value - must be 1-254");
          }
          else
          {
            sipThreshold = tempSipThreshold;
            EEPROM.update(sipThresholdAddr,sipThreshold); 
            Serial.println("RQS|OK");
          }
        }   
        else if (strstr(serialReadString, puffSetThresholdCommand))
        {
          String puffThreshData = serialReadString;
          puffThreshData.replace("RQ_SETPUFFTHRESH_", "");
          int tempPuffThreshold = puffThreshData.toInt();
          if (tempPuffThreshold <= 0 || tempPuffThreshold > 254)
          {
            Serial.println("RQE|Invalid value - must be 1-254");
          }
          else
          {
            puffThreshold = tempPuffThreshold;
            EEPROM.update(puffThresholdAddr,puffThreshold); 
            Serial.println("RQS|OK");
          }
        } 
        else if (strstr(serialReadString, setTouchThresholdCommand))
        {
          String touchThreshData = serialReadString;
          touchThreshData.replace("RQ_SETTOUCHTHRESHOLD_", "");
          int tempTouchThreshold = touchThreshData.toInt();
          if (tempTouchThreshold <= 0 || tempTouchThreshold > 65534)
          {
            Serial.println("RQE|Invalid value - must be 1-65534");
          }
          else
          {
            API_C2_SetFingerTouchThreshold(tempTouchThreshold);
            Serial.println("RQS|OK");
          }
        }
        else if (strstr(serialReadString, setLiftThresholdCommand))
        {
          String liftThreshData = serialReadString;
          liftThreshData.replace("RQ_SETLIFTTHRESHOLD_", "");
          int tempLiftThreshold = liftThreshData.toInt();
          if (tempLiftThreshold <= 0 || tempLiftThreshold > 65534)
          {
            Serial.println("RQE|Invalid value - must be 1-65534");
          }
          else
          {
            API_C2_SetFingerLiftOffThreshold(tempLiftThreshold);
            Serial.println("RQS|OK");
          }
        }        
        else if (strstr(serialReadString, setZMinimumCommand))
        {
          String zMinThreshData = serialReadString;
          zMinThreshData.replace("RQ_SETMINZ_", "");
          int tempZMinThreshold = zMinThreshData.toInt();
          if (tempZMinThreshold <= 0 || tempZMinThreshold > 254)
          {
            Serial.println("RQE|Invalid value - must be 1-254");
          }
          else
          {
            zMinThreshold = tempZMinThreshold;
            EEPROM.update(minZAddr,zMinThreshold); 
            Serial.println("RQS|OK");
          }
        }  
        else if (strstr(serialReadString, setZMaxDeltaCommand))
        {
          String zMaxDeltaData = serialReadString;
          zMaxDeltaData.replace("RQ_SETMAXZDELTA_", "");
          int tempZMaxDelta = zMaxDeltaData.toInt();
          if (tempZMaxDelta <= 0 || tempZMaxDelta > 254)
          {
            Serial.println("RQE|Invalid value - must be 1-254");
          }
          else
          {
            zMaxDelta = tempZMaxDelta;
            EEPROM.update(maxZDeltaAddr,zMaxDelta); 
            Serial.println("RQS|OK");
          }
        } 
        else if (strcmp(serialReadString, "RQ_GETADCGAIN") == 0)
        {
          uint8_t adcGainValue = API_C2_getADCGain();
          Serial.print("RQS|");
          Serial.println(adcGainValue, DEC);
        }
        else if (strcmp(serialReadString, "RQ_FORCECOMP") == 0)
        {
          API_C2_forceComp();
          delay(50);
          Serial.println("RQS|OK");
        }
        else if (strstr(serialReadString, setADCGainCommand))
        {
          String adcGainData = serialReadString;
          adcGainData.replace("RQ_SETADCGAIN_", "");
          int tempADCGain = adcGainData.toInt();
          if (tempADCGain < 147 || tempADCGain > 150)
          {
            Serial.println("RQE|Invalid value - must be 148-150");
          }
          else
          {
            API_C2_setADCGain(tempADCGain);
            delay(10);
            API_C2_forceComp();
            delay(50);
            Serial.println("RQS|OK");
          }
        } 
        else if (strcmp(serialReadString, "RQ_GETACTXMIN") == 0)
        {
          uint16_t xActualMin = API_C2_GetActualXMin();
          Serial.print("RQS|");
          Serial.println(xActualMin, DEC);
        }
        else if (strcmp(serialReadString, "RQ_GETACTXMAX") == 0)
        {
          uint16_t xActualMax = API_C2_GetActualXMax();
          Serial.print("RQS|");
          Serial.println(xActualMax, DEC);
        }
        else if (strcmp(serialReadString, "RQ_GETACTYMIN") == 0)
        {
          uint16_t yActualMin = API_C2_GetActualYMin();
          Serial.print("RQS|");
          Serial.println(yActualMin, DEC);
        }
        else if (strcmp(serialReadString, "RQ_GETACTYMAX") == 0)
        {
          uint16_t yActualMax = API_C2_GetActualYMax();
          Serial.print("RQS|");
          Serial.println(yActualMax, DEC);
        }
        else if (strcmp(serialReadString, "RQ_GETSCALEXMIN") == 0)
        {
          uint16_t xScaleMin = API_C2_GetLogicalScalingXMin();
          Serial.print("RQS|");
          Serial.println(xScaleMin, DEC);
        }    
        else if (strcmp(serialReadString, "RQ_GETSCALEXMAX") == 0)
        {
          uint16_t xScaleMax = API_C2_GetLogicalScalingXMax();
          Serial.print("RQS|");
          Serial.println(xScaleMax, DEC);
        } 
        else if (strcmp(serialReadString, "RQ_GETSCALEYMIN") == 0)
        {
          uint16_t yScaleMin = API_C2_GetLogicalScalingYMin();
          Serial.print("RQS|");
          Serial.println(yScaleMin, DEC);
        }    
        else if (strcmp(serialReadString, "RQ_GETSCALEYMAX") == 0)
        {
          uint16_t yScaleMax = API_C2_GetLogicalScalingYMax();
          Serial.print("RQS|");
          Serial.println(yScaleMax, DEC);
        }         
        else if (strstr(serialReadString, setLogicalScaleXMinCommand ))
        {
          String scaleXMinData = serialReadString;
          scaleXMinData.replace("RQ_SETSCALEXMIN_", "");
          int tempScaleXMinData = scaleXMinData.toInt();
          if (tempScaleXMinData < 0 || tempScaleXMinData > 65534)
          {
            Serial.println("RQE|Invalid value - must be 0-65534");
          }
          else
          {
            API_C2_SetLogicalScalingXMin(tempScaleXMinData);
            delay(50);
            SetScalingValues();
            Serial.println("RQS|OK");
          }
        } 
        else if (strstr(serialReadString, setLogicalScaleXMaxCommand ))
        {
          String scaleXMaxData = serialReadString;
          scaleXMaxData.replace("RQ_SETSCALEXMAX_", "");
          int tempScaleXMaxData = scaleXMaxData.toInt();
          if (tempScaleXMaxData < 0 || tempScaleXMaxData > 65534)
          {
            Serial.println("RQE|Invalid value - must be 0-65534");
          }
          else
          {
            API_C2_SetLogicalScalingXMax(tempScaleXMaxData);
            delay(50);
            SetScalingValues();
            Serial.println("RQS|OK");
          }
        } 
        else if (strstr(serialReadString, setLogicalScaleYMinCommand ))
        {
          String scaleYMinData = serialReadString;
          scaleYMinData.replace("RQ_SETSCALEYMIN_", "");
          int tempScaleYMinData = scaleYMinData.toInt();
          if (tempScaleYMinData < 0 || tempScaleYMinData > 65534)
          {
            Serial.println("RQE|Invalid value - must be 0-65534");
          }
          else
          {
            API_C2_SetLogicalScalingYMin(tempScaleYMinData);
            delay(50);
            SetScalingValues();
            Serial.println("RQS|OK");
          }
        } 
        else if (strstr(serialReadString, setLogicalScaleYMaxCommand ))
        {
          String scaleYMaxData = serialReadString;
          scaleYMaxData.replace("RQ_SETSCALEYMAX_", "");
          int tempScaleYMaxData = scaleYMaxData.toInt();
          if (tempScaleYMaxData < 0 || tempScaleYMaxData > 65534)
          {
            Serial.println("RQE|Invalid value - must be 0-65534");
          }
          else
          {
            API_C2_SetLogicalScalingYMax(tempScaleYMaxData);
            delay(50);
            SetScalingValues();
            Serial.println("RQS|OK");
          }
        } 
        else if (strcmp(serialReadString, "RQ_GETSIPTHRESH") == 0)
        {
          Serial.print("RQS|");
          Serial.println(sipThreshold, DEC);
        }  
        else if (strcmp(serialReadString, "RQ_GETPUFFTHRESH") == 0)
        {
          Serial.print("RQS|");
          Serial.println(puffThreshold, DEC);
        }     
        else if (strcmp(serialReadString, "RQ_ISSCALINGENABLED") == 0)
        {
          bool scalingEnabled = API_C2_logicalScalingEnabled();
          Serial.print("RQS|");
          Serial.println(scalingEnabled ? "YES" : "NO");
        } 
        else if (strcmp(serialReadString, "RQ_ENABLESCALING") == 0)
        {
          API_C2_enableLogicalScaling();
          delay(50);
          SetScalingValues();
          Serial.println("RQS|OK");
        }  
        else if (strcmp(serialReadString, "RQ_DISABLESCALING") == 0)
        {
          API_C2_disableLogicalScaling();
          delay(50);
          SetScalingValues();
          Serial.println("RQS|OK");
        }  
        else if (strcmp(serialReadString, "RQ_GETTOUCHTHRESHOLD") == 0)
        {
          uint16_t touchThreshold = API_C2_GetFingerTouchThreshold();
          Serial.print("RQS|");
          Serial.println(touchThreshold, DEC);
        }    
        else if (strcmp(serialReadString, "RQ_GETLIFTTHRESHOLD") == 0)
        {
          uint16_t liftThreshold = API_C2_GetFingerLiftOffThreshold();
          Serial.print("RQS|");
          Serial.println(liftThreshold, DEC);
        } 
        else if (strstr(serialReadString, initCommand))
        {
          String initData = serialReadString;
          initData.replace("RQ_INIT_", "");

          //Do something with initData if needed
          //Not currently implemented
          //Serial.println(initData);
          
          Serial.print("RQS|OK");
        }
        else
        {
          Serial.print("RQE|UNKNOWN_COMMAND: ");
          Serial.println(serialReadString);
        }
      }
      else
      {
        //End of available bytes, but no terminating char found
        serialReadString[makeSerialStringPosition] = inByte;
        serialReadString[makeSerialStringPosition+1] = 0;
        Serial.print("RQE|TERMINATING_CHAR_MISSING: ");
        Serial.println(serialReadString);
      }
    }
  }  
}

void printDataReport(report_t * report)
{
  //Use reportID to determine how to decode the report
  switch(report->reportID)
  {
    case CRQ_ABSOLUTE_REPORT_ID:
        if(outputRaw)
        {
          OutputRawData(report);   
        }
        else
        {
          OutputTouchData(report);             
        }
        break;
    default:
        Serial.print(F("Error: Unknown Report ID: "));
        Serial.println(report->reportID, HEX);
  }
}

void OutputRawData(report_t * report)
{
  int currX = report->abs.fingers[0].x;
  int currY = map(report->abs.fingers[0].y, gyScaleMin, gyScaleMax, gyScaleMax, gyScaleMin);
  int currZX = report->abs.fingers[0].zx;  
  bool currFingerValid = API_C2_isFingerValid(report,0);

  if(outputRawOnlyChanges)
  {
    if(lastRawX == 0 && !currFingerValid)
    {
      return;
    }    
    if(lastRawX != currX || lastRawY != currY || lastRawFingerValid != currFingerValid)
    {
      Serial.print(currX, DEC);
      Serial.print(",");
      Serial.print(currY, DEC);
      Serial.print(",");
      Serial.print(currZX, DEC);
      Serial.print(",");
      Serial.println(currFingerValid ? "1" : "0");
    }
    if(!currFingerValid)
    {
      lastRawX = 0;
      lastRawY = 0;
      lastRawZX = 0;
    }  
    else
    {
      lastRawX = currX;
      lastRawY = currY;
      lastRawZX = currZX;
    }
    lastRawFingerValid = currFingerValid;
  }
  else
  {
    Serial.print(currX, DEC);
    Serial.print(",");
    Serial.print(currY, DEC);
    Serial.print(",");
    Serial.print(currZX, DEC);
    Serial.print(",");
    Serial.println(currFingerValid ? "1" : "0");
  }
}

/** Outputs the information stored in a CRQ_ABSOLUTE report to serial*/
void OutputTouchData(report_t * report)
{
  ulong currentMillis = millis();  
  int currX = report->abs.fingers[0].x;
  int currY = map(report->abs.fingers[0].y, gyScaleMin, gyScaleMax, gyScaleMax, gyScaleMin);
  bool currFingerValid = API_C2_isFingerValid(report,0);
  int currZX = report->abs.fingers[0].zx;
  if(!currFingerValid || currZX < zMinThreshold)
  {
    currZX = 0;
  }
  if(currFingerValid && currZX == 0)
  {
    currFingerValid = false;
  }

  if(startX == 0 && currFingerValid)
  {
    startMillis = currentMillis;
    //lastMillis = startMillis;
    startX = currX;
    startY = currY;
    startZX = currZX;
    lastX = currX;
    lastY = currY;
    lastZX = currZX;
    lastFingerValid = currFingerValid;
  }

  float distance;
  distance = sqrt(((currX-startX)*(currX-startX))+((currY-startY)*(currY-startY)));

  if(currFingerValid && currentMillis < (gestureStartMillis + tapMillisThreshold))
  {
    gesture = true;
  }
  if(distance > distanceThreshold || currentMillis > (startMillis + tapMillisThreshold))
  {
    if(lastX != currX || lastY != currY || lastFingerValid != currFingerValid)
    {
      if(currFingerValid && lastX > 0 && lastZX - currZX > zMaxDelta) // && lastDistance < distanceThreshold)
      {
        //Discard event, finger lift cursor jump prevention
        return;
      }      
      Serial.print(currFingerValid ? currX : lastX == 0 ? currX : lastX, DEC);
      Serial.print(F(","));
      Serial.print(currFingerValid ? currY : lastY == 0 ? currY : lastY, DEC);
      Serial.print(F(","));
      if(currFingerValid)
      {
        Serial.print(currZX, DEC);  
        Serial.print(F("|F"));  
      }
      else
      {
        Serial.print(F("0|"));  
      }
      Serial.println(gesture && currFingerValid ? F("G"):F(""));
      lastX = currX;
      lastY = currY;
      lastZX = currZX;
      lastFingerValid = currFingerValid;
    }    
  }
  else
  {
    if (startX > 0 && !currFingerValid)
    {
      Serial.print(currX, DEC);
      Serial.print(F(","));
      Serial.print(currY, DEC);
      Serial.print(F(","));
      Serial.print(startZX, DEC);  
      Serial.println(F("|G"));       
      gestureStartMillis = currentMillis;
    }
  }

  if(!currFingerValid)
  {
    startX = 0;
    startY = 0;
    gesture = false;
  }  
}
