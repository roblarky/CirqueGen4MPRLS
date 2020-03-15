// Copyright (c) 2018 Cirque Corp. Restrictions apply. See: www.cirque.com/sw-license

#include "API_C2.h"

/***********************************************************/
/***********************************************************/
/********************* HELPER FUNCTIONS ********************/

/** Decodes a packet of a mouse report and puts it into result. */
void decodeMouseReport(uint8_t* packet, report_t* result)
{
    uint8_t* iter = &packet[2];
    if(*iter != MOUSE_REPORT_ID)
    {
        // not a mouse report 
        return; 
    }

    iter++;
    result->mouse.buttons = *iter++;
    result->mouse.xDelta = (int8_t) *iter++;
    result->mouse.yDelta = (int8_t) *iter++;
    result->mouse.scrollDelta = (int8_t) *iter++;
    result->mouse.panDelta = (int8_t) *iter;
}

/** Decodes a keyboard report packet into result. */
void decodeKeyboardReport(uint8_t* packet, report_t* result)
{
    uint8_t* iter = &packet[2];
    if(*iter != KEYBOARD_REPORT_ID)
    {
        // not a keyboard report 
        return; // probably shouldn't leave cleanly
    }
  
    iter++;
    result->keyboard.modifier = *iter++;
    iter++;             //reserved byte
    result->keyboard.keycode[0] = *iter++;
    result->keyboard.keycode[1] = *iter++;
    result->keyboard.keycode[2] = *iter++;
    result->keyboard.keycode[3] = *iter++;
    result->keyboard.keycode[4] = *iter++;
    result->keyboard.keycode[5] = *iter++;
}

uint16_t readFingerStrengthX()
{
    uint16_t xStrength = API_C2_readRegister(0xC384);
    xStrength |= API_C2_readRegister(0xC385) << 8;
    return xStrength;
}

uint16_t readFingerStrengthY()
{
    uint16_t yStrength = API_C2_readRegister(0xC386);
    yStrength |= API_C2_readRegister(0xC387) << 8;
    return yStrength;
}

uint16_t readXPhysicalSize()
{
    uint16_t xSize = API_C2_readRegister(0xC356);
    xSize |= API_C2_readRegister(0xC357) << 8;
    return xSize;
}

uint16_t readYPhysicalSize()
{
    uint16_t ySize = API_C2_readRegister(0xC358);
    ySize |= API_C2_readRegister(0xC359) << 8;
    return ySize;
}

void decodeCirqueAbsoluteReport(uint8_t* packet, report_t* result)
{
    uint8_t* iter = &packet[2];
    uint8_t* zx = &packet[35];
    //uint8_t* zy = &packet[40];
    if(*iter != CRQ_ABSOLUTE_REPORT_ID)
    {
        // not a absolute report 
        return; 
    }
    iter++;
    result->abs.contactFlags = *iter++;

    /*
    result->abs.xStrength = readFingerStrengthX();
    result->abs.yStrength = readFingerStrengthY();
    */
    
    uint8_t i;
    for(i = 0; i < 5; i++)
    {
        result->abs.fingers[i].palm = *iter++;
        
        result->abs.fingers[i].x = (uint16_t ) *iter++;
        result->abs.fingers[i].x |= (uint16_t ) *iter++ << 8;
        
        result->abs.fingers[i].y = (uint16_t ) *iter++;
        result->abs.fingers[i].y |= (uint16_t ) *iter++ << 8;

        if(i == 0) 
        {
          result->abs.fingers[i].zx = (uint8_t ) *zx;
          //result->abs.fingers[i].zy = (uint8_t ) *zy; //Always seems to be the same as zx
        }
    }
    result->abs.buttons = *iter;
}

/** Clears all values of report to zero. 
    Assumes absolute report is the largest of the union. */
void clearReport(report_t* report)
{
    report->reportID = 0;
    // Clears the values in the Union.
    report->abs.contactFlags = 0;
    uint8_t i;
    for(i = 0; i < 5; i++) // each of the 5 fingers
    {
        report->abs.fingers[i].palm = 0;
        report->abs.fingers[i].x = 0;
        report->abs.fingers[i].y = 0;
    }
    report->abs.buttons = 0;
}

uint16_t read16bitRegister(uint32_t address)
{
    uint8_t contents[2] = {0,0};
    HB_readExtendedMemory(address, contents, 2);
    uint16_t result = (uint16_t) contents[0] | (contents[1] << 8);
    return result;
}

/***********************************************************/
/***********************************************************/
/******************* IMPORTANT FUNCTIONS *******************/

/** Initializes the Host Bus I2C connection. Must be run prior 
	to any API calls the I2C bus commonly runs at a clock 
	frequency of 400kHz. The operation of touch system is also 
	tested at 100kHz. */
void API_C2_init(int32_t I2CFrequency, uint8_t I2CAddress)
{
    HB_init(I2CFrequency, I2CAddress);
}

/** Shows when data is available. */ 
bool API_C2_DR_Asserted(void)
{
    return (HB_DR_Asserted()); 
}

/** Reads a report from the Host Bus. The report is read and 
	decoded into the result parameter. Call this when the 
	DR line is asserted. */
void API_C2_getReport(report_t* result) //<-- double check this function with HID reports.
{
    //read packet
    uint8_t packet[PACKET_SIZE]; 
    HB_readReport(packet, PACKET_SIZE); //fills packet with i2c packet
    API_C2_decodeReport(packet, result);
}

/** Reads the contents of a register at a given address */
uint8_t API_C2_readRegister(uint32_t address)
{
    uint8_t contents = 0;
    HB_readExtendedMemory(address, &contents, 1);
    return contents;
}

/** Sets register contents. It is best to first read a register, 
    modify the necessary bits, then write it back. */
void API_C2_writeRegister(uint32_t address, uint8_t value)
{
    HB_writeExtendedMemory(address, &value, 1);
}

/** Reads the System info and puts it into result. */
void API_C2_readSystemInfo(systemInfo_t* result)
{
    result->chipId = API_C2_readRegister(REG_CHIP_ID);
    result->firmwareVersion = API_C2_readRegister(REG_FIRMWARE_VER);
    result->firmwareSubversion = API_C2_readRegister(REG_FIRMWARE_SUBVERSION);
    result->vendorId = read16bitRegister(REG_VENDOR_ID);
    result->productId = read16bitRegister(REG_PRODUCT_ID);
    result->versionId = read16bitRegister(REG_VERSION_ID);
}

/***********************************************************/
/***********************************************************/
/************************* ACTIONS *************************/

/** Sets the report type to CRQ_ABSOLUTE Mode. */
void API_C2_setCRQ_AbsoluteMode(){
    uint8_t feedConfig1 = API_C2_readRegister(0xC2C4);  // read
    feedConfig1 |= 0x02;                                // modify
    API_C2_writeRegister(0xC2C4, feedConfig1);          // write
}

/** Sets the Report type to Mouse and Keyboard reporting 
    This is compliant with HID protocol.
    Gen4 Devices are in Relative mode by default.
    Wait 50ms for it to take effect. */
void API_C2_setRelativeMode(){
    uint8_t feedConfig1 = API_C2_readRegister(0xC2C4);  // read
    feedConfig1 &= ~0x02;                               // modify
    API_C2_writeRegister(0xC2C4, feedConfig1);          // write
}

void API_C2_enableAdvancedAbs(){
    uint8_t feedConfig4 = API_C2_readRegister(0xC2DA);  // read
    feedConfig4 |= 0x01;                                // modify
    API_C2_writeRegister(0xC2DA, feedConfig4);          // write
}

void API_C2_disableAdvancedAbs(){
    uint8_t feedConfig4 = API_C2_readRegister(0xC2DA);  // read
    feedConfig4 &= ~0x01;                               // modify
    API_C2_writeRegister(0xC2DA, feedConfig4);          // write
}

bool API_C2_logicalScalingEnabled() {
  if((API_C2_readRegister(0xC2DA) & 0x08) == 0) {
    return false;
  } else {
    return true;
  }
}

void API_C2_enableLogicalScaling(){
    uint8_t feedConfig4 = API_C2_readRegister(0xC2DA);  // read
    feedConfig4 |= 0x08;                                // modify
    API_C2_writeRegister(0xC2DA, feedConfig4);          // write
}

uint16_t API_C2_GetLogicalScalingXMin() {
    uint16_t scaleLogXMin = API_C2_readRegister(0xC3B6);
    scaleLogXMin |= API_C2_readRegister(0xC3B7) << 8;
    return scaleLogXMin;  
}

void API_C2_SetLogicalScalingXMin(uint16_t value) {
    API_C2_writeRegister(0xC3B6, value & 255);
    API_C2_writeRegister(0xC3B7, value >> 8);
}

uint16_t API_C2_GetLogicalScalingXMax() {
    uint16_t scaleLogXMax = API_C2_readRegister(0xC3B8);
    scaleLogXMax |= API_C2_readRegister(0xC3B9) << 8;
    return scaleLogXMax;  
}

void API_C2_SetLogicalScalingXMax(uint16_t value) {
    API_C2_writeRegister(0xC3B8, value & 255);
    API_C2_writeRegister(0xC3B9, value >> 8);
}

uint16_t API_C2_GetLogicalScalingYMin() {
    uint16_t scaleLogYMin = API_C2_readRegister(0xC3BE);
    scaleLogYMin |= API_C2_readRegister(0xC3BF) << 8;
    return scaleLogYMin;  
}

void API_C2_SetLogicalScalingYMin(uint16_t value) {
    API_C2_writeRegister(0xC3BE, value & 255);
    API_C2_writeRegister(0xC3BF, value >> 8);
}

uint16_t API_C2_GetLogicalScalingYMax() {
    uint16_t scaleLogYMax = API_C2_readRegister(0xC3C0);
    scaleLogYMax |= API_C2_readRegister(0xC3C1) << 8;
    return scaleLogYMax;  
}

void API_C2_SetLogicalScalingYMax(uint16_t value) {
    API_C2_writeRegister(0xC3C0, value & 255);
    API_C2_writeRegister(0xC3C1, value >> 8);
}

uint16_t API_C2_GetActualXMin() {
    uint16_t scaleActualXMin = API_C2_readRegister(0xC3B2);
    scaleActualXMin |= API_C2_readRegister(0xC3B3) << 8;
    return scaleActualXMin;  
}

uint16_t API_C2_GetActualXMax() {
    uint16_t scaleActualXMax = API_C2_readRegister(0xC3B4);
    scaleActualXMax |= API_C2_readRegister(0xC3B5) << 8;
    return scaleActualXMax;  
}

uint16_t API_C2_GetActualYMin() {
    uint16_t scaleActualYMin = API_C2_readRegister(0xC3BA);
    scaleActualYMin |= API_C2_readRegister(0xC3BB) << 8;
    return scaleActualYMin;  
}

uint16_t API_C2_GetActualYMax() {
    uint16_t scaleActualYMax = API_C2_readRegister(0xC3BC);
    scaleActualYMax |= API_C2_readRegister(0xC3BD) << 8;
    return scaleActualYMax;  
}

uint16_t API_C2_GetFingerTouchThreshold() {
    uint16_t touchThreshold = API_C2_readRegister(0xC2FC);
    touchThreshold |= API_C2_readRegister(0xC2FD) << 8;
    return touchThreshold;  
}

void API_C2_SetFingerTouchThreshold(uint16_t value) {
    API_C2_writeRegister(0xC2FC, value & 255);
    API_C2_writeRegister(0xC2FD, value >> 8);
}

uint16_t API_C2_GetFingerLiftOffThreshold() {
    uint16_t liftThreshold = API_C2_readRegister(0xC394);
    liftThreshold |= API_C2_readRegister(0xC395) << 8;
    return liftThreshold;    
}

void API_C2_SetFingerLiftOffThreshold(uint16_t value) {
    API_C2_writeRegister(0xC394, value & 255);
    API_C2_writeRegister(0xC395, value >> 8);
}


void API_C2_disableLogicalScaling(){
    uint8_t feedConfig4 = API_C2_readRegister(0xC2DA);  // read
    feedConfig4 &= ~0x08;                               // modify
    API_C2_writeRegister(0xC2DA, feedConfig4);          // write
}

void API_C2_enableSingleFingerMode(){
    uint8_t feedConfig4 = API_C2_readRegister(0xC2DA);  // read
    feedConfig4 |= 0x80;                                // modify
    API_C2_writeRegister(0xC2DA, feedConfig4);          // write
}

void API_C2_disableSingleFingerMode(){
    uint8_t feedConfig4 = API_C2_readRegister(0xC2DA);  // read
    feedConfig4 &= ~0x80;                               // modify
    API_C2_writeRegister(0xC2DA, feedConfig4);          // write
}

void API_C2_enableAbsXYZZ(){
    uint8_t feedConfig1 = API_C2_readRegister(0xC2C4);  // read
    feedConfig1 |= 0x04;                                // modify
    API_C2_writeRegister(0xC2C4, feedConfig1);          // write
}

void API_C2_disableAbsXYZZ(){
    uint8_t feedConfig1 = API_C2_readRegister(0xC2C4);  // read
    feedConfig1 &= ~0x04;                               // modify
    API_C2_writeRegister(0xC2C4, feedConfig1);          // write
}

uint8_t API_C2_getADCGain(){
    return API_C2_readRegister(0xC2E8);
}

void API_C2_setADCGain(uint8_t value){
    if(value < 147)
    {
      value = 147;
    }
    if(value > 150)
    {
      value = 150;
    }
    API_C2_writeRegister(0xC2E8, value);          // write
}

void API_C2_invertY(){
    uint8_t feedConfig2 = API_C2_readRegister(0xC2C5);  // read
    feedConfig2 |= 0x02;                                // modify
    API_C2_writeRegister(0xC2C5, feedConfig2);          // write
}

void API_C2_revertY(){
    uint8_t feedConfig2 = API_C2_readRegister(0xC2C5);  // read
    feedConfig2 &= ~0x02;                               // modify
    API_C2_writeRegister(0xC2C5, feedConfig2);          // write
}

void API_C2_enableJitter(){
    uint8_t filterControl = API_C2_readRegister(0xC2CB);  // read
    filterControl |= 0x01;                                // modify
    API_C2_writeRegister(0xC2CB, filterControl);          // write
}

void API_C2_disableJitter(){
    uint8_t filterControl = API_C2_readRegister(0xC2CB);  // read
    filterControl &= ~0x01;                               // modify
    API_C2_writeRegister(0xC2CB, filterControl);          // write
}

void API_C2_enablePTPJitter(){
    uint8_t filterControl = API_C2_readRegister(0xC2D0);  // read
    filterControl |= 0x01;                                // modify
    API_C2_writeRegister(0xC2D0, filterControl);          // write
}

void API_C2_disablePTPJitter(){
    uint8_t filterControl = API_C2_readRegister(0xC2D0);  // read
    filterControl &= ~0x01;                               // modify
    API_C2_writeRegister(0xC2D0, filterControl);          // write
}

void API_C2_enableLeastChange(){
    uint8_t filterControl = API_C2_readRegister(0xC2CB);  // read
    filterControl |= 0x02;                                // modify
    API_C2_writeRegister(0xC2CB, filterControl);          // write
}

void API_C2_disableLeastChange(){
    uint8_t filterControl = API_C2_readRegister(0xC2CB);  // read
    filterControl &= ~0x02;                               // modify
    API_C2_writeRegister(0xC2CB, filterControl);          // write
}

void API_C2_enableSmoothing(){
    uint8_t filterControl = API_C2_readRegister(0xC2CB);  // read
    filterControl |= 0x04;                                // modify
    API_C2_writeRegister(0xC2CB, filterControl);          // write
}

void API_C2_disableSmoothing(){
    uint8_t filterControl = API_C2_readRegister(0xC2CB);  // read
    filterControl &= ~0x04;                               // modify
    API_C2_writeRegister(0xC2CB, filterControl);          // write
}

void API_C2_enableHoverFilter(){
    uint8_t filterControl = API_C2_readRegister(0xC2CB);  // read
    filterControl |= 0x20;                                // modify
    API_C2_writeRegister(0xC2CB, filterControl);          // write
}

void API_C2_disableHoverFilter(){
    uint8_t filterControl = API_C2_readRegister(0xC2CB);  // read
    filterControl &= ~0x20;                               // modify
    API_C2_writeRegister(0xC2CB, filterControl);          // write
}

void API_C2_enablePTPLeastChange(){
    uint8_t filterControl = API_C2_readRegister(0xC2D0);  // read
    filterControl |= 0x02;                                // modify
    API_C2_writeRegister(0xC2D0, filterControl);          // write
}

void API_C2_disablePTPLeastChange(){
    uint8_t filterControl = API_C2_readRegister(0xC2D0);  // read
    filterControl &= ~0x02;                               // modify
    API_C2_writeRegister(0xC2D0, filterControl);          // write
}

void API_C2_enablePTPSmoothing(){
    uint8_t filterControl = API_C2_readRegister(0xC2D0);  // read
    filterControl |= 0x04;                                // modify
    API_C2_writeRegister(0xC2D0, filterControl);          // write
}

void API_C2_disablePTPSmoothing(){
    uint8_t filterControl = API_C2_readRegister(0xC2D0);  // read
    filterControl &= ~0x04;                               // modify
    API_C2_writeRegister(0xC2D0, filterControl);          // write
}

/** Stores all registers and comp into flash memory. */
void API_C2_persistToFlash()
{
    uint8_t persistentDataControl = API_C2_readRegister(0xC2DF);   // read
    persistentDataControl |= 0x01;                                 // modify
    API_C2_writeRegister(0xC2DF, persistentDataControl);           // write
}    

/** Enables compensation if it was disabled. 
    The Comp is enabled by default
    Wait 50ms for it to take effect. */
void API_C2_enableComp()
{
    uint8_t compConfig = API_C2_readRegister(0xC2C7);              // read
    compConfig |= 0x3E;                                            // modify
    API_C2_writeRegister(0xC2C7,compConfig);                       // write
}

/** Disables compensation. Compensation can still be forced with 
	API_C2_forceComp. Wait 50ms for it to take effect. */
void API_C2_disableComp()
{
    uint8_t compConfig = API_C2_readRegister(0xC2C7);              // read
    compConfig &= ~0x3E;                                           // modify
    API_C2_writeRegister(0xC2C7,compConfig);                       // write
}

/** Forces a reset of the compensation. 
    Wait 50ms for it to take effect. */
void API_C2_forceComp()
{
  uint8_t feedConfig1 = API_C2_readRegister(0xC2C4);    // read
  feedConfig1 |= 0x80;                                  // modify
  API_C2_writeRegister(0xC2C4, feedConfig1);            // write
}

/** Takes a clean compensation image in the factory
    returns true if successful, false if watchdog timeout
    This takes about 200 ms to run. */
bool API_C2_factoryCalibrate()
{
    uint8_t previousValue = API_C2_readRegister(0xC2C4);           // read
    uint8_t modifiedValue = previousValue | 0x80;                  // modify
    API_C2_writeRegister(0xC2C4, modifiedValue);                   // write
    
    API_Hardware_delay(20);
    
    uint32_t timeout = 0; // times out at 5e6 count.  ~150ms- ish
    while(API_C2_readRegister(0xC2C4) == modifiedValue 
			&& (timeout < 5000000) ) timeout++;
    if(timeout == 500000)
    {
        return false;
    }
    API_C2_writeRegister(0xC2DF, 0x03);                           // write
    
    timeout = 0; // times out at 5e6 count.  ~150ms- ish
    while(API_C2_readRegister(0xC2D4) != 0x00 && (timeout < 5000000)) timeout++;
    if(timeout == 500000)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/** Turns off the feed. The touchpad continues to calculate the touch data, 
	but will stop sending DR signals
    ISSUE: Becomes renabled if awoken from sleep. */
void API_C2_disableFeed()
{
    uint8_t feedConfig1 = API_C2_readRegister(0xC2C4);       // read
    feedConfig1 &= ~0x01;                                    // modify
    API_C2_writeRegister(0xC2C4, feedConfig1);               // write
}

/** Turns on the feed wait 50ms for it to take effect. */
void API_C2_enableFeed()
{
    uint8_t feedConfig1 = API_C2_readRegister(0xC2C4);       // read
    feedConfig1 |= 0x01;                                     // modify
    API_C2_writeRegister(0xC2C4, feedConfig1);               // write
}

/** Turns off tracking.
    The touchpad stops calculating touch data. 
    It then goes to sleep if power control is enabled
    Wait 50ms for it to take effect. */
void API_C2_disableTracking()
{
    uint8_t sysConfig1 = API_C2_readRegister(0xC2C2);        // read
    sysConfig1 &= ~0x02;                                     // modify
    API_C2_writeRegister(0xC2C2, sysConfig1);                // write
}

/** Turns on tracking. */
void API_C2_enableTracking()
{
    uint8_t sysConfig1 = API_C2_readRegister(0xC2C2);        // read
    sysConfig1 |= 0x02;                                      // modify
    API_C2_writeRegister(0xC2C2, sysConfig1);                // write
}

/**********************************************************/
/**********************************************************/
/*********** TOOLS FOR DETERMINIG INPUT EVENTS ************/

/** determines if the finger_num finger corresponds to a valid finger in the report 
    The finger may still have x,y coordinates when the finger is invalid. These 
    coordinates should be ignored */
bool API_C2_isFingerValid(report_t* report, uint8_t finger_num)
{
    if(finger_num > 4 || finger_num < 0 )
    {
        return false;
    }
    
    if(report == NULL || report->reportID != CRQ_ABSOLUTE_REPORT_ID)
    {
        return false;
    }
    
    uint8_t palm = report->abs.fingers[finger_num].palm;
    return ((palm & CRQ_ABSOLUTE_CONFIDENCE_MASK) && !(palm &CRQ_ABSOLUTE_PALM_REJECT_MASK));
}

/** determines if the finger_num finger is in contact with the module */
bool API_C2_isFingerContacted(report_t* report, uint8_t finger_num)
{
    if(finger_num > 4 || finger_num < 0 )
    {
        return false;
    }
    
    if(report == NULL || report->reportID != CRQ_ABSOLUTE_REPORT_ID)
    {
        return false;
    }
    return report->abs.contactFlags &  (0x1 << finger_num); 
}


/** determines if a button is pressed according to its mask. 
    returns false if it's a keyboard report with no button information.*/
bool API_C2_isButtonPressed(report_t* report, uint8_t buttonMask)
{
    uint8_t buttons = 0;
    if(report == NULL)
        return false;
    switch(report->reportID)
    {
        case MOUSE_REPORT_ID:
            buttons = report->mouse.buttons;
            break;
        case CRQ_ABSOLUTE_REPORT_ID:
            buttons = report->abs.buttons;
            break;
        default:
            buttons = 0;
    }
    return buttons & buttonMask;
}

/**********************************************************/
/**********************************************************/
/**********************************************************/

/** Included for testing purposes. API_C2_getReport calls this */
void API_C2_decodeReport(uint8_t* packet, report_t* result)
{
    result->reportID = packet[2];
    //determine which type of report it is and decode it
    switch(result->reportID) //packet id
    {
        case MOUSE_REPORT_ID:
            decodeMouseReport(packet, result);
            break;
        case KEYBOARD_REPORT_ID:
            decodeKeyboardReport(packet, result);
            break;
        case CRQ_ABSOLUTE_REPORT_ID:
            decodeCirqueAbsoluteReport(packet, result);
            break;
        default:
            clearReport(result); //return an empty report
            break;
        
    }
}
