#include "controller.h"

#ifdef NEOPIXELS
int colorSaturation = 32;
int pixelCount = 37;

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation/3);
RgbColor black(0);

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> mStrip(pixelCount, NEOPIXELS_DATA_PIN);

void Controller::initNeoPixelStrip()
{
    mStrip.Begin();
    delay(1);
    mStrip.Show();

    for (size_t i = VALVE1; i <= VALVE32; i++) {
        if (mComponents[static_cast<ComponentID>(i)]->getValue() == OPEN)
            mStrip.SetPixelColor(i, green);
        else
            mStrip.SetPixelColor(i, red);
    }

    delay(1);
    mStrip.Show();
}

void Controller::setNeoPixel(int index, const RgbColor& color)
{
    mStrip.SetPixelColor(index, color);
    delay(1);
    mStrip.Show();
}

#endif

#ifdef BLUETOOTH_SERIAL
#include <BluetoothSerial.h>
extern BluetoothSerial SerialBT;
#endif

Controller::Controller()
    : mXIORefreshRequested1(false), mXIORefreshRequested2(false), mXIORefreshRequested3(false)
{
}

void Controller::init()
{
    Wire.begin(); //join the I2C bus as master
    mXioBoard1.begin(LOW, LOW, LOW, XIO_RESET_PIN , XIO_OE_PIN); // IO_RESET_PIN , XIO_OE_PIN is set in constants.h, 
    //Here 1 PCA9698 as on XIO instance/object with address A0=0, A1=0, A2=0
    mXioBoard2.begin(HIGH, LOW, LOW, XIO_RESET_PIN , XIO_OE_PIN);
    mXioBoard3.begin(LOW, HIGH, LOW, XIO_RESET_PIN , XIO_OE_PIN);

    mComponents[VALVE1] = new Valve(VALVE1_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE2] = new Valve(VALVE2_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE3] = new Valve(VALVE3_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE4] = new Valve(VALVE4_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE5] = new Valve(VALVE5_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE6] = new Valve(VALVE6_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE7] = new Valve(VALVE7_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE8] = new Valve(VALVE8_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE9] = new Valve(VALVE9_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE10] = new Valve(VALVE10_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE11] = new Valve(VALVE11_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE12] = new Valve(VALVE12_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE13] = new Valve(VALVE13_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE14] = new Valve(VALVE14_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE15] = new Valve(VALVE15_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE16] = new Valve(VALVE16_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE17] = new Valve(VALVE17_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE18] = new Valve(VALVE18_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE19] = new Valve(VALVE19_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE20] = new Valve(VALVE20_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE21] = new Valve(VALVE21_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE22] = new Valve(VALVE22_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE23] = new Valve(VALVE23_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE24] = new Valve(VALVE24_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE25] = new Valve(VALVE25_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE26] = new Valve(VALVE26_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE27] = new Valve(VALVE27_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE28] = new Valve(VALVE28_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE29] = new Valve(VALVE29_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE30] = new Valve(VALVE30_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE31] = new Valve(VALVE31_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE32] = new Valve(VALVE32_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE33] = new Valve(VALVE33_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE34] = new Valve(VALVE34_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE35] = new Valve(VALVE35_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE36] = new Valve(VALVE36_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE37] = new Valve(VALVE37_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE38] = new Valve(VALVE38_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE39] = new Valve(VALVE39_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE40] = new Valve(VALVE40_PIN,mXioBoard1, mXIORefreshRequested1, false);
    mComponents[VALVE41] = new Valve(VALVE41_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE42] = new Valve(VALVE42_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE43] = new Valve(VALVE43_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE44] = new Valve(VALVE44_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE45] = new Valve(VALVE45_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE46] = new Valve(VALVE46_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE47] = new Valve(VALVE47_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE48] = new Valve(VALVE48_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE49] = new Valve(VALVE49_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE50] = new Valve(VALVE50_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE51] = new Valve(VALVE51_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE52] = new Valve(VALVE52_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE53] = new Valve(VALVE53_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE54] = new Valve(VALVE54_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE55] = new Valve(VALVE55_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE56] = new Valve(VALVE56_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE57] = new Valve(VALVE57_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE58] = new Valve(VALVE58_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE59] = new Valve(VALVE59_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE60] = new Valve(VALVE60_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE61] = new Valve(VALVE61_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE62] = new Valve(VALVE62_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE63] = new Valve(VALVE63_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE64] = new Valve(VALVE64_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE65] = new Valve(VALVE65_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE66] = new Valve(VALVE66_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE67] = new Valve(VALVE67_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE68] = new Valve(VALVE68_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE69] = new Valve(VALVE69_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE70] = new Valve(VALVE70_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE71] = new Valve(VALVE71_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE72] = new Valve(VALVE72_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE73] = new Valve(VALVE73_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE74] = new Valve(VALVE74_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE75] = new Valve(VALVE75_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE76] = new Valve(VALVE76_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE77] = new Valve(VALVE77_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE78] = new Valve(VALVE78_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE79] = new Valve(VALVE79_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE80] = new Valve(VALVE80_PIN,mXioBoard2, mXIORefreshRequested2, false);
    mComponents[VALVE81] = new Valve(VALVE81_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE82] = new Valve(VALVE82_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE83] = new Valve(VALVE83_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE84] = new Valve(VALVE84_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE85] = new Valve(VALVE85_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE86] = new Valve(VALVE86_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE87] = new Valve(VALVE87_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE88] = new Valve(VALVE88_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE89] = new Valve(VALVE89_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE90] = new Valve(VALVE90_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE91] = new Valve(VALVE91_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE92] = new Valve(VALVE92_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE93] = new Valve(VALVE93_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE94] = new Valve(VALVE94_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE95] = new Valve(VALVE95_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE96] = new Valve(VALVE96_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE97] = new Valve(VALVE97_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE98] = new Valve(VALVE98_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE99] = new Valve(VALVE99_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE100] = new Valve(VALVE100_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE101] = new Valve(VALVE101_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE102] = new Valve(VALVE102_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE103] = new Valve(VALVE103_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE104] = new Valve(VALVE104_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE105] = new Valve(VALVE105_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE106] = new Valve(VALVE106_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE107] = new Valve(VALVE107_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE108] = new Valve(VALVE108_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE109] = new Valve(VALVE109_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE110] = new Valve(VALVE110_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE111] = new Valve(VALVE111_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE112] = new Valve(VALVE112_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE113] = new Valve(VALVE113_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE114] = new Valve(VALVE114_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE115] = new Valve(VALVE115_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE116] = new Valve(VALVE116_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE117] = new Valve(VALVE117_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE118] = new Valve(VALVE118_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE119] = new Valve(VALVE119_PIN,mXioBoard3, mXIORefreshRequested3, false);
    mComponents[VALVE120] = new Valve(VALVE120_PIN,mXioBoard3, mXIORefreshRequested3, false); 

 


    
   
    mComponents[PR1] = new PressureController(PR1_SETPOINT_PIN,
                                              PR1_MEASUREMENT_PIN,
                                              DAC_MAX_VALUE,
                                              ADC_MAX_VALUE);

    mComponents[FS] = new FlowSensorReader(FS_MEASUREMENT_PIN,                                              
                                              ADC_MAX_VALUE);                                          
    /*
    mComponents[PR1] = new PressureController(PR1_I2C_ADDRESS);
    mComponents[PR2] = new PressureController(PR2_I2C_ADDRESS);
    */
    /*
    mComponents[PR2] = new PressureController(PR2_SETPOINT_PIN,
                                              PR2_MEASUREMENT_PIN,
                                              DAC_MAX_VALUE,
                                              ADC_MAX_VALUE);
                                              */


    // Set all pin modes, and default values (LOW; see Valve::Valve())
    mXioBoard1.refreshPinModes();
    mXioBoard1.refreshIO();
    mXioBoard2.refreshPinModes();
    mXioBoard2.refreshIO();
    mXioBoard3.refreshPinModes();
    mXioBoard3.refreshIO();

    mTimer = millis();
    mPressureControlTimer = millis();
   

}

Controller::~Controller()
{}

/**
  * This function should be called by loop().
  */
void Controller::update() //looping in the main.cpp
{
    // Send current pressures every 0.5 second (or so)

    if ((millis() - mTimer) > 50000) {
        //sendComponentValue(PR1);
        

        mTimer = millis();
    }

    if ((millis() - mPressureControlTimer) > 50000) {
        //pressureControl();
        mPressureControlTimer = millis();
    }

#ifdef BLUETOOTH_SERIAL
    if (SerialBT.available())
#else
    if (Serial.available())
#endif
        handleSerialData(); // handle serial data from the PC 

    if (mXIORefreshRequested1) {  // decide wheather to refresh the XIO to change pin value and pin mode
        mXioBoard1.refreshIO();
        mXioBoard1.refreshPinModes();
        //Serial.print("Refreshed XioBoard1");
        mXIORefreshRequested1 = false;
    }

    if (mXIORefreshRequested2) {  // decide wheather to refresh the XIO to change pin value and pin mode
        mXioBoard2.refreshIO();
        mXioBoard2.refreshPinModes();
        //Serial.print("Refreshed XioBoard2");
        mXIORefreshRequested2 = false;
    }

    if (mXIORefreshRequested3) {  // decide wheather to refresh the XIO to change pin value and pin mode
        mXioBoard3.refreshIO();
        mXioBoard3.refreshPinModes();
        //Serial.print("Refreshed XioBoard3");
        mXIORefreshRequested3 = false;
    }    
}

/**
  * Replacement of pinMode() for components connected to the PCA9698
  */
void Controller::xioPinMode(int pin, int mode, XIO &PCA9698Chip)
{
    PCA9698Chip.xioPinModeCached(pin, mode); 
}

/**
  * Replacement of digitalWrite() for components connected to the PCA9698
  *
  * The value is not set immediately, but on the next run of update(). This is to
  * avoid sending too many requests via the i2c bus, which would be too slow.
  * (see XIO library reference for more details)
  */
void Controller::xioDigitalWrite(int pin, int value, XIO &PCA9698Chip, bool &XIORefreshRequested)
{
    PCA9698Chip.xioDigitalWriteCached(pin, value);
    XIORefreshRequested = true;
}

/**
  * @brief Read data from the serial buffer, and execute any instructions received.
  *
  * Data is exchanged between the microcontroller and the PC in two-byte packets.
  * The PC can either request the status of one or all components, or request a new
  * setpoint for a given component.
  * In the first case, the first byte will be STATUS_REQUEST, and the second byte
  * will be a component ID (see constants.h).
  * In the second case, the first byte will be a component ID, and the second byte, the
  * desired setpoint.
  */
void Controller::handleSerialData()
{

#ifdef BLUETOOTH_SERIAL
    while (SerialBT.available() >= 2) {
        int length = SerialBT.available();
        uint8_t firstByte, secondByte;
        firstByte = SerialBT.read();
        secondByte = SerialBT.read();
#else
    while (Serial.available() >= 2) {
        int length = Serial.available();
        uint8_t firstByte, secondByte,temp1,temp2;
        firstByte=Serial.read();
        secondByte=Serial.read();
        //firstByte = 50;
        //secondByte = 122;

#endif
        //Log.notice("Received %d bytes: %d ; %d \n", length, firstByte, secondByte);

        if (firstByte == STATUS_REQUEST) {
            //Log.notice("Status request for component %d \n", secondByte);
            if (secondByte == ALL_COMPONENTS)
                sendAllComponentValues();
            else
                sendComponentValue(static_cast<ComponentID>(secondByte));
        }

        else if (firstByte >= VALVE1 && firstByte < ALL_COMPONENTS) {
            if (mComponents.count(static_cast<ComponentID>(firstByte))) {  //returns the number of elements matching specific key 
                // Set requested value and communicate the new state
                mComponents[static_cast<ComponentID>(firstByte)]->setValue(secondByte);
                sendComponentValue(static_cast<ComponentID>(firstByte));

#ifdef NEOPIXELS
                setNeoPixel(firstByte - VALVE1, (secondByte == OPEN ? green : red));
#endif
            }

        }
    }
}

/**
  * @brief Write the value of the given component to serial.
  */
void Controller::sendComponentValue(ComponentID component)
{
    if (mComponents.count(component)) {
        uint8_t toSend[4] = {START_BYTE, (uint8_t)component, (uint8_t)mComponents[component]->getValue(), END_BYTE};
        #ifdef BLUETOOTH_SERIAL
            SerialBT.write(toSend, 4);
        #else
            Serial.write(toSend, 4);
        #endif
    }
}

/**
  * @brief Write the value of all components to serial.
  */
void Controller::sendAllComponentValues()
{
    for (auto const& i : mComponents) {
        sendComponentValue(i.first);
    }

    sendUptime();
}

/**
  * @brief Send an error code over serial.
  */
void Controller::sendErrorCode(Errors code)
{
    uint8_t toSend[4] = {START_BYTE, ERROR, code, END_BYTE};
    #ifdef BLUETOOTH_SERIAL
        SerialBT.write(toSend, 4);
    #else
        Serial.write(toSend, 4);
    #endif
}

/**
  * @brief Send the current uptime (in seconds) over serial.
  */
void Controller::sendUptime()
{
    unsigned long uptime = millis() / 1000;
    uint8_t value0, value1, value2, value3;

    value0 = (uptime & 0xFF000000) >> 24;
    value1 = (uptime & 0x00FF0000) >> 16;
    value2 = (uptime & 0x0000FF00) >> 8;
    value3 = (uptime & 0x000000FF);

    uint8_t toSend[7] = {START_BYTE, UPTIME, value0, value1, value2, value3, END_BYTE};
    #ifdef BLUETOOTH_SERIAL
        SerialBT.write(toSend, 7);
    #else
        Serial.write(toSend, 7);
    #endif
}

/*
void Controller::pressureControl()
{
    // Pressure control: if the current pressure is above a certain threshold,
    // we turn the pump off. If it is below a certain threshold (all relative to
    // the setpoint), we turn the pump on.
    // Pressure controllers 1 and 2 are connected to pump 1 (positive pressure);


    // For now: just one PR connected to 1 pump.

    PressureController * pc1 = static_cast<PressureController*>(mComponents[PR1]);


    // If the supply pressure is not indicated as too low, and it has been that way
    // for at least 2 seconds, we switch the pump off.

    if (pc1->setPointValue() == 0 && pc2->setPointValue() == 0) {
        pump->setValue(OFF);
    }

    else if (pump->getValue() == ON &&
        !pc1->isInputPressureTooLow() &&
        !pc2->isInputPressureTooLow() &&
        millis() - mPumpLastSwitchOnTime > 5000)
    {
            pump->setValue(OFF);
            sendComponentValue(PUMP1);
    }
    else if (pump->getValue() == OFF &&
             (pc1->isInputPressureTooLow() || pc2-> isInputPressureTooLow()))
    {
        pump->setValue(ON);
        mPumpLastSwitchOnTime = millis();
        sendComponentValue(PUMP1);
    }
}
*/
