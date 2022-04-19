#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Map>
#include <Wire.h>
#include <ArduinoLog.h>

#include "constants.h"
#include "things.h"

#ifdef NEOPIXELS
    #include <NeoPixelBus.h>
#endif


/**
  * The controller class handles all high-level functionality as well as
  * serial communication.
  * Its constructor can be seen as a replacement for the Arduino setup() function,
  * and its update() function as a replacement for the Arduino loop() function.
  *
  * For simplicity and ease of maintenance, the various components (valves, pumps and
  * pressure regulators) are represented by different classes that all inherit from
  * the Thing class. This way, the Controller simply has a list of pointers to Things,
  * and can call setValue and getValue to set and get the current state of each component.
  * This is implemented as a Map, to easily associate a ComponentID (which is communicated
  * via the serial link) to each instance.
  */
class Controller
{

public:
    Controller ();
    void init();
    virtual ~Controller ();

    void update();

    void xioPinMode(int pin, int mode, XIO &PCA9698Chip); 
    void xioDigitalWrite(int pin, int value, XIO &PCA9698Chip, bool &XIORefreshRequested);

    void sendErrorCode(Errors code);


private:
    void handleSerialData();
    void sendComponentValue(ComponentID component);
    void sendAllComponentValues();
    void sendUptime();

    //void pressureControl();

    unsigned long mTimer;
    unsigned long mPressureControlTimer;
   
    XIO mXioBoard1; 
    XIO mXioBoard2;
    XIO mXioBoard3;
    bool mXIORefreshRequested1;
    bool mXIORefreshRequested2;
    bool mXIORefreshRequested3;


    std::map<ComponentID, Thing*> mComponents; 

#ifdef NEOPIXELS

public:
    void initNeoPixelStrip();
    void setNeoPixel(int index, const RgbColor& color);

#endif // def NEOPIXELS
};

#endif
