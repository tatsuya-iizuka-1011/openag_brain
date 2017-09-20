/**
 *  \file openag_ds18b20.h
 *  \brief Sensor module for temperature.
 */

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include "WProgram.h"
#endif


#include <Wire.h>
#include <openag_module.h>
//#include <TSL2561.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>


/**
 * \brief Sensor module for temperature
 */
class Tsl2561 : public Module {
  public:
    Tsl2561(int pin);
    uint8_t begin();
    uint8_t update();
    int get_lux();
    void read_lux();
    void configureSensor();
    sensors_event_t event;

  private:
    int _address;
    bool _send_temperature;
    int _lux;
    Adafruit_TSL2561_Unified tsl;
    uint32_t _time_of_last_query;
    bool _waiting_for_response;
    const static uint32_t _min_update_interval = 2000;

    // Status codes
    static const uint8_t CODE_COULDNT_FIND_ADDRESS = 1;
    static const uint8_t CODE_NO_RESPONSE = 2;
};

