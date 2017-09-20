/**
 *  \file openag_ds18b20.cpp
 *  \brief Sensor module for temperature.
 */
#include "openag_tsl2561.h"

Tsl2561::Tsl2561(int pin) : tsl(TSL2561_ADDR_FLOAT,pin) {
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
  _waiting_for_response = true;
  _time_of_last_query = 0;
}

uint8_t Tsl2561::begin() {
  if(!tsl.begin())
  {
    // There was a problem detecting the TSL2561 ... check your connections
    // pi is set as 12345 as default
    status_code = ERROR;
  }
  //TODO modify following condition
  configureSensor();
  if (status_code == ERROR) {
    status_level = ERROR;
    status_code = CODE_COULDNT_FIND_ADDRESS;
    status_msg = "Unable to find address for sensor";
  }
  return status_level;
}
uint8_t Tsl2561::update() {
  if (millis() - _time_of_last_query > _min_update_interval) {
    //send_query();
    _waiting_for_response = true;
  }
  if (_waiting_for_response) {
    read_lux();
    _waiting_for_response = false;
    _time_of_last_query = millis();
  }
  return status_level;
}
void Tsl2561::read_lux() {
  tsl.getEvent(&event);
  if (event.light) {
    _lux = event.light;
  }
  else {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    //Serial.println("Sensor overload");
    status_level = ERROR;
  }
}

int Tsl2561::get_lux() {
  return _lux;
}
void Tsl2561::configureSensor() {
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
}
