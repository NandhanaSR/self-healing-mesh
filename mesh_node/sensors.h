// sensors.h - the single place that reads analog inputs.
// Swapping potentiometers for an LDR (or anything else) = edit sensor_read() only.
#pragma once
#include <Arduino.h>
void    sensors_init();
int16_t sensor_read(uint8_t which);   // which = 0 or 1, returns 0..4095
