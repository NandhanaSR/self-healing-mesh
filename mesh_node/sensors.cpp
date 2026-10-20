#include "sensors.h"
#include "config.h"
void sensors_init() { pinMode(SENSOR1_PIN, INPUT); pinMode(SENSOR2_PIN, INPUT); }
int16_t sensor_read(uint8_t which) { return (int16_t)analogRead(which == 0 ? SENSOR1_PIN : SENSOR2_PIN); }
