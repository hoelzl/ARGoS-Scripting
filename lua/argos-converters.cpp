#include "argos-converters.hpp"

std::vector<ProximitySensorReading>
to_proximity_sensor_reading_vector
(std::vector<argos::CCI_FootBotProximitySensor::SReading> &readings) {
  std::vector<ProximitySensorReading> result;
    // = new std::vector<ProximitySensorReading>();
  std::vector<argos::CCI_FootBotProximitySensor::SReading>::iterator it;
  for (it = readings.begin(); it < readings.end(); ++it) {
    argos::CCI_FootBotProximitySensor::SReading old_reading = *it;
    ProximitySensorReading new_reading(old_reading.Value, old_reading.Angle);
    result.push_back(new_reading);
  }
  return result;
}
