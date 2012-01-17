#include "argos-converters.hpp"

// Maybe it would be better to return a pointer to a heap-allocated
// vector, but I have to try whether this confuses SWIG into not
// wrapping the result... --tc
//
std::vector<ProximitySensorReading>
to_proximity_sensor_reading_vector
(std::vector<argos::CCI_FootBotProximitySensor::SReading> &readings) {
  std::vector<ProximitySensorReading> result;
  std::vector<argos::CCI_FootBotProximitySensor::SReading>::iterator it;
  for (it = readings.begin(); it < readings.end(); ++it) {
    argos::CCI_FootBotProximitySensor::SReading old_reading = *it;
    ProximitySensorReading new_reading(old_reading.Value, old_reading.Angle);
    result.push_back(new_reading);
  }
  return result;
}
