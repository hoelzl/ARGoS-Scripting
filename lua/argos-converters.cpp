#include "argos-converters.hpp"

std::vector<ProximitySensorReading>
to_proximity_sensor_reading_vector
(std::vector<argos::CCI_FootBotProximitySensor::SReading> &readings) {
  std::cerr << "In function to_proximity_sensor_reading_vector:" << std::endl;
  std::vector<ProximitySensorReading> result;
    // = new std::vector<ProximitySensorReading>();
  std::vector<argos::CCI_FootBotProximitySensor::SReading>::iterator it;
  for (it = readings.begin(); it < readings.end(); ++it) {
    argos::CCI_FootBotProximitySensor::SReading old_reading = *it;
    ProximitySensorReading new_reading(old_reading.Value, old_reading.Angle);
    std::cerr << "Pushining new reading...";
    result.push_back(new_reading);
    std::cerr << "done." << std::endl;
  }
  std::cerr << "Done!" << std::endl;
  return result;
}
