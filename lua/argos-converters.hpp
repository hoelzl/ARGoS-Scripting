#ifndef ARGOS_CONVERTERS
#define ARGOS_CONVERTERS

#include <vector>
#include <iostream>

#include <argos2/common/utility/datatypes/datatypes.h>
#include <argos2/common/utility/datatypes/color.h>

#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>

#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_actuator.h>
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_light_sensor.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_motor_ground_sensor.h>

#include "argos-fixups.hpp"

extern
std::vector<ProximitySensorReading>
to_proximity_sensor_reading_vector
(std::vector<argos::CCI_FootBotProximitySensor::SReading> &readings);

#endif
