#ifndef ARGOS_FIXUPS
#define ARGOS_FIXUPS

#include <vector>

#include <argos2/common/utility/datatypes/datatypes.h>
#include <argos2/common/utility/datatypes/color.h>
#include <argos2/common/utility/math/general.h>
#include <argos2/common/utility/math/range.h>
#include <argos2/common/utility/math/angles.h>
#include <argos2/common/utility/math/vector2.h>
#include <argos2/common/utility/math/vector3.h>

class ProximitySensorReading {
public:
  argos::Real value;
  argos::CRadians angle;

  ProximitySensorReading () :
    value(0.0f) {}

  ProximitySensorReading (argos::Real a_value, argos::CRadians &a_angle) :
    value(a_value), angle(a_angle) {
  }
};

#endif
