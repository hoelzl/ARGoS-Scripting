%module argos
%{
#include <gsl/gsl_rng.h>
#include <argos2/common/utility/datatypes/datatypes.h>
#include <argos2/common/utility/datatypes/color.h>

#include <argos2/common/utility/math/general.h>
#include <argos2/common/utility/math/range.h>
#include <argos2/common/utility/math/angles.h>
#include <argos2/common/utility/math/vector2.h>
#include <argos2/common/utility/math/vector3.h>

#include <argos2/common/utility/argos_random.h>

#include <argos2/common/utility/tinyxml-cpp/ticpp.h>
#include <argos2/common/utility/configuration/argos_configuration.h>

#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>

#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_actuator.h>
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_light_sensor.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_motor_ground_sensor.h>
%}

%rename("%(undercase)s", %$isfunction) "";

%include <string>
%include <vector>
%include <map>

%ignore operator<<;
%ignore operator>>;
// For some reason the definitions for UInt8 and SInt8 are not picked
// up by SWIG...
%{
  typedef signed char SInt8; 
  typedef unsigned char UInt8;
%}
%include <argos2/common/utility/datatypes/datatypes.h>

%rename(ByteArray) CByteArray;
%include <argos2/common/utility/datatypes/byte_array.h>

class std::exception 
{
 public:
  exception() throw() { }
  virtual ~exception() throw();
  virtual const char* what() const throw();
};

// %include <argos2/common/utility/tinyxml-cpp/ticpp.h>

%rename(ArgosException) CARGoSException;
%include <argos2/common/utility/configuration/argos_exception.h>

%rename(ConfigurationNode) TConfigurationNode;
%rename(ConfigurationNodeIterator) TConfigurationNodeIterator;
%ignore SetNodeAttribute(TConfigurationNode& t_node,
			 const std::string& str_attribute,
			 const SInt8 n_value);
%include <argos2/common/utility/configuration/argos_configuration.h>

%rename(BaseConfigurableResource) CBaseConfigurableResource;
%include <argos2/common/utility/configuration/base_configurable_resource.h>

%rename(Memento) CMemento;
%include <argos2/common/utility/configuration/memento.h>

%include <argos2/common/utility/string_utilities.h>

%rename(Color) CColor;
%ignore operator UInt32;
%include <argos2/common/utility/datatypes/color.h>

%ignore Abs(Real);
%include <argos2/common/utility/math/general.h>

%rename(Range) CRange;
%include <argos2/common/utility/math/range.h>

%warnfilter(503) argos::ToRadians;
%warnfilter(503) argos::ToDegrees;
%rename(Radians) CRadians;
%rename(Degrees) CDegrees;
%include <argos2/common/utility/math/angles.h>

%rename(Vector2) CVector2;
%include <argos2/common/utility/math/vector2.h>

%rename(Vector3) CVector3;
%include <argos2/common/utility/math/vector3.h>

typedef struct  {
  // Originally const char *name, but this leads to a warning about
  // leaking memory in SWIG.
  const char * const name;
  unsigned long int max;
  unsigned long int min;
  size_t size;
  void (*set) (void *state, unsigned long int seed);
  unsigned long int (*get) (void *state);
  double (*get_double) (void *state);
} gsl_rng_type;

typedef struct  {
  const gsl_rng_type * type;
  void *state;
} gsl_rng;


namespace argos {
  class CRNG : public argos::CMemento {
  public:
    CRNG(argos::UInt32 un_seed,
	 const std::string& str_type = "mt19937");
    CRNG(argos::CByteArray& c_buffer);
    CRNG(const CRNG& c_rng);
    virtual ~CRNG();
    inline argos::UInt32 GetSeed() const throw() {
      return m_unSeed;
    }
    inline void SetSeed(argos::UInt32 un_seed) throw() {
      m_unSeed = un_seed;
    }
    inline std::string GetType() const throw() {
      return m_strType;
    }
    inline void SetType(const std::string& str_type) {
      m_strType = str_type;
    }
    virtual void SaveState(argos::CByteArray& c_buffer);
    virtual void LoadState(argos::CByteArray& c_buffer);
    void Reset();
    bool Bernoulli(argos::Real f_true = 0.5);
    argos::CRadians Uniform(const argos::CRange<argos::CRadians>& c_range);
    argos::Real Uniform(const argos::CRange<argos::Real>& c_range);
    argos::SInt32 Uniform(const argos::CRange<argos::SInt32>& c_range);
    argos::UInt32 Uniform(const argos::CRange<argos::UInt32>& c_range);
    argos::Real Exponential(argos::Real f_mean);
    argos::Real Gaussian(argos::Real f_std_dev, argos::Real f_mean = 0.0f);
    
  private:
    void CreateRNG();
    void DisposeRNG();
    
  private:
    argos::UInt32 m_unSeed;
    std::string m_strType;
    gsl_rng* m_ptCRNG;
    argos::CRange<argos::UInt32>* m_pcIntegerCRNGRange;
    
  };
}
%nestedworkaround argos::CARGoSRandom::CRNG;
%{
  namespace argos {
    typedef argos::CARGoSRandom::CRNG CRNG;
  }
%}

namespace argos {
  class CCategory : public argos::CMemento { 
  public:
    CCategory(const std::string& str_id,
	      argos::UInt32 un_seed);
    CCategory(argos::CByteArray& c_buffer);
    virtual ~CCategory() {}
    
    inline const std::string& GetId() const throw() {
      return m_strId;
    }
    void SetId(const std::string& str_id) {
      m_strId = str_id;
    }
    inline argos::UInt32 GetSeed() const {
      return m_unSeed;
    }
    void SetSeed(argos::UInt32 un_seed);
    
    virtual void SaveState(argos::CByteArray& c_buffer);
    virtual void LoadState(argos::CByteArray& c_buffer);
    CRNG* CreateRNG(const std::string& str_type = "mt19937");
    void ResetRNGs();
    void ReseedRNGs();
    
  private:
    std::string m_strId;
    std::vector<CRNG*> m_vecRNGList;
    argos::UInt32 m_unSeed;
    CRNG m_cSeeder;
    argos::CRange<argos::UInt32> m_cSeedRange;
  };
}

%nestedworkaround argos::CARGoSRandom::CCategory;
%{
  namespace argos {
    typedef argos::CARGoSRandom::CCategory CCategory;
  }
%}

%rename(Random) CARGoSRandom;
%nodefaultctor CARGoSRandom;
%ignore ~CARGoSRandom;
%include <argos2/common/utility/argos_random.h>

%rename(Actuator) CCI_Actuator;
%include <argos2/common/control_interface/ci_actuator.h>

%rename(Sensor) CCI_Sensor;
%include <argos2/common/control_interface/ci_sensor.h>

%rename(Robot) CCI_Robot;
%include <argos2/common/control_interface/ci_robot.h>

%rename(Controller) CCI_Controller;
%include <argos2/common/control_interface/ci_controller.h>

%rename(FootBotWheelsActuator) CCI_FootBotWheelsActuator;
%include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>

%rename(FootBotLedsActuator) CCI_FootBotLedsActuator;
%include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>

namespace argos {
  struct ProximitySensorReading {
    argos::Real Value;
    argos::CRadians Angle;  
    
    ProximitySensorReading() :
      Value(0.0f) {}
  
    ProximitySensorReading(argos::Real f_value,
			   const argos::CRadians& c_angle) :
      Value(f_value),
      Angle(c_angle) {}
  };
}
%nestedworkaround argos::CCI_FootBotProximitySensor::SReading;
%rename(FootBotProximitySensor) CCI_FootBotProximitySensor;

%include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>
%{
  namespace argos {
    typedef argos::CCI_FootBotProximitySensor::SReading ProximitySensorReading;
  }
%}


%rename(RangeAndBearingSensor) CCI_RangeAndBearingSensor;
%rename(RangeAndBearingReceivedPacket) TRangeAndBearingReceivedPacket;
%rename(RangeAndBearingData) TRangeAndBearingData;
%rename(RawValuesArray) TRawValues;
%{
  namespace argos {
    typedef argos::UInt8 TRangeAndBearingData[10];
    typedef argos::UInt16 TRawValues[12];
  }
%}
%include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>

%rename(RangeAndBearingActuator) CCI_RangeAndBearingActuator;
%include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_actuator.h>


namespace argos {
  struct LightSensorReading {
    argos::Real Value;
    argos::CRadians Angle;

    LightSensorReading() :
      Value(0.0f) {}
  
    LightSensorReading(argos::Real f_value,
		       const argos::CRadians& c_angle) :
      Value(f_value),
      Angle(c_angle) {}
  };
}

%nestedworkaround argos::CCI_FootBotLightSensor::SReading;
%rename(FootBotLightSensor) CCI_FootBotLightSensor;
%{
  namespace argos {
    typedef argos::CCI_FootBotLightSensor::SReading LightSensorReading;
  }
%}

%include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_light_sensor.h>

namespace argos {
  struct MotorGroundSensorReading {
    argos::Real Value;
    argos::CVector2 Offset;
  
    MotorGroundSensorReading() :
      Value(0.0f) {}
   
    MotorGroundSensorReading(argos::Real f_value,
			     const argos::CVector2& c_offset) :
      Value(f_value),
      Offset(c_offset) {}
  };
 }
%nestedworkaround argos::CCI_FootBotMotorGroundSensor::SReading;
%rename(FootBotMotorGroundSensor) CCI_FootBotMotorGroundSensor;
%{
  namespace argos {
    typedef argos::CCI_FootBotMotorGroundSensor::SReading MotorGroundSensorReading;
  }
%}

%include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_motor_ground_sensor.h>
