%module argos
%{
#include "argos-fixups.hpp"

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

#include "argos-converters.hpp"
%}

%rename("%(undercase)s", %$isfunction) "";

%include "std_string.i"
%apply const std::string& {std::string* foo};

%include "std_vector.i"
%include "std_map.i"

%ignore operator<<;
%ignore operator>>;

// Tools to allow debugging the generated bindings from Lua.
%native (print_module_types) print_module_types;
%native (find_module_type) find_module_type;
%{
  int print_module_types(lua_State *L) {
    swig_module_info *module = SWIG_GetModule(L);
    if (!module) {
      luaL_error(L, "could not get module");
    }
    std::cout << "Module: " << module
	      << ", " << module->size
	      << std::endl;
    for (unsigned int i = 0; i < module->size; ++i) {
      std::cout << "Type: " << module->types[i]->name << std::endl;
    }
    return 0;
  }

  int find_module_type(lua_State *L) {
    swig_module_info *module = SWIG_GetModule(L);
    if (!module) {
      luaL_error(L, "could not get module");
      return 0;
    }
    const char *type_name = luaL_checkstring(L, 1);
    swig_type_info *type = SWIG_TypeQuery(type_name);
    if (!type) {
      lua_pushfstring(L, "no such type: %s", type_name);
    }
    else {
      lua_pushstring(L, type->name);
    }
    return 1;
  }
%}

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


%warnfilter (509) TiXmlNode::SetValue;
%warnfilter (509) TiXmlNode::FirstChild;
%warnfilter (509) TiXmlNode::LastChild;
%warnfilter (509) TiXmlNode::IterateChildren;
%warnfilter (509) TiXmlNode::PreviousSibling;
%warnfilter (509) TiXmlNode::NextSibling;
%warnfilter (509) TiXmlNode::NextSiblingElement;
%warnfilter (509) TiXmlNode::FirstChildElement;
%warnfilter (509) TiXmlAttribute::TiXmlAttribute;
%warnfilter (509) TiXmlAttribute::SetName;
%warnfilter (509) TiXmlAttribute::SetValue;
%warnfilter (509) TiXmlAttributeSet::Find;
%warnfilter (509) TiXmlElement::TiXmlElement;
%warnfilter (509) TiXmlElement::Attribute;
%warnfilter (509) TiXmlElement::QueryIntAttribute;
%warnfilter (509) TiXmlElement::QueryDoubleAttribute;
%warnfilter (509) TiXmlElement::SetAttribute;
%warnfilter (509) TiXmlElement::RemoveAttribute;
%warnfilter (509) TiXmlText::TiXmlText;
%warnfilter (509) TiXmlDeclaration::TiXmlDeclaration;
%warnfilter (509) TiXmlStylesheetReference::TiXmlStylesheetReference;
%warnfilter (509) TiXmlDocument::TiXmlDocument;
%warnfilter (509) TiXmlDocument::LoadFile;
%warnfilter (509) TiXmlDocument::SaveFile;
%warnfilter (509) TiXmlHandle::FirstChild;
%warnfilter (509) TiXmlHandle::FirstChildElement;
%warnfilter (509) TiXmlHandle::ChildElement;
%warnfilter (509) TiXmlHandle::Child;
%warnfilter (509) ticpp::Document::Document;
%warnfilter (509) ticpp::Document::LoadFile;
%warnfilter (509) ticpp::Element::Element;


%include <argos2/common/utility/tinyxml-cpp/ticpprc.h>
%include <argos2/common/utility/tinyxml-cpp/tinyxml.h>

%nodefaultctor ticpp::Base;
namespace ticpp {
  class Element;
  class Document;
  class Comment;
  class Text;
  class Declaration;
  class StylesheetReference;

  class Base {
  public:
    template < class T > 
      std::string ToString( const T& value ) const;
    std::string ToString( const std::string& value ) const;
    template < class T > 
      void FromString( const std::string& temp, T* out ) const;
    void FromString( const std::string& temp, std::string* out ) const;
    int Row() const;
    int Column() const;
    bool operator == ( const Base& rhs ) const;
    bool operator != ( const Base& rhs ) const;
    std::string BuildDetailedErrorString() const;
    virtual ~Base();
  protected:
    TiCppRCImp* m_impRC;
    void SetImpRC( TiXmlBase* node );
    void ValidatePointer() const;
    virtual TiXmlBase* GetBasePointer() const;
  };

  class Attribute : public Base {
  private:
    TiXmlAttribute* m_tiXmlPointer;
    TiXmlBase* GetBasePointer() const;
  public:
    Attribute();
    Attribute( const std::string& name, const std::string& value );
    Attribute( TiXmlAttribute* attribute );
    template < class T >
      void GetValue( T* value ) const;
    std::string Value() const;
    template < class T >
      void SetValue( const T& value );
    template < class T >
      void GetName( T* name ) const;
    std::string Name() const;
    template < class T >
      void SetName( const T& name );
    void operator=( const Attribute& copy );
    Attribute( const Attribute& copy );
    ~Attribute();
    Attribute* Next( bool throwIfNoAttribute = true ) const;
    Attribute* Previous( bool throwIfNoAttribute = true ) const;
    void IterateNext( const std::string&, Attribute** next ) const;
    void IteratePrevious( const std::string&, Attribute** previous ) const;
    virtual void Print( FILE* file, int depth ) const;
  private:
    void SetTiXmlPointer( TiXmlAttribute* newPointer );
  };

  class Node : public Base {
  public:
    template < class T >
      void GetValue( T* value) const;
    std::string Value() const;
    template < class T >
      void SetValue( const T& value );
    void Clear();
    Node* Parent( bool throwIfNoParent = true ) const;
    Node* FirstChild( bool throwIfNoChildren = true ) const;
    Node* FirstChild( const char* value, bool throwIfNoChildren = true ) const;
    // Node* FirstChild( const std::string& value, bool throwIfNoChildren = true ) const;
    Node* LastChild( bool throwIfNoChildren = true ) const;
    Node* LastChild( const char* value, bool throwIfNoChildren = true ) const;
    // Node* LastChild( const std::string& value, bool throwIfNoChildren = true ) const;
    Node* IterateChildren( Node* previous ) const;
    Node* IterateChildren( const std::string& value, Node* previous ) const;
    Node* InsertEndChild( Node& addThis );
    Node* LinkEndChild( Node* childNode );
    Node* InsertBeforeChild( Node* beforeThis, Node& addThis );
    Node* InsertAfterChild( Node* afterThis, Node& addThis );
    Node* ReplaceChild( Node* replaceThis, Node& withThis );
    void RemoveChild( Node* removeThis );
    Node* PreviousSibling( bool throwIfNoSiblings = true ) const;
    // Node* PreviousSibling( const std::string& value, bool throwIfNoSiblings = true ) const;
    Node* PreviousSibling( const char* value, bool throwIfNoSiblings = true ) const;
    Node* NextSibling( bool throwIfNoSiblings = true ) const;
    // Node* NextSibling( const std::string& value, bool throwIfNoSiblings = true ) const;
    Node* NextSibling( const char* value, bool throwIfNoSiblings = true ) const;
    template < class T >
      void IterateFirst( const std::string& value, T** first ) const;
    virtual void IterateFirst( const std::string&, Attribute** ) const;
    template < class T >
      void IterateNext( const std::string& value, T** next ) const;
    template < class T >
      void IteratePrevious( const std::string& value, T** previous  ) const;
    Element* NextSiblingElement( bool throwIfNoSiblings = true ) const;
    // Element* NextSiblingElement( const std::string& value, bool throwIfNoSiblings = true ) const;
    Element* NextSiblingElement( const char* value, bool throwIfNoSiblings = true ) const;
    Element* FirstChildElement( bool throwIfNoChildren = true ) const;
    Element* FirstChildElement( const char* value, bool throwIfNoChildren = true ) const;
    // Element* FirstChildElement( const std::string& value, bool throwIfNoChildren = true ) const;
    Document* GetDocument( bool throwIfNoDocument = true ) const;
    template < class T >
      T* To() const;
    Document* ToDocument() const;
    Element* ToElement() const;
    Comment* ToComment() const;
    Text* ToText() const;
    Declaration* ToDeclaration() const;
    StylesheetReference* ToStylesheetReference() const;
    // std::auto_ptr< Node > Clone() const;
    friend std::istream& operator >>( std::istream& in, Node& base );
    friend std::ostream& operator <<( std::ostream& out, const Node& base );
  protected:
    virtual TiXmlNode* GetTiXmlPointer() const = 0;
    Node* NodeFactory( TiXmlNode* tiXmlNode, bool throwIfNull = true, bool rememberSpawnedWrapper = true ) const;
    
  };
  
  template < class T = Node > class Iterator {
  private:
    T* m_p;
    std::string m_value;
  public:
    T* begin( const Node* parent ) const;
    T* end() const;
    Iterator( const std::string& value = "" );
    Iterator( T* node, const std::string& value = "" );
    Iterator( const Iterator& it );
    T* Get() const;
  };
  
  template < class T > class NodeImp : public Node {
   protected:
     T* m_tiXmlPointer;
     TiXmlNode* GetTiXmlPointer() const;
     void SetTiXmlPointer( T* newPointer );
     NodeImp( T* tiXmlPointer );
     virtual void operator=( const NodeImp<T>& copy );
     NodeImp( const NodeImp<T>& copy ) : Node( copy );
   public:
     virtual ~NodeImp();
   };
}

%template (NodeImpComment) ticpp::NodeImp<TiXmlComment>;
%template (NodeImpText) ticpp::NodeImp<TiXmlText>;
%template (NodeImpDocument) ticpp::NodeImp<TiXmlDocument>;
%template (NodeImpElement) ticpp::NodeImp<TiXmlElement>;
%template (NodeImpDeclaration) ticpp::NodeImp<TiXmlDeclaration>;
%template (NodeImpStylesheetReference) ticpp::NodeImp<TiXmlStylesheetReference>;

%warnfilter(302) ticpp::Base;
%warnfilter(302) ticpp::Attribute;
%warnfilter(302) ticpp::Node;
%warnfilter(302) ticpp::Iterator;
%warnfilter(302) ticpp::NodeImp;
%include <argos2/common/utility/tinyxml-cpp/ticpp.h>


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

////////////////////////////////////////////////////////////////////////
 // Sensors and Actuators
////////////////////////////////////////////////////////////////////////

// First thing we need it the fixups for nested classes.
%include argos-fixups.hpp

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

%nestedworkaround argos::CCI_FootBotProximitySensor::SReading;
%rename(FootBotProximitySensor) argos::CCI_FootBotProximitySensor;
%rename(get_readings_internal) argos::CCI_FootBotProximitySensor::GetReadings;
%include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>

%template(ProximitySensorReadings) std::vector<ProximitySensorReading>;

%rename(RangeAndBearingSensor) CCI_RangeAndBearingSensor;
%rename(RangeAndBearingReceivedPacket) TRangeAndBearingReceivedPacket;
%rename(RangeAndBearingData) TRangeAndBearingData;
%rename(RawValuesArray) TRawValues;
%rename(get_readings_internal) argos::CCI_RangeAndBearingSensor::GetReadings;
%{
  namespace argos {
    typedef argos::UInt8 TRangeAndBearingData[10];
    typedef argos::UInt16 TRawValues[12];
  }
%}
%include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>
%template(RangeAndBearingPackets) std::map<std::string, argos::TRangeAndBearingReceivedPacket>;

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
%rename(get_readings_internal) argos::CCI_FootBotLightSensor::GetReadings;

%include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_light_sensor.h>

%{
  namespace argos {
    typedef argos::CCI_FootBotLightSensor::SReading LightSensorReading;
  }
%}
%template (LightSensorReadings) std::vector<argos::LightSensorReading>;

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
%rename(get_readings_internal) argos::CCI_FootBotMotorGroundSensor::GetReadings;

%include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_motor_ground_sensor.h>

%{
  namespace argos {
    typedef argos::CCI_FootBotMotorGroundSensor::SReading MotorGroundSensorReading;
  }
%}
%template (MotorGroundSensorReadings) std::vector<argos::MotorGroundSensorReading>;


%extend argos::CCI_Actuator {
  argos::CCI_FootBotWheelsActuator *
    AsFootBotWheelsActuator() {
      return dynamic_cast<argos::CCI_FootBotWheelsActuator *>($self);
  }
  argos::CCI_FootBotLedsActuator *
    AsFootBotLedsActuator() {
      return dynamic_cast<argos::CCI_FootBotLedsActuator *>($self);
  }
  argos::CCI_RangeAndBearingActuator *
    AsRangeAndBearingActuator() {
      return dynamic_cast<argos::CCI_RangeAndBearingActuator *>($self);
  }
}

%extend argos::CCI_Sensor {
  argos::CCI_FootBotProximitySensor *
    AsFootBotProximitySensor() {
      return dynamic_cast<argos::CCI_FootBotProximitySensor *>($self);
  }
  argos::CCI_RangeAndBearingSensor *
    AsRangeAndBearingSensor() {
      return dynamic_cast<argos::CCI_RangeAndBearingSensor *>($self);
  }
  argos::CCI_FootBotLightSensor *
    AsFootBotLightSensor() {
      return dynamic_cast<argos::CCI_FootBotLightSensor *>($self);
  }
  argos::CCI_FootBotMotorGroundSensor *
    AsFootBotMotorGroundSensor() {
      return dynamic_cast<argos::CCI_FootBotMotorGroundSensor *>($self);
  }
}

%extend argos::CCI_FootBotProximitySensor {
  // Re-implement the shadowed get_readings method.  Use the lower
  // case name to avoid matchint the %ignore directive.
  std::vector<ProximitySensorReading>
    get_readings() {
      argos::CCI_FootBotProximitySensor::TReadings original_readings
  	= $self->GetReadings();
      return to_proximity_sensor_reading_vector(original_readings);
  }
}

%extend argos::CCI_RangeAndBearingSensor {
  // Re-implement the shadowed get_readings method.  Use the lower
  // case name to avoid matchint the %ignore directive.
  const char *
    get_readings() {
      return "RangeAndBearingSensor";
  }
}

%extend argos::CCI_FootBotLightSensor {
  // Re-implement the shadowed get_readings method.  Use the lower
  // case name to avoid matchint the %ignore directive.
  const char *
    get_readings() {
      // argos::CCI_FootBotLightSensor::TReadings original_readings
      // 	= $self->GetReadings();
      return "FootBotLightSensor";
  }
}

%extend argos::CCI_FootBotMotorGroundSensor {
  // Re-implement the shadowed get_readings method.  Use the lower
  // case name to avoid matchint the %ignore directive.
  const char *
    get_readings() {
      // argos::CCI_FootBotMotorGroundSensor::TReadings original_readings
      // 	= $self->GetReadings();
      return "FootBotMotorGroundSensor";
  }
}
