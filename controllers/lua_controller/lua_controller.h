/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *         Matthias Hölzl <tc@xantira.com>
 *
 * This controller is meant to be used with the XML files:
 * xml/lua.xml
 */

#ifndef LUA_CONTROLLER_H
#define LUA_CONTROLLER_H

#include <string>
#include <argos2/common/control_interface/ci_controller.h>
#include "lua.hpp"
#include "lauxlib.h"
#include "lualib.h"
#include "lua_wrapper.h"

using namespace argos;

class LuaController : public CCI_Controller {

public:
   LuaController();
   virtual ~LuaController() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy();

private:
   void InitializeLuaInterpreter(std::string LuaSource);

   inline void PrintModuleState() {
     swig_module_info *moduleInfo = SWIG_GetModule(LuaState);
     std::cerr << "Module: " << moduleInfo
	       << ", " << moduleInfo->size
	       << std::endl;
     for (unsigned int i = 0; i < moduleInfo->size; ++i) {
       std::cerr << "Type: " << moduleInfo->types[i]->name << std::endl;
     }
   }

   inline void CallLuaInitFunction(TConfigurationNode *configurationNode,
				   CCI_Controller *controller) {
     lua_getglobal(LuaState, "init");

     // TConfigurationNode is an abbreviation for ticpp::Element
     swig_type_info *configNodeType
       = SWIG_TypeQuery(LuaState, "_p_ticpp__Element");
     if (configNodeType) {
       SWIG_Lua_NewPointerObj(LuaState, configurationNode, configNodeType, 0);
     }
     else {
       std::cerr << "Type info for TConfigurationNode not found." << std::endl;
     }
     if (lua_pcall(LuaState, 1, 0, 0)) {
       std::cerr << "Error when calling Lua init function:" 
		 << std::endl
		 << lua_tostring(LuaState, -1)
		 << std::endl;
     }
   }

   inline void CallLuaFunction(const char *functionName) {
     lua_getglobal(LuaState, functionName);
     if (lua_pcall(LuaState, 0, 0, 0)) {
       std::cerr << "Error when calling Lua function " 
		 << functionName
		 << ":" << std::endl
		 << lua_tostring(LuaState, -1)
		 << std::endl;
     }
   }

   lua_State *LuaState;
   std::string LuaSource;
};

#endif
