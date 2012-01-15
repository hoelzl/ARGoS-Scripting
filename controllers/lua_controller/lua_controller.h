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

/* Definition of the CCI_Controller class. */
#include <argos2/common/control_interface/ci_controller.h>

#include "lua.hpp"
#include "lauxlib.h"
#include "lualib.h"


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class LuaController : public CCI_Controller {

public:

   /* Class constructor. */
   LuaController();
   /* Class destructor. */
   virtual ~LuaController() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><test_obstacle_avoidance_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy();

private:
   // TODO: actually pass the arg to Lua
   inline void CallLuaFunction(const char *functionName, void* arg) {
     lua_getglobal(LuaState, functionName);
     if (lua_pcall(LuaState, 0, 0, 0)) {
       std::cerr << "Error when calling Lua " 
		 << functionName
		 << " function."
		 << std::endl;
     }
   }

   inline void CallLuaFunction(const char *functionName) {
     lua_getglobal(LuaState, functionName);
     if (lua_pcall(LuaState, 0, 0, 0)) {
       std::cerr << "Error when calling Lua " 
		 << functionName
		 << " function."
		 << std::endl;
     }
   }

   lua_State *LuaState;
   std::string LuaSource;
};

#endif
