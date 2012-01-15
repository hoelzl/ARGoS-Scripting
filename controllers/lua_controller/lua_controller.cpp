#include "lua_controller.h"
#include "lua.hpp"
#include "lauxlib.h"
#include "lualib.h"

#include <iostream>

using namespace std;

static void dumpLuaStack (lua_State *L) {
  int top = lua_gettop(L);
  for (int i = 1; i <= top; ++i) {
    int t = lua_type(L, i);
    switch (t) {
    case LUA_TSTRING: {
      cerr << L << " (" << i << "): " << lua_tostring(L, i);
      break;
    }
// Gcc complains about these cases.  Investigate why.
#if (0)
    case LUA_TBOOLEAN: {
      cerr << L << " (" << i << "): " << lua_toboolean(L, i) ? "true" : "false";
      break;
    }
    case LUA_TNUMBER: {
      cerr << L << " (" << i << "): " << lua_tonumber(L, i);
      break;
    } 
#endif
    default: {
      cerr << L << " (" << i << "): " << lua_typename(L, t); 
      break;
    }
    } 
    cerr << " ";
  } 
  cerr << endl;
}

LuaController::LuaController() :
  LuaState(luaL_newstate()),
  LuaSource("/Users/tc/Prog/Robots/Lua-Foraging/controllers/lua_controller/foraging_controller.lua")
{
  luaL_openlibs(LuaState);
  if (luaL_loadfile(LuaState, LuaSource.c_str())) {
    cerr << "Error while loading Lua controller source." << endl;
    dumpLuaStack(LuaState);
  }
  else if (lua_pcall(LuaState, 0, 0, 0)) {
    cerr << "Error while initializing Lua controller." << endl;
    dumpLuaStack(LuaState);
  }
}

void LuaController::Init(TConfigurationNode& configurationNode) {
  CallLuaFunction("init", &configurationNode);
}

void LuaController::ControlStep() {
  CallLuaFunction("control_step");
}

void LuaController::Reset() {
  CallLuaFunction("reset");
}

void LuaController::Destroy() {
  CallLuaFunction("destroy");
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.  The string is then usable in the XML
 * configuration file to refer to this controller.  When ARGoS reads
 * that string in the XML file, it knows which controller class to
 * instantiate.  See also the XML configuration files for an example
 * of how this is used.
 */
REGISTER_CONTROLLER(LuaController, "lua_controller")
