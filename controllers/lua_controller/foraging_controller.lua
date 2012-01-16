-- foraging-controller.lua
-- A controller that forages for food.

-- Utility Functions.
--------------------

-- The Lua part of the utility functions.  This should go somewhere
-- else.

function argos_utils.log (...)
   argos_utils.log_internal(string.format(...))
end

function argos_utils.log_error (...)
   argos_utils.log_error_internal(string.format(...))
end

-- The Controller.
-----------------

-- Each robot is running with its own Lua state, therefore we can just
-- use a global variable to store the robot's state.
state = {}

function write_init_log_message()
   local log = argos_utils.log
   log("Initializing robot %s", state.uuid)
   if jit then
      log("    Running %s on %s", jit.version, jit.os)
   end
end

function initialize_sensors_and_actuators()
   local log = argos_utils.log
   -- log("Robot: %s", argos.get_robot())
end

function init(configuration_node, controller)
   state.uuid = argos_utils.uuid()
   write_init_log_message()
   print("configuration type", swig_type(configuration_node))
   print("configuration: has alpha?", 
	 configuration_node:has_attribute("alpha"))
   print("configuration: alpha value:",
	 configuration_node:get_attribute("alpha"))
   print("controller type", swig_type(controller))
   local robot = controller:get_robot()
   print("robot type", swig_type(robot))
   local actuator = robot:get_actuator("footbot_wheels")
   print("wheel type", swig_type(actuator))
   actuator = actuator:as_foot_bot_wheels_actuator()
   print("wheel type", swig_type(actuator))
   initialize_sensors_and_actuators()
end

function control_step()
   argos_utils.log("Control step for robot %s", state.uuid)
end

function reset()
   argos_utils.log_error("Resetting %s", state.uuid)
end

function destroy()
   argos_utils.log_error("Destroying %s", state.uuid)
end
