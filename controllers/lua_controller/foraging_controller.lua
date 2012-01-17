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

function init (configuration_node, controller)
   state.uuid = argos_utils.uuid()
   write_init_log_message()
   initialize_sensors_and_actuators(controller)
   initialize_configuration_data(configuration_node)
end

function write_init_log_message ()
   local log = argos_utils.log
   log("Initializing robot %s", state.uuid)
   if jit then
      log("    Running %s on %s", jit.version, jit.os)
   end
end

function initialize_sensors_and_actuators (controller)
   local log = argos_utils.log
   local robot = controller:get_robot()
   state.robot = robot
   state.actuators = {}
   state.sensors = {}
   local tmp = robot:get_actuator("footbot_wheels")
   state.actuators.wheels = tmp:as_foot_bot_wheels_actuator()
   log("Added actuator: %s", swig_type(state.actuators.wheels))
   tmp = robot:get_actuator("footbot_leds")
   state.actuators.leds = tmp:as_foot_bot_leds_actuator()
   log("Added actuator: %s", swig_type(state.actuators.leds))
   tmp = robot:get_sensor("footbot_proximity")
   state.sensors.proximity = tmp:as_foot_bot_proximity_sensor()
   log("Added sensor: %s", swig_type(state.sensors.proximity))
end

function initialize_configuration_data (configuration_node)
   local log = argos_utils.log
   state.alpha = configuration_node:get_attribute("alpha") or 7.5
   log("alpha = %s", state.alpha)
   state.delta = configuration_node:get_attribute("delta") or 0.1
   log("delta = %s", state.delta)
   state.velocity = configuration_node:get_attribute("velocity") or 5
   log("velocity = %s", state.velocity)
end

function control_step ()
   local log = argos_utils.log
   log("Control step for robot %s", state.uuid)
   local type = argos.find_module_type("argos::ProximitySensorReading *")
   log("Type name for argos::ProximitySensorReading *: %s", type)
   local proximity_readings = state.sensors.proximity:get_readings()
   log("proximity_readings: %s", swig_type(proximity_readings))
end

function reset ()
   argos_utils.log_error("Resetting %s", state.uuid)
end

function destroy()
   argos_utils.log_error("Destroying %s", state.uuid)
end
