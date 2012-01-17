-- foraging-controller.lua
-- A controller that forages for food.

-- SCEL
-------

components = {}

initial_component = {
   interface = {
      get = function (self, pattern, location)
	 local remote_component = self.interface:resolve_location(location)
	 remote_component.interface:local_get(pattern, self)
      end,
      put = function (self, tuple, location)
	 local remote_component = self.interface:resolve_location(location)
	 remote_component.interface:local_put(tuple, self)
      end,
      qry = function (self, pattern, location)
	 local remote_component = self.interface:resolve_location(location)
	 remote_component.interface:local_qry(pattern, self)
      end,
      exec = function (process)
	 -- add process to list of coroutines for self and yield
      end,
      new = function (interface, kb, policy, process)
	 new_component = {
	    interface = interface,
	    knowledge_base = kb,
	    policy = policy,
	    process = process
	 }
	 components[#components] = new_component
	 return new_component
      end,
      resolve_location = function (location)
	 return nil
      end
   },
   knowledge_base = {},
   policy = {
      allow_put_access = function (self, tuple, sender)
	 return true
      end,
      allow_get_access = function (self, tuple, sender)
	 return true
      end,
      allow_qry_access = function (self, tuple, sender)
	 return true
      end
   },
   processes = {},
   tuples = {}
}

components[1] = initial_component

function local_put (self, tuple, sender)
   -- Put 'tuple' into the local tuple space if the policy allows it.
   local tuples = self.tuples
   local policy = self.policy
   if (policy:allow_put_access(tuple, sender)) then
      tuples[#tuples + 1] = tuple
   end
end

function local_get (self, pattern, sender)
   -- Get a tuple matching 'pattern' from the local tuple space, if
   -- the policy agrees.
   local tuples = self.tuples
   local policy = self.policy
   if (policy:allow_get_access(tuple, sender)) then
      local index = match_tuple(pattern)
      if index then
	 return tuples[index]
      else
	 return false
      end
   end
end

function local_qry (self, pattern, sender)
   -- Query the local tuple space for 'pattern' if the pollicy allows
   -- it.
   if (self.policy:allow_qry_access(tuple, sender)) then
      local tuples = state.tuples
      local index = match_tuple(pattern)
      if index then
	 return tuples[index]
      else
	 return false
      end
   end
end

function match_tuple (pattern)
   -- Returns the index of a tuple matching pattern or false if no
   -- such tuple exists in the tuple space
   return false
end

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
state = {
}

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
   -- log("alpha = %s", state.alpha)
   state.delta = configuration_node:get_attribute("delta") or 0.1
   -- log("delta = %s", state.delta)
   state.velocity = configuration_node:get_attribute("velocity") or 5
   -- log("velocity = %s", state.velocity)
end

function control_step ()
   local log = argos_utils.log
   log("Control step for robot %s", state.uuid)
   -- argos.print_module_types()
   local proximity_sensor = state.sensors.proximity
   local readings = proximity_sensor:get_readings()
   local size = readings:size()
   local acc = argos.Vector2(0, 0)
   for i = 0, size - 1 do
      local reading = readings[i];
      acc = acc + argos.Vector2(reading.value, reading.angle)
   end
   acc = acc / size
   local angle = acc:angle()
   -- TODO: remove inline constants
   local wheel_actuator = state.actuators.wheels
   local uuid = state.uuid
   local v = state.velocity
   if math.abs(angle:get_value()) < 0.1 and acc:length() < 0.1 then
      log("%s: going straight ahead", uuid)
      wheel_actuator:set_linear_velocity(v, v)
   else
      log("%s: turning", uuid)
      if angle:get_value() < 0 then
	 wheel_actuator:set_linear_velocity(0, v)
      else
	 wheel_actuator:set_linear_velocity(v, 0)
      end
   end

   -- Set the LEDs.
   local leds = state.actuators.leds
   leds:set_all_colors(argos.Color_BLACK)
   angle:unsigned_normalize()
   local led_index = angle / argos.Radians_TWO_PI * 12
   leds:set_single_color(led_index, argos.Color_RED)
end

function reset ()
   argos_utils.log_error("Resetting %s", state.uuid)
end

function destroy()
   argos_utils.log_error("Destroying %s", state.uuid)
end
