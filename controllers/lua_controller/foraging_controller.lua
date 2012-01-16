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

function init(configuration_node)
   state.uuid = argos_utils.uuid()
   argos_utils.log("Init for robot %s", state.uuid)
   -- Some stupid examples.
   -- print("sin(pi)     = ", argos.sin(argos.Radians_PI))
   -- print("sin(2_pi/2) = ", argos.sin(argos.Radians_TWO_PI/2.0))
   -- print("sin(pi*2)   = ", argos.sin(argos.Radians_PI * 2.0))
   -- print("sin(2_pi)   = ", argos.sin(argos.Radians_TWO_PI))
   -- This does not work, unfortunately, but for understandable
   -- reasons.
   -- print("sin(2*pi)   = ", argos.sin(2.0 * argos.Radians_PI))
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
