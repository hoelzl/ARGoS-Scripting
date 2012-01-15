-- foraging-controller.lua
-- A controller that forages for food.

function init(configuration_node)
   print("Lua: init")
   print("sin(pi) = ", argos.sin(argos.Radians_PI))
   print("sin(2*pi) = ", argos.sin(argos.Radians_TWO_PI))
end

function control_step()
   print("Lua: control_step")
end

function reset()
   print("Lua: reset")
end

function destroy()
   print("Lua: destroy")
end
