ARGoS Scripting
===============

This is a small experiment to test the integration of scripting
languages into the ARGoS simulator for robot swarms.  The wrapping of
ARGoS is done using the SWIG interface generator.

Complation Notes
----------------

The makefiles in this repository assume that you have LuaJIT
installed.  If you want to use the traditional Lua implementation,
just change the file `controllers/lua_controller/CMakeLists.txt` from

	target_link_libraries(lua_controller luajit-5.1 uuid)

to

	target_link_libraries(lua_controller lua uuid)

and everything will continue to work (albeit more slowly).

For using LuaJIT on 64-bit Mac OS X you have to compile the ARGoS
simulator with the following linker flags: `-Wl,-pagezero_size,10000
-Wl,-image_base,100000000`.  A quick and dirty hack to achieve this is
to modify the file `argos2/simulator/CMakeLists.txt` by changing the
line

	target_link_libraries(argos argos2_simulator)

to

	target_link_libraries(argos argos2_simulator -Wl,-pagezero_size,10000 -Wl,-image_base,100000000)

If you don't want to compile ARGoS yourself you'll have to use the
standard Lua interpreter as described above.