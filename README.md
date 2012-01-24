ARGoS Scripting
===============

This is a small experiment to test the integration of scripting
languages into the ARGoS simulator for robot swarms.  The wrapping of
ARGoS is done using the SWIG interface generator.

Compilation Notes
-----------------

To rebuild the wrapper for your ARGoS installation change to the `lua`
directory and execute `build-swig-wapper.sh`:

          cd lua; ./build-swig-wrapper.sh; cd ..

This should result in output similar to the following:

    Language subdirectory: lua
    Search paths:
        ./
        /usr/include/
        /opt/local/include/
        /usr/include/c++/4.2.1/
        ./swig_lib/lua/
        /opt/local/share/swig/2.0.4/lua/
        ./swig_lib/
        /opt/local/share/swig/2.0.4/
    Preprocessing...
    Starting language-specific parse...
    Processing types...
    C++ analysis...
    Generating wrappers...
    Done

Then create a `build` directory, switch to that directory, run `cmake
..` and finally run `make`.  To invoke the lua controller run

    launch_argos -nc xml/lua-controller.xml

from the root directory.

The makefiles in this repository assume that you have the traditional
Lua implementation installed and that your linker can find it in the
standard paths.

If you want to use the fabulous LuaJIT, just change the file
`controllers/lua_controller/CMakeLists.txt` from

        target_link_libraries(lua-controller lua uuid)

to

        target_link_libraries(lua-controller luajit-5.1 uuid)

and everything should continue to work (albeit more quickly).

For using LuaJIT on 64-bit Mac OS X you have to compile the ARGoS
simulator with the following linker flags: `-Wl,-pagezero_size,10000
-Wl,-image_base,100000000`.  A quick and dirty hack to achieve this is
to modify the file `argos2/simulator/CMakeLists.txt` by changing the
line

        target_link_libraries(argos argos2_simulator)

to

        target_link_libraries(argos argos2_simulator -Wl,-pagezero_size,10000 -Wl,-image_base,100000000)

If you don't want to compile ARGoS yourself you'll have to stay with
the standard Lua interpreter for the time being.