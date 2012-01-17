link_directories(/usr/local/lib /opt/local/lib /usr/lib)
# add_module(lua_wrapper MODULE lua_wrapper.cpp)
add_library(lua_controller MODULE lua_wrapper.cpp lua_controller.cpp)

# Uncomment the following lines (and comment out the line after that)
# to compile with LuaJIT.  Not that this might require a custom build
# of ARGoS on Mac OS.
#
# And some day I will figure out how to use CMake and make this a
# compile time option.  --tc
#
# target_link_libraries(lua_controller luajit-5.1 uuid)
target_link_libraries(lua_controller lua uuid)
