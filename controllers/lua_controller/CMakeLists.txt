link_directories(/usr/local/lib /opt/local/lib)
add_library(lua_wrapper MODULE lua_wrapper.cpp)
add_library(lua_controller MODULE lua_controller.cpp)
target_link_libraries(lua_controller lua)
