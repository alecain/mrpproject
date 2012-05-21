cmake_minimum_required(VERSION 2.4)
PROJECT (player_test)

add_library(Vector2d Vector2d.cpp)
add_library(Scan Scan.cpp)
target_link_libraries(Vector2d m)

SET (CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules")
#SET (CMAKE_MODULE_PATH "/usr/local/dcs/share/cmake/Modules:/usr/share/cmake-2.8/Modules")
SET(CMAKE_BUILD_TYPE debug)
INCLUDE (UsePlayerC++)
find_package(GLUT)
find_package(OpenGL)

#add_executable(map map.cpp Particle.cpp)
#target_link_libraries(map Vector2d)

PLAYER_ADD_PLAYERCPP_CLIENT (navigator SOURCES map.cpp navigator.cpp Particle.cpp Ploc.cpp safeGoto.cpp LINKFLAGS)
target_link_libraries(navigator ${GLUT_LIBRARY} ${OPENGL_LIBRARY} X11 Xext Vector2d Scan)
