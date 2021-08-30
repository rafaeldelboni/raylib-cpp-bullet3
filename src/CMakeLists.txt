add_executable(
  main
  main.cpp)

add_dependencies(
  main
  resources)

target_link_libraries(
  main
  raylib
  raylib-cpp
  BulletDynamics
  BulletCollision
  LinearMath)
