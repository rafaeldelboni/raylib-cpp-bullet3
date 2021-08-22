# Bullet3 Hello World
add_executable(
  main-bullet
  main-bullet.cpp
)

add_dependencies(
  main-bullet
  resources
)

target_link_libraries(
  main-bullet
  BulletDynamics
  BulletCollision
  LinearMath)

# Raylib Hello World
add_executable(
  main-raylib
  main-raylib.cpp)

add_dependencies(
  main-raylib
  resources)

target_link_libraries(
  main-raylib
  raylib
  raylib-cpp)


# Raylib + Bullet3 Hello World
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
