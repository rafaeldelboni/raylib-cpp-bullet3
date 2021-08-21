add_executable(
  main
  main.cpp
)

add_dependencies(
  main
  resources
)

target_link_libraries(
  main
  raylib-cpp raylib
)
