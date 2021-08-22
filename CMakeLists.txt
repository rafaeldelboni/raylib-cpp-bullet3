# Setup
cmake_minimum_required(VERSION 3.20)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FetchContent)
set(FETCHCONTENT_QUIET 0)

project (raylib-cpp-bullet3
    VERSION 0.0.1
    DESCRIPTION "raylib-cpp C++ & bullet3"
    HOMEPAGE_URL "https://github.com/rafaeldelboni/raylib-cpp-bullet3"
    LANGUAGES C CXX)

# Fetch Raylib
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_SHALLOW    TRUE
  GIT_PROGRESS   TRUE
  GIT_TAG        master)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # don't build the supplied examples
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE) # or games

FetchContent_MakeAvailable(raylib)

# Fetch Raylib-cpp
FetchContent_Declare(
  raylib_cpp
  GIT_REPOSITORY https://github.com/robloach/raylib-cpp
  GIT_SHALLOW    TRUE
  GIT_PROGRESS   TRUE
  GIT_TAG        master)
set(BUILD_RAYLIB_CPP_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_GetProperties(raylib_cpp)
if(NOT raylib_cpp_POPULATED)
  FetchContent_Populate(raylib_cpp)

  file(COPY ${raylib_cpp_SOURCE_DIR}/vendor/raylib/src/raylib.h DESTINATION ${raylib_cpp_SOURCE_DIR}/include)
  file(COPY ${raylib_cpp_SOURCE_DIR}/vendor/raylib/src/raymath.h DESTINATION ${raylib_cpp_SOURCE_DIR}/include)

  add_subdirectory(${raylib_cpp_SOURCE_DIR} ${raylib_cpp_BINARY_DIR})
endif()

# Fetch Bullet3
FetchContent_Declare(
  bullet
  GIT_REPOSITORY https://github.com/bulletphysics/bullet3
  GIT_SHALLOW    TRUE
  GIT_PROGRESS   TRUE
  GIT_TAG        master)
set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
set(BUILD_PYBULLET OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)

FetchContent_GetProperties(bullet)
if(NOT bullet_POPULATED)
  FetchContent_Populate(bullet)
  add_subdirectory(${bullet_SOURCE_DIR} ${bullet_BINARY_DIR})
  target_include_directories(Bullet3Collision PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(Bullet3Common PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(Bullet3Dynamics PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(Bullet3Geometry PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(Bullet3OpenCL_clew PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(Bullet2FileLoader PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(BulletCollision PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(BulletDynamics PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(BulletInverseDynamics PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(BulletSoftBody PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
  target_include_directories(LinearMath PUBLIC ${BULLET_PHYSICS_SOURCE_DIR}/src)
endif()

# Add symbolic link to resources folder in build
add_custom_target(resources)
FILE(TO_NATIVE_PATH "${PROJECT_SOURCE_DIR}/resources" source)
FILE(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/resources" destination)
IF (WIN32)
  add_custom_command(
    TARGET resources POST_BUILD
    COMMAND mklink /D ${destination} ${source}
    DEPENDS ${destination}
    COMMENT "symbolic link resources folder from ${source} => ${destination}"
    )
ELSE()
  add_custom_command(
    TARGET resources POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${source} ${destination}
    DEPENDS ${destination}
    COMMENT "symbolic link resources folder from ${source} => ${destination}"
    )
ENDIF()

# C++
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generates a compile_commands.json file containing the exact compiler calls
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Include Directory
add_subdirectory(src)
