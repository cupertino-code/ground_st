add_library(control
    include/protocol.h
    control.c
    include/common.h
    include/control.h
)

# Set include directories
target_include_directories(control
    PUBLIC ${CMAKE_SOURCE_DIR}/src/control/include)
target_include_directories(control
    PRIVATE ${CMAKE_SOURCE_DIR}/src/control)
include(FetchContent)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG master)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

target_link_libraries(control  PRIVATE libmisc yaml-cpp guicore)
set_target_properties(control PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/control)
