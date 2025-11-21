add_library(libmisc
    include/crsf_protocol.h
    utils.c
    include/utils.h
)

# Set include directories
target_include_directories(libmisc
    PUBLIC ${CMAKE_SOURCE_DIR}/src/libmisc/include)

# Link the library to necessary dependencies
#target_link_libraries(libmisc  PRIVATE                 )
set_target_properties(libmisc PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libmisc)
