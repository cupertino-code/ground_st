add_library(crsf2net
    circ_buf.c
    circ_buf.h
    crsf2net.c
    include/crsf2net.h
)

# Set include directories
target_include_directories(crsf2net
    PUBLIC ${CMAKE_SOURCE_DIR}/src/crsf2net/include)
target_include_directories(crsf2net
    PRIVATE ${CMAKE_SOURCE_DIR}/src/crsf2net)
# Link the library to necessary dependencies
target_link_libraries(crsf2net  PRIVATE libmisc control guicore pthread)
set_target_properties(crsf2net PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/crsf2net)
