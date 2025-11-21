add_library(guicore
    MainWindow.cpp
    MainWindow.h
    DrawingAreas.cpp
    DrawingAreas.h
    main_update.h
    config.cpp
    config.h
)

# Set include directories
target_include_directories(guicore
    PUBLIC ${CMAKE_SOURCE_DIR}/src/guicore ${GTKMM_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMERMM_INCLUDE_DIRS})

# Link the library to necessary dependencies
target_link_libraries(guicore
    PRIVATE libmisc crsf2net control ${GTKMM_LIBRARIES} ${GSTREAMER_LIBRARIES} ${GSTREAMERMM_LIBRARIES})
set_target_properties(guicore PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/guicore)
