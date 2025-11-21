# video-streamer/src/app/CMakeLists.txt
add_executable(ground_station  main.cpp)

# Link the executable to your custom library and the external dependencies
target_link_libraries(ground_station PRIVATE 
    guicore
    crsf2net
    libmisc
    control
    ${GTKMM_LIBRARIES} ${GSTREAMER_LIBRARIES})

set_target_properties(ground_station PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
