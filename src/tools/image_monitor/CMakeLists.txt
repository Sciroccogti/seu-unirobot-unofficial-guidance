include_directories(${Qt5Widgets_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

set(TARGET_NAME image_monitor)

add_executable(${TARGET_NAME}
        main.cpp
        ${TARGET_NAME}.cpp)

target_link_libraries(${TARGET_NAME}
        opencv_core
        opencv_imgcodecs
        opencv_imgproc
        ui
        robot
        tcp_client
        debuger_options
        Qt5::Widgets
        Qt5::Core
        Qt5::Gui)

install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_SYSTEM_PROCESSOR}")
