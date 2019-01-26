include_directories(${Qt5Widgets_INCLUDE_DIRS})

set(TARGET_NAME team_monitor)

add_executable(${TARGET_NAME}
        main.cpp
        ${TARGET_NAME}.cpp)

target_link_libraries(${TARGET_NAME}
        boost_system
        pthread
        Qt5::Widgets
        Qt5::Core
        Qt5::Gui)

install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_SYSTEM_PROCESSOR}")
