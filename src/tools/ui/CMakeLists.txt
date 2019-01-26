include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_library(ui STATIC
            camera_setter.cpp
            joint_revise.cpp
            walk_remote.cpp
            ImageLabel.cpp
            RobotGL.cpp)

target_link_libraries(ui
            opencv_core
            opencv_imgproc
            tcp_client
            glut
            GLU
            GL
            Qt5::Widgets
            Qt5::Core
            Qt5::Gui)