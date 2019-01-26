add_library(sensor STATIC
            button.cpp
            camera.cpp
            motor.cpp
            imu.cpp)
            
target_link_libraries(sensor
            MVSDK
            server
            dynamixel
            gpio
            boost_system)
