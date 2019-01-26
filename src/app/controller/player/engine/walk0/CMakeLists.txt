add_library(walk STATIC
            Spline.cpp
            CubicSpline.cpp
            SmoothSpline.cpp
            Polynom.cpp
            WalkEngine.cpp)

target_link_libraries(walk pthread)