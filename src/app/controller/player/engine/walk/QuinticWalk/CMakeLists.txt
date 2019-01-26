file(GLOB WALKFILES "*.cpp")
add_library(quinticwalk STATIC
            ${WALKFILES})   
