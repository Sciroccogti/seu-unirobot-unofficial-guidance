add_library(controller_options STATIC
            options.cpp)
            
target_link_libraries(controller_options
            boost_program_options)
