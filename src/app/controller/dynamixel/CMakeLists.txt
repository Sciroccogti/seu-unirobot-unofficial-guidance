add_library(dynamixel STATIC
            group_bulk_read.cpp
            group_bulk_write.cpp
            group_sync_read.cpp
            group_sync_write.cpp
            packet_handler.cpp
            port_handler.cpp
            port_handler_linux.cpp
            protocol1_packet_handler.cpp
            protocol2_packet_handler.cpp)
