
        #find_package( Threads )

        add_executable(dynamixel_test
        test_thread.cpp
        subsrc/group_bulk_read.cpp
        subsrc/group_bulk_write.cpp
        subsrc/group_sync_read.cpp
        subsrc/group_sync_write.cpp
        subsrc/packet_handler.cpp
        subsrc/port_handler.cpp
        subsrc/port_handler_linux.cpp
        subsrc/protocol1_packet_handler.cpp
        subsrc/protocol2_packet_handler.cpp
        subsrc/trajectory.cpp
        subsrc/controller.cpp
        subsrc/dynamixel_read_write.cpp
        subsrc/linear_read_write.cpp     
        )
        

        target_link_libraries(dynamixel_test pthread)
        #target_link_libraries( dynamixel_test ${CMAKE_THREAD_LIBS_INIT} )