
add_executable(ulog_receiver_bin
    main.cpp
)

target_link_libraries(ulog_receiver_bin
    mavlink_comms
    ulog_receiver
)
