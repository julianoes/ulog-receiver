add_library(mavlink_comms
    mavlink_comms.cpp
)

target_include_directories(mavlink_comms
    PUBLIC
    .
    c_library_v2
)

include_directories(c_library_v2)

target_link_libraries(mavlink_comms
    pthread
)
