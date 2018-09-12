
#include "mavlink_comms.h"
#include "ulog_receiver.h"

#include <iostream>
#include <thread>
#include <chrono>

int main(int /* argc */, char** /* argv */)
{
    MAVLinkComms mavlink_comms;
    mavlink_comms.connect(14540);

    UlogReceiver ulog_receiver;

    while (true) {
        std::cout << "Waiting ... " << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
