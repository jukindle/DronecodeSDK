
#include <chrono>
#include <cstdint>
#include <dronecore/action.h>
#include <dronecore/dronecore.h>
#include <dronecore/telemetry.h>
#include <iostream>
#include <thread>

using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using namespace dronecore;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}
std::vector< float > heights;
std::vector< float > longs;
std::vector< float > lats;
int main(int argc, char **argv)
{
    DroneCore dc;
    std::string connection_url;
    ConnectionResult connection_result;

    bool discovered_system = false;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Connection failed: " << connection_result_str(connection_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    std::cout << "Waiting to discover system..." << std::endl;
    dc.register_on_discover([&discovered_system](uint64_t uuid) {
        std::cout << "Discovered system with UUID: " << uuid << std::endl;
        discovered_system = true;
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
    // seconds.
    sleep_for(std::chrono::seconds(2));

    if (!discovered_system) {
        std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting." << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    // We don't need to specify the UUID if it's only one system anyway.
    // If there were multiple, we could specify it with:
    // dc.system(uint64_t uuid);
    System &system = dc.system();
    auto telemetry = std::make_shared<Telemetry>(system);
    auto action = std::make_shared<Action>(system);

    // We want to listen to the altitude of the drone at 5 Hz.
    const Telemetry::Result set_rate_result = telemetry->set_rate_position(5.0);
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Setting rate failed:" << Telemetry::result_str(set_rate_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }



    /////////////////// IMPORTANT FOR KNIGHTS /////////////////////////
    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry->position_async([](Telemetry::Position position) {
        std::cout << TELEMETRY_CONSOLE_TEXT // set to blue
                  << "Altitude: " << position.relative_altitude_m << " m"
                  << NORMAL_CONSOLE_TEXT // set to default color again
                  << std::endl;
        heights.push_back(position.relative_altitude_m);
        longs.push_back(position.longitude_deg);
        lats.push_back(position.latitude_deg);
    });

    /////////////////// END IMPORTANT FOR KNIGHTS /////////////////////////

    // Wait until vehicle is ready to arm.
    while (telemetry->health_all_ok() != true) {
        std::cout << ERROR_CONSOLE_TEXT << "Vehicle not ready to arm" << NORMAL_CONSOLE_TEXT
                  << std::endl;
        sleep_for(std::chrono::seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const ActionResult arm_result = action->arm();

    if (arm_result != ActionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << action_result_str(arm_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    getchar();
    std::cout << "Finished..." << std::endl << std::endl << std::endl;




    /////////////////// IMPORTANT FOR KNIGHTS /////////////////////////
    std::vector<float>::const_iterator heights_i = heights.begin();
    std::vector<float>::const_iterator longs_i = longs.begin();
    std::vector<float>::const_iterator lats_i = lats.begin();

    // ECHO WAYPOINTS IN JSON FORMAT
    std::cout << "Waypoints in JSON format: " << std::endl << std::endl;
    std::cout << "{\"Waypoints\": [";
    while (heights_i < heights.end()) {
      std::cout << "{\"lng\": " << *longs_i << ", \"lat\": " << *lats_i << ", \"height\": " << *heights_i << "}";
      heights_i++;
      longs_i++;
      lats_i++;
      if (heights_i < heights.end()) std::cout << ", ";
    }
    std::cout << "]}" << std::endl << std::endl;

    /////////////////// END IMPORTANT FOR KNIGHTS /////////////////////////

    return 0;
}
