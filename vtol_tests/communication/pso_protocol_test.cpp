#include "mavlink_sender.hpp"
#include "pso_allocation.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#define CHECK(value) do { if (!(value)) { std::cerr << "Failed: " #value << '\n'; std::exit(1); } } while (0)

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    pso_runtime::Optimizer optimizer(argv[1]);
    auto result = optimizer.solve({}, {{0,0,-10},{0,0.1,0}}, optimizer.initialActuatorState(), {16, 5, 1});
    int master, slave;
    char port[128];
    CHECK(openpty(&master, &slave, port, nullptr, nullptr) == 0);
    MAVLinkSender sender;
    CHECK(sender.init(port, 115200));
    CHECK(sender.sendPSOCommand(result));
    pollfd descriptor{master, POLLIN, 0};
    CHECK(poll(&descriptor, 1, 1000) == 1);
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    auto size = read(master, bytes, sizeof(bytes));
    CHECK(size > 0);
    mavlink_message_t message{};
    mavlink_status_t status{};
    bool complete = false;
    for (ssize_t i = 0; i < size; ++i) {
        complete |= mavlink_parse_char(MAVLINK_COMM_0, bytes[i], &message, &status) != 0;
    }
    CHECK(complete && message.msgid == MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY);
    mavlink_debug_float_array_t packet{};
    mavlink_msg_debug_float_array_decode(&message, &packet);
    CHECK(vtol_pso::valid_allocation(message, packet));
    CHECK(!result.position_converged);
    CHECK(packet.data[12] == static_cast<float>(result.objective_evaluations));
    for (unsigned i = 0; i < 9; ++i) {
        CHECK(packet.data[i] == static_cast<float>(result.actuators[i]));
    }
    auto invalid = packet;
    for (unsigned i = 0; i < 13; ++i) {
        invalid = packet;
        invalid.data[i] = std::numeric_limits<float>::quiet_NaN();
        CHECK(!vtol_pso::valid_allocation(message, invalid));
        invalid.data[i] = std::numeric_limits<float>::infinity();
        CHECK(!vtol_pso::valid_allocation(message, invalid));
    }
    for (auto field : {9u, 10u, 11u, 12u}) {
        invalid = packet;
        invalid.data[field] = -1.f;
        CHECK(!vtol_pso::valid_allocation(message, invalid));
    }
    invalid = packet; invalid.data[11] = 0.5f;
    CHECK(!vtol_pso::valid_allocation(message, invalid));
    for (float count : {0.f, 1.5f, 16777218.f}) {
        invalid = packet; invalid.data[12] = count;
        CHECK(!vtol_pso::valid_allocation(message, invalid));
    }
    invalid = packet; invalid.data[11] = 1.f;
    CHECK(vtol_pso::valid_allocation(message, invalid));
    invalid = packet; invalid.array_id = 1;
    CHECK(!vtol_pso::valid_allocation(message, invalid));
    invalid = packet; invalid.name[9] = 'x';
    CHECK(!vtol_pso::valid_allocation(message, invalid));
    invalid = packet; invalid.name[0] = 'X';
    CHECK(!vtol_pso::valid_allocation(message, invalid));
    message.sysid = 1;
    CHECK(!vtol_pso::valid_allocation(message, packet));
    message.sysid = 245; message.compid = 1;
    CHECK(!vtol_pso::valid_allocation(message, packet));
    close(master);
    close(slave);
    std::cout << "Real solver -> companion serial sender -> PX4 validation: PASS\n";
}
