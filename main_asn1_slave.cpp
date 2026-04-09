/**
 * @file main_asn1_slave.cpp
 * @brief Modbus Slave demo application (TCP via Boost.Asio).
 *
 * Starts a slave on port 502 listening on TCP, and
 * pre-populates some registers/coils so the master demo can read
 * meaningful values.  Stop with Ctrl-C or SIGINT.
 *
 * Usage:
 *   modbus_slave              -- listen on TCP
 */

#include "modbus_asn1_slave.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

/** @brief Global flag set by the signal handler to request a clean shutdown. */
static std::atomic<bool> g_quit{false};

/**
 * @brief Signal handler for SIGINT / SIGTERM.
 */
static void sigHandler(int) { g_quit = true; }

/**
 * @brief Entry point for the slave demo.
 * @return 0 on clean shutdown, 1 if the server fails to start.
 */
int main(int argc, char* argv[])
{
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    auto logger = [](const std::string& msg) {
        std::cout << msg << '\n' << std::flush;
    };

    Modbus::Slave slave(
        /*unitId=*/1,
        /*port=*/Modbus::DEFAULT_PORT,
        logger);

    // --- Pre-populate data model -------------------------------------------

    // Coils 0-4
    slave.setCoil(0, true);
    slave.setCoil(1, false);
    slave.setCoil(2, true);
    slave.setCoil(3, true);
    slave.setCoil(4, false);

    // Discrete inputs 0-3  (read-only from master side)
    slave.setDiscreteInput(0, true);
    slave.setDiscreteInput(1, true);
    slave.setDiscreteInput(2, false);
    slave.setDiscreteInput(3, true);

    // Holding registers 0-4
    slave.setHoldingRegister(0,  1000);
    slave.setHoldingRegister(1,  2000);
    slave.setHoldingRegister(2,  3000);
    slave.setHoldingRegister(3,  4000);
    slave.setHoldingRegister(4,  5000);

    // Input registers 0-3 (read-only)
    slave.setInputRegister(0, 100);
    slave.setInputRegister(1, 200);
    slave.setInputRegister(2, 300);
    slave.setInputRegister(3, 400);

    // --- Start server -------------------------------------------------------
    if (!slave.start()) {
        std::cerr << "[Slave] Failed to start\n";
        return 1;
    }

    std::cout << "[Slave] Running (TCP). Press Ctrl-C to stop...\n" << std::flush;

    while (!g_quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    slave.stop();
    return 0;
}
