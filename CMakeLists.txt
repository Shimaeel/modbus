cmake_minimum_required(VERSION 3.16)
project(ModbusASN1 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_policy(SET CMP0144 NEW)

# ---- Boost (header-only: Asio + System are header-only since 1.69) ----
find_package(Boost 1.70 REQUIRED)

# ---- Slave executable ----
add_executable(modbus_slave
    slave/modbus_asn1_slave.cpp
    slave/main_asn1_slave.cpp
)

# ---- Master executable ----
add_executable(modbus_master
    master/modbus_master.cpp
    master/transport.cpp
    master/main_master.cpp
    functions/read_coils.cpp
    functions/read_discrete_inputs.cpp
    functions/read_holding_registers.cpp
    functions/read_input_registers.cpp
    functions/write_single_coil.cpp
    functions/write_single_register.cpp
    functions/write_multiple_coils.cpp
    functions/write_multiple_registers.cpp
    functions/mask_write_register.cpp
    functions/read_write_multiple_registers.cpp
    functions/read_fifo_queue.cpp
)

# ---- Link Boost (header-only) + platform libs ----
target_link_libraries(modbus_slave  PRIVATE Boost::headers)
target_link_libraries(modbus_master PRIVATE Boost::headers)

if(WIN32)
    # Boost.Asio on Windows still needs Winsock
    target_link_libraries(modbus_slave  PRIVATE ws2_32 mswsock)
    target_link_libraries(modbus_master PRIVATE ws2_32 mswsock)
endif()

# ---- Compiler warnings ----
if(MSVC)
    target_compile_options(modbus_slave  PRIVATE /W4)
    target_compile_options(modbus_master PRIVATE /W4)
else()
    target_compile_options(modbus_slave  PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(modbus_master PRIVATE -Wall -Wextra -Wpedantic)
endif()
