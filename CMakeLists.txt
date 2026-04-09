cmake_minimum_required(VERSION 3.16)
project(ModbusASN1 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_policy(SET CMP0144 NEW)

# ---- Boost (header-only: Asio + System are header-only since 1.69) ----
find_package(Boost 1.70 REQUIRED)

# ---- Slave executable ----
add_executable(modbus_slave
    modbus_asn1_slave.cpp
    main_asn1_slave.cpp
)

# ---- Master executable ----
add_executable(modbus_master
    modbus_asn1_master.cpp
    main_asn1_master.cpp
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
