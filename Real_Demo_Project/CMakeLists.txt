cmake_minimum_required(VERSION 3.10)  # Versão mínima do CMake
project(DNP3_Modbus_Integration)      # Nome do projeto integrado

# Configurações do C++ (usando C++11 ou superior)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Procura pela biblioteca libmodbus
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# Adiciona o diretório de include da libmodbus
include_directories(${LIBMODBUS_INCLUDE_DIRS})

#include_directories(/usr/local/include/opendnp3/gen)
#include_directories(/usr/local/include/opendnp3/app)

# Adiciona o executável principal (integra DNP3 e Modbus)
add_executable(dnp3_modbus_integration main.cpp)

# Linka as bibliotecas necessárias
target_link_libraries(dnp3_modbus_integration PRIVATE opendnp3 ${LIBMODBUS_LIBRARIES})
#target_link_libraries(dnp3_modbus_integration PRIVATE ${LIBMODBUS_LIBRARIES})


# Define propriedades adicionais para o alvo (opcional)
set_target_properties(dnp3_modbus_integration PROPERTIES FOLDER cpp/src)

# Instala o executável no diretório bin (opcional)
install(TARGETS dnp3_modbus_integration RUNTIME DESTINATION bin)