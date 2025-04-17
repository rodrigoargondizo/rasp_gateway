#include <opendnp3/ConsoleLogger.h>
#include <opendnp3/DNP3Manager.h>
#include <opendnp3/logging/LogLevels.h>
#include <opendnp3/outstation/DefaultOutstationApplication.h>
#include <opendnp3/outstation/IUpdateHandler.h>
#include <opendnp3/outstation/UpdateBuilder.h>
#include <opendnp3/outstation/OutstationStackConfig.h>
#include <opendnp3/outstation/SimpleCommandHandler.h>
#include <opendnp3/channel/IPEndpoint.h>
#include <opendnp3/channel/PrintingChannelListener.h>

#include <stdio.h>
#include <stdlib.h>
#include <modbus/modbus.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>

using namespace std;
using namespace opendnp3;

#define NUM_SLAVES 3
#define NUM_HOLDING_REGISTERS 2  // Para o primeiro escravo
#define NUM_INPUT_REGISTERS 1    // Para os outros escravos

struct SlaveState {
    int32_t analog_value = 0;
    bool connection_status = false;
    bool last_connection_state = false;
    int failure_count = 0;
    const int max_failures_before_zero = 5;
};

DatabaseConfig ConfigureDatabase() {
    DatabaseConfig config;

    // Configura pontos analógicos
    for(int i = 0; i < NUM_SLAVES; i++) {
        config.analog_input[i] = AnalogConfig();
        config.analog_input[i].clazz = PointClass::Class2;
        
        // Primeiro escravo (holding register) usa Group30Var1
        if (i == 0) {
            config.analog_input[i].svariation = StaticAnalogVariation::Group30Var1;
        } 
        // Outros escravos usam Group30Var2
        else {
            config.analog_input[i].svariation = StaticAnalogVariation::Group30Var2;
        }
    }

    // Configura pontos binários para status de conexão
    for (int i = 0; i < NUM_SLAVES; i++) {
        config.binary_input[i] = BinaryConfig();
        config.binary_input[i].clazz = PointClass::Class1;
        config.binary_input[i].svariation = StaticBinaryVariation::Group1Var2;
    }

    return config;
}

typedef struct {
    const char* ip;
    int port;
    int is_first_slave;
    int dnp3_analog_index;
    int dnp3_status_index;
} SlaveConfig;

void UpdateDNP3Values(UpdateBuilder& builder, const vector<SlaveState>& slave_states) {
    for (size_t i = 0; i < slave_states.size(); i++) {
        const SlaveState& state = slave_states[i];

        // Atualiza valor analógico
        if (state.failure_count >= state.max_failures_before_zero) {
            builder.Update(Analog(0), i);
        } else {
            builder.Update(Analog(state.analog_value), i);
        }

        // Atualiza status de conexão
        bool connection_failed = !state.connection_status;
        builder.Update(Binary(connection_failed), i);

        if (state.connection_status != state.last_connection_state) {
            builder.Update(Binary(connection_failed, Flags(0x01)), i);
        }
    }
}

void HandleCommunicationFailure(SlaveState& state) {
    state.connection_status = false;
    state.failure_count++;
    if (state.failure_count >= state.max_failures_before_zero) {
        state.analog_value = 0; // Garante que o valor será 0 no DNP3
    }
}

int main() {
    const auto logLevels = levels::NORMAL | levels::NOTHING;
    DNP3Manager manager(1, ConsoleLogger::Create());

    // Configuração do canal DNP3
    auto channel = std::shared_ptr<IChannel>(nullptr);
    try {
        channel = manager.AddTCPServer("server", logLevels, ServerAcceptMode::CloseExisting,
                                       IPEndpoint("192.168.100.176", 20000),
                                       PrintingChannelListener::Create());
    } catch (const std::exception& e) {
        std::cerr << "Erro ao configurar canal DNP3: " << e.what() << '\n';
        return -1;
    }

    // Configuração da pilha DNP3
    OutstationStackConfig config(ConfigureDatabase());
    config.outstation.eventBufferConfig = EventBufferConfig::AllTypes(10);
    config.outstation.params.allowUnsolicited = true;
    config.link.LocalAddr = 2;
    config.link.RemoteAddr = 1;
    config.link.KeepAliveTimeout = TimeDuration::Seconds(30);

    auto outstation = channel->AddOutstation("outstation", SuccessCommandHandler::Create(),
                                             DefaultOutstationApplication::Create(), config);
    outstation->Enable();

    // Configuração dos escravos Modbus
    SlaveConfig slaves[NUM_SLAVES] = {
        {"192.168.100.120", 502, 1, 0, 0},  // Primeiro escravo (holding registers)
        {"192.168.100.121", 502, 0, 1, 1},  // Segundo escravo (input registers)
        {"192.168.100.122", 502, 0, 2, 2}   // Terceiro escravo (input registers)
    };

    // Endereços Modbus
    const int FIRST_HOLDING_REG = 23322 - 1; // Ajuste para offset 0-based
    const int SECOND_INPUT_REG = 39 - 1;     // Ajuste para offset 0-based

    vector<SlaveState> slave_states(NUM_SLAVES);

    // Buffer para leitura Modbus
    uint16_t tab_reg[NUM_HOLDING_REGISTERS > NUM_INPUT_REGISTERS ?
                     NUM_HOLDING_REGISTERS : NUM_INPUT_REGISTERS];

    while (true) {
        UpdateBuilder builder;

        for (int i = 0; i < NUM_SLAVES; ++i) {
            modbus_t* ctx = modbus_new_tcp(slaves[i].ip, slaves[i].port);
            slave_states[i].connection_status = false;

            if (ctx == NULL) {
                cerr << "Erro ao criar contexto Modbus" << endl;
                HandleCommunicationFailure(slave_states[i]);
                continue;
            }

            // Configura timeout
            modbus_set_response_timeout(ctx, 1, 0); // 1 segundo
            modbus_set_byte_timeout(ctx, 1, 0);

            if (modbus_connect(ctx) == -1) {
                cerr << "Falha na conexão" << endl;
                modbus_free(ctx);
                HandleCommunicationFailure(slave_states[i]);
                continue;
            }

            // Tentativa de leitura
            bool read_success = false;
            if (slaves[i].is_first_slave) {
                if (modbus_read_registers(ctx, FIRST_HOLDING_REG, NUM_HOLDING_REGISTERS, tab_reg) != -1) {
                    
                    
                    // Big-endian (padrão Modbus)
                    //int32_t valor = (tab_reg[0] << 16) | tab_reg[1];

                    // Little-endian (alguns dispositivos)
                    //int32_t valor = (tab_reg[1] << 16) | tab_reg[0];

                    int32_t valor_combinado = (int32_t)((tab_reg[0] << 16) | tab_reg[1]);
                    slave_states[i].analog_value = static_cast<int32_t>(valor_combinado);
                    cout << "Escravo " << i << " (Holding): " << valor_combinado << endl;
                    read_success = true;
                }
            } else {
                if (modbus_read_input_registers(ctx, SECOND_INPUT_REG, NUM_INPUT_REGISTERS, tab_reg) != -1) {
                    slave_states[i].analog_value = static_cast<int16_t>(tab_reg[0]);
                    read_success = true;
                }
            }

            if (read_success) {
                slave_states[i].connection_status = true;
                slave_states[i].failure_count = 0;
            } else {
                HandleCommunicationFailure(slave_states[i]);
            }

            modbus_close(ctx);
            modbus_free(ctx);
        }

        // Aplica atualizações DNP3
        UpdateDNP3Values(builder, slave_states);
        outstation->Apply(builder.Build());

        this_thread::sleep_for(chrono::seconds(1));
    }

    return 0;
}
