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
#define NUM_HOLDING_REGISTERS 2
#define NUM_INPUT_REGISTERS 1

struct SlaveState {
    int32_t analog_value = 0;
    bool connection_status = false;
    bool last_connection_state = false;
    int failure_count = 0;
    const int max_failures_before_zero = 5;
};

DatabaseConfig ConfigureDatabase() {
    DatabaseConfig config;

    for(int i = 0; i < NUM_SLAVES; i++) {
        config.analog_input[i] = AnalogConfig();
        config.analog_input[i].clazz = PointClass::Class2;
        config.analog_input[i].svariation = (i == 0) ? StaticAnalogVariation::Group30Var1 
                                                   : StaticAnalogVariation::Group30Var2;

        config.binary_input[i] = BinaryConfig();
        config.binary_input[i].clazz = PointClass::Class1;
        config.binary_input[i].svariation = StaticBinaryVariation::Group1Var2;
    }

    return config;
}

typedef struct {
    const char* ip;
    int port;
    int slave_id;
    int is_first_slave;
    int dnp3_analog_index;
    int dnp3_status_index;
} SlaveConfig;

void UpdateDNP3Values(UpdateBuilder& builder, const vector<SlaveState>& slave_states) {
    for (size_t i = 0; i < slave_states.size(); i++) {
        const SlaveState& state = slave_states[i];
        builder.Update(Analog((state.failure_count >= state.max_failures_before_zero) ? 0 : state.analog_value), i);
        bool connection_failed = !state.connection_status;
        builder.Update(Binary(connection_failed, (state.connection_status != state.last_connection_state) ? Flags(0x01) : Flags()), i);
    }
}

void HandleCommunicationFailure(SlaveState& state) {
    state.connection_status = false;
    state.failure_count++;
    if (state.failure_count >= state.max_failures_before_zero) {
        state.analog_value = 0;
    }
}

// Função para converter registros Modbus em int32_t considerando sinal
int32_t modbusRegistersToInt32(uint16_t* regs, bool is_signed) {
    int32_t value = (regs[0] << 16) | regs[1];
    if (is_signed && (value & 0x80000000)) {
        value |= 0xFFFFFFFF00000000; // Estende o sinal para 64 bits
    }
    return value;
}

int main() {
    const auto logLevels = levels::NORMAL | levels::NOTHING;
    DNP3Manager manager(1, ConsoleLogger::Create());

    auto channel = manager.AddTCPServer("server", logLevels, ServerAcceptMode::CloseExisting,
                                      IPEndpoint("10.1.1.223", 20000),
                                      PrintingChannelListener::Create());

    OutstationStackConfig stackConfig(ConfigureDatabase());
    stackConfig.outstation.eventBufferConfig = EventBufferConfig::AllTypes(10);
    stackConfig.outstation.params.allowUnsolicited = true;
    stackConfig.link.LocalAddr = 2;
    stackConfig.link.RemoteAddr = 1;

    auto outstation = channel->AddOutstation("outstation", SuccessCommandHandler::Create(),
                                           DefaultOutstationApplication::Create(), stackConfig);
    outstation->Enable();

    SlaveConfig slaves[NUM_SLAVES] = {
        {"10.1.1.116", 502, 1, 1, 0, 0},  
        {"10.1.1.41", 502, 1, 0, 1, 1},
        {"10.1.1.42", 502, 1, 0, 2, 2}
    };

    const int HOLDING_REG_OFFSET = 23322;
    const int INPUT_REG_OFFSET = 37;
    vector<SlaveState> slave_states(NUM_SLAVES);
    uint16_t tab_reg[max(NUM_HOLDING_REGISTERS, NUM_INPUT_REGISTERS)];

    while (true) {
        UpdateBuilder builder;

        for (int i = 0; i < NUM_SLAVES; ++i) {
            modbus_t* ctx = modbus_new_tcp(slaves[i].ip, slaves[i].port);
            if (!ctx) {
                cerr << "Erro ao criar contexto Modbus para " << slaves[i].ip << endl;
                HandleCommunicationFailure(slave_states[i]);
                continue;
            }

            modbus_set_slave(ctx, slaves[i].slave_id);
            modbus_set_response_timeout(ctx, 3, 0);

            if (modbus_connect(ctx) == -1) {
                cerr << "Falha na conexão com " << slaves[i].ip << endl;
                modbus_free(ctx);
                HandleCommunicationFailure(slave_states[i]);
                continue;
            }

            bool read_success = false;
            try {
                if (slaves[i].is_first_slave) {
                    int rc = modbus_read_registers(ctx, HOLDING_REG_OFFSET, NUM_HOLDING_REGISTERS, tab_reg);
                    if (rc == -1) throw modbus_strerror(errno);
                    
                    // Usa a nova função de conversão que trata valores negativos
                    slave_states[i].analog_value = modbusRegistersToInt32(tab_reg, true);
                    cout << "Slave " << i << " (Holding): " << slave_states[i].analog_value << endl;
                } else {
                    int rc = modbus_read_input_registers(ctx, INPUT_REG_OFFSET, NUM_INPUT_REGISTERS, tab_reg);
                    if (rc == -1) throw modbus_strerror(errno);
                    
                    // Para input registers de 16 bits, trata o sinal se necessário
                    slave_states[i].analog_value = static_cast<int16_t>(tab_reg[0]);
                    cout << "Slave " << i << " (Input): " << slave_states[i].analog_value << endl;
                }
                read_success = true;
            } catch (const char* e) {
                cerr << "Erro na leitura do slave " << i << ": " << e << endl;
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

        UpdateDNP3Values(builder, slave_states);
        outstation->Apply(builder.Build());
        this_thread::sleep_for(chrono::seconds(1));
    }

    return 0;
}