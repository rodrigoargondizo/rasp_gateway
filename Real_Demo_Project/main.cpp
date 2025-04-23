// Inclui bibliotecas necessarias
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
#include <mutex>
#include <atomic>

using namespace std;
using namespace opendnp3;

// Constantes de configuracao do sistema
#define NUM_SLAVES 3             // Numero de dispositivos Modbus (slaves) para monitorar
#define NUM_HOLDING_REGISTERS 2  // Numero de registros holding para leitura (32 bits)
#define NUM_INPUT_REGISTERS 1    // Numero de registros input para leitura (16 bits)

// Mutexes para sincronizacao
mutex data_mutex;                // Protege dados compartilhados entre threads
mutex dnp3_update_mutex;         // Protege atualizacoes DNP3

// Estrutura para armazenar estado de cada slave Modbus
struct SlaveState {
    int32_t analog_value = 0;    // Valor analogico atual do slave
    bool connection_status = false; // Status atual da conexao
    bool last_connection_state = true;  // Status anterior da conexao
    bool connection_changed = true;     // Flag indicando mudanca no status
    int failure_count = 0;       // Contador de falhas consecutivas
    const int max_failures_before_zero = 5; // Max falhas antes de zerar valor
    chrono::system_clock::time_point last_change_time; // Timestamp da ultima mudanca
};

// Funcao para configurar banco de dados DNP3
DatabaseConfig ConfigureDatabase() {
    DatabaseConfig config;

    // Configura entradas analogicas e binarias para cada slave
    for(int i = 0; i < NUM_SLAVES; i++) {
        // Configuracao entrada analogica
        config.analog_input[i] = AnalogConfig();
        config.analog_input[i].clazz = PointClass::Class2;
        config.analog_input[i].svariation = (i == 0) ? StaticAnalogVariation::Group30Var1 
                                                   : StaticAnalogVariation::Group30Var2;

        // Configuracao entrada binaria
        config.binary_input[i] = BinaryConfig();
        config.binary_input[i].clazz = PointClass::Class1;
        config.binary_input[i].svariation = StaticBinaryVariation::Group1Var2;
        config.binary_input[i].evariation = EventBinaryVariation::Group2Var2;
    }

    return config;
}

// Estrutura para configuracao de slave Modbus
typedef struct {
    const char* ip;             // Endereco IP do slave
    int port;                   // Porta do slave
    int slave_id;               // ID do slave
    int is_first_slave;         // Flag para primeiro slave (tipo de registro diferente)
    int dnp3_analog_index;      // Indice DNP3 para valor analogico
    int dnp3_status_index;      // Indice DNP3 para status
} SlaveConfig;

// Atualiza valores DNP3 baseado no estado do slave
void UpdateDNP3Values(UpdateBuilder& builder, SlaveState& state, int index) {
    // Atualiza valor analogico (zera se muitas falhas)
    builder.Update(Analog((state.failure_count >= state.max_failures_before_zero) ? 0 : state.analog_value), index);
    
    // Atualiza status da conexao se houve mudanca
    if(state.connection_changed || state.last_change_time == chrono::system_clock::time_point()) {
        bool connection_failed = !state.connection_status;
        builder.Update(Binary(connection_failed, Flags(0x01)), index);
    }
}

// Trata falha de comunicacao com slave
void HandleCommunicationFailure(SlaveState& state) {
    bool previous_status = state.connection_status;
    state.connection_status = false;
    state.connection_changed = (state.connection_status != previous_status);
    state.failure_count++;
    
    // Zera valor analogico se muitas falhas
    if (state.failure_count >= state.max_failures_before_zero) {
        state.analog_value = 0;
    }

    // Registra hora da mudanca se status alterou
    if(state.connection_changed) {
        state.last_change_time = chrono::system_clock::now();
    }
}

// Trata comunicacao bem sucedida com slave
void HandleCommunicationSuccess(SlaveState& state) {
    bool previous_status = state.connection_status;
    state.connection_status = true;
    state.connection_changed = (state.connection_status != previous_status);
    state.failure_count = 0;

    // Registra hora da mudanca se status alterou
    if(state.connection_changed) {
        state.last_change_time = chrono::system_clock::now();
    }
}

// Converte registros Modbus para inteiro 32 bits
int32_t modbusRegistersToInt32(uint16_t* regs, bool is_signed) {
    int32_t value = (regs[0] << 16) | regs[1];
    if (is_signed && (value & 0x80000000)) {
        value |= 0xFFFFFFFF00000000;
    }
    return value;
}

// Funcao de thread para monitorar um slave Modbus
void PollSlave(int slave_index, SlaveConfig* slaves, vector<SlaveState>* slave_states, shared_ptr<opendnp3::IOutstation> outstation) {
    const int HOLDING_REG_OFFSET = 23322;  // Offset holding register para primeiro slave
    const int INPUT_REG_OFFSET = 37;       // Offset input register para outros slaves
    uint16_t tab_reg[max(NUM_HOLDING_REGISTERS, NUM_INPUT_REGISTERS)];

    // Atraso inicial escalonado para evitar congestionamento
    this_thread::sleep_for(chrono::milliseconds(100 * slave_index));

    while (true) {
        bool read_success = false;
        
        // Cria novo contexto Modbus para cada tentativa
        modbus_t* ctx = modbus_new_tcp(slaves[slave_index].ip, slaves[slave_index].port);
        if (ctx) {
            modbus_set_slave(ctx, slaves[slave_index].slave_id);
            modbus_set_response_timeout(ctx, 1, 0);
            modbus_set_byte_timeout(ctx, 0, 500000);

            if (modbus_connect(ctx) != -1) {
                try {
                    // Primeiro slave usa holding registers, outros usam input registers
                    if (slaves[slave_index].is_first_slave) {
                        int rc = modbus_read_registers(ctx, HOLDING_REG_OFFSET, NUM_HOLDING_REGISTERS, tab_reg);
                        if (rc != -1) {
                            lock_guard<mutex> lock(data_mutex);
                            (*slave_states)[slave_index].analog_value = modbusRegistersToInt32(tab_reg, true);
                            cout << "Slave " << slave_index << " (Holding): " << (*slave_states)[slave_index].analog_value << endl;
                            HandleCommunicationSuccess((*slave_states)[slave_index]);
                            read_success = true;
                        }
                    } else {
                        int rc = modbus_read_input_registers(ctx, INPUT_REG_OFFSET, NUM_INPUT_REGISTERS, tab_reg);
                        if (rc != -1) {
                            lock_guard<mutex> lock(data_mutex);
                            (*slave_states)[slave_index].analog_value = static_cast<int16_t>(tab_reg[0]);
                            cout << "Slave " << slave_index << " (Input): " << (*slave_states)[slave_index].analog_value << endl;
                            HandleCommunicationSuccess((*slave_states)[slave_index]);
                            read_success = true;
                        }
                    }
                } catch (const char* e) {
                    cerr << "Erro lendo slave " << slave_index << ": " << e << endl;
                }
                modbus_close(ctx);
            }
            modbus_free(ctx);
        }

        if (!read_success) {
            lock_guard<mutex> lock(data_mutex);
            cerr << "Falha comunicacao com slave " << slave_index << endl;
            HandleCommunicationFailure((*slave_states)[slave_index]);
        }

        // Atualiza valores DNP3
        {
            lock_guard<mutex> dnp_lock(dnp3_update_mutex);
            UpdateBuilder builder;
            
            {
                lock_guard<mutex> data_lock(data_mutex);
                UpdateDNP3Values(builder, (*slave_states)[slave_index], slave_index);
                
                // Reseta flag de mudanca se foi enviada
                if ((*slave_states)[slave_index].connection_changed) {
                    auto now = chrono::system_clock::now();
                    auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - (*slave_states)[slave_index].last_change_time).count();
                    if (elapsed > 100) { // Espera pelo menos 100ms antes de resetar
                        (*slave_states)[slave_index].connection_changed = false;
                    }
                }
            }
            
            outstation->Apply(builder.Build());
        }

        this_thread::sleep_for(chrono::seconds(1));
    }
}

int main() {
    // Inicializa gerenciador DNP3 com logging
    const auto logLevels = levels::NORMAL | levels::NOTHING;
    DNP3Manager manager(1, ConsoleLogger::Create());

    // Cria canal TCP server
    auto channel = manager.AddTCPServer("server", logLevels, ServerAcceptMode::CloseExisting,
                                     IPEndpoint("10.1.1.223", 20000),
                                     PrintingChannelListener::Create());

    // Configura stack outstation DNP3
    OutstationStackConfig stackConfig(ConfigureDatabase());
    stackConfig.outstation.eventBufferConfig = EventBufferConfig::AllTypes(100);
    stackConfig.outstation.params.allowUnsolicited = true;
    stackConfig.link.LocalAddr = 2;
    stackConfig.link.RemoteAddr = 1;

    // Cria e ativa outstation
    auto outstation = channel->AddOutstation("outstation", SuccessCommandHandler::Create(),
                                          DefaultOutstationApplication::Create(), stackConfig);
    outstation->Enable();

    /*
    * Configuracao dos dispositivos Modbus (slaves) que serao monitorados:
    * Cada entrada contem:
    * 1. IP do dispositivo
    * 2. Porta Modbus (normalmente 502)
    * 3. ID do escravo Modbus
    * 4. Tipo de registro (1 = holding registers, 0 = input registers)
    * 5. Indice do ponto analogico no DNP3
    * 6. Indice do ponto binario (status) no DNP3
    *
    * O primeiro slave usa holding registers (32 bits) enquanto os demais usam input registers (16 bits)
    */
    SlaveConfig slaves[NUM_SLAVES] = {
        {"10.1.1.116", 502, 1, 1, 0, 0},
        {"10.1.1.41", 502, 1, 0, 1, 1},
        {"10.1.1.42", 502, 1, 0, 2, 2}
    };

    vector<SlaveState> slave_states(NUM_SLAVES);
    
    // Envia status inicial desconectado
    {
        UpdateBuilder builder;
        for (int i = 0; i < NUM_SLAVES; ++i) {
            builder.Update(Binary(true, Flags(0x01)), i); // Status inicial = falha
            slave_states[i].last_change_time = chrono::system_clock::now();
        }
        outstation->Apply(builder.Build());
    }

    // Cria threads para monitorar cada slave
    vector<thread> threads;

    for (int i = 0; i < NUM_SLAVES; ++i) {
        threads.emplace_back(PollSlave, i, slaves, &slave_states, outstation);
    }

    // Desvincula threads (elas rodam indefinidamente)
    for (auto& t : threads) {
        t.detach();
    }

    // Mantem thread principal rodando
    while(true) {
        this_thread::sleep_for(chrono::hours(1));
    }

    return 0;
}