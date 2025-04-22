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

// Definições de constantes para configuração do sistema
#define NUM_SLAVES 3             // Número de dispositivos Modbus (slaves) a serem monitorados
#define NUM_HOLDING_REGISTERS 2  // Número de registros holding para leitura (32 bits)
#define NUM_INPUT_REGISTERS 1    // Número de registros input para leitura (16 bits)

// Estrutura para armazenar o estado de cada slave Modbus
struct SlaveState {
    int32_t analog_value = 0;            // Valor analógico lido do slave
    bool connection_status = false;       // Status atual da conexão
    bool last_connection_state = false;   // Último estado da conexão (para detecção de mudanças)
    int failure_count = 0;                // Contador de falhas consecutivas
    const int max_failures_before_zero = 5; // Número máximo de falhas antes de zerar o valor
};

// Configuração do banco de dados DNP3
DatabaseConfig ConfigureDatabase() {
    DatabaseConfig config;

    // Configura os pontos para cada slave
    for(int i = 0; i < NUM_SLAVES; i++) {
        // Configuração de entradas analógicas
        config.analog_input[i] = AnalogConfig();
        config.analog_input[i].clazz = PointClass::Class2;  // Classe do ponto (prioridade)
        // Variação estática diferente para o primeiro slave
        config.analog_input[i].svariation = (i == 0) ? StaticAnalogVariation::Group30Var1 
                                                   : StaticAnalogVariation::Group30Var2;

        // Configuração de entradas binárias (status de conexão)
        config.binary_input[i] = BinaryConfig();
        config.binary_input[i].clazz = PointClass::Class1;  // Classe de prioridade mais alta
        config.binary_input[i].svariation = StaticBinaryVariation::Group1Var2;
    }

    return config;
}

// Estrutura de configuração para cada slave Modbus
typedef struct {
    const char* ip;             // Endereço IP do slave
    int port;                   // Porta Modbus (normalmente 502)
    int slave_id;               // ID do dispositivo Modbus
    int is_first_slave;         // Flag para identificar o primeiro slave (usa holding registers)
    int dnp3_analog_index;      // Índice do ponto analógico no DNP3
    int dnp3_status_index;      // Índice do ponto binário (status) no DNP3
} SlaveConfig;

// Atualiza os valores no builder DNP3 com base nos estados dos slaves
void UpdateDNP3Values(UpdateBuilder& builder, const vector<SlaveState>& slave_states) {
    for (size_t i = 0; i < slave_states.size(); i++) {
        const SlaveState& state = slave_states[i];
        // Atualiza valor analógico (zera após muitas falhas)
        builder.Update(Analog((state.failure_count >= state.max_failures_before_zero) ? 0 : state.analog_value), i);
        // Atualiza status binário (falha de conexão)
        bool connection_failed = !state.connection_status;
        // Sinaliza mudança de estado se necessário
        builder.Update(Binary(connection_failed, (state.connection_status != state.last_connection_state) ? Flags(0x01) : Flags()), i);
    }
}

// Manipula falhas de comunicação com um slave
void HandleCommunicationFailure(SlaveState& state) {
    state.connection_status = false;
    state.failure_count++;
    // Zera o valor após muitas falhas consecutivas
    if (state.failure_count >= state.max_failures_before_zero) {
        state.analog_value = 0;
    }
}

// Converte registros Modbus em um valor inteiro de 32 bits com tratamento de sinal
int32_t modbusRegistersToInt32(uint16_t* regs, bool is_signed) {
    int32_t value = (regs[0] << 16) | regs[1];
    // Trata valor negativo se necessário
    if (is_signed && (value & 0x80000000)) {
        value |= 0xFFFFFFFF00000000; // Estende o sinal para 64 bits
    }
    return value;
}

int main() {
    // Configuração inicial do DNP3
    const auto logLevels = levels::NORMAL | levels::NOTHING;
    DNP3Manager manager(1, ConsoleLogger::Create());

    // Cria canal TCP para comunicação DNP3
    auto channel = manager.AddTCPServer("server", logLevels, ServerAcceptMode::CloseExisting,
                                      IPEndpoint("10.1.1.223", 20000),
                                      PrintingChannelListener::Create());

    // Configuração da pilha DNP3
    OutstationStackConfig stackConfig(ConfigureDatabase());
    stackConfig.outstation.eventBufferConfig = EventBufferConfig::AllTypes(10);  // Buffer de eventos
    stackConfig.outstation.params.allowUnsolicited = true;  // Permite mensagens não solicitadas
    stackConfig.link.LocalAddr = 2;    // Endereço local DNP3
    stackConfig.link.RemoteAddr = 1;   // Endereço remoto DNP3

    // Cria a outstation DNP3
    auto outstation = channel->AddOutstation("outstation", SuccessCommandHandler::Create(),
                                           DefaultOutstationApplication::Create(), stackConfig);
    outstation->Enable();  // Habilita a outstation
	
/*
 * Configuração dos dispositivos Modbus (slaves) que serão monitorados:
 * Cada entrada contém:
 * 1. IP do dispositivo
 * 2. Porta Modbus (normalmente 502)
 * 3. ID do escravo Modbus
 * 4. Tipo de registro (1 = holding registers, 0 = input registers)
 * 5. Índice do ponto analógico no DNP3
 * 6. Índice do ponto binário (status) no DNP3
 *
 * O primeiro slave usa holding registers (32 bits) enquanto os demais usam input registers (16 bits)
 */
    SlaveConfig slaves[NUM_SLAVES] = {
        {"10.1.1.116", 502, 1, 1, 0, 0},  // Primeiro slave (holding registers)
        {"10.1.1.41", 502, 1, 0, 1, 1},    // Segundo slave (input registers)
        {"10.1.1.42", 502, 1, 0, 2, 2}     // Terceiro slave (input registers)
    };

    // Offsets dos registros Modbus
    const int HOLDING_REG_OFFSET = 23322;  // Offset para holding registers (primeiro slave)
    const int INPUT_REG_OFFSET = 37;       // Offset para input registers (outros slaves)
    
    vector<SlaveState> slave_states(NUM_SLAVES);  // Estados dos slaves
    uint16_t tab_reg[max(NUM_HOLDING_REGISTERS, NUM_INPUT_REGISTERS)];  // Buffer para leitura Modbus

    // Loop principal
    while (true) {
        UpdateBuilder builder;  // Construtor de atualizações DNP3

        // Itera sobre todos os slaves Modbus
        for (int i = 0; i < NUM_SLAVES; ++i) {
            // Cria contexto Modbus TCP
            modbus_t* ctx = modbus_new_tcp(slaves[i].ip, slaves[i].port);
            if (!ctx) {
                cerr << "Erro ao criar contexto Modbus para " << slaves[i].ip << endl;
                HandleCommunicationFailure(slave_states[i]);
                continue;
            }

            // Configura contexto Modbus
            modbus_set_slave(ctx, slaves[i].slave_id);
            modbus_set_response_timeout(ctx, 3, 0);  // Timeout de 3 segundos

            // Tenta conectar ao slave
            if (modbus_connect(ctx) == -1) {
                cerr << "Falha na conexão com " << slaves[i].ip << endl;
                modbus_free(ctx);
                HandleCommunicationFailure(slave_states[i]);
                continue;
            }

            bool read_success = false;
            try {
                // Lê registros diferentes dependendo se é o primeiro slave ou não
                if (slaves[i].is_first_slave) {
                    // Lê holding registers (32 bits)
                    int rc = modbus_read_registers(ctx, HOLDING_REG_OFFSET, NUM_HOLDING_REGISTERS, tab_reg);
                    if (rc == -1) throw modbus_strerror(errno);
                    
                    // Converte para int32 com tratamento de sinal
                    slave_states[i].analog_value = modbusRegistersToInt32(tab_reg, true);
                    cout << "Slave " << i << " (Holding): " << slave_states[i].analog_value << endl;
                } else {
                    // Lê input registers (16 bits)
                    int rc = modbus_read_input_registers(ctx, INPUT_REG_OFFSET, NUM_INPUT_REGISTERS, tab_reg);
                    if (rc == -1) throw modbus_strerror(errno);
                    
                    // Converte para int16 com sinal
                    slave_states[i].analog_value = static_cast<int16_t>(tab_reg[0]);
                    cout << "Slave " << i << " (Input): " << slave_states[i].analog_value << endl;
                }
                read_success = true;
            } catch (const char* e) {
                cerr << "Erro na leitura do slave " << i << ": " << e << endl;
            }

            // Atualiza estado da conexão
            if (read_success) {
                slave_states[i].connection_status = true;
                slave_states[i].failure_count = 0;
            } else {
                HandleCommunicationFailure(slave_states[i]);
            }

            // Libera recursos Modbus
            modbus_close(ctx);
            modbus_free(ctx);
        }

        // Aplica atualizações no DNP3
        UpdateDNP3Values(builder, slave_states);
        outstation->Apply(builder.Build());
        
        // Espera 1 segundo antes da próxima iteração
        this_thread::sleep_for(chrono::seconds(1));
    }

    return 0;
}