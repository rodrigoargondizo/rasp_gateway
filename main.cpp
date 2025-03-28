#include <opendnp3/ConsoleLogger.h>
#include <opendnp3/DNP3Manager.h>
#include <opendnp3/channel/PrintingChannelListener.h>
#include <opendnp3/logging/LogLevels.h>
#include <opendnp3/outstation/DefaultOutstationApplication.h>
#include <opendnp3/outstation/IUpdateHandler.h>
#include <opendnp3/outstation/SimpleCommandHandler.h>
#include <opendnp3/outstation/UpdateBuilder.h>
#include <modbus/modbus.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

using namespace std;
using namespace opendnp3;

// Configuração do banco de dados DNP3
DatabaseConfig ConfigureDatabase() {
    DatabaseConfig config;
    
    // Ponto analógico (valor do escravo Modbus)
    config.analog_input[0] = AnalogConfig();
    config.analog_input[0].clazz = PointClass::Class2;
    config.analog_input[0].svariation = StaticAnalogVariation::Group30Var2;
    
    // Ponto binário (status da conexão) - índice 0
    config.binary_input[0] = BinaryConfig();
    config.binary_input[0].clazz = PointClass::Class1;
    config.binary_input[0].svariation = StaticBinaryVariation::Group1Var2;
    
    return config;
}

struct State {
    // Para o valor analógico
    int16_t analog = 0;
    int16_t last_valid_value = 0;
    
    // Para o status de conexão
    bool modbus_connected = false;
    bool last_connection_state = false;
    int failure_count = 0;
    const int max_failures_before_zero = 5;
};

void AddUpdates(UpdateBuilder& builder, State& state) {
    // Atualiza o valor analógico (índice 0)
    if (state.failure_count >= state.max_failures_before_zero) {
        builder.Update(Analog(0), 0);
    } else {
        builder.Update(Analog(state.last_valid_value), 0);
    }
    
    // Atualiza o status binário (índice 0)
    // true = falha de conexão, false = conexão OK
    bool connection_failed = !state.modbus_connected;
    builder.Update(Binary(connection_failed), 0);
    
    // Se o status mudou, envia como evento com flag de qualidade
    if (state.modbus_connected != state.last_connection_state) {
        builder.Update(Binary(connection_failed, Flags(0x01)), 0);
        state.last_connection_state = state.modbus_connected;
    }
}

bool TryModbusReconnect(modbus_t* ctx, const char* ip, int port, int slave_id, State& state) {
    cout << "Tentando reconectar ao Modbus..." << endl;
    
    if (state.modbus_connected) {
        modbus_close(ctx);
        state.modbus_connected = false;
    }

    if (ctx == nullptr) {
        ctx = modbus_new_tcp(ip, port);
        if (ctx == nullptr) {
            cerr << "Falha ao criar novo contexto Modbus" << endl;
            return false;
        }
        modbus_set_response_timeout(ctx, 1, 0);
        modbus_set_byte_timeout(ctx, 1, 0);
    }

    if (modbus_set_slave(ctx, slave_id) == -1) {
        cerr << "Erro ao configurar ID do escravo: " << modbus_strerror(errno) << endl;
        modbus_free(ctx);
        return false;
    }

    if (modbus_connect(ctx) == -1) {
        cerr << "Falha na reconexão: " << modbus_strerror(errno) << endl;
        return false;
    }

    state.modbus_connected = true;
    cout << "Conexão Modbus restabelecida!" << endl;
    return true;
}

bool ReadModbusValues(modbus_t* ctx, const char* ip, int port, int slave_id, State& state) {
    uint16_t tab_reg[1];
    
    if (!state.modbus_connected) {
        if (!TryModbusReconnect(ctx, ip, port, slave_id, state)) {
            state.failure_count++;
            return false;
        }
    }

    modbus_flush(ctx);
    int rc = modbus_read_registers(ctx, 0, 1, tab_reg);
    
    if (rc == -1) {
        cerr << "Erro na leitura: " << modbus_strerror(errno) << endl;
        modbus_close(ctx);
        state.modbus_connected = false;
        state.failure_count++;
        return false;
    }

    state.analog = static_cast<int16_t>(tab_reg[0]);
    state.last_valid_value = state.analog;
    state.failure_count = 0;
    return true;
}

volatile sig_atomic_t shutdown_flag = 0;

void signal_handler(int signal) {
    shutdown_flag = 1;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Configuração DNP3
    DNP3Manager manager(1, ConsoleLogger::Create());
    auto channel = manager.AddTCPServer("server", levels::NORMAL, 
                                      ServerAcceptMode::CloseExisting,
                                      IPEndpoint("192.168.100.176", 20000),
                                      PrintingChannelListener::Create());

    OutstationStackConfig config(ConfigureDatabase());
    config.outstation.eventBufferConfig = EventBufferConfig::AllTypes(10);
    config.outstation.params.allowUnsolicited = true;
    config.link.LocalAddr = 2;
    config.link.RemoteAddr = 1;

    auto outstation = channel->AddOutstation("outstation", 
                                           SuccessCommandHandler::Create(),
                                           DefaultOutstationApplication::Create(), 
                                           config);
    outstation->Enable();

    // Configuração Modbus
    const char* modbus_ip = "192.168.100.120";
    const int modbus_port = 502;
    const int modbus_slave_id = 1;
    
    modbus_t* ctx = modbus_new_tcp(modbus_ip, modbus_port);
    modbus_set_response_timeout(ctx, 1, 0);
    modbus_set_byte_timeout(ctx, 1, 0);

    State state;
    state.modbus_connected = false;
    state.last_connection_state = false;

    // Loop principal
    while (!shutdown_flag) {
        bool read_success = ReadModbusValues(ctx, modbus_ip, modbus_port, modbus_slave_id, state);

        UpdateBuilder builder;
        AddUpdates(builder, state);
        outstation->Apply(builder.Build());

        // Log de status
        if (!read_success) {
            if (state.failure_count >= state.max_failures_before_zero) {
                cout << "Falha prolongada - Enviando 0 (Status: FALHA)" << endl;
            } else {
                cout << "Falha temporária - Último valor válido: " 
                     << state.last_valid_value << " (Status: FALHA)" << endl;
            }
        } else {
            cout << "Leitura OK - Valor atual: " << state.analog 
                 << " (Status: CONECTADO)" << endl;
        }

        this_thread::sleep_for(chrono::seconds(1));
    }

    // Limpeza
    if (state.modbus_connected) {
        modbus_close(ctx);
    }
    modbus_free(ctx);

    cout << "Encerrando programa..." << endl;
    return 0;
}