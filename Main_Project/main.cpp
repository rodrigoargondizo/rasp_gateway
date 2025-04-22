// Bibliotecas necessarias para comunicacao DNP3 e Modbus
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
#include <modbus/modbus.h>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

using namespace std;
using namespace opendnp3;

// Estrutura para armazenar o estado da aplicacao
struct State {
    int16_t analog = 0;                     // Valor analogico de entrada
    bool led_status = 0;                     // Estado do LED
    bool button_status = 0;                  // Estado do botao
    int16_t last_valid_value = 0;            // Ultimo valor analogico valido
    const int COIL_LIGAR = 0;                // Endereco da bobina para ligar
    const int COIL_DESLIGAR = 1;             // Endereco da bobina para desligar
    const int COIL_STATUS_LED = 2;           // Endereco da bobina do LED
    const int COIL_STATUS_BUTTON = 3;        // Endereco da bobina do botao
        
    bool modbus_connected = false;           // Status da conexao Modbus
    bool last_connection_state = false;      // Estado anterior da conexao
    int failure_count = 0;                   // Contador de falhas consecutivas
    const int max_failures_before_zero = 5;  // Maximo de falhas antes de enviar zero
};

// Configura os pontos da base de dados DNP3
DatabaseConfig ConfigureDatabase()
{
    DatabaseConfig config;

    // Configura ponto analogico de entrada
    config.analog_input[0] = AnalogConfig();
    config.analog_input[0].clazz = PointClass::Class2;
    config.analog_input[0].svariation = StaticAnalogVariation::Group30Var2;

    // Configura pontos binarios de entrada
    config.binary_input[0] = BinaryConfig();
    config.binary_input[0].clazz = PointClass::Class1;
    config.binary_input[0].svariation = StaticBinaryVariation::Group1Var2;

    config.binary_input[1] = BinaryConfig();
    config.binary_input[1].clazz = PointClass::Class1;
    config.binary_input[1].svariation = StaticBinaryVariation::Group1Var2;

    config.binary_input[2] = BinaryConfig();
    config.binary_input[2].clazz = PointClass::Class1;
    config.binary_input[2].svariation = StaticBinaryVariation::Group1Var2;

    return config;
}

// Adiciona atualizacoes ao outstation DNP3
void AddUpdates(UpdateBuilder& builder, State& state) {
    // Atualiza valor analogico
    if (state.failure_count >= state.max_failures_before_zero) {
        builder.Update(Analog(0), 0);
    } else {
        builder.Update(Analog(state.last_valid_value), 0);
    }
    
    // Atualiza status da conexao
    bool connection_failed = !state.modbus_connected;
    builder.Update(Binary(connection_failed), 0);
    
    // Atualiza flag de status se houve mudanca
    if (state.modbus_connected != state.last_connection_state) {
        builder.Update(Binary(connection_failed, Flags(0x01)), 0);
        state.last_connection_state = state.modbus_connected;
    }

    // Atualiza entradas binarias
    builder.Update(Binary(state.led_status), 1);
    builder.Update(Binary(state.button_status), 2);
}

// Tenta reconectar ao dispositivo Modbus
bool TryModbusReconnect(modbus_t* ctx, const char* ip, int port, int slave_id, State& state) {
    cout << "Tentando reconectar ao Modbus..." << endl;
    
    // Fecha conexao existente se houver
    if (state.modbus_connected) {
        modbus_close(ctx);
        state.modbus_connected = false;
    }

    // Cria novo contexto se necessario
    if (ctx == nullptr) {
        ctx = modbus_new_tcp(ip, port);
        if (ctx == nullptr) {
            cerr << "Falha ao criar novo contexto Modbus" << endl;
            return false;
        }
        modbus_set_response_timeout(ctx, 1, 0);
        modbus_set_byte_timeout(ctx, 1, 0);
    }

    // Configura ID do escravo
    if (modbus_set_slave(ctx, slave_id) == -1) {
        cerr << "Erro ao configurar ID do escravo: " << modbus_strerror(errno) << endl;
        modbus_free(ctx);
        return false;
    }

    // Tenta conexao
    if (modbus_connect(ctx) == -1) {
        cerr << "Falha na reconexao: " << modbus_strerror(errno) << endl;
        return false;
    }

    state.modbus_connected = true;
    cout << "Conexao Modbus restabelecida!" << endl;
    return true;
}

// Le valores do dispositivo Modbus
bool ReadModbusValues(modbus_t* ctx, const char* ip, int port, int slave_id, State& state) {
    uint16_t tab_reg[1];
    uint8_t led_status, button_status;
    
    // Tenta reconectar se nao estiver conectado
    if (!state.modbus_connected) {
        if (!TryModbusReconnect(ctx, ip, port, slave_id, state)) {
            state.failure_count++;
            return false;
        }
    }

    // Le registro analogico
    modbus_flush(ctx);
    int rc = modbus_read_registers(ctx, 0, 1, tab_reg);
    
    if (rc == -1) {
        cerr << "Erro na leitura do potenciometro: " << modbus_strerror(errno) << endl;
        modbus_close(ctx);
        state.modbus_connected = false;
        state.failure_count++;
        return false;
    }

    // Le status do LED
    rc = modbus_read_bits(ctx, state.COIL_STATUS_LED, 1, &led_status);
    
    if (rc == -1) {
        cerr << "Erro na leitura do status do LED: " << modbus_strerror(errno) << endl;
        modbus_close(ctx);
        state.modbus_connected = false;
        state.failure_count++;
        return false;
    }

    // Le status do botao
    rc = modbus_read_bits(ctx, state.COIL_STATUS_BUTTON, 1, &button_status);
    
    if (rc == -1) {
        cerr << "Erro na leitura do coil do Botao: " << modbus_strerror(errno) << endl;
        modbus_close(ctx);
        state.modbus_connected = false;
        state.failure_count++;
        return false;
    }

    // Atualiza estado com novos valores
    state.analog = static_cast<int16_t>(tab_reg[0]);
    state.last_valid_value = state.analog;
    state.led_status = (led_status == 1);
    state.button_status = !(button_status == 1);
    state.failure_count = 0;
    return true;
}

// Flag para tratamento de sinal
volatile sig_atomic_t shutdown_flag = 0;

// Funcao para tratamento de sinal
void signal_handler(int signal) {
    shutdown_flag = 1;
}

// Liga dispositivo via Modbus
bool LigarDispositivo(modbus_t* ctx, const State& state) {
    if (modbus_write_bit(ctx, state.COIL_LIGAR, true) == -1) {
        std::cerr << "Erro ao ligar: " << modbus_strerror(errno) << std::endl;
        return false;
    }
    modbus_flush(ctx);
    return true;
}

// Desliga dispositivo via Modbus
bool DesligarDispositivo(modbus_t* ctx, const State& state) {
    if (modbus_write_bit(ctx, state.COIL_DESLIGAR, true) == -1) {
        std::cerr << "Erro ao desligar: " << modbus_strerror(errno) << std::endl;
        return false;
    }
    modbus_flush(ctx);
    return true;
}

// Manipulador de comandos personalizado para DNP3
class DirectOperateOnlyHandler : public SimpleCommandHandler {
private:
    modbus_t* ctx;
    State state;

public:
    DirectOperateOnlyHandler(modbus_t* modbus_ctx, const State& initialState)
        : SimpleCommandHandler(CommandStatus::SUCCESS),
          ctx(modbus_ctx),
          state(initialState) {}

    // Manipula comandos de controle
    CommandStatus Operate(const ControlRelayOutputBlock& command, 
                          uint16_t index,
                          IUpdateHandler& handler,
                          OperateType opType) override {
        if (opType == OperateType::DirectOperate) {
            switch (index) {
                case 0:
                std::cout << "Direct Operate: LED ON" << std::endl;
                    return LigarDispositivo(ctx, state) ? 
                           CommandStatus::SUCCESS : 
                           CommandStatus::HARDWARE_ERROR;
                case 1:
                std::cout << "Direct Operate: LED OFF" << std::endl;
                    return DesligarDispositivo(ctx, state) ? 
                           CommandStatus::SUCCESS : 
                           CommandStatus::HARDWARE_ERROR;
                default:
                    return CommandStatus::NOT_SUPPORTED;
            }
        }
        return CommandStatus::NOT_SUPPORTED;
    }
};

int main() {
    // Configura handlers de sinal
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Parametros de conexao Modbus
    const char* modbus_ip = "192.168.100.120";
    const int modbus_port = 502;
    const int modbus_slave_id = 1;

    // Cria contexto Modbus
    modbus_t* ctx = modbus_new_tcp(modbus_ip, modbus_port);

    // Tenta conexao inicial
    while (!shutdown_flag) {
        if (modbus_connect(ctx) == -1) {
            std::cerr << "Erro ao conectar ao dispositivo: " << modbus_strerror(errno) << std::endl;
            std::cerr << "Tentando novamente em 5 segundos..." << std::endl;
            
            std::this_thread::sleep_for(std::chrono::seconds(5));
        } else {
            std::cout << "Conexao Modbus estabelecida com sucesso!" << std::endl;
            break;
        }
    }

    // Configura ID do escravo Modbus
    if (modbus_set_slave(ctx, 1) == -1)
    {
        std::cerr << "Erro ao configurar ID do escravo: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return -1;
    }

    // Verifica conexao
    if (modbus_connect(ctx) == -1)
    {
        std::cerr << "Erro ao conectar ao dispositivo: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return -1;
    }
    
    // Inicializa estado da aplicacao
    State state;

    // Configura niveis de log DNP3
    const auto logLevels = levels::NORMAL | levels::NOTHING;

    // Cria gerenciador DNP3
    DNP3Manager manager(1, ConsoleLogger::Create());

    // Configura canal DNP3
    auto channel = std::shared_ptr<IChannel>(nullptr);
    try
    {
        channel = manager.AddTCPServer("server", logLevels, ServerAcceptMode::CloseExisting, IPEndpoint("192.168.100.176", 20000),
                                       PrintingChannelListener::Create());
    }
    catch(const std::exception& e)
    {
        std::cerr << "Erro ao configurar canal DNP3: " << e.what() << '\n';
        return -1;
    }

    // Configura stack DNP3 outstation
    OutstationStackConfig config(ConfigureDatabase());

    config.outstation.eventBufferConfig = EventBufferConfig::AllTypes(10);
    
    config.outstation.params.allowUnsolicited = true;
    
    config.link.LocalAddr = 2;
    config.link.RemoteAddr = 1; 
    
    config.link.KeepAliveTimeout = TimeDuration::Seconds(30);

    // Cria instancia outstation
    auto outstation = channel->AddOutstation(
        "outstation", 
        std::make_shared<DirectOperateOnlyHandler>(ctx, state), 
        DefaultOutstationApplication::Create(), 
        config
    );

    outstation->Enable();

    // Loop principal da aplicacao
    while (!shutdown_flag) {
        // Le valores do Modbus
        bool read_success = ReadModbusValues(ctx, modbus_ip, modbus_port, modbus_slave_id, state);

        // Atualiza pontos DNP3
        UpdateBuilder builder;
        AddUpdates(builder, state);
        outstation->Apply(builder.Build());

        // Log de status
        if (!read_success) {
            if (state.failure_count >= state.max_failures_before_zero) {
                cout << "Falha prolongada - Enviando 0 (Status: FALHA)" << endl;
            } else {
                cout << "Falha temporaria - Ultimo valor valido: " 
                     << state.last_valid_value << " (Status: FALHA)" << endl;
            }
        } else {
            cout << "Leitura OK - Valor atual: " << state.analog 
                 << " (Status: CONECTADO)" << endl;
        }

        this_thread::sleep_for(chrono::seconds(1));
    }

    // Limpeza da conexao Modbus
    if (state.modbus_connected) {
        modbus_close(ctx);
    }
    modbus_free(ctx);

    cout << "Encerrando programa..." << endl;
    return 0;
}