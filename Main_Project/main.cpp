/*
 * TCC - Rodrigo Faria Argondizo
 * Implementacao 1 - Outstation DNP3 integrada a estacao de testes com ESP8266
 * Gateway Modbus-TCP para DNP3
 * 
 * Este codigo atua como um gateway entre dispositivos Modbus-TCP e um mestre DNP3,
 * convertendo dados de um protocolo para outro e fornecendo funcionalidade de comando.
 * 
 * Funcionalidades principais:
 * - Leitura de registros Modbus (potenciometro, botao e LED)
 * - Exposicao dos dados via DNP3 como pontos analogicos e binarios
 * - Controle do dispositivo via comandos DNP3 (ligar/desligar)
 * - Tratamento de falhas de conexao Modbus
 */

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

// Estrutura para armazenar os valores lidos do Modbus
struct State {
    // Para o valor analogico
    int16_t analog = 0;
    bool led_status = 0;
    bool button_status = 0;
    int16_t last_valid_value = 0; // Armazena o ultimo valor valido
    const int COIL_LIGAR = 0;   // Endereco do coil para ligar
    const int COIL_DESLIGAR = 1; // Endereco do coil para desligar
    const int COIL_STATUS_LED = 2; // Endereco do coil de status do LED
    const int COIL_STATUS_BUTTON = 3; // (Estado do botao - Logica invertida: 1=botao solto, 0=botao pressionado)
        
    // Para o status de conexao
    bool modbus_connected = false;
    bool last_connection_state = false;
    int failure_count = 0;
    const int max_failures_before_zero = 5;
};

//Estrutura para configurar os pontos DNP3
DatabaseConfig ConfigureDatabase()
{
    DatabaseConfig config; // Inicializa um DatabaseConfig vazio

    // Ponto analogico (valor do Potenciometro do escravo Modbus) - indice 0
    config.analog_input[0] = AnalogConfig();
    config.analog_input[0].clazz = PointClass::Class2;
    config.analog_input[0].svariation = StaticAnalogVariation::Group30Var2;


    // Ponto binario (status da conexao) - indice 0
    config.binary_input[0] = BinaryConfig();
    config.binary_input[0].clazz = PointClass::Class1;
    config.binary_input[0].svariation = StaticBinaryVariation::Group1Var2;

    // Ponto binario (status do LED) - indice 1
    config.binary_input[1] = BinaryConfig();
    config.binary_input[1].clazz = PointClass::Class1;
    config.binary_input[1].svariation = StaticBinaryVariation::Group1Var2;

    // Ponto binario (status do Botao) - indice 2
    config.binary_input[2] = BinaryConfig();
    config.binary_input[2].clazz = PointClass::Class1;
    config.binary_input[2].svariation = StaticBinaryVariation::Group1Var2;

    return config;
}

//Estrutura que atualiza os valores Modbus nos pontos DNP3
void AddUpdates(UpdateBuilder& builder, State& state) {
    // Atualiza o valor analogico (indice 0)
    if (state.failure_count >= state.max_failures_before_zero) {
        builder.Update(Analog(0), 0);
    } else {
        builder.Update(Analog(state.last_valid_value), 0);
    }
    
    // Atualiza o status binario (indice 0)
    // true = falha de conexao, false = conexao OK
    bool connection_failed = !state.modbus_connected;
    builder.Update(Binary(connection_failed), 0);
    
    // Se o status mudou, envia como evento com flag de qualidade
    if (state.modbus_connected != state.last_connection_state) {
        builder.Update(Binary(connection_failed, Flags(0x01)), 0);
        state.last_connection_state = state.modbus_connected;
    }

    // Atualiza o status binario do LED (indice 1)
    builder.Update(Binary(state.led_status), 1);

    // Atualiza o status binario do LED (indice 2)
    builder.Update(Binary(state.button_status), 2);
}

//Funcao que tenta reconectar aos dispositivos Modbus em caso de falha
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
        cerr << "Falha na reconexao: " << modbus_strerror(errno) << endl;
        return false;
    }

    state.modbus_connected = true;
    cout << "Conexao Modbus restabelecida!" << endl;
    return true;
}

//Funcao de Leitura dos Pontos Modbus
bool ReadModbusValues(modbus_t* ctx, const char* ip, int port, int slave_id, State& state) {
    uint16_t tab_reg[1];
    uint8_t led_status, button_status;
    
    if (!state.modbus_connected) {
        if (!TryModbusReconnect(ctx, ip, port, slave_id, state)) {
            state.failure_count++;
            return false;
        }
    }

    modbus_flush(ctx);
    int rc = modbus_read_registers(ctx, 0, 1, tab_reg);
    
    if (rc == -1) {
        cerr << "Erro na leitura do potenciometro: " << modbus_strerror(errno) << endl;
        modbus_close(ctx);
        state.modbus_connected = false;
        state.failure_count++;
        return false;
    }

    // Le o status do LED (coil 2)
    rc = modbus_read_bits(ctx, state.COIL_STATUS_LED, 1, &led_status);
    
    if (rc == -1) {
        cerr << "Erro na leitura do status do LED: " << modbus_strerror(errno) << endl;
        modbus_close(ctx);
        state.modbus_connected = false;
        state.failure_count++;
        return false;
    }

        // Le o status do Botao (coil 3)
    rc = modbus_read_bits(ctx, state.COIL_STATUS_BUTTON, 1, &button_status);
    
    if (rc == -1) {
        cerr << "Erro na leitura do coil do Botao: " << modbus_strerror(errno) << endl;
        modbus_close(ctx);
        state.modbus_connected = false;
        state.failure_count++;
        return false;
    }

    state.analog = static_cast<int16_t>(tab_reg[0]);
    state.last_valid_value = state.analog;
    state.led_status = (led_status == 1); // Converte para bool (true = ligado)
    state.button_status = !(button_status == 1); // Converte para bool (true = ligado)
    state.failure_count = 0;
    return true;
}

volatile sig_atomic_t shutdown_flag = 0;

void signal_handler(int signal) {
    shutdown_flag = 1;
}

// Funcao para ligar dispositivo Modbus
bool LigarDispositivo(modbus_t* ctx, const State& state) {
    if (modbus_write_bit(ctx, state.COIL_LIGAR, true) == -1) {
        std::cerr << "Erro ao ligar: " << modbus_strerror(errno) << std::endl;
        return false;
    }
    modbus_flush(ctx); // Garante que os dados foram enviados corretamente
    return true;
}

// Funcao para desligar dispositivo Modbus
bool DesligarDispositivo(modbus_t* ctx, const State& state) {
    if (modbus_write_bit(ctx, state.COIL_DESLIGAR, true) == -1) {
        std::cerr << "Erro ao desligar: " << modbus_strerror(errno) << std::endl;
        return false;
    }
    modbus_flush(ctx);
    return true;
}

// Classe para manipulacao de comandos DNP3
class DirectOperateOnlyHandler : public SimpleCommandHandler {
private:
    modbus_t* ctx;
    State state;

public:
    DirectOperateOnlyHandler(modbus_t* modbus_ctx, const State& initialState)
        : SimpleCommandHandler(CommandStatus::SUCCESS),
          ctx(modbus_ctx),
          state(initialState) {}

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

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Configuracao Modbus
    const char* modbus_ip = "192.168.100.120";
    const int modbus_port = 502;
    const int modbus_slave_id = 1;

    // Inicializa o contexto Modbus
    modbus_t* ctx = modbus_new_tcp(modbus_ip, modbus_port); // Substitua pelo IP e porta do seu dispositivo

    // Loop infinito para tentar conectar ate o sucesso
    while (!shutdown_flag) {
        if (modbus_connect(ctx) == -1) {
            std::cerr << "Erro ao conectar ao dispositivo: " << modbus_strerror(errno) << std::endl;
            std::cerr << "Tentando novamente em 5 segundos..." << std::endl;
            
            std::this_thread::sleep_for(std::chrono::seconds(5));
        } else {
            std::cout << "Conexao Modbus estabelecida com sucesso!" << std::endl;
            break;  // Sai do loop se a conexao for bem-sucedida
        }
    }

    // Define o ID do escravo Modbus
    if (modbus_set_slave(ctx, 1) == -1) // Substitua 1 pelo ID do seu escravo
    {
        std::cerr << "Erro ao configurar ID do escravo: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return -1;
    }

    // Conecta ao dispositivo Modbus
    if (modbus_connect(ctx) == -1)
    {
        std::cerr << "Erro ao conectar ao dispositivo: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return -1;
    }
    
    State state;

    // Configura o nivel de log para exibir apenas erros criticos
    const auto logLevels = levels::NORMAL | levels::NOTHING;

    // Inicializa o gerenciador DNP3 com log minimo
    DNP3Manager manager(1, ConsoleLogger::Create());

    // Configura o canal TCP DNP3
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
    // Configura o outstation DNP3
    OutstationStackConfig config(ConfigureDatabase());

    // Configuracao do buffer de eventos
    config.outstation.eventBufferConfig = EventBufferConfig::AllTypes(10);
    
    // Habilita dados nao solicitados
    config.outstation.params.allowUnsolicited = true;
    // Slave
    config.link.LocalAddr = 2;
    // Master
    config.link.RemoteAddr = 1; 
    
    config.link.KeepAliveTimeout = TimeDuration::Seconds(30);

    //Outstation com manipulador de comando Direct Operate integrado com Modbus
    auto outstation = channel->AddOutstation(
        "outstation", 
        std::make_shared<DirectOperateOnlyHandler>(ctx, state), 
        DefaultOutstationApplication::Create(), 
        config
    );

    outstation->Enable();

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
                cout << "Falha temporaria - Ultimo valor valido: " 
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