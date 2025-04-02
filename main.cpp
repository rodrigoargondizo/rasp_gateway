#include <opendnp3/DNP3Manager.h>
#include <opendnp3/outstation/OutstationStackConfig.h>
#include <opendnp3/outstation/SimpleCommandHandler.h>
#include <opendnp3/outstation/DefaultOutstationApplication.h>
#include <opendnp3/ConsoleLogger.h>
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
    // Para o valor analógico
    int16_t analog = 0;
    int16_t last_valid_value = 0; // Armazena o último valor válido
    const int COIL_LIGAR = 0;   // Endereço do coil para ligar
    const int COIL_DESLIGAR = 1; // Endereço do coil para desligar
        
    // Para o status de conexão
    bool modbus_connected = false;
    bool last_connection_state = false;
    int failure_count = 0;
    const int max_failures_before_zero = 5;
};

//Estrutura para configurar os pontos DNP3
DatabaseConfig ConfigureDatabase()
{
    DatabaseConfig config; // Inicializa um DatabaseConfig vazio

    // Ponto analógico (valor do Potenciometro do escravo Modbus) - índice 0
    config.analog_input[0] = AnalogConfig();
    config.analog_input[0].clazz = PointClass::Class2;
    config.analog_input[0].svariation = StaticAnalogVariation::Group30Var2;


    // Ponto binário (status da conexão) - índice 0
    config.binary_input[0] = BinaryConfig();
    config.binary_input[0].clazz = PointClass::Class1;
    config.binary_input[0].svariation = StaticBinaryVariation::Group1Var2;

    // Ponto binário (status da do LED) - índice 1
    config.binary_input[1] = BinaryConfig();
    config.binary_input[1].clazz = PointClass::Class1;
    config.binary_input[1].svariation = StaticBinaryVariation::Group1Var2;

    return config;
}

// Função para ligar dispositivo Modbus
bool LigarDispositivo(modbus_t* ctx, const State& state) {
    if (modbus_write_bit(ctx, state.COIL_LIGAR, true) == -1) {
        std::cerr << "Erro ao ligar: " << modbus_strerror(errno) << std::endl;
        return false;
    }
    modbus_flush(ctx); // Garante que os dados foram enviados corretamente
    return true;
}

// Função para desligar dispositivo Modbus
bool DesligarDispositivo(modbus_t* ctx, const State& state) {
    if (modbus_write_bit(ctx, state.COIL_DESLIGAR, true) == -1) {
        std::cerr << "Erro ao desligar: " << modbus_strerror(errno) << std::endl;
        return false;
    }
    modbus_flush(ctx);
    return true;
}

// Classe para manipulação de comandos DNP3
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

    // Inicializa o contexto Modbus
    modbus_t *ctx = modbus_new_tcp("192.168.100.120", 502); // Substitua pelo IP e porta do seu dispositivo
    if (ctx == nullptr)
    {
        std::cerr << "Erro ao criar contexto Modbus: " << modbus_strerror(errno) << std::endl;
        return -1;
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

    // Configura o nível de log para exibir apenas erros críticos
    const auto logLevels = levels::NORMAL | levels::NOTHING;

    // Inicializa o gerenciador DNP3 com log mínimo
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

    // Configuração do buffer de eventos
    config.outstation.eventBufferConfig = EventBufferConfig::AllTypes(10);
    
    // Habilita dados não solicitados
    config.outstation.params.allowUnsolicited = true;
    // Slave
    config.link.LocalAddr = 2;
    // Master
    config.link.RemoteAddr = 1; 
	
    config.link.KeepAliveTimeout = TimeDuration::Seconds(30);

    //Outstation com manipulador de comando Direct Operate
    //auto outstation = channel->AddOutstation("outstation", std::make_shared<DirectOperateOnlyHandler>(),
    //                                         DefaultOutstationApplication::Create(), config);

    //Outstation com manipulador de comando Direct Operate integrado com Modbus
    auto outstation = channel->AddOutstation(
        "outstation", 
        std::make_shared<DirectOperateOnlyHandler>(ctx, state), 
        DefaultOutstationApplication::Create(), 
        config
    );

    outstation->Enable();

    // Loop principal
    while (true)
    {
        // Aguarda 1 segundo antes da próxima leitura
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

// Fecha a conexão Modbus e libera o contexto
modbus_close(ctx);
modbus_free(ctx);

std::cout << "Encerrando programa..." << std::endl;
return 0;
}