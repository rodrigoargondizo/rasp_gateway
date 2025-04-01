#include <opendnp3/DNP3Manager.h>
#include <opendnp3/outstation/OutstationStackConfig.h>
#include <opendnp3/outstation/SimpleCommandHandler.h>
#include <opendnp3/outstation/DefaultOutstationApplication.h>
#include <opendnp3/ConsoleLogger.h>
#include <opendnp3/channel/IPEndpoint.h>
#include <opendnp3/channel/PrintingChannelListener.h>

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

using namespace opendnp3;

DatabaseConfig ConfigureDatabase()
{
    DatabaseConfig config; // Inicializa um DatabaseConfig vazio

    //Adiciona apenas um ponto analógico com índice 0
    //config.analog_input[0] = AnalogConfig();
    //config.analog_input[0].clazz = PointClass::Class2;
    //config.analog_input[0].svariation = StaticAnalogVariation::Group30Var2;

    
    // Adiciona 1 ponto binário de saída (para comandos)
    config.binary_output_status[0] = BOStatusConfig();

    return config;
}


class DirectOperateOnlyHandler : public SimpleCommandHandler {
public:
    DirectOperateOnlyHandler() : SimpleCommandHandler(CommandStatus::SUCCESS) {}

    CommandStatus Operate(const ControlRelayOutputBlock& command, 
                        uint16_t index,
                        IUpdateHandler& handler,
                        OperateType opType) override {
        if (opType == OperateType::DirectOperate && index == 0) {
            std::cout << "1" << std::endl;
        }
        return SimpleCommandHandler::Operate(command, index, handler, opType);
    }
};

int main() {

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

    auto outstation = channel->AddOutstation("outstation", std::make_shared<DirectOperateOnlyHandler>(),
                                             DefaultOutstationApplication::Create(), config);
    outstation->Enable();

    std::cout << "Outstation DNP3 rodando na porta 20000" << std::endl;
    std::cout << "Aguardando DirectOperate no índice 5..." << std::endl;

    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}