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

using namespace std;
using namespace opendnp3;

// Configuração do banco de dados DNP3 para 1 ponto analógico
DatabaseConfig ConfigureDatabase()
{
    DatabaseConfig config; // Inicializa um DatabaseConfig vazio

    // Adiciona apenas um ponto analógico com índice 0
    config.analog_input[0] = AnalogConfig();
    config.analog_input[0].clazz = PointClass::Class2;
    config.analog_input[0].svariation = StaticAnalogVariation::Group30Var2;

    return config;
}

// Estrutura para armazenar os valores lidos do Modbus
struct State
{
    int16_t analog = 0;
    int16_t last_valid_value = 0; // Armazena o último valor válido
    const int COIL_LIGAR = 0;   // Endereço do coil para ligar
    const int COIL_DESLIGAR = 1; // Endereço do coil para desligar
};

// Função para adicionar atualizações ao outstation DNP3
void AddUpdates(UpdateBuilder& builder, State& state)
{
    builder.Update(Analog(state.analog), 0); // Atualiza o ponto analógico no índice 0
}

// Função para ler valores do Modbus
bool ReadModbusValues(modbus_t* ctx, State& state)
{
    uint16_t tab_reg[1];

    // Lê 1 holding register a partir do endereço 0 (registro 40001)
    int rc = modbus_read_registers(ctx, 0, 1, tab_reg);
    if (rc == -1)
    {
        std::cerr << "Erro ao ler holding register: " << modbus_strerror(errno) << std::endl;
        return false;
    }

    // Atualiza o valor analógico com o valor lido do Modbus
    state.analog = static_cast<int16_t>(tab_reg[0]);
    state.last_valid_value = state.analog; // Armazena como último valor válido

    return true;
}


// Função para escrever em coils do Modbus
bool WriteModbusCoils(modbus_t* ctx, State& state)
{
    // Verifica o valor analógico e controla os coils
    if (state.analog == 1024) {
        // LIGAR - Usa state.COIL_LIGAR para acessar o endereço
        std::cout << "Enviando comando LIGAR (coil " << state.COIL_LIGAR << ")..." << std::endl;
        if (modbus_write_bit(ctx, state.COIL_LIGAR, true) == -1) {
            std::cerr << "Erro ao ligar: " << modbus_strerror(errno) << std::endl;
            return false;
        }
        std::cout << "Comando LIGAR enviado com sucesso!" << std::endl;
        
        // Verificação opcional
        uint8_t estado;
        if (modbus_read_bits(ctx, state.COIL_LIGAR, 1, &estado) != -1) {
            std::cout << "Estado do coil: " << (estado ? "ATIVO" : "INATIVO") << std::endl;
        }
    } 
    else if (state.analog == 0) {
        // DESLIGAR - Usa state.COIL_DESLIGAR para acessar o endereço
        std::cout << "Enviando comando DESLIGAR (coil " << state.COIL_DESLIGAR << ")..." << std::endl;
        if (modbus_write_bit(ctx, state.COIL_DESLIGAR, true) == -1) {
            std::cerr << "Erro ao desligar: " << modbus_strerror(errno) << std::endl;
            return false;
        }
        std::cout << "Comando DESLIGAR enviado com sucesso!" << std::endl;
        
        // Verificação opcional
        uint8_t estado;
        if (modbus_read_bits(ctx, state.COIL_DESLIGAR, 1, &estado) != -1) {
            std::cout << "Estado do coil: " << (estado ? "ATIVO" : "INATIVO") << std::endl;
        }
    }
    else {
        std::cout << "Valor analógico intermediário (" << state.analog << "), nenhuma ação tomada." << std::endl;
    }
    
    return true;
}


int main(int argc, char* argv[])
{
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

    auto outstation = channel->AddOutstation("outstation", SuccessCommandHandler::Create(),
                                             DefaultOutstationApplication::Create(), config);
    outstation->Enable();

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

    // Loop principal
    while (true)
    {
        // Lê os valores do Modbus
        if (ReadModbusValues(ctx, state))
        {
            // Atualiza os valores no outstation DNP3
            UpdateBuilder builder;
            AddUpdates(builder, state);
            outstation->Apply(builder.Build());

            std::cout << "Valores atualizados: Analog = " << state.analog << std::endl;
        // Controle do LED baseado no valor analógico
        if (!WriteModbusCoils(ctx, state)) {
            std::cerr << "Falha no controle dos coils" << std::endl;
        }
        }
        // Aguarda 1 segundo antes da próxima leitura
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Fecha a conexão Modbus e libera o contexto
    modbus_close(ctx);
    modbus_free(ctx);

    std::cout << "Encerrando programa..." << std::endl;
    return 0;
}
