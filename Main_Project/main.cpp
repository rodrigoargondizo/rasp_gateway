/*
 * TCC - Rodrigo Faria Argondizo
 * Implementação 1 - Oustation DNP3 integrada a estação de testes com ESP8266
 * Gateway Modbus-TCP para DNP3
 * 
 * Este código atua como um gateway entre dispositivos Modbus-TCP e um mestre DNP3,
 * convertendo dados de um protocolo para outro e fornecendo funcionalidade de comando.
 * 
 * Funcionalidades principais:
 * - Leitura de registros Modbus (potenciômetro, botão e LED)
 * - Exposição dos dados via DNP3 como pontos analógicos e binários
 * - Controle do dispositivo via comandos DNP3 (ligar/desligar)
 * - Tratamento de falhas de conexão Modbus
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
 
 /**
  * @brief Estrutura para armazenar o estado do dispositivo Modbus e da conexão
  * 
  * Contém os valores atuais lidos do dispositivo Modbus, status de conexão,
  * e informações para tratamento de falhas.
  */
 struct State {
     // Valor analógico lido do potenciômetro
     int16_t analog = 0;
     
     // Status digital do LED
     bool led_status = 0;
     
     // Status digital do botão (lógica invertida)
     bool button_status = 0;
     
     // Último valor válido lido (usado em caso de falha)
     int16_t last_valid_value = 0;
     
     // Endereços Modbus para os coils
     const int COIL_LIGAR = 0;         // Coil para comando de ligar
     const int COIL_DESLIGAR = 1;      // Coil para comando de desligar
     const int COIL_STATUS_LED = 2;    // Coil de status do LED
     const int COIL_STATUS_BUTTON = 3; // Coil de status do botão (1=botão solto, 0=botão pressionado)
     
     // Status e controle de conexão Modbus
     bool modbus_connected = false;            // Estado atual da conexão
     bool last_connection_state = false;      // Último estado conhecido (para detecção de mudança)
     int failure_count = 0;                   // Contador de falhas consecutivas
     const int max_failures_before_zero = 5;  // Número máximo de falhas antes de enviar zero
 };
 
 /**
  * @brief Configura os pontos de dados DNP3
  * 
  * Define a estrutura de banco de dados para o outstation DNP3, incluindo
  * tipos de pontos, classes e variações estáticas.
  * 
  * @return DatabaseConfig Configuração do banco de dados DNP3
  */
 DatabaseConfig ConfigureDatabase() {
     DatabaseConfig config;
     
     // Configuração do ponto analógico (potenciômetro) - índice 0
     config.analog_input[0] = AnalogConfig();
     config.analog_input[0].clazz = PointClass::Class2;          // Classe 2 (dados periódicos)
     config.analog_input[0].svariation = StaticAnalogVariation::Group30Var2; // Representação de 16 bits
 
     // Configuração do ponto binário (status de conexão) - índice 0
     config.binary_input[0] = BinaryConfig();
     config.binary_input[0].clazz = PointClass::Class1;          // Classe 1 (dados importantes)
     config.binary_input[0].svariation = StaticBinaryVariation::Group1Var2; // Status binário simples
 
     // Configuração do ponto binário (status do LED) - índice 1
     config.binary_input[1] = BinaryConfig();
     config.binary_input[1].clazz = PointClass::Class1;
     config.binary_input[1].svariation = StaticBinaryVariation::Group1Var2;
 
     // Configuração do ponto binário (status do botão) - índice 2
     config.binary_input[2] = BinaryConfig();
     config.binary_input[2].clazz = PointClass::Class1;
     config.binary_input[2].svariation = StaticBinaryVariation::Group1Var2;
 
     return config;
 }
 
 /**
  * @brief Atualiza os valores DNP3 com base no estado atual
  * 
  * @param builder Objeto UpdateBuilder para construir as atualizações DNP3
  * @param state Estado atual do dispositivo Modbus
  */
 void AddUpdates(UpdateBuilder& builder, State& state) {
     // Atualiza o valor analógico (potenciômetro)
     if (state.failure_count >= state.max_failures_before_zero) {
         // Envia zero em caso de falhas prolongadas
         builder.Update(Analog(0), 0);
     } else {
         // Envia o último valor válido lido
         builder.Update(Analog(state.last_valid_value), 0);
     }
     
     // Atualiza o status de conexão (binário)
     bool connection_failed = !state.modbus_connected;
     builder.Update(Binary(connection_failed), 0);
     
     // Se o status de conexão mudou, envia como evento com flag de qualidade
     if (state.modbus_connected != state.last_connection_state) {
         builder.Update(Binary(connection_failed, Flags(0x01)), 0);
         state.last_connection_state = state.modbus_connected;
     }
 
     // Atualiza o status do LED
     builder.Update(Binary(state.led_status), 1);
 
     // Atualiza o status do botão
     builder.Update(Binary(state.button_status), 2);
 }
 
 /**
  * @brief Tenta reconectar ao dispositivo Modbus em caso de falha
  * 
  * @param ctx Contexto Modbus
  * @param ip Endereço IP do dispositivo Modbus
  * @param port Porta Modbus
  * @param slave_id ID do escravo Modbus
  * @param state Estado atual da conexão
  * @return true se a reconexão foi bem-sucedida
  * @return false se a reconexão falhou
  */
 bool TryModbusReconnect(modbus_t* ctx, const char* ip, int port, int slave_id, State& state) {
     cout << "Tentando reconectar ao Modbus..." << endl;
     
     // Fecha a conexão existente se estiver aberta
     if (state.modbus_connected) {
         modbus_close(ctx);
         state.modbus_connected = false;
     }
 
     // Cria um novo contexto se necessário
     if (ctx == nullptr) {
         ctx = modbus_new_tcp(ip, port);
         if (ctx == nullptr) {
             cerr << "Falha ao criar novo contexto Modbus" << endl;
             return false;
         }
         // Configura timeouts curtos para tentativas de reconexão rápidas
         modbus_set_response_timeout(ctx, 1, 0);
         modbus_set_byte_timeout(ctx, 1, 0);
     }
 
     // Configura o ID do escravo
     if (modbus_set_slave(ctx, slave_id) == -1) {
         cerr << "Erro ao configurar ID do escravo: " << modbus_strerror(errno) << endl;
         modbus_free(ctx);
         return false;
     }
 
     // Tenta conectar
     if (modbus_connect(ctx) == -1) {
         cerr << "Falha na reconexão: " << modbus_strerror(errno) << endl;
         return false;
     }
 
     state.modbus_connected = true;
     cout << "Conexão Modbus restabelecida!" << endl;
     return true;
 }
 
 /**
  * @brief Lê os valores atuais do dispositivo Modbus
  * 
  * @param ctx Contexto Modbus
  * @param ip Endereço IP do dispositivo
  * @param port Porta Modbus
  * @param slave_id ID do escravo
  * @param state Estado atual para armazenar os valores lidos
  * @return true se a leitura foi bem-sucedida
  * @return false se ocorreu um erro
  */
 bool ReadModbusValues(modbus_t* ctx, const char* ip, int port, int slave_id, State& state) {
     uint16_t tab_reg[1];      // Buffer para leitura do registrador analógico
     uint8_t led_status;       // Buffer para status do LED
     uint8_t button_status;    // Buffer para status do botão
     
     // Tenta reconectar se não estiver conectado
     if (!state.modbus_connected) {
         if (!TryModbusReconnect(ctx, ip, port, slave_id, state)) {
             state.failure_count++;
             return false;
         }
     }
 
     // Limpa o buffer antes de novas leituras
     modbus_flush(ctx);
     
     // Lê o valor analógico (potenciômetro) - registrador 0
     int rc = modbus_read_registers(ctx, 0, 1, tab_reg);
     
     if (rc == -1) {
         cerr << "Erro na leitura do potenciometro: " << modbus_strerror(errno) << endl;
         modbus_close(ctx);
         state.modbus_connected = false;
         state.failure_count++;
         return false;
     }
 
     // Lê o status do LED (coil 2)
     rc = modbus_read_bits(ctx, state.COIL_STATUS_LED, 1, &led_status);
     
     if (rc == -1) {
         cerr << "Erro na leitura do status do LED: " << modbus_strerror(errno) << endl;
         modbus_close(ctx);
         state.modbus_connected = false;
         state.failure_count++;
         return false;
     }
 
     // Lê o status do botão (coil 3)
     rc = modbus_read_bits(ctx, state.COIL_STATUS_BUTTON, 1, &button_status);
     
     if (rc == -1) {
         cerr << "Erro na leitura do coil do Botao: " << modbus_strerror(errno) << endl;
         modbus_close(ctx);
         state.modbus_connected = false;
         state.failure_count++;
         return false;
     }
 
     // Atualiza o estado com os valores lidos
     state.analog = static_cast<int16_t>(tab_reg[0]);
     state.last_valid_value = state.analog;
     state.led_status = (led_status == 1); // Converte para bool (true = ligado)
     state.button_status = !(button_status == 1); // Inverte a lógica (true = pressionado)
     state.failure_count = 0; // Reseta o contador de falhas após leitura bem-sucedida
     
     return true;
 }
 
 // Flag global para controle de encerramento
 volatile sig_atomic_t shutdown_flag = 0;
 
 /**
  * @brief Manipulador de sinais para encerramento gracioso
  * 
  * @param signal Sinal recebido
  */
 void signal_handler(int signal) {
     shutdown_flag = 1;
 }
 
 /**
  * @brief Envia comando para ligar o dispositivo via Modbus
  * 
  * @param ctx Contexto Modbus
  * @param state Estado atual (contém endereços dos coils)
  * @return true se o comando foi enviado com sucesso
  * @return false se ocorreu um erro
  */
 bool LigarDispositivo(modbus_t* ctx, const State& state) {
     if (modbus_write_bit(ctx, state.COIL_LIGAR, true) == -1) {
         std::cerr << "Erro ao ligar: " << modbus_strerror(errno) << std::endl;
         return false;
     }
     modbus_flush(ctx); // Garante que o comando foi enviado
     return true;
 }
 
 /**
  * @brief Envia comando para desligar o dispositivo via Modbus
  * 
  * @param ctx Contexto Modbus
  * @param state Estado atual (contém endereços dos coils)
  * @return true se o comando foi enviado com sucesso
  * @return false se ocorreu um erro
  */
 bool DesligarDispositivo(modbus_t* ctx, const State& state) {
     if (modbus_write_bit(ctx, state.COIL_DESLIGAR, true) == -1) {
         std::cerr << "Erro ao desligar: " << modbus_strerror(errno) << std::endl;
         return false;
     }
     modbus_flush(ctx);
     return true;
 }
 
 /**
  * @brief Manipulador de comandos DNP3 para operação direta
  * 
  * Implementa a interface SimpleCommandHandler para traduzir comandos DNP3
  * em operações Modbus.
  */
 class DirectOperateOnlyHandler : public SimpleCommandHandler {
 private:
     modbus_t* ctx;   // Contexto Modbus
     State state;     // Estado atual
 
 public:
     /**
      * @brief Constrói um novo manipulador de comandos
      * 
      * @param modbus_ctx Contexto Modbus para envio de comandos
      * @param initialState Estado inicial do dispositivo
      */
     DirectOperateOnlyHandler(modbus_t* modbus_ctx, const State& initialState)
         : SimpleCommandHandler(CommandStatus::SUCCESS),
           ctx(modbus_ctx),
           state(initialState) {}
 
     /**
      * @brief Manipula comandos de operação direta (DirectOperate)
      * 
      * @param command Comando recebido
      * @param index Índice do ponto (0 = ligar, 1 = desligar)
      * @param handler Manipulador de atualizações (não usado)
      * @param opType Tipo de operação (apenas DirectOperate é suportado)
      * @return CommandStatus Resultado da operação
      */
     CommandStatus Operate(const ControlRelayOutputBlock& command, 
                           uint16_t index,
                           IUpdateHandler& handler,
                           OperateType opType) override {
         if (opType == OperateType::DirectOperate) {
             switch (index) {
                 case 0: // Comando para ligar
                     std::cout << "Direct Operate: LED ON" << std::endl;
                     return LigarDispositivo(ctx, state) ? 
                            CommandStatus::SUCCESS : 
                            CommandStatus::HARDWARE_ERROR;
                 case 1: // Comando para desligar
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
     // Configura manipuladores de sinal para encerramento gracioso
     signal(SIGINT, signal_handler);
     signal(SIGTERM, signal_handler);
 
     // Configuração Modbus
     const char* modbus_ip = "192.168.100.120"; // IP do dispositivo Modbus
     const int modbus_port = 502;               // Porta padrão Modbus
     const int modbus_slave_id = 1;             // ID do escravo Modbus
 
     // Inicializa o contexto Modbus
     modbus_t* ctx = modbus_new_tcp(modbus_ip, modbus_port);
     if (ctx == nullptr) {
         std::cerr << "Falha ao criar contexto Modbus" << std::endl;
         return -1;
     }
 
     // Configura timeouts para operações Modbus
     modbus_set_response_timeout(ctx, 1, 0);  // 1 segundo de timeout
     modbus_set_byte_timeout(ctx, 1, 0);
 
     // Loop de tentativa de conexão inicial
     while (!shutdown_flag) {
         if (modbus_connect(ctx) == -1) {
             std::cerr << "Erro ao conectar ao dispositivo: " << modbus_strerror(errno) << std::endl;
             std::cerr << "Tentando novamente em 5 segundos..." << std::endl;
             std::this_thread::sleep_for(std::chrono::seconds(5));
         } else {
             std::cout << "Conexão Modbus estabelecida com sucesso!" << std::endl;
             break;
         }
     }
 
     // Configura o ID do escravo Modbus
     if (modbus_set_slave(ctx, modbus_slave_id) == -1) {
         std::cerr << "Erro ao configurar ID do escravo: " << modbus_strerror(errno) << std::endl;
         modbus_free(ctx);
         return -1;
     }
 
     State state; // Estado inicial do dispositivo
 
     // Configuração do DNP3
     const auto logLevels = levels::NORMAL | levels::NOTHING; // Log mínimo
 
     // Inicializa o gerenciador DNP3 com 1 thread de processamento
     DNP3Manager manager(1, ConsoleLogger::Create());
 
     // Configura o canal TCP DNP3 (servidor)
     auto channel = std::shared_ptr<IChannel>(nullptr);
     try {
         channel = manager.AddTCPServer(
             "server", 
             logLevels, 
             ServerAcceptMode::CloseExisting, 
             IPEndpoint("192.168.100.176", 20000), // Endereço e porta do outstation
             PrintingChannelListener::Create()
         );
     } catch(const std::exception& e) {
         std::cerr << "Erro ao configurar canal DNP3: " << e.what() << '\n';
         return -1;
     }
 
     // Configuração do outstation DNP3
     OutstationStackConfig config(ConfigureDatabase());
 
     // Configuração do buffer de eventos
     config.outstation.eventBufferConfig = EventBufferConfig::AllTypes(10);
     
     // Habilita dados não solicitados (unsolicited responses)
     config.outstation.params.allowUnsolicited = true;
     
     // Configuração do link DNP3
     config.link.LocalAddr = 2;   // Endereço do outstation
     config.link.RemoteAddr = 1;  // Endereço do mestre
     config.link.KeepAliveTimeout = TimeDuration::Seconds(30);
 
     // Cria o outstation com manipulador de comandos personalizado
     auto outstation = channel->AddOutstation(
         "outstation", 
         std::make_shared<DirectOperateOnlyHandler>(ctx, state), 
         DefaultOutstationApplication::Create(), 
         config
     );
 
     outstation->Enable(); // Habilita o outstation
 
     // Loop principal de leitura e atualização
     while (!shutdown_flag) {
         // Lê valores do Modbus
         bool read_success = ReadModbusValues(ctx, modbus_ip, modbus_port, modbus_slave_id, state);
 
         // Prepara e aplica atualizações DNP3
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
 
         // Intervalo entre leituras
         this_thread::sleep_for(chrono::seconds(1));
     }
 
     // Limpeza e encerramento
     if (state.modbus_connected) {
         modbus_close(ctx);
     }
     modbus_free(ctx);
 
     cout << "Encerrando programa..." << endl;
     return 0;
 }