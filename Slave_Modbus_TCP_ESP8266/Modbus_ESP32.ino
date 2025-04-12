/**
 * @file Escravo_Modbus_ESP8266.ino
 * @brief Implementação de um dispositivo escravo Modbus-TCP com ESP8266
 * 
 * Este código configura um ESP8266 como um dispositivo escravo Modbus-TCP que:
 * - Liga e desliga um LED via coils Modbus
 * - Lê um potenciômetro e disponibiliza via registrador holding
 * - Lê um botão físico com debounce e disponibiliza via coil
 */

 #include <Arduino.h>
 #include <ESP8266WiFi.h>
 #include <ModbusIP_ESP8266.h>
 
 // ========== CONFIGURAÇÃO DE REDE ==========
 const IPAddress ip(192, 168, 100, 120);      // IP estático do dispositivo
 const IPAddress gateway(192, 168, 100, 1);    // Gateway da rede
 const IPAddress subnet(255, 255, 255, 0);     // Máscara de sub-rede
 const char* ssid = "casa.exe";                // Nome da rede WiFi
 const char* password = "AfxJ457T";            // Senha da rede WiFi
 
 // ========== ENDEREÇOS MODBUS (0-based) ==========
 const uint16_t COIL_LIGAR = 0;       // Endereço 00001 - Coil para comando de ligar
 const uint16_t COIL_DESLIGAR = 1;    // Endereço 00002 - Coil para comando de desligar
 const uint16_t COIL_STATUS_LED = 2;  // Endereço 00003 - Coil de status do LED (leitura)
 const uint16_t COIL_BOTAO_D0 = 3;    // Endereço 00004 - Estado do botão (lógica invertida)
 const uint16_t HREG_POT = 0;         // Endereço 40001 - Registrador holding para potenciômetro
 
 // ========== PINAGEM ==========
 const uint8_t LED_PIN = D2;          // Pino do LED (saída)
 const uint8_t POT_PIN = A0;          // Pino analógico do potenciômetro
 const uint8_t BUTTON_PIN = D0;       // Pino digital do botão (entrada com pull-up)
 
 ModbusIP mb;  // Objeto ModbusIP
 
 // ========== VARIÁVEIS PARA DEBOUNCE DO BOTÃO ==========
 bool lastButtonState = HIGH;          // Último estado estável do botão
 unsigned long lastDebounceTime = 0;   // Tempo do último cambio de estado
 const unsigned long debounceDelay = 50;  // Tempo de debounce em ms
 
 /**
  * @brief Configuração inicial do dispositivo
  */
 void setup() {
     Serial.begin(115200);
     
     // Configuração dos pinos
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LOW);           // Inicia com LED desligado
     pinMode(BUTTON_PIN, INPUT_PULLUP);    // Configura botão com resistor pull-up interno
 
     // Conexão WiFi
     WiFi.config(ip, gateway, subnet);
     WiFi.begin(ssid, password);
     while (WiFi.status() != WL_CONNECTED) {
         delay(500);
         Serial.print(".");
     }
     Serial.println("\nConectado! IP: " + WiFi.localIP().toString());
 
     // Configuração do servidor Modbus
     mb.server();  // Inicia o servidor Modbus
     
     // Adiciona os registradores Modbus
     mb.addCoil(COIL_LIGAR);         // Coil para comando de ligar
     mb.addCoil(COIL_DESLIGAR);      // Coil para comando de desligar
     mb.addCoil(COIL_STATUS_LED);    // Coil para leitura do status do LED
     mb.addCoil(COIL_BOTAO_D0);      // Coil para leitura do botão
     mb.addHreg(HREG_POT);           // Registrador holding para o potenciômetro
 
     /**
      * @brief Callback para escrita na coil de ligar (00001)
      * 
      * @param reg Ponteiro para o registrador
      * @param val Valor a ser escrito
      * @return uint16_t Valor que foi efetivamente escrito
      */
     mb.onSetCoil(COIL_LIGAR, [](TRegister* reg, uint16_t val) {
         if (val) {  // Reage apenas quando escrevem 1
             digitalWrite(LED_PIN, HIGH);             // Liga o LED fisicamente
             mb.Coil(COIL_STATUS_LED, true);          // Atualiza o status do LED
             Serial.println("Ligado via coil 00001");
             mb.Coil(COIL_LIGAR, false);              // Auto-reset do comando
         }
         return val;
     });
 
     /**
      * @brief Callback para escrita na coil de desligar (00002)
      */
     mb.onSetCoil(COIL_DESLIGAR, [](TRegister* reg, uint16_t val) {
         if (val) {  // Reage apenas quando escrevem 1
             digitalWrite(LED_PIN, LOW);              // Desliga o LED fisicamente
             mb.Coil(COIL_STATUS_LED, false);         // Atualiza o status do LED
             Serial.println("Desligado via coil 00002");
             mb.Coil(COIL_DESLIGAR, false);           // Auto-reset do comando
         }
         return val;
     });
 }
 
 /**
  * @brief Loop principal do programa
  */
 void loop() {
     mb.task();  // Processa requisições Modbus
     
     // Atualiza leitura do potenciômetro a cada 500ms
     static uint32_t lastUpdate = 0;
     if (millis() - lastUpdate >= 500) {
         lastUpdate = millis();
         uint16_t val = analogRead(POT_PIN);
         
         // Só atualiza se a diferença for significativa (>5)
         if (abs(val - mb.Hreg(HREG_POT)) > 5) {
             mb.Hreg(HREG_POT, val);  // Atualiza registrador holding
             Serial.println("Pot: " + String(val));
         }
     }
 
     // Leitura do botão com debounce
     bool reading = digitalRead(BUTTON_PIN);
     
     // Se o estado mudou, reinicia o timer de debounce
     if (reading != lastButtonState) {
         lastDebounceTime = millis();
     }
     
     // Se passou tempo suficiente desde a última mudança
     if ((millis() - lastDebounceTime) > debounceDelay) {
         // Atualiza o registrador Modbus (invertido por causa do PULLUP)
         // HIGH = botão solto, LOW = botão pressionado
         mb.Coil(COIL_BOTAO_D0, !reading);
     }
     
     lastButtonState = reading;  // Atualiza o último estado estável
     delay(10);  // Pequeno delay para evitar sobrecarga
 }