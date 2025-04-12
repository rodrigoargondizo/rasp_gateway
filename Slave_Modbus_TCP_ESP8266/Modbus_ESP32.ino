/**
 * @file Escravo_Modbus_ESP8266.ino
 * @brief Implementacao de um dispositivo escravo Modbus-TCP com ESP8266
 * 
 * Este codigo configura um ESP8266 como um dispositivo escravo Modbus-TCP que:
 * - Liga e desliga um LED via coils Modbus
 * - Le um potenciometro e disponibiliza via registrador holding
 * - Le um botao fisico com debounce e disponibiliza via coil
 */

 #include <Arduino.h>
 #include <ESP8266WiFi.h>
 #include <ModbusIP_ESP8266.h>
 
 // ========== CONFIGURACAO DE REDE ==========
 const IPAddress ip(192, 168, 100, 120);      // IP estatico do dispositivo
 const IPAddress gateway(192, 168, 100, 1);   // Gateway da rede
 const IPAddress subnet(255, 255, 255, 0);    // Mascara de sub-rede
 const char* ssid = "casa.exe";               // Nome da rede WiFi
 const char* password = "AfxJ457T";           // Senha da rede WiFi
 
 // ========== ENDERECOS MODBUS (0-based) ==========
 const uint16_t COIL_LIGAR = 0;       // Endereco 00001 - Coil para comando de ligar
 const uint16_t COIL_DESLIGAR = 1;    // Endereco 00002 - Coil para comando de desligar
 const uint16_t COIL_STATUS_LED = 2;  // Endereco 00003 - Coil de status do LED (leitura)
 const uint16_t COIL_BOTAO_D0 = 3;    // Endereco 00004 - Estado do botao (logica invertida)
 const uint16_t HREG_POT = 0;         // Endereco 40001 - Registrador holding para potenciometro
 
 // ========== PINAGEM ==========
 const uint8_t LED_PIN = D2;          // Pino do LED (saida)
 const uint8_t POT_PIN = A0;          // Pino analogico do potenciometro
 const uint8_t BUTTON_PIN = D0;       // Pino digital do botao (entrada com pull-up)
 
 ModbusIP mb;  // Objeto ModbusIP
 
 // ========== VARIAVEIS PARA DEBOUNCE DO BOTAO ==========
 bool lastButtonState = HIGH;          // Ultimo estado estavel do botao
 unsigned long lastDebounceTime = 0;   // Tempo do ultimo cambio de estado
 const unsigned long debounceDelay = 50;  // Tempo de debounce em ms
 
 /**
  * @brief Configuracao inicial do dispositivo
  */
 void setup() {
     Serial.begin(115200);
     
     // Configuracao dos pinos
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LOW);           // Inicia com LED desligado
     pinMode(BUTTON_PIN, INPUT_PULLUP);    // Configura botao com resistor pull-up interno
 
     // Conexao WiFi
     WiFi.config(ip, gateway, subnet);
     WiFi.begin(ssid, password);
     while (WiFi.status() != WL_CONNECTED) {
         delay(500);
         Serial.print(".");
     }
     Serial.println("\nConectado! IP: " + WiFi.localIP().toString());
 
     // Configuracao do servidor Modbus
     mb.server();  // Inicia o servidor Modbus
     
     // Adiciona os registradores Modbus
     mb.addCoil(COIL_LIGAR);         // Coil para comando de ligar
     mb.addCoil(COIL_DESLIGAR);      // Coil para comando de desligar
     mb.addCoil(COIL_STATUS_LED);    // Coil para leitura do status do LED
     mb.addCoil(COIL_BOTAO_D0);      // Coil para leitura do botao
     mb.addHreg(HREG_POT);           // Registrador holding para o potenciometro
 
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
     mb.task();  // Processa requisicoes Modbus
     
     // Atualiza leitura do potenciometro a cada 500ms
     static uint32_t lastUpdate = 0;
     if (millis() - lastUpdate >= 500) {
         lastUpdate = millis();
         uint16_t val = analogRead(POT_PIN);
         
         // So atualiza se a diferenca for significativa (>5)
         if (abs(val - mb.Hreg(HREG_POT)) > 5) {
             mb.Hreg(HREG_POT, val);  // Atualiza registrador holding
             Serial.println("Pot: " + String(val));
         }
     }
 
     // Leitura do botao com debounce
     bool reading = digitalRead(BUTTON_PIN);
     
     // Se o estado mudou, reinicia o timer de debounce
     if (reading != lastButtonState) {
         lastDebounceTime = millis();
     }
     
     // Se passou tempo suficiente desde a ultima mudanca
     if ((millis() - lastDebounceTime) > debounceDelay) {
         // Atualiza o registrador Modbus (invertido por causa do PULLUP)
         // HIGH = botao solto, LOW = botao pressionado
         mb.Coil(COIL_BOTAO_D0, !reading);
     }
     
     lastButtonState = reading;  // Atualiza o ultimo estado estavel
     delay(10);  // Pequeno delay para evitar sobrecarga
 }
 