// Inclui bibliotecas necessarias
#include <Arduino.h>          // Biblioteca basica do Arduino
#include <ESP8266WiFi.h>      // Para conexao WiFi no ESP8266
#include <ModbusIP_ESP8266.h> // Para protocolo Modbus via IP

// Configuracoes de rede
const IPAddress ip(192, 168, 100, 120);      // IP fixo do dispositivo
const IPAddress gateway(192, 168, 100, 1);    // Gateway da rede
const IPAddress subnet(255, 255, 255, 0);     // Mascara de rede
const char* ssid = "casa.exe";                // Nome da rede WiFi
const char* password = "AfxJ457T";            // Senha da rede WiFi

// Definicoes dos registradores Modbus
const uint16_t COIL_LIGAR = 0;        // Coil para ligar LED (endereco 00001)
const uint16_t COIL_DESLIGAR = 1;     // Coil para desligar LED (endereco 00002)
const uint16_t COIL_STATUS_LED = 2;   // Coil para status do LED (endereco 00003)
const uint16_t COIL_BOTAO_D0 = 3;     // Coil para estado do botao (endereco 00004)
const uint16_t HREG_POT = 0;          // Registrador para valor do potenciometro

// Definicoes dos pinos
const uint8_t LED_PIN = D2;           // Pino do LED
const uint8_t POT_PIN = A0;           // Pino do potenciometro
const uint8_t BUTTON_PIN = D0;        // Pino do botao

ModbusIP mb;  // Objeto ModbusIP

// Variaveis para debounce do botao
bool lastButtonState = HIGH;                  // Ultimo estado do botao
unsigned long lastDebounceTime = 0;           // Tempo do ultimo debounce
const unsigned long debounceDelay = 50;       // Tempo de debounce em ms

void setup() {
    Serial.begin(115200);  // Inicia serial com baud rate 115200
    
    // Configura pinos
    pinMode(LED_PIN, OUTPUT);                // Define pino do LED como saida
    digitalWrite(LED_PIN, LOW);              // Inicia LED desligado
    pinMode(BUTTON_PIN, INPUT_PULLUP);       // Define pino do botao como entrada com pull-up

    // Configura conexao WiFi
    WiFi.config(ip, gateway, subnet);        // Define configuracao de rede
    WiFi.begin(ssid, password);              // Conecta na rede WiFi
    while (WiFi.status() != WL_CONNECTED) {  // Espera conexao
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConectado! IP: " + WiFi.localIP().toString());

    mb.server();  // Inicia servidor Modbus
    
    // Adiciona registradores Modbus
    mb.addCoil(COIL_LIGAR);          // Coil para ligar LED
    mb.addCoil(COIL_DESLIGAR);       // Coil para desligar LED
    mb.addCoil(COIL_STATUS_LED);     // Coil para status do LED
    mb.addCoil(COIL_BOTAO_D0);       // Coil para estado do botao
    mb.addHreg(HREG_POT);            // Registrador para valor do potenciometro

    // Callbacks para alteracao das coils
    mb.onSetCoil(COIL_LIGAR, [](TRegister* reg, uint16_t val) {
        if (val) {  // Se coil for ativada
            digitalWrite(LED_PIN, HIGH);          // Liga LED
            mb.Coil(COIL_STATUS_LED, true);       // Atualiza status do LED
            Serial.println("Ligado via coil 00001");
            mb.Coil(COIL_LIGAR, false);           // Reseta coil
        }
        return val;
    });

    mb.onSetCoil(COIL_DESLIGAR, [](TRegister* reg, uint16_t val) {
        if (val) {  // Se coil for ativada
            digitalWrite(LED_PIN, LOW);           // Desliga LED
            mb.Coil(COIL_STATUS_LED, false);      // Atualiza status do LED
            Serial.println("Desligado via coil 00002");
            mb.Coil(COIL_DESLIGAR, false);        // Reseta coil
        }
        return val;
    });
}

void loop() {
    mb.task();  // Processa requisicoes Modbus
    
    // Leitura periodica do potenciometro
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate >= 500) {  // A cada 500ms
        lastUpdate = millis();
        uint16_t val = analogRead(POT_PIN);  // Le valor do potenciometro
        
        // Atualiza registrador se diferenca maior que 5 (para evitar ruido)
        if (abs(val - mb.Hreg(HREG_POT)) > 5) {
            mb.Hreg(HREG_POT, val);  // Atualiza registrador
            Serial.println("Pot: " + String(val));
        }
    }

    // Leitura do botao com debounce
    bool reading = digitalRead(BUTTON_PIN);  // Le estado atual do botao
    
    if (reading != lastButtonState) {  // Se estado mudou
        lastDebounceTime = millis();   // Reseta tempo do debounce
    }
    
    // Apos tempo de debounce, atualiza coil do botao
    if ((millis() - lastDebounceTime) > debounceDelay) {
        mb.Coil(COIL_BOTAO_D0, !reading);  // Inverte logica (pull-up)
    }
    
    lastButtonState = reading;  // Atualiza ultimo estado do botao
    delay(10);  // Pequeno delay para estabilidade
}