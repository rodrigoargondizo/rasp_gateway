#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>

// ========== CONFIGURAÇÃO ==========
const IPAddress ip(192, 168, 100, 120);
const IPAddress gateway(192, 168, 100, 1);
const IPAddress subnet(255, 255, 255, 0);
const char* ssid = "casa.exe";
const char* password = "AfxJ457T";

// Endereços Modbus (0-based)
const uint16_t COIL_LIGAR = 0;      // Endereço 00001
const uint16_t COIL_DESLIGAR = 1;   // Endereço 00002
const uint16_t COIL_STATUS_LED = 2;  // Endereço 00003 (Status do LED)
const uint16_t COIL_BOTAO_D0 = 3;   // Endereço 00004 (Estado do botão - Lógica invertida: 1=botão solto, 0=botão pressionado)
const uint16_t HREG_POT = 0;        // Endereço 40001

// Pinos
const uint8_t LED_PIN = D2;         // LED 
const uint8_t POT_PIN = A0;
const uint8_t BUTTON_PIN = D0;      // Botão físico

ModbusIP mb;

// Variáveis para debounce do botão
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);     // Inicia desligado
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Botão com pull-up interno

    // WiFi
    WiFi.config(ip, gateway, subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500);
    Serial.println("Conectado! IP: " + WiFi.localIP().toString());

    // Configuração Modbus
    mb.server();
    mb.addCoil(COIL_LIGAR);
    mb.addCoil(COIL_DESLIGAR);
    mb.addCoil(COIL_STATUS_LED);    // Coil de status do LED
    mb.addCoil(COIL_BOTAO_D0);      // Coil para leitura do botão
    mb.addHreg(HREG_POT);

    // Callbacks para as coils
    mb.onSetCoil(COIL_LIGAR, [](TRegister* reg, uint16_t val) {
        if (val) {  // Só reage quando escreverem 1
            digitalWrite(LED_PIN, HIGH);  // Liga
            mb.Coil(COIL_STATUS_LED, true); // Atualiza status
            Serial.println("Ligado via coil 00001");
            mb.Coil(COIL_LIGAR, false);  // Auto-reset
        }
        return val;
    });

    mb.onSetCoil(COIL_DESLIGAR, [](TRegister* reg, uint16_t val) {
        if (val) {  // Só reage quando escreverem 1
            digitalWrite(LED_PIN, LOW);  // Desliga
            mb.Coil(COIL_STATUS_LED, false); // Atualiza status
            Serial.println("Desligado via coil 00002");
            mb.Coil(COIL_DESLIGAR, false);  // Auto-reset
        }
        return val;
    });
}

void loop() {
    mb.task();
    
    // Atualiza potenciômetro
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate >= 500) {
        lastUpdate = millis();
        uint16_t val = analogRead(POT_PIN);
        if (abs(val - mb.Hreg(HREG_POT)) > 5) {
            mb.Hreg(HREG_POT, val);
            Serial.println("Pot: " + String(val));
        }
    }

    // Leitura do botão com debounce
    bool reading = digitalRead(BUTTON_PIN);
    
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // Atualiza o registrador Modbus com o estado do botão (invertido por causa do PULLUP)
        mb.Coil(COIL_BOTAO_D0, !reading); // ! porque PULLUP (LOW quando pressionado)
    }
    
    lastButtonState = reading;
    delay(10);
}