#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>

const IPAddress ip(192, 168, 100, 120);
const IPAddress gateway(192, 168, 100, 1);
const IPAddress subnet(255, 255, 255, 0);
const char* ssid = "casa.exe";
const char* password = "AfxJ457T";

const uint16_t COIL_LIGAR = 0;
const uint16_t COIL_DESLIGAR = 1;
const uint16_t COIL_STATUS_LED = 2;
const uint16_t COIL_BOTAO_D0 = 3;
const uint16_t HREG_POT = 0;

const uint8_t LED_PIN = D2;
const uint8_t POT_PIN = A0;
const uint8_t BUTTON_PIN = D0;

ModbusIP mb;

bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
    Serial.begin(115200);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    WiFi.config(ip, gateway, subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConectado! IP: " + WiFi.localIP().toString());

    mb.server();
    
    mb.addCoil(COIL_LIGAR);
    mb.addCoil(COIL_DESLIGAR);
    mb.addCoil(COIL_STATUS_LED);
    mb.addCoil(COIL_BOTAO_D0);
    mb.addHreg(HREG_POT);

    mb.onSetCoil(COIL_LIGAR, [](TRegister* reg, uint16_t val) {
        if (val) {
            digitalWrite(LED_PIN, HIGH);
            mb.Coil(COIL_STATUS_LED, true);
            Serial.println("Ligado via coil 00001");
            mb.Coil(COIL_LIGAR, false);
        }
        return val;
    });

    mb.onSetCoil(COIL_DESLIGAR, [](TRegister* reg, uint16_t val) {
        if (val) {
            digitalWrite(LED_PIN, LOW);
            mb.Coil(COIL_STATUS_LED, false);
            Serial.println("Desligado via coil 00002");
            mb.Coil(COIL_DESLIGAR, false);
        }
        return val;
    });
}

void loop() {
    mb.task();
    
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate >= 500) {
        lastUpdate = millis();
        uint16_t val = analogRead(POT_PIN);
        
        if (abs(val - mb.Hreg(HREG_POT)) > 5) {
            mb.Hreg(HREG_POT, val);
            Serial.println("Pot: " + String(val));
        }
    }

    bool reading = digitalRead(BUTTON_PIN);
    
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
        mb.Coil(COIL_BOTAO_D0, !reading);
    }
    
    lastButtonState = reading;
    delay(10);
}