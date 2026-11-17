/*
  ECG sur ESP32-S3 (ESP32-S3-Zero) : lit la sortie de l'ampli sur GPIO1 et
  l'envoie en Bluetooth Low Energy vers la page web ecg_live.html.

  ALIMENTATION
    ESP32 : câble USB-C branché sur un chargeur 5 V (ou une batterie externe).
            Aucun lien de données avec le PC : tout passe en BLE.
    Ampli : MCP6004 alimenté par la broche 3V3 de l'ESP32
            (3V3 -> pin 4 du MCP6004, GND -> pin 11). Donc Vref = 1,65 V.

  CÂBLAGE DU SIGNAL
    Pin 8 (sortie AOP 3)  ->  1 kOhm  ->  GPIO1  ->  100 nF -> GND
    GND de l'ampli        ->  GND de l'ESP32      (masse commune obligatoire)
    Ne jamais dépasser 3,3 V sur une broche de l'ESP32.

  PROTOCOLE (notification BLE, 20 octets) :
    octets 0-1  : numéro de paquet (uint16, little-endian)
    octets 2-19 : 9 échantillons uint16 little-endian, en dixièmes de mV
                  (1650,0 mV est envoyé 16500)
    Fréquence : 250 échantillons/s (ADC lu à 1 kHz, 8 lectures moyennées,
    puis moyenne de 4 points consécutifs).
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const int      ADC_PIN     = 2;                // GPIO1 = ADC1
const uint32_t TICK_US     = 1000;             // 1 kHz
const int      N_READS     = 8;                // lectures ADC moyennées par tick
const int      TICKS_PER_SAMPLE = 4;           // 1 kHz / 4 = 250 Hz
const int      SAMPLES_PER_PACKET = 9;         // 2 + 9*2 = 20 octets

BLECharacteristic *ecgChar = nullptr;
volatile bool connected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { connected = true; }
  void onDisconnect(BLEServer *s) override {
    connected = false;
    BLEDevice::startAdvertising();             // redevient visible pour se reconnecter
  }
};

void setup() {
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);  // plage ~0 à 3,1 V

  BLEDevice::init("ECG-ESP32");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  ecgChar = service->createCharacteristic(CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  ecgChar->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  static uint32_t next = micros();
  static float acc = 0;
  static int tickCount = 0;
  static uint16_t seq = 0;
  static uint8_t pkt[20];
  static int nInPkt = 0;

  uint32_t now = micros();
  if ((int32_t)(now - next) < 0) {             // pas encore l'heure du prochain tick
    yield();
    return;
  }
  next += TICK_US;                             // cadence fixe, sans dérive

  uint32_t s = 0;
  for (int i = 0; i < N_READS; i++) s += analogReadMilliVolts(ADC_PIN);
  acc += s / (float)N_READS;

  if (++tickCount < TICKS_PER_SAMPLE) return;
  float mv = acc / TICKS_PER_SAMPLE;           // un échantillon à 250 Hz
  acc = 0;
  tickCount = 0;

  uint16_t v = (uint16_t)constrain((int)lroundf(mv * 10.0f), 0, 65535);
  pkt[2 + 2 * nInPkt]     = v & 0xFF;
  pkt[2 + 2 * nInPkt + 1] = v >> 8;

  if (++nInPkt == SAMPLES_PER_PACKET) {
    nInPkt = 0;
    if (connected) {
      pkt[0] = seq & 0xFF;
      pkt[1] = seq >> 8;
      ecgChar->setValue(pkt, sizeof(pkt));
      ecgChar->notify();
    }
    seq++;
  }
}
