#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

// Broker MQTT
const char* mqtt_server = "rmq2.pptik.id";
const int mqtt_port = 1883;
const char* mqtt_username = "/smart-meter:smart-meter";
const char* mqtt_password = "MkeF65py";
const char* mqtt_clientid = "esp32_client";

// GUID
const char* device_guid = "725a76fd-6d7c-9475-6431-499e432d2d0a";

WiFiClient espClient;
PubSubClient client(espClient);

volatile int flow_frequency; // Mengukur pulsa sensor aliran
float vol = 0.0; // Volume total
unsigned char flowsensor = 34; // Input Sensor
unsigned long currentTime;
unsigned long cloopTime;
float volume_per_pulse = 0.00222;

const char* reset_topic = "Log";
const char* feedback_topic = "feedback";

#define EEPROM_SIZE 512
#define VOLUME_ADDR 0
#define CHECKSUM_ADDR (VOLUME_ADDR + sizeof(float))

const int resetPin = 15; // Pin untuk pushbutton
const int relayPin = 5; // Pin untuk relay (D5)
unsigned long buttonPressTime = 0;
bool buttonPressed = false;

void IRAM_ATTR flow() {
  flow_frequency++;
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);
  WiFiManager wm;
  bool res;

  res = wm.autoConnect("SMART-METER", "12345678"); // AP dengan password

  if (!res) {
    Serial.println("Failed to connect");
  } else {
    Serial.println("connected...yeey :)");
  }

  delay(10);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String incomingMessage;
  for (unsigned int i = 0; i < length; i++) {
    incomingMessage += (char)payload[i];
  }
  Serial.print("Pesan diterima [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(incomingMessage);

  if (String(topic) == reset_topic && incomingMessage == "reset data") {
    vol = 0.0;
    Serial.println("Volume direset menjadi 0");
    client.publish(feedback_topic, "Reset berhasil", true);

    saveVolumeToEEPROM();
  }

  // Kontrol relay berdasarkan pesan MQTT
  if (String(topic) == "Log" && incomingMessage.startsWith(device_guid)) {
    int value = incomingMessage.substring(37).toInt(); // Ekstraksi nilai dari pesan (diasumsikan device_guid panjangnya 37 karakter)
    if (value == 1) {
      digitalWrite(relayPin, LOW); // Matikan relay
      Serial.println("Relay dimatikan");
    } else if (value == 0) {
      digitalWrite(relayPin, HIGH); // Nyalakan relay
      Serial.println("Relay dinyalakan");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Mencoba menghubungkan ke MQTT...");
    if (client.connect(mqtt_clientid, mqtt_username, mqtt_password)) {
      Serial.println("terhubung");
      client.subscribe(reset_topic);
      client.subscribe("Log");
    } else {
      Serial.print("gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

uint8_t calculateChecksum(float volume) {
  uint8_t* p = (uint8_t*)&volume;
  uint8_t checksum = 0;
  for (int i = 0; i < sizeof(float); i++) {
    checksum ^= p[i];
  }
  return checksum;
}

void saveVolumeToEEPROM() {
  EEPROM.put(VOLUME_ADDR, vol);
  uint8_t checksum = calculateChecksum(vol);
  EEPROM.put(CHECKSUM_ADDR, checksum);
  EEPROM.commit();
  Serial.println("Volume tersimpan ke EEPROM");
}

bool readVolumeFromEEPROM() {
  float storedVolume;
  uint8_t storedChecksum, calculatedChecksum;
  EEPROM.get(VOLUME_ADDR, storedVolume);
  EEPROM.get(CHECKSUM_ADDR, storedChecksum);
  calculatedChecksum = calculateChecksum(storedVolume);
  
  if (storedChecksum == calculatedChecksum) {
    vol = storedVolume;
    Serial.print("Volume tersimpan di EEPROM: ");
    Serial.println(storedVolume);
    return true;
  } else {
    Serial.println("Checksum mismatch, data mungkin korup.");
    return false;
  }
}

void sendVolumeFromEEPROM() {
  readVolumeFromEEPROM(); // Pastikan nilai terbaru dibaca dari EEPROM
  String message = "EEPROM : " + String(vol);
  char mqtt_topic[70];
  snprintf(mqtt_topic, sizeof(mqtt_topic), "Log");
  client.publish(mqtt_topic, message.c_str(), true);
  Serial.print("Mengirimkan data dari EEPROM ke MQTT: ");
  Serial.println(message);
}

void checkResetButton() {
  if (digitalRead(resetPin) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressTime = millis();
      Serial.println("Tombol ditekan");
    } else {
      if (millis() - buttonPressTime >= 5000) {
        Serial.println("Mereset pengaturan WiFi");
        WiFiManager wm;
        wm.resetSettings();
        ESP.restart();
      }
    }
  } else {
    if (buttonPressed) {
      Serial.println("Tombol dilepas sebelum 5 detik");
    }
    buttonPressed = false;
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  WiFi.mode(WIFI_STA);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);

  pinMode(relayPin, OUTPUT); // Inisialisasi pin relay sebagai output
  digitalWrite(relayPin, HIGH); // Matikan relay saat boot

  attachInterrupt(digitalPinToInterrupt(flowsensor), flow, RISING);
  currentTime = millis();
  cloopTime = currentTime;
  pinMode(resetPin, INPUT_PULLUP);

  if (!readVolumeFromEEPROM()) {
    vol = 0.0; // Jika data di EEPROM rusak, reset volume ke 0
  }
  sendVolumeFromEEPROM();
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  currentTime = millis();
  if (currentTime >= (cloopTime + 5000)) {
    cloopTime = currentTime;
    if (flow_frequency != 0) {
      vol += (flow_frequency * volume_per_pulse);
      Serial.print("Volume: ");
      Serial.print(vol);
      Serial.println(" LITER");

      // Membuat objek JSON
      StaticJsonDocument<200> jsonDocument;
      jsonDocument["guidDevice"] = device_guid;

      // Konversi nilai volume ke string dengan 2 desimal
      char volumeString[10];
      dtostrf(vol, 1, 2, volumeString);
      jsonDocument["Volume"] = volumeString;

      // Serialize objek JSON ke string
      char jsonBuffer[512];
      serializeJson(jsonDocument, jsonBuffer);

      // Publish pesan MQTT dengan objek JSON sebagai payload
      client.publish("Log", jsonBuffer, true);

      flow_frequency = 0;

      saveVolumeToEEPROM(); // Simpan volume yang diperbarui ke EEPROM
    } else {
      Serial.println("Volume: 0 LITER");
    }
  }

  checkResetButton();
}
