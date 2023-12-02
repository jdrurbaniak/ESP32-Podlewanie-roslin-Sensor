/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/?s=esp-now
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Based on JC Servaye example: https://github.com/Servayejc/esp_now_sender/
*/

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <vector>

// Set your Board and Server ID 
#define BOARD_ID 2
#define MAX_CHANNEL 13  // for North America // 13 in Europe
#define MOISTURE_SENSOR_PIN 32
#define WATER_PUMP_PIN 25

uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct struct_outgoing_message {
  uint8_t msgType;
  uint8_t id;
  float humidity;
  unsigned int readingId;
} struct_outgoing_message;

typedef struct struct_incoming_message {
  uint8_t msgType;
  float minimumMoistureLevel;
  float maximumMoistureLevel;
} struct_incoming_message;

typedef struct struct_pairing {       // new structure for pairing
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
} struct_pairing;

struct_outgoing_message myData;
struct_incoming_message inData;
struct_pairing pairingData;

enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType {PAIRING, DATA,};
MessageType messageType;

#ifdef SAVE_CHANNEL
  int lastChannel;
#endif  
int channel = 1;
 
float h = 0.0;

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 1800000;        // Interval at which to publish sensor readings - 30min
bool isReportRateIncreased = true; // Kiedy dopiero co podlaliśmy roślinę 
// #define NO_PUMP
#ifdef NO_PUMP
  const long increasedReportRate = interval;
#else
  const long increasedReportRate = 60000; // 1 min
#endif

const int delayBetweenReadings = 10;
const float numberOfReadings = 10.0;
unsigned long start;                // used to measure Pairing time
unsigned int readingId = 0;   
int wateringDurationMs = 1000;
float moistureTresholdPercentage = 50.0;
float maximumMoistureLevel = 55.0;

const float HumidityAirValue = 2650.0;
const float HumidityWaterValue = 1400.0;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readHumidity() {
  std::vector<float> readings;
  float readingsSum = 0.0;
  for(int i = 0; i < numberOfReadings; i++)
  {
    float reading = analogRead(MOISTURE_SENSOR_PIN);
    readings.push_back(reading);
    readingsSum += reading;
    delay(delayBetweenReadings);
  }

  float deviationSum = 0.0;
  float readingsAverage = readingsSum / numberOfReadings;
  std::vector<float> deviations;

  for(int i = 0; i < numberOfReadings; i++)
  {
    float deviation = abs(readings.at(i) - readingsAverage);
    deviationSum += deviation;
    deviations.push_back(deviation);
  }

  float averageDeviation = deviationSum / numberOfReadings;

  readingsSum = 0.0;
  float numberOfValidReadings = 0.0;

  for(int i = 0; i < numberOfReadings; i++)
  {
    if(deviations.at(i) < 2.0*averageDeviation)
    {
      readingsSum += readings.at(i);
      numberOfValidReadings += 1.0;
    }
  }

  Serial.print("Avg deviation: ");
  Serial.println(averageDeviation);
  Serial.print("Valid readings: ");
  Serial.println(numberOfValidReadings);
  
  if(numberOfValidReadings == 0)
  {
    h = 0.0;
  }
  else
  {
    h = mapFloat(readingsSum/numberOfValidReadings, HumidityAirValue, HumidityWaterValue, 0.0, 100.0);
    h = constrain(h, 0.0, 100.0);
  }
  
  Serial.println(h);
  return h;
}

void waterPlant(int wateringDuration, uint8_t pumpPin)
{
  digitalWrite(pumpPin, LOW);
  Serial.println("LOW");
  delay(wateringDuration);
  digitalWrite(pumpPin, HIGH);
  Serial.println("HIGH");
}

void addPeer(const uint8_t * mac_addr, uint8_t chan){
  esp_now_peer_info_t peer;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  Serial.println();
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0];
  switch (type) {
  case DATA :      // we received data from server
    memcpy(&inData, incomingData, sizeof(inData));
    Serial.print("SetPoint minimumMoistureLevel = ");
    Serial.println(inData.minimumMoistureLevel);
    moistureTresholdPercentage = inData.minimumMoistureLevel;
    Serial.print("SetPoint maxiumumMoistureLevel = ");
    Serial.println(inData.maximumMoistureLevel);
    maximumMoistureLevel = inData.maximumMoistureLevel;
    break;

  case PAIRING:    // we received pairing data from server
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if (pairingData.id == 0) {              // the message comes from server
      printMAC(mac_addr);
      Serial.print("Pairing done for ");
      printMAC(pairingData.macAddr);
      Serial.print(" on channel " );
      Serial.print(pairingData.channel);    // channel used by the server
      Serial.print(" in ");
      Serial.print(millis()-start);
      Serial.println("ms");
      addPeer(pairingData.macAddr, pairingData.channel); // add the server  to the peer list 
      #ifdef SAVE_CHANNEL
        lastChannel = pairingData.channel;
        EEPROM.write(0, pairingData.channel);
        EEPROM.commit();
      #endif  
      pairingStatus = PAIR_PAIRED;             // set the pairing status
    }
    break;
  }  
}

PairingStatus autoPairing(){
  switch(pairingStatus) {
    case PAIR_REQUEST:
    Serial.print("Pairing request on channel "  );
    Serial.println(channel);

    // set WiFi channel   
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
    }

    // set callback routines
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
  
    // set pairing data to send to the server
    pairingData.msgType = PAIRING;
    pairingData.id = BOARD_ID;     
    pairingData.channel = channel;

    // add peer and send request
    addPeer(serverAddress, channel);
    esp_now_send(serverAddress, (uint8_t *) &pairingData, sizeof(pairingData));
    previousMillis = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis();
    if(currentMillis - previousMillis > 250) {
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel ++;
      if (channel > MAX_CHANNEL){
         channel = 1;
      }   
      pairingStatus = PAIR_REQUEST;
    }
    break;

    case PAIR_PAIRED:
      // nothing to do here 
    break;
  }
  return pairingStatus;
}  

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(WATER_PUMP_PIN, OUTPUT);
  digitalWrite(WATER_PUMP_PIN, LOW);
  digitalWrite(WATER_PUMP_PIN, HIGH);
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  start = millis();

  #ifdef SAVE_CHANNEL 
    EEPROM.begin(10);
    lastChannel = EEPROM.read(0);
    Serial.println(lastChannel);
    if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL) {
      channel = lastChannel; 
    }
    Serial.println(channel);
  #endif  
  pairingStatus = PAIR_REQUEST;
}  

long currentInterval;

void loop() {
  if (autoPairing() == PAIR_PAIRED) {
    unsigned long currentMillis = millis();
    currentInterval = (isReportRateIncreased == true) ? increasedReportRate : interval;
    if (currentMillis - previousMillis >= currentInterval) {
      // Save the last time a new reading was published
      previousMillis = currentMillis;
      //Set values to send
      myData.msgType = DATA;
      myData.id = BOARD_ID;
      myData.humidity = readHumidity();
      myData.readingId = readingId++;
      esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &myData, sizeof(myData));
      if(myData.humidity < moistureTresholdPercentage)
      {
        if(myData.humidity != 0)
        waterPlant(wateringDurationMs, WATER_PUMP_PIN);
        isReportRateIncreased = true;
      }
      else if(myData.humidity >= maximumMoistureLevel)
      {
        isReportRateIncreased = false;
      }
      else
      {
        waterPlant(wateringDurationMs, WATER_PUMP_PIN);
      }
    }
  }
}