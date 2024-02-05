// Based on JC Servaye example: https://github.com/Servayejc/esp_now_sender/

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <vector>

#define MAX_CHANNEL 13
#define MOISTURE_SENSOR_PIN 32
#define WATER_PUMP_PIN 25

uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct struct_outgoing_message {
  uint8_t msgType;
  float humidity;
} struct_outgoing_message;

typedef struct struct_incoming_message {
  uint8_t msgType;
  float minimumMoistureLevel;
  float maximumMoistureLevel;
} struct_incoming_message;

typedef struct struct_pairing {
    uint8_t msgType;
    uint8_t macAddr[6];
    uint8_t channel;
} struct_pairing;

struct_outgoing_message myData;
struct_incoming_message inData;
struct_pairing pairingData;

enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
RTC_DATA_ATTR PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType {PAIRING, DATA,};
MessageType messageType;

#ifdef SAVE_CHANNEL
  int lastChannel;
#endif  
RTC_DATA_ATTR int channel = 1;
 
float h = 0.0;

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;   // Stores last time temperature was published
unsigned int deepSleepDelay = 5000;
bool isDeepSleepScheduled = false;
const long interval = 1800000;        // Interval at which to publish sensor readings - 30min
RTC_DATA_ATTR bool isReportRateIncreased = false; // Kiedy dopiero co podlaliśmy roślinę 
// #define NO_PUMP
#ifdef NO_PUMP
  const long increasedReportRate = interval;
#else
  const long increasedReportRate = 1200000; // 20 min
#endif

const int delayBetweenReadings = 10;
const float numberOfReadings = 10.0;
int wateringDurationMs = 3000;
RTC_DATA_ATTR float moistureTresholdPercentage = 50.0;
RTC_DATA_ATTR float maximumMoistureLevel = 55.0;

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

long currentInterval;

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Received data from ");
  printMAC(mac_addr);
  Serial.println();
  uint8_t type = incomingData[0];
  switch (type) {
  case DATA :
    memcpy(&inData, incomingData, sizeof(inData));
    Serial.print("minimumMoistureLevel = ");
    Serial.println(inData.minimumMoistureLevel);
    moistureTresholdPercentage = inData.minimumMoistureLevel;
    Serial.print("maxiumumMoistureLevel = ");
    Serial.println(inData.maximumMoistureLevel);
    maximumMoistureLevel = inData.maximumMoistureLevel;
    Serial.println("Going to sleep");
    esp_sleep_enable_timer_wakeup(currentInterval*1000);
    esp_deep_sleep_start();
    break;

  case PAIRING:
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    printMAC(mac_addr);
    Serial.println("Paired MAC: ");
    printMAC(pairingData.macAddr);
    Serial.print("Channel: " );
    Serial.print(pairingData.channel);
    addPeer(pairingData.macAddr, pairingData.channel);
    #ifdef SAVE_CHANNEL
      lastChannel = pairingData.channel;
      EEPROM.write(0, pairingData.channel);
      EEPROM.commit();
    #endif  
    pairingStatus = PAIR_PAIRED;
    break;
  }  
}

const uint8_t maxPairingAttempts = 5;
int pairingAttempts = 0;

PairingStatus autoPairing(){
  switch(pairingStatus) {
    case PAIR_REQUEST:
    Serial.print("Pairing request on channel "  );
    Serial.println(channel);
  
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
  
    pairingData.msgType = PAIRING;
    pairingData.channel = channel;

    addPeer(serverAddress, channel);
    esp_now_send(serverAddress, (uint8_t *) &pairingData, sizeof(pairingData));
    previousMillis = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
      currentMillis = millis();
      if(currentMillis - previousMillis > 250) {
        previousMillis = currentMillis;
        channel ++;
        if (channel > MAX_CHANNEL){
          channel = 1;
          pairingAttempts++;
        }   
        pairingStatus = PAIR_REQUEST;
      }
    break;

    case PAIR_PAIRED:
    break;
  }
  return pairingStatus;
}  

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("RTC pairingStatus: ");
  Serial.println(pairingStatus);
  Serial.print("RTC channel: ");
  Serial.println(channel);
  Serial.print("RTC isReportRateIncreased: ");
  Serial.println(isReportRateIncreased);
  pinMode(WATER_PUMP_PIN, OUTPUT);
  digitalWrite(WATER_PUMP_PIN, LOW);
  digitalWrite(WATER_PUMP_PIN, HIGH);
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

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

void loop() {
  if(isDeepSleepScheduled == false)
  {
    if (autoPairing() == PAIR_PAIRED || pairingAttempts > maxPairingAttempts) {
      currentInterval = (isReportRateIncreased == true) ? increasedReportRate : interval;
      
      myData.msgType = DATA;
      myData.humidity = readHumidity();
      if(myData.humidity < moistureTresholdPercentage)
      {
        isReportRateIncreased = true;
        if(myData.humidity != 0)
        waterPlant(wateringDurationMs, WATER_PUMP_PIN);
      }
      else if(myData.humidity >= maximumMoistureLevel)
      {
        isReportRateIncreased = false;
      }
      else if(isReportRateIncreased == true)
      {
        waterPlant(wateringDurationMs, WATER_PUMP_PIN);
      }
      esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &myData, sizeof(myData));
      if(result == ESP_OK && pairingStatus == PAIR_PAIRED)
      {
        Serial.print("Waiting for data for ");
        Serial.print(deepSleepDelay);
        Serial.println("ms");
        isDeepSleepScheduled = true;
        previousMillis = millis();
      }
      else
      {
        Serial.println("Going to sleep");
        esp_sleep_enable_timer_wakeup(currentInterval*1000);
        esp_deep_sleep_start();
      }
    }
  }
  else
  {
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= deepSleepDelay)
    {
      Serial.println("Going to sleep");
      esp_sleep_enable_timer_wakeup(currentInterval*1000);
      esp_deep_sleep_start();
    }
  }
}