#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <EasyNextionLibrary.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin Definitions
#define PH_SENSOR_PIN     36
#define ORP_SENSOR_PIN    39
#define TDS_SENSOR_PIN    34
#define DO_SENSOR_PIN     35
#define TEMP_SENSOR_PIN   32

#define RX_PIN_GSM 16
#define TX_PIN_GSM 17
#define RST_PIN 15

#define RX_PIN_NEXTION 9
#define TX_PIN_NEXTION 10

// Network Credentials
const char apn[] = "internet";
const char usernamemqtt[] = "water_quality";
const char passmqtt[] = "f98tYucDJ54HgXwa";
const char broker[] = "112.78.33.179";
const int brokerPort = 1883;

OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// Sensor Calibration Structure
struct SensorCalibration {
    float phOffset = 0;
    float orpOffset = 0;
    float tdsOffset = -228;
    float doOffset = 0;
    float tempOffset = 2.9;
};

// Averaged Sensor Data Structure
struct AveragedSensorData {
    float ph[12];       // 12 * 5 seconds = 60 seconds
    float orp[12];
    float tds[12];
    float doValue[12];
    float temperature[12];
    int currentIndex = 0;
    int dataCount = 0;
};

// Sensor Data Structure
typedef struct {
    float ph;
    float orp;
    float tds;
    float doValue;
    float temperature;
    bool isConnected;
} SensorData;

// Status sensor
struct SensorStatus {
    enum Status {
        ERROR,
        WORKING,
        BELOW,
        ABOVE
    };
    
    Status phStatus = ERROR;
    Status tdsStatus = ERROR;
    Status orpStatus = ERROR;
    Status doStatus = ERROR;
    Status tempStatus = ERROR;
};
struct TimeOffset {
    int32_t secondsOffset = 0;  // Offset dalam detik
    bool isTimeSet = false;     // Flag untuk menandai apakah waktu sudah diset
} timeOffset;

// Global Variables
SensorCalibration sensorCal;
AveragedSensorData averagedData;
SensorData xSensorData;
SensorStatus sensorStatus;

// Hardware Serial Interfaces
HardwareSerial SerialAT(1);     // For SIM800
HardwareSerial SerialNextion(2);// For Nextion

// GSM and MQTT Objects
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Nextion Display Object
EasyNex myNextion(SerialNextion);

// Semaphores for Resource Synchronization
SemaphoreHandle_t xMQTTMutex;
SemaphoreHandle_t xSerialMutex;

// Status Variables
String mqttStatus = "Disconnected";

// Task Handles
TaskHandle_t xTaskSensorRead = NULL;
TaskHandle_t xTaskNextionUpdate = NULL;
TaskHandle_t xTaskMQTTPublish = NULL;

// Task Delays
const TickType_t xSensorDelay = pdMS_TO_TICKS(5000);      // 5 detik
const TickType_t xNextionDelay = pdMS_TO_TICKS(10000);    // 10 detik
const TickType_t xMQTTDelay = pdMS_TO_TICKS(5000);       // 60 detik

// Function Prototypes
int readAnalogStable(int pin);
bool checkSensor(int pin, float value, float minValue, float maxValue);
float adcToVoltage(int adcValue);
float readPH();
float readTDS();
float readORP();
float readDO();
float readTemperature();
float calculateAverage(float* data, int count);
String generateTimestamp();
String getDayName(int dayIndex);
void vTaskSensorRead(void *pvParameters);
void vTaskNextionUpdate(void *pvParameters);
void vTaskMQTTPublish(void *pvParameters);
void setupGSMAndGPRS();

// Function Definitions
int readAnalogStable(int pin) {
    int samples[10];
    for (int i = 0; i < 10; i++) {
        samples[i] = analogRead(pin);
        delay(10);
    }
    
    // Sort samples
    for (int i = 0; i < 9; i++) {
        for (int j = i + 1; j < 10; j++) {
            if (samples[i] > samples[j]) {
                int temp = samples[i];
                samples[i] = samples[j];
                samples[j] = temp;
            }
        }
    }
    
    return samples[5]; // Return median
}

bool checkSensor(int pin, float value, float minValue, float maxValue) {
    int rawValue = readAnalogStable(pin);
    // Cek apakah pembacaan ADC dalam range normal (tidak 0 atau max ADC)
    if (rawValue <= 0 || rawValue >= 4095) return false;
    // Cek apakah nilai yang dikonversi masuk akal
    if (value < minValue || value > maxValue) return false;
    return true;
}

float adcToVoltage(int adcValue) {
    return adcValue * (3.3 / 4095.0);
}

float readPH() {
    int rawValue = readAnalogStable(PH_SENSOR_PIN);
    
    // Cek koneksi sensor
    if (rawValue <= 0 || rawValue >= 4095) {
        sensorStatus.phStatus = SensorStatus::ERROR;
        return 0;
    }

    float voltage = adcToVoltage(rawValue);
    float phValue = 7.0 + ((2.5 - voltage) / 0.6) + sensorCal.phOffset;
    
    // Update status berdasarkan range
    if (phValue < 6.5) {
        sensorStatus.phStatus = SensorStatus::BELOW;
    } else if (phValue > 8.5) {
        sensorStatus.phStatus = SensorStatus::ABOVE;
    } else {
        sensorStatus.phStatus = SensorStatus::WORKING;
    }
    
    return constrain(phValue, 0, 14);
}

float readTDS() {
    int rawValue = readAnalogStable(TDS_SENSOR_PIN);
    
    // Cek koneksi sensor
    if (rawValue <= 0 || rawValue >= 4095) {
        sensorStatus.tdsStatus = SensorStatus::ERROR;
        return 0;
    }

    float voltage = adcToVoltage(rawValue);
    float tdsValue = (165.45 * voltage * voltage * voltage) - (315.78 * voltage * voltage) + (1102.92 * voltage);
    float finalValue = tdsValue + sensorCal.tdsOffset;
    
    // Update status berdasarkan range
    if (finalValue < 50) {
        sensorStatus.tdsStatus = SensorStatus::BELOW;
    } else if (finalValue > 500) {
        sensorStatus.tdsStatus = SensorStatus::ABOVE;
    } else {
        sensorStatus.tdsStatus = SensorStatus::WORKING;
    }
    
    return finalValue;
}

float readORP() {
    int rawValue = readAnalogStable(ORP_SENSOR_PIN);
    
    // Cek koneksi sensor
    if (rawValue <= 0 || rawValue >= 4095) {
        sensorStatus.orpStatus = SensorStatus::ERROR;
        return 0;
    }

    float voltage = adcToVoltage(rawValue);
    float orpValue = ((voltage * 1000) - 1036) / 2.5;
    float finalValue = orpValue + sensorCal.orpOffset;
    
    // Update status berdasarkan range
    if (finalValue < 300) {
        sensorStatus.orpStatus = SensorStatus::BELOW;
    } else if (finalValue > 500) {
        sensorStatus.orpStatus = SensorStatus::ABOVE;
    } else {
        sensorStatus.orpStatus = SensorStatus::WORKING;
    }
    
    return finalValue;
}

float readDO() {
    int rawValue = readAnalogStable(DO_SENSOR_PIN);
    
    // Cek koneksi sensor
    if (rawValue <= 0 || rawValue >= 4095) {
        sensorStatus.doStatus = SensorStatus::ERROR;
        return 0;
    }

    float voltage = adcToVoltage(rawValue);
    float doValue = voltage * 100;
    float finalValue = doValue + sensorCal.doOffset;
    
    // Update status berdasarkan range
    if (finalValue < 6.5) {
        sensorStatus.doStatus = SensorStatus::BELOW;
    } else if (finalValue > 8.0) {
        sensorStatus.doStatus = SensorStatus::ABOVE;
    } else {
        sensorStatus.doStatus = SensorStatus::WORKING;
    }
    
    return finalValue;
}

float readTemperature() {
    tempSensor.requestTemperatures(); // Send the command to get temperatures
    float temperature = tempSensor.getTempCByIndex(0);  // Get temperature in Celsius
    
    // Check if reading was successful
    if (temperature == DEVICE_DISCONNECTED_C) {
        sensorStatus.tempStatus = SensorStatus::ERROR;
        return 0;
    }

    float finalValue = temperature + sensorCal.tempOffset;
    
    // Update status based on range
    if (finalValue < 10) {
        sensorStatus.tempStatus = SensorStatus::BELOW;
    } else if (finalValue > 30) {
        sensorStatus.tempStatus = SensorStatus::ABOVE;
    } else {
        sensorStatus.tempStatus = SensorStatus::WORKING;
    }
    
    return finalValue;
}

float calculateAverage(float* data, int count) {
    if (count == 0) return 0;
    
    float sum = 0;
    for (int i = 0; i < count; i++) {
        sum += data[i];
    }
    return sum / count;
}

String generateTimestamp() {
    time_t now = time(nullptr);
    struct tm* p_tm = localtime(&now);
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%02d-%02d-%d %02d:%02d:%02d", 
        p_tm->tm_mday, p_tm->tm_mon + 1, p_tm->tm_year + 1900, 
        p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);
    return String(buffer);
}

const char* dayNames[] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};

String getDayName(int dayIndex) {
    if (dayIndex >= 0 && dayIndex < 7) {
        return String(dayNames[dayIndex]); // Explicitly convert const char* to String
    }
    return String("Unknown"); // Return String instead of const char*
}
void setCustomTime(int year, int month, int day, int hour, int minute, int second) {
    struct tm timeinfo = {0};
    timeinfo.tm_year = year - 1900;  // Tahun dimulai dari 1900
    timeinfo.tm_mon = month - 1;     // Bulan dimulai dari 0
    timeinfo.tm_mday = day;
    timeinfo.tm_hour = hour;
    timeinfo.tm_min = minute;
    timeinfo.tm_sec = second;
    
    time_t now;
    time(&now);  // Mendapatkan waktu sistem saat ini
    
    // Hitung offset
    time_t targetTime = mktime(&timeinfo);
    timeOffset.secondsOffset = targetTime - now;
    timeOffset.isTimeSet = true;
}
time_t getAdjustedTime() {
    time_t now;
    time(&now);
    return timeOffset.isTimeSet ? now + timeOffset.secondsOffset : now;
}

void vTaskSensorRead(void *pvParameters) {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (;;) {
        // Read sensors
        float currentPH = readPH();
        float currentORP = readORP();
        float currentTDS = readTDS();
        float currentDO = readDO();
        float currentTemp = readTemperature();

        // Store in averaged data
        averagedData.ph[averagedData.currentIndex] = currentPH;
        averagedData.orp[averagedData.currentIndex] = currentORP;
        averagedData.tds[averagedData.currentIndex] = currentTDS;
        averagedData.doValue[averagedData.currentIndex] = currentDO;
        averagedData.temperature[averagedData.currentIndex] = currentTemp;

        // Update index and data count
        averagedData.currentIndex = (averagedData.currentIndex + 1) % 12;
        if (averagedData.dataCount < 12) {
            averagedData.dataCount++;
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 seconds interval
    }
}
String getStatusString(SensorStatus::Status status) {
    switch(status) {
        case SensorStatus::ERROR: return "Error";
        case SensorStatus::WORKING: return "Working";
        case SensorStatus::BELOW: return "Below";
        case SensorStatus::ABOVE: return "Above";
        default: return "Unknown";
    }
}

void vTaskNextionUpdate(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (;;) {
        // Get current time
        time_t adjustedNow = getAdjustedTime();
        struct tm* timeInfo = localtime(&adjustedNow);
        
        // Format strings for display
        String dayName = getDayName(timeInfo->tm_wday);
        char dateStr[20];
        char timeStr[20];
        
        // Format date as dd-mm-yyyy
        snprintf(dateStr, sizeof(dateStr), "%02d-%02d-%04d",
                 timeInfo->tm_mday, timeInfo->tm_mon + 1, timeInfo->tm_year + 1900);
        
        // Format time as hh:mm:ss
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
                 timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
        
        // Update time components on Nextion
        myNextion.writeStr("t0.txt", dayName);
        myNextion.writeStr("t1.txt", String(dateStr));
        myNextion.writeStr("t2.txt", String(timeStr));
        
        // Calculate sensor averages
        float avgPH = calculateAverage(averagedData.ph, averagedData.dataCount);
        float avgORP = calculateAverage(averagedData.orp, averagedData.dataCount);
        float avgTDS = calculateAverage(averagedData.tds, averagedData.dataCount);
        float avgDO = calculateAverage(averagedData.doValue, averagedData.dataCount);
        float avgTemp = calculateAverage(averagedData.temperature, averagedData.dataCount);
        
        // Update global sensor data
        xSensorData.ph = avgPH;
        xSensorData.orp = avgORP;
        xSensorData.tds = avgTDS;
        xSensorData.doValue = avgDO;
        xSensorData.temperature = avgTemp;
        
        // Update sensor values on Nextion
        myNextion.writeStr("t3.txt", String(avgPH, 2));
        myNextion.writeStr("t4.txt", String(avgTDS, 2));
        myNextion.writeStr("t5.txt", String(avgORP, 2));
        myNextion.writeStr("t6.txt", String(avgDO, 2));
        myNextion.writeStr("t7.txt", String(avgTemp, 2));
        
        // Update sensor status on Nextion
        myNextion.writeStr("t12.txt", getStatusString(sensorStatus.phStatus));
        myNextion.writeStr("t13.txt", getStatusString(sensorStatus.tdsStatus));
        myNextion.writeStr("t14.txt", getStatusString(sensorStatus.orpStatus));
        myNextion.writeStr("t15.txt", getStatusString(sensorStatus.doStatus));
        myNextion.writeStr("t16.txt", getStatusString(sensorStatus.tempStatus));
        
        vTaskDelay(xNextionDelay - pdMS_TO_TICKS(1000));
    }
}

bool mqttReconnect() {
    Serial.println("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    
    // Hapus null terminator dari string credentials
    String username(usernamemqtt);
    String password(passmqtt);
    
    if (mqtt.connect(clientId.c_str(), username.c_str(), password.c_str())) {
        Serial.println("MQTT Connected Successfully");
        mqttStatus = "Connected";
        // Tambahkan delay kecil setelah koneksi
        vTaskDelay(pdMS_TO_TICKS(100));
        return true;
    } else {
        Serial.print("MQTT Connection Failed, rc=");
        Serial.println(mqtt.state());
        mqttStatus = "Disconnected";
        return false;
    }
}
void vTaskMQTTPublish(void *pvParameters) {
    for (;;) {
        // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      Serial.println("GPRS disconnected!");
      Serial.print(F("Connecting to "));
      Serial.print("internet");
      if (!modem.gprsConnect("internet", "", "")) {
        Serial.println("GPRS connection failed");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        return;
      }
      if (modem.isGprsConnected()) { Serial.println("GPRS reconnected"); }
    }

        if (!mqtt.connected()) {
            if (!mqttReconnect()) {
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }

        mqtt.loop();

        if (xSemaphoreTake(xMQTTMutex, portMAX_DELAY) == pdTRUE) {
            StaticJsonDocument<500> jsonDoc;
            
            // Pastikan semua string diakhiri dengan benar
             String timestamp = generateTimestamp();
            jsonDoc["timestamp"] = timestamp;
            
            // PH
            JsonObject phObj = jsonDoc.createNestedObject("ph");
            phObj["value"] = xSensorData.ph;
            phObj["unit"] = "pH";

            // TDS
            JsonObject tdsObj = jsonDoc.createNestedObject("tds");
            tdsObj["value"] = xSensorData.tds;
            tdsObj["unit"] = "ppm";

            // ORP
            JsonObject orpObj = jsonDoc.createNestedObject("orp");
            orpObj["value"] = xSensorData.orp;
            orpObj["unit"] = "mV";

            // DO
            JsonObject doObj = jsonDoc.createNestedObject("do");
            doObj["value"] = xSensorData.doValue;
            doObj["unit"] = "mg/L";

            // Temperature
            JsonObject tempObj = jsonDoc.createNestedObject("temperature");
            tempObj["value"] = xSensorData.temperature;
            tempObj["unit"] = "°C";

            JsonObject sensorStatusObj = jsonDoc.createNestedObject("sensor_status");
            // Fungsi untuk mengonversi enum status ke string
            auto getStatusString = [](SensorStatus::Status status) -> String {
                switch(status) {
                    case SensorStatus::ERROR: return "Error";
                    case SensorStatus::WORKING: return "Working";
                    case SensorStatus::BELOW: return "Below";
                    case SensorStatus::ABOVE: return "Above";
                    default: return "Unknown";
                }
            };
            sensorStatusObj["pH"] = getStatusString(sensorStatus.phStatus);
            sensorStatusObj["tds"] = getStatusString(sensorStatus.tdsStatus);
            sensorStatusObj["orp"] = getStatusString(sensorStatus.orpStatus);
            sensorStatusObj["do"] = getStatusString(sensorStatus.doStatus);
            sensorStatusObj["temperature"] = getStatusString(sensorStatus.tempStatus);
            char jsonBuffer[500];
            memset(jsonBuffer, 0, sizeof(jsonBuffer)); // Clear buffer first
            
            size_t len = serializeJson(jsonDoc, jsonBuffer);
            
            // Debug print dengan length
            Serial.print("Message length: ");
            Serial.println(len);
            Serial.print("Publishing: ");
            Serial.println(jsonBuffer);

            bool publishSuccess = false;
            int retries = 3;
            
            while (retries > 0 && !publishSuccess) {
                if (mqtt.publish("/varx/water-quality/PDAM-KLATEN-NODE1", jsonBuffer, len)) {
                    publishSuccess = true;
                    Serial.println("Publish success");
                    mqttStatus = "Connected";
                } else {
                    Serial.print("Publish failed, retries left: ");
                    Serial.println(retries);
                    retries--;
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }

            if (!publishSuccess) {
                Serial.println("All publish attempts failed");
                mqttStatus = "Disconnected";
            }

            xSemaphoreGive(xMQTTMutex);
        }

        vTaskDelay(xMQTTDelay - pdMS_TO_TICKS(350));
    }
}

void setupGSMAndGPRS() {
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  digitalWrite(RST_PIN, LOW);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  digitalWrite(RST_PIN, HIGH);

  Serial.println("Restarting modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  Serial.println("Connecting to GSM network...");
  if (!modem.waitForNetwork()) {
    Serial.println("Network connection failed");
    return;
  }
  Serial.println("Connected to GSM network");

  Serial.println("Setting up GPRS...");
  if (!modem.gprsConnect(apn,"","")) {
    Serial.println("GPRS connection failed");
    return;
  }
  Serial.println("GPRS connected");

  // Set MQTT server
  mqtt.setServer(broker, brokerPort);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(120);  // Set keep-alive to 60 seconds
  mqtt.setSocketTimeout(60);  // Set socket timeout to 30 seconds

  // Koneksi MQTT
  Serial.println("Connecting to MQTT Broker...");
  String clientId = "ESP32Client-" + String(random(0xffff), HEX);
  
  if (mqtt.connect(clientId.c_str(), usernamemqtt, passmqtt)) {
    Serial.println("MQTT Connected Successfully");
    mqttStatus = "Connected";
  } else {
    Serial.print("MQTT Connection Failed, rc=");
    Serial.print(mqtt.state());
    mqttStatus = "Disconnected";
  }
}

void setup() {
    Serial.begin(115200);
    SerialAT.begin(9600, SERIAL_8N1, RX_PIN_GSM, TX_PIN_GSM);
    SerialNextion.begin(9600, SERIAL_8N1, RX_PIN_NEXTION, TX_PIN_NEXTION);
    setupGSMAndGPRS();

    tempSensor.begin();
    tempSensor.setWaitForConversion(false);
    
    xMQTTMutex = xSemaphoreCreateMutex();
    xSerialMutex = xSemaphoreCreateMutex();
    setCustomTime(2024, 12, 17, 11, 40, 0);
    myNextion.begin();

    xTaskCreate(vTaskSensorRead, "SensorTask", 4096, NULL, 5, &xTaskSensorRead);
    xTaskCreate(vTaskNextionUpdate, "NextionUpdateTask", 4096, NULL, 5, &xTaskNextionUpdate);
    xTaskCreate(vTaskMQTTPublish, "MQTTPublishTask", 8192, NULL, 5, &xTaskMQTTPublish);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}