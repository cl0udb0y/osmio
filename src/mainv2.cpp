#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include "config.h"

#define FLOW_SENSOR_PIN D2
#define NO_FLOW_TIMEOUT 2000 // Timeout to determine no flow in milliseconds

float calibrationFactor = 0.0; // Default value, will be loaded from config
float kFactor = 0.0;           // Default value, will be loaded from config

AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Update every 60 seconds

volatile int pulseCount = 0;
unsigned long lastPulseTime = 0;
bool flowDetected = false;

struct FilterData
{
    float initialLitres;
    float processedLitres;
    char lastChanged[20];
    unsigned long lastChangedTimestamp;
    float remainingLitres;
    unsigned long remainingDays;
};

FilterData carbonFilter = {0.0, 0.0, "", 0, 0.0, 0};
FilterData kdfGacFilter = {0.0, 0.0, "", 0, 0.0, 0};
FilterData ceramicFilter = {0.0, 0.0, "", 0, 0.0, 0};

struct TotalData
{
    float allTimeLitres;
    char lastReset[20];
    unsigned long lastFullResetTimestamp;
};

TotalData totalData = {0.0, "", 0};

unsigned long oldTime = 0;
unsigned long lastPublishTime = 0;
String macAddr;

#define INITIALIZED_FLAG_ADDRESS 0
#define CARBON_FILTER_ADDRESS sizeof(bool)
#define KDF_GAC_FILTER_ADDRESS (CARBON_FILTER_ADDRESS + sizeof(FilterData))
#define CERAMIC_FILTER_ADDRESS (KDF_GAC_FILTER_ADDRESS + sizeof(FilterData))
#define TOTAL_LITRES_ADDRESS (CERAMIC_FILTER_ADDRESS + sizeof(FilterData))

void setup_wifi();
void callback(char *topic, byte *payload, unsigned int length);
bool reconnect();
void saveFilterData(int address, FilterData &data);
void loadFilterData(int address, FilterData &data);
void initializeEEPROM();
void publishUsage();
void publishFilterData(const char *filterName, FilterData &filterData, float maxLitres, unsigned long maxDays);
void publishAllTimeData();
void updateTimestamp(char *buffer, size_t bufferSize);
void initializeFilterData(FilterData &data);
void eraseEEPROM();
bool publishWithRetry(const char *topic, const char *payload, int retryCount = 3);
void calculateRemainingLifespan(FilterData &data, float maxLitres, unsigned long maxDays);
void saveTotalData(int address, TotalData &data);
void loadTotalData(int address, TotalData &data);
bool loadConfig(const char *filename, const char *sensorName);
void calculateFlow();

IRAM_ATTR void pulseCounter()
{
    pulseCount++;
    lastPulseTime = millis();
}

void updateTimestamp(char *buffer, size_t bufferSize)
{
    time_t now = timeClient.getEpochTime();
    struct tm *tm = localtime(&now);
    strftime(buffer, bufferSize, "%Y-%m-%d %H:%M:%S", tm);
}

String formatValue(float value) {
    if (value >= 1000000) {
        return String(value / 1000000, 2) + "M";
    } else if (value >= 1000) {
        return String(value / 1000, 2) + "k";
    } else {
        return String(value, 2);
    }
}


void setup()
{
    Serial.begin(115200);
    if (!LittleFS.begin())
    {
        Serial.println("Failed to mount file system");
        return;
    }

    EEPROM.begin(512);

    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    timeClient.begin();

    initializeEEPROM();
    loadFilterData(CARBON_FILTER_ADDRESS, carbonFilter);
    loadFilterData(KDF_GAC_FILTER_ADDRESS, kdfGacFilter);
    loadFilterData(CERAMIC_FILTER_ADDRESS, ceramicFilter);
    loadTotalData(TOTAL_LITRES_ADDRESS, totalData);

    // Load sensor configuration from file
    if (!loadConfig("/config.json", "YF-G1"))
    {
        Serial.println("Failed to load sensor configuration!");
        while (1)
            ; // Halt execution if the sensor configuration fails
    }

    // Serve static files
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // Handle data requests
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request)
              {

        JsonDocument doc;
        doc["totalLitres"] = formatValue(totalData.allTimeLitres);
        doc["flowrate"] = formatValue(pulseCount / kFactor); // Example flow rate calculation
        doc["lastReset"] = totalData.lastReset;
        doc["carbonTotal"] = formatValue(carbonFilter.processedLitres);
        doc["carbonChanged"] = carbonFilter.lastChanged;
        doc["carbonRemaining"] = formatValue(carbonFilter.remainingLitres);
        doc["carbonRemainingDays"] = carbonFilter.remainingDays;
        doc["kdfgacTotal"] = formatValue(kdfGacFilter.processedLitres);
        doc["kdfgacChanged"] = kdfGacFilter.lastChanged;
        doc["kdfgacRemaining"] = formatValue(kdfGacFilter.remainingLitres);
        doc["kdfgacRemainingDays"] = kdfGacFilter.remainingDays;
        doc["ceramicTotal"] = formatValue(ceramicFilter.processedLitres);
        doc["ceramicChanged"] = ceramicFilter.lastChanged;
        doc["ceramicRemaining"] = formatValue(ceramicFilter.remainingLitres);
        doc["ceramicRemainingDays"] = ceramicFilter.remainingDays;
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        request->send(200, "application/json", jsonResponse); });

    // Handle form submission for reset
    server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        String filterType;
        String dateStr;

        if (request->hasParam("filter", true)) {
            filterType = request->getParam("filter", true)->value();
        }

        if (request->hasParam("date", true)) {
            dateStr = request->getParam("date", true)->value();
        }

        char dateBuffer[20];
        time_t resetTime;
        struct tm tm;
        memset(&tm, 0, sizeof(struct tm)); // Initialize to zero

        if (strptime(dateStr.c_str(), "%Y-%m-%d %H:%M:%S", &tm)) {
            resetTime = mktime(&tm);
            strftime(dateBuffer, sizeof(dateBuffer), "%Y-%m-%d %H:%M:%S", &tm);
            Serial.print("Parsed date: ");
            Serial.println(dateBuffer);
        } else {
            Serial.println("Failed to parse date, using current time");
            updateTimestamp(dateBuffer, sizeof(dateBuffer));
            resetTime = timeClient.getEpochTime();
        }

        if (filterType == "carbon") {
            carbonFilter.initialLitres = totalData.allTimeLitres;
            carbonFilter.processedLitres = 0.0;
            strncpy(carbonFilter.lastChanged, dateBuffer, sizeof(carbonFilter.lastChanged));
            carbonFilter.lastChangedTimestamp = resetTime;
            saveFilterData(CARBON_FILTER_ADDRESS, carbonFilter);
            Serial.println("Carbon filter reset.");
        } else if (filterType == "kdfgac") {
            kdfGacFilter.initialLitres = totalData.allTimeLitres;
            kdfGacFilter.processedLitres = 0.0;
            strncpy(kdfGacFilter.lastChanged, dateBuffer, sizeof(kdfGacFilter.lastChanged));
            kdfGacFilter.lastChangedTimestamp = resetTime;
            saveFilterData(KDF_GAC_FILTER_ADDRESS, kdfGacFilter);
            Serial.println("KDF/GAC filter reset.");
        } else if (filterType == "ceramic") {
            ceramicFilter.initialLitres = totalData.allTimeLitres;
            ceramicFilter.processedLitres = 0.0;
            strncpy(ceramicFilter.lastChanged, dateBuffer, sizeof(ceramicFilter.lastChanged));
            ceramicFilter.lastChangedTimestamp = resetTime;
            saveFilterData(CERAMIC_FILTER_ADDRESS, ceramicFilter);
            Serial.println("Ceramic filter reset.");
        } else if (filterType == "full") {
            // Reset total data
            totalData.allTimeLitres = 0.0;
            strncpy(totalData.lastReset, dateBuffer, sizeof(totalData.lastReset));
            totalData.lastFullResetTimestamp = resetTime;
            saveTotalData(TOTAL_LITRES_ADDRESS, totalData);

            // Reset filter data
            carbonFilter.initialLitres = 0.0;
            carbonFilter.processedLitres = 0.0;
            strncpy(carbonFilter.lastChanged, dateBuffer, sizeof(carbonFilter.lastChanged));
            carbonFilter.lastChangedTimestamp = resetTime;
            saveFilterData(CARBON_FILTER_ADDRESS, carbonFilter);

            kdfGacFilter.initialLitres = 0.0;
            kdfGacFilter.processedLitres = 0.0;
            strncpy(kdfGacFilter.lastChanged, dateBuffer, sizeof(kdfGacFilter.lastChanged));
            kdfGacFilter.lastChangedTimestamp = resetTime;
            saveFilterData(KDF_GAC_FILTER_ADDRESS, kdfGacFilter);

            ceramicFilter.initialLitres = 0.0;
            ceramicFilter.processedLitres = 0.0;
            strncpy(ceramicFilter.lastChanged, dateBuffer, sizeof(ceramicFilter.lastChanged));
            ceramicFilter.lastChangedTimestamp = resetTime;
            saveFilterData(CERAMIC_FILTER_ADDRESS, ceramicFilter);

            Serial.println("Full reset performed.");
        }

        // Redirect back to the main page
        request->send(200, "text/html", "<html><body><h1>Reset Completed</h1><a href=\"/\">Back to Home</a></body></html>"); });

    server.begin();
    oldTime = millis(); // Initialize oldTime at the beginning
}

void loop()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
    timeClient.update();

    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - oldTime;

    if (elapsedTime >= 1000)
    { // Update every second (or when enough time has passed)
        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));

        calculateFlow();

        pulseCount = 0;        // Reset pulse count AFTER calculating flow
        oldTime = currentTime; // Update oldTime AFTER calculating flow

        attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
    }
}

void calculateFlow()
{
    // Volume based on pulse count (your original calculation)
    float litresThisPeriod = (pulseCount / calibrationFactor) * kFactor;
    // Check for NaN before updating totalData
    if (!isnan(litresThisPeriod))
    {
        totalData.allTimeLitres += litresThisPeriod;
    }
    else
    {
        Serial.println("Warning: NaN detected in litresThisPeriod. Total volume not updated.");
    }

    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - oldTime;
    oldTime = currentTime; // Update oldTime for the next calculation

    // Calculate pulse frequency
    float pulseFrequency = (float)pulseCount / (elapsedTime / 1000.0);

    // Calculate flow rate (combine both methods)
    float flowRate = pulseFrequency / kFactor; // Frequency-based calculation

    // Check for no flow detection
    if (currentTime - lastPulseTime > NO_FLOW_TIMEOUT)
    {
        flowDetected = false;
    }
    else
    {
        flowDetected = true;
    }

    if (flowDetected)
    {
        /*   Serial.print("Flow rate: ");
           Serial.print(flowRate);
           Serial.print(" L/min\t");
           Serial.print("Total Volume: ");
           Serial.print(totalData.allTimeLitres);
           Serial.print(" L\t");
           Serial.print("Pulse Frequency: ");
           Serial.print(pulseFrequency);
           Serial.println(" Hz\t");
      */
    }

    // Calculate processedLitres for each filter
    carbonFilter.processedLitres = totalData.allTimeLitres - carbonFilter.initialLitres;
    kdfGacFilter.processedLitres = totalData.allTimeLitres - kdfGacFilter.initialLitres;
    ceramicFilter.processedLitres = totalData.allTimeLitres - ceramicFilter.initialLitres;

    // Save the updated filter data to EEPROM
    saveFilterData(CARBON_FILTER_ADDRESS, carbonFilter);
    saveFilterData(KDF_GAC_FILTER_ADDRESS, kdfGacFilter);
    saveFilterData(CERAMIC_FILTER_ADDRESS, ceramicFilter);
    saveTotalData(TOTAL_LITRES_ADDRESS, totalData);
    publishUsage();
    publishAllTimeData();
}

bool reconnect()
{
    static unsigned long lastReconnectAttempt = 0;
    unsigned long now = millis();
    if (now - lastReconnectAttempt < 5000)
    {
        return false;
    }
    lastReconnectAttempt = now;

    if (client.connected())
    {
        return true;
    }

    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-" + macAddr;
    if (client.connect(clientId.c_str(), "", ""))
    {
        Serial.println("connected");
        client.subscribe((baseTopic + macAddr + resetFilterTopic).c_str());
        Serial.print("Subscribed to: ");
        Serial.println(baseTopic + macAddr + resetFilterTopic);
        lastReconnectAttempt = 0;
        return true;
    }
    else
    {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
    }
    return false;
}

void saveFilterData(int address, FilterData &data)
{
    EEPROM.put(address, data);
    EEPROM.commit();
}

void loadFilterData(int address, FilterData &data)
{
    EEPROM.get(address, data);
}

void saveTotalData(int address, TotalData &data)
{
    EEPROM.put(address, data);
    EEPROM.commit();
}

void loadTotalData(int address, TotalData &data)
{
    EEPROM.get(address, data);
}

void initializeEEPROM()
{
    bool initialized;
    EEPROM.get(INITIALIZED_FLAG_ADDRESS, initialized);
    if (!initialized)
    {
        eraseEEPROM();
        initialized = true;
        EEPROM.put(INITIALIZED_FLAG_ADDRESS, initialized);
        EEPROM.commit();
    }
}

void publishUsage()
{
    publishFilterData("carbonFilter", carbonFilter, CARBON_FILTER_LITRES, CARBON_FILTER_DAYS);
    publishFilterData("kdfGacFilter", kdfGacFilter, KDF_GAC_FILTER_LITRES, KDF_GAC_FILTER_DAYS);
    publishFilterData("ceramicFilter", ceramicFilter, CERAMIC_FILTER_LITRES, CERAMIC_FILTER_DAYS);
    publishAllTimeData();
}

void publishFilterData(const char *filterName, FilterData &filterData, float maxLitres, unsigned long maxDays)
{
    JsonDocument doc;
    calculateRemainingLifespan(filterData, maxLitres, maxDays);

    doc[filterName]["totalLitres"] = filterData.processedLitres;
    doc[filterName]["lastChanged"] = filterData.lastChanged;
    doc[filterName]["remainingLife"] = String(filterData.remainingDays) + " days / " + String(filterData.remainingLitres) + " L";

    char jsonBuffer[300];
    size_t n = serializeJson(doc, jsonBuffer);

    if (publishWithRetry(("home/" + macAddr + "/" + String(filterName)).c_str(), jsonBuffer, n))
    {
        // Serial.println("MQTT message published successfully");
    }
    else
    {
        Serial.println("Failed to publish MQTT message");
    }
}

void publishAllTimeData()
{
    JsonDocument doc;
    doc["allTimeLitres"] = totalData.allTimeLitres;
    doc["lastFullReset"] = totalData.lastFullResetTimestamp;

    char jsonBuffer[200];
    size_t n = serializeJson(doc, jsonBuffer);

    if (publishWithRetry(("home/" + macAddr + "/allTime").c_str(), jsonBuffer, n))
    {
        // Serial.println("MQTT message published successfully");
    }
    else
    {
        Serial.println("Failed to publish MQTT message");
    }
}

void initializeFilterData(FilterData &data)
{
    data.initialLitres = 0.0;
    updateTimestamp(data.lastChanged, sizeof(data.lastChanged));
    data.lastChangedTimestamp = timeClient.getEpochTime();
}

void eraseEEPROM()
{
    for (size_t i = 0; i < EEPROM.length(); i++)
    {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
}

bool publishWithRetry(const char *topic, const char *payload, int retryCount)
{
    for (int i = 0; i < retryCount; i++)
    {
        if (client.publish(topic, payload))
        {
            return true;
        }
        delay(1000);
    }
    return false;
}

void setup_wifi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    macAddr = WiFi.macAddress();
    macAddr.replace(":", "");
}

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.write(payload, length);
    Serial.println();

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error)
    {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
    }

    const char *command = doc["command"];
    const char *filter = doc["filter"];
    const char *date = doc["date"];
    char dateBuffer[20];
    time_t resetTime;

    if (date)
    {
        strncpy(dateBuffer, date, sizeof(dateBuffer));
        struct tm tm;
        if (strptime(date, "%Y-%m-%d %H:%M:%S", &tm))
        {
            resetTime = mktime(&tm);
        }
        else
        {
            Serial.println("Failed to parse date, using current time");
            resetTime = timeClient.getEpochTime();
        }
    }
    else
    {
        updateTimestamp(dateBuffer, sizeof(dateBuffer));
        resetTime = timeClient.getEpochTime();
    }

    if (command && strcmp(command, "full_reset") == 0)
    {
        // Perform a full reset
        totalData.allTimeLitres = 0.0;
        totalData.lastFullResetTimestamp = resetTime;
        strncpy(totalData.lastReset, dateBuffer, sizeof(totalData.lastReset));
        Serial.println("Performing full reset.");
        saveTotalData(TOTAL_LITRES_ADDRESS, totalData);

        // Reset filter data
        carbonFilter.initialLitres = 0.0;
        carbonFilter.processedLitres = 0.0;
        strncpy(carbonFilter.lastChanged, dateBuffer, sizeof(carbonFilter.lastChanged));
        carbonFilter.lastChangedTimestamp = resetTime;
        saveFilterData(CARBON_FILTER_ADDRESS, carbonFilter);

        kdfGacFilter.initialLitres = 0.0;
        kdfGacFilter.processedLitres = 0.0;
        strncpy(kdfGacFilter.lastChanged, dateBuffer, sizeof(kdfGacFilter.lastChanged));
        kdfGacFilter.lastChangedTimestamp = resetTime;
        saveFilterData(KDF_GAC_FILTER_ADDRESS, kdfGacFilter);

        ceramicFilter.initialLitres = 0.0;
        ceramicFilter.processedLitres = 0.0;
        strncpy(ceramicFilter.lastChanged, dateBuffer, sizeof(ceramicFilter.lastChanged));
        ceramicFilter.lastChangedTimestamp = resetTime;
        saveFilterData(CERAMIC_FILTER_ADDRESS, ceramicFilter);
    }
    else if (filter)
    {
        if (strcmp(filter, "carbon") == 0)
        {
            carbonFilter.initialLitres = totalData.allTimeLitres;
            carbonFilter.processedLitres = 0.0;
            strncpy(carbonFilter.lastChanged, dateBuffer, sizeof(carbonFilter.lastChanged));
            carbonFilter.lastChangedTimestamp = resetTime;
            saveFilterData(CARBON_FILTER_ADDRESS, carbonFilter);
            Serial.println("Carbon filter reset.");
        }
        else if (strcmp(filter, "kdfgac") == 0)
        {
            kdfGacFilter.initialLitres = totalData.allTimeLitres;
            kdfGacFilter.processedLitres = 0.0;
            strncpy(kdfGacFilter.lastChanged, dateBuffer, sizeof(kdfGacFilter.lastChanged));
            kdfGacFilter.lastChangedTimestamp = resetTime;
            saveFilterData(KDF_GAC_FILTER_ADDRESS, kdfGacFilter);
            Serial.println("KDF/GAC filter reset.");
        }
        else if (strcmp(filter, "ceramic") == 0)
        {
            ceramicFilter.initialLitres = totalData.allTimeLitres;
            ceramicFilter.processedLitres = 0.0;
            strncpy(ceramicFilter.lastChanged, dateBuffer, sizeof(ceramicFilter.lastChanged));
            ceramicFilter.lastChangedTimestamp = resetTime;
            saveFilterData(CERAMIC_FILTER_ADDRESS, ceramicFilter);
            Serial.println("Ceramic filter reset.");
        }
    }
}

bool loadConfig(const char *filename, const char *sensorName)
{
    File configFile = LittleFS.open(filename, "r");
    if (!configFile)
    {
        Serial.println("Failed to open config file");
        return false;
    }

    size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error)
    {
        Serial.println("Failed to parse config file");
        return false;
    }

    JsonArray sensors = doc["sensors"];
    for (JsonObject sensor : sensors)
    {
        if (strcmp(sensor["name"], sensorName) == 0)
        {
            calibrationFactor = sensor["calibrationFactor"];
            kFactor = sensor["kFactor"];
            Serial.print("Loaded calibrationFactor: ");
            Serial.println(calibrationFactor);
            Serial.print("Loaded kFactor: ");
            Serial.println(kFactor);
            return true;
        }
    }

    Serial.println("Sensor not found in config file");
    return false;
}

void calculateRemainingLifespan(FilterData &data, float maxLitres, unsigned long maxDays) {
    // Calculate the days since the filter was last changed
    unsigned long currentTime = timeClient.getEpochTime();
    unsigned long daysSinceChanged = (currentTime - data.lastChangedTimestamp) / 86400;

    // Infer processed litres if no litres have been recorded yet and days have passed
    if (data.processedLitres == 0 && daysSinceChanged > 0) {
        float dailyUsage = maxLitres / maxDays;
        data.processedLitres = daysSinceChanged * dailyUsage;
    }

    // Ensure that processed litres continue to accumulate correctly
    // This assumes `data.processedLitres` is being updated correctly elsewhere in the code

    // Calculate the remaining litres and days
    data.remainingLitres = maxLitres - data.processedLitres;
    if (data.remainingLitres < 0) {
        data.remainingLitres = 0; // Ensure remaining litres do not go negative
    }

    data.remainingDays = (maxDays > daysSinceChanged) ? (maxDays - daysSinceChanged) : 0;

    // Debug statements
  /*  Serial.print("Current time: "); Serial.println(currentTime);
    Serial.print("Last changed timestamp: "); Serial.println(data.lastChangedTimestamp);
    Serial.print("Days since changed: "); Serial.println(daysSinceChanged);
    Serial.print("Processed litres: "); Serial.println(data.processedLitres);
    Serial.print("Remaining litres: "); Serial.println(data.remainingLitres);
    Serial.print("Remaining days: "); Serial.println(data.remainingDays);
    */
}


