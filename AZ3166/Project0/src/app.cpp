#include "app.h"
// #include <SystemTickCounter.h> // Include per timestamp - COMMENTED! USE millis()

#define DEBUG_MODE false // Imposta a true per abilitare log di debug più dettagliati

// Funzione di logging con timestamp e categoria
void logMessage(const char* category, const char* message, ...) {
    char buffer[256];
    va_list args;
    va_start(args, message);
    vsnprintf(buffer, sizeof(buffer), message, args);
    va_end(args);

    unsigned long timestamp = millis(); // Usa millis() per il timestamp nel log
    Serial.printf("[%lu] %s: %s\n", timestamp, category, buffer);
}

// Funzione per mettere in sleep mode spegnendo tutto tranne il display
void App::enterSleepModeWithDisplay(unsigned long sleepDuration) {
    logMessage("SLEEP", "Entering sleep mode with display for %lu ms", sleepDuration);

    // Spegni WiFi e altre periferiche non necessarie
	//WiFi.disconnect();
    digitalWrite(LED_WIFI, LOW);
    digitalWrite(LED_AZURE, LOW);
    digitalWrite(LED_USER, LOW);
    rgbLed.turnOff();
    // Se ci sono altri moduli da spegnere, farlo qui

    // Mostra il messaggio di sleep sul display
    displayText("Sleep Mode", "Zzzz....");

    // Attendi in modo non bloccante (qui usiamo un ciclo con piccoli delay)
    unsigned long sleepStart = millis();
    while (millis() - sleepStart < sleepDuration) {
        delay(50); // Breve delay per cedere il controllo
    }

    logMessage("SLEEP", "Woke up from sleep mode");

    // Ri-inizializza i moduli necessari, ad esempio il WiFi
    if (!initWifi()) {
        logMessage("ERROR", "WiFi re-init failed after wake up");
    }
    // Eventuali altre periferiche riavviabili possono essere inizializzate qui
}

// Definizione del costruttore
App::App()
{
    // Inizializzazione delle variabili membro
    // Impostazione dei colori nell'array rgb
    rgb[RED]     = {255, 0, 0};
    rgb[GREEN]   = {0, 200, 0};
    rgb[BLUE]    = {0, 0, 150};
    rgb[YELLOW]  = {255, 255, 0};
    rgb[CYAN]    = {0, 255, 255};
    rgb[MAGENTA] = {255, 0, 255};

    buttonAState = 0;
    buttonBState = 0;
    keyIndex = 0;
    status = WL_IDLE_STATUS;
    statusSensors = 4;
    showSensors = false;
    counter = 0;
    action = WIFISTATUS;
    led = 0;
}

void App::init()
{
    displayText("INIT", "init...");
    logMessage("INIT", "Starting initialization");

    displayText("INIT", "led...");
    logMessage("INIT", "Initializing LEDs");
    initLeds();
    setRGBcolor(MAGENTA);
    displayText("INIT", "led OK");
    logMessage("INIT", "LEDs initialized");

    displayText("INIT", "sensors...");
    logMessage("INIT", "Initializing sensors");
    initSensors();
    displayText("INIT", "sensors OK");
    logMessage("INIT", "Sensors initialized");

    displayText("INIT", "WIFI...");
    logMessage("INIT", "Initializing WiFi");
    // Configurazione WiFi
    ssid = "Casa RS";
    pass = "richksecure4967";
    if (!initWifi()) { // Usa il valore booleano di ritorno
        displayText("INIT", "WIFI FAIL");
        logMessage("ERROR", "WiFi initialization failed, check credentials and network");
        while(1); // Blocco in caso di errore WiFi critico
    }
    displayText("INIT", "WIFI OK");
    logMessage("INIT", "WiFi initialized and connected");

    // Inizializza l'OLED display UNA SOLA VOLTA in App::init()
    displayText("INIT", "OLED..."); // Messaggio INIT OLED
    display.init();
    displayText("INIT", "OLED OK");
    logMessage("INIT", "OLED Display initialized");


    // Imposta l'azione predefinita
    action = SENSORSSERVERCLIENT;

    buttonAState = 0;
    buttonBState = 0;

    displayText("INIT", "Init done");
    logMessage("INIT", "Initialization complete");
}

void App::loop()
{
    setRGBcolor(BLUE);
    refreshLeds();
    doAction();
    setRGBcolor(GREEN);
}

void App::doAction()
{
    updateButtonsState();
    switch (action)
    {
    case SENSORSSERVERCLIENT:
        sendSensorDataToServer();
        break;
    case WIFISTATUS:
        showWifiInfo();
        break;
    case SENSORS:
        showSensorsInfo();
        break;
    case SERVERHTTP:
        monitoringHTTPServerConnections(false);
        break;
    default:
        displayText("ERROR doAction", "default case");
        logMessage("ERROR", "Unknown action in doAction: %d", action);
        break;
    }
}

void App::updateButtonsState()
{
    if (IsButtonClicked(USER_BUTTON_A))
    {
        switch (action)
        {
        case WIFISTATUS:
            action = SENSORS;
            break;
        case SENSORS:
            action = SENSORSSERVERCLIENT;
            break;
        case SENSORSSERVERCLIENT:
            action = SERVERHTTP;
            break;
        case SERVERHTTP:
            action = WIFISTATUS;
            break;
        default:
            displayText("ERROR updateButtonsState", "default case");
            logMessage("ERROR", "Unknown action in updateButtonsState (Button A): %d", action);
            break;
        }
        delay(50); // Debounce
        logMessage("INPUT", "Button A pressed, action changed to: %d", action);
    }

    if (IsButtonClicked(USER_BUTTON_B))
    {
        if (action == WIFISTATUS)
        {
            action = SERVERHTTP; // Cambia stato a SERVERHTTP
        }
        else if (action == SENSORS)
        {
            statusSensors = (statusSensors + 1) % NUMSENSORS;
            showSensors = true;
            logMessage("SENSOR_DISP", "Button B pressed, cycling to next sensor display: %d", statusSensors);
        }
        delay(50); // Debounce
    }
}

void App::showSensorsInfo()
{
    switch (statusSensors)
    {
    case 0:
        showHumidTempSensor();
        break;
    case 1:
        showPressureSensor();
        break;
    case 2:
        showMagneticSensor();
        break;
    case 3:
        showMotionGyroSensor();
        break;
    case 4:
        showMotionAccelSensor();
        break;
    default:
        logMessage("ERROR", "Invalid statusSensors value: %d", statusSensors);
        displayText("ERROR", "Sensor Status Error");
        break;
    }
    delay(100);
}

void App::showWifiInfo()
{
    printWifiStatus();
    delay(100);
}

void App::refreshLeds()
{
    int irda_status = IrdaSensor->IRDATransmit(&counter, 1, 100);
    if (irda_status != 0)
    {
        snprintf(buffInfo, sizeof(buffInfo), "Unable to transmit through IRDA");
        displayText("refreshLeds", buffInfo);
        logMessage("WARNING", "IRDA transmit failed with status: %d", irda_status);
        delay(2000);
    }
}

void App::setRGBcolor(COLOR color2)
{
    color = color2;
    rgbLed.setColor(rgb[color].red, rgb[color].green, rgb[color].blue);
}

void App::initSerial()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Attendi la connessione seriale
    }
    logMessage("INIT", "Serial communication initialized");
}

bool App::initWifi()
{
    if (WiFi.status() == WL_NO_SHIELD)
    {
        displayText("initWifi", "WiFi shield not present");
        logMessage("ERROR", "WiFi shield not present");
        return false;
    }

    int attempt = 0;
    while (status != WL_CONNECTED)
    {
        snprintf(buffInfo, sizeof(buffInfo), "Attempt %d to connect to SSID %s", ++attempt, ssid);
        displayText("initWifi", buffInfo);
        logMessage("WIFI", buffInfo);

        status = WiFi.begin(ssid, pass);
        if (status == WL_CONNECTED) break;

        delay(1000);
        if (attempt > 5) {
            logMessage("ERROR", "Failed to connect to WiFi after multiple attempts");
            return false;
        }
    }
    printWifiStatus();
    return true;
}

void App::printWifiStatus()
{
    IPAddress ip = WiFi.localIP();
    long rssi = WiFi.RSSI();

    char ipString[16];
    snprintf(ipString, sizeof(ipString), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    snprintf(buffInfo, sizeof(buffInfo), "SSID: %s§RSSI: %ld dBm§IP:%s", WiFi.SSID(), rssi, ipString);

    Serial.println(buffInfo);
    logMessage("WIFI", buffInfo);
    displayText("WIFI STATUS", buffInfo);
    delay(100);
}

void App::initLeds()
{
    pinMode(LED_WIFI, OUTPUT);
    pinMode(LED_AZURE, OUTPUT);
    pinMode(LED_USER, OUTPUT);
    digitalWrite(LED_WIFI, LOW);
    digitalWrite(LED_AZURE, LOW);
    digitalWrite(LED_USER, LOW);
    rgbLed.turnOff();
    logMessage("INIT", "Onboard LEDs and RGB LED initialized");
}
void App::initSensors() {
    ext_i2c = new DevI2C(D14, D15);
    if (ext_i2c == nullptr) {
        logMessage("ERROR", "Failed to create DevI2C instance");
        displayText("INIT", "I2C Error");
        return; // Errore I2C è critico, si può uscire (o gestire diversamente)
    }

    acc_gyro = new LSM6DSLSensor(*ext_i2c, D4, D5);
    if (acc_gyro == nullptr) {
        logMessage("ERROR", "Failed to create LSM6DSLSensor instance");
        displayText("INIT", "LSM6DSLS Error"); // Errore specifico per LSM6DSL
    } else {
        if (acc_gyro->init(NULL) != 0) {
            logMessage("ERROR", "LSM6DSLSensor init failed");
            displayText("INIT", "LSM6DSLS Init Fail"); // Errore specifico per init LSM6DSL
            delete acc_gyro;
            acc_gyro = nullptr; // Imposta a nullptr per sicurezza
        } else {
            acc_gyro->enableAccelerator();
            acc_gyro->enableGyroscope();
            logMessage("INIT", "LSM6DSLSensor initialized");
        }
    }

    ht_sensor = new HTS221Sensor(*ext_i2c);
    if (ht_sensor == nullptr) {
        logMessage("ERROR", "Failed to create HTS221Sensor instance");
        displayText("INIT", "HTS221 Error"); // Errore specifico per HTS221
    } else {
        if (ht_sensor->init(NULL) != 0) {
            logMessage("ERROR", "HTS221Sensor init failed");
            displayText("INIT", "HTS221 Init Fail"); // Errore specifico per init HTS221
            delete ht_sensor;
            ht_sensor = nullptr; // Imposta a nullptr per sicurezza
        } else {
            logMessage("INIT", "HTS221Sensor initialized");
        }
    }

    magnetometer = new LIS2MDLSensor(*ext_i2c);
    if (magnetometer == nullptr) {
        logMessage("ERROR", "Failed to create LIS2MDLSensor instance");
        displayText("INIT", "LIS2MDL Error"); // Errore specifico per LIS2MDL
    } else {
        if (magnetometer->init(NULL) != 0) {
            logMessage("ERROR", "LIS2MDLSensor init failed");
            displayText("INIT", "LIS2MDL Init Fail"); // Errore specifico per init LIS2MDL
            delete magnetometer;
            magnetometer = nullptr; // Imposta a nullptr per sicurezza
        } else {
            logMessage("INIT", "LIS2MDLSensor initialized");
        }
    }

    IrdaSensor = new IRDASensor();
    if (IrdaSensor == nullptr) {
        logMessage("ERROR", "Failed to create IRDASensor instance");
        displayText("INIT", "IRDA Error"); // Errore specifico per IRDA
    } else {
        if (IrdaSensor->init() != 0) {
            logMessage("ERROR", "IRDASensor init failed");
            displayText("INIT", "IRDA Init Fail"); // Errore specifico per init IRDA
            delete IrdaSensor;
            IrdaSensor = nullptr; // Imposta a nullptr per sicurezza
        } else {
            logMessage("INIT", "IRDASensor initialized");
        }
    }

    pressureSensor = new LPS22HBSensor(*ext_i2c);
    if (pressureSensor == nullptr) {
        logMessage("ERROR", "Failed to create LPS22HBSensor instance");
        displayText("INIT", "LPS22HB Error"); // Errore specifico per LPS22HB
    } else {
        if (pressureSensor->init(NULL) != 0) {
            logMessage("ERROR", "LPS22HBSensor init failed");
            displayText("INIT", "LPS22HB Init Fail"); // Errore specifico per init LPS22HB
            delete pressureSensor;
            pressureSensor = nullptr; // Imposta a nullptr per sicurezza
        } else {
            logMessage("INIT", "LPS22HBSensor initialized");
        }
    }

    statusSensors = 4;
    counter = 0;
    showSensors = false;
    logMessage("INIT", "Sensor initialization process completed (errors may have occurred)"); // Indica che il processo è finito, anche con errori
}

void App::showMotionGyroSensor()
{
    if (acc_gyro == nullptr) return;
    acc_gyro->getGAxes(axes);
    snprintf(buffInfo, sizeof(buffInfo), "x: %d§y: %d§z: %d", axes[0], axes[1], axes[2]);
    displayText("Gyroscope", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Gyroscope: x=%d, y=%d, z=%d", axes[0], axes[1], axes[2]);
}

void App::showMotionAccelSensor()
{
    if (acc_gyro == nullptr) return;
    acc_gyro->getXAxes(axes);
    snprintf(buffInfo, sizeof(buffInfo), "x: %d§y: %d§z: %d", axes[0], axes[1], axes[2]);
    displayText("Accelerometer", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Accelerometer: x=%d, y=%d, z=%d", axes[0], axes[1], axes[2]);
}

void App::showPressureSensor()
{
    if (pressureSensor == nullptr) return;
    float pressure = 0, temperature = 0;
    if (pressureSensor->getPressure(&pressure) != 0 || pressureSensor->getTemperature(&temperature) != 0) {
        logMessage("WARNING", "Error reading pressure or temperature sensor");
        snprintf(buffInfo, sizeof(buffInfo), "Error reading sensor");
        displayText("Environment 1", buffInfo);
        return;
    }
    float windChill = 13.12 + 0.6215 * temperature - 11.37 * pow(pressure, 0.16) + 0.3965 * temperature * pow(pressure, 0.16);
    int snprintfResult = snprintf(buffInfo, sizeof(buffInfo), "P: %shPa§Temp: %sC§Wind Chill: %.1f°C", f2s(pressure, 2), f2s(temperature, 1), windChill);
    if (snprintfResult >= sizeof(buffInfo)) {
        logMessage("WARNING", "String truncated in showPressureSensor for display!");
        // Eventualmente, visualizzare un messaggio di errore sull'OLED se si vuole segnalare il troncamento visivamente
    }
    displayText("Environment 1", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Pressure: %.2f hPa, Temperature: %.1f C, Wind Chill: %.1f C", pressure, temperature, windChill);
}

void App::showHumidTempSensor()
{
    if (ht_sensor == nullptr) return;
    ht_sensor->reset();
    float temperatureC = 0, humidity = 0;
    if (ht_sensor->getTemperature(&temperatureC) != 0 || ht_sensor->getHumidity(&humidity) != 0) {
        logMessage("WARNING", "Error reading humidity/temperature sensor");
        snprintf(buffInfo, sizeof(buffInfo), "Error reading sensor");
        displayText("Environment 2", buffInfo);
        return;
    }
    float heatIndex = -8.78469475556 + 1.61139411 * temperatureC + 2.33854883889 * humidity +
                      -0.14611605 * temperatureC * humidity +
                      -0.012308094 * pow(temperatureC, 2) +
                      -0.0164248277778 * pow(humidity, 2) +
                      0.002211732 * pow(temperatureC, 2) * humidity +
                      0.00072546 * temperatureC * pow(humidity, 2) +
                      -0.000003582 * pow(temperatureC, 2) * pow(humidity, 2);
    snprintf(buffInfo, sizeof(buffInfo), "Temp: %sC§Hum: %s%%§Heat Ind: %.1f", f2s(temperatureC, 1), f2s(humidity, 1), heatIndex);
    displayText("Environment 2", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Temperature: %.1f C, Humidity: %.1f %% Heat Index: %.1f", temperatureC, humidity, heatIndex);
}

void App::showMagneticSensor()
{
    if (magnetometer == nullptr) return;
    magnetometer->getMAxes(axes);
    snprintf(buffInfo, sizeof(buffInfo), "x: %d§y: %d§z: %d", axes[0], axes[1], axes[2]);
    displayText("Magnetometer", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Magnetometer: x=%d, y=%d, z=%d", axes[0], axes[1], axes[2]);
}

bool App::IsButtonClicked(unsigned char ulPin)
{
    pinMode(ulPin, INPUT);
    return digitalRead(ulPin) == LOW;
}

void App::displayText(const char *title, const char *body)
{
    display.clean(); // Pulisce lo schermo ad ogni chiamata, ma NON re-inizializza
    const int maxLineLength = 16;
    char line[maxLineLength + 1] = {0};
    int bodyLength = strlen(body);
    int lineStartIndex = 0;
    int lineNumber = 1;

    // Stampa del titolo sulla prima riga
    display.print(0, title, true);

    while (lineNumber < 4 && lineStartIndex < bodyLength)
    {
        int copyLength = (bodyLength - lineStartIndex < maxLineLength) ? (bodyLength - lineStartIndex) : maxLineLength;
        strncpy(line, body + lineStartIndex, copyLength);
        line[copyLength] = '\0';

        char *specialCharPos = strchr(line, '§');
        if (specialCharPos)
        {
            *specialCharPos = '\0';
            display.print(lineNumber++, line, true);
            lineStartIndex += (specialCharPos - line) + 1;
        }
        else
        {
            display.print(lineNumber++, line, true);
            lineStartIndex += copyLength;
        }
    }
    delay(500); // delay rimane
}


void App::monitoringHTTPServerConnections(bool blockingSocket)
{
    static WiFiServer wiFiServer(80);
    static bool serverStarted = false;

    if (!serverStarted)
    {
        displayText("Server HTTP", "Listening...");
        logMessage("HTTP_SERVER", "Starting HTTP server");

        wiFiServer.begin();
        Serial.println("Server started and listening on port 80");

        IPAddress localIP = WiFi.localIP();
        char ipStringBuffer[30];
        snprintf(ipStringBuffer, sizeof(ipStringBuffer), "Listening...§IP: %d.%d.%d.%d", localIP[0], localIP[1], localIP[2], localIP[3]);

        displayText("Server HTTP", ipStringBuffer);
        logMessage("HTTP_SERVER", "HTTP server listening on IP: %s", ipStringBuffer);

        serverStarted = true;
    }

    WiFiClient client = wiFiServer.available();
    if (client)
    {
        logMessage("HTTP_SERVER", "New client connected");
        displayText("Client Connected", "Processing...");
        digitalWrite(LED_AZURE, HIGH);

        unsigned long startTime = millis();
        bool currentLineIsBlank = true;
        String request = "";

        while (client.connected() && millis() - startTime < 5000) // Timeout 5 secondi
        {
            if (client.available())
            {
                char c = client.read();
                Serial.write(c);
                request += c;

                if (c == '\n' && currentLineIsBlank)
                {
                    logMessage("HTTP_SERVER", "Sending HTTP response");
                    char jsonResponseBuffer[1024];
                    int bufferLen = sizeof(jsonResponseBuffer);
                    int payloadLen = 0;
                    float pressure = 0, temperaturePR = 0, temperatureHT = 0, humidity = 0;
                    int accelAxes[3] = {0}, gyroAxes[3] = {0}, magAxes[3] = {0};
                    bool sensorsReadOk = true;

                    if (pressureSensor->getPressure(&pressure) != 0 || pressureSensor->getTemperature(&temperaturePR) != 0)
                    {
                        logMessage("WARNING", "Error reading pressure/temperature sensor in HTTP server");
                        sensorsReadOk = false;
                    }
                    if (ht_sensor->getTemperature(&temperatureHT) != 0 || ht_sensor->getHumidity(&humidity) != 0)
                    {
                        logMessage("WARNING", "Error reading humidity/temperature sensor in HTTP server");
                        sensorsReadOk = false;
                    }
                    if (acc_gyro->getXAxes(accelAxes) != 0 || acc_gyro->getGAxes(gyroAxes) != 0)
                    {
                        logMessage("WARNING", "Error reading accel/gyro sensors in HTTP server");
                        sensorsReadOk = false;
                    }
                    if (magnetometer->getMAxes(magAxes) != 0)
                    {
                        logMessage("WARNING", "Error reading magnetometer in HTTP server");
                        sensorsReadOk = false;
                    }

                    unsigned long timestamp = millis();
                    long rssi = WiFi.RSSI();
                    payloadLen = snprintf(jsonResponseBuffer, bufferLen,
                                          "{\"timestamp\":%lu,\"rssi_dbm\":%ld,\"temperature_sensor_1_celsius\":%.2f,"
                                          "\"temperature_sensor_2_celsius\":%.2f,\"pressure_hpa\":%.2f,\"humidity_percent\":%.2f,"
                                          "\"accelero_x_mg\":%d,\"accelero_y_mg\":%d,\"accelero_z_mg\":%d,"
                                          "\"gyro_x_mdps\":%d,\"gyro_y_mdps\":%d,\"gyro_z_mdps\":%d,"
                                          "\"mag_x_mgauss\":%d,\"mag_y_mgauss\":%d,\"mag_z_mgauss\":%d, \"sensors_ok\":%s}",
                                          timestamp, rssi, temperaturePR, temperatureHT, pressure, humidity,
                                          accelAxes[0], accelAxes[1], accelAxes[2], gyroAxes[0], gyroAxes[1], gyroAxes[2],
                                          magAxes[0], magAxes[1], magAxes[2], sensorsReadOk ? "true" : "false");

                    if (payloadLen >= bufferLen)
                    {
                        logMessage("ERROR", "JSON response buffer too small, data truncated in HTTP Server!");
                        jsonResponseBuffer[bufferLen - 1] = '\0';
                    }

                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    client.println("Connection: close");
                    client.println("");
                    client.print(jsonResponseBuffer);
                    break;
                }

                if (c == '\n')
                {
                    currentLineIsBlank = true;
                }
                else if (c != '\r')
                {
                    currentLineIsBlank = false;
                }
            }
        }

        displayText("Client Disconnected", "Processing...§Waiting...");
        logMessage("HTTP_SERVER", "Client disconnected, waiting for new connection");
        digitalWrite(LED_AZURE, LOW);
        client.stop();
    }
}



void App::sendSensorDataToServer() {
    // Stati della macchina a stati
    enum SendState {
        STATE_INIT,           // Raccolta dati e preparazione payload
        STATE_CONNECT,        // Tentativo di connessione al server
        STATE_SEND,           // Invio della richiesta POST
        STATE_WAIT_RESPONSE,  // Attesa della risposta dal server
        STATE_WAIT_NEXT       // Attesa del prossimo ciclo di invio
    };

    // Variabili statiche per mantenere lo stato tra le chiamate
    static SendState state = STATE_INIT;
    static unsigned long stateTimestamp = 0;
    static int attempt = 0;
    static unsigned long backoffDelay = 1000; // Inizia con 1 secondo
    static WiFiClient client;
    static char jsonPayloadBuffer[1024];
    static int payloadLen = 0;

    unsigned long now = millis();

    switch (state) {
        case STATE_INIT: {
            // Preparazione iniziale: raccolta dati sensori e costruzione del JSON
            displayText("SendDataToServer", "Initializing...");
            logMessage("DATA_TO_SERVER", "Starting sendSensorDataToServer action");

          // Raccolta dati sensori
const int NUM_READINGS = 5;
float pressureSum = 0, temperaturePRSum = 0, temperatureHTSum = 0, humiditySum = 0;
int accelAxes[3] = {0}, gyroAxes[3] = {0}, magAxes[3] = {0};
bool sensorsReadOk = true;

for (int i = 0; i < NUM_READINGS; ++i) {
    float pressure = 0, temperaturePR = 0, temperatureHT = 0, humidity = 0;
    int accelAxesTemp[3] = {0}, gyroAxesTemp[3] = {0}, magAxesTemp[3] = {0};

    if (pressureSensor->getPressure(&pressure) == 0 && pressureSensor->getTemperature(&temperaturePR) == 0) {
        pressureSum += pressure;
        temperaturePRSum += temperaturePR;
    } else {
        logMessage("WARNING", "Error reading pressure/temperature sensor");
        sensorsReadOk = false;
    }

    if (ht_sensor->getTemperature(&temperatureHT) == 0 && ht_sensor->getHumidity(&humidity) == 0) {
        temperatureHTSum += temperatureHT;
        humiditySum += humidity;
    } else {
        logMessage("WARNING", "Error reading humidity/temperature sensor");
        sensorsReadOk = false;
    }

    if (acc_gyro->getXAxes(accelAxesTemp) == 0 && acc_gyro->getGAxes(gyroAxesTemp) == 0) {
        for (int j = 0; j < 3; ++j) {
            accelAxes[j] += accelAxesTemp[j];
            gyroAxes[j] += gyroAxesTemp[j];
        }
    } else {
        logMessage("WARNING", "Error reading accel/gyro sensors");
        sensorsReadOk = false;
    }

    if (magnetometer->getMAxes(magAxesTemp) == 0) {
        for (int j = 0; j < 3; ++j) {
            magAxes[j] += magAxesTemp[j];
        }
    } else {
        logMessage("WARNING", "Error reading magnetometer");
        sensorsReadOk = false;
    }
}

// Calcolo della media delle letture
float pressure = pressureSum / NUM_READINGS;
float temperaturePR = temperaturePRSum / NUM_READINGS;
float temperatureHT = temperatureHTSum / NUM_READINGS;
float humidity = humiditySum / NUM_READINGS;

// Creazione del payload JSON
payloadLen = snprintf(jsonPayloadBuffer, sizeof(jsonPayloadBuffer),
    "{\"timestamp\":%lu,\"rssi_dbm\":%ld,"
    "\"temperature_sensor_1_celsius\":%.2f,"
    "\"temperature_sensor_2_celsius\":%.2f,\"pressure_hpa\":%.2f,\"humidity_percent\":%.2f,"
    "\"accelero_x_mg\":%d,\"accelero_y_mg\":%d,\"accelero_z_mg\":%d,"
    "\"gyro_x_mdps\":%d,\"gyro_y_mdps\":%d,\"gyro_z_mdps\":%d,"
    "\"mag_x_mgauss\":%d,\"mag_y_mgauss\":%d,\"mag_z_mgauss\":%d,"
    "\"sensors_ok\":%s}",
    millis(), WiFi.RSSI(),
    temperaturePR, temperatureHT, pressure, humidity,
    accelAxes[0] / NUM_READINGS, accelAxes[1] / NUM_READINGS, accelAxes[2] / NUM_READINGS,
    gyroAxes[0] / NUM_READINGS, gyroAxes[1] / NUM_READINGS, gyroAxes[2] / NUM_READINGS,
    magAxes[0] / NUM_READINGS, magAxes[1] / NUM_READINGS, magAxes[2] / NUM_READINGS,
    sensorsReadOk ? "true" : "false");


            if (payloadLen >= sizeof(jsonPayloadBuffer)) {
                logMessage("ERROR", "JSON payload buffer too small, data truncated!");
                jsonPayloadBuffer[sizeof(jsonPayloadBuffer) - 1] = '\0';
            }

            // Passa al tentativo di connessione
            state = STATE_CONNECT;
            stateTimestamp = now;
            break;
        }
        case STATE_CONNECT: {
            // Attende il backoff esponenziale
            if (now - stateTimestamp < backoffDelay) break;

            const char *serverHost = "192.168.0.218";
            const char *serverPath = "/sdl/upload-sensor-data";
            const unsigned short port = 8080;

            displayText("Connecting", "Connecting to server...");
            logMessage("DATA_TO_SERVER", "Attempt %d: Connecting to server: %s:%d%s", attempt+1, serverHost, port, serverPath);

            client.stop(); // Chiude eventuali connessioni precedenti
            if (client.connect(serverHost, port)) {
                digitalWrite(LED_AZURE, HIGH);
                displayText("Connected", "Sending data...");
                logMessage("DATA_TO_SERVER", "Connected to server");
                state = STATE_SEND;
                stateTimestamp = now;
            } else {
                logMessage("WARNING", "Connection attempt %d failed", attempt+1);
                attempt++;
                // Incrementa il tempo di backoff fino a un massimo di 30 secondi
                backoffDelay = (backoffDelay < 30000) ? backoffDelay * 2 : 30000;
                stateTimestamp = now;
                setRGBcolor(RED);  // Segnala il fallimento della connessione
            }
            break;
        }
        case STATE_SEND: {
            // Invio della richiesta HTTP POST
            const char *serverHost = "192.168.0.218";
            const char *serverPath = "/sdl/upload-sensor-data";

            client.print("POST "); client.print(serverPath); client.println(" HTTP/1.1");
            client.print("Host: "); client.println(serverHost);
            client.println("Content-Type: application/json");
            client.print("Content-Length: "); client.println(payloadLen);
            client.println("Connection: close");
            client.println("x-api-key: fweW2qo21qxmoCECWf23d"); // Chiave API
            client.println("");
            client.print(jsonPayloadBuffer);

            displayText("Data Sent", "Waiting for response...");
            logMessage("DATA_TO_SERVER", "Data sent to server, waiting for response");
            state = STATE_WAIT_RESPONSE;
            stateTimestamp = now;
            break;
        }
        case STATE_WAIT_RESPONSE: {
            const unsigned long timeoutMs = 10000; // Timeout di 10 secondi
            static String responseLine = "";
            if (client.connected() && (now - stateTimestamp < timeoutMs)) {
                if (client.available()) {
                    char c = client.read();
                    Serial.write(c);
                    if (c == '\n') {
                        // Se la linea è vuota, abbiamo terminato gli header
                        if (responseLine.length() == 0) {
                            logMessage("DATA_TO_SERVER", "Response received from server");
                            displayText("Response", "Data sent successfully.");
                            // Resetta il backoff dopo il successo
                            attempt = 0;
                            backoffDelay = 1000;
                            state = STATE_WAIT_NEXT;
                            stateTimestamp = now;
                        }
                        responseLine = "";
                    } else if (c != '\r') {
                        responseLine += c;
                    }
                }
            } else {
                // Se il timeout è scaduto o la connessione è persa
                if (now - stateTimestamp >= timeoutMs) {
                    logMessage("WARNING", "Timeout waiting for server response or server disconnected");
                    displayText("Response", "No response/Timeout");
                    setRGBcolor(RED);
                    client.stop();
                    attempt++;
                    backoffDelay = (backoffDelay < 30000) ? backoffDelay * 2 : 30000;
                    state = STATE_WAIT_NEXT;
                    stateTimestamp = now;
                }
            }
            break;
        }
        case STATE_WAIT_NEXT: {
            // Invece di attendere attivamente, entra in sleep mode per 60 secondi
            const unsigned long nextCycleDelay = 60000; // 60 secondi
            digitalWrite(LED_AZURE, LOW);
            displayText("Disconnected", "Zzzz....");
            logMessage("DATA_TO_SERVER", "Cycle finished, entering sleep mode with display");

            enterSleepModeWithDisplay(nextCycleDelay);

            state = STATE_INIT;
            stateTimestamp = millis();
            break;
        }
        default:
            state = STATE_INIT;
            break;
    }
}

