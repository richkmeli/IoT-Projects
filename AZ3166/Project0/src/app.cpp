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


// Definizione del costruttore
App::App()
{
    // Inizializzazione delle variabili membro
    rgb[0] = {255, 0, 0};
    rgb[1] = {0, 200, 0};
    rgb[2] = {0, 0, 150};

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
    // Inizializzazione LED
    rgb[0] = {255, 0, 0};
    rgb[1] = {0, 200, 0};
    rgb[2] = {0, 0, 150};
    led = 0;
    initLeds();
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
    if (!initWifi()) { // Corrected to use boolean return value
        displayText("INIT", "WIFI FAIL");
        logMessage("ERROR", "WiFi initialization failed, check credentials and network");
        while(1); // Blocco in caso di errore WiFi critico
    }
    displayText("INIT", "WIFI OK");
    logMessage("INIT", "WiFi initialized and connected");

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
        delay(50); // Debounce - potrebbe essere migliorato con millis() se necessario per maggiore reattività
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
        delay(50); // Debounce - potrebbe essere migliorato con millis() se necessario per maggiore reattività
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
    } // <-- ADDED missing closing brace here!
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
        ;
    } // Attendi che la connessione seriale sia stabilita
    logMessage("INIT", "Serial communication initialized");
}

bool App::initWifi() // Modified to return bool
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
        if (status == WL_CONNECTED) break; // Esci dal loop se connesso

        delay(1000);
        if (attempt > 5) { // Limita il numero di tentativi
            logMessage("ERROR", "Failed to connect to WiFi after multiple attempts");
            return false; // Indicate WiFi initialization failure
        }
    }
    printWifiStatus();
    return true; // Indicate WiFi initialization success
}


void App::printWifiStatus()
{
    IPAddress ip = WiFi.localIP();
    long rssi = WiFi.RSSI();

    char ipString[16];
    snprintf(ipString, sizeof(ipString), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    snprintf(buffInfo, sizeof(buffInfo), "SSID: %s§RSSI: %ld dBm§IP:%s", WiFi.SSID(), rssi, ipString);

    Serial.println(buffInfo); // Debug seriale
    logMessage("WIFI", buffInfo);
    displayText("WIFI STATUS", buffInfo);
    delay(100);
}

void App::initLeds()
{
    pinMode(LED_WIFI, OUTPUT);
    pinMode(LED_AZURE, OUTPUT);
    pinMode(LED_USER, OUTPUT);
    digitalWrite(LED_WIFI, LOW); // Spegne LED WiFi all'inizio
    digitalWrite(LED_AZURE, LOW); // Spegne LED Azure all'inizio
    digitalWrite(LED_USER, LOW);  // Spegne LED USER all'inizio
    rgbLed.turnOff();
    logMessage("INIT", "Onboard LEDs and RGB LED initialized");
}

void App::initSensors()
{
    ext_i2c = new DevI2C(D14, D15);
    if (ext_i2c == nullptr) {
        logMessage("ERROR", "Failed to create DevI2C instance");
        displayText("INIT", "I2C Error");
        return; // Esci se l'inizializzazione I2C fallisce
    }

    acc_gyro = new LSM6DSLSensor(*ext_i2c, D4, D5);
    if (acc_gyro == nullptr) {
        logMessage("ERROR", "Failed to create LSM6DSLSensor instance");
        displayText("INIT", "Sensor Error");
        delete ext_i2c; // Pulisci la memoria allocata per I2C
        ext_i2c = nullptr;
        return;
    }
    if (acc_gyro->init(NULL) != 0) {
        logMessage("ERROR", "LSM6DSLSensor init failed");
    } else {
        acc_gyro->enableAccelerator();
        acc_gyro->enableGyroscope();
        logMessage("INIT", "LSM6DSLSensor initialized");
    }


    ht_sensor = new HTS221Sensor(*ext_i2c);
    if (ht_sensor == nullptr) {
        logMessage("ERROR", "Failed to create HTS221Sensor instance");
        displayText("INIT", "Sensor Error");
        delete acc_gyro; // Pulisci la memoria allocata per gli altri sensori già creati
        acc_gyro = nullptr;
        delete ext_i2c;
        ext_i2c = nullptr;
        return;
    }
    if (ht_sensor->init(NULL) != 0) {
        logMessage("ERROR", "HTS221Sensor init failed");
    } else {
        logMessage("INIT", "HTS221Sensor initialized");
    }


    magnetometer = new LIS2MDLSensor(*ext_i2c);
    if (magnetometer == nullptr) {
        logMessage("ERROR", "Failed to create LIS2MDLSensor instance");
        displayText("INIT", "Sensor Error");
        delete ht_sensor;
        ht_sensor = nullptr;
        delete acc_gyro;
        acc_gyro = nullptr;
        delete ext_i2c;
        ext_i2c = nullptr;
        return;
    }
    if (magnetometer->init(NULL) != 0) {
        logMessage("ERROR", "LIS2MDLSensor init failed");
    } else {
         logMessage("INIT", "LIS2MDLSensor initialized");
    }


    IrdaSensor = new IRDASensor();
    if (IrdaSensor == nullptr) {
        logMessage("ERROR", "Failed to create IRDASensor instance");
         displayText("INIT", "Sensor Error");
        delete magnetometer;
        magnetometer = nullptr;
        delete ht_sensor;
        ht_sensor = nullptr;
        delete acc_gyro;
        acc_gyro = nullptr;
        delete ext_i2c;
        ext_i2c = nullptr;
        return;
    }
    if (IrdaSensor->init() != 0) {
         logMessage("ERROR", "IRDASensor init failed");
    } else {
        logMessage("INIT", "IRDASensor initialized");
    }


    pressureSensor = new LPS22HBSensor(*ext_i2c);
    if (pressureSensor == nullptr) {
        logMessage("ERROR", "Failed to create LPS22HBSensor instance");
         displayText("INIT", "Sensor Error");
        delete IrdaSensor;
        IrdaSensor = nullptr;
        delete magnetometer;
        magnetometer = nullptr;
        delete ht_sensor;
        ht_sensor = nullptr;
        delete acc_gyro;
        acc_gyro = nullptr;
        delete ext_i2c;
        ext_i2c = nullptr;
        return;
    }
    if (pressureSensor->init(NULL) != 0) {
         logMessage("ERROR", "LPS22HBSensor init failed");
    } else {
        logMessage("INIT", "LPS22HBSensor initialized");
    }


    statusSensors = 4;
    counter = 0;
    showSensors = false;
    logMessage("INIT", "All sensors initialized");
}

void App::showMotionGyroSensor()
{
    if (acc_gyro == nullptr) return; // Check if sensor is initialized

    acc_gyro->getGAxes(axes);
    snprintf(buffInfo, sizeof(buffInfo), "x: %d§y: %d§z: %d", axes[0], axes[1], axes[2]);
    displayText("Gyroscope", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Gyroscope: x=%d, y=%d, z=%d", axes[0], axes[1], axes[2]);
}

void App::showMotionAccelSensor()
{
     if (acc_gyro == nullptr) return; // Check if sensor is initialized

    acc_gyro->getXAxes(axes);
    snprintf(buffInfo, sizeof(buffInfo), "x: %d§y: %d§z: %d", axes[0], axes[1], axes[2]);
    displayText("Accelerometer", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Accelerometer: x=%d, y=%d, z=%d", axes[0], axes[1], axes[2]);
}

void App::showPressureSensor()
{
    if (pressureSensor == nullptr) return; // Check sensor initialization

    float pressure = 0;
    float temperature = 0;
    if (pressureSensor->getPressure(&pressure) != 0 || pressureSensor->getTemperature(&temperature) != 0) {
        logMessage("WARNING", "Error reading pressure or temperature sensor");
        snprintf(buffInfo, sizeof(buffInfo), "Error reading sensor");
        displayText("Environment 1", buffInfo);
        return;
    }


    // Temperatura percepita (Wind Chill) come esempio
    float windChill = 13.12 + 0.6215 * temperature - 11.37 * pow(pressure, 0.16) + 0.3965 * temperature * pow(pressure, 0.16);

    snprintf(buffInfo, sizeof(buffInfo), "P: %shPa§Temp: %sC§Wind Chill: %.1f°C", f2s(pressure, 2), f2s(temperature, 1), windChill);
    displayText("Environment 1", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Pressure: %.2f hPa, Temperature: %.1f C, Wind Chill: %.1f C", pressure, temperature, windChill);
}

void App::showHumidTempSensor()
{
    if (ht_sensor == nullptr) return; // Check sensor initialization

    ht_sensor->reset(); // Reset potrebbe non essere necessario ad ogni lettura, controllare datasheet

    float temperatureC = 0;
    float humidity = 0;
    if (ht_sensor->getTemperature(&temperatureC) != 0 || ht_sensor->getHumidity(&humidity) != 0) {
        logMessage("WARNING", "Error reading humidity/temperature sensor");
        snprintf(buffInfo, sizeof(buffInfo), "Error reading sensor");
        displayText("Environment 2", buffInfo);
        return;
    }


    // Calcola l'indice di calore
    float heatIndex = -8.78469475556 + 1.61139411 * temperatureC + 2.33854883889 * humidity +
                       -0.14611605 * temperatureC * humidity +
                       -0.012308094 * pow(temperatureC, 2) +
                       -0.0164248277778 * pow(humidity, 2) +
                       0.002211732 * pow(temperatureC, 2) * humidity +
                       0.00072546 * temperatureC * pow(humidity, 2) +
                       -0.000003582 * pow(temperatureC, 2) * pow(humidity, 2);

    // Approssima l'indice di calore a una cifra decimale
    snprintf(buffInfo, sizeof(buffInfo), "Temp: %sC§Hum: %s%%§Heat Ind: %.1f", f2s(temperatureC, 1), f2s(humidity, 1), heatIndex);
    displayText("Environment 2", buffInfo);
    if (DEBUG_MODE) logMessage("SENSOR_DATA", "Temperature: %.1f C, Humidity: %.1f %% Heat Index: %.1f", temperatureC, humidity, heatIndex);
}

void App::showMagneticSensor()
{
    if (magnetometer == nullptr) return; // Check sensor initialization
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
    // Mostra il titolo sulla prima riga
    textOutDevKitScreen(0, title, 1);

    // Buffer per ogni riga del body
    char line[17] = {0}; // 16 caratteri più terminatore null

    int bodyLength = strlen(body);
    int lineStartIndex = 0;
    int lineEndIndex = 16;
    int lineNumber = 1;

    while (lineNumber < 4 && lineStartIndex < bodyLength)
    { // Righe 1 a 3
        int copyLength = lineEndIndex - lineStartIndex;

        // Assicurati che non superiamo la lunghezza del corpo
        if (lineEndIndex > bodyLength)
        {
            lineEndIndex = bodyLength;
        }

        // Copia la sottostringa nella riga
        strncpy(line, body + lineStartIndex, copyLength);
        line[copyLength] = '\0'; // Assicurati che la stringa sia terminata

        // Controlla la presenza di § e gestisci il ritorno a capo
        char *specialCharPos = strchr(line, '§');
        if (specialCharPos != nullptr)
        {
            *specialCharPos = '\0';             // Termina la stringa prima di §
            line[specialCharPos - line] = '\0'; // Assicurati che la stringa sia terminata

            // Mostra la riga e pulisce il buffer per la riga successiva
            textOutDevKitScreen(lineNumber++, line, 1);

            // Inizia la prossima riga
            lineStartIndex += (specialCharPos - line) + 1; // Salta il carattere §
            lineEndIndex = lineStartIndex + 16;            // Aggiorna l'indice finale
        }
        else
        {
            // Mostra la riga normale
            textOutDevKitScreen(lineNumber++, line, 1);
            lineStartIndex = lineEndIndex;
            lineEndIndex += 16;
        }
    }

    // Se ci sono righe rimanenti (per i casi in cui body è più lungo di 48 caratteri)
    while (lineNumber <= 3)
    {
        textOutDevKitScreen(lineNumber++, "", 1); // Pulisce le righe rimanenti
    }
}

void App::monitoringHTTPServerConnections(bool blockingSocket)
{
    // Display initial status
    displayText("Server HTTP", "Listening...");
    logMessage("HTTP_SERVER", "Starting HTTP server");

    WiFiServer wiFiServer(80);
    wiFiServer.begin();
    Serial.println("Server started and listening on port 80");

    IPAddress localIP = WiFi.localIP();
    char ipStringBuffer[30]; // Buffer per stringa IP
    snprintf(ipStringBuffer, sizeof(ipStringBuffer), "Listening...§IP: %d.%d.%d.%d", localIP[0], localIP[1], localIP[2], localIP[3]);

    // Display IP address
    displayText("Server HTTP", ipStringBuffer);
    logMessage("HTTP_SERVER", "HTTP server listening on IP: %s", ipStringBuffer);

    while (true)
    {
        WiFiClient client = wiFiServer.available();
        if (client)
        {
            logMessage("HTTP_SERVER", "New client connected");
            displayText("Client Connected", "Processing...");

            digitalWrite(LED_AZURE, HIGH);

            boolean currentLineIsBlank = true;
            while (client.connected())
            {
                if (client.available())
                {
                    char c = client.read();
                    Serial.write(c);

                    if (c == '\n' && currentLineIsBlank)
                    {
                        logMessage("HTTP_SERVER", "Sending HTTP response");

                        // Buffer per la risposta JSON (dimensionare adeguatamente)
                        char jsonResponseBuffer[1024]; // Dimensiona a sufficienza!
                        int bufferLen = sizeof(jsonResponseBuffer);
                        int payloadLen = 0; // Lunghezza attuale del payload JSON


                        // Collect sensor data - con controllo errori
                        float pressure = 0, temperaturePR = 0, temperatureHT = 0, humidity = 0;
                        int accelAxes[3] = {0}, gyroAxes[3] = {0}, magAxes[3] = {0};
                        bool sensorsReadOk = true;

                        if (pressureSensor->getPressure(&pressure) != 0 || pressureSensor->getTemperature(&temperaturePR) != 0) {
                            logMessage("WARNING", "Error reading pressure/temperature sensor in HTTP server");
                            sensorsReadOk = false;
                        }
                        if (ht_sensor->getTemperature(&temperatureHT) != 0 || ht_sensor->getHumidity(&humidity) != 0) {
                            logMessage("WARNING", "Error reading humidity/temperature sensor in HTTP server");
                            sensorsReadOk = false;
                        }
                        if (acc_gyro->getXAxes(accelAxes) != 0 || acc_gyro->getGAxes(gyroAxes) != 0) {
                            logMessage("WARNING", "Error reading accel/gyro sensors in HTTP server");
                            sensorsReadOk = false;
                        }
                        if (magnetometer->getMAxes(magAxes) != 0) {
                            logMessage("WARNING", "Error reading magnetometer in HTTP server");
                            sensorsReadOk = false;
                        }

                        unsigned long timestamp = millis(); // Usa millis() - FIXED!
                        long rssi = WiFi.RSSI();


                        // Crea il JSON usando snprintf - **GESTIONE ERRORI BUFFER!**
                        payloadLen = snprintf(jsonResponseBuffer, bufferLen,
                                              "{\"timestamp\":%lu,\"rssi_dbm\":%ld,\"temperature_sensor_1_celsius\":%.2f,"
                                              "\"temperature_sensor_2_celsius\":%.2f,\"pressure_hpa\":%.2f,\"humidity_percent\":%.2f,"
                                              "\"accelero_x_mg\":%d,\"accelero_y_mg\":%d,\"accelero_z_mg\":%d,"
                                              "\"gyro_x_mdps\":%d,\"gyro_y_mdps\":%d,\"gyro_z_mdps\":%d,"
                                              "\"mag_x_mgauss\":%d,\"mag_y_mgauss\":%d,\"mag_z_mgauss\":%d, \"sensors_ok\":%s}",
                                              timestamp, rssi, temperaturePR, temperatureHT, pressure, humidity,
                                              accelAxes[0], accelAxes[1], accelAxes[2], gyroAxes[0], gyroAxes[1], gyroAxes[2],
                                              magAxes[0], magAxes[1], magAxes[2], sensorsReadOk ? "true" : "false");


                        if (payloadLen >= bufferLen) {
                            logMessage("ERROR", "JSON response buffer too small, data truncated in HTTP Server!");
                            // Gestire errore buffer troppo piccolo - troncare o allocare dinamicamente (più complesso)
                            // Per ora, tronchiamo la stringa
                            jsonResponseBuffer[bufferLen - 1] = '\0';
                        }


                        // Send HTTP response
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-Type: application/json");
                        client.println("Connection: close");
                        client.println("");
                        client.print(jsonResponseBuffer); // Usa print per char array

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
            delay(1000);

            digitalWrite(LED_AZURE, LOW);
            client.stop();
        }
        else
        {
            // Usa snprintf per formattare la stringa IP nel displayText
            displayText("Server HTTP", ipStringBuffer);
            delay(100);
        }
    }
}



void App::sendSensorDataToServer(){
    displayText("SendDataToServer", "Initializing...");
    logMessage("DATA_TO_SERVER", "Starting sendSensorDataToServer action");

    const char *serverHost = "192.168.0.218";
    const char *serverPath = "/sdl/upload-sensor-data";
    const unsigned short port = 8080;

    WiFiClient client;
    bool connectedToServer = false;

    displayText("Connecting", "Connecting to server...");
    logMessage("DATA_TO_SERVER", "Connecting to server: %s:%d%s", serverHost, port, serverPath);

    if (client.connect(serverHost, port))
    {
        digitalWrite(LED_AZURE, HIGH);
        displayText("Connected", "Sending data...");
        logMessage("DATA_TO_SERVER", "Connected to server");
        connectedToServer = true;
    }
    else
    {
        setRGBcolor(RED);
        displayText("Connection", "Failed. Retrying...");
        logMessage("WARNING", "Connection to server failed");
        return; // Esci dalla funzione in caso di fallimento connessione
    }


    // Raccolta dati sensori - con gestione errori
    float pressure = 0, temperaturePR = 0, temperatureHT = 0, humidity = 0;
    int accelAxes[3] = {0}, gyroAxes[3] = {0}, magAxes[3] = {0};
    bool sensorsReadOk = true;


    if (pressureSensor->getPressure(&pressure) != 0 || pressureSensor->getTemperature(&temperaturePR) != 0) {
        logMessage("WARNING", "Error reading pressure/temperature sensor");
        sensorsReadOk = false;
    }
    if (ht_sensor->getTemperature(&temperatureHT) != 0 || ht_sensor->getHumidity(&humidity) != 0) {
        logMessage("WARNING", "Error reading humidity/temperature sensor");
        sensorsReadOk = false;
    }
    if (acc_gyro->getXAxes(accelAxes) != 0 || acc_gyro->getGAxes(gyroAxes) != 0) {
        logMessage("WARNING", "Error reading accel/gyro sensors");
        sensorsReadOk = false;
    }
    if (magnetometer->getMAxes(magAxes) != 0) {
        logMessage("WARNING", "Error reading magnetometer");
        sensorsReadOk = false;
    }


    // Buffer per il payload JSON (dimensionare adeguatamente!)
    char jsonPayloadBuffer[1024]; // Dimensiona buffer a sufficienza
    int bufferLen = sizeof(jsonPayloadBuffer);
    int payloadLen = 0; // Lunghezza attuale payload JSON

    unsigned long timestamp = millis(); // Usa millis() - FIXED!
    long rssi = WiFi.RSSI();


    // Crea il JSON payload usando snprintf - **GESTIONE ERRORI BUFFER!**
    payloadLen = snprintf(jsonPayloadBuffer, bufferLen,
                          "{\"timestamp\":%lu,\"rssi_dbm\":%ld,\"temperature_sensor_1_celsius\":%.2f,"
                          "\"temperature_sensor_2_celsius\":%.2f,\"pressure_hpa\":%.2f,\"humidity_percent\":%.2f,"
                          "\"accelero_x_mg\":%d,\"accelero_y_mg\":%d,\"accelero_z_mg\":%d,"
                          "\"gyro_x_mdps\":%d,\"gyro_y_mdps\":%d,\"gyro_z_mdps\":%d,"
                          "\"mag_x_mgauss\":%d,\"mag_y_mgauss\":%d,\"mag_z_mgauss\":%d, \"sensors_ok\":%s}",
                          timestamp, rssi, temperaturePR, temperatureHT, pressure, humidity,
                          accelAxes[0], accelAxes[1], accelAxes[2], gyroAxes[0], gyroAxes[1], gyroAxes[2],
                          magAxes[0], magAxes[1], magAxes[2], sensorsReadOk ? "true" : "false");


    if (payloadLen >= bufferLen) {
        logMessage("ERROR", "JSON payload buffer too small, data truncated in sendSensorDataToServer!");
        // Gestire errore buffer troppo piccolo - troncare o allocare dinamicamente (più complesso)
        // Per ora, tronchiamo la stringa per sicurezza
        jsonPayloadBuffer[bufferLen - 1] = '\0';
    }


    // Prepara la richiesta POST
    client.print("POST "); client.print(serverPath); client.println(" HTTP/1.1");
    client.print("Host: "); client.println(serverHost);
    client.println("Content-Type: application/json");
    client.print("Content-Length: "); client.println(payloadLen);
    client.println("Connection: close");
    client.println("x-api-key: fweW2qo21qxmoCECWf23d"); // Inserisci qui la tua chiave
    client.println("");          // Fine degli header
    client.print(jsonPayloadBuffer); // Corpo della richiesta - usa print per char array

    displayText("Data Sent", "Waiting for response...");
    logMessage("DATA_TO_SERVER", "Data sent to server, waiting for response");


    // Attendi la risposta del server (con timeout per evitare blocchi indefiniti)
    unsigned long startTime = millis();
    unsigned long timeoutMs = 10000; // Timeout di 10 secondi
    boolean responseReceived = false;
    boolean currentLineIsBlank = true;

    while (client.connected() && (millis() - startTime < timeoutMs)) {
        if (client.available()) {
            char c = client.read();
            Serial.write(c); // Debug seriale

            if (c == '\n' && currentLineIsBlank) {
                logMessage("DATA_TO_SERVER", "Response received from server");
                displayText("Response", "Data sent successfully.");
                responseReceived = true;
                delay(3000); // Visualizza messaggio successo per 3 secondi
                break;
            }
            if (c == '\n') {
                currentLineIsBlank = true;
            } else if (c != '\r') {
                currentLineIsBlank = false;
            }
        }
        delay(1); // Piccolo delay per evitare loop troppo stretto
    }

    if (!responseReceived) {
        logMessage("WARNING", "Timeout waiting for server response or server disconnected");
        displayText("Response", "No response/Timeout");
        setRGBcolor(RED); // Segnala errore con LED rosso
        delay(3000); // Visualizza errore per 3 secondi
    }


    // Chiudi la connessione
    delay(100);
    client.stop();
    digitalWrite(LED_AZURE, LOW);
    displayText("Disconnected", "Waiting 60s...");
    logMessage("DATA_TO_SERVER", "Disconnected from server, waiting 60 seconds before next data send");

    // Attendi 60 secondi (non bloccante con millis())
    unsigned long waitStartTime = millis();
    while (millis() - waitStartTime < 60000) {
        // Qui potresti fare altre operazioni non bloccanti se necessario
        delay(1); // Piccolo delay per non bloccare completamente
    }

    logMessage("DATA_TO_SERVER", "Waiting period of 60 seconds finished, ready for next data send cycle");
}