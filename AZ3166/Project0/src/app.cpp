#include "app.h"

// Definizione del costruttore
App::App() {
    // Inizializzazione delle variabili membro
    rgb[0] = {255, 0, 0};
    rgb[1] = {0, 255, 0};
    rgb[2] = {0, 0, 255};

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

void App::init() {
    textOutDevKitScreen(0, "INIT\r\n", 1);

    // Inizializzazione LED
    rgb[0] = {255, 0, 0};
    rgb[1] = {0, 255, 0};
    rgb[2] = {0, 0, 255};
    led = 0;
    initLeds();
    setRGBcolor(BLUE);

    // Configurazione WiFi
    ssid = "Casa RS";
    pass = "richksecure4967";
    initSensors();
    initWifi();

    // Imposta l'azione predefinita
    action = WIFISTATUS;

    buttonAState = 0;
    buttonBState = 0;
}

void App::loop() {
    refreshLeds();
    doAction();
    setRGBcolor(GREEN);
}

void App::doAction() {
    updateButtonsState();
    switch (action) {
        case SENSORSSERVERCLIENT:
            startSensorsServerClient();
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
            textOutDevKitScreen(1, "ERROR doAction", 1);
            textOutDevKitScreen(2, "default case", 1);
            break;
    }
}

void App::updateButtonsState() {
    if (IsButtonClicked(USER_BUTTON_A)) {
        switch (action) {
            case SENSORSSERVERCLIENT:
                action = WIFISTATUS;
                break;
            case WIFISTATUS:
                action = SENSORS;
                break;
            case SENSORS:
                action = SERVERHTTP;
                break;
            case SERVERHTTP:
                action = SENSORSSERVERCLIENT;
                break;
            default:
                textOutDevKitScreen(1, "ERROR updateButtonsState", 1);
                textOutDevKitScreen(2, "default case", 1);
                break;
        }
        delay(50);
    }
    if (IsButtonClicked(USER_BUTTON_B)) {
        if (action == SENSORS) {
            statusSensors = (statusSensors + 1) % NUMSENSORS;
            showSensors = true;
        }
        delay(50);
    }
}

void App::showSensorsInfo() {
    switch (statusSensors) {
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
    }
    delay(100);
}

void App::showWifiInfo() {
    printWifiStatus();
    delay(100);
}

void App::startSensorsServerClient() {
    WiFiClient client;
    const char* serverIp = "192.168.0.254";
    const unsigned short port = 80;  // Porta predefinita per HTTP

    if (client.connect(serverIp, port)) {
        digitalWrite(LED_AZURE, HIGH);
        textOutDevKitScreen(1, "sending data", 1);

        boolean currentLineIsBlank = true;
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                Serial.write(c);

                float pressure = 0;
                float temperaturePR = 0;
                pressureSensor->getPressure(&pressure);
                pressureSensor->getTemperature(&temperaturePR);
                ht_sensor->reset();
                float temperatureHT = 0;
                ht_sensor->getTemperature(&temperatureHT);
                float humidity = 0;
                ht_sensor->getHumidity(&humidity);

                if (c == '\n' && currentLineIsBlank) {
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: close");
                    client.println("Refresh: 2");
                    client.println("");
                    client.println("<!DOCTYPE HTML>");
                    client.println("<html><body>");
                    client.print("{");
                    client.print("\"temperature_sensor_1\":");
                    client.print(temperaturePR);
                    client.print(",");
                    client.print("\"temperature_sensor_2\":");
                    client.print(temperatureHT);
                    client.print(",");
                    client.print("\"pressure\":");
                    client.print(pressure);
                    client.print(",");
                    client.print("\"humidity\":");
                    client.print(humidity);
                    client.println("}");
                    client.println("</body></html>");
                    break;
                }
                if (c == '\n') {
                    currentLineIsBlank = true;
                } else if (c != '\r') {
                    currentLineIsBlank = false;
                }
            }
        }
        delay(1);
        client.stop();
        digitalWrite(LED_AZURE, LOW);
        textOutDevKitScreen(1, "client disconnected", 1);
    } else {
        textOutDevKitScreen(1, "connection failed", 1);
    }
}

void App::monitoringHTTPServerConnections(bool blockingSocket) {
    textOutDevKitScreen(0, "Server HTTP\r\n", 1);
    textOutDevKitScreen(1, "listening...\r\n\r\n", 1);

    WiFiServer wiFiServer(80);
    wiFiServer.begin();
    Serial.println("Server started and listening on port 80");

    IPAddress localIP = WiFi.localIP();
    String ipString = "IP: " + String(localIP[0]) + "." +
                             String(localIP[1]) + "." +
                             String(localIP[2]) + "." +
                             String(localIP[3]);

    textOutDevKitScreen(2, ipString.c_str(), 1);

    while (true) {
        WiFiClient client = wiFiServer.available();
        if (client) {
            Serial.println("New client connected");
            textOutDevKitScreen(0, "Client Connected", 1);
            digitalWrite(LED_AZURE, HIGH);

            boolean currentLineIsBlank = true;
            while (client.connected()) {
                if (client.available()) {
                    char c = client.read();
                    Serial.write(c);

                    float pressure = 0;
                    float temperaturePR = 0;
                    pressureSensor->getPressure(&pressure);
                    pressureSensor->getTemperature(&temperaturePR);
                    ht_sensor->reset();
                    float temperatureHT = 0;
                    ht_sensor->getTemperature(&temperatureHT);
                    float humidity = 0;
                    ht_sensor->getHumidity(&humidity);

                    if (c == '\n' && currentLineIsBlank) {
                        Serial.println("Sending HTTP response");

                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-Type: text/html");
                        client.println("Connection: close");
                        client.println("Refresh: 2");
                        client.println("");
                        client.println("<!DOCTYPE HTML>");
                        client.println("<html><body>");
                        client.print("{");
                        client.print("\"temperature_sensor_1\":");
                        client.print(temperaturePR);
                        client.print(",");
                        client.print("\"temperature_sensor_2\":");
                        client.print(temperatureHT);
                        client.print(",");
                        client.print("\"pressure\":");
                        client.print(pressure);
                        client.print(",");
                        client.print("\"humidity\":");
                        client.print(humidity);
                        client.println("}");
                        client.println("</body></html>");
                        break;
                    }
                    if (c == '\n') {
                        currentLineIsBlank = true;
                    } else if (c != '\r') {
                        currentLineIsBlank = false;
                    }
                }
            }

            textOutDevKitScreen(1, "Client Disconnected", 1);
            textOutDevKitScreen(2, "Processing...", 1);
            textOutDevKitScreen(3, "Waiting...", 1);
            delay(1000);

            digitalWrite(LED_AZURE, LOW);
            client.stop();
        } else {
            textOutDevKitScreen(0, "Server HTTP\r\n", 1);
            textOutDevKitScreen(1, "listening...\r\n\r\n", 1);
            textOutDevKitScreen(2, ipString.c_str(), 1);
            delay(100);
        }
    }
}

void App::refreshLeds() {
    int irda_status = IrdaSensor->IRDATransmit(&counter, 1, 100);
    if (irda_status != 0) {
        snprintf(buffInfo, sizeof(buffInfo), "Unable to transmit through IRDA\r\n");
        textOutDevKitScreen(1, buffInfo, 1);
        delay(2000);
    }
}

void App::setRGBcolor(COLOR color2) {
    color = color2;
    rgbLed.setColor(rgb[color].red, rgb[color].green, rgb[color].blue);
}

void App::initSerial() {
    Serial.begin(115200);
    while (!Serial) { ; } // Attendi che la connessione seriale sia stabilita
}

void App::initWifi() {
    if (WiFi.status() == WL_NO_SHIELD) {
        textOutDevKitScreen(1, "WiFi shield not present\r\n", 1);
        while (true);
    }

    int attempt = 0;
    while (status != WL_CONNECTED) {
        snprintf(buffInfo, sizeof(buffInfo), "Attempt %d to connect to SSID %s\r\n", ++attempt, ssid);
        textOutDevKitScreen(1, buffInfo, 1);

        status = WiFi.begin(ssid, pass);
        delay(1000);
    }
    printWifiStatus();
}

void App::printWifiStatus() {
    IPAddress ip = WiFi.localIP();
    long rssi = WiFi.RSSI();

    // Costruzione manuale della stringa IP
    char ipString[16];
    snprintf(ipString, sizeof(ipString), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    // Assicurati che il buffer sia sufficiente e formato correttamente
    snprintf(buffInfo, sizeof(buffInfo), "WIFI STATUS\r\nSSID: %s\r\nRSSI: %ld dBm\r\nIP:%s\r\n", WiFi.SSID(), rssi, ipString);

    // Debug: Stampa il buffer sulla seriale
    Serial.println(buffInfo); // Solo per il debug, rimuovi questa riga se non necessaria

    // Visualizza il testo sul display
    textOutDevKitScreen(0, buffInfo, 1);
    delay(100);
}


void App::initLeds() {
    pinMode(LED_WIFI, OUTPUT);
    pinMode(LED_AZURE, OUTPUT);
    pinMode(LED_USER, OUTPUT);
    rgbLed.turnOff();
}

void App::initSensors() {
    ext_i2c = new DevI2C(D14, D15);
    acc_gyro = new LSM6DSLSensor(*ext_i2c, D4, D5);
    acc_gyro->init(NULL);
    acc_gyro->enableAccelerator();
    acc_gyro->enableGyroscope();

    ht_sensor = new HTS221Sensor(*ext_i2c);
    ht_sensor->init(NULL);

    magnetometer = new LIS2MDLSensor(*ext_i2c);
    magnetometer->init(NULL);

    IrdaSensor = new IRDASensor();
    IrdaSensor->init();

    pressureSensor = new LPS22HBSensor(*ext_i2c);
    pressureSensor->init(NULL);

    statusSensors = 4;
    counter = 0;
    showSensors = false;
}

void App::showMotionGyroSensor() {
    acc_gyro->getGAxes(axes);
    snprintf(buffInfo, sizeof(buffInfo), "Gyroscope \r\n    x:%d   \r\n    y:%d   \r\n    z:%d  ", axes[0], axes[1], axes[2]);
    textOutDevKitScreen(0, buffInfo, 1);
}

void App::showMotionAccelSensor() {
    acc_gyro->getXAxes(axes);
    snprintf(buffInfo, sizeof(buffInfo), "Accelerometer \r\n    x:%d   \r\n    y:%d   \r\n    z:%d  ", axes[0], axes[1], axes[2]);
    textOutDevKitScreen(0, buffInfo, 1);
}

void App::showPressureSensor() {
    float pressure = 0;
    float temperature = 0;
    pressureSensor->getPressure(&pressure);
    pressureSensor->getTemperature(&temperature);
    snprintf(buffInfo, sizeof(buffInfo), "Environment\r\nPressure: %shPa\r\nTemp: %sC \r\n", f2s(pressure, 2), f2s(temperature, 1));
    textOutDevKitScreen(0, buffInfo, 1);
}

void App::showHumidTempSensor() {
    ht_sensor->reset();
    float temperature = 0;
    ht_sensor->getTemperature(&temperature);
    temperature = temperature * 1.8 + 32; // Convert from Celsius to Fahrenheit
    float humidity = 0;
    ht_sensor->getHumidity(&humidity);
    snprintf(buffInfo, sizeof(buffInfo), "Environment \r\n Temp:%sF    \r\n Humidity:%s%% \r\n", f2s(temperature, 1), f2s(humidity, 1));
    textOutDevKitScreen(0, buffInfo, 1);
}

void App::showMagneticSensor() {
    magnetometer->getMAxes(axes);
    snprintf(buffInfo, sizeof(buffInfo), "Magnetometer  \r\n    x:%d     \r\n    y:%d     \r\n    z:%d     ", axes[0], axes[1], axes[2]);
    textOutDevKitScreen(0, buffInfo, 1);
}

bool App::IsButtonClicked(unsigned char ulPin) {
    pinMode(ulPin, INPUT);
    return digitalRead(ulPin) == LOW;
}
