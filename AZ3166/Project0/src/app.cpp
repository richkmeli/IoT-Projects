
#include "app.h"

bool App::init() {
    textOutDevKitScreen(0, "INIT\r\n", 1);
    // LEDS
    rgb[0] = {255, 0, 0};
    rgb[1] = {0, 255, 0};
    rgb[2] = {0, 0, 255};

    led = 0;
    initLeds();
    // setup color
    setRGBcolor(BLUE);

    ssid = (char *) "Casa RS";      //  your network SSID (name)
    pass = (char *) "richksecure4967";   // your network password

    // initSerial();
    initSensors();
    initWifi();
//    WiFiServer wiFiServer = WiFiServer(80);
//    wiFiServer.begin();

    // default action
    action = WIFISTATUS;

    buttonAState = 0;
    buttonBState = 0;
}

bool App::loop() {
    /* textOutDevKitScreen(0, "LOOP\r\n", 1);
     textOutDevKitScreen(1, "", 1);
     textOutDevKitScreen(2, "", 1);
     textOutDevKitScreen(3, "", 1);*/

    refreshLeds();

    doAction();
    setRGBcolor(GREEN);
}

void App::doAction() {
    updateButtonsState();
    switch (action) {
        case SENSORSSERVERCLIENT :
            startSensorsServerClient();
            break;
        case WIFISTATUS :
            showWifiInfo();
            break;
        case SENSORS :
            showSensorsInfo();
            break;
        case SERVERHTTP :
            // TODO multithread
            monitoringHTTPServerConnections(false);
            break;
        default :
            textOutDevKitScreen(1, "ERROR doAction", 1);
            textOutDevKitScreen(2, "default case", 1);
            textOutDevKitScreen(3, "", 1);
            break;
    }
}

void App::updateButtonsState() {
    if (IsButtonClicked(USER_BUTTON_A)) {
        switch (action) {
            case SENSORSSERVERCLIENT :
                action = WIFISTATUS;
                break;
            case WIFISTATUS :
                action = SENSORS;
                break;
            case SENSORS :
                action = SERVERHTTP;
                break;
            case SERVERHTTP :
                action = SENSORSSERVERCLIENT;
                break;
            default :
                textOutDevKitScreen(1, "ERROR updateButtonsState", 1);
                textOutDevKitScreen(2, "default case", 1);
                textOutDevKitScreen(3, "", 1);
                break;
        }

        //showSensors = false;
        //showWifiInfoB = true;
        delay(50);
    }
    if (IsButtonClicked(USER_BUTTON_B)) {
        if (action == SENSORS) {
            statusSensors = (statusSensors + 1) % NUMSENSORS;
            showSensors = true;
        }
        //showWifiInfoB = false;
        delay(50);
    }


}

void App::showSensorsInfo() {
    //if(showSensors){
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
    // }
}

void App::showWifiInfo() {
    //if(showWifiInfoB){
    printWifiStatus();
    delay(100);
    // }
}

void App::startSensorsServerClient() {
    WiFiClient client;
    const char* serverIp = "192.168.0.254";
    const unsigned short port = 80;  // Porta predefinita per HTTP

    if (client.connect(serverIp, port)) {
        digitalWrite(LED_AZURE, 1);
        textOutDevKitScreen(1, "sending data", 1);
        textOutDevKitScreen(2, "", 1);
        textOutDevKitScreen(3, "", 1);
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
        digitalWrite(LED_AZURE, 0);
        textOutDevKitScreen(1, "client disconnected", 1);
        textOutDevKitScreen(2, "", 1);
        textOutDevKitScreen(3, "", 1);
    } else {
        textOutDevKitScreen(1, "connection failed", 1);
    }
}

void App::monitoringHTTPServerConnections(bool blockingSocket) {
    // Mostra lo stato iniziale
    textOutDevKitScreen(0, "Server HTTP\r\n", 1);
    textOutDevKitScreen(1, "listening...\r\n\r\n", 1);
    textOutDevKitScreen(2, "                 ", 1);
    textOutDevKitScreen(3, "                 ", 1);

    // Inizializza il server sulla porta 80
    WiFiServer wiFiServer(80);
    wiFiServer.begin();
    Serial.println("Server started and listening on port 80");

    // Ottieni l'IP locale del dispositivo
    IPAddress localIP = WiFi.localIP();

    // Costruisci la stringa dell'indirizzo IP manualmente
    String ipString = "IP: ";
    ipString += localIP[0];
    ipString += ".";
    ipString += localIP[1];
    ipString += ".";
    ipString += localIP[2];
    ipString += ".";
    ipString += localIP[3];

    // Mostra l'IP del server
    textOutDevKitScreen(2, ipString.c_str(), 1);

    while (true) {
        WiFiClient client = wiFiServer.available();
        if (client) {
            Serial.println("New client connected");
            textOutDevKitScreen(0, "Client Connected", 1);
            digitalWrite(LED_AZURE, 1);  // Accende l'indicatore LED per il client

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

            // Messaggio di stato finale sul display
            textOutDevKitScreen(1, "Client Disconnected", 1);
            textOutDevKitScreen(2, "Processing...", 1);
            textOutDevKitScreen(3, "Waiting...", 1);
            delay(1000);  // Mostra il messaggio di disconnessione per un secondo

            // Spegne l'indicatore LED
            digitalWrite(LED_AZURE, 0);
            client.stop();
        } else {
            // Messaggio di attesa per nuovi client
            textOutDevKitScreen(0, "Server HTTP\r\n", 1);
            textOutDevKitScreen(1, "listening...\r\n\r\n", 1);
            textOutDevKitScreen(2, ipString.c_str(), 1);
            textOutDevKitScreen(3, "", 1);
            delay(100);  // Breve ritardo per evitare l'uso eccessivo della CPU
        }
    }
}


void App::refreshLeds() {
    /*Blink around every 0.5 sec*/
    // counter++;
    int irda_status = IrdaSensor->IRDATransmit(&counter, 1, 100);
    if (irda_status != 0) {
        snprintf(buffInfo, sizeof(buffInfo), "Unable to tran\r\nsmit through IRDA\r\n\r\n");
        textOutDevKitScreen(1, buffInfo, 1);
        delay(2000);
    }


    // if(counter > 5){
    //digitalWrite(LED_WIFI, led);
    //digitalWrite(LED_AZURE, led);
    //digitalWrite(LED_USER, led);
    //led = !led;

    // rgbLed.setColor(rgb[color].red, rgb[color].green, rgb[color].blue);
    // color = (color + 1) % (sizeof(rgb) / sizeof(struct _tagRGB));
    //     counter = 0;
    // }
}

void App::setRGBcolor(COLOR color2) {
    color = color2;// % (sizeof(rgb) / sizeof(struct _tagRGB));
    rgbLed.setColor(rgb[color].red, rgb[color].green, rgb[color].blue);
}


void App::initSerial() {
    //Initialize serial and wait for port to open:
    Serial.begin(115200);
    while (!Serial) { ; // wait for serial port to connect. Needed for Leonardo only
    }
}

void App::initWifi() {
    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD) {
        //Serial.println("WiFi shield not present");
        textOutDevKitScreen(1, "WiFi shield not present\r\n", 1);
// don't continue:
        while (true);
    }

    // attempt to connect to Wifi network:
    int i = 0;
    while (status != WL_CONNECTED) {

        snprintf(buffInfo, sizeof(buffInfo), "Attempt %d to \r\nconnect to SSID\r\n%s \r\n", ++i, ssid);
        textOutDevKitScreen(1, buffInfo, 1);

        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);

        // wait 10 seconds for connection:
        delay(1000);
    }
    printWifiStatus();
}

void App::printWifiStatus() {
    // print the SSID and received signal strength of the network you're attached to:
    IPAddress ip = WiFi.localIP();
    long rssi = WiFi.RSSI();
    // Get IP address
    ip = WiFi.localIP();

    snprintf(buffInfo, sizeof(buffInfo), "WIFI STATUS\r\nSSID: %s\r\nRSSI: %ld dBm \r\nIP: %s\r\n", WiFi.SSID(), rssi,
             ip.get_address());
    textOutDevKitScreen(0, buffInfo, 1);
    delay(50);

    // you're connected now, so print out the status:
    //printWifiStatus();
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
    //isConnected = false;
}

void App::showMotionGyroSensor() {
    acc_gyro->getGAxes(axes);
    char buff[128];
    snprintf(buffInfo, sizeof(buffInfo), "Gyroscope \r\n    x:%d   \r\n    y:%d   \r\n    z:%d  ", axes[0], axes[1],
             axes[2]);
    textOutDevKitScreen(0, buffInfo, 1);
}

void App::showMotionAccelSensor() {
    acc_gyro->getXAxes(axes);
    char buff[128];
    snprintf(buffInfo, sizeof(buffInfo), "Accelorometer \r\n    x:%d   \r\n    y:%d   \r\n    z:%d  ", axes[0], axes[1],
             axes[2]);
    textOutDevKitScreen(0, buffInfo, 1);
}

void App::showPressureSensor() {
    float pressure = 0;
    float temperature = 0;
    pressureSensor->getPressure(&pressure);
    pressureSensor->getTemperature(&temperature);
    char buff[128];

    snprintf(buffInfo, sizeof(buffInfo), "Environment\r\nPressure: \r\n    %shPa\r\nTemp: %sC \r\n", f2s(pressure, 2),
             f2s(temperature, 1));
    textOutDevKitScreen(0, buffInfo, 1);
}

void App::showHumidTempSensor() {
    ht_sensor->reset();
    float temperature = 0;
    ht_sensor->getTemperature(&temperature);
    //convert from C to F
    temperature = temperature * 1.8 + 32;
    float humidity = 0;
    ht_sensor->getHumidity(&humidity);

    char buff[128];
    snprintf(buffInfo, sizeof(buffInfo), "Environment \r\n Temp:%sF    \r\n Humidity:%s%% \r\n          \r\n",
             f2s(temperature, 1), f2s(humidity, 1));
    textOutDevKitScreen(0, buffInfo, 1);
}

void App::showMagneticSensor() {
    magnetometer->getMAxes(axes);
    char buff[128];
    snprintf(buffInfo, sizeof(buffInfo), "Magnetometer  \r\n    x:%d     \r\n    y:%d     \r\n    z:%d     ", axes[0],
             axes[1], axes[2]);
    textOutDevKitScreen(0, buffInfo, 1);
}

bool App::IsButtonClicked(unsigned char ulPin) {
    pinMode(ulPin, INPUT);
    int buttonState = digitalRead(ulPin);
    if (buttonState == LOW) {
        return true;
    }
    return false;
}


const char *App::RC4EncryptDecrypt(const char *pszText, const char *pszKey) {
    unsigned char sbox[256];
    unsigned char key[256], k;
    int m, n, i, j, ilen;
    char *res = (char *) pszText;

    memset(sbox, 0, 256);
    memset(key, 0, 256);

    i = 0, j = 0, n = 0;
    ilen = strlen(pszKey);

    for (m = 0; m < 256; m++)  /* Initialize the key sequence */
    {
        *(key + m) = pszKey[(m % ilen)];
        *(sbox + m) = m;
    }
    for (m = 0; m < 256; m++) {
        n = (n + *(sbox + m) + *(key + m)) & 0xff;
        SWAP(*(sbox + m), *(sbox + n));
    }

    ilen = strlen(pszText);
    for (m = 0; m < ilen; m++) {
        i = (i + 1) & 0xff;
        j = (j + *(sbox + i)) & 0xff;
        SWAP(*(sbox + i), *(sbox + j));  /* randomly Initialize
              the key sequence */
        k = *(sbox + ((*(sbox + i) + *(sbox + j)) & 0xff));
        if (k == pszText[m])       /* avoid '\0' among the
              encoded text; */
            k = 0;
        //*(res + m) ^= k;
        res[m] = (res[m]) ^ k;
    }

    /* remove Key traces in memory  */
    memset(sbox, 0, 256);
    memset(key, 0, 256);

    return res;
}

