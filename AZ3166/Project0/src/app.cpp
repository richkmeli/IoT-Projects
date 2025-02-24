#include "app.h"

// Definizione delcostruttore
App::App()
{
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

void App::init()
{
   displayText("INIT", "init...");

   displayText("INIT", "led...");
   // Inizializzazione LED
   rgb[0] = {255, 0, 0};
   rgb[1] = {0, 255, 0};
   rgb[2] = {0, 0, 255};
   led = 0;
   initLeds();
   setRGBcolor(BLUE);
   displayText("INIT", "led OK");

   displayText("INIT", "sensors...");
   initSensors();
   displayText("INIT", "sensors OK");

   displayText("INIT", "WIFI...");
   // Configurazione WiFi
   ssid = "Casa RS";
   pass = "richksecure4967";
   initWifi();
   displayText("INIT", "WIFI OK");

   // Imposta l'azione predefinita
   action = SENSORSSERVERCLIENT;

   buttonAState = 0;
   buttonBState = 0;
}

void App::loop()
{
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
         break;
      }
      delay(50);
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
      }
      delay(50);
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
      snprintf(buffInfo, sizeof(buffInfo), "Unable to transmit through IRDA§");
      displayText("refreshLeds", buffInfo);
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
}

void App::initWifi()
{
   if (WiFi.status() == WL_NO_SHIELD)
   {
      displayText("initWifi", "WiFi shield not present");
      while (true)
         ;
   }

   int attempt = 0;
   while (status != WL_CONNECTED)
   {
      snprintf(buffInfo, sizeof(buffInfo), "Attempt %d to connect to SSID %s§", ++attempt, ssid);
      displayText("initWifi", buffInfo);

      status = WiFi.begin(ssid, pass);
      delay(1000);
   }
   printWifiStatus();
}

void App::printWifiStatus()
{
   IPAddress ip = WiFi.localIP();
   long rssi = WiFi.RSSI();

   // Costruzione manuale della stringa IP
   char ipString[16];
   snprintf(ipString, sizeof(ipString), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

   // Assicurati che il buffer sia sufficiente e formato correttamente
   snprintf(buffInfo, sizeof(buffInfo), "SSID: %s§RSSI: %ld dBm§IP:%s", WiFi.SSID(), rssi, ipString);

   // Debug: Stampa il buffer sulla seriale
   Serial.println(buffInfo); // Solo per il debug, rimuovi questa riga se non necessaria

   // Visualizza il testo sul display
   displayText("WIFI STATUS", buffInfo);
   delay(100);
}

void App::initLeds()
{
   pinMode(LED_WIFI, OUTPUT);
   pinMode(LED_AZURE, OUTPUT);
   pinMode(LED_USER, OUTPUT);
   rgbLed.turnOff();
}

void App::initSensors()
{
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

void App::showMotionGyroSensor()
{
   acc_gyro->getGAxes(axes);
   snprintf(buffInfo, sizeof(buffInfo), "x: %d§y: %d§z: %d", axes[0], axes[1], axes[2]);
   displayText("Gyroscope", buffInfo);
}

void App::showMotionAccelSensor()
{
   acc_gyro->getXAxes(axes);
   snprintf(buffInfo, sizeof(buffInfo), "x: %d§y: %d§z: %d", axes[0], axes[1], axes[2]);
   displayText("Accelerometer", buffInfo);
}

void App::showPressureSensor()
{
   float pressure = 0;
   float temperature = 0;
   pressureSensor->getPressure(&pressure);
   pressureSensor->getTemperature(&temperature);

   // Temperatura percepita (Wind Chill) come esempio
   float windChill = 13.12 + 0.6215 * temperature - 11.37 * pow(pressure, 0.16) + 0.3965 * temperature * pow(pressure, 0.16);

   snprintf(buffInfo, sizeof(buffInfo), "P: %shPa§Temp: %sC§Wind Chill: %.1f°C", f2s(pressure, 2), f2s(temperature, 1), windChill);
   displayText("Environment 1", buffInfo);
}

void App::showHumidTempSensor()
{
   ht_sensor->reset();
   float temperatureC = 0;
   ht_sensor->getTemperature(&temperatureC);
   float humidity = 0;
   ht_sensor->getHumidity(&humidity);

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
}

void App::showMagneticSensor()
{
   magnetometer->getMAxes(axes);
   snprintf(buffInfo, sizeof(buffInfo), "x: %d§y: %d§z: %d", axes[0], axes[1], axes[2]);
   displayText("Magnetometer", buffInfo);
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

   WiFiServer wiFiServer(80);
   wiFiServer.begin();
   Serial.println("Server started and listening on port 80");

   IPAddress localIP = WiFi.localIP();
   String ipString = "IP: " + String(localIP[0]) + "." +
                     String(localIP[1]) + "." +
                     String(localIP[2]) + "." +
                     String(localIP[3]);

   // Display IP address
   displayText("Server HTTP", ("Listening...§" + ipString).c_str());

   while (true)
   {
      WiFiClient client = wiFiServer.available();
      if (client)
      {
         Serial.println("New client connected");
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
                  Serial.println("Sending HTTP response");

                  // Collect sensor data
                  float pressure = 0;
                  float temperaturePR = 0;
                  pressureSensor->getPressure(&pressure);
                  pressureSensor->getTemperature(&temperaturePR);

                  ht_sensor->reset();
                  float temperatureHT = 0;
                  ht_sensor->getTemperature(&temperatureHT);
                  float humidity = 0;
                  ht_sensor->getHumidity(&humidity);

                  // Get accelerometer and gyroscope data
                  int accelAxes[3] = {0};
                  int gyroAxes[3] = {0};

                  // Use the correct methods to get sensor data
                  acc_gyro->getXAxes(accelAxes); // Get accelerometer axes
                  acc_gyro->getGAxes(gyroAxes);  // Get gyroscope axes

                  // Get magnetometer data
                  int magAxes[3] = {0};
                  magnetometer->getMAxes(magAxes); // Get magnetometer axes

                  // Create JSON response
                  String jsonResponse = "{";
                  jsonResponse += "\"temperature_sensor_1\":" + String(temperaturePR) + ",";
                  jsonResponse += "\"temperature_sensor_2\":" + String(temperatureHT) + ",";
                  jsonResponse += "\"pressure\":" + String(pressure) + ",";
                  jsonResponse += "\"humidity\":" + String(humidity) + ",";
                  jsonResponse += "\"accelero_x\":" + String(accelAxes[0]) + ",";
                  jsonResponse += "\"accelero_y\":" + String(accelAxes[1]) + ",";
                  jsonResponse += "\"accelero_z\":" + String(accelAxes[2]) + ",";
                  jsonResponse += "\"gyro_x\":" + String(gyroAxes[0]) + ",";
                  jsonResponse += "\"gyro_y\":" + String(gyroAxes[1]) + ",";
                  jsonResponse += "\"gyro_z\":" + String(gyroAxes[2]) + ",";
                  jsonResponse += "\"mag_x\":" + String(magAxes[0]) + ",";
                  jsonResponse += "\"mag_y\":" + String(magAxes[1]) + ",";
                  jsonResponse += "\"mag_z\":" + String(magAxes[2]);
                  jsonResponse += "}";

                  // Send HTTP response
                  client.println("HTTP/1.1 200 OK");
                  client.println("Content-Type: application/json");
                  client.println("Connection: close");
                  client.println("");
                  client.println(jsonResponse);

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
         delay(1000);

         digitalWrite(LED_AZURE, LOW);
         client.stop();
      }
      else
      {
         displayText("Server HTTP", ("Listening...§" + ipString).c_str());
         delay(100);
      }
   }
}



void App::sendSensorDataToServer(){
   displayText("SendDataToServer", "Initializing...");

   const char *serverHost = "192.168.0.218";
   const char *serverPath = "/sdl/upload-sensor-data";
   const unsigned short port = 8080;

   WiFiClient client;

   // Ciclo continuo per invio periodico (modalità esclusiva)
  // while (true){
      displayText("Sensor Data", "Starting data send...");
      displayText("Connecting", "Connecting to server...");

      if (client.connect(serverHost, port))
      {
         digitalWrite(LED_AZURE, HIGH);
         displayText("Connected", "Sending data...");

         // Raccogli i dati dai sensori
         float pressure = 0;
         float temperaturePR = 0;
         pressureSensor->getPressure(&pressure);
         pressureSensor->getTemperature(&temperaturePR);

         ht_sensor->reset();
         float temperatureHT = 0;
         ht_sensor->getTemperature(&temperatureHT);
         float humidity = 0;
         ht_sensor->getHumidity(&humidity);

         // Crea il JSON dei dati dei sensori
         String jsonPayload = "{";
         jsonPayload += "\"temperature_sensor_1\":" + String(temperaturePR) + ",";
         jsonPayload += "\"temperature_sensor_2\":" + String(temperatureHT) + ",";
         jsonPayload += "\"pressure\":" + String(pressure) + ",";
         jsonPayload += "\"humidity\":" + String(humidity);
         jsonPayload += "}";

         // Prepara la richiesta POST
         client.println("POST " + String(serverPath) + " HTTP/1.1");
         client.println("Host: " + String(serverHost));
         client.println("Content-Type: application/json");
         client.println("Content-Length: " + String(jsonPayload.length()));
         client.println("Connection: close");
         client.println("x-api-key: fweW2qo21qxmoCECWf23d"); // Inserisci qui la tua chiave
         client.println("");          // Fine degli header
         client.println(jsonPayload); // Corpo della richiesta

         displayText("Data Sent", "Waiting for response...");

         // Attendi la risposta del server
         boolean currentLineIsBlank = true;
         while (client.connected())
         {
            if (client.available())
            {
               char c = client.read();
               Serial.write(c); // Debug via seriale

               if (c == '\n' && currentLineIsBlank)
               {
                  Serial.println("Response received");
                  displayText("Response", "Data sent successfully.");
                  delay(3000);
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

         // Chiudi la connessione
         delay(100);
         client.stop();
         digitalWrite(LED_AZURE, LOW);
         displayText("Disconnected", "Waiting 60s...");
      }
      else
      {
         displayText("Connection", "Failed. Retrying...");
      }

      // Attendi 60 secondi prima di un nuovo invio
      delay(60000);
 //  }
}
