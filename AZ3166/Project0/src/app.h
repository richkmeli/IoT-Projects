#ifndef APP_H_
#define APP_H_

#include <HTS221Sensor.h>
#include <IoT_DevKit_HW.h>
#include <IrDASensor.h>
#include <LIS2MDLSensor.h>
#include <LPS22HBSensor.h>
#include <LSM6DSLSensor.h>
#include <RGB_LED.h>
#include <Sensor.h>

#include <SPI.h>
#include <AZ3166WiFiClient.h>
#include <AZ3166WiFiServer.h>
#include <AZ3166WiFiUdp.h>
#include "AZ3166WiFi.h"
#include "http_client.h"
#include "IoT_DevKit_HW.h"
#include "SystemVersion.h"
#include "SystemTickCounter.h"
#include "telemetry.h"

#include <fatfs_exfuns.h>
#include <SFlashBlockDevice.h>

// 0 - Humidity & Temperature Sensor
// 1 - Pressure Sensor
// 2 - Magnetic Sensor
// 3 - Gyro Sensor
// 4 - Motion (Acceleration) Sensor
#define NUMSENSORS 5  // 5 sensors to display

#define SWAP(a, b) ((a) ^= (b), (b) ^= (a), (a) ^= (b))

enum Action {
    SENSORSSERVERCLIENT = 0, WIFISTATUS = 1, SENSORS = 2, SERVERHTTP = 3
};

class TagRGB {
public:
    int red;
    int green;
    int blue;
};

enum COLOR {
    RED = 0, GREEN = 1, BLUE = 2
};

class App {
private:
    TagRGB rgb[3];
    /*static*/ char buffInfo[128];
    /*static*/ int buttonAState;
    /*static*/ int buttonBState;

    char *ssid;      //  your network SSID (name)
    char *pass;   // your network password
    int keyIndex = 0;                 // your network key Index number (needed only for WEP)
    int status = WL_IDLE_STATUS;

    //WiFiServer wiFiServer;

    /*static*/ int statusSensors;
    /*static*/ bool showSensors;
//static bool showWifiInfoB;
//static bool isConnected;
    /*static*/ unsigned char counter;

    /*static*/ Action action;
    /*static*/ RGB_LED rgbLed;
    /*static*/ COLOR color;

    /*static*/int led;

    DevI2C *ext_i2c;
    LSM6DSLSensor *acc_gyro;
    HTS221Sensor *ht_sensor;
    LIS2MDLSensor *magnetometer;
    IRDASensor *IrdaSensor;
    LPS22HBSensor *pressureSensor;

    int axes[3];
    char wifiBuff[128];
    char firmwareBuff[128];
    char firmwareTelemetryBuff[64];
public:
    App() {}
    /*static*/bool init();
    /*static*/bool loop();
    void doAction();
    void updateButtonsState();
    void showSensorsInfo();
    void showWifiInfo();
    void startSensorsServerClient();
    void monitoringHTTPServerConnections(bool blockingSocket);
    void refreshLeds();
    void setRGBcolor(COLOR color2);
    void initSerial();
    void initWifi();
    void printWifiStatus();
    void initLeds();
    void initSensors();
    void showMotionGyroSensor();
    void showMotionAccelSensor();
    void showPressureSensor();
    void showHumidTempSensor();
    void showMagneticSensor();
    bool IsButtonClicked(unsigned char ulPin);
    const char *RC4EncryptDecrypt(const char *pszText, const char *pszKey);
};

#endif // APP_H_