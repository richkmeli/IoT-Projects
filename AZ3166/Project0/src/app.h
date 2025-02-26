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
#include <Arduino.h>
#include <OTAFirmwareUpdate.h>
#include "FATFileSystem.h"
#include "SFlashBlockDevice.h"
#include "fatfs_exfuns.h"
#include "AudioClassV2.h"

#include <SPI.h>
#include <OledDisplay.h>
#include <AZ3166WiFiClient.h>
#include <AZ3166WiFiServer.h>
#include <AZ3166WiFiUdp.h>
#include "AZ3166WiFi.h"
#include "http_client.h"
#include "SystemVersion.h"
#include "SystemTickCounter.h"
#include "telemetry.h"

#include <fatfs_exfuns.h>
#include <SFlashBlockDevice.h>

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
    RED = 0,
    GREEN = 1,
    BLUE = 2,
    YELLOW = 3,
    CYAN = 4,
    MAGENTA = 5
};

class App {
private:
    TagRGB rgb[3];
    char buffInfo[128];
    int buttonAState;
    int buttonBState;

    char *ssid;      // your network SSID (name)
    char *pass;      // your network password
    int keyIndex = 0; // your network key Index number (needed only for WEP)
    int status = WL_IDLE_STATUS;

    int statusSensors;
    bool showSensors;
    unsigned char counter;
    Action action;
    RGB_LED rgbLed;
    COLOR color;
    int led;

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
    OLEDDisplay display;

public:
    App();  // Costruttore dichiarato, ma senza definizione nel file header
    void init();
    void loop();
    void doAction();
    void updateButtonsState();
    void showSensorsInfo();
    void showWifiInfo();
    void sendSensorDataToServer();
    void monitoringHTTPServerConnections(bool blockingSocket);
    void refreshLeds();
    void setRGBcolor(COLOR color2);
    void initSerial();
	bool initWifi();
    void printWifiStatus();
    void initLeds();
    void initSensors();
    void showMotionGyroSensor();
    void showMotionAccelSensor();
    void showPressureSensor();
    void showHumidTempSensor();
    void showMagneticSensor();
    bool IsButtonClicked(unsigned char ulPin);
  	void displayText(const char* title, const char* body);
    void enterSleepModeWithDisplay(unsigned long sleepDuration);
};

#endif // APP_H_
