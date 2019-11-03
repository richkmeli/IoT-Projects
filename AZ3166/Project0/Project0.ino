/*
 * Project 0:
 * Sensors - WebServer - WifiInfo
 * */
#include "src/app.h"

static App app;

void setup(){
    //App::init();
    app.init();
}

void loop(){
   app.loop();
    //App::loop();
}
