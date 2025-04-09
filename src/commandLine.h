#pragma once
#include "jimlib.h"

// TODO: move CommandLineInterface stuff here, tie it in with JStuff independently 


// registers SETTEMP, CURRENTTEMP, HIST commands with CLI.  Returns heat on/off
// value from check() function 
class CliTempControl {
    public:
        CliVariable<float> setTemp;
        CliVariable<float> currentTemp;
        CliVariable<float> hist;
        CliVariable<int> heat;
        CliVariable<float> minTemp;
        JStuff *j;
    
        CliTempControl(JStuff *js, float temp, float h) :
            j(js), setTemp(js->cli, "setTemp", temp),
            currentTemp(js->cli, "currentTemp", temp), 
            hist(js->cli, "hist", h),
            heat(js->cli, "heat", 0),
            minTemp(js->cli, "minTemp", 5) {}
    
        bool check(float temp) { 
            currentTemp = temp;
            if (temp > setTemp) { 
                heat = 0;
            } else if (temp >= minTemp && temp < setTemp - hist) { 
                heat = 1;
            }
            return heat;
        }	
        void pub() { 
            j->out("setTemp: %6.2f currentTemp: %6.2f heat: %d", (float)setTemp, (float)currentTemp, (int)heat);
        }
    };
    