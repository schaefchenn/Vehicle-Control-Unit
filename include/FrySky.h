/* Copyright (c) 2024 AUDEx Project, Victor Glekler

This code is part of the AUDEx project, developed for controlling vehicles as part of the Automotive Development in 1:x
initiative. It is intended for educational and non-commercial use.

Permission is granted to use, modify, and distribute this code for educational purposes, provided that proper credit is given 
to the original developers. This code is provided "as-is" without any warranties or guarantees of functionality, safety, or fitness for any particular purpose.

For any questions or inquiries, please contact https://github.com/VICLER */

#include <Arduino.h>
#include "SBUS.h"

enum rx_mode_enum{
  PPM_MODE = 0,
  SBUS_MODE
};

#define RX_THROTTLE_CH  0
#define RX_STEERING_CH  1
#define RX_MAX_CHANNELS 8

struct PPMData {
  bool available = 0;
  bool failsafe = 0;
  uint16_t channels[RX_MAX_CHANNELS] = {1500,1500,1500,1500,1500,1500,1500,1500};
};

volatile static PPMData _ppm_data;  // ppm data buffer used in interrupt.

struct SBUSData{
  float channelsCal[RX_MAX_CHANNELS] = {1, 1, 1, 1, 1, 1, 1, 1};
  uint16_t channels[RX_MAX_CHANNELS] = {1500,1500,1500,1500,1500,1500,1500,1500};
  bool failSafe;
  bool lostFrame;
  bool available;
};

enum drive_mode_enum{
  DRIVE_MODE_XBOX = 1,
  DRIVE_MODE_CAN,
  DRIVE_MODE_XBOX_LIMITED,
  DRIVE_MODE_RADIO_RX
};


static void PPM_ISR(); // isr for ppm receiver signal
SBUS sbusReceiver = SBUS(Serial1);  // hardware serial 1 for sbus receiver
const int receiverPin = 4;  // radio receiver pin
const uint8_t rxMode = PPM_MODE;  // default rx mode

struct FRYSKY{
  uint8_t steeringAngle;
  uint16_t throttle;
};



//==================================================================================//

void setupFRYSKY () {
    Serial.println("Initializing FrySky Pro Module");
    Serial.println("Selecting Mode...");

    if(rxMode == PPM_MODE){
        attachInterrupt(receiverPin, PPM_ISR, RISING);   // isr for measuring ppm signal from radio receiver
        Serial.println("PPM Receiver ready");

    } else if (rxMode == SBUS_MODE){
        sbusReceiver.begin(receiverPin, 5, true);
        Serial.println("SBUS Receiver ready");
    }
}

// isr for reading ppm rx signal
void IRAM_ATTR PPM_ISR(){
    static portMUX_TYPE _isr_mux = portMUX_INITIALIZER_UNLOCKED;
    static uint8_t ch_index = 0;
    static int64_t last_us;

    portENTER_CRITICAL_ISR(&_isr_mux);
    int64_t curr_us = esp_timer_get_time();
    int64_t ch_width = curr_us - last_us;
    last_us = curr_us;
    portEXIT_CRITICAL_ISR(&_isr_mux);

    if(ch_width > 3000 and ch_width < 12000) // sync
    {
        ch_index = 0;
        _ppm_data.available = false;
        if(_ppm_data.failsafe) _ppm_data.failsafe = false;
        return;
    }
    else if(ch_width > 12000)   // pulse width to long -> receiver not connected
    {   
        _ppm_data.failsafe = true;
        _ppm_data.available = false; 
        return;
    }

    if(ch_index < RX_MAX_CHANNELS)
        _ppm_data.channels[ch_index++] = constrain(ch_width, 1000, 2000);
    _ppm_data.available = true;
}

bool getPPMData(PPMData& data){
    for(byte i = 0; i < RX_MAX_CHANNELS; i++)
        data.channels[i] = _ppm_data.channels[i];
    return _ppm_data.available;
}

bool getSbusData(SBUSData& data){
    if(sbusReceiver.readCal(data.channelsCal, &data.failSafe, &data.lostFrame)){
        for(byte i = 0; i < RX_MAX_CHANNELS; i++){
            data.channels[i] = data.channelsCal[i] * 1000 / 2 + 1500;    // convert channel values to 1000 - 2000
            data.channels[i] = constrain(data.channels[i], 1000, 2000);
        }
        return 1;
    }
    return 0;
}

FRYSKY getData() {
    FRYSKY frysky;
    
    uint16_t throttle_us = 1500;
    uint16_t steering_us = 1500;
    bool data_available = false;  // Initialize to false
    bool failsafe = false;        // Initialize to false
    
    if(rxMode == PPM_MODE){
        PPMData ppm_data;
        data_available = getPPMData(ppm_data);  // Get PPM data
        failsafe = ppm_data.failsafe;

        throttle_us = ppm_data.channels[RX_THROTTLE_CH];
        steering_us = ppm_data.channels[RX_STEERING_CH];

        frysky.throttle = throttle_us;
        frysky.steeringAngle = map(steering_us, 1000, 2000, 0, 180);

    } else if(rxMode == SBUS_MODE){
        SBUSData sbus_data;
        data_available = getSbusData(sbus_data);  // Get SBUS data
        failsafe = sbus_data.failSafe;

        throttle_us = sbus_data.channels[RX_THROTTLE_CH];
        steering_us = sbus_data.channels[RX_STEERING_CH];

        frysky.throttle = throttle_us;
        frysky.steeringAngle = map(steering_us, 1000, 2000, 0, 180);
    }

    if(data_available){
        return frysky;  // Return if data is available
    } else if(failsafe) {
        frysky.throttle = 1500;
        frysky.steeringAngle = 90;  // Set failsafe values
        return frysky;
    }

    // Default return in case neither data nor failsafe triggered
    frysky.throttle = 1500;
    frysky.steeringAngle = 90;  // Default failsafe values
    return frysky;
}