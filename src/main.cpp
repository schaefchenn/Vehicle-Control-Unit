/* Copyright (c) 2024 AUDEx Project, Marcel Kaminski

This code is part of the AUDEx project, developed for controlling vehicles as part of the Automotive Development in 1:x
initiative. It is intended for educational and non-commercial use.

Permission is granted to use, modify, and distribute this code for educational purposes, provided that proper credit is given 
to the original developers. This code is provided "as-is" without any warranties or guarantees of functionality, safety, or fitness for any particular purpose.

For any questions or inquiries, please contact https://github.com/schaefchenn or wnw164@haw-hamburg.de */

#include <Arduino.h>
#include <CANBUS.h>
#include <XBOX.h>
#include <MANEUVER.h>
#include <FRYSKY.h>

// Core definitions (assuming you have dual-core ESP32)
static const BaseType_t pro_cpu = 0; // protocol core
static const BaseType_t app_cpu = 1; // application core

// Initialize CPU cores
TaskHandle_t Task1;
TaskHandle_t Task2;

SemaphoreHandle_t driveModeMutex;

// Set CAN ID
#define CANBUS_ID 0x15    // put your CAN ID here

// CAN send values
int8_t driveMode = 2;     // 1 = XBOX Controller; 0 = CANBUS Drive Input
int16_t throttle;
uint8_t steeringAngle;    // 90 is default
int16_t voltage;
int8_t velocity;
int8_t acknowledged;

// CAN recieve values
uint8_t canDMODE;
int16_t canTHROTTLE;
uint8_t canSTEERING;
int16_t canVOLTAGE;
int8_t canVELOCITY;
int8_t canACKNOWLEDGED;



//==================================================================================//

void CANBUS (void * pvParameters) {
  while (1){
    CANRECIEVER msg = canReceiver();

    if (msg.recieved) {
      Serial.print("recieved");
      Serial.print("\tid: 0x");
      Serial.print(msg.id, HEX);

      if (msg.extended) {
        Serial.print("\textended");
      }

      if (CAN.packetRtr()) {
        Serial.print("\trtr");
        Serial.print("\trequested length: ");
        Serial.print(msg.reqLength);

      } else {

        if (xSemaphoreTake(driveModeMutex, portMAX_DELAY) == pdTRUE) { 
          driveMode = msg.driveMode;                     // Critical section
          xSemaphoreGive(driveModeMutex);           // Give mutex after critical section
          vTaskDelay(10 / portTICK_PERIOD_MS);      // Small delay after critical section to yield
        }

        canTHROTTLE = msg.throttle;
        canSTEERING = msg.steeringAngle;

        Serial.print("\tlength: ");
        Serial.print(msg.length);
        Serial.print("\tdrive mode: ");
        Serial.print(msg.driveMode);
        Serial.print("\tthrottle: ");
        Serial.print(msg.throttle);
        Serial.print("\tsteering angle: ");
        Serial.print(msg.steeringAngle);
        Serial.print("\tvoltage: ");
        Serial.print(msg.voltage);
        Serial.print("\tvelocity: ");
        Serial.print(msg.velocity);
        Serial.print("\tacknowledged: ");
        Serial.print(msg.acknowledged);
        Serial.println();
      }
    }

    // yield
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}



void VCU (void * pvParameters){
  while(1){
    
    switch (driveMode){
      case 0: {
        // Initialize MANEUVER inside a block to avoid the jump error
        throttle = canTHROTTLE;
        steeringAngle = canSTEERING;
        MANEUVER maneuver = drive(throttle, steeringAngle);
        break;  // Exit the switch statement
      }
      /*
      case 1: {
        // Initialize XBOX inside a block to avoid the jump error
        XBOX xboxData = getXboxData();

        if (xboxData.isConnected){
          throttle = map(xboxData.rightTrigger - xboxData.leftTrigger, -1023, 1023, 1000, 2000);
          steeringAngle = map(xboxData.joyLHoriValue, 0, 65535, 0 + steeringOffset, 180 - steeringOffset);
          if(xboxData.buttonA == 1){
            canSender(CANBUS_ID, 1, throttle, steeringAngle, 1029, 40, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS); // debounce delay
          } else {
            MANEUVER maneuver = drive(throttle, steeringAngle);
            canSender(CANBUS_ID, 1, throttle, maneuver.steeringAngle, 1029, 30, 0);
          }
        }
        */

        case 2: {
          vTaskDelay(5000 / portTICK_PERIOD_MS);
          steeringAngle = 90; // steeringOffset;
          throttle = 1500;
          MANEUVER maneuver = drive(throttle, steeringAngle);
          vTaskDelay(10000 / portTICK_PERIOD_MS);
          steeringAngle = 90; // steeringOffset;
          throttle = 1600;
          maneuver = drive(throttle, steeringAngle);
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          steeringAngle = 45; // steeringOffset;
          maneuver = drive(throttle, steeringAngle + steeringOffset);
          vTaskDelay(600 / portTICK_PERIOD_MS);
          steeringAngle = 90; // steeringOffset;
          maneuver = drive(throttle, steeringAngle);
          vTaskDelay(300 / portTICK_PERIOD_MS);
          throttle = 1000;
          maneuver = drive(throttle, steeringAngle);
          vTaskDelay(300 / portTICK_PERIOD_MS);
          throttle = 1500;
          maneuver = drive(throttle, steeringAngle);
          vTaskDelay(5000 / portTICK_PERIOD_MS);
          driveMode = 0;
        }

        break;  // Exit the switch statement

      case 3: {
        FRYSKY frysky = getData();
        //Serial.printf("throttle: %d, steering: %d\n", frysky.throttle, frysky.steeringAngle);

        MANEUVER maneuver = drive(frysky.throttle, frysky.steeringAngle);
        canSender(CANBUS_ID, 2, frysky.throttle, maneuver.steeringAngle, 1680, 00, 0);

        break;  // Exit the switch statement
      }
    }

    vTaskDelay(12 / portTICK_PERIOD_MS);
  }
}



//==================================================================================//

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial);

  // initialize maneuverability
  setupMANEUVER();

  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // Setup CAN communication and ECU Components
  //setupXBOX();
  setupCANBUS();
  setupFRYSKY();


  driveModeMutex = xSemaphoreCreateMutex();

  // Start CANcommunication (priority set to 1, 0 is the lowest priority)
  xTaskCreatePinnedToCore(CANBUS,                                       // Function to be called
                          "Controller Area Network Message Recieving",  // Name of task
                          8192,                                         // Increased stack size
                          NULL,                                         // Parameter to pass to function
                          2,                                            // Increased priority
                          NULL,                                         // Task handle
                          app_cpu);

  // Start CANcommunication (priority set to 1, 0 is the lowest priority)
  xTaskCreatePinnedToCore(VCU,                                          // Function to be called
                          "Electromic Controll Unit Functionality",     // Name of task
                          8192,                                         // Increased stack size
                          NULL,                                         // Parameter to pass to function
                          2,                                            // Increased priority
                          NULL,                                         // Task handle
                          app_cpu);                                     // Assign to protocol core  
}

void loop() {
  // NO CODE HERE!
}