#include <XboxSeriesXControllerESP32_asukiaaa.hpp>  // Xbox controller library for ESP32

struct XBOX {
    bool isConnected;
    float joyLHoriValue;
    float rightTrigger;
    float leftTrigger;
    bool buttonA;
    bool buttonB;
    bool buttonX;
    bool buttonY;
    bool buttonLB;
    bool buttonRB;
    bool buttonStart;
    bool buttonSelect;
    bool buttonLStick;
    bool buttonRStick;
    bool buttonCrossUP;
    bool buttonCrossDOWN;
    bool buttonCrossLEFT;
    bool buttonCrossRIGHT;
};

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;   // Xbox controller object

const int ledPin = 2;   // Pin for built-in LED
int flag = 0;           // handling for the first connection 


//==================================================================================//

void setupXBOX () {
    // Initialize serial communication
    Serial.println("\nInitializing Vehicle...");
    Serial.println("Starting NimBLE Client...");
    
    // Setting up XBOX controller connection
    xboxController.begin();

    // Set up the built-in LED
    pinMode(ledPin, OUTPUT);

    Serial.println("Ready for Bluetooth Connection!");
}


//==================================================================================//

// Function to demonstrate vibration on the XBOX controller
void demoVibration() {
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
    repo.v.select.center = true;   // Enable center vibration
    repo.v.select.left = false;    // Disable left vibration
    repo.v.select.right = false;   // Disable right vibration
    repo.v.select.shake = false;   // Disable shake
    repo.v.power.center = 80;      // Set center vibration power to 80%
    repo.v.timeActive = 35;        // Set active time to 0.35 seconds
    Serial.println("\nController Connected Successfully");
    xboxController.writeHIDReport(repo); // Send vibration report to controller
}


XBOX getXboxData() {
    xboxController.onLoop(); // Process controller loop

    XBOX data; // Create an instance of the struct to hold the return values

    if (xboxController.isConnected()) { // Check if controller is connected
        if (xboxController.isWaitingForFirstNotification()) {
            Serial.println("waiting for first notification");

            return data;
        } else {
            if (flag == 0) {
                demoVibration(); // Demonstrate vibration on first connection
                digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED
                Serial.println("Address: " + xboxController.buildDeviceAddressStr()); // Print controller address
                Serial.println("battery " + String(xboxController.battery) + "%"); // Print battery status
                flag += 1;

                data.isConnected = true;
                //sendCanData(driverReady);
            }

            // Read trigger and joaystick values
            data.joyLHoriValue = (float)xboxController.xboxNotif.joyLHori;
            data.rightTrigger = (float)xboxController.xboxNotif.trigRT;
            data.leftTrigger = (float)xboxController.xboxNotif.trigLT;

            // Read button states
            data.buttonA = xboxController.xboxNotif.btnA;
            data.buttonB = xboxController.xboxNotif.btnB;
            data.buttonX = xboxController.xboxNotif.btnX;
            data.buttonY = xboxController.xboxNotif.btnY;
            data.buttonLB = xboxController.xboxNotif.btnLB;
            data.buttonRB = xboxController.xboxNotif.btnRB;
            data.buttonStart = xboxController.xboxNotif.btnStart;
            data.buttonSelect = xboxController.xboxNotif.btnSelect;
            data.buttonLStick = xboxController.xboxNotif.btnLS;
            data.buttonRStick = xboxController.xboxNotif.btnRS;
            data.buttonCrossUP = xboxController.xboxNotif.btnRS;
            data.buttonCrossDOWN = xboxController.xboxNotif.btnRS;
            data.buttonCrossLEFT = xboxController.xboxNotif.btnRS;
            data.buttonCrossRIGHT = xboxController.xboxNotif.btnRS;

            data.isConnected = true;

            return data;
        }
    } else {
        data.isConnected = false;

        // Restart ESP if connection failed multiple times
        if (xboxController.getCountFailedConnection() > 2) {
            ESP.restart();
        }

        // Handle disconnection
        if (flag >= 1){
            digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED
            Serial.println("not connected");
            flag = 0;
        }

        return data;
    }
}
