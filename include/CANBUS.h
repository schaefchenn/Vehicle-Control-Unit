#include <CAN.h>

#define TX_GPIO_NUM   17  // Connects to CTX
#define RX_GPIO_NUM   16  // Connects to CRX

struct CANRECIEVER {
  bool recieved;
  bool extended;
  bool rtr;
  int reqLength;
  int length;
  int id;
  int8_t driveMode;
  int16_t throttle;
  uint8_t steeringAngle;
  int16_t voltage;
  int8_t velocity;
  int8_t acknowledged;
};


//==================================================================================//

void setupCANBUS() {
  Serial.println ("CAN Receiver/Receiver");

  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 1 Mbps
  if (!CAN.begin (1E6)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }
}


//==================================================================================//

void canSender(int CANBUS_ID, int8_t driveMode, int16_t throttle, uint8_t steeringAngle, int16_t voltage, int8_t velocity, int8_t acknowledged) {
  //Serial.print("Sending packet ... ");

  CAN.beginPacket(CANBUS_ID);  // Sets the ID and clears the transmit buffer
  

  CAN.write(driveMode); // Write driveMode as 1 byte
  

  // Break throttle value into two bytes and write them
  CAN.write((uint8_t)(throttle >> 8)); // High byte
  CAN.write((uint8_t)(throttle & 0xFF)); // Low byte
  

  CAN.write(steeringAngle); // Write steering angle as 1 byte
  

  // Break voltage value into two bytes and write them
  CAN.write((uint8_t)(voltage >> 8)); // High byte
  CAN.write((uint8_t)(voltage & 0xFF)); // Low byte

  CAN.write(velocity); // Write 1 byte
  CAN.write(acknowledged); // Write 1 byte


  CAN.endPacket();

  //Serial.println("done");
}

CANRECIEVER canReceiver() {
  CANRECIEVER msg;

  msg.recieved = false;
  msg.extended = false;
  msg.rtr = false;

  int packetSize = CAN.parsePacket();

  if (packetSize) {
    msg.recieved = true;

    if (CAN.packetExtended()) {
      msg.extended = true;
    }

    if (CAN.packetRtr()) {
      msg.rtr = true;
    }

    msg.id = CAN.packetId();

    if (CAN.packetRtr()) {
      msg.reqLength = CAN.packetDlc();

    } else {
      msg.length = packetSize;

      // Read and print integer values
      if (packetSize >= 4) { // Ensure we have at least 4 bytes
        int8_t driveMode = CAN.read(); // Read 8-bit signed integer

        // Read the next two bytes and combine them into a int16_t
        uint8_t highByte = CAN.read();
        uint8_t lowByte = CAN.read();
        int16_t throttle = (highByte << 8) | lowByte; // Combine bytes
        // Interpret as signed integer
        if (throttle & 0x8000) { // Check if the sign bit is set
          throttle |= 0xFFFF0000; // Sign-extend to 32-bit
        }

        uint8_t steeringAngle = CAN.read(); // Read 8-bit signed integer

        // Read the next two bytes and combine them into a int16_t
        highByte = CAN.read();
        lowByte = CAN.read();
        int16_t voltage = (highByte << 8) | lowByte; // Combine bytes
        // Interpret as signed integer
        if (voltage & 0x8000) { // Check if the sign bit is set
          voltage |= 0xFFFF0000; // Sign-extend to 32-bit
        }

        int8_t velocity = CAN.read(); // Read 8-bit signed integer
        int8_t acknowledged = CAN.read(); // Read 8-bit signed integer

        msg.driveMode = driveMode;
        msg.throttle = throttle;
        msg.steeringAngle = steeringAngle;
        msg.voltage = voltage/100;
        msg.velocity = velocity;
        msg.acknowledged = acknowledged;
      }
    }
  }

  return msg;
}