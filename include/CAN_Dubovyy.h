#include <driver/twai.h>  // Einbinden der TWAI-Treiberbibliothek für CAN-Kommunikation
#include <Wire.h>
#include <Arduino.h>

 
// Definition der CAN TX und RX Pins
#define CAN_TX_PIN GPIO_NUM_17 // esp mini 21
#define CAN_RX_PIN GPIO_NUM_16 // esp mini 20
 
 
// Rad- und Getriebeparameter
#define WHEEL_DIAMETER_CM 10.0        // Durchmesser des Rades in cm
#define GEAR_RATIO (43.0 / 10.0)      // Tatsächliches Übersetzungsverhältnis 10:43 = 4.3 : 1
#define PI 3.14159265358979323846
 
void setupCanDubovyy() {
  // Konfiguration des CAN-Treibers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Setzt die CAN-Bus-Geschwindigkeit auf 500 kbps
  twai_filter_config_t f_config = {
      .acceptance_code = 0x111 << 21,         // Akzeptiert nur Nachrichten mit ID 0x111
      .acceptance_mask = ~(0x7FF << 21),      // Filtert alle anderen IDs
      .single_filter = true
  };
 
  // Installation des CAN-Treibers
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {

    Serial.println("Fehler: CAN-Treiber!");
    while (1);
  }
 
  // Starten des CAN-Treibers
  if (twai_start() != ESP_OK) {
    Serial.println("Fehler: Start CAN!");
    while (1);
  }
}
 
void recieveCanDubovyy() {
  twai_message_t message;
 
  if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    if (message.identifier == 0x111 && message.data_length_code == 4) {
      // Verarbeitung der empfangenen Umdrehungen pro Sekunde (Motordrehzahl)
      uint32_t motor_revolutions = (message.data[0] << 24) |
                                   (message.data[1] << 16) |
                                   (message.data[2] << 8)  |
                                    message.data[3];
 
      float motor_revolutions_per_second = motor_revolutions / 100.0; // Skala zurückrechnen
      float wheel_revolutions_per_second = motor_revolutions_per_second / GEAR_RATIO; // Anpassung an das echte Übersetzungsverhältnis
      float wheel_circumference_cm = PI * WHEEL_DIAMETER_CM; // Umfang des Rades in cm
      float speed_cm_per_second = wheel_revolutions_per_second * wheel_circumference_cm;
      float speed_kmh = (speed_cm_per_second / 100000.0) * 3600.0;
 
 
      Serial.print("Motordrehzahl:");
      Serial.printf("%.2f U/s", motor_revolutions_per_second);  
      Serial.print("\t");


      
      // Raddrehzahl anzeigen
      Serial.print("Raddrehzahl:");
      Serial.printf("%.2f U/s", wheel_revolutions_per_second);
      Serial.print("\t");
 
      // Geschwindigkeit anzeigen
      Serial.print("Geschwindigkeit:");
      Serial.printf("%.2f km/h", speed_kmh);
      Serial.println("  ");

    } else {
      Serial.println("Fehler: Nachricht!");
    }
  } else {
    Serial.println("Fehler: Kein Empfang!");
  }
}