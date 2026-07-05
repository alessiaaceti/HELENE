#include <Arduino.h>
#include <ESP32CAN.h>

CAN_device_t CAN_cfg;
unsigned long last_send = 0;

void setup() {
  Serial.begin(1000000);
  delay(2000);
  Serial.println("\n--- TEST RETE CAN BUS (NO ROS) ---");

  // Utilizziamo l'Enum nativo della libreria (solitamente CAN_SPEED_200KBPS)
  CAN_cfg.speed = CAN_SPEED_200KBPS; 
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(20, sizeof(CAN_frame_t));
  
  if (ESP32Can.CANInit() == 0) {
    Serial.println("CAN Hardware Inizializzato con successo!");
  } else {
    Serial.println("ERRORE: Inizializzazione CAN fallita a livello hardware.");
  }
}

void loop() {
  // 1. Invio di un frame di test verso il Giunto Slave 2 ogni 500ms
  if (millis() - last_send > 500) {
    last_send = millis();
    
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 2; // Destinato al Giunto 2
    tx_frame.FIR.B.DLC = 8;
    memset(tx_frame.data.u8, 0xAA, 8); // Pattern di test
    
    int result = ESP32Can.CANWriteFrame(&tx_frame);
    if (result == 0) {
      Serial.println(">> Invio OK sul CAN (MsgID: 2)");
    } else {
      Serial.println(">> ERRORE INVIO CAN: Mancano i nodi Slave o i terminatori!");
    }
  }

  // 2. Controllo se qualche Giunto Slave sta rispondendo in rete
  CAN_frame_t rx_frame;
  while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 1 / portTICK_PERIOD_MS) == pdTRUE) {
    if (rx_frame.FIR.B.RTR != CAN_RTR) {
      Serial.print("<< Ricevuto pacchetto da SLAVE! MsgID: ");
      Serial.print(rx_frame.MsgID);
      Serial.print(" | Dati (primi 2 byte): ");
      Serial.print(rx_frame.data.u8[0]);
      Serial.print(" , ");
      Serial.println(rx_frame.data.u8[1]);
    }
  }
}