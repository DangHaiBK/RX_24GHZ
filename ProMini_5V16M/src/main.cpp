#include <Arduino.h>

#include "Servo.h"
#include "RF24.h"

#include "config/config.h"
#include "config/radioLink.h"

/* Radio class instance */
RF24 radio(CE_PIN, CSN_PIN);

/* Enable send or not variables */
bool send_enable;

/* Timestamp variables */
unsigned long last_receive;
unsigned long toggle_period;
unsigned long last_send;

/* radiolink protocol variables */
radiolink_protocol_t radio_link_send;
radiolink_protocol_t radio_link_response;
radiolink_protocol_t curData;

/* Servo instance for PWM output */
Servo throttle_output;
Servo yaw_output;
Servo pitch_output;
Servo roll_output;
Servo AUX1_output;
Servo AUX2_output;

/* Function declaration */
bool check_payload_receive(radiolink_protocol_t payload);

void setup() {

#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  packet.pitch = 10;
  packet.roll = 12;
  packet.yaw = 16;
#endif  /* SERIAL_DEBUG */

  pinMode(LED_STATUS_RX, OUTPUT);

  /* Setup radio NRF24 */
  if (!radio.begin()) {
#ifdef SERIAL_DEBUG
    Serial.println("Radio module is not connected");
#endif  /* SERIAL_DEBUG */
    digitalWrite(LED_STATUS_RX, LOW);
    for (;;) {
#ifdef SERIAL_DEBUG
      Serial.println("Waiting for connection ...");
#endif  /* SERIAL_DEBUG */
      if (radio.begin()) {
        break;
      }
      delay(1000);
    }
  }

#ifdef SERIAL_DEBUG
  Serial.println("Radio module is connected");
#endif  /* SERIAL_DEBUG */

  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(CHANNEL_NRF24);
  radio.setCRCLength(RF24_CRC_16);

  /* Delay for stabilization */
  delay(1000);

  /* Indicate led status */
  digitalWrite(LED_STATUS_RX, HIGH);

  //radio.openWritingPipe(TX_ADDR);
  radio.openReadingPipe(2, TX_ADDR);
  radio.startListening();

  /* Assign timestamp */
  toggle_period = millis();
  last_receive = millis();
  last_send = millis();

}

void loop() {
  if (radio.available()) {
    while(radio.available()) {
      toggle_period = millis();
      radio.read(&radio_link_response, sizeof(radio_link_response));
      if (check_payload_receive(radio_link_response)) {
        last_receive = millis();
        curData.payload[0] = radio_link_response.payload[0];
        curData.payload[1] = radio_link_response.payload[1];
        curData.payload[2] = radio_link_response.payload[2];
        curData.payload[3] = radio_link_response.payload[3];
        curData.payload[4] = radio_link_response.payload[4];
        curData.payload[5] = radio_link_response.payload[5];
      }
    }
  }

  throttle_output.writeMicroseconds(curData.payload[0]);
  throttle_output.writeMicroseconds(curData.payload[1]);
  throttle_output.writeMicroseconds(curData.payload[2]);
  throttle_output.writeMicroseconds(curData.payload[3]);
  throttle_output.writeMicroseconds(curData.payload[4]);
  throttle_output.writeMicroseconds(curData.payload[5]);

  if (millis() - last_send >= SENDING_STATUS_PERIOD) {
    last_send = millis();
    if (send_enable) {
      radio.stopListening();
      radio.openWritingPipe(TX_ADDR);

      radio_link_send.startByte = RADIOLINK_START_BYTE;
      radio_link_send.endByte = RADIOLINK_END_BYTE;
      radio_link_send.infoByte = RADIOLINK_STATUS_PACKET_OK;
      radio.write(&radio_link_send, sizeof(radio_link_send));

      radio.openReadingPipe(2, TX_ADDR);
      radio.startListening();
    }
  }

  if (millis() - last_receive >= SIGNAL_LOST_TIMEOUT) {
    send_enable = false;
    if (millis() - toggle_period >= LOSS_TOGGLE_PERIOD) {
      toggle_period = millis();
      digitalWrite(LED_STATUS_RX, !digitalRead(LED_STATUS_RX));
    }
  }
  else {
    send_enable = true;
    digitalWrite(LED_STATUS_RX, HIGH);
  }
}

bool check_payload_receive(radiolink_protocol_t payload) {
  if ((payload.startByte == RADIOLINK_START_BYTE) && (payload.endByte == RADIOLINK_END_BYTE) && (payload.infoByte == RADIOLINK_INFO_STICK_POS)) {
  return true;
  }
  else {
    return false;
  }
}