// This code is for the HTCC-AB01 as SPI Slave

// // Include SPI function File
#include <SPI.h>

// Include main LoRa function File 
#include "LoRaWan_APP.h"
#include "Arduino.h"

// Defining the SPI Pins for the HTCC-AB01 SPI Bus

// #define PIN_SCK P4_2 // Connected to ESP32-Cam Pin 14 SCLK
// #define PIN_MOSI P4_0 // Connected to ESP32-Cam Pin 12 MOSI
// #define PIN_MISO P4_1 // Connected to ESP32-Cam Pin 13 MISO
// #define PIN_CS P6_1 // Connected to ESP32-Cam Pin 15 (DO NOT CHANGE)

#define PIN_SCK P4_2
#define PIN_MOSI P4_0
#define PIN_MISO P4_1
#define PIN_CS P6_1

// Defining the main pins for Lora Communication

#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

// Default confiurations for the AB01 LoRa
#define RF_FREQUENCY              915000000 // Match this to AB02
#define TX_OUTPUT_POWER           20
#define LORA_BANDWIDTH            0 // 125kHz
#define LORA_SPREADING_FACTOR     8
#define LORA_CODINGRATE           1
#define LORA_PREAMBLE_LENGTH      8
#define LORA_SYMBOL_TIMEOUT       0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON      false

#define MAX_LORA_PAYLOAD 17 // Size of bytes set via LoRa
// #define MAX_IMAGE_SIZE 4000  // Max Buffer Size for image in AB01

uint8_t chunk[MAX_LORA_PAYLOAD];  // Reserve 1 byte for sequence number

// Defining the size of the each chunk
#define CHUNK_SIZE 32

// Reduce the transmission speed of the SPI clock just incase
static const int spiClk = 50000; // 500KHz

// Functions for receiving image data from ESP32
uint8_t receiveByte();
uint16_t receiveImageSize();

static RadioEvents_t RadioEvents;

// uint8_t image_buffer[MAX_IMAGE_SIZE];

// Declaring global variables for Sparse and Mask buffers
uint8_t Sparse_buffer[2244];
uint8_t Mask_buffer[1152];


static uint16_t total_received = 0;
static uint16_t image_size = 0;

enum State {
  WAIT_FOR_IMAGE,
  READY_TO_SEND,
  SENDING_CHUNKS
};

State currentState = WAIT_FOR_IMAGE;

void receiveImageViaSPI();
void prepareSendingChunks();
void sendImageVIALoRa();
void onTxDone();

uint16_t chunkIndex = 0;     // Position in image_buffer for current LoRa send
uint8_t seq = 0;             // Sequence number of chunk

volatile bool canSend = false;        // Track when LoRa TX is done
volatile bool txDoneFlag = false;     // Checking if the TX flag is set to TRUE or FALSE

void onTxDone() {
  Serial.printf("✅ TX Done triggered for chunk #%d\n", seq - 1);
  canSend = true;
  Radio.Sleep();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  RadioEvents.TxDone = onTxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
    LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_CS, INPUT);
  pinMode(PIN_MISO, OUTPUT);

  Serial.println("======= HTCC-AB01 SPI Slave receiving image data =======");
  Serial.println("================ LoRa is Initialized! =================");
  Serial.println("===============  Start of Image Data ===================");
  digitalWrite(PIN_MISO, LOW);

}

void loop() {

  if (digitalRead(PIN_CS) == LOW) {
    receiveImageViaSPI();
  }

  if (currentState == READY_TO_SEND){
    prepareSendingChunks();
    currentState = SENDING_CHUNKS;
  }

  if (currentState == SENDING_CHUNKS ){
    sendImageVIALoRa();
  }  
}

void receiveImageViaSPI(){
  
  total_received = 0;

  // Step 1: Receive size header
  image_size = (receiveByte() << 8) | receiveByte();
  if (image_size > 4000) return;

  // Step 2: Receive Non-Zero values first
  while (total_received < 2244  && digitalRead(PIN_CS) == LOW) {
    Sparse_buffer[total_received++] = receiveByte();
  }

  // Printing the Non-Zero values
  for (int i = 0; i < total_received; i++) {
    Serial.printf("%02X", Sparse_buffer[i]);
  }

  Serial.println();
  Serial.printf("Received %d bytes from ESP32-Cam (Expected: 2244)\n", total_received);

  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_MISO, INPUT);
      
  currentState = READY_TO_SEND;
}

void prepareSendingChunks(){
  chunkIndex = 0;    // Reset chunk position to beginning of image_buffer
  seq = 0;           // Reset sequence number for LoRa packets
  canSend = true;    // Set to true so first chunk can be sent immediately
}

void sendImageVIALoRa() {
  // Serial.println(image_size);

  for (uint16_t i = 0; i < image_size; i += (MAX_LORA_PAYLOAD - 1)) { 
    uint8_t chunk_len = min((MAX_LORA_PAYLOAD - 1), image_size - i);

    // Serial.println();
    // Serial.println(i);
    // Serial.println();

    // Send Sparse value first 
    chunk[0] = seq++;  // Sequence number
    memcpy(&chunk[1], &Sparse_buffer[i], chunk_len);

    canSend = false;
    txDoneFlag = false;

    Radio.Sleep();  // Forced Reset on the Radio
    Radio.Send(chunk, chunk_len + 1);
    Serial.println("✅ Radio.Send() called");

    Serial.printf("📤 Sent chunk #%d (%d bytes)\n", chunk[0], chunk_len + 1);

    Serial.println();
    for (uint8_t j = 0; j < chunk_len + 1; j++) {
    Serial.printf("%02X ", chunk[j]);
    }
    Serial.println();  // Newline after printing all bytes

    // Radio.IrqProcess();
    uint32_t start = millis();
    while (!canSend && millis() - start < 5000) {
      Radio.IrqProcess();  // Call IrqProcess after the sending each wave of chunks
      delay(1);
    }

    Serial.println("Radio.Irqprocess() is called.");
    Serial.println("Wave is completed.");

    delay(100);
  }

  chunk[0] = 0x8D;
  chunk[1] = 0xFF;      // End marker sequence
  chunk[2] = 0x00;
        // End payload

  canSend = false;
  Radio.Sleep();
  Radio.Send(chunk, 3);

  Serial.println("✅ End marker Radio.Send() called");
  Serial.println("📤 Sent END marker: FF 00");

  uint32_t start = millis();
  while (!canSend && millis() - start < 5000) {
    Radio.IrqProcess();
    delay(1);
  }

  Serial.println("✅ End marker sent successfully.");
  
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_CS, INPUT);
  pinMode(PIN_MISO, OUTPUT);
}

// Main function receive the image data in bits from the ESP32 Cam
uint8_t receiveByte() {
  uint8_t value = 0;
  for (int bit = 7; bit >= 0; bit--) {
    while (digitalRead(PIN_SCK) == LOW);
    delayMicroseconds(2);
    if (digitalRead(PIN_MOSI)) {
      value |= (1 << bit);
    }
    digitalWrite(PIN_MISO, 0);
    while (digitalRead(PIN_SCK) == HIGH);
  }
  return value;
}

uint16_t receiveImageSize() {
  uint8_t high = receiveByte();
  uint8_t low = receiveByte();
  return ((uint16_t)high << 8) | low;
}
