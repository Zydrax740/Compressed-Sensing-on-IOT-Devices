#include "LoRaWan_APP.h"
#include "Arduino.h"

#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define RF_FREQUENCY              915000000 // Match this to AB01
#define TX_OUTPUT_POWER           20
#define LORA_BANDWIDTH            0     // 125 kHz
#define LORA_SPREADING_FACTOR     8
#define LORA_CODINGRATE           1         // 4/5
#define LORA_PREAMBLE_LENGTH      8
#define LORA_SYMBOL_TIMEOUT       0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON      false

#define MAX_IMAGE_SIZE            4000  // Adjust as needed
#define MAX_LORA_PAYLOAD          17        // 1 byte for seq, 16 bytes for data (payload)

static RadioEvents_t RadioEvents;

uint8_t image_buffer[MAX_IMAGE_SIZE];
bool received_chunks[128];  // Track which chunks have been received

uint16_t image_offset = 0;
uint8_t expected_total_chunks = 0;
bool image_complete = false;

// Image Buffer logic
uint8_t full_image_buffer[MAX_IMAGE_SIZE];
uint16_t full_image_offset = 0;
uint16_t chunk_counter = 0;
const uint16_t max_chunks = MAX_IMAGE_SIZE / (MAX_LORA_PAYLOAD - 1);  // assuming 15 bytes per chunk

void setup() {
  Serial.begin(115200);
  // Serial.println("================ AB02 LoRa Image Receiver ================");

  RadioEvents.RxDone = onRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
    LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  // Start receiving
  Radio.Rx(0);
}

void loop() {
  Radio.IrqProcess();  // Required for event callbacks on CubeCell

  if (image_complete) {
    Serial.println("All image chunks received!");
    for (int i = 0; i < full_image_offset; i++) {
      Serial.printf("%02X", full_image_buffer[i]);  // Print entire image as hex
    }
    Serial.println();

    image_complete = false;  // Reset for the next image
    memset(received_chunks, 0, sizeof(received_chunks));
    full_image_offset = 0;
    chunk_counter = 0;
  }
}

void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  // Serial.println("Waiting for data from AB01....");

  for (int i = 0; i < size; i++) {
    Serial.printf("%02X ", payload[i]);
  }
  Serial.println();

  storeImageChunk(payload, size);

  // Check if all chunks have been received or if max chunks have been reached
  if (chunk_counter >= max_chunks || full_image_offset + (size - 1) >= MAX_IMAGE_SIZE) {
    image_complete = true;
    Serial.println("end");
  }

  Radio.Rx(0);  // Listen for more data
}

void storeImageChunk(uint8_t* payload, uint16_t size) {
  if (full_image_offset + (size - 1) <= MAX_IMAGE_SIZE) {
    // Skip the sequence byte (first byte of each chunk) and copy the rest to the buffer
    memcpy(&full_image_buffer[full_image_offset], payload + 1, size - 1);
    full_image_offset += size - 1;
    chunk_counter++;  // Increment the chunk counter

    // // Debug: Print how much data has been received
    // Serial.printf("Received chunk #%d, total received: %d bytes\n", chunk_counter, full_image_offset);
  }
  else {
    // If we receive more data than the max size, we can ignore the extra data
    Serial.println("Warning: Data overflow, ignoring extra data.");
  }
}
