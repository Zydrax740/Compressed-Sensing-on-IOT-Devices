#include <stdio.h>
#include <string.h>
#include "soc/spi_struct.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "camera_pins.h"
#include "esp_camera.h"

// Defining SPI Pins for ESP32 Cam Ai Thinker 
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 12
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

// Defining the DIRECT MEMORY ACCESS CHANNEL for SPI
#define DMA_CHAN     2

// Setting the SPI_HOST to HSPI
#define SPI_HOST    HSPI_HOST  // Use HSPI to avoid conflict with flash

// Defining size of the buffers for sending and receiving
uint8_t recvbuf[128];
uint8_t sendbuf[128];

spi_device_handle_t spi;

// Initialize Camera 
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.frame_size = FRAMESIZE_96X96;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void printSparseBufferManually(uint8_t* sparse_buffer, size_t size) {
  // Print each byte in the sparse buffer as a hexadecimal value
  for (size_t i = 0; i < size; i++) {
    // Print each byte as a hexadecimal value
    Serial.printf("%02X", sparse_buffer[i]);

    // // Optionally, add a newline after every 16 bytes for better readability
    // if ((i + 1) % 16 == 0) {
    //   Serial.println();
    // }
  }
}

// Function to print the contents of mask_buffer in binary format
void printMaskBuffer(uint8_t* mask_buffer, size_t mask_size) {
    for (size_t i = 0; i < mask_size; i++) {
        Serial.print(mask_buffer[i]);  // Extract each bit and print it
    }
    Serial.println();  // New line after printing the whole buffer
}

void generatingChaoticSequence(camera_fb_t* fb, int row, int column, float r = 1.99f, float x0 = 0.7f) {
  
  float x = x0; // initial chaotic seed

  // Global buffer for sparse data
  const int max_buffer_size = 4000;

  // Used to calculate the data size of the sparse representation
  int total_sparse_bytes = 0;
  int total_nonzero_pixels = 0;

  // Temporary buffer to hold pixel values
  uint8_t* sparse_buffer = (uint8_t*)malloc(max_buffer_size);  // Max 2 bytes per pixel
  if (!sparse_buffer) {
  Serial.println("Memory allocation for Non-Zero Values failed!");
  return;
  }
  // Temporary buffer to hold mask values
  uint8_t* mask_buffer = (uint8_t*)malloc(row * column * sizeof(uint8_t));  // 1 byte for each pixel's mask value (0 or 1)
    if (!mask_buffer) {
  Serial.println("Memory allocation for Mask failed!");
  return;
  }
  
  int sparse_index = 0;
  int mask_index = 0;

  // Generate based on 96x96 image frame
  for (int j = 0; j < column * row; j++) {

    // Generate chaotic value
    x = (x / abs(x)) - (r * x);

  // // Print the first 20 chaotic sequence values
  //   if (j < 20) {
  //     Serial.printf("Chaotic value at index %d: %f\n", j, x);
  //   }
  
    // Threshold for generating Chaotic Mask
    int mask = (x > 0.5) ? 1 : 0;
 
    // Storing Binary values of the mask
    mask_buffer[mask_index] = (uint8_t)(mask);
    mask_index++ ;

    // Applying Chaotic Mask to image data to get sparse representation
    int sparse_value = fb->buf[j] * mask;

    if (sparse_value != 0) {
      sparse_buffer[sparse_index] = (uint8_t)(sparse_value); // Sparse Value
      sparse_index++;

      // // // Debugging statement to check the sparse pixel values after chaotic masking
      // Serial.printf("Pixel %d\n", sparse_value);
    }
  }

  int row_nonzero_pixels = sparse_index / 2;
  total_nonzero_pixels += row_nonzero_pixels;
  total_sparse_bytes += sparse_index;

  // // Buffer for mask values in bytes
  // uint8_t* mask_bytes = (uint8_t*)malloc(mask_index / 8);  // One byte for every 8 bits in the mask

  // // Convert the mask to bytes
  // int byte_index = 0;
  // for (int i = 0; i < mask_index; i += 8) {
  //   uint8_t byte = 0;

  //   for (int j = 0; j < 8 && (i + j) < mask_index; j++) {
  //     byte |= (mask_buffer[i + j] << (7 - j));  // Shift each bit to the correct position in the byte
  //   }
  //   mask_bytes[byte_index++] = byte;
  // }

  // // Print the mask bytes
  // Serial.println("Mask as Bytes Data:");

  // for (int i = 0; i < byte_index; i++) {
  //   Serial.printf("%02X", mask_bytes[i]);
  // }
  // Serial.println();

  // Checking sizes of the two data
  Serial.printf("Size of Sparse Data: %d bytes\n", sparse_index);

  // Checking VALUES of the Sparse Data
  printSparseBufferManually(sparse_buffer, sparse_index);

  // Checking mask values
  printMaskBuffer(mask_buffer, mask_index);

  // Call SPI transmit function
  if (sparse_index > 0){
    sendData(sparse_buffer, sparse_index);
  }

  delay(100);

  // Empties Sparse buffer for next image
  free(sparse_buffer);
  free(mask_buffer);

}

// Main function to send each row of Sparse data via SPI
void sendData(uint8_t* sparse_buffer, size_t sparse_size) {

  const size_t chunk_size = 4; // 4 bytes at a time (2 bytes for index + 2 bytes for value)
  size_t sent = 0;

  int data_size = sparse_size;

  // --- Send size header first
  uint8_t size_header[2] = { (uint8_t)(data_size >> 8), (uint8_t)(data_size & 0xFF) };
  spi_transaction_t header = {};
  header.length = 16; // 2 bytes for the size header
  header.tx_buffer = size_header;

  digitalWrite(PIN_NUM_CS, LOW);
  delay(5);  // Let slave prepare
  spi_device_transmit(spi, &header);  // Send size header

  // --- Send sparse buffer first, then mask buffer
  while (sent < sparse_size) {
    size_t to_send = min(chunk_size, data_size - sent);
    spi_transaction_t trans = {};

    // Sending Non-Zero values first
    trans.length = to_send * 8;  // Multiply by 8 to convert from bytes to bits
    trans.tx_buffer = sparse_buffer + sent;  // Point to the current chunk of sparse data
    spi_device_transmit(spi, &trans);  // Send the chunk
    sent += to_send;  // Increment the number of sent bytes
  }

  delayMicroseconds(1000);
  Serial.println();
  Serial.printf("✅ Sent %d bytes of data via SPI\n", data_size);
  digitalWrite(PIN_NUM_CS, HIGH);  // End second transmission
}


void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize Camera function
  initCamera();

  Serial.println("Initializing ESP32 SPI Master and Camera");

  // Initialize send/recv buffers
  memset(recvbuf, 0, sizeof(recvbuf));
  memset(sendbuf, 0, sizeof(sendbuf));

  // === SPI BUS CONFIGURATION ===
  spi_bus_config_t buscfg;
  memset(&buscfg, 0, sizeof(buscfg));
  buscfg.mosi_io_num = PIN_NUM_MOSI;
  buscfg.miso_io_num = PIN_NUM_MISO;
  buscfg.sclk_io_num = PIN_NUM_CLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 32;

  // === DEVICE CONFIGURATION ===
  spi_device_interface_config_t devcfg;
  memset(&devcfg, 0, sizeof(devcfg));
  devcfg.clock_speed_hz = 50000;  // 100kHz
  devcfg.mode = 0;
  devcfg.spics_io_num = PIN_NUM_CS;
  devcfg.queue_size = 1;

  // Initialize SPI slave interface
  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, DMA_CHAN));
  ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));

  digitalWrite(PIN_NUM_CS, HIGH); // Default Setting the CS to HIGH
}

void loop() {

  // Get the total amount of heap memory available
  size_t freeHeap = ESP.getFreeHeap();
  size_t totalHeap = ESP.getHeapSize();
  size_t minFreeHeap = ESP.getMinFreeHeap();  // Minimum heap size since boot

  // ------------------------- Capture and Normalize the values of the Grayscale Image ---------------------------------
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");

    // Print memory statistics to the Serial Monitor
    Serial.printf("Total Heap Memory: %u bytes\n", totalHeap);
    Serial.printf("Free Heap Memory: %u bytes\n", freeHeap);
    Serial.printf("Minimum Free Heap Memory since boot: %u bytes\n", minFreeHeap);
    return;
  }

  // Step 1: Capture image

  // Debugging line to check image
  // Serial.printf("Image captured! Size: %d bytes\n", fb->len);
  // for (size_t i = 0; i < fb->len; i++) {
  //   Serial.printf("%02X", fb->buf[i]);
  // }

  float image_size = fb->len; // size of image

  // Calling function with size of 96x96 
  generatingChaoticSequence(fb, 96, 96);

  esp_camera_fb_return(fb);  // Empty the image buffer
  vTaskDelay(pdMS_TO_TICKS(300000));  // Wait 5 minutes before the next capture
}
