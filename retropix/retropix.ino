// Arducam MEGA Reference Links::
// Getting Started: https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/
// Arduino UNO R3 Details: https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/example/ArduinoUNOR3.html
// C++ API: https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/C%2B%2B_ApiDoc.html#c-api-reference
// Host<->Camera Communication Protocol: https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/HostCommunicationProtocol.html#from-host-to-camera
// Github Source: https://github.com/ArduCAM/Arducam_Mega
// Arducam Header File: https://github.com/ArduCAM/Arducam_Mega/blob/main/src/Arducam/ArducamCamera.h

// Adafruit SDCard Breakout Board+ Links::
// Tutorial: https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial/arduino-wiring

// PhotoDiode BPW34 Reference Links::
// Tutorial: https://deepbluembedded.com/arduino-photodiode-light-sensor-bpw34-circuit-code-example/
// 

// Arduino Reference Links::
// Serial UART port: https://docs.arduino.cc/language-reference/en/functions/communication/serial
// Software Serial UART: https://docs.arduino.cc/learn/built-in-libraries/software-serial/
// AltSoftSerial (If necessary): https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html

#include "ArducamLink.h"
#include "Arducam_Mega.h"
#include <SD.h>

// Baud Rates
#define VID_BAUD 921600
#define IMG_BAUD 115200

// Start/End of Image Codes
#define SOI_1 0xFF
#define SOI_2 0xD8
#define EOI_1 0xFF
#define EOI_2 0xD9

#define UART_HEADER 0xFF
#define OUT_START_CMD 0xAA
#define OUT_STOP_CMD 0xBB
#define OUT_CMD_IMG_TRANSFER 0x01
#define OUT_CMD_MEGA_INFO 0x02
#define OUT_CMD_STREAM_OFF 0x06
#define IN_START_CMD 0x55
#define IN_STOP_CMD 0xAA

// Buffer size for storing chunks of image data before writing to SD 
#define IMG_BUFFER_SIZE 0xFF    // 0xFF = 255 bytes

// SD Card declarations
const int SD_CS = 10;       // chip select pin for SD card
uint8_t img_count = 0;      // image file counter (i.e., 0.jpg, 1.jpg, ...)
char img_name[10] = {0};    // buffer to hold generated filenames
File out_file;              // File object for the SD card
uint8_t current_img_byte = 0;
uint8_t next_img_byte = 0;
uint8_t jpg_head_flag = 0;  // flags if image transmission is underway
unsigned int img_buffer_idx = 0;
uint8_t img_buffer[IMG_BUFFER_SIZE] = {0};

// Arducam Mega declarations
const int MEGA_CS = 7;      // chip select pin for Arducam
Arducam_Mega mega(MEGA_CS); // initialize camera using chip select pin
ArducamLink uart;

uint8_t current_cmd_byte = 0;
uint8_t cmd_buffer[20] = {0};
uint8_t cmd_length = 0;

uint8_t send_flag = TRUE; // marks completion of data transmission
uint32_t read_img_length = 0; // byte counter for image length

// Photodiode Declarations
char pd_message[50];                // debug message for photodiode readings
bool is_taking_picture = false;     // flag for determining when a picture is being taken (or not)
const int pd_threshold = 2;         // must have n consecutive readings of ambient light detection to trigger a photo
uint8_t pd_threshold_counter = 0;   // counter for threshold
const int pd_interval_ms = 500;     // interval in ms for taking readings from photodiode
unsigned long pd_last_read = 0;     // keeps track of when the last photodiode reading was taken

void setup() {
  pinMode(SD_CS, OUTPUT); // Set the SD Card chip select pin to output mode

  //// Arducam Mega setup
  Serial.begin(IMG_BAUD);
  Serial.println(F("Hello Arduino UNO!"));
  // Initialize camera configuration, defaulting to JPEG data format at maximum resolution
  mega.begin();
  Serial.println(F("Mega started!"));

  // initialize the SD Card
  while (!SD.begin(SD_CS)) {
    Serial.println(F("SD Card Error!"));
    delay(1000);
  }
  Serial.println(F("SD Card detected."));
}

void loop() {
  // if we aren't currently taking a picture...
  if (is_taking_picture == false) {
    // and if our pd readings haven't reached the threshold needed to trigger a photograph...
    if (pd_threshold_counter < pd_threshold) {
      // take another reading (at defined interval) and update...
      if (millis() - pd_last_read > pd_interval_ms) {
        pd_last_read = millis();
        int pd_value = analogRead(A0); // read photodiode raw output
        
        // debugging message
        sprintf(pd_message, "Photodiode Voltage: %d", pd_value);
        Serial.println(pd_message);
        
        if (pd_value == 0) {
          // if no light is detected              
          pd_threshold_counter++; // update counter
          sprintf(pd_message, "No Light: %d", pd_threshold_counter);
          Serial.println(pd_message);
        } else if (pd_value > 0) {
          pd_threshold_counter = 0; // reset counter
        }
      }
    } else {
      // reset the counter and trip flag to take a picture
      pd_threshold_counter = 0;
      is_taking_picture = true;
      Serial.println(F("Start taking picture..."));
    }
  } else {
    // CAM_IMAGE_MODE_QVGA is 320x240
    mega.takePicture(CAM_IMAGE_MODE_QVGA, CAM_IMAGE_PIX_FMT_JPG);
    while (mega.getReceivedLength()) {
      current_img_byte = next_img_byte;
      next_img_byte = mega.readByte();
      if (jpg_head_flag == 1) {
        img_buffer[img_buffer_idx++] = next_img_byte;
        if (img_buffer_idx >= IMG_BUFFER_SIZE) {
          out_file.write(img_buffer, img_buffer_idx);
          img_buffer_idx = 0;
        }
      }
      if (current_img_byte == SOI_1 && next_img_byte == SOI_2) {
        jpg_head_flag = 1;
        sprintf(img_name, "%d.jpg", img_count);
        img_count++;
        out_file = SD.open(img_name, FILE_WRITE|FILE_READ);
        if (!out_file) {
          Serial.println(F("File open failed."));
          while (1);
        }
        img_buffer[img_buffer_idx++] = current_img_byte;
        img_buffer[img_buffer_idx++] = next_img_byte;
      }
      if (current_img_byte == EOI_1 && next_img_byte == EOI_2) {
        jpg_head_flag = 0;
        out_file.write(img_buffer, img_buffer_idx);
        img_buffer_idx = 0;
        out_file.close();
        Serial.println(F("Image save succeeded."));
        break;
      }
    }
    if (millis() - pd_last_read > pd_interval_ms) {
      pd_last_read = millis();
      int pd_value = analogRead(A0); // read photodiode raw output
      
      // debugging message
      sprintf(pd_message, "Photodiode Voltage: %d", pd_value);
      Serial.println(pd_message);
      
      if (pd_value > 0) {
        pd_threshold_counter = 0;
        Serial.println(F("Photograph Done!"));
        is_taking_picture = false;
      } else {
        Serial.println(F("Smile! You're on candid camera!"));
      }
    }
  }  
}
