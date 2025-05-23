/**
* retropix.ino
*
* Main program file for the RetroPix vintage camera to digital conversion project.
*
* Description:
* This Arduino sketch controls the RetroPix retrofit system, which captures digital photographs using an Arducam Mega 5MP camera mounted inside a vintage medium format camera housing. 
* The program manages camera initialization, image capture triggered by ambient light detection through a photodiode sensor, and storage of captured images to a microSD card.
*
* Major Functions:
* - Initialize the camera, microSD card, and light detection sensor
* - Detect when the camera shutter opens via a photodiode signal
* - Capture and save an image when triggered
* - Manage image data transfer and buffering to avoid data loss
* - Log status messages for debugging and diagnostics
*
* Hardware:
* - Arduino Uno R3
* - Arducam Mega 5MP SPI Camera
* - Photodiode light sensor
* - MicroSD card module (or integrated with Arducam)
* - 9V battery power source
*
* Team 1: Matt Caldwell, Mary Jasper, Aislinn Powell
* Class: EGR 122 - Engineering Design with Professor Gill
* Institution/Semester: J. Sargeant Reynolds Community College, Spring 2025
* Final Submission: 4/29/2025
*/

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
#define DEBUG_INFO 1
#define DEBUG_WARN 2
#define DEBUG_ERR 3
#define DEBUG_OFF 4

// Buffer size for storing chunks of image data before writing to SD 
#define IMG_BUFFER_SIZE 0xFF    // 0xFF = 255 bytes

// Session ID, used in generating photo IDs since we don't
// have a good way to get the date/time w/out extra components
long session_id;

// Global debug log level threshold
uint8_t global_debug = DEBUG_INFO;

// SD Card declarations
const int SD_CS = 10;       // chip select pin for SD card
uint8_t img_count = 0;      // image file counter (i.e., 0.jpg, 1.jpg, ...)
char img_name[20] = {0};    // buffer to hold generated filenames
File out_file;              // File object for the SD card
uint8_t current_img_byte = 0;
uint8_t next_img_byte = 0;
uint8_t jpg_head_flag = 0;  // flags if image transmission is underway
unsigned int img_buffer_idx = 0;
uint8_t img_buffer[IMG_BUFFER_SIZE] = {0};

// Arducam Mega declarations
const int MEGA_CS = 7;      // chip select pin for Arducam
Arducam_Mega mega(MEGA_CS); // initialize camera using chip select pin

// Photodiode Declarations
char pd_message[50];                // debug message for photodiode readings
bool is_taking_picture = false;     // flag for determining when a picture is being taken (or not)
bool picture_taken = false;
bool picture_saved = false;
const int pd_threshold = 2;         // must have n consecutive readings of ambient light detection to trigger a photo
uint8_t pd_threshold_counter = 0;   // counter for threshold
const int pd_interval_ms = 500;     // interval in ms for taking readings from photodiode
unsigned long pd_last_read = 0;     // keeps track of when the last photodiode reading was taken

// Define function prototypes
void log(const __FlashStringHelper* message, int debug_lvl);
void log(const char* message, int debug_lvl);
void handleAmbientLightDetection();
bool shouldReadAmbientLight();
int readAndUpdateAmbientLight();
void takeAndSavePicture();
void awaitShutterClose();

void setup() {   
  Serial.begin(IMG_BAUD); // Set Serial Monitor  BAUD Rate

  pinMode(SD_CS, OUTPUT); // Set the SD Card chip select pin to output mode

  // Seed randomness for session ID (get some entropy from unused analog pin 1)
  randomSeed(analogRead(A1));
  session_id = random(100000,999999); // 6-digit session ID
  char id_msg[20];
  sprintf(id_msg, "Session ID: %ld", session_id);
  log(id_msg, DEBUG_INFO);

  // Arducam Mega setup
  log(F("Hello Arduino UNO!"), DEBUG_INFO);
  // Initialize camera configuration, defaulting to JPEG data format at maximum resolution
  CamStatus cam_status = mega.begin();
  //CamStatus cam_status = CAM_ERR_SUCCESS;
  if (cam_status != CAM_ERR_SUCCESS) {
    log(F("Arducam Mega failed to start!"), DEBUG_ERR);
  } else {
    log(F("Mega started!"), DEBUG_INFO);

    // initialize the SD Card
    while (!SD.begin(SD_CS)) {
      log(F("SD Card Error!"), DEBUG_ERR);
      delay(1000);
    }
    log(F("SD Card detected."), DEBUG_INFO);
  }  
}

 
/** loop/0
 * 
 * Main Loop
 * While not currently taking and processing a picture,
 * wait for the shutter to open and light to be detected.
 * 
 * When detected light meets our threshold, switch to
 * picture taking and saving mode until save is successful
 * (or an error occurs).
 *
 * When picture has been saved, wait for the shutter to close
 * and then switch back to standby mode waiting for next picture.
 *
 * Runs until powered off.
 *
 * return `void`
 */
void loop() {
  if (!is_taking_picture) {
    handleAmbientLightDetection();
  } else if (is_taking_picture && !picture_saved) {
    takeAndSavePicture();
  } else {
    awaitShutterClose();
  }  
}

void handleAmbientLightDetection() {  
  // if our pd readings haven't reached the threshold needed to trigger a photograph...
  if (pd_threshold_counter < pd_threshold && shouldReadAmbientLight()) {
    // take another reading (at defined interval) and update...
    int pd_value = readAndUpdateAmbientLight();      
    // TODO - change to != 0 in prod
    if (pd_value == 0) {
      // if no light is detected              
      pd_threshold_counter++; // update counter
      sprintf(pd_message, "No Light: %d", pd_threshold_counter);
      log(pd_message, DEBUG_INFO);
    } else if (pd_value > 0) {
      pd_threshold_counter = 0; // reset counter
    }
  } else if (pd_threshold_counter >= pd_threshold) {
    // reset the counter and trip flag to take a picture
    pd_threshold_counter = 0;
    is_taking_picture = true;
    log(F("Start taking picture..."), DEBUG_INFO);
  }
}

void takeAndSavePicture() {
  if (!picture_taken) {
    log(F("Smile! You're on candid camera!"), DEBUG_INFO);
    // CAM_IMAGE_MODE_QVGA is 320x240
    // CAM_IMAGE_MODE_WQXGA2 is 2592x1944    
    mega.takePicture(CAM_IMAGE_MODE_QVGA, CAM_IMAGE_PIX_FMT_JPG);
    picture_taken = true;
  } else {
    // while there is still data to receive and while picture hasn't been saved
    // continue to process data
    while (mega.getReceivedLength() && !picture_saved) {
      current_img_byte = next_img_byte;
      next_img_byte = mega.readByte();

      // save next byte of data to file
      if (jpg_head_flag == 1) {
        img_buffer[img_buffer_idx++] = next_img_byte;
        if (img_buffer_idx >= IMG_BUFFER_SIZE) {
          out_file.write(img_buffer, img_buffer_idx);
          img_buffer_idx = 0;
        }
      }

      // start of image marker detected
      // name and open new file and begin saving it
      if (current_img_byte == SOI_1 && next_img_byte == SOI_2) {
        jpg_head_flag = 1;
        sprintf(img_name, "%d-%d.jpg", session_id, img_count);
        img_count++;
        out_file = SD.open(img_name, FILE_WRITE | FILE_READ);
        if (!out_file) {
          log(F("File open failed. Resetting camera. Please ensure SD card is inserted."), DEBUG_ERR);
          picture_taken = false;
          is_taking_picture = false;
          mega.reset();
          break;
        }
        img_buffer[img_buffer_idx++] = current_img_byte;
        img_buffer[img_buffer_idx++] = next_img_byte;
      }

      // end of image marker detected
      // write out last bytes of file and clean up file handler
      // mark picture as saved, break out of loop
      if (current_img_byte == EOI_1 && next_img_byte == EOI_2) {
        jpg_head_flag = 0;
        out_file.write(img_buffer, img_buffer_idx);
        img_buffer_idx = 0;
        out_file.close();
        picture_saved = true;
        picture_taken = false;
        log(F("Image save succeeded."), DEBUG_INFO);
        break;
      }
    }
  }
}

void awaitShutterClose() {
  if (shouldReadAmbientLight()) {
    int pd_value = readAndUpdateAmbientLight();
    // TODO - update to == 0 for prod
    if (pd_value > 0) {
      log(F("Photograph Done!"), DEBUG_INFO);
      picture_saved = false;
      is_taking_picture = false;
    }
  }
}

bool shouldReadAmbientLight() {
  return millis() - pd_last_read > pd_interval_ms;
}

int readAndUpdateAmbientLight() {
  pd_last_read = millis();
  int pd_value = analogRead(A0);
  sprintf(pd_message, "Photodiode Reading: %d", pd_value);
  log(pd_message, DEBUG_INFO);
  return pd_value;
}

void log(const __FlashStringHelper* message, int debug_lvl) {
  if (debug_lvl < global_debug) {
    return;
  }
  switch (debug_lvl) {
    case DEBUG_INFO:
      Serial.println(message);
      break;

    case DEBUG_WARN:
      Serial.println(message);
      break;

    case DEBUG_ERR:
      Serial.println(message);
      break;

    default:
      break;
  }
}

void log(const char* message, int debug_lvl) {
  if (debug_lvl < global_debug) {
    return;
  }
  switch (debug_lvl) {
    case DEBUG_INFO:
      Serial.println(message);
      break;

    case DEBUG_WARN:
      Serial.println(message);
      break;

    case DEBUG_ERR:
      Serial.println(message);
      break;

    default:
      break;
  }
}
