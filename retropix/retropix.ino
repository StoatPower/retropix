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

// Reads image data from the Arducam buffer and sends it over UART
// `imagebuf` -> pointer to the image data buffer in the cameras internal memory
// `length` -> the number of bytes to read in this function call
// return `send_flag` -> transmission status
uint8_t uartReadBuffer(uint8_t* imagebuf, uint8_t length) {
  // JPEG images always start with the Start of Image (SOI) marker which is `0xFF 0xD8`
  // This checks whether the first two bytes of the image buffer `imagebuf[]` match the SOI marker
  if (imagebuf[0] == SOI_1 && imagebuf[1] == SOI_2) {
    jpg_head_flag = 1;       // flags image transmission underway ?
    read_img_length = 0;    // reset byte counter for how much data sent
    // Send special header to Host Serial interface to signal that an image is to be transferred from Arduino
    // I'm not sure what the 0xFF does yet, it might just flush the buffer:
    // 0xAA -> starts transmission
    // 0x01 -> `cmdtype` that has a `payloadlength` and `payload` signifying "image length" and "image date" respectively
    uart.arducamUartWrite(UART_HEADER); 
    uart.arducamUartWrite(OUT_START_CMD); 
    uart.arducamUartWrite(OUT_CMD_IMG_TRANSFER);

    // Send image size (4 bytes, little-endian order)
    // Get first (right-most/least-significant bit) byte of picture length: bitwise AND (mask) with 0xFF i.e. 0b11111111 
    // which strips any bits from the left three bytes in order to get only the right byte
    // and then cast to uint8_t
    uart.arducamUartWrite((uint8_t)(mega.getTotalLength() & 0xFF));

    // Get second byte of picture length: bitwise shift right by one byte and then mask and cast
    uart.arducamUartWrite((uint8_t)(mega.getTotalLength() >> 8) & 0xFF);

    // Get third byte of picture length: bitwise shift right by two bytes and then mask and cast
    uart.arducamUartWrite((uint8_t)(mega.getTotalLength() >> 16) & 0xFF);

    // Get last byte of picture length: bitwise shift right by three bytes and then mask and cast
    uart.arducamUartWrite((uint8_t)(mega.getTotalLength() >> 24) & 0xFF);

    // Send image format information - encode JPG format in upper nibble (half a byte)
    // CAM_IMAGE_PIX_FMT_JPG is 0x01, so this masks 0b00000001 with 0b00001111 = 0b00000001
    // then bitwise shefts left by a nibble to get 0b00010000
    // and then bitwise OR with 0b00000001 to get 0b00010001
    // Unsure what the second nibble of 0x01 is for...
    uart.arducamUartWrite(((CAM_IMAGE_PIX_FMT_JPG & 0x0F) << 4) | 0x01);
  }
  // If we are actively sending an image
  if (jpg_head_flag == 1) {
    // udpate the total number of bytes to send
    read_img_length += length; 
    // send each byte of image data into the image buffer at the appropriate index
    for (uint8_t i = 0; i < length; i++) {
      uart.arducamUartWrite(imagebuf[i]);
    }
  }
  // Detect the end of the image and send end marker
  if (read_img_length == mega.getTotalLength()) {
    jpg_head_flag = 0; // mark completion
    // Send the STOP code of 0xFFBB (or 0x55BB?)
    // NOTE: It appears this code uses 0xFF to begin communication from the camera to the host
    // but the documentation of the communication protocol uses 0x55. Keep this in mind and test both.
    uart.arducamUartWrite(UART_HEADER);
    uart.arducamUartWrite(OUT_STOP_CMD);
  }
  return send_flag;
}

void handleUartReadStop() {
  read_img_length = 0;
  jpg_head_flag = 0;
  uint32_t streamoffLength = 9;

  // STOP code
  uart.arducamUartWrite(UART_HEADER);
  uart.arducamUartWrite(OUT_STOP_CMD);

  // START new transfer of "streamoff" command
  uart.arducamUartWrite(UART_HEADER);
  uart.arducamUartWrite(OUT_START_CMD);
  uart.arducamUartWrite(OUT_CMD_STREAM_OFF); // "streamoff" command
  uart.arducamUartWriteBuff((uint8_t*)&streamoffLength, 4); // send as 4 bytes 0x09000000 (see docs)
  uart.printf("streamoff"); // print "streamoff"

  // STOP code
  uart.arducamUartWrite(UART_HEADER);
  uart.arducamUartWrite(OUT_STOP_CMD);
}

void setup() {
  pinMode(SD_CS, OUTPUT); // Set the SD Card chip select pin to output mode

  //// Arducam Mega setup
  // Initialize serial communication, i.e. - communicate via USB to host PC
  // Note: UART is unnecessary for production mode and should be disabled  
  uart.arducamUartBegin(IMG_BAUD);
  uart.sendDataPack(7, "Hello Arduino UNO!");
  // Initialize camera configuration, defaulting to JPEG data format at maximum resolution
  mega.begin();
  uart.sendDataPack(8, "Mega started!");
  // Register callback function for streaming mode (not sure if we need this)
  // uartReadBuffer = BUFFER_CALLBACK function
  // 200 = uint8_t blockSize (transmission length?)
  // WARNING - transmission length should be less than 255
  mega.registerCallBack(uartReadBuffer, 200, handleUartReadStop);

  // initialize the SD Card
  while (!SD.begin(SD_CS)) {
    uart.sendDataPack(9, "SD Card Error!");
    // delay 1000??
  }
  uart.sendDataPack(10, "SD Card detected.");
}

void loop() {
  // Checks the number of bytes (chars) available for reading from the serial port.
  // This is data that's already arrived and stored in the serial receive buffer (which holds 64 bytes)
  if (uart.arducamUartAvailable()) {
    current_cmd_byte = uart.arducamUartRead(); // reads one byte from UART
    delay(5); // delay for 5ms to allow more bytes to arrive in UART buffer

    // 0x55 signals the start of a command to send to the camera, so if we read one    
    if (current_cmd_byte == IN_START_CMD) {
      // while there are still bytes to be read from the serial port...
      while (uart.arducamUartAvailable()) {
        // read the next byte and store it in the command buffer at the proper index (cmd_length)
        cmd_buffer[cmd_length] = uart.arducamUartRead();
        // If we read 0xAA, we need to stop break to process the command
        // There will be specific commands and parameters between 0x55 and 0xAA (see communication protocol docs)
        if (cmd_buffer[cmd_length] == IN_STOP_CMD) {
          break;
        }
        cmd_length++; // if we haven't read a 0xAA (stop) yet, increase the command buffer index and keep reading
      }
      // once we have read the stop command, we need to flush the buffer (i.e. read through any remaining junk data)
      uart.arducamFlush();
      // and then process the command by passing a ref to the camera and the contents of the command buffer
      uart.uartCommandProcessing(&mega, cmd_buffer);
      // once the command is fully processed, reset the command length to 0
      cmd_length = 0;
    }
  }
  // run the camera's stream processing thread so it can capture images/video/data
  // i.e. - this ensures the `uartReadBuffer` callback can be run when the camera is ready
  // to send data back after capturing
  mega.captureThread();

  // if we aren't currently taking a picture...
  if (is_taking_picture == false) {
    // and if our pd readings haven't reached the threshold needed to trigger a photograph...
    if (pd_threshold_counter < pd_threshold) {
      // take another reading (at defined interval) and update...
      if (millis() - pd_last_read > pd_interval_ms) {
        pd_last_read = millis();
        int pd_value = analogRead(A0); // read photodiode raw output
        
        // debugging message
        // sprintf(pd_message, "Photodiode Voltage: %d", pd_value);
        // uart.sendDataPack(11, pd_message);
        
        if (pd_value == 0) {
          // if no light is detected              
          pd_threshold_counter++; // update counter
          // sprintf(pd_message, "No Light: %d", pd_threshold_counter);
          // uart.sendDataPack(11, pd_message);
        } else if (pd_value > 0) {
          pd_threshold_counter = 0; // reset counter
        }
      }
    } else {
      // reset the counter and trip flag to take a picture
      pd_threshold_counter = 0;
      is_taking_picture = true;      
      uart.sendDataPack(12, "Start taking picture...");
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
      // sprintf(pd_message, "Photodiode Voltage: %d", pd_value);
      // uart.sendDataPack(11, pd_message);
      
      if (pd_value > 0) {
        pd_threshold_counter = 0;
        uart.sendDataPack(11, "Photograph Done!");
        is_taking_picture = false;
      } else {
        uart.sendDataPack(12, "Smile! You're on candid camera!");
      }
    }
  }  
}
