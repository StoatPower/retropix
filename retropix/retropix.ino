// Arducam MEGA Reference Links::
// Getting Started: https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/
// Arduino UNO R3 Details: https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/example/ArduinoUNOR3.html
// C++ API: https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/C%2B%2B_ApiDoc.html#c-api-reference
// Host<->Camera Communication Protocol: https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/HostCommunicationProtocol.html#from-host-to-camera
// Github Source: https://github.com/ArduCAM/Arducam_Mega
// Arducam Header File: https://github.com/ArduCAM/Arducam_Mega/blob/main/src/Arducam/ArducamCamera.h

// PhotoDiode BPW34 Reference Links::
// Tutorial: https://deepbluembedded.com/arduino-photodiode-light-sensor-bpw34-circuit-code-example/
// 

// Arduino Reference Links::
// Serial UART port: https://docs.arduino.cc/language-reference/en/functions/communication/serial
// Software Serial UART: https://docs.arduino.cc/learn/built-in-libraries/software-serial/
// AltSoftSerial (If necessary): https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html

#include "ArducamLink.h"
#include "Arducam_Mega.h"
// #include "SoftwareSerial.h"

// Arducam Mega declarations
const int CS = 7;
Arducam_Mega mega(CS);
ArducamLink uart;

uint8_t current_byte = 0;

uint8_t cmdBuffer[20] = {0};
uint8_t cmdLength = 0;

uint8_t sendFlag = TRUE; // marks completion of data transmission
uint32_t readImageLength = 0; // byte counter for image length
uint8_t jpegHeadFlag = 0; // flags if image transmission is underway

// Message for Photodiode Reading
char photodiodeBuffer[12];
char photodiodeMessage[50];


// Reads image data from the Arducam buffer and sends it over UART
// `imagebuf` -> pointer to the image data buffer in the cameras internal memory
// `length` -> the number of bytes to read in this function call
// return `sendFlag` -> transmission status
uint8_t readBuffer(uint8_t* imagebuf, uint8_t length) {
  // JPEG images always start with the Start of Image (SOI) marker which is `0xFF 0xD8`
  // This checks whether the first two bytes of the image buffer `imagebuf[]` match the SOI marker
  if (imagebuf[0] == 0xFF && imagebuf[1] == 0xD8) {
    jpegHeadFlag = 1;       // flags image transmission underway ?
    readImageLength = 0;    // reset byte counter for how much data sent
    // Send special header to Host Serial interface to signal that an image is to be transferred from Arduino
    // I'm not sure what the 0xFF does yet, it might just flush the buffer:
    // 0xAA -> starts transmission
    // 0x01 -> `cmdtype` that has a `payloadlength` and `payload` signifying "image length" and "image date" respectively
    uart.arducamUartWrite(0xFF); 
    uart.arducamUartWrite(0xAA); 
    uart.arducamUartWrite(0x01);

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
  if (jpegHeadFlag == 1) {
    // udpate the total number of bytes to send
    readImageLength += length; 
    // send each byte of image data into the image buffer at the appropriate index
    for (uint8_t i = 0; i < length; i++) {
      uart.arducamUartWrite(imagebuf[i]);
    }
  }
  // Detect the end of the image and send end marker
  if (readImageLength == mega.getTotalLength()) {
    jpegHeadFlag = 0; // mark completion
    // Send the STOP code of 0xFFBB (or 0x55BB?)
    // NOTE: It appears this code uses 0xFF to begin communication from the camera to the host
    // but the documentation of the communication protocol uses 0x55. Keep this in mind and test both.
    uart.arducamUartWrite(0xFF);
    uart.arducamUartWrite(0xBB);
  }
  return sendFlag;
}

void handleStop() {
  readImageLength = 0;
  jpegHeadFlag = 0;
  uint32_t streamoffLength = 9;

  // STOP code
  uart.arducamUartWrite(0xFF);
  uart.arducamUartWrite(0xBB);

  // START new transfer of "streamoff" command
  uart.arducamUartWrite(0xFF);
  uart.arducamUartWrite(0xAA);
  uart.arducamUartWrite(0x06); // "streamoff" command
  uart.arducamUartWriteBuff((uint8_t*)&streamoffLength, 4); // send as 4 bytes 0x09000000 (see docs)
  uart.printf("streamoff"); // print "streamoff"

  // STOP CODE
  uart.arducamUartWrite(0xFF);
  uart.arducamUartWrite(0xBB);
}

void setup() {
  //// Arducam Mega setup
  // Initialize serial communication, i.e. - communicate via USB to host PC
  // Note: UART is unnecessary for production mode and should be disabled  
  uart.arducamUartBegin(921600);
  uart.sendDataPack(7, "Hello Arduino UNO!");
  // Initialize camera configuration, defaulting to JPEG data format at maximum resolution
  mega.begin();
  uart.sendDataPack(8, "Mega started!");
  // Register callback function for streaming mode (not sure if we need this)
  // readBuffer = BUFFER_CALLBACK function
  // 200 = uint8_t blockSize (transmission length?)
  // WARNING - transmission length should be less than 255
  mega.registerCallBack(readBuffer, 200, handleStop);

  //// Debugging SoftwareSerial setup
  // pinMode(rxPin, INPUT);
  // pinMode(txPin, OUTPUT);
  // debugSerial.begin(9600);
  // debugSerial.println("Debug SoftwareSerial Initialized.");
}

void loop() {
  // Checks the number of bytes (chars) available for reading from the serial port.
  // This is data that's already arrived and stored in the serial receive buffer (which holds 64 bytes)
  if (uart.arducamUartAvailable()) {
    current_byte = uart.arducamUartRead(); // reads one byte from UART
    delay(5); // delay for 5ms to allow more bytes to arrive in UART buffer

    // 0x55 signals the start of a command to send to the camera, so if we read one    
    if (current_byte == 0x55) {
      // while there are still bytes to be read from the serial port...
      while (uart.arducamUartAvailable()) {
        // read the next byte and store it in the command buffer at the proper index (cmdLength)
        cmdBuffer[cmdLength] = uart.arducamUartRead();
        // If we read 0xAA, we need to stop break to process the command
        // There will be specific commands and parameters between 0x55 and 0xAA (see communication protocol docs)
        if (cmdBuffer[cmdLength] == 0xAA) {
          break;
        }
        cmdLength++; // if we haven't read a 0xAA (stop) yet, increase the command buffer index and keep reading
      }
      // once we have read the stop command, we need to flush the buffer (i.e. read through any remaining junk data)
      uart.arducamFlush();
      // and then process the command by passing a ref to the camera and the contents of the command buffer
      uart.uartCommandProcessing(&mega, cmdBuffer);
      // once the command is fully processed, reset the command length to 0
      cmdLength = 0;
    }
  }
  // run the camera's stream processing thread so it can capture images/video/data
  // i.e. - this ensures the `readBuffer` callback can be run when the camera is ready
  // to send data back after capturing
  mega.captureThread();

  // Photodiode Debugging (Every 500ms)
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 500) {
    lastRead = millis();

    int sensorValue = analogRead(A0); // read photodiode
    // float voltage = sensorValue * (5.0 / 1023.0);
    // dtostrf(voltage, 8, 4, photodiodeBuffer);
    // sprintf(photodiodeMessage, "Photodiode Voltage: %s", photodiodeBuffer);
    sprintf(photodiodeMessage, "Photodiode Voltage: %d", sensorValue*10);
    uart.sendDataPack(9, photodiodeMessage);
  }
}
