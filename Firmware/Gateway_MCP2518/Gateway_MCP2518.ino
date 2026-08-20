/*******************************************************************************
 * @file        Gateway_MCP2518.ino
 * @brief       Prototype Logging Firmware for CAN Gateway 
 * @author      Nick Poole
 * @date        August 19, 2026
 * 
 * This is the first prototype release firmware for CAN Gateway hardware v02
 * It receives CAN traffic on 6 channels simultaneously (at 250kbps) and logs
 * all frames in human-readable format on an inserted SD card. The debug menu
 * allows a user to set the time in unix timestamp format, which is kept using
 * the on-board RTC and used to timestamp the log files. CAN traffic can also
 * be streamed to the Serial terminal. This firmware will drop CAN frames if
 * all 6 busses are fully saturated. 
 * 
 * Copyright (c) 2026 Rising Tide Research Foundation
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the “Software”), to deal 
 * in the Software without restriction, including without limitation the rights 
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
 * of the Software, and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all 
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************/

// Arduino IDE Board Settings
// https://github.com/espressif/arduino-esp32
// FQBN: esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=enabled,LoopCore=0
// Board: "ESP32S3 Dev Module"
// USB CDC On Boot: "Enabled"
// CPU Frequency: "240MHz (WiFi)"
// Core Debug Level: None
// USB DFU On Boot: "Disabled"
// Erase All Flash Before Sketch Upload: "Disabled"
// Events Run On: "Core 1"
// Flash Mode: "QIO 80MHz"
// Flash Size: "16MB (128Mb)"
// JTAG Adapter: "Disabled"
// Arduino Runs On: "Core 0"
// USB Firmware MSC On Boot: "Disabled"
// Partition Scheme: "Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)"
// PSRAM: "QSPI PSRAM"
// Upload Mode: "UART 0 / Hardware CDC"
// Upload Speed: "921600"
// USB Mode: "Hardware CDC and JTAG"

#include <SparkFun_I2C_Expander_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_I2C_Expander_Arduino_Library
#include <RV3028C7.h> // https://github.com/MacroYau/RV-3028-C7-Arduino-Library
#include <atomic> 
#include <SPI.h> 
#include <esp_task_wdt.h> 
#include <freertos/queue.h> 
#include "mcp2518fd_can.h" // Local Fork
#include "FS.h" 
#include "SD_MMC.h" 
#include "time.h" 

// Storage related parameters
#define SIZE_CAN_RX_QUEUE_IN_FRAMES     1024
#define SIZE_SDMMC_BUFFER_IN_BYTES      65536
#define SIZE_SDMMC_CHUNK_WRITE_IN_BYTES 32767
#define SIZE_MAXIMUM_LOGFILE_IN_MBYTES  200

// Error printing helpers
#define ERR_CAN_FAILED_TO_READ_BUFFER_STATUS  0x01
#define ERR_SD_FAILED_TO_OPEN_FILE            0x02
#define ERR_SD_FAILED_TO_WRITE_FILE           0x04
#define ERR_SD_FAILED_TO_DELETE_FILE          0x08
#define ERR_LOG_BUFFER_WRAP                   0x10
#define ERR_CAN_FAILED_TO_READ_REGISTER       0x20
#define ERR_CAN_RX_PASSIVE                    0x40
#define ERR_CAN_TX_PASSIVE                    0x80
#define ERR_CAN_BUS_OFF                       0x100
#define VERBOSE_ERR true
uint16_t errorRegister = 0;
uint8_t busErrorFlags[6];

#define CAN_ERR_CHECK_INTERVAL_SECONDS 30 // Check CAN controllers for bus errors at most this often
time_t lastCheckCANErr = 0;               // Keep track of the last time CAN controllers were polled for errors
bool checkCANErr = 0;                     // Put the burden of time_t comparisons on core 0 task and use flag to alert core 1 task
bool freshCANErr = 0;                     // Flag to alert loggingTask when there is an error

// Log about queue overflow at most this often
#define CAN_QUEUE_OVERFLOW_LOG_INTERVAL_SECONDS 10 

// Flush a partial buffer to SD if the bus is quiet for this many milliseconds
#define TIMEOUT_SD_FLUSH_MS 50 

// GPIO Expander
SFE_PCA95XX io(PCA95XX_PCA9534);

// Real-Time Clock
RV3028C7 rtc;

// Reuseable timeval global
struct timeval tv;

// Task handles for pinning to separate cores
TaskHandle_t canTask;
TaskHandle_t loggingTask;

// Struct for the CAN frame buffer
struct CANFrame
{
  uint8_t channel;
  time_t timeSec;
  suseconds_t timeuSec;
  uint8_t data[8];
  uint32_t id;
  uint8_t dlc;
  bool rtr;
  bool ext;
};

// Queue for incoming CAN traffic to write CAN traffic to (queue of struct CANFrame)
QueueHandle_t canRxQueue;

uint32_t canRxOverflowCount;                 // Number of times canRxQueue has overflowed since boot
std::atomic<uint32_t> canRxOverflowInterval; // Number of times canRxQueue has overflowed since last log line
time_t canRxOverflowLastLog;                 // Timestamp of the last time we logged overflows

// Buffer for writing to storage
char bufferSD[SIZE_SDMMC_BUFFER_IN_BYTES];
uint16_t SDpos = 0;

// Calculate the number of chunk writes to approach maximum logfile size
const uint32_t maxLogSizeChunks = (SIZE_MAXIMUM_LOGFILE_IN_MBYTES * 1000000) / SIZE_SDMMC_CHUNK_WRITE_IN_BYTES;
// Track the number of chunks written to open logfile
uint32_t chunkWrites = 0;

// Map GPIO Expander pins to CAN EN pins
uint8_t canEnable[] = {4, 5, 3, 2, 1, 0};

// CAN Transceiver objects
mcp2518fd *CAN0 = new mcp2518fd(39);
mcp2518fd *CAN1 = new mcp2518fd(40);
mcp2518fd *CAN2 = new mcp2518fd(48);
mcp2518fd *CAN3 = new mcp2518fd(47);
mcp2518fd *CAN4 = new mcp2518fd(8);
mcp2518fd *CAN5 = new mcp2518fd(18);

// Put the CAN objects in an iterable form
mcp2518fd *canChannel[] = {CAN0, CAN1, CAN2, CAN3, CAN4, CAN5};

// Instantiate SPI Classes
SPIClass *spi0 = new SPIClass(FSPI);
SPIClass *spi1 = new SPIClass(HSPI);

// Lazy flag to break from serial menu loop
bool exitMenu = false;
bool debugMenuActive = false;

// Flag to change the destination of CAN logs
typedef enum
{
  CAN_TO_SDMMC,
  CAN_TO_SERIAL
} CAN_TRAFFIC_FLOW;
CAN_TRAFFIC_FLOW canFlow = CAN_TO_SDMMC;

// Number of times to attempt intialization of each CAN transceiver on failure
uint8_t mcpInitRetry = 5;

// Name of the open logfile
char openLogfile[20];

// Add an error code to the register for printing in VERBOSE mode
void ERR(uint16_t errorCode)
{
  errorRegister = errorRegister | errorCode;
  return;
}

/************************************************************************** 
 * Function:    setTimeFromRTC
 * Purpose:     Set the ESP32 system time from the RTC
 * Parameters:  None
 * Returns:     bool  - true if successful 
**************************************************************************/
bool setTimeFromRTC()
{
  uint32_t rtcTime = rtc.getUnixTimestamp();
  struct timeval systime;
  int rc;
  systime.tv_sec = rtcTime;
  systime.tv_usec = 0;
  rc = settimeofday(&systime, NULL);
  if (rc == 0)
  {
    if (VERBOSE_ERR)
    {
      Serial.println("System time set successfully");
    }
    return 1;
  }
  else
  {
    if (VERBOSE_ERR)
    {
      Serial.println("System time set failed");
    }
    return 0;
  }
  return 0;
}

/************************************************************************** 
 * Function:    checkCANErrors
 * Purpose:     Call the MCP2518 Library's checkError() function for 
 *              channel and store in busErrorFlags[] for channel
 * Parameters:  uint8_t   - CAN channel number
 * Returns:     void 
**************************************************************************/
void checkCANErrors(uint8_t channel)
{
  byte errorFlags;
  canChannel[channel]->checkError(&errorFlags);
  busErrorFlags[channel] = errorFlags;
  return;
}

/************************************************************************** 
 * Function:    pollCANForMsg
 * Purpose:     Poll CAN controller @ channel for a new message.
 * Parameters:  uint8_t   - CAN channel number
 * Returns:     bool      - true if successful
**************************************************************************/
bool pollCANForMsg(uint8_t channel)
{
  CANFrame newFrame;

  // If there's no message in the buffer then skip
  if (canChannel[channel]->readMsgBuf(&newFrame.dlc, newFrame.data) == 1)
  {
    return 0;
  }
  // Else, get the message

  gettimeofday(&tv, NULL); // Update timeval

  // Fill in frame data
  newFrame.channel = channel;
  newFrame.timeSec = tv.tv_sec;
  newFrame.timeuSec = tv.tv_usec;
  newFrame.rtr = canChannel[channel]->isRemoteRequest();
  newFrame.ext = canChannel[channel]->isExtendedFrame();
  newFrame.id = canChannel[channel]->getCanId();

  // Attempt to queue up the received frame
  if (!xQueueSend(canRxQueue, &newFrame, 0))
  {
    // CAN RX queue is full, the other task will log this as a warning
    canRxOverflowCount++;
    canRxOverflowInterval++;
  }
  return 1;
}

/************************************************************************** 
 * Function:    writeFile
 * Purpose:     Write a character array to a file
 * Parameters:  FS      - Target filesystem
 *              char*   - Filepath
 *              char*   - message to write
 * Returns:     void
**************************************************************************/
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);
  Serial.println("");

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }
  if (file.print(message))
  {
  }
  else
  {
    ERR(ERR_SD_FAILED_TO_WRITE_FILE);
  }
  file.close();
  return;
}

/************************************************************************** 
 * Function:    appendFile
 * Purpose:     Append a character array to the end of a file
 * Parameters:  FS      - Target filesystem
 *              char*   - Filepath
 *              char*   - message to append
 * Returns:     void
**************************************************************************/
void appendFile(fs::FS &fs, const char *path, const char *message)
{
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }
  if (file.print(message))
  {
  }
  else
  {
    ERR(ERR_SD_FAILED_TO_WRITE_FILE);
  }
  file.close();
  return;
}

/************************************************************************** 
 * Function:    readFile
 * Purpose:     Print the contents of a file to Serial
 * Parameters:  FS      - Target filesystem
 *              char*   - Filepath
 * Returns:     void
**************************************************************************/
void readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\n", path);
  Serial.println("");

  File file = fs.open(path);
  if (!file)
  {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }

  Serial.print("Read from file: ");
  while (file.available())
  {
    Serial.write(file.read());
  }
  file.close();
  return;
}

/************************************************************************** 
 * Function:    deleteFile
 * Purpose:     Delete a file
 * Parameters:  FS      - Target filesystem
 *              char*   - Filepath
 * Returns:     void
**************************************************************************/
void deleteFile(fs::FS &fs, const char *path)
{
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path))
  {
  }
  else
  {
    ERR(ERR_SD_FAILED_TO_DELETE_FILE);
  }
  return;
}

/************************************************************************** 
 * Function:    openNewLog
 * Purpose:     Format a new filename, open the file for writing, and 
 *              reset file size counter
 * Parameters:  None
 * Returns:     void
**************************************************************************/
void openNewLog()
{
  gettimeofday(&tv, NULL);
  sprintf(openLogfile, "/log%u.txt", tv.tv_sec);
  writeFile(SD_MMC, openLogfile, "START\n");
  chunkWrites = 0;
  return;
}

/************************************************************************** 
 * Function:    printErrors
 * Purpose:     Pretty-print the contents of the Error Register to Serial
 * Parameters:  None
 * Returns:     void
**************************************************************************/
void printErrors()
{
  if (errorRegister & ERR_CAN_FAILED_TO_READ_BUFFER_STATUS)
  {
    Serial.println("No buffer flags were read. Possible SPI Error.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_OPEN_FILE)
  {
    Serial.println("Failed to open file on storage.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_WRITE_FILE)
  {
    Serial.println("Failed to write to file on storage.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_DELETE_FILE)
  {
    Serial.println("Failed to delete file from storage.");
  }
  if (errorRegister & ERR_LOG_BUFFER_WRAP)
  {
    Serial.println("Log buffer has wrapped around.");
  }
  if (errorRegister & ERR_CAN_FAILED_TO_READ_REGISTER)
  {
    Serial.println("Register read from MCP2518 was not as expected. Possible SPI Error.");
  }
  if (errorRegister & ERR_CAN_RX_PASSIVE)
  {
    for (uint8_t i = 0; i < 6; i++)
    {
      if (busErrorFlags[i] & 0x08)
      {
        Serial.print("Can controller ");
        Serial.print(i);
        Serial.println(" went to RX Passive state.");
      }
    }
  }
  if (errorRegister & ERR_CAN_TX_PASSIVE)
  {
    for (uint8_t i = 0; i < 6; i++)
    {
      if (busErrorFlags[i] & 0x10)
      {
        Serial.print("Can controller ");
        Serial.print(i);
        Serial.println(" went to TX Passive state.");
      }
    }
  }
  if (errorRegister & ERR_CAN_BUS_OFF)
  {
    for (uint8_t i = 0; i < 6; i++)
    {
      if (busErrorFlags[i] & 0x20)
      {
        Serial.print("Can controller ");
        Serial.print(i);
        Serial.println(" went to Bus Off state.");
      }
    }
  }
  errorRegister = 0;
  return;
}

/************************************************************************** 
 * Function:    printRxOverflow
 * Purpose:     Periodically print a warning if any CAN RX messages were 
 *              dropped (The queue was full when pollCANForMsg tried to 
 *              push to it)
 * Parameters:  None
 * Returns:     void
**************************************************************************/
void printRxOverflow()
{
  gettimeofday(&tv, NULL);
  time_t now = tv.tv_sec;
  if (canRxOverflowInterval > 0 && now - canRxOverflowLastLog > CAN_QUEUE_OVERFLOW_LOG_INTERVAL_SECONDS)
  {
    uint64_t intervalCount = canRxOverflowInterval.exchange(0);
    canRxOverflowLastLog = now;
    Serial.print("RX Queue Overflow recent=");
    Serial.print(intervalCount);
    Serial.print(" total=");
    Serial.println(canRxOverflowCount);
  }
}

/************************************************************************** 
 * Function:    setup
 * Purpose:     Initialize all peripherals, create RXQueue, Pin tasks, etc.
 * Parameters:  None
 * Returns:     void
**************************************************************************/
void setup()
{
  Serial.begin(921600);
  // while(!Serial.available()){vTaskDelay(xDelay);}
  Wire.begin(1, 2);

  // Assign SD_MMC interface pins
  SD_MMC.setPins(6, 5, 7, 15, 16, 17);

  // Start SD_MMC interface
  if (!SD_MMC.begin())
  {
    if (VERBOSE_ERR)
    {
      Serial.println("Storage Mount Failed");
    }
    return;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE)
  {
    if (VERBOSE_ERR)
    {
      Serial.println("No SD_MMC card attached");
    }
    return;
  }

  // Initialize the RV-3028-C7 Real Time Clock
  if (rtc.begin() == false)
  {
    if (VERBOSE_ERR)
    {
      Serial.println("Real-Time Clock not detected");
    }
  }

  // Initialize the PCA9554 GPIO expander
  if (io.begin(0x3F) == false)
  {
    if (VERBOSE_ERR)
    {
      Serial.println("GPIO Expander not detected");
    }
  }

  // Enable all MCP2518s via the GPIO expander
  for (uint8_t i = 0; i < 6; i++)
  {
    io.pinMode(canEnable[i], OUTPUT);
    io.digitalWrite(canEnable[i], 1);
  }

  // Update RTC config
  uint8_t backupReg = rtc.readByteFromEEPROM(0x37);
  // Trickle charge enable + Direct switching mode + 3kOhm charge resistance
  backupReg |= 0b00110100;
  rtc.writeByteToEEPROM(0x37, backupReg);

  //  Set system time from RTC
  setTimeFromRTC();

  // Start Logfile
  openNewLog();

  // Create CAN RX buffer queue
  canRxQueue = xQueueCreate(SIZE_CAN_RX_QUEUE_IN_FRAMES, sizeof(CANFrame));

  // Create pinned task
  xTaskCreatePinnedToCore(canMonitor,         // Task Function
                          "CAN Monitor Task", // Task Name
                          20000,              // Stack Size (words)
                          NULL,               // Input Param
                          1,                  // Priority
                          &canTask,           // Task Handle
                          1);                 // Core where the task should run

  // Create pinned task
  xTaskCreatePinnedToCore(appMain,         // Task Function
                          "Main App Task", // Task Name
                          20000,           // Stack Size (words)
                          NULL,            // Input Param
                          0,               // Priority
                          &loggingTask,    // Task Handle
                          0);              // Core where the task should run

  esp_task_wdt_init(10, false);
  esp_task_wdt_add(loggingTask);
}

/************************************************************************** 
 * Function:    initCAN
 * Purpose:     Start SPI peripherals and attempt to initialize all 
 *              CAN controllers. Report status to Serial. Will retry
 *              each channel mcpInitRetry number of times.
 * Parameters:  None
 * Returns:     void
**************************************************************************/
void initCAN()
{
  // Start SPI busses
  spi0->begin(14, 12, 13, -1);
  spi1->begin(11, 9, 10, -1);

  // Call beginTransaction() on both SPI busses. We only call this
  // once at startup because using begin/endTransaction slows SPI
  // transactions to a crawl
  spi0->beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  spi1->beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

  // Configure and start each CAN controller
  for (uint8_t i = 0; i < 6; i++)
  {
    canChannel[i]->setMode(CAN_CLASSIC_MODE);
    if (i < 3)
    {
      canChannel[i]->setSPI(spi0);
    }
    else
    {
      canChannel[i]->setSPI(spi1);
    }

    for (uint8_t retry = 0; retry < mcpInitRetry; retry++)
    {
      canChannel[i]->begin(CAN_250KBPS, MCP2518FD_40MHz, false);
      delay(100);
      if (canChannel[i]->getMode() != CAN_CLASSIC_MODE)
      {
        if (VERBOSE_ERR)
        {
          Serial.print("Error initializing MCP2518 Channel ");
          Serial.println(i);
          Serial.print("Retry ");
          Serial.print(retry);
          Serial.print(" of ");
          Serial.print(mcpInitRetry);
        }
      }
      else
      {
        Serial.print("MCP2518 Channel ");
        Serial.print(i);
        Serial.println(" init success.");
        break;
      }
    }
  }
  return;
}

/************************************************************************** 
 * Function:    canMonitor
 * Purpose:     This task is pinned to Core 1 and monopolizes the core
 *              in order to poll the CAN controllers as quickly as 
 *              possible. At 250kbps, the minimum frame length (depending
 *              on bit packing) is 188uS. Ideally, the for-loop in this
 *              function executes in less time, so as not to back up the
 *              CAN controller FIFO
 * Parameters:  void*   - Task Parameters
 * Returns:     void
**************************************************************************/
void canMonitor(void *parameter)
{
  esp_task_wdt_init(10, false);
  esp_task_wdt_add(NULL);

  initCAN();

  for (;;)
  {
    // Poll all of the CAN transceivers
    for (uint8_t i = 0; i < 6; i++)
    {
      pollCANForMsg(i);
    }
    if (checkCANErr)
    {
      for (uint8_t i = 0; i < 6; i++)
      {
        checkCANErrors(i);
      }
      checkCANErr = 0;
      freshCANErr = 1;
    }
    esp_task_wdt_reset();
  }
  return;
}

/************************************************************************** 
 * Function:    ANSI_clear
 * Purpose:     Convenience function to write the ANSI Terminal Clear 
 *              Escape Sequence. Arduino IDE Serial Terminal doesn't 
 *              respect ANSI Escape codes but most terminal emulators do
 * Parameters:  None
 * Returns:     void
**************************************************************************/
void ANSI_clear() { Serial.write("\033[2J\033[H", 7); }

/************************************************************************** 
 * Function:    wait
 * Purpose:     Convenience function to empty the Serial receive buffer 
 *              and wait for user input during menu navigation without 
 *              blocking appMain()
 * Parameters:  None
 * Returns:     void
**************************************************************************/
void wait()
{
  while (Serial.available())
  {
    Serial.read();
  } // Empty Serial Buffer
  debugMenuActive = true;
  appMain(NULL);
  debugMenuActive = false;
  return;
}

/************************************************************************** 
 * Function:    debugMenu
 * Purpose:     Display Serial Debug Menu. Logging continues while the 
 *              menu is open.
 * Parameters:  None
 * Returns:     void
**************************************************************************/
void debugMenu()
{
  exitMenu = false; // Lazy flag to break from menu loop
  uint32_t setTime;
  for (;;)
  {
    Serial.println("DEBUG MENU");
    Serial.println("---------------------");
    Serial.println("1) Start New Log");
    Serial.println("2) Delete Current Log");
    Serial.println("3) Test SD Card Read/Write");
    Serial.println("4) Set System Time");
    Serial.println("5) Live CAN Traffic");
    Serial.println("6) Exit Debug Menu");
    wait();
    ANSI_clear();
    switch (Serial.read())
    {
    case '1':
      openNewLog();
      Serial.println("Attempting to Close Log and open New Log...");
      Serial.println("Press Any Key To Return");
      wait();
      Serial.read(); // Throw away the "any key"
      ANSI_clear();
      break;
    case '2':
      deleteFile(SD_MMC, openLogfile);
      openNewLog();
      Serial.println("Attempting to Delete Log and open New Log...");
      Serial.println("Press Any Key To Return");
      wait();
      Serial.read(); // Throw away the "any key"
      ANSI_clear();
      break;
    case '3':
      Serial.println("Attempting to Write to SD");
      writeFile(SD_MMC, "/test.txt", "TEST SUCCESSFUL!");
      Serial.println("Attempting to Read from SD");
      readFile(SD_MMC, "/test.txt");
      deleteFile(SD_MMC, "/test.txt");
      Serial.println("Press Any Key To Return");
      wait();
      Serial.read(); // Throw away the "any key"
      ANSI_clear();
      break;
    case '4':
      Serial.print("Current Time is: ");
      // Serial.println(rtc.getCurrentDateTime());
      Serial.println(rtc.getUnixTimestamp());
      Serial.println("Enter new unix time:");
      wait();
      setTime = strtoul(Serial.readString().c_str(), NULL, 10);
      if (setTime == 0)
      {
        Serial.println("Time will not be changed.");
        Serial.println("Press Any Key To Return");
        wait();
        Serial.read(); // Throw away the "any key"
        ANSI_clear();
        break;
      }
      if (rtc.setUnixTimestamp(setTime, true))
      {
        Serial.print("Time successfully set to: ");
        Serial.println(rtc.getUnixTimestamp());
        setTimeFromRTC();
        gettimeofday(&tv, NULL);
        Serial.println(tv.tv_sec);
        Serial.println("Press Any Key To Return");
      }
      else
      {
        Serial.print("Failed to set time. Check your formatting.");
        Serial.println("Press Any Key To Return");
      }
      wait();
      Serial.read(); // Throw away the "any key"
      ANSI_clear();
      break;
    case '5':
      Serial.println("Logging will be suspended during Live View");
      Serial.println("Press Any Key To Continue");
      Serial.println("Press again at any time to Exit Live View");
      wait();
      Serial.read(); // Throw away the "any key"
      canFlow = CAN_TO_SERIAL;
      wait();
      Serial.read(); // Throw away the "any key"
      ANSI_clear();
      canFlow = CAN_TO_SDMMC;
      openNewLog();
      break;
    case '6':
      exitMenu = true;
      break;
    default:
      Serial.println("Invalid Command (Try '6' to leave menu?)");
      break;
    }
    while (Serial.available())
    {
      Serial.read();
    } // Empty Serial Buffer
    if (exitMenu)
    {
      break;
    }
  }
  return;
}

/************************************************************************** 
 * Function:    stringToSDBuffer
 * Purpose:     Convenience function to write the contents of a 
 *              character array to the SD write buffer. Appends a 
 *              newline to the end.
 * Parameters:  char* - Character array to write to SD  
 * Returns:     void
**************************************************************************/
void stringToSDBuffer(char *inString)
{
  for (uint8_t idx = 0; inString[idx] != '\0'; idx++)
  {
    bufferSD[SDpos] = inString[idx];
    SDpos++;
  }
  bufferSD[SDpos] = '\n';
  SDpos++;
  return;
}

/************************************************************************** 
 * Function:    appMain
 * Purpose:     This task is pinned to Core 0. All of the application
 *              logic is spawned from here. It consumes CAN frames from
 *              the RXQueue, formats them, and delivers them to either
 *              storage or Serial. This task also manages various timers
 *              and launches the debug menu.
 * Parameters:  void*   - Task Parameters
 * Returns:     void
**************************************************************************/
void appMain(void *parameter)
{
  for (;;)
  {
    CANFrame frame;
    // Receive a frame from other task. Blocks up to TIMEOUT_SD_FLUSH_MS.
    bool new_frame = xQueueReceive(canRxQueue, &frame, pdMS_TO_TICKS(TIMEOUT_SD_FLUSH_MS));
    if (new_frame)
    {
      // Format each byte of frame data and append to a character array
      // Give values under 0x10 a leading zero. Terminate array with '\0'
      char rawtoa[64];
      uint8_t rawidx = 0;
      char buf[3];
      for (uint8_t tmpidx = 0; tmpidx < frame.dlc; tmpidx++)
      {
        itoa(frame.data[tmpidx], buf, 16);
        if (frame.data[tmpidx] < 0x10)
        {
          rawtoa[rawidx] = '0';
          rawidx++;
          for (uint8_t bufidx = 0; buf[bufidx] != '\0'; bufidx++)
          {
            rawtoa[rawidx] = buf[bufidx];
            rawidx++;
          }
        }
        else
        {
          for (uint8_t bufidx = 0; buf[bufidx] != '\0'; bufidx++)
          {
            rawtoa[rawidx] = buf[bufidx];
            rawidx++;
          }
        }
        memset(buf, 0, sizeof(buf));
      }
      rawtoa[rawidx] = '\0';
      // Format the log entry using sprintf
      char logEntry[64];
      if (frame.rtr) // If the frame is a Request Frame, format accordingly
      {
        sprintf(logEntry, "(%ld.%ld) can%d %X#R", frame.timeSec, frame.timeuSec, frame.channel, frame.id);
      }
      else
      {
        sprintf(logEntry, "(%ld.%ld) can%d %X#%s", frame.timeSec, frame.timeuSec, frame.channel, frame.id, rawtoa);
      }
      if (canFlow == CAN_TO_SDMMC)
      {
        stringToSDBuffer(logEntry);
      }
      else if (canFlow == CAN_TO_SERIAL)
      {
        Serial.println(logEntry);
      }
    }

    // Flush to SD if either of:
    // - SIZE_SDMMC_CHUNK_WRITE_IN_BYTES waiting to write
    // - No message received for TIMEOUT_SD_FLUSH_MS and there is anything to write
    if (SDpos > SIZE_SDMMC_CHUNK_WRITE_IN_BYTES || (!new_frame && SDpos > 0))
    {
      chunkWrites++;
      appendFile(SD_MMC, openLogfile, bufferSD);
      memset(bufferSD, '\0', sizeof(bufferSD));
      // If filesize is getting too big (> SIZE_MAXIMUM_LOGFILE_IN_MBYTES)
      // start a new one
      if (chunkWrites > maxLogSizeChunks)
      {
        chunkWrites = 0;
        openNewLog();
      }
      SDpos = 0;
    }

    // Check if the CAN_ERR_CHECK_INTERVAL_SECONDS has elapsed since
    // last CAN error check. If so, flag the Core 1 process.
    gettimeofday(&tv, NULL);
    if (tv.tv_sec - lastCheckCANErr > CAN_ERR_CHECK_INTERVAL_SECONDS)
    {
      lastCheckCANErr = tv.tv_sec;
      checkCANErr = 1;
    }

    // If new CAN bus errors are present, set ERR bits and record to log
    if (freshCANErr)
    {
      char errString[64];
      gettimeofday(&tv, NULL);
      for (uint8_t i = 0; i < 6; i++)
      {
        // RX PASSIVE
        if (busErrorFlags[i] & 0x08)
        {
          ERR(ERR_CAN_RX_PASSIVE);
          sprintf(errString, "(%ld.%ld) ERR:can%d went to rx passive state", tv.tv_sec, tv.tv_usec, i);
          stringToSDBuffer(errString);
        }
        // TX PASSIVE
        if (busErrorFlags[i] & 0x10)
        {
          ERR(ERR_CAN_TX_PASSIVE);
          sprintf(errString, "(%ld.%ld) ERR:can%d went to tx passive state", tv.tv_sec, tv.tv_usec, i);
          stringToSDBuffer(errString);
        }
        // BUS OFF
        if (busErrorFlags[i] & 0x20)
        {
          ERR(ERR_CAN_BUS_OFF);
          sprintf(errString, "(%ld.%ld) ERR:can%d went to bus off state", tv.tv_sec, tv.tv_usec, i);
          stringToSDBuffer(errString);
        }
      }
      freshCANErr = 0;
    }

    esp_task_wdt_reset();

    if (!Serial.available())
    {
      if (errorRegister && VERBOSE_ERR)
      {
        printErrors();
      }
      printRxOverflow();
    }
    if (Serial.available() && !debugMenuActive) // If the user has sent a Serial character, launch the menu
    {
      debugMenu();
    }
    if (Serial.available() && debugMenuActive) // If the menu is active, we're in a second instance of appMain. Break out.
    {
      break;
    }
  }
  return;
}

// The Arduino IDE expects this but it's more convenient to create our own tasks
// and assign them whatever core/priority we want and just kill this one
void loop()
{
  vTaskDelete(NULL);
}