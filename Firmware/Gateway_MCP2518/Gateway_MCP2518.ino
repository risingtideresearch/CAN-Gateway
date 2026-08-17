// FQBN: esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=enabled,LoopCore=0

#include <atomic>
#include <SPI.h>
#include <SparkFun_I2C_Expander_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_I2C_Expander_Arduino_Library
#include <esp_task_wdt.h>
#include <freertos/queue.h>
#include <RV3028C7.h>
#include "mcp2518fd_can.h"
#include "FS.h"
#include "SD_MMC.h"
#include "time.h"

// Storage related parameters
#define SIZE_CAN_RX_QUEUE_IN_FRAMES 1024
#define SIZE_SDMMC_BUFFER_IN_BYTES 65536
#define SIZE_SDMMC_CHUNK_WRITE_IN_BYTES 32767
#define SIZE_MAXIMUM_LOGFILE_IN_MBYTES 200 

// Error printing helpers
#define ERR_CAN_FAILED_TO_READ_BUFFER_STATUS 0x01
#define ERR_SD_FAILED_TO_OPEN_FILE 0x02
#define ERR_SD_FAILED_TO_WRITE_FILE 0x04
#define ERR_SD_FAILED_TO_DELETE_FILE 0x08
#define ERR_LOG_BUFFER_WRAP 0x10
#define ERR_CAN_FAILED_TO_READ_REGISTER 0x12
#define VERBOSE_ERR true
uint8_t errorRegister = 0;

#define CAN_QUEUE_OVERFLOW_LOG_INTERVAL_MS 10000  // Log about queue overflow at most this often

#define TIMEOUT_SD_FLUSH_MS 50 // Flush a partial buffer to SD if the bus is quiet for this many milliseconds

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
struct CANFrame{
  uint8_t  channel; 
  time_t  timeSec;
  suseconds_t  timeuSec;
  uint8_t  data[8];
  uint32_t  id;  
  uint8_t  dlc;
  bool  rtr;
  bool  ext;
};

// Queue for incoming CAN traffic to write CAN traffic to (queue of struct CANFrame)
QueueHandle_t canRxQueue;

uint32_t canRxOverflowCount;    // Number of times canRxQueue has overflowed since boot
std::atomic<uint32_t> canRxOverflowInterval; // Number of times canRxQueue has overflowed since last log line
uint32_t canRxOverflowLastLog;  // Timestamp of the last time we logged overflows (limited to CAN_QUEUE_OVERFLOW_LOG_INTERVAL_MS)

// Buffer for writing to storage
char bufferSD[SIZE_SDMMC_BUFFER_IN_BYTES];
uint16_t SDpos = 0;

// Calculate the number of chunk writes to approach maximum logfile size
const uint32_t maxLogSizeChunks = (SIZE_MAXIMUM_LOGFILE_IN_MBYTES*1000000)/SIZE_SDMMC_CHUNK_WRITE_IN_BYTES;
// Track the number of chunks written to open logfile
uint32_t chunkWrites = 0;

// Map GPIO Expander pins to CAN EN pins
uint8_t canEnable[] = {4,5,3,2,1,0}; 

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

// Number of times to attempt intialization of each CAN transceiver on failure
uint8_t mcpInitRetry = 5; 

// Name of the open logfile
char openLogfile[20];

// Add an error code to the register for printing in VERBOSE mode
void ERR(uint8_t errorCode) {
  errorRegister = errorRegister | errorCode;
  return;
}

// Set the ESP32 system time from the RTC
bool setTimeFromRTC() {
  uint32_t rtcTime = rtc.getUnixTimestamp();
  struct timeval systime;
  int rc;
  //Serial.println(rtcTime);
  systime.tv_sec = rtcTime;
  systime.tv_usec = 0;
  rc = settimeofday(&systime, NULL);
  if (rc == 0) {
    if (VERBOSE_ERR) {
      Serial.println("System time set successfully");
    }
    return 1;
  } else {
    if (VERBOSE_ERR) {
      Serial.println("System time set failed");
    }
    return 0;    
  }
  return 0;
}

// Get RXMsg from MCP2518 over SPI. Time critical. 
bool rx(uint8_t channel) {

  //digitalWrite(41, 1);

  CANFrame newFrame;

  if (canChannel[channel]->readMsgBuf(&newFrame.dlc, newFrame.data) == 1) {
    return 0;
  }
  // Else, get the message

	gettimeofday(&tv, NULL);

  // This can be optimized. Basically copying a structure that
  // already exists in the lib. Could modify lib to just hand
  // over the struct
  newFrame.channel = channel;
  newFrame.timeSec = tv.tv_sec;
  newFrame.timeuSec = tv.tv_usec;
  newFrame.rtr = canChannel[channel]->isRemoteRequest();
  newFrame.ext = canChannel[channel]->isExtendedFrame();
  newFrame.id = canChannel[channel]->getCanId();

  if (!xQueueSend(canRxQueue, &newFrame, 0)) {
    // CAN RX queue is full, the other task will log this as a warning
    canRxOverflowCount++;
    canRxOverflowInterval++;
  }
  
  //digitalWrite(41, 0);

  return 1;
}

/***SD Convenience Functions***/
void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);
  Serial.println("");

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }
  if (file.print(message)) {
  } else {
    ERR(ERR_SD_FAILED_TO_WRITE_FILE);
  }
  file.close();
  return;
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }
  if (file.print(message)) {
  } else {
    ERR(ERR_SD_FAILED_TO_WRITE_FILE);
  }
  file.close();
  return;
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  Serial.println("");

  File file = fs.open(path);
  if (!file) {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  return;
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
  } else {
    ERR(ERR_SD_FAILED_TO_DELETE_FILE);
  }
  return;
}
/***************************/

// Format new filename, open file for writing, reset size counter
void openNewLog() {
  gettimeofday(&tv, NULL);
  sprintf(openLogfile, "/log%u.txt", tv.tv_sec);
  writeFile(SD_MMC, openLogfile, "START\n");
  chunkWrites = 0;
  return;
}

// Pretty-print the contents of the Error Register to Serial
void printErrors() {
  if (errorRegister & ERR_CAN_FAILED_TO_READ_BUFFER_STATUS) {
    Serial.println(
        "MCP2515 interrupt fired but no buffer flags were read. Possible SPI "
        "Error.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_OPEN_FILE) {
    Serial.println("Failed to open file on storage.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_WRITE_FILE) {
    Serial.println("Failed to write to file on storage.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_DELETE_FILE) {
    Serial.println("Failed to delete file from storage.");
  }
  if (errorRegister & ERR_LOG_BUFFER_WRAP) {
    Serial.println("Log buffer has wrapped around.");
  }
  if (errorRegister & ERR_CAN_FAILED_TO_READ_REGISTER) {
    Serial.println("Register read from MCP2515 was not as expected. Possible "
    "SPI Error.");
  }
  errorRegister = 0;
  return;
}

// Periodically print a warning if any CAN RX messages were dropped
void printRxOverflow() {
  size_t now = millis();
  if (canRxOverflowInterval > 0 && canRxOverflowLastLog - now > CAN_QUEUE_OVERFLOW_LOG_INTERVAL_MS) {
    Serial.print("RX Queue Overflow recent=");
    Serial.print(canRxOverflowInterval);
    Serial.print(" total=");
    Serial.println(canRxOverflowCount);
    canRxOverflowInterval = 0;
    canRxOverflowLastLog = now;
  }
}

// Initialize ALL THE THINGS
void setup() {

  Serial.begin(921600);
  // while(!Serial.available()){vTaskDelay(xDelay);}
  Wire.begin(1, 2);

  // Assign SD_MMC interface pins
  SD_MMC.setPins(6, 5, 7, 15, 16, 17);

  // Start SD_MMC interface
  if (!SD_MMC.begin()) {
    if (VERBOSE_ERR) {
      Serial.println("Storage Mount Failed");
    }
    return;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    if (VERBOSE_ERR) {
      Serial.println("No SD_MMC card attached");
    }
    return;
  }

  // Initialize the RV-3028-C7 Real Time Clock
  if (rtc.begin() == false) {
    if (VERBOSE_ERR) {
      Serial.println("Real-Time Clock not detected");
    }
  }

  // Initialize the PCA9554 GPIO expander
  if (io.begin(0x3F) == false) {
    if (VERBOSE_ERR) {
      Serial.println("GPIO Expander not detected");
    }
  }

  // Enable all MCP2518s via the GPIO expander
  for (uint8_t i = 0; i < 6; i++) {
    io.pinMode(canEnable[i], OUTPUT);
    io.digitalWrite(canEnable[i], 1);
  }

  // Update RTC config
  uint8_t backupReg = rtc.readByteFromEEPROM(0x37);
  // Trickle charge enable + Direct switching mode + 3kOhm charge resistance
  backupReg |= 0b00110100; 
  rtc.writeByteToEEPROM(0x37, backupReg);
  //Serial.println(rtc.readByteFromEEPROM(0x37));
  // Set system time from RTC
  setTimeFromRTC();

  // Start Logfile
  openNewLog();
  
  //DEBUG FLAGS
  /*
  pinMode(41, OUTPUT);
  digitalWrite(41, 0);
  pinMode(42, OUTPUT);
  digitalWrite(42, 0);
  pinMode(45, OUTPUT);
  digitalWrite(45, 0);
  pinMode(46, OUTPUT);
  digitalWrite(46, 0);
  */
  
  // Create CAN RX buffer queue
  canRxQueue = xQueueCreate(SIZE_CAN_RX_QUEUE_IN_FRAMES, sizeof(CANFrame));

  // Create pinned task
  xTaskCreatePinnedToCore(canMonitor,   // Task Function
                          "CAN Monitor Task", // Task Name
                          20000,        // Stack Size (words)
                          NULL,         // Input Param
                          1,            // Priority
                          &canTask,     // Task Handle
                          1);           // Core where the task should run  

  // Create pinned task
  xTaskCreatePinnedToCore(appMain,   // Task Function
                          "Main App Task", // Task Name
                          20000,        // Stack Size (words)
                          NULL,         // Input Param
                          0,            // Priority
                          &loggingTask,     // Task Handle
                          0);           // Core where the task should run                            

}

void initCAN() {
  // Start SPI busses
  spi0->begin(14, 12, 13, -1);
  spi1->begin(11, 9, 10, -1);

  spi0->beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  spi1->beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

  // Configure and start each CAN tcvr
  
  for (uint8_t i = 0; i < 6; i++) {
    canChannel[i]->setMode(CAN_CLASSIC_MODE);
    if (i<3) {
      canChannel[i]->setSPI(spi0);
    }else{
      canChannel[i]->setSPI(spi1);
    }
    
    for(uint8_t retry = 0; retry < mcpInitRetry; retry++) {
      canChannel[i]->begin(CAN_250KBPS, MCP2518FD_40MHz, false);
      delay(100);
      if (canChannel[i]->getMode() != CAN_CLASSIC_MODE) {
        if (VERBOSE_ERR) {
          Serial.print("Error initializing MCP2518 Channel ");
          Serial.println(i);
          Serial.print("Retry ");
          Serial.print(retry);
          Serial.print(" of ");
          Serial.print(mcpInitRetry);
        }
      }else{
        Serial.print("MCP2518 Channel ");
        Serial.print(i);
        Serial.println(" init success.");
        break;
      }
    }
  }
  return;  
}

// This task is pinned to Core 1. Its job is mostly to get interrupted.
// It also waits for Serial and reports errors. 
void canMonitor(void *parameter) {

  esp_task_wdt_init(10, false); 
  esp_task_wdt_add(NULL);

  initCAN();

  for (;;) {
    // Poll all of the CAN transceivers  
    for (uint8_t i = 0; i < 6; i++) {
      rx(i);
    } 
    esp_task_wdt_reset();
  }
  return;
}

// Convenience function to write the ANSI Terminal Clear Escape Sequence
// Arduino IDE Serial Terminal doesn't respect ANSI Escape codes but 
// Most terminal emulators do
void ANSI_clear() { Serial.write("\033[2J\033[H", 7); }

// Convenience function to empty the Serial receive buffer and wait for 
// user input during menu navigation
void wait() {
  while (Serial.available()) {
    Serial.read();
  }  // Empty Serial Buffer
  while (!Serial.available()) {
    delay(200);
      esp_task_wdt_reset();
      taskYIELD();  
  }
  return;
}

// Debug menu. Pauses logging and allows manipulation of MCP2515 and Storage
void debugMenu() {
  exitMenu = false; // Lazy flag to break from menu loop
  uint32_t setTime;
  for (;;) {
    Serial.println("DEBUG MENU");
    Serial.println("---------------------");
    Serial.println("1) Dump Log");
    Serial.println("2) Delete Log");
    Serial.println("3) Test SD Card");
    Serial.println("4) Set Time");
    Serial.println("5) Resume Logging");
    wait();
    ANSI_clear();
    switch (Serial.read()) {
      case '1': 
        readFile(SD_MMC, openLogfile);
        Serial.println("Press Any Key To Return");
        wait();
        ANSI_clear();
        break;
      case '2':
        deleteFile(SD_MMC, openLogfile);
        openNewLog();
        break;
      case '3':
        Serial.println("Attempting to Write to SD");
        writeFile(SD_MMC, "/test.txt", "TEST SUCCESSFUL!");
        Serial.println("Attempting to Read from SD");
        readFile(SD_MMC, "/test.txt");
        deleteFile(SD_MMC, "/test.txt");
        Serial.println("Press Any Key To Return");
        wait();
        ANSI_clear();
        break;
      case '4':
        Serial.print("Current Time is: ");
        //Serial.println(rtc.getCurrentDateTime());
        Serial.println(rtc.getUnixTimestamp());
        Serial.println("Enter new unix time:");
        wait();
        setTime = strtoul(Serial.readString().c_str(), NULL, 10);
        if (setTime == 0) {
          Serial.println("Time will not be changed.");
          Serial.println("Press Any Key To Return");  
          wait();
          ANSI_clear();
          break;
        }
        if (rtc.setUnixTimestamp(setTime, true)) {
          Serial.print("Time successfully set to: ");
          Serial.println(rtc.getUnixTimestamp());
          setTimeFromRTC();
          gettimeofday(&tv, NULL);
          Serial.println(tv.tv_sec);
          Serial.println("Press Any Key To Return");          
        } else {
          Serial.print("Failed to set time. Check your formatting.");
          Serial.println("Press Any Key To Return");  
        }
        wait();
        ANSI_clear();
        break;
      case '5':
        exitMenu = true;
        break;
      default:
        Serial.println("Invalid Command (Try '5' to leave menu?)");
        break;
    }
    while (Serial.available()) {
      Serial.read();
    }  // Empty Serial Buffer
    if (exitMenu) {
      break;
    }
  }
  return;
}

// This task is pinned to Core 0. Its job is to read CAN frames
// from the buffer queue, format them, and write to the storage.
void appMain(void *parameter) {
  esp_task_wdt_init(10, false); 
  esp_task_wdt_add(NULL);

  for (;;) {
    //digitalWrite(42, 1);
    CANFrame frame;
    // Receive a frame from other task. Blocks up to TIMEOUT_SD_FLUSH_MS.
    bool new_frame = xQueueReceive(canRxQueue, &frame, pdMS_TO_TICKS(TIMEOUT_SD_FLUSH_MS));
    if (new_frame) {
      digitalWrite(45, 1);
      char rawtoa[64];
      uint8_t rawidx = 0;
      char buf[3];
      for (uint8_t tmpidx = 0; tmpidx < frame.dlc; tmpidx++) {
        itoa(frame.data[tmpidx], buf, 16);
        if (frame.data[tmpidx] < 0x10) {
          rawtoa[rawidx] = '0';
          rawidx++;
          for (uint8_t bufidx = 0; buf[bufidx] != '\0'; bufidx++) {
            rawtoa[rawidx] = buf[bufidx];
            rawidx++;
          }
        } else {
          for (uint8_t bufidx = 0; buf[bufidx] != '\0'; bufidx++) {
            rawtoa[rawidx] = buf[bufidx];
            rawidx++;
          }        
        }  
        memset(buf, 0, sizeof(buf));
      }
      rawtoa[rawidx] = '\0';
      char logEntry[64];
      if (frame.rtr) {
        sprintf(logEntry, "(%ld.%ld) can%d %X#R", frame.timeSec, frame.timeuSec, frame.channel, frame.id);
      } else {
        sprintf(logEntry, "(%ld.%ld) can%d %X#%s", frame.timeSec, frame.timeuSec, frame.channel, frame.id, rawtoa);
      }
      for (uint8_t logidx = 0; logEntry[logidx] != '\0'; logidx++) {
        bufferSD[SDpos] = logEntry[logidx];
        SDpos++;
      }
      bufferSD[SDpos] = '\n';
      SDpos++;
      //digitalWrite(45, 0);
    }

    // Flush to SD if either of:
    // - SIZE_SDMMC_CHUNK_WRITE_IN_BYTES waiting to write
    // - No message received for TIMEOUT_SD_FLUSH_MS and there is anything to write
    if (SDpos > SIZE_SDMMC_CHUNK_WRITE_IN_BYTES || (!new_frame && SDpos > 0)) {
      //digitalWrite(46, 1);
      chunkWrites++;
      appendFile(SD_MMC, openLogfile, bufferSD);
      memset(bufferSD, '\0', sizeof(bufferSD));
      // If filesize is getting too big (> SIZE_MAXIMUM_LOGFILE_IN_MBYTES)
      // start a new one
      if (chunkWrites > maxLogSizeChunks) {
        chunkWrites = 0;
        openNewLog();
      }
      SDpos = 0;
      //digitalWrite(46, 0);
    }

    esp_task_wdt_reset();
  
    if (!Serial.available()) {
      if (errorRegister && VERBOSE_ERR) {
        printErrors();
      }
      printRxOverflow();
    }
    if (Serial.available()) {
      appendFile(SD_MMC, openLogfile, bufferSD);
      memset(bufferSD, '\0', sizeof(bufferSD));
      SDpos = 0;
      debugMenu();
    }
    //digitalWrite(42, 0);
  }
}

// The Arduino IDE expects this but it's more convenient to create our own tasks
// and assign them whatever core/priority we want and just kill this one
void loop() {
  vTaskDelete(NULL);
}
