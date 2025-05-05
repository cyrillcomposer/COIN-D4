/*
 * ESP32 Interface for Coin D4 LiDAR - Optimized Version
 * 
 * Features:
 * - Multi-core processing (Core 0: Serial input, Core 1: Processing)
 * - Integer-only angle calculations for speed
 * - Fixed serial buffer initialization sequence
 * - Proper filtering for accurate readings in all ranges
 * - Optimized memory usage and performance
 */

 #include <Arduino.h>
 #include <atomic>
 
 // LiDAR control commands
 const uint8_t START_LIDAR[4] = {0xAA, 0x55, 0xF0, 0x0F};
 const uint8_t STOP_LIDAR[4] = {0xAA, 0x55, 0xF5, 0x0A};
 
 // LiDAR serial connection
 #define LIDAR_SERIAL Serial2
 #define LIDAR_RX_PIN 16
 #define LIDAR_TX_PIN 17
 #define LIDAR_BAUDRATE 230400
 #define LIDAR_RX_BUFFER_SIZE 1024
 
 // Buffer and packet constants
 #define BUFFER_SIZE 4096
 #define BUFFER_MASK (BUFFER_SIZE - 1)
 #define PACKAGE_PAID_BYTES 10
 #define HEADER_BYTE1 0xAA
 #define HEADER_BYTE2 0x55
 #define PH 0x55AA
 
 // Data filtering constants
 #define MIN_VALID_DISTANCE 150  // 15.0cm (in 0.1mm units)
 #define MIN_VALID_INTENSITY 65
 
 // Angle constants (integer only)
 #define ANGLE_UNITS_PER_DEGREE 64  // 64 units = 1 degree
 #define ANGLE_360_UNITS (360 * ANGLE_UNITS_PER_DEGREE)
 
 // Batching constants
 #define BATCH_SIZE 512
 #define BATCH_THRESHOLD 128
 
 // Circular buffer with atomic counters
 uint8_t buffer[BUFFER_SIZE] __attribute__((aligned(4)));
 std::atomic<uint32_t> writePos(0);
 std::atomic<uint32_t> readPos(0);
 
 // Batch output buffer
 char batchBuffer[BATCH_SIZE];
 volatile uint16_t batchPos = 0;
 
 // Synchronization
 portMUX_TYPE dataMutex = portMUX_INITIALIZER_UNLOCKED;
 
 // Task handles
 TaskHandle_t inputTask;
 TaskHandle_t processTask;
 
 // Statistics
 std::atomic<uint32_t> packetsProcessed(0);
 std::atomic<uint32_t> checksumErrors(0);
 std::atomic<uint32_t> filteredPoints(0);
 
 // Flush batch buffer - called from processing task only
 inline void flushBatch() {
   if (batchPos > 0) {
     Serial.write(batchBuffer, batchPos);
     batchPos = 0;
   }
 }
 
 // Add to batch buffer - called from processing task only
 inline void addToBatch(const char* data, uint16_t len) {
   if (batchPos + len >= BATCH_SIZE - 1) {
     flushBatch();
   }
   memcpy(&batchBuffer[batchPos], data, len);
   batchPos += len;
 }
 
 // Get available bytes safely
 inline uint32_t getAvailableBytes() {
   uint32_t currentWrite = writePos.load();
   uint32_t currentRead = readPos.load();
   return (currentWrite - currentRead) & BUFFER_MASK;
 }
 
 // Input task - only reads from LiDAR serial and adds to buffer
 void inputTaskCode(void* parameter) {
   while (true) {
     // Only read if buffer has space
     while (LIDAR_SERIAL.available()) {
       uint32_t currentWrite = writePos.load();
       uint32_t nextWrite = (currentWrite + 1) & BUFFER_MASK;
       
       // Check for buffer full
       if (nextWrite != readPos.load()) {
         portENTER_CRITICAL(&dataMutex);
         buffer[currentWrite] = LIDAR_SERIAL.read();
         portEXIT_CRITICAL(&dataMutex);
         
         writePos.store(nextWrite);
       } else {
         // Buffer full, drop data
         LIDAR_SERIAL.read();  // Discard byte
       }
     }
     
     // Short yield to allow other task to run
     vTaskDelay(1);
   }
 }
 
 // Safe buffer read with mutex
 inline uint8_t safeReadBuffer(uint32_t pos) {
   uint8_t value;
   portENTER_CRITICAL(&dataMutex);
   value = buffer[(readPos.load() + pos) & BUFFER_MASK];
   portEXIT_CRITICAL(&dataMutex);
   return value;
 }
 
 // Integer-based angle correction approximation
 inline int32_t calculateAngleCorrection(uint16_t distance) {
   if (distance == 0) return 0;
   
   // Simple integer approximation of the original trigonometric correction
   if (distance < 500) {
     // More correction for close objects (roughly similar curve to the original)
     return (500 - distance) / 10;
   } else if (distance < 1000) {
     // Less correction for medium-range objects
     return (500 - distance/2) / 20;
   }
   // Minimal correction for distant objects
   return 0;
 }
 
 // Process task - parses LiDAR data and outputs results
 void processTaskCode(void* parameter) {
   char tempBuffer[32];
   
   while (true) {
     uint32_t available = getAvailableBytes();
     
     if (available >= PACKAGE_PAID_BYTES) {
       // Check for valid header
       if (safeReadBuffer(0) == HEADER_BYTE1 && safeReadBuffer(1) == HEADER_BYTE2) {
         uint8_t sampleCount = safeReadBuffer(3);
         uint16_t packetSize = PACKAGE_PAID_BYTES + sampleCount * 3;
         
         if (available >= packetSize) {
           // Calculate checksum
           uint16_t checksumCal = PH;
           
           // Get header values
           checksumCal ^= (safeReadBuffer(3) << 8) | safeReadBuffer(2);
           
           uint16_t firstAngleRaw = safeReadBuffer(4) | (safeReadBuffer(5) << 8);
           uint16_t lastAngleRaw = safeReadBuffer(6) | (safeReadBuffer(7) << 8);
           checksumCal ^= firstAngleRaw;
           checksumCal ^= lastAngleRaw;
           
           // Calculate sample checksum
           uint16_t valu16 = 0;
           for (uint16_t i = 0; i < sampleCount * 3; i++) {
             uint8_t dataByte = safeReadBuffer(PACKAGE_PAID_BYTES + i);
             
             if (i % 3 == 0) {
               valu16 = dataByte;
               checksumCal ^= dataByte;
             } else if (i % 3 == 1) {
               valu16 = dataByte;
             } else {  // i % 3 == 2
               valu16 |= (dataByte << 8);
               checksumCal ^= valu16;
             }
           }
           
           // Verify checksum
           uint16_t packetChecksum = safeReadBuffer(8) | (safeReadBuffer(9) << 8);
           
           if (checksumCal == packetChecksum) {
             packetsProcessed++;
             
             // Extract angles and shift right by 1 (as in original SDK)
             int32_t firstAngle = firstAngleRaw >> 1;
             int32_t lastAngle = lastAngleRaw >> 1;
             
             // Calculate angle increment using integer math
             int32_t angleIncrement = 0;
             if (sampleCount > 1) {
               if (lastAngle < firstAngle) {
                 angleIncrement = (ANGLE_360_UNITS + lastAngle - firstAngle) / (sampleCount - 1);
               } else {
                 angleIncrement = (lastAngle - firstAngle) / (sampleCount - 1);
               }
             }
             
             // Process all samples in the packet
             for (uint8_t i = 0; i < sampleCount; i++) {
               uint16_t offset = PACKAGE_PAID_BYTES + i * 3;
               
               // Extract data
               uint8_t byte0 = safeReadBuffer(offset);
               uint8_t byte1 = safeReadBuffer(offset + 1);
               uint8_t byte2 = safeReadBuffer(offset + 2);
               
               // Calculate distance and quality
               uint16_t distance = (byte2 << 6) | (byte1 >> 2);
               uint8_t quality = ((byte1 & 0x03) << 6) | (byte0 >> 2);
               
               // Apply critical filter from the original SDK
               if (distance <= MIN_VALID_DISTANCE && quality <= MIN_VALID_INTENSITY) {
                 filteredPoints++;
                 continue;  // Skip this point
               }
               
               // Calculate angle with integer math
               int32_t angle = firstAngle + i * angleIncrement;
               
               // Apply integer-based angle correction
               angle += calculateAngleCorrection(distance);
               
               // Normalize angle
               while (angle >= ANGLE_360_UNITS) angle -= ANGLE_360_UNITS;
               while (angle < 0) angle += ANGLE_360_UNITS;
               
               // Convert to whole degrees (integer division)
               uint16_t angleDegrees = angle / ANGLE_UNITS_PER_DEGREE;
               
               // Format output with integer degrees
               int len = sprintf(tempBuffer, "%d,%d,%d\n", angleDegrees, distance, quality);
               
               // Add to batch output
               addToBatch(tempBuffer, len);
               
               // Flush batch if threshold reached
               if (batchPos >= BATCH_THRESHOLD) {
                 flushBatch();
               }
             }
           } else {
             checksumErrors++;
           }
           
           // Advance read position
           readPos.store((readPos.load() + packetSize) & BUFFER_MASK);
         } else {
           // Not enough data yet, wait for more
           vTaskDelay(1);
         }
       } else {
         // Invalid header, advance by 1
         readPos.store((readPos.load() + 1) & BUFFER_MASK);
       }
     } else {
       // Flush any remaining data in batch buffer
       flushBatch();
       vTaskDelay(1);
     }
   }
 }
 
 void setup() {
   Serial.begin(115200);
   while (!Serial) { delay(10); }
   
   // First set the buffer size, THEN begin the serial port
   LIDAR_SERIAL.setRxBufferSize(LIDAR_RX_BUFFER_SIZE);
   LIDAR_SERIAL.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
   
   delay(1000);
   
   // Start LiDAR
   LIDAR_SERIAL.write(START_LIDAR, 4);
   
   // Create dual-core tasks
   xTaskCreatePinnedToCore(
     inputTaskCode,   // Task function
     "InputTask",     // Name
     2048,            // Stack size (bytes)
     NULL,            // Parameters
     2,               // Priority (higher)
     &inputTask,      // Task handle
     0                // Core 0
   );
   
   xTaskCreatePinnedToCore(
     processTaskCode, // Task function
     "ProcessTask",   // Name
     4096,            // Stack size (bytes)
     NULL,            // Parameters
     1,               // Priority
     &processTask,    // Task handle
     1                // Core 1
   );
   
   Serial.println("# Multi-core LiDAR with integer math");
 }
 
 void loop() {
   // Output statistics every 5 seconds
   static uint32_t lastStats = 0;
   if (millis() - lastStats > 5000) {
     Serial.printf("# Packets:%lu Errors:%lu Filtered:%lu\n", 
                  packetsProcessed.load(), checksumErrors.load(), filteredPoints.load());
     lastStats = millis();
   }
   
   // Main loop can be used for other tasks
   delay(1000);
 }
