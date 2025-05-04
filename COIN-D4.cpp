
/*
 * ESP32 Interface for Coin D4 LiDAR - Ultra-Optimized Version
 * 
 * Additional optimizations:
 * - Dual-core processing (Core 0: Serial input, Core 1: Processing/output)
 * - Lock-free ring buffer with atomic operations
 * - Batch serial output
 * - Custom fast integer-to-string conversion
 * - Compiler optimization hints
 * - Zero-copy processing
 */

 #include <Arduino.h>
 #include <atomic>
 
 // LiDAR control commands
 const uint8_t START_LIDAR[4] = {0xAA, 0x55, 0xF0, 0x0F};
 
 // Serial configuration
 #define LIDAR_SERIAL Serial2
 #define LIDAR_RX_PIN 16
 #define LIDAR_TX_PIN 17
 #define LIDAR_BAUDRATE 230400
 
 // Buffer configuration - must be power of 2
 #define BUFFER_SIZE 4096
 #define BUFFER_MASK (BUFFER_SIZE - 1)
 #define PACKAGE_PAID_BYTES 10
 #define BATCH_SIZE 512  // Batch output buffer
 
 // Protocol constants
 #define HEADER_BYTE1 0xAA
 #define HEADER_BYTE2 0x55
 #define PH 0x55AA
 
 // Pre-calculated constants
 #define ANGLE_SCALE 100
 #define ANGLE_360_SCALED (360 * 64 * ANGLE_SCALE)
 #define ANGLE_DIVISOR 64
 
 // Lock-free circular buffer with atomic operations
 volatile uint8_t buffer[BUFFER_SIZE] __attribute__((aligned(4)));
 std::atomic<uint32_t> writePos(0);
 std::atomic<uint32_t> readPos(0);
 
 // Batch output buffer
 char batchBuffer[BATCH_SIZE];
 uint16_t batchPos = 0;
 
 // Task handles for dual-core operation
 TaskHandle_t inputTask;
 TaskHandle_t processTask;
 
 // Fast integer to string conversion (optimized for our specific use case)
 inline char* fast_itoa(uint32_t value, char* buffer) {
   char* ptr = buffer;
   
   // Handle up to 5 digits (max 65535)
   if (value >= 10000) {
     *ptr++ = '0' + (value / 10000);
     value %= 10000;
   }
   if (value >= 1000) {
     *ptr++ = '0' + (value / 1000);
     value %= 1000;
   }
   if (value >= 100) {
     *ptr++ = '0' + (value / 100);
     value %= 100;
   }
   if (value >= 10) {
     *ptr++ = '0' + (value / 10);
     value %= 10;
   }
   *ptr++ = '0' + value;
   
   return ptr;
 }
 
 // Fast angle formatting (optimized for 0-359 with 2 decimals)
 inline char* format_angle(uint32_t angleHundredths, char* buffer) {
   char* ptr = buffer;
   uint32_t whole = angleHundredths / 100;
   uint32_t frac = angleHundredths % 100;
   
   // Whole part (0-359)
   if (whole >= 100) {
     *ptr++ = '0' + (whole / 100);
     whole %= 100;
   }
   if (whole >= 10 || angleHundredths >= 1000) {
     *ptr++ = '0' + (whole / 10);
     whole %= 10;
   }
   *ptr++ = '0' + whole;
   
   // Decimal point
   *ptr++ = '.';
   
   // Fractional part (always 2 digits)
   *ptr++ = '0' + (frac / 10);
   *ptr++ = '0' + (frac % 10);
   
   return ptr;
 }
 
 // Core 0: High-priority serial input task
 void inputTaskCode(void* parameter) {
   while (true) {
     // Read all available data
     while (LIDAR_SERIAL.available()) {
       uint32_t nextPos = (writePos.load() + 1) & BUFFER_MASK;
       
       // Check for buffer overflow
       if (nextPos != readPos.load()) {
         buffer[writePos.load()] = LIDAR_SERIAL.read();
         writePos.store(nextPos);
       }
     }
     
     // Yield to prevent watchdog timeout
     vTaskDelay(1);
   }
 }
 
 // Flush batch buffer
 inline void flushBatch() {
   if (batchPos > 0) {
     Serial.write(batchBuffer, batchPos);
     batchPos = 0;
   }
 }
 
 // Add to batch buffer
 inline void addToBatch(const char* data, uint16_t len) {
   if (batchPos + len >= BATCH_SIZE - 1) {
     flushBatch();
   }
   memcpy(batchBuffer + batchPos, data, len);
   batchPos += len;
 }
 
 // Core 1: Processing and output task
 void processTaskCode(void* parameter) {
   char tempBuffer[32];
   
   while (true) {
     uint32_t currentReadPos = readPos.load();
     uint32_t currentWritePos = writePos.load();
     
     // Calculate available bytes
     uint32_t available = (currentWritePos - currentReadPos) & BUFFER_MASK;
     
     // Process if we have at least a header
     if (available >= PACKAGE_PAID_BYTES) {
       // Check header
       if (buffer[currentReadPos] == HEADER_BYTE1 && 
           buffer[(currentReadPos + 1) & BUFFER_MASK] == HEADER_BYTE2) {
         
         uint8_t sampleCount = buffer[(currentReadPos + 3) & BUFFER_MASK];
         uint32_t packetSize = PACKAGE_PAID_BYTES + sampleCount * 3;
         
         if (available >= packetSize) {
           // Fast checksum calculation
           uint16_t checksumCal = PH;
           
           // Process header checksum
           checksumCal ^= (buffer[(currentReadPos + 3) & BUFFER_MASK] << 8) | 
                         buffer[(currentReadPos + 2) & BUFFER_MASK];
           
           // Angles
           uint16_t firstAngleRaw = buffer[(currentReadPos + 4) & BUFFER_MASK] | 
                                   (buffer[(currentReadPos + 5) & BUFFER_MASK] << 8);
           uint16_t lastAngleRaw = buffer[(currentReadPos + 6) & BUFFER_MASK] | 
                                  (buffer[(currentReadPos + 7) & BUFFER_MASK] << 8);
           
           checksumCal ^= firstAngleRaw;
           checksumCal ^= lastAngleRaw;
           
           // Sample data checksum
           uint32_t dataStart = (currentReadPos + PACKAGE_PAID_BYTES) & BUFFER_MASK;
           uint16_t valu16 = 0;
           
           for (uint32_t i = 0; i < sampleCount * 3; i++) {
             uint8_t dataByte = buffer[(dataStart + i) & BUFFER_MASK];
             
             switch (i % 3) {
               case 0:
                 valu16 = dataByte;
                 checksumCal ^= dataByte;
                 break;
               case 1:
                 valu16 = dataByte;
                 break;
               case 2:
                 valu16 |= (dataByte << 8);
                 checksumCal ^= valu16;
                 break;
             }
           }
           
           // Verify checksum
           uint16_t packetChecksum = buffer[(currentReadPos + 8) & BUFFER_MASK] | 
                                    (buffer[(currentReadPos + 9) & BUFFER_MASK] << 8);
           
           if (checksumCal == packetChecksum) {
             // Process valid packet
             uint32_t firstAngle = (firstAngleRaw >> 1) * ANGLE_SCALE;
             uint32_t lastAngle = (lastAngleRaw >> 1) * ANGLE_SCALE;
             
             int32_t angleIncrement = 0;
             if (sampleCount > 1) {
               if (lastAngle < firstAngle) {
                 angleIncrement = (ANGLE_360_SCALED + lastAngle - firstAngle) / (sampleCount - 1);
               } else {
                 angleIncrement = (lastAngle - firstAngle) / (sampleCount - 1);
               }
             }
             
             // Process samples with batch output
             for (uint8_t i = 0; i < sampleCount; i++) {
               uint32_t offset = (dataStart + i * 3) & BUFFER_MASK;
               
               // Calculate angle
               uint32_t angleScaled = firstAngle + i * angleIncrement;
               if (angleScaled >= ANGLE_360_SCALED) {
                 angleScaled -= ANGLE_360_SCALED;
               }
               uint32_t angleHundredths = angleScaled / ANGLE_DIVISOR;
               
               // Extract data
               uint8_t byte0 = buffer[offset];
               uint8_t byte1 = buffer[(offset + 1) & BUFFER_MASK];
               uint8_t byte2 = buffer[(offset + 2) & BUFFER_MASK];
               
               uint16_t distance = (byte2 << 6) | (byte1 >> 2);
               uint8_t quality = ((byte1 & 0x03) << 6) | (byte0 >> 2);
               
               // Format output using optimized functions
               char* ptr = tempBuffer;
               ptr = format_angle(angleHundredths, ptr);
               *ptr++ = ',';
               ptr = fast_itoa(distance, ptr);
               *ptr++ = ',';
               ptr = fast_itoa(quality, ptr);
               *ptr++ = '\n';
               
               // Add to batch
               addToBatch(tempBuffer, ptr - tempBuffer);
             }
           }
           
           // Advance read position
           readPos.store((currentReadPos + packetSize) & BUFFER_MASK);
         } else {
           // Not enough data yet
           vTaskDelay(1);
         }
       } else {
         // Invalid header, advance by 1
         readPos.store((currentReadPos + 1) & BUFFER_MASK);
       }
     } else {
       // Flush any pending output
       flushBatch();
       vTaskDelay(1);
     }
   }
 }
 
 void setup() {
   Serial.begin(115200);
   while (!Serial) { delay(10); }
   
   LIDAR_SERIAL.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
   
   // Configure serial buffers for maximum performance
   LIDAR_SERIAL.setRxBufferSize(1024);
   Serial.setTxBufferSize(1024);
   
   delay(1000);
   
   // Start LiDAR
   LIDAR_SERIAL.write(START_LIDAR, 4);
   
   // Create high-priority input task on Core 0
   xTaskCreatePinnedToCore(
     inputTaskCode,   // Task function
     "InputTask",     // Name
     4096,           // Stack size
     NULL,           // Parameters
     2,              // Priority (higher)
     &inputTask,     // Task handle
     0               // Core 0
   );
   
   // Create processing task on Core 1
   xTaskCreatePinnedToCore(
     processTaskCode, // Task function
     "ProcessTask",   // Name
     8192,           // Stack size
     NULL,           // Parameters
     1,              // Priority (normal)
     &processTask,   // Task handle
     1               // Core 1
   );
   
   Serial.println("# LiDAR Ultra-Optimized Started");
 }
 
 void loop() {
   // Main loop can be used for other tasks or left empty
   vTaskDelay(1000);
 }
