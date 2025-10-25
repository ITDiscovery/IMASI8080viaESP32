/*
 * network_task.cpp
 *
 * Implements the Core 0 network task. Handles WiFi, NTP, and Telnet.
 * All debug output uses Serial.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <time.h>           // For NTP
#include "network_task.h"  // Includes WiFiServer, WiFiClient, extern queue handles
#include "config.h"        // Includes project settings like NETWORK_TRACE

// --- NTP Configuration ---
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = NTP_GMT_OFFSET_SEC;
const int daylightOffset_sec = NTP_DAYLIGHT_OFFSET_SEC;  // Daylight Saving offset = +1 hour

// --- Telnet Configuration ---
#define TELNET_PORT 23
WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient; // Holds the current connected client (only one supported)

// --- Inter-Core Queues (Declared extern in network_task.h) ---
extern QueueHandle_t telnet_to_emu_queue;
extern QueueHandle_t emu_to_telnet_queue;

/**
 * @brief Reads data from the emulator output queue and sends it to Telnet.
 */
void sendEmuOutputToTelnet() {
  if (telnetClient && telnetClient.connected()) {
    uint8_t data_byte;
    // Check if there's data waiting from the emulator
    // Check queue handle validity as well
    if (emu_to_telnet_queue != NULL && uxQueueMessagesWaiting(emu_to_telnet_queue) > 0) {
        #ifdef NETWORK_TRACE // Add trace for attempting to read
        Serial.printf("sendEmuOutputToTelnet: %d bytes waiting in emu_to_telnet_queue.\n", uxQueueMessagesWaiting(emu_to_telnet_queue));
        #endif

        // Read all available bytes without blocking
        while (xQueueReceive(emu_to_telnet_queue, &data_byte, (TickType_t)0) == pdTRUE) {
            #ifdef NETWORK_TRACE // Add trace for successful read and attempted write
            Serial.printf("  -> Read 0x%02X ('%c') from queue. Writing to Telnet client...\n", data_byte, isprint(data_byte) ? data_byte : '.');
            #endif
            // Write the byte to the Telnet client
            size_t written = telnetClient.write(data_byte);
            if (written == 0) {
                 #ifdef NETWORK_TRACE
                 Serial.println("  !!! Telnet client write failed (returned 0) !!!");
                 #endif
                 // If write fails consistently, might indicate client disconnected abruptly
                 // break; // Optionally break the loop on write failure
            }
        }
    }
    // No else needed - if no data or queue invalid, just do nothing this pass
  } else {
      // Optional Trace: Indicate why no send attempt happened (can be noisy)
      #ifdef NETWORK_TRACE
      if (!telnetClient) Serial.println("sendEmuOutputToTelnet: No Telnet client object.");
      else if (!telnetClient.connected()) Serial.println("sendEmuOutputToTelnet: Telnet client not connected.");
      #endif
  }
}


/**
 * @brief Handles incoming Telnet client data and sends it to the emulator input queue.
 */
void handleTelnetClientInput() {
  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    #ifdef NETWORK_TRACE
    Serial.print("handleTelnetClientInput: Data available from Telnet client. Reading: ");
    #endif
    while (telnetClient.available()) {
      char c = telnetClient.read();
      uint8_t data_byte = (uint8_t)c;

      #ifdef NETWORK_TRACE
      Serial.printf(" 0x%02X ('%c')", data_byte, isprint(data_byte) ? data_byte : '.');
      #endif

      // Send the byte to the emulator queue
      // Check queue handle validity
      if (telnet_to_emu_queue != NULL) {
          if (xQueueSend(telnet_to_emu_queue, &data_byte, (TickType_t)(10 / portTICK_PERIOD_MS)) != pdTRUE) {
              #ifdef NETWORK_TRACE
              Serial.print(" [Queue FULL!]");
              #endif
              // Handle queue full error - maybe notify user or drop char?
          }
      } else {
          #ifdef NETWORK_TRACE
          Serial.print(" [Queue Invalid!]");
          #endif
      }
    }
    #ifdef NETWORK_TRACE
    Serial.println(); // Newline after reading all available data
    #endif
  }
}

/**
 * @brief FreeRTOS task for all network management (WiFi, NTP, Telnet).
 *
 * This task is pinned to Core 0 and runs completely independently of
 * the i8080 emulation on Core 1. Output uses Serial.
 */
void wifiTask(void *pvParameters) {
  // Announce which core we are running on
  #ifdef NETWORK_TRACE
  Serial.printf("wifiTask: Task started on Core %d\n", xPortGetCoreID());
  #endif

  // Check if queue handles are valid after task start (they should be if created before task launch)
  if (telnet_to_emu_queue == NULL || emu_to_telnet_queue == NULL) {
      Serial.println("!!! wifiTask FATAL: Inter-core queue handles are NULL! Halting Core 0 task. !!!");
      while(1) { vTaskDelay(1000 / portTICK_PERIOD_MS); } // Stop task execution
  } else {
      #ifdef NETWORK_TRACE
      Serial.println("wifiTask: Inter-core queue handles are valid.");
      #endif
  }


  // 1. Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int connect_tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS); // Use FreeRTOS delay
    Serial.print(".");
    connect_tries++;
    if (connect_tries > 40) { // Timeout after ~20 seconds
        Serial.println("\n!!! wifiTask: WiFi connection timed out! Check SSID/Password. Halting task. !!!");
         while(1) { vTaskDelay(1000 / portTICK_PERIOD_MS); } // Stop task execution
    }
  }
  Serial.print("WiFi Connected. IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.flush(); // Flush after IP

  // 2. Configure NTP
  #ifdef INTER_TIME
  Serial.println("wifiTask: Configuring NTP...");
  #endif
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // 3. Start Telnet Server
  #ifdef NETWORK_TRACE
  Serial.println("wifiTask: --- Attempting to start Telnet server ---");
  #endif
  telnetServer.begin();
  #ifdef NETWORK_TRACE
  Serial.printf("Telnet server started on port %d\n", TELNET_PORT);
  #endif

  // 4. Main Network Loop (Handles NTP, Telnet, WiFi Status)
  struct tm timeinfo;
  unsigned long lastNtpSyncMillis = 0;
  // Reduce initial delay, sync sooner after boot
  const unsigned long initialNtpDelayMillis = 5000;
  const unsigned long ntpSyncIntervalMillis = 3600000; // 1 hour
  const unsigned long ntpRetryIntervalMillis = 60000;  // 1 minute
  unsigned long currentNtpInterval = initialNtpDelayMillis;

  unsigned long lastWifiCheckMillis = 0;
  const unsigned long wifiCheckIntervalMillis = 10000; // 10 seconds

  for (;;) {
    // --- A. Check for new Telnet clients ---
    if (telnetServer.hasClient()) {
      #ifdef NETWORK_TRACE
      Serial.println("wifiTask: telnetServer.hasClient() is TRUE");
      #endif
      if (telnetClient && telnetClient.connected()) {
        Serial.println("wifiTask: Disconnecting previous Telnet client to accept new one.");
        telnetClient.println("\n[Connection closed by server (new client connected)]");
        telnetClient.stop();
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to allow cleanup
      }

      #ifdef NETWORK_TRACE
      Serial.println("wifiTask: Attempting telnetServer.accept()...");
      #endif
      telnetClient = telnetServer.accept();
      #ifdef NETWORK_TRACE
      Serial.printf("wifiTask: telnetServer.accept() returned: %s\n", telnetClient ? "Client object" : "NULL");
      #endif

      if (telnetClient && telnetClient.connected()) {
        #ifdef NETWORK_TRACE
        Serial.print("wifiTask: New Telnet client connected: ");
        Serial.println(telnetClient.remoteIP());
        #endif
        // --- Reset Queues for New Client ---
        if(telnet_to_emu_queue != NULL) xQueueReset(telnet_to_emu_queue);
        if(emu_to_telnet_queue != NULL) xQueueReset(emu_to_telnet_queue);
        #ifdef NETWORK_TRACE
        Serial.println("wifiTask: Inter-core queues reset for new client.");
        #endif
        // --- Send Welcome Message ---
        telnetClient.println("\n------------------------------------");
        telnetClient.println(" Welcome to the IMSAI 8080 Emulator");
        telnetClient.println("------------------------------------");
        telnetClient.printf (" ESP32 IP: %s\n", WiFi.localIP().toString().c_str());
        telnetClient.println("[Use Ctrl+] or Ctrl+D to disconnect]");
        telnetClient.println("------------------------------------");
        // Don't print "> " here, let the emulator send its prompt
      } else {
         Serial.println("wifiTask: Telnet client connection attempt failed immediately after accept().");
         if(telnetClient) { telnetClient.stop(); } // Ensure client object is stopped if invalid
      }
    }

    // --- B. Handle existing Telnet client I/O ---
    // Check for disconnection
    if (telnetClient && !telnetClient.connected()) {
      #ifdef NETWORK_TRACE
      Serial.println("wifiTask: Telnet client disconnected.");
      #endif
      telnetClient.stop();
      // Optionally reset queues on disconnect too?
      // if(telnet_to_emu_queue != NULL) xQueueReset(telnet_to_emu_queue);
      // if(emu_to_telnet_queue != NULL) xQueueReset(emu_to_telnet_queue);
    }

    // Process input from Telnet -> Emulator
    handleTelnetClientInput();

    // Process output from Emulator -> Telnet
    sendEmuOutputToTelnet();


    // --- C. Check if it's time for NTP sync ---
    if (millis() - lastNtpSyncMillis >= currentNtpInterval) {
      #ifdef NETWORK_TRACE
      Serial.println("wifiTask: Attempting NTP sync...");
      #endif
      if (getLocalTime(&timeinfo, 5000)) { // Reduced NTP timeout
        #ifdef NETWORK_TRACE
        Serial.println("\n--- NTP TIME SYNC (CORE 0) ---");
        Serial.print("  Raw Time: ");
        Serial.print(asctime(&timeinfo)); // asctime includes newline
        Serial.println("---------------------------------");
        #endif // NETWORK_TRACE
        currentNtpInterval = ntpSyncIntervalMillis; // Set interval for next sync (1 hour)
        // Optionally send sync status to Telnet (can be noisy)
        // if (telnetClient && telnetClient.connected()) {
        //    telnetClient.print("\n[NTP Sync OK: ");
        //    telnetClient.print(asctime(&timeinfo));
        // }
      } else {
        Serial.println("wifiTask: Failed to obtain NTP time.");
        currentNtpInterval = ntpRetryIntervalMillis; // Retry sooner (1 minute)
        // Optionally send sync status to Telnet
        // if (telnetClient && telnetClient.connected()) {
        //    telnetClient.println("\n[NTP Sync Failed]");
        // }
      }
      lastNtpSyncMillis = millis();
    }

    // --- D. Periodic WiFi Status Check ---
    if (millis() - lastWifiCheckMillis >= wifiCheckIntervalMillis) {
      if (WiFi.status() != WL_CONNECTED) {
         Serial.println("!!! wifiTask: WiFi connection LOST! Attempting reconnect...");
         WiFi.disconnect(); // Try full disconnect/reconnect
         vTaskDelay(100 / portTICK_PERIOD_MS);
         WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      } else {
         // Optional status print
         #ifdef NETWORK_TRACE
         Serial.printf("wifiTask: WiFi Status OK (%s)\n", WiFi.localIP().toString().c_str());
         #endif
      }
      lastWifiCheckMillis = millis();
    }

    // --- E. Yield to other tasks ---
    vTaskDelay(5 / portTICK_PERIOD_MS); // Slightly shorter delay for potentially better responsiveness
  } // end infinite loop
}

/**
 * @brief Public function to launch the network task on Core 0. Called from setup().
 */
void network_task_init() {
  // Use Serial because this is called before Core 0 task starts using Serial exclusively
  Serial.println("network_task_init: Spawning wifiTask on Core 0...");

  // Create the FreeRTOS task
  xTaskCreatePinnedToCore(
      wifiTask,         // Function to implement the task
      "WiFiTask",       // Name of the task
      8192,             // Stack size in words (check ESP32 docs, might be bytes - 8192 bytes is safer)
      NULL,             // Task input parameter
      1,                // Priority (higher numbers are higher priority)
      NULL,             // Task handle (not needed)
      0                 // Core ID (0)
  );

  // Give the task a brief moment to start up - run by Core 1, okay here.
  delay(100);
}

