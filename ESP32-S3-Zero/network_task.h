/*
 * wifi.h
 *
 * Manages all network connectivity (WiFi, NTP, Telnet, Web)
 * using a dedicated FreeRTOS task on Core 0.
 */

#ifndef NETWORK_TASK_H
#define NETWORK_TASK_H

#include <WiFiServer.h> // Needed for Telnet server
#include <WiFiClient.h> // Needed for Telnet client

/**
 * @brief Initializes and spawns the main network task (wifiTask) on Core 0.
 *
 * This function is called from setup() on Core 1, but the task it
 * creates will run independently on Core 0.
 */
void network_task_init();

#endif // NETWORK_TASK_H