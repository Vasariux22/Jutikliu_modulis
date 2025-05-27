#pragma once
#include <string.h>

extern volatile bool tcpConnected;
extern int sock;
extern volatile bool wifiRunning;

void startWiFiSoftAP();
bool connectToTCPServer();
void sendPacket(const void *data, size_t len);
void enableWiFi();
void disableWiFi();


