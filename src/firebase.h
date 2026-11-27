#pragma once
#include <Arduino.h>

bool wifiConnect(uint32_t timeoutMs=20000);
bool firebaseBegin();                        // Anonymous sign-in + token storage
bool firebaseReady();                        // Reconnect and refresh as needed
String firebaseUID();                        // Anonymous UID for database path
int firebasePut(const String& path, const String& json); // HTTP PUT (204 success)
int firebaseGet(const String& path, String& response);  // HTTP GET (200 success)
