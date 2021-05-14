#pragma once

#include <Arduino.h>
#include <string.h>

void initRS232();

bool hasEvenParity(char x);
char convertSendChar(char raw);
char convertRecvChar(char raw);
int charIndex(char *string, int start, char c);
bool pressureRawRequest(char *request, char responseString[200],
                        char error[100]);
bool readPressureSensor(double *value, double *tempVal, char *pressureType,
                        char error[100]);
bool flowRawRequest(int flowNum, char *request, char responseString[200],
                    char error[100]);
bool readFlow(int flowNum, double values[4], char gas[100], char error[100]);
