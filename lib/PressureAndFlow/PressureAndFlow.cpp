#include "PressureAndFlow.h"

void initRS232() {
  Serial1.begin(9600);  // Flow 1 (Stack) // COMMENTED THIS LINE OUT TO TEST MAIN HSK CONNECTION-KEITH
  Serial5.begin(9600);  // Flow 2 (Shield)
  Serial2.begin(19200); // Pressure
}

bool hasEvenParity(char x) {
  unsigned int count = 0, i;
  char b = 0x01;

  for (i = 0; i < 8; i++) {
    if (x & (b << i)) {
      count++;
    }
  }

  if ((count % 2)) {
    return false;
  }
  return true;
}

char convertSendChar(char raw) {
  if (hasEvenParity(raw)) {
    return raw;
  }
  return raw + 0x80;
}

char convertRecvChar(char raw) {
  if (raw > 0x80) {
    return raw - 0x80;
  }
  return raw;
}

int charIndex(char *string, int start, char c) {
  for (int i = start; i < strlen(string); i++) {
    if (string[i] == c) {
      return i;
    }
  }
  return -1;
}

bool pressureRawRequest(char *request, char responseString[200],
                        char error[100]) {
  int rindex = 0;
  int timeout = 0;
  char chunk;

  while (Serial2.available()) { // Flush
    Serial2.read();
  }
  for (int k = 0; k < strlen(request); k++) {
    Serial2.write(convertSendChar(request[k]));
  }
  while (!Serial2.available()) {
    if (timeout > 2000) {
      strcpy(error, "Timeout");
      return false;
    }
    timeout += 10;
    delay(10);
  }
  // Serial.println("DEBUG: It was available, moving to reading...");
  if (Serial2.available()) {
    delay(100);
    responseString[0] = '\0';
    rindex = 0;
    while (Serial2.available()) {
      chunk = convertRecvChar(Serial2.read());
      responseString[rindex] = chunk;
      responseString[rindex + 1] = '\0';
      rindex++;
      if (strlen(responseString) > 198) {
        strcpy(error, "Bad read: Too long");
        return false;
      }
    }
  } // else {

  // Serial.println("DEBUG Pressure: Failed to get a second available...");
  //}
  // Check to make sure the read was terminated correctly
  if (!(responseString[strlen(responseString) - 1] == '\n') ||
      !(responseString[strlen(responseString) - 2] == '\r')) {
    // Serial.print("DEBUG Pressure: BAD READ: ");
    // Serial.println(responseString);
    strcpy(error, "Bad read");
    return false;
  }
  if (!(responseString[strlen(responseString) - 3] == 0x06)) {
    // Serial.println("DEBUG Pressure: Did not receive ACK");
    // Serial.print("DEBUG: ");
    // Serial.println(responseString);
    strcpy(error, "Bad read: Did not receive ACK");
    return false;
  }
  // Serial.print("DEBUG Pressure: Final char is ");
  // Serial.println(responseString[strlen(responseString)-3], HEX);
  responseString[strlen(responseString) - 2] = '\0';
  return true;
}

bool readPressureSensor(double *value, double *tempVal, char *pressureType,
                        char error[100]) {
  char responseString[200];
  char request[20] = "#01PS\r\n";
  char selection[9];
  bool retval;
  retval = pressureRawRequest(request, responseString, error);
  if (!retval) {
    return retval;
  }
  // Start Parsing
  for (int i = 2; i < 11; i++) {
    selection[i - 3] = responseString[i];
  }

  selection[8] = '\0';
  *value = atof(selection);

  strcpy(request, "#01PT\r\n");
  retval = pressureRawRequest(request, responseString, error);
  *pressureType = responseString[3];
  if (!retval) {
    return retval;
  }

  strcpy(request, "#01ST\r\n");
  retval = pressureRawRequest(request, responseString, error);
  if (!retval) {
    return retval;
  }
  // Start Parsing
  for (int i = 3; i < 11; i++) {
    selection[i - 3] = responseString[i];
  }
  selection[8] = '\0';
  *tempVal = atof(selection);

  return true;
}

bool flowRawRequest(int flowNum, char *request, char responseString[200],
                    char error[100]) {
  int rindex = 0;
  int timeout = 0;
  char chunk;

  // Read in from meter
  if (flowNum == 1) {
    while (Serial1.available()) { // Flush
      Serial1.read();
    }
    Serial1.write(request);
    while (!Serial1.available()) {
      if (timeout > 2000) {
        strcpy(error, "Timeout");
        return false;
      }
      timeout += 10;
      delay(10);
    }
    // Serial.println("DEBUG: It was available, moving to reading...");
    if (Serial1.available()) {
      delay(100);
      responseString[0] = '\0';
      rindex = 0;
      while (Serial1.available()) {
        chunk = Serial1.read();
        responseString[rindex] = chunk;
        responseString[rindex + 1] = '\0';
        rindex++;
        if (strlen(responseString) > 198) {
          strcpy(error, "Bad read: Too long");
          return false;
        }
      }
      // Serial.print("DEBUG: Response is: ");
      // Serial.println(responseString);
    } // else {
      // Serial.println("DEBUG: Failed to get a second available...");
      //}
  } else if (flowNum == 2) {
    while (Serial5.available()) { // Flush
      Serial5.read();
    }
    Serial5.write(request);
    while (!Serial5.available()) {
      if (timeout > 2000) {
        strcpy(error, "Timeout");
        return false;
      }
      timeout += 10;
      delay(10);
    }
    // Serial.println("DEBUG: It was available, moving to reading...");
    if (Serial5.available()) {
      delay(100);
      responseString[0] = '\0';
      rindex = 0;
      while (Serial5.available()) {
        char chunk = Serial5.read();
        responseString[rindex] = chunk;
        responseString[rindex + 1] = '\0';
        rindex++;
        if (strlen(responseString) > 198) {
          strcpy(error, "Bad read: Too long");
          return false;
        }
      }
      // Serial.print("DEBUG: Response is: ");
      // Serial.println(responseString);
    } // else {
      // Serial.println("DEBUG: Failed to get a second available...");
      //}
  } else {
    strcpy(error, "Invalid Flowmeter");
    return false;
  }
  // Check to make sure the read was terminated correctly
  if (!(responseString[strlen(responseString) - 1] == '\r')) {
    // Serial.print("DEBUG: BAD READ: ");
    // Serial.println(responseString);
    strcpy(error, "Bad read");
    return false;
  }
  responseString[strlen(responseString) - 1] = '\0';
  return true;
}

bool readFlow(int flowNum, double values[4], char gas[100], char error[100]) {
  char responseString[200];
  char request[20];
  int prior_index;
  int index = -1;
  char selection[50];
  bool retval;
  if (flowNum == 1) {
    strcpy(request, "A\r");
  } else if (flowNum == 2) {
    strcpy(request, "B\r");
  } else {
    strcpy(error, "Invalid Flowmeter");
    return false;
  }
  retval = flowRawRequest(flowNum, request, responseString, error);
  if (!retval) {
    return retval;
  }
  // Start Parsing
  index = charIndex(responseString, 0, ' ');
  for (int i = 0; i < 4; i++) {
    prior_index = index;
    index = charIndex(responseString, index + 1, ' ');
    if (index == -1) {
      strcpy(error, "Bad Response: Unable to parse");
      return false;
    } else {
      for (int j = prior_index + 1; j < index; j++) {
        selection[j - (prior_index + 1)] = responseString[j];
      }
      selection[index - (prior_index)] = '\0';
      // Serial.print("DEBUG: Selection ");
      // Serial.print(i);
      // Serial.print(": ");
      // Serial.println(selection);
      values[i] = atof(selection);
    }
  }
  while (responseString[index + 1] == ' ') {
    index++;
  }
  for (int j = index + 1; j < strlen(responseString); j++) {
    gas[j - (index + 1)] = responseString[j];
  }
  gas[strlen(responseString) - (index + 1)] = '\0';
  // Serial.print("DEBUG: Gastype: '");
  // Serial.print(gas);
  // Serial.println("' ");
  error = "None";
  return true;
}
