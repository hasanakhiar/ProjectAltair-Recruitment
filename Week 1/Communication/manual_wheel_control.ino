const int NUM_CHANNELS = 8;
int channels[NUM_CHANNELS];

void readChannel() {

  static char buffer[100]; 
  static int bufferIndex = 0;
  static bool receivingPacket = false;

  while (Serial1.available() > 0) {
    char incomingChar = Serial1.read();


    if (incomingChar == '<') {
      bufferIndex = 0; 
      receivingPacket = true;
    } 

    else if (incomingChar == '>' && receivingPacket) {
      buffer[bufferIndex] = '\0'; 
      receivingPacket = false;


      char* value = strtok(buffer, " "); 
      int channelCount = 0;


      while (value != NULL && channelCount < NUM_CHANNELS) {
        channels[channelCount] = atoi(value); 
        value = strtok(NULL, " "); 
        channelCount++;
      }

    } 

    else if (receivingPacket) {
      // Prevent buffer overflow
      if (bufferIndex < sizeof(buffer) - 1) {
        buffer[bufferIndex++] = incomingChar;
      }
    }
  }
}


void setup() {

  Serial.begin(115200); 
  Serial1.begin(115200);

}

void loop() {
  readChannel();
    for (int i = 0; i < NUM_CHANNELS; i++) {
      Serial.print(channels[i]);
      if (i < NUM_CHANNELS - 1) Serial.print(", ");
    }
    Serial.println();

  delay(10); 
}