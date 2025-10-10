// Define the number of channels we expect
const int NUM_CHANNELS = 8;
// This array will hold the final channel values
int channels[NUM_CHANNELS];

/**
 * @brief Reads incoming serial data from Serial1 to parse channel values.
 * * This function handles incomplete data streams by using static variables 
 * to maintain its state across multiple calls from the main loop().
 * It reads characters into a buffer, waiting for a complete packet 
 * defined by start ('<') and end ('>') markers. Once a full packet is 
 * received, it parses the string to populate the global 'channels' array.
 */
void readChannel() {
  // Static variables to preserve state between function calls
  static char buffer[100]; // Buffer to store incoming data, size should be sufficient
  static int bufferIndex = 0;
  static bool receivingPacket = false;

  // Process all available characters in the serial buffer
  while (Serial1.available() > 0) {
    char incomingChar = Serial1.read();

    // Check for the start character
    if (incomingChar == '<') {
      bufferIndex = 0; // Reset buffer
      receivingPacket = true;
    } 
    // Check for the end character while we are in a packet
    else if (incomingChar == '>' && receivingPacket) {
      buffer[bufferIndex] = '\0'; // Null-terminate the string
      receivingPacket = false;

      // Packet is complete, now parse it.
      // We use a temporary pointer for strtok to walk through the string.
      char* value = strtok(buffer, " "); 
      int channelCount = 0;

      // Loop through the string, splitting it by spaces
      while (value != NULL && channelCount < NUM_CHANNELS) {
        channels[channelCount] = atoi(value); // Convert string part to integer
        value = strtok(NULL, " "); // Get the next part
        channelCount++;
      }
      
      // Optional: You can print the received values for debugging
      /*
      Serial.print("Received values: ");
      for (int i = 0; i < NUM_CHANNELS; i++) {
        Serial.print(channels[i]);
        Serial.print(" ");
      }
      Serial.println();
      */

    } 
    // If we are in a packet, add the character to the buffer
    else if (receivingPacket) {
      // Prevent buffer overflow
      if (bufferIndex < sizeof(buffer) - 1) {
        buffer[bufferIndex++] = incomingChar;
      }
    }
  }
}

// Your standard Arduino setup and loop functions
void setup() {
  // Start serial communication for debugging output (to computer)
  Serial.begin(115200); 
  // Start serial communication for reading from the other Arduino
  Serial1.begin(115200);
}

void loop() {
  // Call the function repeatedly to check for and process incoming data
  readChannel();
  
  // You can now use the values in the 'channels' array for motor control, etc.
  // For example:
  // int throttle = channels[0];
  // int steering = channels[1];
  
  delay(10); 
}