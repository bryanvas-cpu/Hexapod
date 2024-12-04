#include <SCServo.h>

// Create an SMS_STS object
SMS_STS sms_sts;
#define S_RXD 18
#define S_TXD 19

// Servo IDs and number of servos
const int num_servos = 18;
int servo_ids[num_servos] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

// Communication parameters
const int baud_rate = 115200;       // Serial communication with PC/ROS
const int servo_baud_rate = 1000000; // Communication with servos
String incoming_message = "";       // Stores the incoming string from ROS
String feedback = "";
// Servo states for feedback
float positions[num_servos] = {0.0}; // Current servo positions
float velocities[num_servos] = {0.0}; // Current servo velocities

void setup() {
  // Start communication with PC/ROS
  Serial.begin(baud_rate);

  // Start communication with the SCServo controller
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);// Use Serial2 for communication with SCServo
  sms_sts.pSerial = &Serial1;
  delay(1000);

  Serial.println("SMS_STS initialized and ready.");
}

void loop() {
  // Check for incoming commands from ROS
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { // End of command
      Serial.print("PROCESSING");
      processCommand(incoming_message);
      Serial.print("DONE PROCESSING");
      incoming_message = ""; // Clear the message buffer
    } else {
      incoming_message += c;
    }
  }

  // Send feedback to ROS periodically
  static unsigned long last_feedback_time = 0;
  if (millis() - last_feedback_time > 100) { // 100 ms interval
    sendFeedback();
    last_feedback_time = millis();
  }
}

// Process the incoming command string
void processCommand(const String &command) {
  // Split the command into individual values
  float positions_cmd[num_servos];
  int index = 0;

  int start = 0;
  int end = command.indexOf(',');

  while (end != -1 && index < num_servos) {
    positions_cmd[index] = command.substring(start, end).toFloat();
    Serial.println(positions_cmd[index]);
    start = end + 1;
    end = command.indexOf(',', start);
    index++;
  }

  // If the last value is not followed by a comma
  if (index < num_servos && start < command.length()) {
    positions_cmd[index] = command.substring(start).toFloat();
    index++;
  }

  // Move servos to the specified positions
  for (int i = 0; i < num_servos; i++) {
    int target_position = map(positions_cmd[i], 0, 6.283185, 0, 4095); // Convert degrees to SCServo range
    Serial.print(target_position);
    Serial.print(" \n");
    sms_sts.WritePosEx(servo_ids[i], target_position, 1200, 50);    // Write position with speed=100
  }
}

// Send servo feedback to ROS
void sendFeedback() {
  feedback = ""; // Reset feedback string

  for (int i = 0; i < num_servos; i++) {
    // Read position and speed for the current servo
    int pos = sms_sts.ReadPos(servo_ids[i]);
    int speed = sms_sts.ReadSpeed(servo_ids[i]);

    // Check if position and speed are valid (not -1, which indicates failure)
    if (pos != -1 && speed != -1) {
      // Convert position and speed to radians
      positions[i] = pos * (2 * 3.14159 / 4095.0); // Position in radians
      velocities[i] = speed * (2 * 3.14159 / 4095.0); // Speed in radians per second

      // positions[i] = pos ; // Position in radians
      // velocities[i] = speed ;

      // Add position and velocity to feedback string
      feedback += String(positions[i], 2); // Position with 2 decimal places
      feedback += ",";
      feedback += String(velocities[i], 2); // Velocity with 2 decimal places
    } else {
      // Handle read error by appending "ERR" for both position and velocity
      feedback += "ERR,ERR";
    }

    // Add a comma separator if not the last servo
    if (i < num_servos - 1) {
      feedback += ",";
    }
  }

  // Add newline to indicate end of feedback
  feedback += "\n";

  // Send feedback to the serial monitor or another connected system
  Serial.print(feedback);
}

