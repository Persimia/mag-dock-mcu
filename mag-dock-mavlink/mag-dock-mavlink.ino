#include <MAVLink.h>

#define SWITCH_PIN 2
#define MAGNET_PIN 3

volatile bool switchState = HIGH; // Store the current switch state
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds
IntervalTimer myTimer;             // Create an interval timer
unsigned long lastDebounceTime = 0; // Timestamp of the last switch state change
volatile bool timerRunning = false; // Flag to indicate if the timer is running
elapsedMillis sinceLastLoop;

IntervalTimer heartbeatTimer;             // Create an interval timer
unsigned long heartbeatTimer_ms = 1000; // Debounce delay in milliseconds

// MAVLink variables
#define MAVLINK_SYS_ID 1
#define MAVLINK_COMP_ID 2
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

// Function declarations
void send_attached_msg();
void send_detached_msg();

// Function to send MAVLink heartbeat (or another custom message)
void send_mavlink_heartbeat() {
  mavlink_msg_heartbeat_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
  // mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "att_st", (float)!switchState);
  // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Serial2.write(buf, len);
}

void send_attached_msg() {
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "att_st", 1.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

void send_detached_msg() {
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "att_st", 0.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

// Interrupt service routine for starting the timer
void startTimer() {
  // Start the timer if it's not already running
  if (!timerRunning) {
    myTimer.begin(timerCallback, debounceDelay * 1000); // Convert to microseconds
    timerRunning = true;
  }
}

// Timer callback function
void timerCallback() {
  // Stop the timer after handling the switch state
  myTimer.end();
  timerRunning = false;

  // Read the state of the switch
  switchState = digitalRead(SWITCH_PIN);
  if (!switchState) {
    send_attached_msg();  // Send attach message via MAVLink
  } else {
    send_detached_msg();  // Send detach message via MAVLink
  }
}

void setup() {
  // Setup switch input pin
  pinMode(SWITCH_PIN, INPUT_PULLUP);     // Set the switch pin as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), startTimer, CHANGE);

  // Set LED outputs
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Set PWM outputs
  pinMode(MAGNET_PIN, OUTPUT);
  digitalWrite(MAGNET_PIN, LOW);

  // Initialize serial communication for MAVLink
  Serial2.begin(115200);

  // Setup heartbeat timer
  heartbeatTimer.begin(send_mavlink_heartbeat, heartbeatTimer_ms * 1000); // Convert to microseconds
}

void loop() {
  mavlink_message_t received_msg;
  mavlink_status_t status;
  // Check for incoming MAVLink messages
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    // Try to parse a message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg, &status)) {
      // Handle message by checking its ID
      switch (received_msg.msgid) {
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
          // Handle attach or detach commands (can be expanded)
          mavlink_named_value_float_t msg_struct;
          mavlink_msg_named_value_float_decode(&received_msg, &msg_struct);
          if (strcmp(msg_struct.name, "attach") == 0) {
              if (abs(msg_struct.value)>0.01){ // attach is nonzero, we want to attach
                digitalWrite(LED_BUILTIN, HIGH);  
                digitalWrite(MAGNET_PIN, HIGH);
              }
              else { // attach is zero, we want to detach
                digitalWrite(LED_BUILTIN, LOW);  
                digitalWrite(MAGNET_PIN, LOW);
              }
          }
          break;
        default:
          break;
      }
    }
  }
}
