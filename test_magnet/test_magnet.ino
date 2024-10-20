#include <stdio.h>

#define SWITCH_PIN 2
volatile bool switchState = HIGH; // Store the current switch state
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds
IntervalTimer myTimer;             // Create an interval timer
unsigned long lastDebounceTime = 0; // Timestamp of the last switch state change
volatile bool timerRunning = false;         // Flag to indicate if the timer is running
elapsedMillis sinceLastLoop;

#define PWM_OUT_PIN 3
#define PWM_MAX 255
#define PWM_MIN 0

void handleSwitchChange() {
    unsigned long currentTime = millis(); // Get the current time

    // Check if the debounce delay has passed
    if (currentTime - lastDebounceTime > debounceDelay) {
      startTimer();
      lastDebounceTime = currentTime; // Update the last debounce time
    }
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
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(PWM_OUT_PIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(PWM_OUT_PIN, LOW);
    }  
}

void setup() {
  //Setup switch input pin
  pinMode(SWITCH_PIN, INPUT_PULLUP);     // Set the switch pin as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), startTimer, CHANGE);

  // set LED outputs
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); 

  //set PWM Outputs
  pinMode(PWM_OUT_PIN, OUTPUT);
  digitalWrite(PWM_OUT_PIN, LOW);
}

void loop() {
  //
}
