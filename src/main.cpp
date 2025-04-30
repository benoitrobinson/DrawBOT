#include <Arduino.h>

// User LEDs
#define LEDU1 25
#define LEDU2 26

// Motor pins (from your project documentation)
#define EN_D 23   // Enable right motor
#define EN_G 4    // Enable left motor
#define IN_1_D 19 // Right motor control 1
#define IN_2_D 18 // Right motor control 2
#define IN_1_G 17 // Left motor control 1
#define IN_2_G 16 // Left motor control 2

// Motor speed constants

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Drawbot - Motor Test");

  // Initialize LED pins
  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  // Initialize motor pins
  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);
  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);

  // Initially turn off motors
  digitalWrite(EN_D, LOW);
  digitalWrite(EN_G, LOW);
}

// Function to control the right motor
void rightMotor(int speed)
{
  // Enable motor
  digitalWrite(EN_D, HIGH);

  if (speed > 0)
  {
    // Forward
    digitalWrite(IN_1_D, HIGH);
    digitalWrite(IN_2_D, LOW);
  }
  else if (speed < 0)
  {
    // Backward
    digitalWrite(IN_1_D, LOW);
    digitalWrite(IN_2_D, HIGH);
  }
  else
  {
    // Stop
    digitalWrite(IN_1_D, LOW);
    digitalWrite(IN_2_D, LOW);
  }
}

// Function to control the left motor
void leftMotor(int speed)
{
  // Enable motor
  digitalWrite(EN_G, HIGH);

  if (speed > 0)
  {
    // Forward
    digitalWrite(IN_1_G, HIGH);
    digitalWrite(IN_2_G, LOW);
  }
  else if (speed < 0)
  {
    // Backward
    digitalWrite(IN_1_G, LOW);
    digitalWrite(IN_2_G, HIGH);
  }
  else
  {
    // Stop
    digitalWrite(IN_1_G, LOW);
    digitalWrite(IN_2_G, LOW);
  }
}

// Function to stop both motors
void stopMotors()
{
  digitalWrite(EN_D, LOW);
  digitalWrite(EN_G, LOW);
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, LOW);
}

void loop()
{
  // Indicate start of test with LEDs
  digitalWrite(LEDU1, HIGH);
  digitalWrite(LEDU2, HIGH);

  // Test sequence with simple movements
  Serial.println("Moving forward");
  rightMotor(1); // Forward
  leftMotor(1);  // Forward
  delay(2000);   // Run for 2 seconds

  Serial.println("Stopping");
  stopMotors();
  delay(1000); // Pause for 1 second

  Serial.println("Turning right");
  rightMotor(-1); // Backward
  leftMotor(1);   // Forward
  delay(1000);    // Turn for 1 second

  Serial.println("Stopping");
  stopMotors();
  delay(1000); // Pause for 1 second

  Serial.println("Turning left");
  rightMotor(1); // Forward
  leftMotor(-1); // Backward
  delay(1000);   // Turn for 1 second

  Serial.println("Stopping");
  stopMotors();
  delay(1000); // Pause for 1 second

  Serial.println("Moving backward");
  rightMotor(-1); // Backward
  leftMotor(-1);  // Backward
  delay(2000);    // Run for 2 seconds

  Serial.println("Test complete, waiting 5 seconds...");
  stopMotors();

  // Blink LEDs during wait
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LEDU1, HIGH);
    digitalWrite(LEDU2, LOW);
    delay(500);
    digitalWrite(LEDU1, LOW);
    digitalWrite(LEDU2, HIGH);
    delay(500);
  }
}