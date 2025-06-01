#include <Arduino.h>

// User LEDs
#define LEDU1 25 // This LED will indicate when the left motor is running
#define LEDU2 26 // This LED will indicate when the right motor is running

// Motor pins
#define EN_G 4    // Enable left motor
#define EN_D 23   // Enable right motor
#define IN_1_G 17 // Left motor control 1
#define IN_2_G 16 // Left motor control 2
#define IN_1_D 19 // Right motor control 1
#define IN_2_D 18 // Right motor control 2

// PWM configurations
#define PWM_FREQ 5000    // PWM frequency in Hz
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define LEFT_MOTOR_CH 0  // PWM channel for left motor
#define RIGHT_MOTOR_CH 1 // PWM channel for right motor

// Encoder pins
#define ENC_G_CH_A 32 // Left encoder channel A
#define ENC_G_CH_B 33 // Left encoder channel B
#define ENC_D_CH_A 27 // Right encoder channel A
#define ENC_D_CH_B 14 // Right encoder channel B

// Variables for encoder readings
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long lastLeftEncoderCount = 0;
long lastRightEncoderCount = 0;

// Interrupt service routines for encoders
void IRAM_ATTR leftEncoderISR()
{
  // If channel B is high when channel A rises, count up, otherwise count down
  if (digitalRead(ENC_G_CH_B))
  {
    leftEncoderCount++;
  }
  else
  {
    leftEncoderCount--;
  }
}

void IRAM_ATTR rightEncoderISR()
{
  // Invert the count direction for the right motor to match the physical direction
  if (digitalRead(ENC_D_CH_B))
  {
    rightEncoderCount--; // Inverted from original
  }
  else
  {
    rightEncoderCount++; // Inverted from original
  }
}

// Motor speed constants

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Drawbot Motor Test with Encoder Feedback");

  // Initialize LED pins
  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  // Initialize motor pins
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);
  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);

  // Configure PWM for motor speed control
  ledcSetup(LEFT_MOTOR_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_MOTOR_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(EN_G, LEFT_MOTOR_CH);
  ledcAttachPin(EN_D, RIGHT_MOTOR_CH);

  // Initialize encoder pins as inputs with pull-up resistors
  pinMode(ENC_G_CH_A, INPUT_PULLUP);
  pinMode(ENC_G_CH_B, INPUT_PULLUP);
  pinMode(ENC_D_CH_A, INPUT_PULLUP);
  pinMode(ENC_D_CH_B, INPUT_PULLUP);

  // Attach interrupts to encoder channels
  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), rightEncoderISR, RISING);

  // Initially turn off motors and LEDs
  ledcWrite(LEFT_MOTOR_CH, 0);  // 0 speed
  ledcWrite(RIGHT_MOTOR_CH, 0); // 0 speed
  digitalWrite(LEDU1, LOW);
  digitalWrite(LEDU2, LOW);

  // Reset encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Wait a moment before starting
  delay(2000);
  Serial.println("Starting test...");
}

// Function to print encoder information
void printEncoderInfo()
{
  // Check if encoder counts have changed
  if (leftEncoderCount != lastLeftEncoderCount || rightEncoderCount != lastRightEncoderCount)
  {
    // Print left encoder information
    Serial.print("Left encoder: ");
    Serial.print(leftEncoderCount);
    float leftRotations = (float)leftEncoderCount / 12.0 / 150.0;
    Serial.print(" (");
    Serial.print(leftRotations, 3);
    Serial.print(" rotations)  |  ");

    // Print right encoder information
    Serial.print("Right encoder: ");
    Serial.print(rightEncoderCount);
    float rightRotations = (float)rightEncoderCount / 12.0 / 150.0;
    Serial.print(" (");
    Serial.print(rightRotations, 3);
    Serial.println(" rotations)");

    // Update last counts
    lastLeftEncoderCount = leftEncoderCount;
    lastRightEncoderCount = rightEncoderCount;
  }
}

void loop()
{
  // Test both wheels simultaneously
  digitalWrite(LEDU1, HIGH); // Turn on left LED
  digitalWrite(LEDU2, HIGH); // Turn on right LED
  Serial.println("\n--- Both wheels moving forward ---");

  // Reset encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Set both motors to move forward
  digitalWrite(IN_1_G, HIGH);
  digitalWrite(IN_2_G, LOW);
  // Reverse the direction for the right motor
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, HIGH);

  // Enable both motors with reduced speed (100 out of 255)
  ledcWrite(LEFT_MOTOR_CH, 186);  // Adjust this value to control speed (0-255)
  ledcWrite(RIGHT_MOTOR_CH, 180); // Adjust this value to control speed (0-255)

  // Keep the motors running for 3 seconds while monitoring encoders
  unsigned long startTime = millis();
  while (millis() - startTime < 3000)
  {
    printEncoderInfo();
    delay(50);
  }

  // Stop both motors and turn off LEDs
  ledcWrite(LEFT_MOTOR_CH, 0);  // 0 speed
  ledcWrite(RIGHT_MOTOR_CH, 0); // 0 speed
  digitalWrite(LEDU1, LOW);
  digitalWrite(LEDU2, LOW);
  Serial.println("Both wheels stopped");

  // Display final encoder readings
  Serial.println("\nFinal encoder readings:");
  printEncoderInfo();

  // Wait before repeating the test
  Serial.println("Waiting for 5 seconds before repeating...");
  delay(5000);
}
