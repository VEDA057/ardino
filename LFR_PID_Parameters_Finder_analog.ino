#include <PID_v1.h>

// Define pins for L298N motor driver
const byte ENA = 10; // Enable Pin for Motor A
const byte IN1 = 11; // Motor A Input 1
const byte IN2 = 12; // Motor A Input 2
const byte ENB = 5;  // Enable Pin for Motor B
const byte IN3 = 8;  // Motor B Input 1
const byte IN4 = 9;  // Motor B Input 2

// Define pins for IR sensors
const int sensorPins[] = {A0, A1, A2, A3, A4}; // IR sensor pins
const int numSensors = 5;

// Define PID parameters
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;

// Define variables for storing previous errors
double previous_error = 0;
double integral = 0;

// Define PID objects
double input, output, setpoint;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255); // Set the output limits for motor control
  
  // Initialize serial communication
  Serial.begin(9600);

  // Perform dynamic tuning of PID parameters
  dynamicTuning();
}

void loop() {
  // Read sensor inputs and calculate error
  int error = calculateError();
  
  // Update PID setpoint and compute PID output
  setpoint = 0; // Setpoint is the center of the line
  input = error;
  pid.Compute();
  
  // Apply PID output to motor control
  adjustMotors(output);
  
  // Print PID output for debugging
  Serial.print("PID Output: ");
  Serial.println(output);

  // Print PID parameters
  Serial.print("PID Parameters - Kp: ");
  Serial.print(Kp);
  Serial.print(", Ki: ");
  Serial.print(Ki);
  Serial.print(", Kd: ");
  Serial.println(Kd);
  
  // Print sensor values
  Serial.print("Sensor Values: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(analogRead(sensorPins[i]));
    Serial.print("\t");
  }
  Serial.println();
  
  // Print previous error
  Serial.print("Previous Error: ");
  Serial.println(previous_error);

  // Delay for stability
  delay(10);
}

void dynamicTuning() {
  double best_error = 1000000; // Initialize with a large value
  double best_Kp = 0;
  double best_Ki = 0;
  double best_Kd = 0;
  
  // Define tuning range and step size for each PID constant
  double Kp_min = 0.1;
  double Kp_max = 10.0;
  double Ki_min = 0.0;
  double Ki_max = 1.0;
  double Kd_min = 0.0;
  double Kd_max = 0.1;
  double K_step = 0.1;

  for (double kp = Kp_min; kp <= Kp_max; kp += K_step) {
    for (double ki = Ki_min; ki <= Ki_max; ki += K_step) {
      for (double kd = Kd_min; kd <= Kd_max; kd += K_step) {
        pid.SetTunings(kp, ki, kd); // Set PID parameters
        
        double total_error = 0;
        for (int i = 0; i < 1000; i++) { // Run the robot for 1000 iterations and calculate total error
          int error = calculateError();
          setpoint = 0;
          input = error;
          pid.Compute();
          total_error += abs(input); // Access the error directly from the input variable
          delay(10);
        }
        
        // Update best parameters if current PID constants yield lower total error
        if (total_error < best_error) {
          best_error = total_error;
          best_Kp = kp;
          best_Ki = ki;
          best_Kd = kd;
        }
      }
    }
  }
  
  // Set PID with the best parameters found
  Kp = best_Kp;
  Ki = best_Ki;
  Kd = best_Kd;
  pid.SetTunings(Kp, Ki, Kd);
  
  // Print the best PID parameters on the screen
  Serial.print("Best PID Parameters - Kp: ");
  Serial.print(best_Kp);
  Serial.print(", Ki: ");
  Serial.print(best_Ki);
  Serial.print(", Kd: ");
  Serial.println(best_Kd);
}

int calculateError() {
  int total = 0;
  int weighted_sum = 0;

  for (int i = 0; i < numSensors; i++) {
    int sensor_value = analogRead(sensorPins[i]);
    total += sensor_value;
    weighted_sum += (i - (numSensors - 1) / 2) * sensor_value;
  }

  if (total == 0) {
    // All sensors off the line, return last error
    return previous_error;
  }

  // Calculate error
  double error = weighted_sum / total;

  // Update previous error for next iteration
  previous_error = error;

  return error;
}

void adjustMotors(double pid_output) {
  // Adjust motor speeds based on PID output
  int motor_speed = (int)pid_output;

  // Adjust motor speeds based on PID output
  int motor1_speed = motor_speed;
  int motor2_speed = motor_speed;

  // Modify motor speeds based on error sign for differential drive
  if (pid_output > 0) {
    motor1_speed -= pid_output;
  } else {
    motor2_speed += pid_output;
  }

  // Apply motor speeds to motors
  analogWrite(ENA, abs(motor1_speed));
  analogWrite(ENB, abs(motor2_speed));
  digitalWrite(IN1, (motor1_speed >= 0) ? HIGH : LOW);
  digitalWrite(IN2, (motor1_speed >= 0) ? LOW : HIGH);
  digitalWrite(IN3, (motor2_speed >= 0) ? HIGH : LOW);
  digitalWrite(IN4, (motor2_speed >= 0) ? LOW : HIGH);
}
