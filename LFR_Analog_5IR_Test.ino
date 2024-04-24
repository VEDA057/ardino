// Define pins for analog IR sensor array
const byte sensorPin1 = A0; // Sensor 1
const byte sensorPin2 = A1; // Sensor 2
const byte sensorPin3 = A2; // Sensor 3
const byte sensorPin4 = A3; // Sensor 4
const byte sensorPin5 = A4; // Sensor 5

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // IR sensor pins setup
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(sensorPin4, INPUT);
  pinMode(sensorPin5, INPUT);
}

void loop() {
  // Read and print sensor values
  int sensorValue1 = analogRead(sensorPin1);
  int sensorValue2 = analogRead(sensorPin2);
  int sensorValue3 = analogRead(sensorPin3);
  int sensorValue4 = analogRead(sensorPin4);
  int sensorValue5 = analogRead(sensorPin5);

  // Print sensor values to serial monitor
  Serial.print("Sensor 1: ");
  Serial.print(sensorValue1);
  Serial.print("\tSensor 2: ");
  Serial.print(sensorValue2);
  Serial.print("\tSensor 3: ");
  Serial.print(sensorValue3);
  Serial.print("\tSensor 4: ");
  Serial.print(sensorValue4);
  Serial.print("\tSensor 5: ");
  Serial.println(sensorValue5);

  // Wait for a short delay before reading again
  delay(100);
}
