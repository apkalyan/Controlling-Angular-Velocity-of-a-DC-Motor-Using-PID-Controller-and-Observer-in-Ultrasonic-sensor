// Pins for ultrasonic sensor
const int trigPin = 9;
const int echoPin = 10;

// Kalman filter variables
float Q = 0.1;  // Process noise covariance
float R = 0.1;  // Measurement noise covariance
float x_est_last = 0;  // Last estimate
float P_last = 1;      // Last error covariance

float K;  // Kalman gain
float P;  // Current error covariance
float x_est;  // Current estimate
float P_temp;
float z_measured;  // Measured distance
float y;  // Innovation (measurement - prediction)

long duration;
float distance;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Setup pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo time and convert to distance
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;  // Speed of sound: 343 m/s, divide by 2 for round trip
  
  // Measured distance
  z_measured = distance;
  
  // Prediction update
  x_est = x_est_last;
  P = P_last + Q;
  
  // Measurement update
  K = P / (P + R);  // Kalman gain
  x_est = x_est + K * (z_measured - x_est);  // Update estimate with measurement
  P = (1 - K) * P;  // Update error covariance
  
  // Save last estimate and error covariance
  x_est_last = x_est;
  P_last = P;
  
  // Print results for Serial Plotter
  Serial.print("Measured Distance: ");
  Serial.print(z_measured);
  Serial.print(" cm, Kalman Filtered Distance: ");
  Serial.print(x_est);
  Serial.println(" cm");
  
  // Output for Serial Plotter
  Serial.print("Measured:");
  Serial.print(z_measured);
  Serial.print("\tKalman:");
  Serial.println(x_est);

  delay(100);  // Delay for stability
}
