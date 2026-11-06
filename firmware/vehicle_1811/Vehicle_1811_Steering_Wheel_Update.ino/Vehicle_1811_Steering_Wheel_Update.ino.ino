// HS-422 diagnostic using the SAME ServoTimer2 library as the vehicle.
// Disconnect traction power and steering linkage before testing.
// Connect HS-422 signal to Uno D3, external regulated 5-6 V power,
// and join the external power ground to Arduino GND.

#include <ServoTimer2.h>
#include <IoStream.h>

ServoTimer2 steeringWheel;

const int SERVO_PIN = 3;

// HS-422 pulse widths
const int LEFT_US   = 1200;
const int CENTER_US = 1500;
const int RIGHT_US  = 1800;

void setup() {

  Serial.begin(9600);

  steeringWheel.attach(SERVO_PIN);
  steeringWheel.write(CENTER_US);

  Serial.println("HS-422 Diagnostic Test");
  Serial.println("Enter L, R, or C:");
}

void loop() {

  if (Serial.available() > 0) {

    // Similar to cin >> command
    char command = Serial.read();

    if (command == 'L' || command == 'l') {

      steeringWheel.write(LEFT_US);

      // Similar to cout
      Serial.println("Turning LEFT");
      Serial.print("Pulse width: ");
      Serial.println(LEFT_US);
    }

    else if (command == 'R' || command == 'r') {

      steeringWheel.write(RIGHT_US);

      Serial.println("Turning RIGHT");
      Serial.print("Pulse width: ");
      Serial.println(RIGHT_US);
    }

    else if (command == 'C' || command == 'c') {

      steeringWheel.write(CENTER_US);

      Serial.println("Returning to CENTER");
      Serial.print("Pulse width: ");
      Serial.println(CENTER_US);
    }
  }
}