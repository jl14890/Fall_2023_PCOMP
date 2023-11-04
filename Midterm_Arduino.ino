// PComp 2023
// Digital Output - Button with Counter and LED Brightness Control

#include <Servo.h>

Servo myservo;
Servo myservo02;

const int buttonPin = 2;  // Button pin
const int pwmPin = 9;
const int motorPin = 10;
const int motorPin02 = 11;



// variables will change:
int buttonState = 0;       // variable for reading the pushbutton state
int lastButtonState = 0;   // last state of the button
int buttonPressCount = 0;  // counter for the number of button presses
int ledBrightness = 0;     // LED brightness

void setup() {
  pinMode(buttonPin, INPUT);  // initialize digital pin 2 (buttonPin) as an input.
  pinMode(pwmPin, OUTPUT);    // initialize digital pin 9 (ledPin) as an output for PWM.
  pinMode(13, OUTPUT);

  pinMode(pwmPin, OUTPUT);  // initialize PWM pin 10 as an output.
  pinMode(buttonPin, INPUT);

  Serial.begin(9600);  // initialize serial communication at 9600 bits per second

  myservo.attach(motorPin);  // attaches the servo on pin 10 to the servo object
  myservo.write(0);          // starts servo on 0 degrees

  myservo02.attach(motorPin02);
  myservo02.write(0);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check for button press (i.e., the button's state has changed from LOW to HIGH):
  if (buttonState == HIGH && lastButtonState == LOW) {
    buttonPressCount++;  // increment the button press counter
    Serial.println("Button pressed! Total count: " + String(buttonPressCount));

    ledBrightness = min(buttonPressCount * 2.55, 255);
    analogWrite(pwmPin, ledBrightness);

    // Serial.println(ledBrightness);


    // Servo logic for flapping wings:
    myservo.write(40);
    myservo02.write(140);
    delay(100);

    int currentAngle = myservo.read();
    Serial.println("Servo 1 angle: " + String(currentAngle));

    myservo.write(60);
    myservo02.write(120);
    delay(100);

    int currentAngle2 = myservo.read();
    Serial.println("Servo 1 angle: " + String(currentAngle2));
    // Second Time
    // myservo.write(140);
    // myservo02.write(30);
    // delay(1000);

    // myservo.write(0);
    // myservo02.write(150);
    // delay(1000);
  }


  //green led indicator
  if (buttonState == HIGH) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }



  lastButtonState = buttonState;  // save the current button state for comparison next time

  delay(50);  // add a short delay to prevent reading noise as multiple button presses
}
