

#define Q1 2 // Q1 pin number
#define Q2 9 // Q2 pin number
#define intPin 5 // interupt pin used to record time used in the interupt
#define pot A0 // potentiometer pin
#define Mot1 10 // pwm Pin
#define dirMot1 12 //Direction of pin 1

#define encoderRes 500 // measured value

volatile long encoderCount = 0; // Initialize encoder count as a global variable
unsigned long timer = 0;
bool first = false;

void encoderISR() {
  digitalWrite(intPin, HIGH);
  if (digitalRead(Q2) == HIGH) {
    encoderCount--; // Count up if pin Q2 is also high
  }
  else {
    encoderCount++; // Count down if pin Q2 is low
  }
  digitalWrite(intPin, LOW);
}


void setup() {
  // put your setup code here, to run once:
 
  Serial.begin(9600);
  pinMode(pot,INPUT);
  pinMode(Q1, INPUT_PULLUP); // Set pin Q1 as input with pull-up resistor
  pinMode(Q2, INPUT_PULLUP); // Set pin Q2 as input with pull-up resistor
  pinMode(Mot1,OUTPUT);
  pinMode(dirMot1,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Q1), encoderISR, RISING); // Attach interrupt to pin Q14

  timer = millis();
}

void loop() {
  int potValue = analogRead(A0); // Read potentiometer value
  int pwmOutput = map(potValue, 0, 700, 0 , 255); // Map the potentiometer value from 0 to 255
  analogWrite(Mot1, pwmOutput); // Send PWM signal to L298N Enable pin
  // put your main code here, to run repeatedly:


  if(millis()-timer >= 1*1000){
    Serial.print("Time used: ");
    Serial.println(millis()-timer);
    Serial.print("Encoder ticks: ");
    Serial.println(encoderCount);
    timer = millis();
    encoderCount = 0;
  }

}
