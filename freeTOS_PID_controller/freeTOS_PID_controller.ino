#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <Stream.h>
#include <semphr.h>

#define Q1 2 // Q1 pin number
#define Q2 9 // Q2 pin number

#define pot A0 // potentiometer pin
#define Mot1 11 // pwm Pin
#define Mot2 12 //Direction of pin 1

#define BUFFER_SIZE 20
#define THRESHOLD   10 // Threshold for when i execute the serial related task

// Define global variables
volatile int encoderCount = 0;

unsigned long prevtime=0;
float prev_e = 0;
float prev_inte = 0;
float PID[3];
// Buffer for the serial read
char buffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0; // This indicates current position in the buffer

// Semaphore for the serial read
SemaphoreHandle_t serialSem;

// Define functions
void encoderISR(); // Encoder interrupt
void TaskReadFromSerial( void *pvParameters ); // Get commands
void TaskController( void *pvParameters ); // motor controller
void set_PID();
void set_motor_PWM(int pwm, int direction);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {  }

  // Setup pins
  pinMode(pot,INPUT);
  pinMode(Q1, INPUT_PULLUP); // Set pin Q1 as input with pull-up resistor
  pinMode(Q2, INPUT_PULLUP); // Set pin Q2 as input with pull-up resistor
  pinMode(Mot1,OUTPUT);
  pinMode(Mot2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Q1), encoderISR, RISING); // Attach interrupt to pin Q14


  serialSem = xSemaphoreCreateBinary();

  if ( serialSem != NULL )
    xSemaphoreGive( serialSem );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  
  xTaskCreate(TaskController, "Motor Control",  256,  NULL,  2,  NULL);
}

void loop() {
  // Nothing wow
}

// Interrupt event in case the encoders tick
void encoderISR() {
  //digitalWrite(intPin, HIGH);
  if (digitalRead(Q2) == HIGH) {
    encoderCount--; // Count up if pin Q2 is also high
  }
  else {
    encoderCount++; // Count down if pin Q2 is low
  }
  //digitalWrite(intPin, LOW);
}

// If the serial reacts, then I take the semaphore and read the serial
void serialEvent() {
  while (Serial.available()) {
    char c = Serial.read();
    if (bufferIndex < BUFFER_SIZE && c != '\n'){
      buffer[bufferIndex++] = c;
    } else if(bufferIndex >= BUFFER_SIZE) {
      // Buffer overflow! Ignore the rest of the incoming data.
      //Serial.println("Buffer overflow!");
      break;
    }
  }
  xSemaphoreGive(serialSem); // Semaphore indicating we are done reading
}

// Task for motor control
void TaskController(void *pvParameters)
{
  (void) pvParameters;
  
  for (;;) // A Task shall never return or exit.
  {
    if (xSemaphoreTake(serialSem, (TickType_t) 10) == pdTRUE) { // If there are over 10 characters in the buffer it means, we have gotten new PID values
      if (bufferIndex >= THRESHOLD) {
        char serialout[10];
        char temp;
        // We got data from the serial port and the buffer has reached the threshold. Print it.
        int currentPID = 1;
        // WIP
        for (int i = 0; i < bufferIndex; i++) {
          temp = buffer[i];
          switch(currentPID){
            case 1:
              // read from buffer
              currentPID = 2; 
              k++;
              break;
            case 2:
              // read from buffer
              currentPID = 3;
              break;
            case 3:
              // read from buffer
              break;
          }
          serialout[i] = temp;
        }
        for (int i = 0; i < bufferIndex; i++) {
          
        }
        for (int i = 0; i < bufferIndex; i++) {
          
        }
        bufferIndex = 0; // Reset the buffer index.
      } else {
        // The buffer has not yet reached the threshold. Perform the PID control with current values processing.
        set_PID();
      }
    }
  }
}

void set_PID(){
  int potValue = analogRead(A0); // Read potentiometer value
  int pwmOutput = map(potValue, 0, 700, 0 , 255); // Map the potentiometer value from 0 to 255
  
  float ref = pwmOutput*0.118; // RPS
  
  unsigned long dt = millis() - prevtime;
  prevtime = millis();

  float error = ref-1/(encoderCount*(dt/1000));// RPS   (encoderCount*circum)/encResolution/(100/dt);
  encoderCount = 0; // Reset the encoder counter    
  
  float de=(error-prev_e)/dt;

  float integral_e = prev_inte + dt*(error+prev_e)/2;
  prev_e = error;
  prev_inte = integral_e;
  
  // Convert v_hat to PWM value
  int pwm = (PID[0]*error+PID[1]*de+PID[2]*integral_e)/0.118;

  set_motor_PWM(pwm);
  vTaskDelay(100/portTICK_PERIOD_MS);
}

void set_motor_PWM(int pwm){
  if(pwm > 0){
    // Set direction
    digitalWrite(Mot2, HIGH);
    // Send the PWM signal
    analogWrite(Mot1, pwm);       
  }else{
    digitalWrite(Mot2, LOW);
    analogWrite(Mot1, abs(pwm));
  }
}
