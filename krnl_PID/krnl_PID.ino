#include <krnl.h>
#include <avr/wdt.h>

//// CONSTANTS //////
#define ENCODER_PIN 2 // Quadratic pin 1
#define DIRECTION_PIN 9 // Quadratic pin 2
#define Mot1 10 // pwm Pin
#define Mot2 12 //Direction of pin 1
#define pot A0 // potentiometer pin

#define STK_SIZE 200 // Stack size for the krnl
#define LOG_BUFFER_SIZE 100 // Buffer size for the LOG
#define MSG_BUFFER_SIZE 20 // Buffer size for the message buffer

// PID values
#define ENCODER_RESOLUTION 500
#define PWM_TO_RPS_FACTOR 0.118
#define INTEGRAL_MIN -1000
#define INTEGRAL_MAX 1000
#define POTEN_MAX 700

/////////////////////

// Krnl variables
struct k_t *pt1, *pt2, *pt3;

struct k_msg_t *pMsg1, *pMsg2;
int mar1[2], mar2[2];

char stak1[STK_SIZE], stak2[STK_SIZE], stak3[STK_SIZE];;

// Define global variables
volatile int icnt = 0;

unsigned long prevtime=0;
float prev_e = 0;
float integral_e = 0;
float PID[3];
// Buffer for the serial read
char serialBuffer[32]; // Buffer for the serial message
volatile uint8_t bufferIndex = 0; // This indicates current position in the buffer

// Encoder interrupt task
#if defined (__AVR_ATmega2560__) || defined (__AVR_ATmega1280__)
ISR(INT4_vect, ISR_NAKED) {
#else
ISR(INT0_vect, ISR_NAKED) {
#endif
  PUSHREGS();
  if (!k_running)
    goto exitt;


  // If we are here the encoder has moved, so we check the direction by the second pin
  if (digitalRead(DIRECTION_PIN)) {
    icnt--;
  } else {
    icnt++;
  }

  K_CHG_STAK();

exitt:
  POPREGS();
  RETI();
}


// Installing the interrupt task
void installISR()
{
  DI();
  pinMode(ENCODER_PIN, INPUT);
  digitalWrite(ENCODER_PIN, HIGH);

  pinMode(DIRECTION_PIN, INPUT);
  digitalWrite(DIRECTION_PIN, HIGH);

#if defined (__AVR_ATmega2560__) || defined (__AVR_ATmega1280__)
  EIMSK |= (1 << INT4);
  EICRB |= (1 << ISC41);
#else
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC01);
#endif

  EI();
}


// Serial reader task
void serialReaderTask() {
  while (1) {
    if (Serial.available()) { // If someone has written in the serial we read it
      char c = Serial.read();
      if (c == '\n' || bufferIndex >= MSG_BUFFER_SIZE - 1) { // If we exceed the serial, or find a newline we start treating the data
        char* token = strtok(serialBuffer, ","); // Split the data by commas
        int j = 0;
        while (token != NULL) {
          PID[j] = atof(token); // save the data into the PID array
          token = strtok(NULL, ",");
          j++;
        }
        bufferIndex = 0; // Reset the serial buffer
      } else {
        // Save the current read char into the buffer
        serialBuffer[bufferIndex++] = c;
      }
    }
    // Timing of readerTask is not important
    k_sleep(300);
  }
}


void taskController(){
  // let krnl send a signal to the semaphore every 3 ticks
  //k_set_sem_timer(s1, 100);
  while (1){
    //k_wait(s1, 0); // Wait sleep until next loop
    PID_controller();
    k_sleep(3);
  }
}


// Motor controller
void PID_controller(){
  int potValue = analogRead(A0); // Read potentiometer value
  int pwmOutput = map(potValue, 0, POTEN_MAX, 0 , 255); // Map the potentiometer value from 0 to 255
  float ref = pwmOutput * PWM_TO_RPS_FACTOR;

  //Serial.println(ref);
  int dt = millis() - prevtime;

  // send values to log
  k_send(pMsg1, &pwmOutput);
  k_send(pMsg2, &dt);

  prevtime = millis();
  // We could recieve the message here, but it's already accessible
  float vel = (float(icnt) / ENCODER_RESOLUTION) / (float(dt) / 1000); // RPS
  
  float error = ref-vel;// RPS   (icnt*circum)/encResolution/(100/dt);
  icnt = 0; // Reset the encoder counter
   
  float de=(error-prev_e)/dt;
 
  integral_e += error * dt;
  integral_e = constrain(integral_e, INTEGRAL_MIN, INTEGRAL_MAX); // stop clamping

  // Convert v_hat to PWM value
  int pwm = (PID[0]*error+PID[1]*integral_e+PID[2]*de);

  set_motor_PWM(pwm);
  prev_e = error;
}

void set_motor_PWM(int pwm){
  if(pwm > 255){
      pwm = 255;
    }else if(pwm < -255){
      pwm = -255;
    }
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

// Logging task
void loggingTask() {
  char res;
  int pwm, dt, lost; // don't really care about this
  while (1) {
    // retrive the ref value to log
    k_receive(pMsg1, &pwm, 0, &lost);
    k_receive(pMsg2, &dt, 0, &lost);

    // Send the data via the serial interface in a CSV format
    if(dt != 0){ // Just for safety reasons
      Serial.print("Input:");
      Serial.print(pwm * PWM_TO_RPS_FACTOR);
      Serial.print(",Output:");
      Serial.println((float(icnt) / ENCODER_RESOLUTION) / (float(dt) / 1000));
    }

    // Add a delay for controlling the logging frequency (e.g., log every 100 ms)
    k_sleep(100);
  }
}

void setup() {
  // idk why jens does this..
  wdt_reset();
  wdt_disable();

  Serial.begin(31250);
  delay(200);
  
  // Setup pins
  pinMode(pot,INPUT);
  pinMode(Mot1,OUTPUT);
  pinMode(Mot2,OUTPUT);
  
  // init krnl
  k_init(3, 0, 2);
  // Initiate the tasks
  pt1 = k_crt_task(taskController, 11, stak1, STK_SIZE); 
  pt2 = k_crt_task(serialReaderTask, 10, stak2, STK_SIZE); // Needs to be lower than the rest
  pt3 = k_crt_task(loggingTask, 12, stak3, STK_SIZE); // low priority logger task

  // Start the message
  pMsg1 = k_crt_send_Q(1, sizeof(int), mar1); // reference value
  pMsg2 = k_crt_send_Q(1, sizeof(int), mar2); // Time in between values

  // Initiate the ISR
  installISR();
  
  Serial.println("start the krnl");
  k_start();  // start kernel with tick speed 1 milli seconds
}

void loop() {
  // Since we are using the krnl, we never loop
  Serial.println("Something is wrong");
}
