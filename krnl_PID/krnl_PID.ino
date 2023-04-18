#include <krnl.h>
#include <avr/wdt.h>

#define ENCODER_PIN 2
#define DIRECTION_PIN 9

#define STK_SIZE 200

#define pot A0 // potentiometer pin
#define Mot1 10 // pwm Pin
#define Mot2 12 //Direction of pin 1

#define BUFFER_SIZE 20

// Krnl variables
struct k_t *pt1, *pt2;

struct k_msg_t *pMsg;
char mar[10 * 2];

char stak1[STK_SIZE], stak2[STK_SIZE];

// Define global variables
volatile int icnt = 0;

unsigned long prevtime=0;
float prev_e = 0;
float prev_inte = 0;
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
  // Send the increment message
  ki_send(pMsg, (void *)&icnt);

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
      if (c == '\n' || bufferIndex >= sizeof(serialBuffer) - 1) { // If we exceed the serial, or find a newline we start treating the data
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
    // Not sure how much sleep this should have, any seems fine to me
    k_sleep(300);
  }
}


void taskController(){
    while (1){
  PID_controller();
  // Wait sleep until next loop
  k_sleep(100);
    }
}


// Motor controller
void PID_controller(){
  int potValue = analogRead(A0); // Read potentiometer value
  int pwmOutput = map(potValue, 0, 700, 0 , 255); // Map the potentiometer value from 0 to 255


  float ref = pwmOutput*0.118; // RPmS
  //Serial.println(ref);
  unsigned long dt = millis() - prevtime;
  prevtime = millis();
  float vel = (float(icnt)/500) / (float(dt)/1000); // RPmS
  
  float error = ref-vel;// RPmS   (icnt*circum)/encResolution/(100/dt);

  icnt = 0; // Reset the encoder counter
    
  float de=(error-prev_e)/dt;
 
  //float integral_e = prev_inte + dt*(error+prev_e)/2;
  
  prev_e = error;
  
  
  //Serial.println(integral_e);   
  
  // Convert v_hat to PWM value
  //(PID[0]*error+PID[1]*de+PID[2]*integral_e)
  int pwm = (PID[0]*error+PID[1]*de)/0.118;
  /*Serial.print("pwm: ");
  Serial.println(pwm);
  Serial.print("error: ");
  Serial.println(error); */
  set_motor_PWM(pwm);
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


void setup() {
  // idk why jens does this..
  wdt_reset();
  wdt_disable();

  Serial.begin(9600);
  while (!Serial) {  }
  
  // Setup pins
  pinMode(pot,INPUT);
  pinMode(Mot1,OUTPUT);
  pinMode(Mot2,OUTPUT);
  
  // init krnl
  k_init(2, 0, 1);
  // Initiate the tasks
  pt1 = k_crt_task(taskController, 10, stak1, STK_SIZE); // Needs to be lower than the rest
  pt2 = k_crt_task(serialReaderTask, 11, stak2, STK_SIZE); 
  
  // Start the message
  pMsg = k_crt_send_Q(10, 2, mar);

  // Initiate the ISR
  installISR();
  
  Serial.println("start the krnl");
  k_start();  // start kernel with tick speed 1 milli seconds
}

void loop() {
  // Since we are using the krnl, we never loop
  Serial.println("Something is wrong");
}




