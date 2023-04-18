#include <krnl.h>
#include <avr/wdt.h>

#define STK_SIZE 200
#define ENCODER_PIN 2
#define DIRECTION_PIN 10

struct k_t *pt1, *pt2;

struct k_msg_t *pMsg, *pMsg2;
char mar[10 * 2];
char mar2[10 * 2];

volatile int icnt = 0;

char stak1[STK_SIZE], stak2[STK_SIZE];


int loopNr = 0;

float PID[3]; // float array to store P, I, and D values
char serialBuffer[32]; // Buffer for the serial message
int serialBufferIndex = 0; // Index so i know how far in I am


void aktuer01() { // Dummy task to print out all my data
  Serial.println("aktuer");
  Serial.println(loopNr++);
  Serial.print("P: ");
  Serial.print(PID[0]);
  Serial.print(", I: ");
  Serial.print(PID[1]);
  Serial.print(", D: ");
  Serial.println(PID[2]);
  Serial.print("Encoder: ");
  Serial.println(icnt);
}

// Task that pretends it is doing shit
void busy_task(void) {
  while (1) {
    k_sleep(1000);
    k_eat_msec(200);
    aktuer01();
  }
}

// Serial reader task
void serialReaderTask() {
  while (1) {
    if (Serial.available()) { // If someone has written in the serial we read it
      char c = Serial.read();
      if (c == '\n' || serialBufferIndex >= sizeof(serialBuffer) - 1) { // If we exceed the serial, or find a newline we start treating the data
        char* token = strtok(serialBuffer, ","); // Split the data by commas
        int j = 0;
        while (token != NULL) {
          PID[j] = atof(token); // save the data into the PID array
          token = strtok(NULL, ",");
          j++;
        }
        serialBufferIndex = 0; // Reset the serial buffer
      } else {
        // Save the current read char into the buffer
        serialBuffer[serialBufferIndex++] = c;
      }
    }
    // Not sure how much sleep this should have, any seems fine to me
    k_sleep(300);
  }
}

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
    icnt++;
  } else {
    icnt--;
  }
  // Send the increment message
  ki_send(pMsg, (void *)&icnt);

  K_CHG_STAK();

exitt:
  POPREGS();
  RETI();
}

// Installing the interrupt task
void installISR2()
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


void setup() {
  // idk why jens does this..
  wdt_reset();
  wdt_disable();
  
  Serial.begin(9600);
  pinMode(13, OUTPUT);

  // init krnl so you can create 2 tasks, no semaphores and 1 message queue
  k_init(2, 0, 1);
  // Initiate the tasks
  pt1 = k_crt_task(busy_task, 11, stak1, STK_SIZE);
  pt2 = k_crt_task(serialReaderTask, 10, stak2, STK_SIZE); // Needs to be lower than the rest
  // Start the message
  pMsg = k_crt_send_Q(10, 2, mar);
  // Initiate the ISR
  installISR2();
  k_start();  // start kernel with tick speed 1 milli seconds
}

void loop() {
  /* loop will never be called */
}


extern "C" {

	void k_breakout() // called every task shift from dispatcher
	{


		if (pRun->nr == 0)  // 0 is dummy task - the eater of excessive CPU when all user tasks are idling
		{
			PORTB = PORTB | B00100000;  // led13 (bit 5) on let the rest be untouched
		}
		else {
			PORTB = PORTB & B11011111;  // led13 off uno
		}
		/* using D8-D13 use following instead of teh code above*/
		/* PORTB = (1 << pRun->nr); */
	}

}
