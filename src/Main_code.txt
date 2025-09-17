/* ============================================================================
   ATmega328P (16 MHz) — Bare-metal port of your Arduino sketch
   - L298N mapping preserved:
       ENA→PD5(D5), ENB→PD6(D6), IN1→PD2(D2), IN2→PD3(D3), IN3→PD4(D4), IN4→PD7(D7)
     HC-SR04: ECHO→PB0(D8), TRIG→PB1(D9)
     LEDs: PD0(D0), PD1(D1)  (UART pins; no Serial used)
     Buttons: PC1(A1), PC2(A2)
     IR sensors: PB2(D10), PB3(D11), PB4(D12)  (HIGH on black, LOW on white)
   - Timers:
       T0: Fast PWM on D5/D6 + overflow ISR for millis (~1.024 ms ticks)
       T1: free-running counter for pulseIn (0.5 µs ticks @ prescaler 8)
   ============================================================================ */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

/* ---------------- Arduino-like shim ---------------- */

enum { INPUT=0, OUTPUT=1, INPUT_PULLUP=2 };
enum { LOW=0, HIGH=1 };

static inline void pinMode(uint8_t dpin, uint8_t mode) {
  if (dpin <= 7) { // PD0..PD7 => D0..D7
    uint8_t bit = dpin;
    if (mode == OUTPUT) { DDRD |=  (1<<bit); }
    else {
      DDRD &= ~(1<<bit);
      if (mode == INPUT_PULLUP) PORTD |=  (1<<bit);
      else                      PORTD &= ~(1<<bit);
    }
  } else if (dpin <= 13) { // PB0..PB5 => D8..D13
    uint8_t bit = dpin - 8;
    if (mode == OUTPUT) { DDRB |=  (1<<bit); }
    else {
      DDRB &= ~(1<<bit);
      if (mode == INPUT_PULLUP) PORTB |=  (1<<bit);
      else                      PORTB &= ~(1<<bit);
    }
  } else { // A0..A5 => D14..D19 (PC0..PC5)
    uint8_t bit = dpin - 14;
    if (mode == OUTPUT) { DDRC |=  (1<<bit); }
    else {
      DDRC &= ~(1<<bit);
      if (mode == INPUT_PULLUP) PORTC |=  (1<<bit);
      else                      PORTC &= ~(1<<bit);
    }
  }
}

static inline void digitalWrite(uint8_t dpin, uint8_t val) {
  if (dpin <= 7) {
    if (val) PORTD |=  (1<<dpin);
    else     PORTD &= ~(1<<dpin);
  } else if (dpin <= 13) {
    uint8_t bit = dpin - 8;
    if (val) PORTB |=  (1<<bit);
    else     PORTB &= ~(1<<bit);
  } else {
    uint8_t bit = dpin - 14;
    if (val) PORTC |=  (1<<bit);
    else     PORTC &= ~(1<<bit);
  }
}

static inline int digitalRead(uint8_t dpin) {
  if (dpin <= 7)       return (PIND & (1<<dpin)) ? HIGH : LOW;
  else if (dpin <= 13) return (PINB & (1<<(dpin-8))) ? HIGH : LOW;
  else                 return (PINC & (1<<(dpin-14))) ? HIGH : LOW;
}

/* ---- analogWrite for D5(OC0B) and D6(OC0A), Fast PWM non-inverting ---- */
static inline void analogWrite(uint8_t dpin, uint8_t val) {
  if (dpin == 6) { // PD6, OC0A
    OCR0A = val;
  } else if (dpin == 5) { // PD5, OC0B
    OCR0B = val;
  } else {
    // For other pins we just do a digital write threshold (rarely needed here)
    digitalWrite(dpin, val >= 128 ? HIGH : LOW);
  }
}

/* ---- millis() via Timer0 overflow (976.5625 Hz -> ~1.024 ms per tick) ---- */
volatile uint32_t g_millis = 0;
ISR(TIMER0_OVF_vect) {
  g_millis += 1; // ~1.024 ms per increment (good enough for 20/250 ms windows)
}
static inline uint32_t millis(void) {
  uint32_t m;
  uint8_t sreg = SREG; cli(); m = g_millis; SREG = sreg; return m;
}

/* ---- delay wrappers ---- */
static inline void delay(uint32_t ms) {
  while (ms--) _delay_ms(1);
}
static inline void delayMicroseconds(uint16_t us) {
  while (us--) _delay_us(1);
}

/* ---- micros & pulseIn using Timer1 (0.5 µs ticks @ prescaler = 8) ---- */
static inline void timer1_init_for_micros(void) {
  TCCR1A = 0;
  TCCR1B = (1<<CS11); // prescaler 8 -> 2 MHz -> 0.5 µs per tick
}

static inline uint32_t micros(void) {
  // 0.5 µs per tick; we return in µs by dividing by 2
  uint16_t t = TCNT1;
  return ((uint32_t)t) / 2;
}

/* pulseIn(pin, state, timeout_us): returns pulse width in microseconds or 0 on timeout */
static inline uint32_t pulseIn(uint8_t dpin, uint8_t state, uint32_t timeout_us) {
  uint32_t start = micros();

  // wait for any previous pulse to end
  while (digitalRead(dpin) == state) {
    if ((micros() - start) >= timeout_us) return 0;
  }

  // wait for the pulse to start
  start = micros();
  while (digitalRead(dpin) != state) {
    if ((micros() - start) >= timeout_us) return 0;
  }

  // measure the pulse
  uint32_t pulseStart = micros();
  while (digitalRead(dpin) == state) {
    if ((micros() - start) >= timeout_us) return 0;
  }
  uint32_t pulseEnd = micros();
  return (pulseEnd - pulseStart);
}

/* ---- Timer0 setup for Fast PWM on D5/D6 + overflow ISR ---- */
static inline void timer0_init_pwm_and_millis(void) {
  // Fast PWM: WGM01:0 = 3, WGM02 = 0  (mode 3)
  TCCR0A = (1<<WGM00) | (1<<WGM01);
  TCCR0B = 0; // set prescaler later (same as your sketch)

  // Non-inverting PWM on OC0A (PD6) & OC0B (PD5)
  TCCR0A |= (1<<COM0A1) | (1<<COM0B1);

  // Prescaler /64 (matches your sketch TCCR0B lower bits = 0x03)
  TCCR0B |= (1<<CS01) | (1<<CS00); // 0b011 -> /64

  // Enable overflow interrupt for millis
  TIMSK0 |= (1<<TOIE0);

  // Init duty
  OCR0A = 0; // ENB
  OCR0B = 0; // ENA
}

/* ---------------- Your original constants & logic ---------------- */

#define USE_SERIAL 0  // (UART pins used as LEDs; no Serial here)

/* --- IR Sensor Pins (Arduino numbering, shim maps to ports) --- */
#define leftIR   10
#define middleIR 11
#define rightIR  12

/* --- L298N Motor Control Pins --- */
#define ENA 5
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 7
#define ENB 6

/* --- Ultrasonic Sensor Pins --- */
#define trigPin 9
#define echoPin 8

/* --- Push Buttons & LEDs --- */
#define button1 A1  // 15
#define button2 A2  // 16
#define greenLED 0  // PD0
#define blueLED  1  // PD1
#define redLED   A0 // 14

/* --- Mode Speeds & Turn Durations --- */
// Mode 1: Line Follow
const int LINE_SPEED = 50;
const int LINE_FAST = 160;
const int LINE_TURN_SHORT = 30;
const int LINE_TURN_LONG  = 300;
const int LINE_TURN_SPEED_SLOW = 65;
const int LINE_TURN_SPEED_FAST = 120;

// Mode 2: Obstacle + Line Follow
const int OBST_SPEED = 50;
const int OBST_FAST  = 160;
const int OBST_TURN_SHORT = 30;
const int OBST_TURN_LONG  = 120;
const int OBST_TURN_SPEED_SLOW = 50;
const int OBST_TURN_SPEED_FAST = 60;

/* --- Obstacle Detection --- */
bool obstacleDetected = false;
const int OBSTACLE_DISTANCE = 20;
const int OBSTACLE_CONFIRM_COUNT = 2;
int obstacleDetectionCount = 1;

/* --- Non-blocking Turn --- */
unsigned long turnStartTime = 0;
bool isTurning = false;
// Replace Arduino String with robust enum for bare C:
typedef enum { TURN_NONE=0, TURN_LEFT=1, TURN_RIGHT=2 } TurnDir;
TurnDir turnDirection = TURN_NONE;

int turnDuration = 300;
int turnSpeed = 120;

/* --- Obstacle LED Blinking --- */
unsigned long previousBlinkTime = 0;
const long blinkInterval = 100;
bool ledState = false;

/* --- Sensor Timing --- */
unsigned long lastIRReadTime = 0;
const int IR_READ_INTERVAL = 20;
unsigned long lastUltraSonicTime = 0;
const int ULTRASONIC_READ_INTERVAL = 250;

/* --- Mode Variable --- */
int mode = 0;

/* ----------- Forward declarations ----------- */
void setMode(int newMode);
void lineFollow(int currentMode);
void obstacleLineFollow(void);
int  getDistance(void);
void startTurn(TurnDir direction, int duration, int speed);
void continueTurn(void);
void moveForward(int speed);
void stopCar(void);

/* ------------------- SETUP ------------------- */
void setup(void) {
  pinMode(leftIR,   INPUT);
  pinMode(middleIR, INPUT);
  pinMode(rightIR,  INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  /* Configure timers */
  timer0_init_pwm_and_millis();  // PWM + millis (prescaler /64 same as sketch)
  timer1_init_for_micros();      // pulseIn timing

  sei(); // enable global interrupts

  /* Your sketch had: TCCR0B = (TCCR0B & 0b11111000) | 0x03; already matched above */

  #if USE_SERIAL
    // Not used in this bare-metal port.
  #endif

  delay(300);

  setMode(0);
  stopCar();
}

/* ------------------- LOOP ------------------- */
void loop(void) {
  if (digitalRead(button1) == LOW) { setMode(1); delay(300); }
  if (digitalRead(button2) == LOW) { setMode(2); delay(300); }

  if (obstacleDetected) {
    unsigned long currentTime = millis();
    if (currentTime - previousBlinkTime >= (unsigned long)blinkInterval) {
      previousBlinkTime = currentTime;
      ledState = !ledState;
      digitalWrite(redLED, ledState ? HIGH : LOW);
    }
  } else {
    digitalWrite(redLED, LOW);
  }

  if (mode == 1)       lineFollow(1);
  else if (mode == 2)  obstacleLineFollow();
  else                 stopCar();
}

/* ------------------- MODE FUNCTION ------------------- */
void setMode(int newMode) {
  mode = newMode;
  obstacleDetected = false;
  isTurning = false;
  obstacleDetectionCount = 0;
  turnDirection = TURN_NONE;

  digitalWrite(greenLED, (mode == 1) ? HIGH : LOW);
  digitalWrite(blueLED,  (mode == 2) ? HIGH : LOW);
  digitalWrite(redLED,   LOW);
}

/* ------------------- LINE FOLLOWING ------------------- */
void lineFollow(int currentMode) {
  unsigned long currentTime = millis();
  if (currentTime - lastIRReadTime >= (unsigned long)IR_READ_INTERVAL) {
    lastIRReadTime = currentTime;

    int leftValue   = digitalRead(leftIR);
    int middleValue = digitalRead(middleIR);
    int rightValue  = digitalRead(rightIR);

    int forwardSpeed = (currentMode == 1) ? LINE_SPEED : OBST_SPEED;
    int fastSpeedVal = (currentMode == 1) ? LINE_FAST  : OBST_FAST;

    int turnShort = (currentMode == 1) ? LINE_TURN_SHORT : OBST_TURN_SHORT;
    int turnLong  = (currentMode == 1) ? LINE_TURN_LONG  : OBST_TURN_LONG;
    int turnSlow  = (currentMode == 1) ? LINE_TURN_SPEED_SLOW : OBST_TURN_SPEED_SLOW;
    int turnFast  = (currentMode == 1) ? LINE_TURN_SPEED_FAST : OBST_TURN_SPEED_FAST;

    if (leftValue == HIGH && middleValue == HIGH && rightValue == HIGH) {
      moveForward(forwardSpeed);
    } else if (leftValue == LOW && middleValue == LOW && rightValue == LOW) {
      stopCar();
    } else if (leftValue == LOW && middleValue == HIGH && rightValue == LOW) {
      moveForward(fastSpeedVal);
    }
    // LEFT TURN
    else if (leftValue == HIGH && middleValue == LOW && rightValue == LOW) {
      startTurn(TURN_LEFT, turnShort, turnSlow);
    } else if (leftValue == HIGH && middleValue == HIGH && rightValue == LOW) {
      startTurn(TURN_LEFT, turnLong,  turnFast);
    }
    // RIGHT TURN
    else if (leftValue == LOW && middleValue == LOW && rightValue == HIGH) {
      startTurn(TURN_RIGHT, turnShort, turnSlow);
    } else if (leftValue == LOW && middleValue == HIGH && rightValue == HIGH) {
      startTurn(TURN_RIGHT, turnLong,  turnFast);
    }
    else {
      moveForward(fastSpeedVal);
    }
  }
}

/* ------------------- LINE FOLLOW + OBSTACLE ------------------- */
void obstacleLineFollow(void) {
  unsigned long currentTime = millis();
  if (currentTime - lastUltraSonicTime >= (unsigned long)ULTRASONIC_READ_INTERVAL) {
    lastUltraSonicTime = currentTime;
    int distance = getDistance();
    if (distance <= OBSTACLE_DISTANCE && distance > 0) {
      obstacleDetectionCount++;
      if (obstacleDetectionCount >= OBSTACLE_CONFIRM_COUNT && !obstacleDetected) {
        obstacleDetected = true;
        stopCar();
        isTurning = false;
      }
    } else {
      obstacleDetectionCount = 0;
      if (obstacleDetected) obstacleDetected = false;
    }
  }
  if (!obstacleDetected) {
    if (isTurning) continueTurn();
    else           lineFollow(2); // Mode 2
  }
}

/* ------------------- ULTRASONIC ------------------- */
int getDistance(void) {
  uint8_t s = SREG; cli();
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  uint32_t duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout
  SREG = s;
  if (duration == 0) return OBSTACLE_DISTANCE + 10;
  // speed of sound ~0.034 cm/us, divide by 2 for round-trip
  return (int)((duration * 34UL) / 2000UL); // (duration * 0.034 / 2) in cm
}

/* ------------------- TURNING ------------------- */
void startTurn(TurnDir direction, int duration, int speed) {
  turnDirection = direction;
  turnStartTime = millis();
  turnDuration  = duration;
  turnSpeed     = speed;
  isTurning     = true;

  if (direction == TURN_LEFT) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, turnSpeed);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, turnSpeed);
  } else { // TURN_RIGHT
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, turnSpeed);
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, turnSpeed);
  }
}

void continueTurn(void) {
  if (millis() - turnStartTime >= (unsigned long)turnDuration) {
    isTurning = false;
    int leftValue   = digitalRead(leftIR);
    int middleValue = digitalRead(middleIR);
    int rightValue  = digitalRead(rightIR);

    if (middleValue == HIGH || (leftValue == LOW && rightValue == LOW)) {
      moveForward((mode == 1) ? LINE_SPEED : OBST_SPEED);
    } else if ((turnDirection == TURN_LEFT  && leftValue  == HIGH) ||
               (turnDirection == TURN_RIGHT && rightValue == HIGH)) {
      startTurn(turnDirection, turnDuration, turnSpeed);
    }
  }
}

/* ------------------- MOTOR CONTROL ------------------- */
void moveForward(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, (uint8_t)speed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, (uint8_t)speed);
}

void stopCar(void) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

/* ------------------- main ------------------- */
int main(void) {
  setup();
  for(;;) {
    loop();
  }
}
