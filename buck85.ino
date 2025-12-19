/*

NOTE:  YOU MUST USE THE BOD 4.3V TARGET FOR THIS TO WORK CORRECTLY.

Simple? 5 volt buck convert that uses a P-channel mosfet attached to PIN_PWM.

How to use:

1.  A freshly programmed device will try to hit 0.650V by default.
2.  To program:
   2.1 Apply your reference voltage to the output pads
   2.2 Short the CONFIG pad to ground (ideally with some resistance) for at least 2+ seconds continuously
   2.3 Disconnect the CONFIG pad and turn off your reference voltage
   2.4 Verify the programmed voltage is close to target (go back to 2.1 if it's off by too much)
   2.5 (note: If there is a huge delta between the previous target it might take a few programming cycles to get the target right)

It uses a roughly 5:1 divider on PIN_ADC so we can map 5 volts to below the 1.1 internal reference
voltage.

if PIN_CONFIG is grounded it goes into "config mode" where it reads PIN_ADC and averages it into
a running total.  Once PIN_CONFIG goes high again it stores the trained ADC value in EEPROM and 
goes into "pre gen" mode.

This sketch assumes the following (see the gerber files with the project too...):

1.  A somewhat stable 5 volt input is provided.  This isn't really designed for lower input voltages
    but it may work a slightly lower (though the mosfet might have higher resistance)

2.  The output is capped at a min of TOO_LOW (currently: 63.53mV) so lower voltages are not possible.
    Expect a range of roughly 100mV to 4600mV thereabouts though it's probably only stable starting at just above Vf (so around 600mV)
    You will want to cap TOO_HIGH to something like 950 (4.64V) because higher than that and the circuit can be inefficient 
    (both in power and it's ability to respond to load changes)

3.  It's not highly precise... the PWM is 8-bits and the ADC is 10-bit.  So expect step sizes of probably around 15-20mV

4.  It's meant for 1A continuous load max. The control resolution (one ADC count) is ~4.88mV, and the PWM resolution is 19.5mV per
    PWM_BIG_STEP. Not really suitable for highly variable loads (motors/etc).

5.  There's a resistor in series with the PWM pin and mosfet gate (I used a 100 Ohm)

6.  There's a pullup to Vcc attached to the mosfet gate (I used a 10kOhm)

7.  There's output filtering (I used 2x22uF ceramic + 1x220uF aluminum caps) (ensure the ceramics are low ESR)

8.  There's input filtering (I used 1x22uF ceramic + 1x220uF aluminum caps)

9.  There's a 5:1 divider attached to the ADC pin (I used 1% 35.7kOhm + 10kOhm resistors)

10.  The mosfet can easily switch and has low ON resistance with the -5V the tiny85 provides (I used a DMG2301L)

11.  The inductor is shielded and can saturate a bit more than 1 amp (I used a Fixed Inductor 47uH 1.5 amp 156mOhm)

12.  There's a suitable fast diode (I used a 40V 550mV 2A DSS24 diode)

Layout:

            /------------\
   N/C    - |            | - VCC (5V)
   CONFIG - | attiny85   | - N/C
   FB     - |            | - DEBUG PIN
   GND    - |            | - PWM out to mosfet gate (via 100Ohm resistor)
            \------------/

*/

#include <EEPROM.h>

// use PIN_DEBUG as serial debugger (serial code is hard coded to use PB1)
//#define DEBUG

// use PIN_DEBUG as PWM debugger
#ifndef DEBUG
//#define DEBUG_PWM
#endif

#define PIN_DEBUG  PB1
#define PIN_CONFIG PB3
#define PIN_PWM    PB0 // if you change this you need to change the timer code too in setup()
#define PIN_ADC    A2  // use A* name not PB* name here

#define TOO_LOW 13                 // anything below 64mV is considered a short (13 * 5000/1023 == 63.53mV)
#define TOO_HIGH 950               // anything above is considered illsuited for the circuit(950 * 5000/1023 == 4643mV)

#define DEFAULT_TARGET (133U)      // 133 * 5000 / 1023 == 650mV

#define TRAINING_LOOPS 4 // how many times do we average the training adc value before stopping

enum {
   FSM_GEN,
   FSM_GEN_FAULT, // turn off mosfet to disable output because we detected a fault
   FSM_CONFIG_SENSE, // sense and average the ADC code 
   FSM_WAIT_RECOVERY, // state to enter when fault occurs
};

unsigned char training_loop, fsm_state, cur_pwm = 224, training_cnt = 0; // set cur_pwm to mostly on because it's a P-channel so a HIGH means turn the mosfet off
unsigned long pre_gen_time;
unsigned current_adc, target_adc, training_adc = 0;
int integral_sum = 0; // accumulated error over time

#ifdef DEBUG
#define TX_PIN PIN_DEBUG

//#define BAUD_RATE 2400UL
//#define BAUD_RATE 9600UL
#define BAUD_RATE 19200UL

// Calculate the time (in microseconds) for one bit period: 1,000,000 / BAUD_RATE
#define BIT_PERIOD_US (1000000UL / BAUD_RATE)

// Initialize the TX pin to be an output (High when idle)
void tinySerial_begin() {
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH); // Line is HIGH when idle
  delay(5*64UL); // Small startup delay
}

// Sends a single byte (non-blocking when called)
void tinySerial_write(unsigned char data) {
  unsigned char mask;
  // Use bit 1 for PB1
  #define TX_BIT 1 

  cli();
  
  // 1. Send START BIT (LOW for one period)
  PORTB &= ~(_BV(TX_BIT)); // FAST: Set PB1 LOW
  delayMicroseconds(BIT_PERIOD_US);

  // 2. Send DATA BITS (8 bits, LSB first)
  for (mask = 1; mask > 0; mask <<= 1) {
    if (data & mask) {
      PORTB |= _BV(TX_BIT); // FAST: Set PB1 HIGH
    } else {
      PORTB &= ~(_BV(TX_BIT)); // FAST: Set PB1 LOW
    }
    delayMicroseconds(BIT_PERIOD_US);
  }

  // 3. Send STOP BIT (HIGH for one period)
  PORTB |= _BV(TX_BIT); // FAST: Set PB1 HIGH
  delayMicroseconds(BIT_PERIOD_US);

  sei();
}
// Sends a string (converts it to bytes)
void tinySerial_print(char *str) {
  while (pgm_read_byte(str)) {
    tinySerial_write(pgm_read_byte(str++));
  }
}

// Sends a string followed by a carriage return and newline
void tinySerial_println(char *str) {
  tinySerial_print(str);
  tinySerial_write('\r');
  tinySerial_write('\n');
}

// Prints a signed 16-bit integer (up to +/- 32767).
void tinySerial_printInt(int n) {
  char buf[6]; // Buffer size for -32767 + null terminator
  char *str = buf;
  char *p;
  unsigned int k;
  
  // 1. Handle negative sign
  if (n < 0) {
    tinySerial_write('-');
    k = -n; // Convert to positive unsigned value
  } else {
    k = n;
  }
  
  // 2. Handle 0 separately
  if (k == 0) {
    tinySerial_write('0');
    return;
  }
  
  // 3. Convert integer to reverse ASCII string (e.g., 123 becomes '3', '2', '1')
  while (k > 0) {
    // 0x30 is ASCII '0'
    *str++ = (char)(k % 10 + 0x30); 
    k /= 10;
  }
  *str = '\0'; // Null-terminate the temporary string
  
  // 4. Print the characters in the correct order (reverse the temporary string)
  for (p = str - 1; p >= buf; p--) {
    tinySerial_write(*p);
  }
}
#endif

void load_target()
{
  target_adc = ((unsigned)EEPROM.read(1) << 8) | ((unsigned)EEPROM.read(2));
#ifdef DEBUG
  tinySerial_print(PSTR("target_adc <= "));
  tinySerial_printInt(target_adc);
  tinySerial_println(PSTR(""));
#endif  
}

void store_target(unsigned target)
{
  EEPROM.write(1, (target) >> 8);
  EEPROM.write(2, target & 0xFF);
  EEPROM.write(0, 0xA5);
}

// adjust the PWM to target the target_adc
void update_pwm()
{
  int new_pwm, delta;
  unsigned sum;
  unsigned char x;

  current_adc = analogRead(PIN_ADC);
  delta = (int)current_adc - (int)target_adc; // how far off are we in steps of 5000mV / 1023 == 4.88mV
  integral_sum += delta;
  new_pwm = (int)cur_pwm;

  // adjust signed pwm count
  new_pwm += (delta >> 1);

  // saturate the integral error so we avoid winding up too hard
  if (integral_sum > 255) {
    integral_sum = 255;
  } else if (integral_sum < -256) {
    integral_sum = -256;
  }
  new_pwm += (integral_sum + 128) >> 8;

  // saturate the PWM value
  if (new_pwm > 255) {
    new_pwm = 255;
  } else if (new_pwm < 0) {
    new_pwm = 0;
  }
  cur_pwm = new_pwm;
  OCR0A = cur_pwm;
}

void setup_gen()
{
  integral_sum = 0;
  fsm_state = FSM_GEN; // go back to pre-gen phase
  load_target();
  cur_pwm = 255; // start with the MOSFET fully off
  OCR0A = cur_pwm;
  delay(25*64);
}

void setup() {
  TCCR1 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12) | _BV(CS13)); // Stop Timer1

#ifdef DEBUG  
  tinySerial_begin();
  tinySerial_println(PSTR("\r\n\r\nHello from Buck85\r\n"));
#endif
#ifdef DEBUG_PWM
  pinMode(PIN_DEBUG, OUTPUT);
#endif

  // disable the mosfet
  pinMode(PIN_PWM, OUTPUT);

  // 2. Configure Timer0 for Fast PWM on OC0A (PB0)
  // COM0A1 = 1: Non-inverting PWM (Clear on match, set at BOTTOM)
  // WGM01 & WGM00 = 1: Fast PWM Mode
  TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);

  // 3. Set the frequency (Prescaler)
  // Your original code used (TCCR0B & 0xF8) | 0x01 which is "No Prescaling"
  TCCR0B = (TCCR0B & 0xF8) | 0x01; 

  // 4. Initial state: OFF for P-Channel (255 = Gate High)
  OCR0A = 255;
 
  // configure config pin
  pinMode(PIN_CONFIG, INPUT_PULLUP);

  // configure ADC to use internal ref
  pinMode(PIN_ADC, INPUT);
  analogReference(INTERNAL);

  // configure EEPROM
  if (EEPROM.read(0) != 0xA5) {
    // configure to a low voltage
    store_target(DEFAULT_TARGET);
  }

  // let things settle for a second
  setup_gen();
}
void loop() 
{
#ifdef DEBUG
  static unsigned char foo = 0;
  if ((fsm_state == FSM_GEN || fsm_state == FSM_PRE_GEN) && !++foo) {
    tinySerial_printInt(fsm_state);
    tinySerial_print(PSTR(", ")); // Print separator
    tinySerial_printInt(current_adc);
    tinySerial_print(PSTR(", ")); // Print separator
    tinySerial_printInt(target_adc);
    tinySerial_print(PSTR(", ")); // Print separator
    tinySerial_printInt(cur_pwm);
    tinySerial_println(PSTR("")); // Add newline
    foo = 0;
  }
#endif
#ifdef DEBUG_PWM
  analogWrite(PIN_DEBUG, 48 * (1 + fsm_state));
#endif  
  // detect config pin...
  if (fsm_state != FSM_CONFIG_SENSE && digitalRead(PIN_CONFIG) == LOW) {
    // wait 25ms to make sure it's really low
    delay(25*64);
    if (digitalRead(PIN_CONFIG) == LOW) {
#ifdef DEBUG
      tinySerial_println(PSTR("Going into FSM_CONFIG_SENSE mode..."));
#endif      
      fsm_state = FSM_CONFIG_SENSE;
      training_adc = 0;
      training_cnt = 0;
      training_loop = 0;
      OCR0A = 255;
      delay(500*64);
      delay(500*64); // wait before sampling to let any stored inductance to dissipate
      TCCR0B = (TCCR0B & 0xF8) | 0x00; // Stop Timer0 (Clock source = No clock)
    }
  }

  switch (fsm_state) {
    case FSM_GEN:
      // in this mode we're good to go and can generate the programmed target voltage
      update_pwm();
      if (current_adc < TOO_LOW) {
        // a short occured jump to FSM_GEN_FAULT
        fsm_state = FSM_GEN_FAULT;
#ifdef DEBUG
        tinySerial_println(PSTR("Going from FSM_GEN to FSM_GEN_FAULT because ADC was too low."));
#endif
      }
      break;
    case FSM_GEN_FAULT:
      // a fault occurred, so let's sleep for a second, reset to 0.5V and try again
      OCR0A = 255;
      fsm_state = FSM_WAIT_RECOVERY;
      pre_gen_time = millis() + 64000; // 64x prescaler * 1000ms
      break;
    case FSM_WAIT_RECOVERY:
      if (millis() > pre_gen_time) {
          setup_gen();
#ifdef DEBUG
          tinySerial_println(PSTR("Going from FSM_WAIT_RECOVERY back to FSM_GEN"));
#endif
      }
      break;
    case FSM_CONFIG_SENSE:
      // if the pin is still low read the ADC and average
      if (digitalRead(PIN_CONFIG) == LOW) {
        if (training_loop < TRAINING_LOOPS) {
          training_adc += analogRead(PIN_ADC);
          delayMicroseconds(50); // delay to let ADC settle before next sample
          if (++training_cnt == 32) {
            training_adc >>= 5;
            training_cnt = 0;
            ++training_loop;
          }
        }
      } else {
        // pin is HIGH let's ensure it stays high for 10ms and then store
        TCCR0B = (TCCR0B & 0xF8) | 0x01; // turn TIMER0 back on
        delay(25*64);
        if (digitalRead(PIN_CONFIG) == HIGH) {
          // pin still high so let's assume it was released and store the trained data
          // only store valid training data
          if (training_adc > TOO_LOW && training_adc <= TOO_HIGH) {
#ifdef DEBUG
            tinySerial_print(PSTR("Going from FSM_CONFIG_SENSE to FSM_GEN: trained ADC == "));
            tinySerial_printInt(training_adc);
            tinySerial_println(PSTR(""));
#endif
            store_target(training_adc);
            setup_gen();
            training_adc = 0;
            training_cnt = 0;
          } else {
#ifdef DEBUG
            tinySerial_println(PSTR("Invalid sensed voltage during training, going to FSM_GEN state."));
#endif
            setup_gen();
          }
        }
      }
  }
}
