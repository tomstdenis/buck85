/*
 * =================================================================================
 * PROJECT: Buck85 - ATtiny85 Programmable Buck Converter
 * VERSION: 1.0.0
 * EFFICIENCY: ~79-83% @ 5V Input (at a max of 1A output, gets about +20C at the diode/mosfet with a 1A load)
 * =================================================================================
 * * DESCRIPTION:
 * A software-controlled DC-DC Buck Converter using an ATtiny85. It features a 
 * "Training Mode" to set target voltages via an external reference and uses a 
 * PID-lite control loop to maintain stability under load.
 * * SAFETY NOTE: 
 * You MUST program the ATtiny85 with BOD (Brown-Out Detection) set to 4.3V 
 * to ensure the MOSFET shuts down safely during power loss.
 * * HOW TO CONFIGURE (Training Mode):
 * 1. Apply your desired target voltage to the OUTPUT pads (as a reference).
 * 2. Short the CONFIG pad to GND for at least 2 seconds.
 * 3. Release the CONFIG pad and remove your external reference.
 * 4. The device will now store this ADC value in EEPROM and target it on startup.
 * Note: If jumping from a very high to a very low voltage, you may need 
 * to repeat this 2-3 times to allow the output caps to fully discharge.
 * * HARDWARE SPECIFICATIONS:
 * - Input: 5V DC (Stable)
 * - Output Range: ~0.6V to 4.6V (Limited by Diode Vf and MOSFET overhead)
 * - Max Load: 1.0A Continuous
 * - Control Loop: 31.25kHz Fast PWM (8-bit) / 10-bit ADC Feedback
 * - Resolution: ~4.88mV (ADC) / ~19.5mV (PWM step)
 * * COMPONENT BILL OF MATERIALS (As Tested):
 * - MCU: ATtiny85 (Internal 8MHz clock)
 * - MOSFET: DMG2301L (P-Channel)
 * - Diode: DSS24 Schottky (40V, 2A)
 * - Inductor: 47uH Shielded (1.5A Saturation, <160mOhm DCR)
 * - Feedback: 5:1 Voltage Divider (35.7k / 10k 1% resistors)
 * - Gate Drive: 100 Ohm series resistor + 10k Ohm pull-up to VCC
 * - Filtering:
 *    - output: 2 X7R 22uF caps, 1 220uF aluminum cap
 *    - input: 1 X7R 22uF cap, 1 220uF aluminum cap
 * * PIN MAP:
 *          /------------\
 * N/C    - | PB5    VCC | - 5V Input
 * CONFIG - | PB3    PB2 | - N/C
 * FB     - | PB4    PB1 | - DEBUG / Serial TX
 * GND    - | GND    PB0 | - PWM Out (to MOSFET Gate)
 *          \------------/
 * * TROUBLESHOOTING:
 * - High Ripple (Vpp): Check inductor saturation and ESR of output caps. 
 * Ensure ANTI_WINDUP_MS is tuned to your specific LC filter inertia.
 * - Fault Mode: If output drops below 64mV (TOO_LOW), the system enters 
 * FSM_GEN_FAULT to protect against shorts.
 * =================================================================================
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
#define ANTI_WINDUP_MS (128+32)    // how many "millis()" to wait between updating the pwm. Recall to scale by 64 due to prescaling (160 == 2.5ms which seems to work for me)

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
  static int last_adc = 0;
  static unsigned long last_sample_time = 0;
  static byte persistent_error_timer = 0;
  int new_pwm, delta, slope;

  // read the ADC and compute delta
  current_adc = analogRead(PIN_ADC);
  delta = (int)current_adc - (int)target_adc;

  // we want to only update the PWM signal ever ANTI_WINDUP_MS window
  // though there are times we're way off base we might want to update sooner

  // DYNAMIC TIMING:
  // If the error is huge (>100mV), bypass the wait and react now!
  // Otherwise, use the standard "stable" window.
  bool urgent = (abs(delta) > 20);  

  if (!urgent && millis() - last_sample_time < (ANTI_WINDUP_MS)) return;
  last_sample_time = millis();

  slope = (int)current_adc - last_adc;
  last_adc = current_adc;

  new_pwm = (int)cur_pwm;

  // 1. DAMPING (D-Term)
  // We apply the "Brake" first to stabilize the slope.
  if (slope > 1) {
    new_pwm += 1;
  }
  if (slope < -1) {
    new_pwm -= 1;
  }

  // 2. SMART DEADZONE & HYSTERESIS
  if (abs(delta) > 4) {
    // Proportional move for significant errors
    if (delta > 0) {
      new_pwm += 1; 
    } else {
      new_pwm -= 1;
    }
    persistent_error_timer = 0; // Reset timer
  } else if (abs(delta) >= 2) {
    // For small errors, don't jitter. Only nudge if it stays off-target.
    persistent_error_timer++;
    if (persistent_error_timer > 3) { 
        if (delta > 0) {
          new_pwm += 1;
        } else {
          new_pwm -= 1;
        }
        persistent_error_timer = 0;
    }
  } else {
    persistent_error_timer = 0;
  }

  // 3. INTEGRAL (Fine Tuning)
  // We only let the integral work when we are very close to target.
  if (abs(delta) < 10) {
    integral_sum += delta;
  } else {
    integral_sum = (integral_sum * 1) / 2; // Quickly bleed off integral if error is large
  }
  
  if (integral_sum > 120) {
    new_pwm += 1;
    integral_sum = 0;
  }
  if (integral_sum < -120) {
    new_pwm -= 1;
    integral_sum = 0;
  }

  // Saturate
  if (new_pwm > 255) {
    new_pwm = 255;
  }
  if (new_pwm < 0) {
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
