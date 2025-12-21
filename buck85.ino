/*
 * =================================================================================
 * PROJECT: Buck85 - ATtiny85 Programmable Buck Converter
 * VERSION: 1.0.0
 * EFFICIENCY: ~79-83% @ 5V Input (at a max of 1A output, gets about +20C at the diode/mosfet with a 1A load)
 * =================================================================================
 * * DESCRIPTION:
 * A software-controlled DC-DC Buck Converter using an ATtiny85. It features a 
 * "Training Mode" to set target voltages via an external reference and uses a 
 * linear + non-linear control loop to maintain stability under load.
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

#define NLINEAR_GAIN (4)           // non-linear gain
#define LINEAR_GAIN (8)            // linear gain
#define L_RATIO (192)              // linear weight (out of 256)
#define NL_RATIO (256 - L_RATIO)   // non-linear weight

// use Timer1 for 64MHz
#define USE_TIMER1

// can't use Timer1 if debuging is enabled because Timer1 will be outputting to OC1A and !OC1A pins (PB1 and PB0)
#if defined(DEBUG) || defined(DEBUG_PWM)
#undef USE_TIMER1
#endif

#ifndef USE_TIMER1
#define MS_TO_TICKS(x) ((x) * 32)      // 32us period means there are about 32 ticks per ms
#define EFFORT_TO_PWM(x) (255 - (x))   // normal PWM with a P-CH we want LOW for ON
#define PWM_CTR_REG OCR0A              // Timer Compare Register
#else
#define MS_TO_TICKS(x) ((x) * 64)      // 16us period means there are about 64 ticks per ms
#define EFFORT_TO_PWM(x) ((x))         // inverted OC1A PWM with a P-CH we want HIGH for ON
#define PWM_CTR_REG OCR1A              // Timer Compare Register
#endif

#define TRAINING_LOOPS 4 // how many times do we average the training adc value before stopping

enum {
   FSM_GEN,
   FSM_GEN_FAULT, // turn off mosfet to disable output because we detected a fault
   FSM_CONFIG_SENSE, // sense and average the ADC code 
   FSM_CONFIG_SENSE_WAIT, // initial startup waiting for training to start
   FSM_WAIT_RECOVERY, // state to enter when fault occurs
};

// Timing and System Variables
volatile unsigned long control_ticks = 0;  // Increments every PWM period (32us(Timer0) or 16us(Timer1))
long control_effort = 224L << 8;  // Start duty (scaled by 256)

unsigned char 
  training_loop,
  fsm_state,
  training_cnt,
  hist[256];
unsigned hist_idx;
unsigned long pre_gen_time;
int ramped_adc, current_adc, target_adc, training_adc = 0;

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
  delayMicroseconds(5000); // Small startup delay
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

#ifndef USE_TIMER1
// PWM TIMER0 interrupt, this fires every 32uS (31.25KHz)
ISR(TIM0_OVF_vect) {
#else
// PWM TIMER1 interrupt, this fires every 16uS (62.5KHz)
// TODO: fill in vector name for Timer1 overflow
ISR(TIMER1_OVF_vect) {
#endif
    control_ticks++;
}

void update_pwm()
{
  if (fsm_state != FSM_GEN || (ADCSRA & _BV(ADSC))) {
    return;
  }

  // 1. read ADC
  int current_adc = ADCW;

  // 2. Soft Target Ramping
  if (ramped_adc < target_adc) ramped_adc++;
  else if (ramped_adc > target_adc) ramped_adc--;

  if (current_adc > ramped_adc + 25) { 
      // Massive overshoot detected! 
      // Clear history to middle-ground to drop duty cycle immediately
      for (int i = 0; i < 256; i++) hist[i] = 0x88; 
      control_effort -= (64<<8); // Hard drop to PWM effort
      if (control_effort < 256) {
        control_effort = 256;
      }
  } else if (ramped_adc > current_adc + 25) {
    // massive undershoot
    control_effort += (4<<8);
    if (control_effort > 65024L) {
      control_effort = 65024L;
    }
  }

  // 3. History Indexing & Table Update (Using notched signal)
  if (abs(current_adc - ramped_adc) > 1) {
    hist_idx = ((hist_idx << 1) | (current_adc < ramped_adc)) & 0x1FF;
  }

  uint16_t byte_idx = hist_idx >> 1;
  uint8_t raw_byte = hist[byte_idx];
  uint8_t nibble = (hist_idx & 1) ? (raw_byte >> 4) : (raw_byte & 0x0F);
  uint8_t old_nibble = nibble;

  if (current_adc < ramped_adc && nibble < 15) nibble++;
  else if (current_adc > ramped_adc && nibble > 0) nibble--;

  if (hist_idx & 1) hist[byte_idx] = (raw_byte & 0x0F) | (nibble << 4);
  else             hist[byte_idx] = (raw_byte & 0xF0) | nibble;

  // 4. Hybrid Control Execution
  long delta = 0;
  if (current_adc < ramped_adc) {
    delta = (long)L_RATIO * LINEAR_GAIN;
    if (nibble > 8) {
      delta += (long)NL_RATIO * NLINEAR_GAIN;
    }
  } else if (current_adc > ramped_adc) {
    delta = (long)L_RATIO * -LINEAR_GAIN;
    if (nibble < 8) {
      delta -= (long)NL_RATIO * NLINEAR_GAIN;
    }
  }
  delta >>= 8;

  // 5. Apply, Clamp & Update Output
  long new_effort = (long)control_effort + delta;
  if (new_effort > 65024L) new_effort = 65024L;
  if (new_effort < 256L)   new_effort = 256L;
  control_effort = new_effort;

  PWM_CTR_REG = EFFORT_TO_PWM((uint8_t)(control_effort >> 8));
  ADCSRA |= (1 << ADSC); 
}

void setup_gen()
{
  cli(); // Disable interrupts while resetting state
  
  fsm_state = FSM_GEN; 
  load_target();
  
  // 1. Reset non-linear history
  unsigned x;
  for (x = 0; x < 256; x++) {
    hist[x] = 0x88; // middle value in both nibbles
  }
  hist_idx = 0;
  
  // 2. Start with MOSFET fully OFF (Effort 0 = OCR0A 255)
  control_effort = 0;
  ramped_adc = 32; // initially target a low voltage and climb to the real target
  PWM_CTR_REG = EFFORT_TO_PWM(control_effort >> 8); 

  // 3. Ensure ADC is ready and kick off the first sample
  ADCSRA |= _BV(ADEN)| _BV(ADSC); 

  sei(); // Re-enable interrupts
  delayMicroseconds(50000); // Give the LC filter a moment to settle
}

void setup() {
  cli();


#ifdef DEBUG  
  tinySerial_begin();
  tinySerial_println(PSTR("\r\n\r\nHello from Buck85\r\n"));
#endif
#ifdef DEBUG_PWM
  DDRB |= (1 << PIN_DEBUG);
  PORTB &= ~(1 << PIN_DEBUG);
#endif

#ifndef USE_TIMER1
  // use Timer0 at 8MHz / 1 / 256 == 31.25KHz
  // disable timer1
  TCCR1 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12) | _BV(CS13)); // Stop Timer1

  // disable the mosfet
  DDRB |= (1 << PIN_PWM);

  // 2. Configure Timer0 for Fast PWM on OC0A (PB0)
  // COM0A1 = 1: Non-inverting PWM (Clear on match, set at BOTTOM)
  // WGM01 & WGM00 = 1: Fast PWM Mode
  TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);

  // 3. Set the frequency (Prescaler)
  TCCR0B = (TCCR0B & 0xF8) | 0x01; 

  // 4. Enable Timer0 Overflow Interrupt
  TIMSK |= _BV(TOIE0);
#else
  // --- Timer1 High Speed PWM Setup ---
  // 1. Enable the PLL and wait for it to lock
  PLLCSR |= _BV(PLLE);               // Enable PLL
  delayMicroseconds(100);            // Short wait for PLL to stabilize
  while (!(PLLCSR & _BV(PLOCK)));    // Wait for PLOCK bit to be set
  PLLCSR |= _BV(PCKE);               // Enable PLL as PCK (64MHz)

  // 2. Configure Timer1 for PWM on PB0 (OC1A)
  // PWM1A = 1: Enable PWM mode based on comparator OCR1A
  // COM1A0 = 1: Inverted Mode (Clear on Match, Set at Bottom, but inverted output)
  // This means as OCR1A increases, the pin spends MORE time LOW (ON for P-CH)
  TCCR1 = _BV(PWM1A) | _BV(COM1A0) | _BV(CS11) | _BV(CS10); 
  // CS11|CS10 = Prescaler /4. Result: 64MHz / 4 / 256 = 62.5kHz

  // 3. Disable Timer0 to save power/cycles (Optional since we use Timer1 ticks now)
  TCCR0B = 0; 
  
  // 4. Enable Timer1 Overflow Interrupt
  TIMSK |= _BV(TOIE1);

  // 5. Ensure PB0 is an output
  DDRB |= (1 << PIN_PWM);
#endif

  // 5. Initial state: OFF for P-Channel (255 = Gate High)
  control_effort = 0 << 8;
  ramped_adc = 32;
  PWM_CTR_REG = EFFORT_TO_PWM((control_effort) >> 8);

  // configure config pin
  pinMode(PIN_CONFIG, INPUT_PULLUP);

  // configure ADC to use internal ref
  pinMode(PIN_ADC, INPUT);

  // Prescaler 64 is binary 110 (ADPS2=1, ADPS1=1, ADPS0=0)
  ADCSRA &= ~(_BV(ADPS0)); // Clear ADPS0
  ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); // Set ADPS2 and ADPS1 and ADEN
  ADMUX = _BV(REFS1) | _BV(MUX1);

  // Trigger NEXT ADC Conversion
  ADCSRA |= (1 << ADSC);

  sei();

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
  OCR1B = 32 * (1 + fsm_state);
#endif  
  // detect config pin...
  if (fsm_state != FSM_CONFIG_SENSE && digitalRead(PIN_CONFIG) == LOW) {
    // wait 25ms to make sure it's really low
    delayMicroseconds(25000);
    if (digitalRead(PIN_CONFIG) == LOW) {
#ifdef DEBUG
      tinySerial_println(PSTR("Going into FSM_CONFIG_SENSE mode..."));
#endif      
      fsm_state = FSM_CONFIG_SENSE_WAIT;
      training_adc = 0;
      training_cnt = 0;
      training_loop = 0;
      PWM_CTR_REG = EFFORT_TO_PWM(255);
      pre_gen_time = control_ticks + MS_TO_TICKS(1000);
    }
  }

  switch (fsm_state) {
    case FSM_GEN:
      // in this mode we're good to go and can generate the programmed target voltage
      update_pwm();
      if (current_adc < TOO_LOW) {
        // a short occured jump to FSM_GEN_FAULT
        //fsm_state = FSM_GEN_FAULT;
#ifdef DEBUG
        tinySerial_println(PSTR("Going from FSM_GEN to FSM_GEN_FAULT because ADC was too low."));
#endif
      }
      break;
    case FSM_GEN_FAULT:
      // a fault occurred, so let's sleep for a second, reset to 0.5V and try again
      control_effort = 255 << 8;
      PWM_CTR_REG = EFFORT_TO_PWM(control_effort >> 8);
      fsm_state = FSM_WAIT_RECOVERY;
      pre_gen_time = control_ticks + MS_TO_TICKS(1000);
      break;
    case FSM_WAIT_RECOVERY:
      if (control_ticks >= pre_gen_time) {
          setup_gen();
#ifdef DEBUG
          tinySerial_println(PSTR("Going from FSM_WAIT_RECOVERY back to FSM_GEN"));
#endif
      }
      break;
    case FSM_CONFIG_SENSE_WAIT:
      if (control_ticks >= pre_gen_time) {
        fsm_state = FSM_CONFIG_SENSE;
      }
      break;
    case FSM_CONFIG_SENSE:
      // if the pin is still low read the ADC and average
      if (digitalRead(PIN_CONFIG) == LOW) {
        if (training_loop < TRAINING_LOOPS) {
          ADCSRA |= (1 << ADSC);
          while (ADCSRA & (1 << ADSC));
          training_adc += ADCW;
          delayMicroseconds(50); // delay to let ADC settle before next sample
          if (++training_cnt == 32) {
            training_adc >>= 5;
            training_cnt = 0;
            ++training_loop;
          }
        }
      } else {
        // pin is HIGH let's ensure it stays high for 25ms and then store
        delayMicroseconds(25000);
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
