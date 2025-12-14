/*

Simple? 5 volt buck convert that uses a P-channel mosfet attached to PIN_PWM.

It uses a roughly 5:1 divider on PIN_ADC so we can map 5 volts to below the 1.1 internal reference
voltage.

The buck converter starts in a "pre gen" phase where it tries to output roughly 500mV for 150ms.
If it can't or it detects a short (because PIN_ADC reads below TOO_LOW) it goes into a fault state.
If it can hold it it goes into "gen" mode where it tries to output the stored value.

If a fault is detected during "pre gen" or "gen" phases it goes into fault state.  Where it
turns off the mosfet, waits 1 second, then goes back into "pre gen" mode trying to output 500mV.  This 
lets it "self recover" from intermittent shorts or excessive loads.

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

*/

#include <EEPROM.h>

// use PB1 as a PWM signal to debug 
#define DEBUG

#ifdef DEBUG
#include <SoftwareSerial.h>
SoftwareSerial softSerial(PB1, PB2); // PB2==TX so that's what you should read
#endif
#define PIN_CONFIG PB3
#define PIN_PWM PB0 // if you change this you need to change the timer code too in setup()
#define PIN_ADC PB4

#define TOO_LOW 13                 // anything below 64mV is considered a short (13 * 5000/1023 == 63.53mV)
#define TOO_HIGH 950               // anything above is considered illsuited for the circuit(950 * 500/1023 == 4643mV)

#define PRE_GEN_MAX_DELTA 4        // how close to the target training voltage before going into GEN phase (4 * 5000/1023 == 19.55mV)
#define PRE_GEN_TARGET 133         // Pre generation we target about 650mV (133 * 5000/1023 == 650mV)
#define PRE_GEN_HOLD 150           // how many milliseconds to hold

#define ADC_AVERAGE 4              // how many samples to read at once (too many and it'll take too long to reach a stable target, too little and you'll get noise...)
#define ADC_AVERAGE_BITS 2         // log2(ADC_AVERAGE)
#define ADC_HYSTERESIS 2           // how many ADC codes to assume are close enough
#define PWM_BIG_STEP 8             // take large steps of 8 * 5000/1023 = 39.1mV if the error is that large
#define PWM_SMALL_STEP 1           // take smaller steps if the error is small

enum {
   FSM_PRE_GEN=0, // low output before target output to detect shorts
   FSM_GEN,
   FSM_GEN_FAULT, // turn off mosfet to disable output because we detected a fault
   FSM_CONFIG_SENSE, // sense and average the ADC code 
};

unsigned char fsm_state, cur_pwm = 224, training_cnt = 0; // set cur_pwm to mostly on because it's a P-channel so a HIGH means turn the mosfet off
unsigned long pre_gen_time;
unsigned current_adc, target_adc, training_adc = 0;
int integral_sum = 0; // accumulated error over time

void load_target()
{
  target_adc = ((unsigned)EEPROM.read(1) << 8) | ((unsigned)EEPROM.read(2));
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
  int new_pwm, delta, sign;
  uint32_t sum;
  unsigned char x;

  for (sum = x = 0; x < ADC_AVERAGE; x++) {
    sum += analogRead(PIN_ADC);
  }
  sum >>= ADC_AVERAGE_BITS;
  current_adc = sum;

  delta = (int)current_adc - (int)target_adc; // how far off are we in steps of 5000mV / 1023 == 4.88mV
  integral_sum += delta;
  new_pwm = (int)cur_pwm;
  if (abs(delta) > ADC_HYSTERESIS) {
    // calc signed delta
    if (delta < 0) {
      sign = -1;
    } else {
      sign = 1;
    }

    // adjust signed pwm count
    if (abs(delta) >= PWM_BIG_STEP) {
      new_pwm += PWM_BIG_STEP * sign;
    } else if (delta) {
      new_pwm += PWM_SMALL_STEP * sign;
    }
  }
  
  // saturate the integral error so we avoid winding up too hard
  if (integral_sum > 127) {
    integral_sum = 127;
  } else if (integral_sum < -128) {
    integral_sum = -128;
  }
  new_pwm += integral_sum >> 5;

  // saturate the PWM value
  if (new_pwm > 255) {
    new_pwm = 255;
  } else if (new_pwm < 0) {
    new_pwm = 0;
  }
  cur_pwm = new_pwm;
  analogWrite(PIN_PWM, cur_pwm);
}

void setup_pregen()
{
  integral_sum = 0;
  fsm_state = FSM_PRE_GEN; // go back to pre-gen phase
  target_adc = PRE_GEN_TARGET;
  cur_pwm = 192; // 75% duty cycle to ensure the mosfet is on (25% of the time)
  analogWrite(PIN_PWM, cur_pwm);
  delay(5); // let the circuit pre-charge the capacitors a bit before reading the feedback pin
  pre_gen_time = millis() + PRE_GEN_HOLD; // must sustain low voltage for time before proceeding to target
}

void setup() {
  // disable the mosfet
  pinMode(PIN_PWM, OUTPUT);
  TCCR0B = (TCCR0B & 0xF8) | 0x01; // set PIN_PWM to 31.25KHz
  analogWrite(PIN_PWM, 255); // turn off MOSFET
#ifdef DEBUG  
  softSerial.begin(1200);
#endif
  // configure config pin
  pinMode(PIN_CONFIG, INPUT_PULLUP);
  // configure ADC to use internal ref
  analogReference(INTERNAL);

  // configure EEPROM
  if (EEPROM.read(0) != 0xA5) {
    // configure to a low voltage (around 0.5V out of 5V)
    store_target(PRE_GEN_TARGET);
  }

  // let things settle for a second
  setup_pregen();
}

void loop() 
{
#ifdef DEBUG
  static unsigned long foo = 0;
  if (++foo == 1024) {
    softSerial.print("FSM state: ");
    softSerial.println(fsm_state);
    softSerial.print("ADC reading == ");
    softSerial.println(current_adc);
    softSerial.print("PWM == ");
    softSerial.println(cur_pwm);
    foo = 0;
  }
#endif  
  // detect config pin...
  if (digitalRead(PIN_CONFIG) == LOW) {
    // wait 10ms to make sure it's really low
    delay(10);
    if (digitalRead(PIN_CONFIG) == LOW) {
      fsm_state = FSM_CONFIG_SENSE;
    }
  }

  switch (fsm_state) {
    case FSM_PRE_GEN:
      // the "pre-gen" phase tries to hold a low voltage for some time to detect shorts on boot up
      update_pwm(); // adjust pwm signal
      if (current_adc < TOO_LOW) {
        // a short occured jump to FSM_GEN_FAULT
#ifdef DEBUG
        softSerial.println("Going from FSM_PRE_GEN to FSM_GEN_FAULT because ADC was too low.");
#endif
        fsm_state = FSM_GEN_FAULT;
      } else {
        if (millis() >= pre_gen_time && abs((int)current_adc - (int)target_adc) < PRE_GEN_MAX_DELTA) {
          fsm_state = FSM_GEN;
          load_target();
          integral_sum = 0;
        }
      }
      break;
    case FSM_GEN:
      // in this mode we're good to go and can generate the programmed target voltage
      update_pwm();
      if (current_adc < TOO_LOW) {
        // a short occured jump to FSM_GEN_FAULT
        fsm_state = FSM_GEN_FAULT;
#ifdef DEBUG
        softSerial.println("Going from FSM_GEN to FSM_GEN_FAULT because ADC was too low.");
#endif
      }
      break;
    case FSM_GEN_FAULT:
      // a fault occurred, so let's sleep for a second, reset to 0.5V and try again
      analogWrite(PIN_PWM, 255); // turn off MOSFET
      delay(1000);
      setup_pregen();
#ifdef DEBUG
        softSerial.println("Going from FSM_GEN_FAULT back to FSM_PRE_GEN");
#endif
      break;
    case FSM_CONFIG_SENSE:
      // if the pin is still low read the ADC and average
      if (digitalRead(PIN_CONFIG) == LOW) {
        training_adc += analogRead(PIN_ADC);
        if (++training_cnt == 64) {
          training_adc >>= 6;
          training_cnt = 0;
        }
      } else {
        // pin is HIGH let's ensure it stays high for 10ms and then store
        delay(10);
        if (digitalRead(PIN_CONFIG) == HIGH) {
          // pin still high so let's assume it was released and store the trained data
          if (training_cnt) {
            training_adc /= training_cnt;
          }
          // only store valid training data
          if (training_adc > TOO_LOW && training_adc <= TOO_HIGH) {
#ifdef DEBUG
            softSerial.println("Going from FSM_CONFIG_SENSE to FSM_PRE_GEN.");
#endif
            store_target(training_adc);
            setup_pregen();
            training_adc = 0;
            training_cnt = 0;
          }
        }
      }
  }
}
