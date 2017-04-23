/*
 Example sketch for the PS4 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */


#include <PS4USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif
#include <avr/wdt.h>

#define ADC_AVG_LEN 4
#define PEDAL_AVG_LEN 8
#define MAX_PEDAL_PERIOD 100000
#define MIN_PEDAL_PERIOD  15000

#define EASY_PEDAL_MIN 0.0
#define EASY_PEDAL_MAX 10.0

#define MED_PEDAL_MIN 0.5
#define MED_PEDAL_MAX 40.0

#define HARD_PEDAL_MIN 1.0
#define HARD_PEDAL_MAX 90.0

#define LED_DEBUG 0

#define MAX_RESET 7
#define GIMX_RESET A5

uint8_t difficulty = 1;
uint16_t bit_toggle = 4;
uint16_t steer_min = 30608;
uint16_t steer_max = 42600;
USB Usb;
PS4USB PS4(&Usb);
uint8_t bit_toggle_flag = 0;
bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
uint32_t loop_count;
uint8_t send_count;
uint8_t auth_state;
bool rx_flag;
// uint8_t gear = 1;
unsigned long last_tx;
unsigned long last_rx;
uint8_t resp[64] = {0};
uint8_t check_response[16] = {0};
uint8_t send_check_response;
uint8_t report_len;
uint8_t b;
bool toggle;
bool tog;
int8_t rx_counter;
uint8_t auth_msg_count = 0;
bool auth_rx_flag = 0;
uint8_t auth_cycles = 0;
uint16_t pedal_int = 0;
// volatile int pedal_tmp = 0;
// volatile int pedal_period = 0;
double random_double = 0.0;
uint16_t random_counter = 0;
bool last_thing = 0;
uint8_t buffer_out[64] = {0};
uint8_t buffer_out_len = 0;
uint8_t buffer_out_flag = 0;
uint8_t buffer_out_micros = 0;
uint8_t update_flag = 0;
uint8_t bytenum = 0xFF - 96;
unsigned long ps4_up;
unsigned long last_report_tx;
unsigned long last_report_sent_micros;
// extern volatile unsigned long timer0_millis;
uint8_t byte_in;
uint8_t head_chall[6] = {0xf0, 0x03, 0x00, 0x00, 0x40, 0x00};
uint8_t head_check[6] =  {0xf2, 0x03, 0x00, 0x00, 0x10, 0x00};
uint8_t head_boot0[2] = {0x11, 0x01};
uint8_t head_boot1[2] = {0x22, 0x01};
uint8_t head_boot2[2] = {0x33, 0x01};
uint8_t head_reqst[6] = {0xf1, 0x03, 0x00, 0x00, 0x40, 0x00};
uint8_t head_match = 0;
uint8_t match_chall = 0;
uint8_t match_check = 0;
uint8_t match_reqst = 0;
uint8_t match_boot0 = 0;
uint8_t match_boot1 = 0;
uint8_t match_boot2 = 0;
bool tx_report_flag = 0;
uint8_t main_state = 0;
unsigned int tcnt2;
bool difficulty_change_flag = 0;
// bool gear_change_flag = 0;
volatile uint32_t adc_avg_tmp;
volatile uint8_t adc_loop_count;
volatile uint8_t adc_channel;
volatile uint16_t adc_tmp;
volatile uint16_t adc_tmp_steer[ADC_AVG_LEN];
volatile uint16_t adc_tmp_brake[ADC_AVG_LEN];
volatile uint32_t pedal_tmp_avg[PEDAL_AVG_LEN];
uint8_t pedal_tmp_counter = 0;
bool connected = 0;


uint8_t report[66] = {
  0xff, 0x40, 0x01, 0x80, 0x80, 0x80, 0x80, 0x08, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t ps4_out[64] = {0};

// ff 40 01 80 80 80 80 08 00 00 00 00 00 00 00 00 - 16
// 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 - 32
// 00 00 00 00 00 00 00 00 00 00 00 00 00 SL SH AL - 48
// AH BL BH ff ff 00 ff ff 00 00 00 00 00 00 00 00
// 00 00


// Initialization
void setupADC(){

  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;

  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= B01000000;

  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;

  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
  ADMUX |= B00000000;
  // ADMUX |= B00001000; // Binary equivalent

  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;

  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= B00100000;

  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= B11111000;

  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= B00000111;

  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B00001000;

  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  sei();

  // Kick off the first ADC
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=B01000000;

}

// void setup_regularinterrupt() {
//    /* First disable the timer overflow interrupt while we're configuring */
//   TIMSK2 &= ~(1<<TOIE2);

//   /* Configure timer2 in normal mode (pure counting, no PWM etc.) */
//   TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
//   TCCR2B &= ~(1<<WGM22);

//   /* Select clock source: internal I/O clock */
//   ASSR &= ~(1<<AS2);

//   /* Disable Compare Match A interrupt enable (only want overflow) */
//   TIMSK2 &= ~(1<<OCIE2A);

//   TCCR2B = (TCCR2B & 0b11111000) | 0x05; //div 128
//   //125000 - 8us


//    Now configure the prescaler to CPU clock divided by 128
//   // TCCR2B |= (1<<CS22)  | (1<<CS20); // Set bits
//   // TCCR2B &= ~(1<<CS21);             // Clear bit

//   /* We need to calculate a proper value to load the timer counter.
//    * The following loads the value 131 into the Timer 2 counter register
//    * The math behind this is:
//    * (CPU frequency) / (prescaler value) = 125000 Hz = 8us.
//    * (desired period) / 8us = 125.
//    * MAX(uint8) + 1 - 125 = 131;
//    */
//   /* Save value globally for later reload in ISR */
//   tcnt2 = 131;

//   /* Finally load end enable the timer */
//   TCNT2 = tcnt2;
//   TIMSK2 |= (1<<TOIE2);
// }

uint16_t avg_steer() {
  adc_avg_tmp = 0;
  for (uint8_t i = 0; i < ADC_AVG_LEN; i++) adc_avg_tmp += adc_tmp_steer[i];
  return adc_avg_tmp >> 2;
}

uint16_t avg_brake() {
  adc_avg_tmp = 0;
  for (uint8_t i = 0; i < ADC_AVG_LEN; i++) adc_avg_tmp += adc_tmp_brake[i];
  return 0xFFFF - (adc_avg_tmp >> 2);
}

// void set_report_bit(uint16_t num) {
//   uint8_t res = (1 << (num & 0b11));
//   if (!digitalRead(A3)) report[num >> 2] |= res;
//   else report[num >> 2] &= ~res;
// }

void watchdogSetup(void) {
  cli();  // disable all interrupts
  wdt_reset(); // reset the WDT timer
  /*
  WDTCSR configuration:
  WDIE = 1: Interrupt Enable
  WDE = 1 :Reset Enable
  WDP3 = 0 :For 2000ms Time-out
  WDP2 = 1 :For 2000ms Time-out
  WDP1 = 1 :For 2000ms Time-out
  WDP0 = 1 :For 2000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  sei();
}

void setup() {
  watchdogSetup();
  // setup_regularinterrupt();
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, OUTPUT);
  pinMode(2, INPUT);
  pinMode(MAX_RESET, OUTPUT );

  digitalWrite(GIMX_RESET, LOW);
  toggle = 0;
  tog = 0;
  loop_count = 0;
  send_count = 0;
  rx_flag = 0;
  update_flag = 0;
  auth_state = 0;
  // micros_adjust = 0;
  report_len = sizeof(report);
  rx_counter = 0;
  auth_msg_count = 0;
  auth_rx_flag = 0;
  send_check_response = 0;
  main_state = 0;
  last_report_tx = 0;
  wdt_reset();
  Serial.begin(500000);
  // Serial.begin(9600);
  // Serial.print("\nSerial -up\n");
  digitalWrite( MAX_RESET, HIGH);  //release MAX3421E from reset
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    while (1) {
    // Serial.print("Halted, ");
    digitalWrite( MAX_RESET, LOW);  //release MAX3421E from reset
    }
  }
  b = 0;

  Serial.write(0x55);
  Serial.write(0x00);

  // Serial.begin(9600);
  setupADC();
  // attachInterrupt(0, timeit, FALLING);
  // timeit();
  attachInterrupt(0, timeit, FALLING);
  digitalWrite(GIMX_RESET, HIGH); // Release the GIMX adaptor from RESET
  wdt_reset();
}

uint32_t pedal_interval;
uint32_t last_time;

void timeit() {
  pedal_interval = micros() - last_time;
  last_time = micros();
  // if (digitalRead(2))
  // else attachInterrupt(0, timeit, RISING);
}

// void rising() {
//   // attachInterrupt(0, falling, FALLING);
//   //Pedalling
//   report[48] = millis() - pedal_tmp;
//   // pedal_period = millis() - pedal_tmp;
//   pedal_tmp = millis();
//   attachInterrupt(0, rising, RISING);
// }

bool matches_head(uint8_t byte_in, uint8_t header[], uint8_t len, uint8_t &match) {
  if (byte_in == header[0]) match = 1;
  else if (byte_in == header[match]) match++;
  else match = 0;
  return match == len;
}

bool matches_chall(uint8_t byte_in) {
  return matches_head(byte_in, head_chall, 6, match_chall);
}

bool matches_check(uint8_t byte_in) {
  return matches_head(byte_in, head_check, 6, match_check);
}

bool matches_reqst(uint8_t byte_in) {
  return matches_head(byte_in, head_reqst, 6, match_reqst);
}

bool matches_boot0(uint8_t byte_in) {
  return matches_head(byte_in, head_boot0, 2, match_boot0);
}

bool matches_boot1(uint8_t byte_in) {
  return matches_head(byte_in, head_boot1, 2, match_boot1);
}

bool matches_boot2(uint8_t byte_in) {
  return matches_head(byte_in, head_boot2, 2, match_boot2);
}

uint32_t pedal_period() {
  uint32_t out;
  if (micros() - last_time > MAX_PEDAL_PERIOD) {
    out = MAX_PEDAL_PERIOD;
  }
  else if (pedal_interval > MAX_PEDAL_PERIOD) {
    out = MAX_PEDAL_PERIOD;
  }
  else {
    out = pedal_interval;
  }
  pedal_tmp_avg[pedal_tmp_counter++] = out;
  out = 0.0;
  for (uint8_t i = 0; i < PEDAL_AVG_LEN; i++) {
    out = out + pedal_tmp_avg[i];
  }
  out = out / PEDAL_AVG_LEN;
  if (pedal_tmp_counter == PEDAL_AVG_LEN) pedal_tmp_counter = 0;
  return out;
}

double pedal_scaled(uint32_t period) {
  random_double = (1.0 / period) * 1000000.0;
  double max = (1.0/ MIN_PEDAL_PERIOD) * 1000000.0;
  double min = (1.0/ MAX_PEDAL_PERIOD) * 1000000.0;
  double range = max - min;
  random_double = random_double - min;
  random_double = random_double / range;
  random_double = random_double * 32.0;
  random_double = (random_double * random_double * random_double);
  random_double = random_double / 32768;
  // random_double = random_double - ((1.0 / MAX_PEDAL_PERIOD) * 1000000);
  // return ((random_double * random_double) / (((1.0/ MIN_PEDAL_PERIOD) * 1000000) * ((1.0/ MIN_PEDAL_PERIOD) * 1000000))) * 100.0;
  double out;
  if (random_double > 1.0) out = 100.0;
  else out = random_double * 100.0;

  switch (difficulty) {
    case 0:
      out = out - EASY_PEDAL_MIN;
      out = out / (EASY_PEDAL_MAX - EASY_PEDAL_MIN);
      out = out * 100.0;
      break;

    case 1:
      out = out - MED_PEDAL_MIN;
      out = out / (MED_PEDAL_MAX - MED_PEDAL_MIN);
      out = out * 100.0;
      break;

    case 2:
      out = out - HARD_PEDAL_MIN;
      out = out / (HARD_PEDAL_MAX - HARD_PEDAL_MIN);
      out = out * 100.0;
      break;
  }
  if (out > 100.0) out = 100.0;
  else if (out < 0) out = 0.0;
  return out;
}

uint16_t tx_counter = 0;

uint32_t led_counter = 0;
uint32_t led_color = 0;
double led_value = 0;
uint16_t tmp = 0;

void loop() {

  Usb.Task();


  if (main_state == 6) {

    if (buffer_out_flag && micros() >= buffer_out_micros && micros() >= (last_report_sent_micros + 2200)) {
      Serial.write(buffer_out, buffer_out_len);
      buffer_out_flag = 0;
      last_report_tx = last_report_tx + 5000;
      if (micros() >= last_report_tx) last_report_tx = last_report_tx + 5000;
    }
    else if (micros() >= last_report_tx) {
      last_report_sent_micros = micros();

      // Send out a push of the PS button to cause a connect event
      if (!connected) {
        if (tx_counter < 1000) report[9] = 0x01;
        else connected = 1;
      }

      Serial.write(report, report_len);
      tx_counter++;
      last_report_tx = last_report_tx + 5000;
    }

  }




  // Red = 0xFF0000,
  // Green = 0xFF00,
  // Blue = 0xFF,
  // Yellow = 0xFFEB04,
  // Lightblue = 0xFFFF,
  // Purple = 0xFF00FF,
  // Purble = 0xFF00FF,
  // White = 0xFFFFFF,
  // Off = 0x00,

  if (PS4.connected()) {
    wdt_reset();

    // Serial.print(PS4.getAnalogHat(LeftHatX));
    // Serial.print(",");
    // Serial.print(PS4.getAnalogHat(LeftHatY));
    // Serial.print(",");
    // Serial.print(PS4.getAnalogHat(RightHatX));
    // Serial.print(",");
    // Serial.print(PS4.getAnalogHat(RightHatY));
    // Serial.print("\n");


    // L2 & R2 as difficulty settings
    if (PS4.getButtonClick(L2) && difficulty_change_flag == 0) {
      difficulty_change_flag = 1;
      if (difficulty > 0) difficulty--;
    }
    else if (PS4.getButtonClick(R2) && difficulty_change_flag == 0) {
      difficulty_change_flag = 1;
      if (difficulty < 3) difficulty++;
    }
    else {
      difficulty_change_flag = 0;
    }

    // if (PS4.getButtonPress(TOUCHPAD)) {
    //   difficulty = (difficulty++ % 3);
    // }

    // if (!digitalRead(A4) && gear_change_flag == 0) {
    //   gear_change_flag = 1;
    //   if (gear > 0) gear--;
    // }
    // else if (!digitalRead(A3) && gear_change_flag == 0) {
    //   gear_change_flag = 1;
    //   if (gear < 7) gear++;
    // }
    // else if (digitalRead(A4) && digitalRead(A3)) {
    //   gear_change_flag = 0;
    // }

    if (main_state == 6) {
      switch (difficulty) {
        case 0:
          PS4.setLed(0xFF00);
          break;

        case 1:
          PS4.setLed(0xFFEB04);
          break;

        case 2:
          PS4.setLed(0xFF0000);
          break;
      }
    }
    else {
      // led_value = led_counter / 1000.0;
      // led_value = (sin(led_value) + 1.0) / 2.0;
      // led_value = led_value * 255;
      tmp = (uint8_t) (128.0 + 128 * sin(led_counter / 500.0));
      uint32_t leds;
      leds = tmp;
      leds = leds << 8;
      leds |= tmp;
      leds = leds << 8;
      leds |= tmp;

      // led_color = (tmp << 4);
      // led_color |= tmp << 2;
      // led_color |= tmp;
      // Serial.print(tmp);
      // Serial.print("\n");
      PS4.setLed(leds);
      led_counter++;
    }

    if (main_state == 0) {
      // noInterrupts();
      Serial.write(0x11);
      Serial.write(0x00);
      // interrupts();
      main_state = 1;
      last_rx = micros();
    }
    else if (main_state == 1) {
      if (last_rx + 1000000 <= micros()) main_state = 0;
      else {
        while(Serial.available()) {
          ps4_up = micros(); // First response from PS4
          byte_in = Serial.read();
          if (matches_boot0(byte_in)) {
            main_state = 2;
            last_rx = micros();
            break;
          }
        }
      }
    }
    else if (main_state == 2 && last_rx + 1849 <= micros()) {
      // noInterrupts();
      Serial.write(0x22);
      Serial.write(0x00);
      // interrupts();
      main_state = 3;
      last_rx = micros();
    }
    else if (main_state == 3) {
      if (last_rx + 1000000 <= micros()) main_state = 2;
      else {
        while(Serial.available()) {
          byte_in = Serial.read();
          if (matches_boot1(byte_in)) {
            main_state = 4;
            break;
            last_rx = micros();
          }
        }
      }
    }
    else if (main_state == 4 && last_rx + 840000 <= micros()) {
      // noInterrupts();
      Serial.write(0x33);
      Serial.write(0x00);
      // interrupts();
      main_state = 5;
      last_rx = micros();
    }
    else if (main_state == 5) {
      if (last_rx + 1000000 <= micros()) main_state = 4;
      else {
        while(Serial.available()) {
          byte_in = Serial.read();
          if (matches_boot2(byte_in)) {
            main_state = 6;
            last_rx = micros();
            last_report_tx = micros() + 7486;
            break;
          }
        }
      }
    }
    else if (main_state == 6) {

      // Check GIMX out for PS4 Challenge messages


      // Read in challenge messages
      while(Serial.available()) {
        byte_in = Serial.read();

        if (auth_rx_flag == 1) {
          ps4_out[rx_counter] = byte_in;
          rx_counter++;

          if (rx_counter == 64) {
            // Weve received the whole challenge part
            auth_rx_flag = 0;
            rx_counter = 0;
            loop_count++;

            if (LED_DEBUG) {
              if (ps4_out[2] == 0) PS4.setLed(0x000033);
              else if (ps4_out[2] == 1) PS4.setLed(0x000066);
              else if (ps4_out[2] == 2) PS4.setLed(0x000099);
              else if (ps4_out[2] == 3) PS4.setLed(0x0000CC);
              else if (ps4_out[2] == 4)  {
                PS4.setLed(0x0000FF);
              }
            }
            PS4.authenticate(64, ps4_out);

          }
        }
        if (matches_chall(byte_in)) {

          if (LED_DEBUG) PS4.setLed(0x000000);

          auth_rx_flag = 1;
          head_match = 0;
          rx_counter = 0;
        }
        else if ( matches_check(byte_in) ) {

          PS4.authenticate2(16, check_response);

          if (LED_DEBUG) {
            if (check_response[2] == 0x00) PS4.setLed(Green);
            else if (check_response[2] == 0x10) PS4.setLed(Yellow);
            else PS4.setLed(Red);
          }

          buffer_out[0] = 0x44;
          buffer_out[1] = 0x10;
          for (uint8_t i = 2; i < 18; i++) buffer_out[i] = check_response[i-2];
          buffer_out_len = 18;
          buffer_out_flag = 1;
          buffer_out_micros = micros() + 1200;
        }
        else if (matches_reqst(byte_in)) {

          // Request response from DS4
          PS4.authenticate3(64, resp);

          if (LED_DEBUG) {
            if (resp[2] & 1) PS4.setLed(0x444444);
            else PS4.setLed(0xFFFFFF);
          }

          buffer_out[0] = 0x44;
          buffer_out[1] = 0x40;
          for (uint8_t i = 2; i < 66; i++) buffer_out[i] = resp[i-2];
          buffer_out_len = 66;
          buffer_out_flag = 1;
          buffer_out_micros = micros() + 1200;
        }
      }

      // Read the controller state
      if (PS4.getButtonPress(UP)) report[7] = 0x00;
      else if (PS4.getButtonPress(UP) && PS4.getButtonPress(RIGHT)) report[7] = 0x01;
      else if(PS4.getButtonPress(RIGHT)) report[7] = 0x02;
      else if (PS4.getButtonPress(DOWN) && PS4.getButtonPress(RIGHT)) report[7] = 0x03;
      else if (PS4.getButtonPress(DOWN)) report[7] = 0x04;
      else if (PS4.getButtonPress(DOWN) && PS4.getButtonPress(LEFT)) report[7] = 0x05;
      else if (PS4.getButtonPress(LEFT)) report[7] = 0x06;
      else if (PS4.getButtonPress(LEFT) && PS4.getButtonPress(UP)) report[7] = 0x07;
      else  report[7] = 0x08;

      // report[7] = 0x08;

      if (PS4.getButtonPress(TRIANGLE)) report[7] |= 0x80;
      else report[7] &= ~0x80;

      if (PS4.getButtonPress(CROSS)) report[7] |= 0x20;
      else report[7] &= ~0x20;

      if (PS4.getButtonPress(SQUARE)) report[7] |= 0x10;
      else report[7] &= ~0x10;

      if (PS4.getButtonPress(CIRCLE) | !digitalRead(A2)) report[7] |= 0x40;
      else report[7] &= ~0x40;


      if (PS4.getButtonPress(PS)) report[9] = 0x01;
      else report[9] = 0x00;

      if (PS4.getButtonPress(SHARE)) report[8] |= 0x10;
      else report[8] &= ~0x10;

      if (PS4.getButtonPress(OPTIONS)) report[8] |= 0x20;
      else report[8] &= ~0x20;

      if (PS4.getButtonPress(L3)) report[8] |= 0x40;
      else report[8] &= ~0x40;

      if (PS4.getButtonPress(R3)) report[8] |= 0x80;
      else report[8] &= ~0x80;

      if (PS4.getButtonPress(L1) | !digitalRead(A4)) report[8] |= 0x01;
      else report[8] &= ~0x01;

      if (PS4.getButtonPress(R1) | !digitalRead(A3)) report[8] |= 0x02;
      else report[8] &= ~0x02;

      // if (update_flag == 0 && (micros() - ps4_up) > 3940000) {
      //   report[45] = 0xfb;
      //   report[46] = 0x7e;
      //   update_flag = 1;
      // }

      report[3] = PS4.getAnalogHat(LeftHatX);
      report[4] = PS4.getAnalogHat(LeftHatY);
      report[5] = PS4.getAnalogHat(RightHatX);
      report[6] = PS4.getAnalogHat(RightHatY);

      //Steering
      tmp = avg_steer();
      if (tmp < steer_min && tmp != 0) steer_min = tmp;
      if (tmp > steer_max) steer_max = tmp;
      tmp = ((double)((double)(tmp - steer_min) / (double)(steer_max - steer_min)) * 65535);
      report[45] = (tmp & 0xFF);
      report[46] = (tmp & 0xFF00) >> 8;

      // Brake
      tmp = avg_brake();
      report[49] = (tmp & 0xFF);
      report[50] = (tmp & 0xFF00) >> 8;

      // Gas
      tmp = pedal_scaled(pedal_period());
      tmp = (tmp / 100.0) * 65535.0;
      if (tmp > 65535) tmp = 65535.0;
      else if(tmp < 0) tmp = 0.0;
      pedal_int = 65535 - (uint16_t) tmp;
      report[47] = pedal_int & 0xFF;
      report[48] = pedal_int >> 8;

    }
  }
  else main_state = 0;

}


// Interrupt service routine for the ADC completion
ISR(ADC_vect){
  if (adc_channel == 0) {
    adc_tmp = ADCL;
    adc_tmp |= (ADCH << 8);
    adc_tmp = adc_tmp << 6;
    adc_tmp_steer[adc_loop_count] = adc_tmp;
    ADMUX &= B11111110;
    adc_channel = 1;
  }
  else {
    adc_tmp = ADCL;
    adc_tmp |= (ADCH << 8);
    adc_tmp = adc_tmp << 6;
    adc_tmp_brake[adc_loop_count] = adc_tmp;
    ADMUX |= B00000001;
    adc_channel = 0;
    adc_loop_count++;
  }

  if (adc_loop_count == ADC_AVG_LEN) adc_loop_count = 0;

}
