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

USB Usb;
PS4USB PS4(&Usb);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
uint32_t loop_count;
uint8_t send_count;
uint8_t auth_state;
bool rx_flag;
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
volatile int pedal_tmp = 0;
volatile int pedal_period = 0;

uint8_t buffer_out[64] = {0};
uint8_t buffer_out_len = 0;
uint8_t buffer_out_flag = 0;
uint8_t buffer_out_micros = 0;
uint8_t update_flag = 0;

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

volatile int readFlag;



uint8_t report[66] = {
  0xff, 0x40, 0x01, 0x80, 0x80, 0x80, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
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
  readFlag = 0;
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

void setup() {
  // setupADC();
  // setup_regularinterrupt();
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(2, INPUT);
  pinMode(10, INPUT);
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
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    while (1); // Halt
  }
  b = 0;
  Serial.begin(500000);
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
      Serial.write(report, report_len);
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

            if (ps4_out[2] == 0) PS4.setLed(0x000033);
            else if (ps4_out[2] == 1) PS4.setLed(0x000066);
            else if (ps4_out[2] == 2) PS4.setLed(0x000099);
            else if (ps4_out[2] == 3) PS4.setLed(0x0000CC);
            else if (ps4_out[2] == 4)  {
              PS4.setLed(0x0000FF);
            }
            PS4.authenticate(64, ps4_out);

          }
        }
        if (matches_chall(byte_in)) {
          PS4.setLed(0x000000);
          auth_rx_flag = 1;
          head_match = 0;
          rx_counter = 0;
        }
        else if ( matches_check(byte_in) ) {

          PS4.authenticate2(16, check_response);

          if (check_response[2] == 0x00) PS4.setLed(Green);
          else if (check_response[2] == 0x10) PS4.setLed(Yellow);
          else PS4.setLed(Red);

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

          if (resp[2] & 1) PS4.setLed(0x444444);
          else PS4.setLed(0xFFFFFF);

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
      else if (PS4.getButtonPress(DOWN)) report[7] = 0x04;
      else if (PS4.getButtonPress(LEFT)) report[7] = 0x06;
      else if(PS4.getButtonPress(RIGHT)) report[7] = 0x02;
      else  report[7] = 0x08;

      if (PS4.getButtonPress(TRIANGLE)) report[7] |= 0x80;
      else report[7] &= ~0x80;

      if (PS4.getButtonPress(CROSS)) report[7] |= 0x20;
      else report[7] &= ~0x20;

      if (PS4.getButtonPress(SQUARE)) report[7] |= 0x10;
      else report[7] &= ~0x10;

      if (PS4.getButtonPress(CIRCLE)) report[7] |= 0x40;
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

      if (PS4.getButtonPress(L1)) report[8] |= 0x01;
      else report[8] &= ~0x01;

      if (PS4.getButtonPress(R1)) report[8] |= 0x02;
      else report[8] &= ~0x02;

      if (PS4.getButtonPress(L2)) report[8] |= 0x04;
      else report[8] &= ~0x04;

      if (PS4.getButtonPress(R2)) report[8] |= 0x08;
      else report[8] &= ~0x08;

      if (update_flag == 0 && (micros() - ps4_up) > 3940000) {
        report[45] = 0xfb;
        report[46] = 0x7e;
        update_flag = 1;
      }

      // Wheel
      report[45] = PS4.getAnalogHat(LeftHatX);;
      report[46] = PS4.getAnalogHat(LeftHatX);
      // report[45] = 128;
      // report[46] = 128;

      // Gas
      report[47] = 255 - PS4.getAnalogButton(R2);
      report[48] = 255 - PS4.getAnalogButton(R2);
      // report[48] = 255;

      // Brake
      report[49] = 255 - PS4.getAnalogButton(L2);
      report[50] = 255 - PS4.getAnalogButton(L2);
      // report[50] = 255;

    }
  }
}

// Interrupt is called once a millisecond,
// ISR(TIMER1_OVF_vect)
// {
//   tx_report_flag = 1;
// }


// // Interrupt service routine for the ADC completion
// ISR(ADC_vect){

//   // Done reading
//   readFlag = 1;

//   if (toggle) {
//     // Steering
//     report[46] = ADCL;
//     report[47] = ADCH;
//     ADMUX |= B00000001;
//     toggle = 0;
//   }
//   else {
//     // Braking
//     report[50] = ADCL;
//     report[51] = ADCH;
//     ADMUX &= B11111110;
//     toggle = 1;
//   }
// }
