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
uint8_t buffer_out_millis = 0;

unsigned long last_report_tx;
extern volatile unsigned long timer0_millis;
uint8_t byte_in;
uint8_t head_chall[6] = {0xf0, 0x03, 0x00, 0x00, 0x40, 0x00};
uint8_t head_check[6] =  {0xf2, 0x03, 0x00, 0x00, 0x10, 0x00};
uint8_t head_boot0[2] = {0x11, 0x01};
uint8_t head_boot1[2] = {0x22, 0x01};
uint8_t head_boot2[2] = {0x33, 0x01};
// uint8_t resp_check_ready[16] = {0xf2, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x69, 0xed, 0x12};
// uint8_t resp_check_wait[16] =  {0xf2, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcc, 0xe8, 0x43, 0x35};
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

// uint8_t chal1[64] {0xf0, 0x01, 0x00, 0x00, 0x4d, 0x6f, 0x72, 0xdb, 0x29, 0x5c, 0x8a, 0xf7, 0x0e, 0xe1, 0x55, 0x94, 0xed, 0x3f, 0x6c, 0xdb, 0xda, 0x6b, 0x16, 0xa2, 0xb2, 0xbe, 0x88, 0xa1, 0x3f, 0x48, 0x6e, 0xd4, 0x3d, 0xb6, 0x38, 0x59, 0xba, 0x04, 0xf3, 0xb4, 0x90, 0x6a, 0x76, 0xf6, 0xff, 0xb6, 0x17, 0x33, 0xb3, 0xea, 0xc7, 0x73, 0x59, 0xdf, 0xb7, 0x48, 0xbf, 0x0d, 0xbb, 0xbd, 0x96, 0xe0, 0xf6, 0x44};
// uint8_t chal2[64] {0xf0, 0x01, 0x01, 0x00, 0x6f, 0x58, 0x1b, 0x0d, 0x5b, 0x41, 0xab, 0x36, 0x28, 0xd1, 0x89, 0xae, 0x8b, 0x3a, 0xf5, 0xe5, 0xf7, 0xd4, 0xd1, 0x28, 0xcc, 0x2b, 0x8b, 0xaa, 0xd9, 0x8d, 0xef, 0xca, 0x74, 0xff, 0x85, 0x96, 0x3e, 0x39, 0x44, 0x63, 0xe2, 0x6d, 0x1b, 0x59, 0x52, 0x80, 0x3c, 0x40, 0xf8, 0xfa, 0xc9, 0x80, 0xeb, 0xd7, 0x49, 0x01, 0x4d, 0x0d, 0x88, 0x36, 0xf7, 0xe8, 0xd3, 0x58};
// uint8_t chal3[64] {0xf0, 0x01, 0x02, 0x00, 0x96, 0x33, 0x20, 0x06, 0xd0, 0x61, 0x6a, 0xc6, 0xe3, 0xac, 0xaa, 0xcb, 0x5a, 0xc2, 0x6b, 0x4b, 0xf9, 0x10, 0x8d, 0xc9, 0x5f, 0x7c, 0xb1, 0x12, 0x2d, 0x6b, 0x8e, 0x8a, 0x9e, 0x52, 0xdb, 0xd8, 0xc2, 0x24, 0x35, 0x0b, 0x0a, 0x2d, 0xd6, 0x65, 0x58, 0x0f, 0x09, 0x6b, 0xde, 0xf2, 0xba, 0x93, 0x6f, 0x52, 0xd4, 0xf7, 0xc8, 0xf7, 0x32, 0x01, 0xdb, 0xed, 0x68, 0x09};
// uint8_t chal4[64] {0xf0, 0x01, 0x03, 0x00, 0x38, 0x1d, 0x31, 0xb2, 0x6e, 0xe3, 0x86, 0x0a, 0xbb, 0xe2, 0xd8, 0x3a, 0x0e, 0xd3, 0x35, 0x87, 0x52, 0x85, 0xd1, 0xe6, 0xb7, 0x25, 0x95, 0x2f, 0xf0, 0x3e, 0x60, 0x38, 0xd8, 0x97, 0xa5, 0x7f, 0x87, 0xd9, 0x4d, 0x05, 0xef, 0x47, 0xa0, 0x9f, 0x75, 0xd3, 0xe4, 0x58, 0xd4, 0x29, 0xa1, 0x23, 0xed, 0x19, 0x8a, 0x0d, 0xdf, 0x8f, 0xa5, 0x8c, 0xc3, 0xd3, 0x2c, 0x44};
// uint8_t chal5[64] {0xf0, 0x01, 0x04, 0x00, 0x3e, 0xc4, 0x94, 0x25, 0x2e, 0xdc, 0xdf, 0xa1, 0x0a, 0x60, 0x44, 0x54, 0xfc, 0x84, 0x02, 0x3f, 0xe8, 0x78, 0xfd, 0xe5, 0xdf, 0x87, 0xbb, 0x71, 0x9a, 0x14, 0xd7, 0xaf, 0x27, 0x0b, 0x9c, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x7a, 0x32, 0xeb};
// High when a value is ready to be read
volatile int readFlag;



uint8_t report[65] = {
  0xff, 0x40, 0x01, 0x80, 0x80, 0x80, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00
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
  auth_state = 0;

  report_len = sizeof(report);
  rx_counter = 0;
  auth_msg_count = 0;
  auth_rx_flag = 0;
  send_check_response = 0;
  main_state = 0;
  last_report_tx = 1000;
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    // Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  // Serial.print(F("\r\nPS4 USB Library Started"));
  b = 0;
  // attachInterrupt(0, rising, RISING);
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

//Sets the millis value
void setMillis(unsigned long new_millis)
{
  uint8_t oldSREG = SREG;

  cli();
  timer0_millis = new_millis;
  SREG = oldSREG;
}

void loop() {

  Usb.Task();

  if (main_state == 6) {

    if (buffer_out_flag && millis() >= buffer_out_millis) {
      Serial.write(buffer_out, buffer_out_len);
      buffer_out_flag = 0;
      last_report_tx = last_report_tx + 5;
      if (millis() >= last_report_tx) last_report_tx = last_report_tx + 5;
    }
    else if (millis() >= last_report_tx) {
      Serial.write(report, report_len);
      last_report_tx = last_report_tx + 5;
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
      noInterrupts();
      Serial.write(0x11);
      Serial.write(0x00);
      interrupts();
      main_state = 1;
      last_rx = millis();
    }
    else if (main_state == 1) {
      if (last_rx + 1000 <= millis()) main_state = 0;
      else {
        while(Serial.available()) {
          byte_in = Serial.read();
          if (matches_boot0(byte_in)) {
            main_state = 2;
            last_rx = millis();
            break;
          }
        }
      }
    }
    else if (main_state == 2 && last_rx + 2 <= millis()) {
      noInterrupts();
      Serial.write(0x22);
      Serial.write(0x00);
      interrupts();
      main_state = 3;
      last_rx = millis();
    }
    else if (main_state == 3) {
      if (last_rx + 1000 <= millis()) main_state = 2;
      else {
        while(Serial.available()) {
          byte_in = Serial.read();
          if (matches_boot1(byte_in)) {
            main_state = 4;
            break;
            last_rx = millis();
          }
        }
      }
    }
    else if (main_state == 4 && last_rx + 800 <= millis()) {
      noInterrupts();
      Serial.write(0x33);
      Serial.write(0x00);
      interrupts();
      main_state = 5;
      last_rx = millis();
    }
    else if (main_state == 5) {
      if (last_rx + 1000 <= millis()) main_state = 4;
      else {
        while(Serial.available()) {
          byte_in = Serial.read();
          if (matches_boot2(byte_in)) {
            main_state = 6;
            last_rx = millis();
            setMillis(0);
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
            // PS4.setLed(Purple);
            // Weve received the whole challenge part
            auth_rx_flag = 0;
            rx_counter = 0;
            loop_count++;

            // if (loop_count == 1) PS4.authenticate(64, &chal1[0]);
            // else if (loop_count == 2) PS4.authenticate(64, &chal2[0]);
            // else if (loop_count == 3) PS4.authenticate(64, &chal3[0]);
            // else if (loop_count == 4) PS4.authenticate(64, &chal4[0]);
            // else if (loop_count == 5) PS4.authenticate(64, &chal5[0]);

            if (ps4_out[2] == 0) PS4.setLed(0x000033);
            else if (ps4_out[2] == 1) PS4.setLed(0x000066);
            else if (ps4_out[2] == 2) PS4.setLed(0x000099);
            else if (ps4_out[2] == 3) PS4.setLed(0x0000CC);
            else if (ps4_out[2] == 4)  {
              PS4.setLed(0x0000FF);
              // auth_state++;
            }
            PS4.authenticate(64, ps4_out);

          }
        }
        if (matches_chall(byte_in)) {
          PS4.setLed(0x000000);
          auth_rx_flag = 1;
          head_match = 0;
          rx_counter = 0;
          // for (uint8_t i = 0; i < 64; i++) ps4_out[i] = Serial.read();
          // PS4.authenticate(64, ps4_out);

          // if (ps4_out[2] == 0) PS4.setLed(0x000033);
          // else if (ps4_out[2] == 1) PS4.setLed(0x000066);
          // else if (ps4_out[2] == 2) PS4.setLed(0x000099);
          // else if (ps4_out[2] == 3) PS4.setLed(0x0000CC);
          // else if (ps4_out[2] == 4)  {
          //   PS4.setLed(0x0000FF);
          //   auth_state++;
          // }
        }
        else if ( matches_check(byte_in) ) {
          // PS4.setLed(Yellow);
          // if (toggle) {
          //   // PS4.setLedFlash(10, 10);
          //   toggle = 0;
          // }
          // else {
          //   // PS4.setLedFlash(0, 0);
          //   toggle = 1;
          // }
          PS4.authenticate2(16, check_response);

          // noInterrupts();
          // Serial.write(0x44);
          // Serial.write(0x10);
          // for (uint8_t i = 0; i < 16; i++) Serial.write(check_response[i]);
          // Serial.write(check_response, 16);

          if (check_response[2] == 0x00) PS4.setLed(Green);
          else if (check_response[2] == 0x10) PS4.setLed(Yellow);
          else PS4.setLed(Red);

          buffer_out[0] = 0x44;
          buffer_out[1] = 0x10;
          for (uint8_t i = 2; i < 18; i++) buffer_out[i] = check_response[i-2];
          buffer_out_len = 18;
          buffer_out_flag = 1;
          buffer_out_millis = millis() + 2;

          // interrupts();
          // Serial.flush();


          // uint8_t ready_flag = 1;
          // uint8_t wait_flag = 1;

          // for (uint8_t i = 0; i < 12; i++) if (check_response[i] != resp_check_ready[i] || i != 1) ready_flag = 0;
          // for (uint8_t i = 0; i < 12; i++) if (check_response[i] != resp_check_wait[i] || i != 1) wait_flag = 0;

          // if (ready_flag) {
          //   PS4.setLed(Green);
          // }
          // else if (wait_flag) {
          //   PS4.setLed(Yellow);
          // }
          // else PS4.setLed(Red);

          // // Check if the response is ready and update LED
          // if (resp[0] == 0xf2 && resp[1] == 0x01 && resp[2] == 0x00) {
          //   // auth_state = 2;
          //   // send_count = 0;
          //   PS4.setLed(Green);
          // }
          // else if (resp[0] == 0xf2 && resp[1] == 0x01 && resp[2] == 0x10) {
          //   PS4.setLed(Yellow);
          // }
          // else PS4.setLed(Red);
        }
        else if (matches_reqst(byte_in)) {
          // if (send_count % 2 == 0) PS4.setLed(0xFFFFFF); // Bright white
          // else PS4.setLed(0x888888); // Dim white

          // Request response from DS4
          PS4.authenticate3(64, resp);

          if (resp[2] & 1) PS4.setLed(0x444444);
          else PS4.setLed(0xFFFFFF);

          buffer_out[0] = 0x44;
          buffer_out[1] = 0x40;
          for (uint8_t i = 2; i < 66; i++) buffer_out[i] = resp[i-2];
          buffer_out_len = 66;
          buffer_out_flag = 1;
          buffer_out_millis = millis() + 2;
          // Send response to PS4
          // noInterrupts();
          // Serial.write(0x44);
          // Serial.write(0x40);
          // Serial.write(resp, 64);
          // for (uint8_t i = 0; i < 64; i++) {
          //   Serial.write(resp[i]);
          // }
          // interrupts();
          // Serial.flush();

          // send_count++;

          // if (resp[2] == 0x12) auth_rx_flag = 0;


            // auth_state = 0;
            // send_count = 0;
            // PS4.setLed(Off);
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


      // Wheel
      report[45] = 128;
      report[46] = PS4.getAnalogHat(LeftHatX);

      // Gas
      report[47] = 255;
      report[48] = 255 - PS4.getAnalogButton(R2);

      report[49] = 255;
      report[50] = 255 - PS4.getAnalogButton(L2);
      // report[46] = 128;
      // report[47] = 255 - PS4.getAnalogButton(R2);

      // report[49] = 255 - PS4.getAnalogButton(L2);


      // if (millis() >= last_tx + 5) {
      //   for(int i = 0; i < report_len; i++) Serial.write(report[i]);
      //   // }

      //   last_tx = millis();
      // }
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



      // ps4_out[rx_counter] = Serial.read();
      // if (ps4_out[(rx_counter-6) % 64] == 0x09 &&
      //     ps4_out[(rx_counter-5) % 64] == 0xf0 &&
      //     ps4_out[(rx_counter-4) % 64] == 0x03 &&
      //     ps4_out[(rx_counter-3) % 64] == 0x00 &&
      //     ps4_out[(rx_counter-2) % 64] == 0x00 &&
      //     ps4_out[(rx_counter-1) % 64] == 0x40 &&
      //     ps4_out[(rx_counter-0) % 64] == 0x00) {
      //   // What follows next is an authentication challenge message
      //   auth_msg_count = 0;
      //   rx_counter = 0;
      //   auth_rx_flag = 1;
      // }
      // else {
      //   rx_counter++;
      //   if (rx_counter == 64) rx_counter = 0;
      // }
      // if (auth_rx_flag == 1 && rx_counter == 64) {
      //   PS4.authenticate(65, &ps4_out[0]);
      //   auth_msg_count++;
      //   rx_counter = 0;
      // }
      // digitalWrite(3, auth_msg_count % 2);