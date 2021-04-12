/*
 Clock Generator Based on Si5351 clock chip
 Assumes hardware is:
   - Feather M0 MCU
   - Si5351 Featherwing (by KJ4QLP)
   - OLED Featherwing Display

 First project is to generate 24-25 MHz clocks for LNB modification
*/
#include "Arduino.h"
#include <si5351.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>

//****Board Pinouts*********************************
//--INPUTS--
#define si_btn      12 // button on Si5351 featherwing
#define oled_btn_a  9
#define oled_btn_b  6
#define oled_btn_c  5  //Conflicts with TX_LED1 on Si5351 featherwing
//note, on featherwing in use, cut trace and rerouted TX_LED1 to D10
//--OUTPUTS--
#define RED_LED     13 // on feather M0 board
#define GREEN_LED   8  // on feather M0 board
#define TX_LED1     10 // on Si5351 featherwing board
#define TX_LED2     A0 // on Si5351 featherwing board

//****Clock Details********************
#define FREQ_CORRECTION 15300UL
#define DEFAULT_FREQ    25000000UL //25 MHz clock
#define FREQ_STEP_A    1000UL //500Hz step
#define FREQ_STEP_B    1000UL //500Hz step
#define FREQ_STEP_C    1000UL //500Hz step

//****Timer Defines*********************************
//#define CPU_HZ 48000000
//#define TIMER_PRESCALER_DIV 1024
//uint32_t sampleRate = 682; //sample rate in milliseconds, determines how often TC5_Handler is called
uint32_t sampleRate = 682; //sample rate in milliseconds, determines how often TC5_Handler is called

//****Class instantiation****************8
Si5351 si5351;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

//**** Global Variables *******
volatile bool si_btn_press = false; //set in button ISR
volatile bool btn_a_press  = false; //set in button ISR
volatile bool btn_b_press  = false; //set in button ISR
volatile bool btn_c_press  = false; //set in button ISR
uint32_t freq = DEFAULT_FREQ;
uint8_t led_state  = 0;
bool    clock_enable = true;

typedef struct step_struct {
  uint32_t hz_1    = 1UL;
  uint32_t hz_10   = 10UL;
  uint32_t hz_100  = 100UL;
  uint32_t khz_1   = 1000UL;
  uint32_t khz_10  = 10000UL;
  uint32_t khz_100 = 100000UL;
  uint32_t mhz_1   = 1000000UL;
}step_struct;

step_struct freq_step;
uint8_t step_idx = 0;
uint32_t step_size = 1;
int8_t step_dir = 1; //1=up, -1=down

void setup() {
  Serial.begin(115200);
  Serial.println("OLED FeatherWing test");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  Serial.println("OLED begun");
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
  
  //initialize digital pin outputs.
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  pinMode(TX_LED1, OUTPUT);
  digitalWrite(TX_LED1, HIGH);
  pinMode(TX_LED2, OUTPUT);
  digitalWrite(TX_LED2, LOW);
  
  //initialize digital pin inputs
  pinMode(si_btn, INPUT_PULLUP); // Use a button connected to pin 12 as a transmit trigger
  pinMode(oled_btn_a, INPUT_PULLUP);
  pinMode(oled_btn_b, INPUT_PULLUP);
  pinMode(oled_btn_c, INPUT_PULLUP);

  //Setup interrupts
  int irq_si_btn = digitalPinToInterrupt(si_btn);
  attachInterrupt(irq_si_btn, si_btn_isr, FALLING);
  int irq_btn_a = digitalPinToInterrupt(oled_btn_a);
  attachInterrupt(irq_btn_a, btn_a_isr, FALLING);
  int irq_btn_b = digitalPinToInterrupt(oled_btn_b);
  attachInterrupt(irq_btn_b, btn_b_isr, FALLING);
  int irq_btn_c = digitalPinToInterrupt(oled_btn_c);
  attachInterrupt(irq_btn_c, btn_c_isr, FALLING);

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, FREQ_CORRECTION);
  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 1); // Disable the clock initially

  //Inititalize display text
  // text display tests
  init_display();
  
}

void loop() {
  si5351.set_freq((freq * 100), SI5351_CLK0);

  if (btn_a_press == true) {
    cycle_step_size();
    btn_a_press = false;
    //si5351.set_freq((freq * 100), SI5351_CLK0);
  }
  if (btn_b_press == true) {
    set_step_dir();
    //si5351.set_freq((freq * 100), SI5351_CLK0);
    btn_b_press = false;
  }
  if (btn_c_press == true) {
    freq += step_size * step_dir;
    //si5351.set_freq((freq * 100), SI5351_CLK0);
    btn_c_press = false;
  }
  if (si_btn_press == true) {
    if (clock_enable){ clock_enable=false;}
    else{clock_enable=true;}
    
    led_state = ~led_state;
    digitalWrite(TX_LED1, led_state);
    display.clearDisplay();
    display.setCursor(10,0);
    display.print("Clock State: ");
    display.print(clock_enable);
    //si5351.set_freq((freq * 100), SI5351_CLK0);
    if(clock_enable){
      si5351.output_enable(SI5351_CLK0, 1);
    }
    else{
      si5351.output_enable(SI5351_CLK0, 0);
    }
    si_btn_press = false;
    display.display();
    delay(1000);
  }

  /*display.clearDisplay();
  display.setCursor(0,0);
  display.print(float(freq / 1e6),6);
  delay(10);
  yield();
  display.display();*/
  update_display();
  delay(100);
}

void set_step_dir(){
  if (step_dir==1){ step_dir=-1;}
  else{step_dir=1;}
}
void cycle_step_size(){
  step_idx += 1;
  if (step_idx>6){step_idx=0;}
  step_size = uint32_t (pow(10,step_idx));
  
}
/*
void cycle_step_size_old(){
  step_idx += 1;
  if (step_idx > 2){
    step_idx=0;
  }
  if(step_idx == 0){
    step_size = freq_step.low;
  }
  else if (step_idx==1){
    step_size = freq_step.med;
  }
  else if (step_idx==2){
    step_size = freq_step.high;
  }
}
*/

//***********DISPLAY FUNCTIONS*******************************
void update_display(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Step: ");
  display.println(step_size);
  display.print(" Dir: ");
  display.println(step_dir);
  display.print("Freq: ");
  display.println(float(freq / 1e6),9);
  delay(10);
  yield();
  display.display();
}
void init_display(){
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(  "Si5351 Clock Gen");
  display.println("Status: Disabled");
  display.println("Freq [MHz]: 25.000");
  display.println("line 4");
  display.setCursor(0,0);
  display.display(); // actually display all of the above
}






//***********OTHER FUNCTIONS*******************************
void btn_a_isr(){
  if(digitalRead(oled_btn_a) == LOW){
    delay(50);   // delay to debounce
    if (digitalRead(oled_btn_a) == LOW){
      btn_a_press = true;
      delay(200); //delay to avoid extra triggers
    }
  }
}

void btn_b_isr(){
  if(digitalRead(oled_btn_b) == LOW){
    delay(50);   // delay to debounce
    if (digitalRead(oled_btn_b) == LOW){
      btn_b_press = true;
      delay(200); //delay to avoid extra triggers
    }
  }
}

void btn_c_isr(){
  if(digitalRead(oled_btn_c) == LOW){
    delay(50);   // delay to debounce
    if (digitalRead(oled_btn_c) == LOW){
      btn_c_press = true;
      delay(200); //delay to avoid extra triggers
    }
  }
}

void si_btn_isr(){
  //SerialUSB.println("THERMO ISR Fired!!!");
  //delay(50);
  if(digitalRead(si_btn) == LOW){
    delay(50);   // delay to debounce
    if (digitalRead(si_btn) == LOW){
      si_btn_press = true;
      delay(200); //delay to avoid extra triggers
    }
  }
}



void toggle_led(){
  led_state = ~led_state;
  digitalWrite(RED_LED, led_state);
  //digitalWrite(GREEN_LED, led_state);
}

//***********END OTHER  FUNCTIONS*******************************
