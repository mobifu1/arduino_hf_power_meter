/* Modified by Bodmer to be an example for TFT_HX8357 library.
   This sketch uses the GLCD font only.

   This test occupies the whole of the display therefore
   will take twice as long as it would on a 320 x 240
   display. Bear this in mind when making performance
   comparisons.

   Make sure all the required font is loaded by editting the
   User_Setup.h file in the TFT_HX8357 library folder.

   Original header is at the end of the sketch, some text in it is
   not applicable to the HX8357 display supported by this example.

    HF-Power Meter: 100-1500 Watt
    10 W = 0.4472 A an 50 Ohm  > P=(I*I)*R
    Die ausgekoppelte FWD Spannung ist dem HF-Strom proportional.
*/
#include <TimerOne.h>
#include <TFT_HX8357.h> // Hardware-specific library
#include "Free_Fonts.h"

TFT_HX8357 tft = TFT_HX8357();

//16-bit color values:
#define BLACK   0xFFFF
#define WHITE   0x0000
#define BLUE    0xFFE0
#define RED     0x07FF
#define GREEN   0xF81F
#define CYAN    0xF800
#define MAGENTA 0x07E0
#define YELLOW  0x001F
#define ORANGE  0x041F
#define GRAY    0x8410

//Display Edges
const int x_edge_left = 0;
const int x_edge_right = 479;
const int y_edge_up = 0;
const int y_edge_down = 319;

const float pi = 3.14159265;
const float to_mV = 4.8828125;

int analog_fwd_Pin = A1;
int analog_rfl_Pin = A0;
int analog_batt_Pin = A2;
int band_val = 0;
float band_factor = 1;
const float pi_div_180 = 0.017578125; //pi / 180
const float swr_warning = 1.2;

//calculate the incomming dc-voltage from SWR-Bridge the right hf-current:

// 160m > 1500W > P = I * I * 50 Ohm > I = 5,477 A > A/D = 2500mV > A/D = 512 bit;
// 160m > 1000W > P = I * I * 50 Ohm > I = 4.472 A > A/D = 2041mV > A/D = 418 bit;
// 160m >  500W > P = I * I * 50 Ohm > I = 3.162 A > A/D = 1444mV > A/D = 296 bit;
// 160m >  100W > P = I * I * 50 Ohm > I = 1.414 A > A/D =  645mV > A/D = 132 bit;
// 160m >   50W > P = I * I * 50 Ohm > I = 1.000 A > A/D =  456mV > A/D =  93 bit;
// 160m >   10W > P = I * I * 50 Ohm > I = 0.616 A > A/D =  281mV > A/D =  57 bit;

// R1=130KOhm, R2=38KOhm, V=1:3.42 > (R2=0-50KOhm)

const float divisor_factor = 0.765;
const float impedance = 50;// 50 Ohm
const float sqr2 = 1.414213562;
const String band_names [9] = {"160m", " 80m", " 40m", " 30m", " 20m", " 17m", " 15m", " 12m", " 10m"};
const float band_factors [9] = {1, 1.023, 1.032, 1.023 , 1.023, 1.056, 1.0, 1.023, 0.980, }; //160m-10m > // correction of the swr-bridge

int log_values[231] = {};

boolean update_values = false;
boolean force_update_values = false;
boolean tick = false;

//hf values
int peak_bar = 0;
int old_peak_bar = 1;
uint16_t peak_reset = 0;
int old_band = 0;
int old_fwd = 0;
int old_rfl = 0;
int smart_fade_out = 5;

//rotary encoder
#define encoder0PinA  2
#define encoder0PinB  3
#define encoder0PinC  5 //common Pin
volatile unsigned int encoder0Pos = 0;

//relais ptt interrupt
#define relais_0 7
boolean ptt_interrupt = false;
float ptt_interrupt_watt = 35;
const float ptt_interrupt_swr = 2.0;

//button
int button_Pin = 4; // switch the button to ground
int button_status = 0;

//menue
int menue_level = 0;
boolean use_batt = false;
int menue_3_choose = 0;
int menue_4_choose = 0;
int old_needle_xpos = 0;
int old_needle_ypos = 0;
int old_needle_left_xpos = 0;
int old_needle_left_ypos = 0;
int old_needle_right_xpos = 0;
int old_needle_right_ypos = 0;
//#########################################################################
//#########################################################################
void setup() {

  pinMode(analog_fwd_Pin, INPUT);
  pinMode(analog_rfl_Pin, INPUT);
  pinMode(analog_batt_Pin, INPUT);
  pinMode (button_Pin, INPUT_PULLUP);
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder0PinC, OUTPUT);
  digitalWrite(encoder0PinC, LOW);
  attachInterrupt(0, doEncoderA, CHANGE); // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE); // encoder pin on interrupt 1 (pin 3)

  pinMode(relais_0, OUTPUT);
  digitalWrite(relais_0, LOW);

  Serial.begin(9600);
  tft.init();
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  ScreenText(WHITE, 10, 10 , 2, F("HF-Power Meter: V1.2-Beta"));// Arduino IDE 1.8.4
  ScreenText(WHITE, 10, 40 , 2, F("Max. 1.5 kW / Bands: 160m-10m"));
  ScreenText(WHITE, 10, 70 , 2, F("50 Ohm Coax Cable"));
  ScreenText(WHITE, 10, 200 , 6, F("DD8ZJ / DL8KX"));

  boolean TickTock = false;
  for (int i = 16 ; i < x_edge_right - 16; i = i + 16) {
    TickTock = !TickTock;
    if (TickTock == false)ScreenText(WHITE, i, 150 , 2, F("-"));
    if (TickTock == true)ScreenText(WHITE, i, 145 , 2, F("."));
    delay(100);
  }

  delay(1000);
  tft.fillScreen(BLACK);
  scale();

  Timer1.initialize(1000000);//microseconds
  Timer1.attachInterrupt(timer_interrupt);

  //  Invert the color
  //  uint16_t i = GRAY ^ 0xFFFF;
  //  SetFilledRect(i , 330, 25, 50, 16) ;
  //  SetFilledRect(BLACK , 80, 220, 100, 16) ;
  //  ScreenText(i, 80, 220, 2, "0x" + String(i, HEX));
  //  delay(10000);

}
//--------------------------------------------------------------------------------------------------------
void loop() {

  if (menue_level == 0) {
    band();
    hf_power_digital();
    batterie();
    encoder_button();
    force_update_values = false;//force to update values
  }
  if (menue_level == 1) {
    hf_power_analog();
    encoder_button();
    force_update_values = false;//force to update values
  }
  if (menue_level == 2) {
    menue_2();
    encoder_button();
  }
  if (menue_level == 3) {
    menue_3();
    encoder_button();
  }
  if (menue_level == 4) {
    menue_4();
    encoder_button();
  }
  if (menue_level == 5) {
    menue_level = 0;
    scale();
  }
}
//----------------------------------------------
//--------------GRAFIK-ROUTINEN-----------------
//----------------------------------------------
void ScreenText(uint16_t color, int xtpos, int ytpos, int text_size , String text) {

  tft.setCursor(xtpos, ytpos);
  tft.setTextFont(1);
  tft.setTextColor(color);
  tft.setTextSize(text_size);
  tft.println(text);
}

void SetLines(uint16_t color , int xl1pos, int yl1pos, int xl2pos, int yl2pos) {
  tft.drawLine(xl1pos, yl1pos, xl2pos, yl2pos, color);
}

void SetPoint(uint16_t color, int xppos, int yppos) {
  tft.drawPixel(xppos, yppos, color);
}

void SetRect(uint16_t color , int xr1pos, int yr1pos, int xr2width, int yr2hight) {
  tft.drawRect(xr1pos, yr1pos, xr2width, yr2hight, color);
}

void SetFilledRect(uint16_t color , int xr1pos, int yr1pos, int xr2width, int yr2hight) {
  tft.fillRect(xr1pos, yr1pos, xr2width, yr2hight, color);
}

void SetCircle(uint16_t color , int xcpos, int ycpos, int radius) {
  tft.drawCircle(xcpos, ycpos, radius, color);
}

void SetFilledCircle(uint16_t color , int xcpos, int ycpos, int radius) {
  tft.fillCircle(xcpos, ycpos, radius, color);
}

void SetTriangle(uint16_t color , int xpeak, int ypeak, int xbottom_left, int ybottom_left, int xbottom_right, int ybottom_right) {
  tft.drawTriangle(xpeak, ypeak, xbottom_left, ybottom_left, xbottom_right, ybottom_right, color);
}

void SetFilledTriangle(uint16_t color , int xpeak, int ypeak, int xbottom_left, int ybottom_left, int xbottom_right, int ybottom_right) {
  tft.fillTriangle(xpeak, ypeak, xbottom_left, ybottom_left, xbottom_right, ybottom_right, color);
}
//--------------------------------------------------------------------------------------------------------
float calc_hf_power(float voltage) { //calculate hf-power from voltage output of swr-bridge

  // http://www.walterzorn.de/grapher/grapher.htm
  // ((x*0.8175)/sqr(2))²/ 50;
  // ((x*0.765)/sqr(2))² / 50;
  // ((x*0.715)/sqr(2))² / 50;

  float voltage_eff;
  float power;

  voltage_eff = voltage / sqr2;
  power = (voltage_eff * voltage_eff) / impedance;
  return power;
}
//--------------------------------------------------------------------------------------------------------
void scale() {

  SetRect(WHITE , x_edge_left, 80, x_edge_right, 20);  //FWD
  SetRect(WHITE , x_edge_left, 130, x_edge_right, 10); //RFL

  for (int x = 0; x < x_edge_right; x += 32) {
    SetPoint(WHITE, x, 101);
    SetPoint(WHITE, x, 141);
  }

  ScreenText(WHITE, 10, 25, 2, "PWR:");
  ScreenText(WHITE, 300, 25, 2, "BAND: ");

  SetRect(WHITE , 430, 290, 30, 15);//Batterie
  SetFilledRect(WHITE , 460, 295, 5, 5);

  SetRect(WHITE , 240, 170, 230, 100);//Logging

  force_update_values = true;
}
//--------------------------------------------------------------------------------------------------------
void hf_power_digital() {  //show FWD / RFL / SWR / Peak-Power

  int fwd = analogRead(analog_fwd_Pin);    // read from sensor pin value:0-1024
  int rfl = analogRead(analog_rfl_Pin);    // read from sensor pin value:0-1024

  //------smart fade out:-----------------------
  if (fwd < 2 && old_fwd > smart_fade_out) {
    old_fwd -= smart_fade_out;
    fwd = old_fwd;
  }
  else {
    old_fwd = fwd;
  }

  if (rfl < 2 && old_rfl > smart_fade_out) {
    old_rfl -= smart_fade_out;
    rfl = old_rfl;
  }
  else {
    old_rfl = rfl;
  }
  //----------------------------------------------

  if (fwd > 0 && fwd < 43) { // R1=130KOhm, R2=38KOhm, V=1:3.42  > 0,7V/3.42=204mV = 42bit A/D Diode Limit Voltage  (R2=0-50KOhm)
    SetFilledTriangle(RED , 200, 290, 192 , 306 , 208 , 306);//innerhalb 0.7V Diodenspannung
    ScreenText(RED, 220, 291, 2 , "Not Exact !");
  }
  else {
    SetFilledRect(BLACK , 190, 291, 160, 16);
  }

  float fwd_float = float(fwd) * band_factor * divisor_factor;
  float rfl_float = float(rfl) * band_factor * divisor_factor;

  float  fwd_watt = calc_hf_power(fwd_float);
  float  rfl_watt = calc_hf_power(rfl_float);

  if (fwd_watt > ptt_interrupt_watt) { // PTT Interrupt wenn TX-Power > Schwellwert
    if (ptt_interrupt_watt > 0 )ptt_interrupt = true; // deaktiviert wenn wert = 0
  }

  if (fwd == 0)peak_reset++;
  if (fwd > 0)peak_reset = 0;

  //fwd bar:
  //int fwd_bar = int(fwd_watt * 0.312);
  int fwd_bar = int(fwd_float);
  if (fwd_bar > x_edge_right - 1)fwd_bar = x_edge_right - 1;
  uint16_t fwd_color;
  for (int i = 2; i < x_edge_right - 6; i += 6) {
    if (i < x_edge_right) fwd_color = RED;
    if (i < 355) fwd_color = ORANGE;
    if (i < 240) fwd_color = YELLOW;
    if (i < 125) fwd_color = GREEN;
    if (i > fwd_bar) fwd_color = BLACK;
    SetFilledRect(fwd_color , i, 81, 5, 18);
  }

  //rfl bar:
  //int rfl_bar = int(rfl_watt * 0.312);
  int rfl_bar = int(rfl_float);
  if (rfl_bar > x_edge_right - 1)rfl_bar = x_edge_right - 1;
  uint16_t rfl_color;
  for (int y = 2; y < x_edge_right - 6; y += 6) {
    if (y < x_edge_right) rfl_color = RED;
    if (y < 355) rfl_color = ORANGE;
    if (y < 240) rfl_color = YELLOW;
    if (y < 125) rfl_color = GREEN;
    if (y > rfl_bar) rfl_color = BLACK;
    SetFilledRect(rfl_color , y, 131, 5, 8) ;
  }

  //fwd & rfl:
  if (update_values == true || force_update_values == true) {
    SetFilledRect(BLACK , 40, 70, 60, 8);
    ScreenText(WHITE, 10, 70, 1 , "FWD: " + String (fwd_watt, 1) + " W");
    //dBm calculation
    double dBm = 30 + (10 * log10(double(fwd_watt)));
    SetFilledRect(BLACK , 70, 25, 100, 16);
    if (fwd > 0)ScreenText(WHITE, 70, 25, 2 , String (dBm, 1) + " dBm");

    SetFilledRect(BLACK , 40, 120, 60, 8);
    ScreenText(WHITE, 10, 120, 1 , "RFL: " + String (rfl_watt, 1) + " W");

    logging(fwd_watt);
  }

  //SWR
  if (update_values == true || force_update_values == true) {
    float swr = (fwd_float + rfl_float) / (fwd_float - rfl_float);
    if (swr > 100)swr = 100;
    if (swr > swr_warning)SetRect(RED , 10, 218, 145, 40);
    if (swr <= swr_warning)SetRect(BLACK , 10, 218, 145, 40);
    SetFilledRect(BLACK , 20, 230, 130, 16);
    if (fwd_float > 0) {
      ScreenText(WHITE, 20, 230, 2 , "SWR: " + String (swr, 1));
      if (swr > ptt_interrupt_swr && fwd > 42) { // PTT Interrupt wenn SWR zu gross
        ptt_interrupt = true;
      }
    }
  }

  //Peak Value
  float peak_watt = 0;
  if (fwd_bar > peak_bar || force_update_values == true) {
    peak_bar = fwd_bar;
    peak_watt = fwd_watt;
  }
  if (peak_reset > 150) {//delay to clear the peak value
    peak_reset = 0;
    peak_bar = 1;
    peak_watt = 0;
  }
  if (peak_bar > 475) peak_bar = 475;
  if (old_peak_bar != peak_bar) {
    SetTriangle(BLACK , old_peak_bar, 102, old_peak_bar - 4, 108, old_peak_bar + 4, 108);
    old_peak_bar = peak_bar;
    SetFilledRect(BLACK , 230, 70, 60, 8);
    if (peak_watt > 1)ScreenText(WHITE, 200, 70, 1, "PEAK: " + String (peak_watt, 1) + " W");
  }
  SetTriangle(WHITE , old_peak_bar, 102, old_peak_bar - 4, 108, old_peak_bar + 4, 108);

  //PTT Interrupt
  if (ptt_interrupt == true) { // PTT Interrupt wenn TX-Power > Schwellwert oder SWR > 2
    digitalWrite(relais_0, HIGH);
    ScreenText(RED, 20, 291, 2 , "PTT -/- !");
  }
  else {
    SetFilledRect(BLACK , 0, 291, 160, 16);
  }

  update_values = false;
}
//--------------------------------------------------------------------------------------------------------
void hf_power_analog() {  //show FWD / RFL / SWR on cross needles

  int fwd = analogRead(analog_fwd_Pin);    // read from sensor pin value:0-1024
  int rfl = analogRead(analog_rfl_Pin);    // read from sensor pin value:0-1024

  //------smart fade out:-----------------------
  if (fwd < 2 && old_fwd > smart_fade_out) {
    old_fwd -= smart_fade_out;
    fwd = old_fwd;
  }
  else {
    old_fwd = fwd;
  }

  if (rfl < 2 && old_rfl > smart_fade_out) {
    old_rfl -= smart_fade_out;
    rfl = old_rfl;
  }
  else {
    old_rfl = rfl;
  }
  //----------------------------------------------

  float fwd_float = float(fwd) * band_factor * divisor_factor;
  float rfl_float = float(rfl) * band_factor * divisor_factor;

  show_cross_needle(rfl_float, fwd_float);

  //swr
  if (update_values == true || force_update_values == true) {
    float swr = (fwd_float + rfl_float) / (fwd_float - rfl_float);
    if (swr > 100)swr = 100;
    if (swr > swr_warning)SetRect(RED , 170, 158, 145, 40);
    if (swr <= swr_warning)SetRect(BLACK , 170, 158, 145, 40);
    SetFilledRect(BLACK , 180, 170, 100, 16);
    if (fwd_float > 0)ScreenText(WHITE, 180, 170, 2 , "SWR: " + String (swr, 1));
  }
}
//--------------------------------------------------------------------------------------------------------
void band() { //show the used band

  if (old_band != band_val || force_update_values == true) {
    old_band = band_val;
    band_factor =  band_factors[band_val]; // factor for calculation of fwd and rfl for different bands
    SetFilledRect(BLACK , 370, 25, 100, 16) ;
    ScreenText(WHITE, 370, 25, 2, band_names[band_val]);
    force_update_values = true;
  }
}
//--------------------------------------------------------------------------------------------------------
void batterie() {

  int batt = analogRead(analog_batt_Pin);  // read from sensor pin value:0-1024
  float voltage_batt = batt * to_mV;
  uint16_t batt_color;
  if (voltage_batt >= 1200)batt_color = GREEN;//NiCd Cell Values in mV
  if (voltage_batt < 1200)batt_color = ORANGE;
  if (voltage_batt < 1150)batt_color = RED;
  if (use_batt == false)batt_color = GREEN;
  SetFilledRect(batt_color , 431, 291, 28, 13);
}
//--------------------------------------------------------------------------------------------------------
void logging(float power) {

  log_values[0] = int(power);
  SetFilledRect(BLACK , 241, 171, 228, 98);

  for (int i = 227; i >= 0 ; i--) {
    log_values[i + 1] = log_values[i];
    if (log_values[i] >= 0 && log_values[i] < 1500) {
      //SetPoint(CYAN, 241 + i, 267 - (log_values[i] / 16));
      SetLines(RED, 241 + i, 266 - (log_values[i] / 16), 241 + i, 266);
    }
  }
}
//--------------------------------------------------------------------------------------------------------
void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder_turn(true);
      //Serial.println ("A:H / B:L +");
    }
    else {
      encoder_turn(false);
      //Serial.println ("A:H / B:H -");
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder_turn(true);
      //Serial.println ("A:L / B:H +");
    }
    else {
      encoder_turn(false);
      //Serial.println ("A:L / B:L -");
    }
  }
  //Serial.println (encoder0Pos, DEC);
}
//--------------------------------------------------------------------------------------------------------
void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder_turn(true);
      //Serial.println ("B:H / A:H +");
    }
    else {
      encoder_turn(false);
      Serial.println ("B:H / A:L -");
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder_turn(true);
      //Serial.println ("B:L / A:L +");
    }
    else {
      encoder_turn(false);
      //Serial.println ("B:L / A:H -");
    }
  }
}
//--------------------------------------------------------------------------------------------------------
void encoder_turn(boolean enc_direction) {

  if (enc_direction == true) { //turn right
    if (menue_level == 0 ) {
      band_val++;
      if (band_val > 8)band_val = 0;
    }
    if (menue_level == 1 ) {

    }
    if (menue_level == 3 ) {
      menue_3_choose++;
      if (menue_3_choose > 2)menue_3_choose = 0;
    }
    if (menue_level == 4 ) {
      if (ptt_interrupt_watt < 100) {
        ptt_interrupt_watt += 10;
      }
      else {
        ptt_interrupt_watt += 100;
      }
      if (ptt_interrupt_watt > 1500) {
        ptt_interrupt_watt = 1500;
      }
    }
  }
  //---------------------------
  if (enc_direction == false) { //turn left
    if (menue_level == 0 ) {
      band_val--;
      if (band_val < 0)band_val = 8;
    }
    if (menue_level == 1 ) {

    }
    if (menue_level == 3 ) {
      menue_3_choose--;
      if (menue_3_choose < 0)menue_3_choose = 2;
    }
    if (menue_level == 4 ) {
      if (ptt_interrupt_watt < 100) {
        ptt_interrupt_watt -= 10;
      }
      else {
        ptt_interrupt_watt -= 100;
      }
      if (ptt_interrupt_watt < 0) {
        ptt_interrupt_watt = 0;
      }
    }
  }
}
//--------------------------------------------------------------------------------------------------------
void encoder_button() { //enter the menue

  int button_val;
  button_val = digitalRead(button_Pin);
  if (button_val == LOW & button_status == 0)button_status++;
  if (button_val == HIGH & button_status == 1) {
    //todo:
    if (menue_level == 4 ) {
      menue_level++;
      force_update_values = true;//force to update values
      tft.fillScreen(BLACK);
    }
    if (menue_level == 3 ) {
      menue_level++;
      tft.fillScreen(BLACK);
    }
    if (menue_level == 2 ) {
      menue_level++;
      tft.fillScreen(BLACK);
    }
    if (menue_level == 1 ) {
      menue_level++;
      tft.fillScreen(BLACK);
    }
    if (menue_level == 0 ) {
      if (ptt_interrupt == true) {
        ptt_interrupt = false;
        digitalWrite(relais_0, LOW);
      }
      else {
        menue_level++;
        tft.fillScreen(BLACK);
      }
    }
    button_status--;
  }
}
//-------------------------------------------------------------------------------------------------------
void menue_2() {

  ScreenText(WHITE, 10, 25, 2, "SWR-Bridge Calibration Factors:");
  SetLines(WHITE, 10, 50, 375, 50);
  for (int i = 0; i < 9; i++) {
    ScreenText(WHITE, 10, 80 + (20 * i), 2, band_names[i] + ": " + String(band_factors [i], 3));
  }
  ScreenText(WHITE, 10, 285, 2, "Divisor: " + String(divisor_factor, DEC));

  SetLines(WHITE, 180, 80, 180, 235);//Coordinate y
  SetLines(WHITE, 180, 235, 479, 235);//Coordinate x
  for (int i = 0; i < 9; i++) { //scala X
    SetLines(WHITE, 180 + (i * 35), 233, 180 + (i * 35), 237);
    ScreenText(WHITE, 170 + (i * 35), 243, 1, band_names [i]);
  }
  SetLines(RED, 181, (100 * band_factors [0]), 216, 100 * band_factors [1]); //160m to 80m
  SetLines(RED, 216, (100 * band_factors [1]), 251, 100 * band_factors [2]); //80m to 40m
  SetLines(RED, 251, (100 * band_factors [2]), 286, 100 * band_factors [3]); //40m to 30m
  SetLines(RED, 286, (100 * band_factors [3]), 321, 100 * band_factors [4]); //30m to 20m
  SetLines(RED, 321, (100 * band_factors [4]), 356, 100 * band_factors [5]); //20m to 17m
  SetLines(RED, 356, (100 * band_factors [5]), 391, 100 * band_factors [6]); //17m to 15m
  SetLines(RED, 391, (100 * band_factors [6]), 426, 100 * band_factors [7]); //15m to 12m
  SetLines(RED, 426, (100 * band_factors [7]), 461, 100 * band_factors [8]); //12m to 10m
}
//-------------------------------------------------------------------------------------------------------
void menue_3() {

  ScreenText(WHITE, 10, 25, 2, "RAW Sensor Data:");
  SetLines(WHITE, 10, 50, 195, 50);
  if (update_values == true) {
    int fwd = analogRead(analog_fwd_Pin);    // read from sensor pin value:0-1024
    int rfl = analogRead(analog_rfl_Pin);    // read from sensor pin value:0-1024
    int batt = analogRead(analog_batt_Pin);  // read from sensor pin value:0-1024

    SetFilledRect(BLACK , 160, 80, 150, 16);
    ScreenText(WHITE, 10, 80, 2 , "Pin:" + String(analog_fwd_Pin) + " > A/D: " + String ((fwd * to_mV), 1) + " mV");

    SetFilledRect(BLACK , 160, 100, 150, 16);
    ScreenText(WHITE, 10, 100, 2 , "Pin:" + String(analog_rfl_Pin) + " > A/D: " + String ((rfl * to_mV), 1) + " mV");

    SetFilledRect(BLACK , 160, 120, 150, 16);
    ScreenText(WHITE, 10, 120, 2 , "Pin:" + String(analog_batt_Pin) + " > A/D: " + String ((batt * to_mV), 1) + " mV");

    if (menue_3_choose == 0) {
      SetFilledRect(BLACK , 0, 0, 5, 300);
      SetFilledCircle(RED , 2, 88, 2);
      show_needle(float(fwd));
    }
    if (menue_3_choose == 1) {
      SetFilledRect(BLACK , 0, 0, 5, 300);
      SetFilledCircle(RED , 2, 108, 2);
      show_needle(float(rfl));
    }
    if (menue_3_choose == 2) {
      SetFilledRect(BLACK , 0, 0, 5, 300);
      SetFilledCircle(RED , 2, 128, 2);
      show_needle(float(batt));
    }

    update_values = false;
  }
}
//--------------------------------------------------------------------------------------------------------
void menue_4() {

  ScreenText(WHITE, 10, 25, 2, "PTT Interruption:");
  SetLines(WHITE, 10, 50, 210, 50);

  SetFilledRect(BLACK , 55, 80, 250, 16);
  ScreenText(WHITE, 10, 80, 2 , "TX :" + String(ptt_interrupt_watt) + " W");

  if (ptt_interrupt_watt == 0) {
    ScreenText(RED, 220, 80, 2 , "Off");
  }

  ScreenText(WHITE, 10, 100, 2 , "SWR:" + String(ptt_interrupt_swr));

  SetFilledRect(BLACK , 0, 0, 5, 300);
  SetFilledCircle(RED , 2, 88, 2);
}
//--------------------------------------------------------------------------------------------------------
void show_needle(float neddle_value) {

  //Position of instrument:
  int xoffset = x_edge_right / 2;
  int yoffset = y_edge_down - 10;

  //scale drawing:
  SetRect(WHITE , xoffset - 150 , yoffset - 149, 300, 160); //frame
  for (float i = 180; i <= 360; i = i + 4) {
    int scale_xpos = (cos(i * pi_div_180)  * 125) + xoffset;;
    int scale_ypos = (sin(i * pi_div_180)  * 125) + yoffset;;
    if (i <= 360) SetFilledCircle(RED , scale_xpos, scale_ypos, 2); //colored scale
    if (i < 315) SetFilledCircle(ORANGE , scale_xpos, scale_ypos, 2);
    if (i < 270) SetFilledCircle(YELLOW , scale_xpos, scale_ypos, 2);
    if (i < 225)  SetFilledCircle(GREEN , scale_xpos, scale_ypos, 2);
  }

  //needle calculation:
  float alfa = ((neddle_value * 0.17578125) + 180) * pi_div_180; // pi/180=0.017453293
  int  needle_xpos = (cos(alfa)  * 120) + xoffset;
  int needle_ypos = (sin(alfa)  * 120) + yoffset;
  SetLines(BLACK , xoffset, yoffset, old_needle_xpos, old_needle_ypos); //old needle
  old_needle_xpos = needle_xpos;
  old_needle_ypos = needle_ypos;
  SetLines(RED , xoffset, yoffset, needle_xpos, needle_ypos); //new needle
  SetFilledCircle(GRAY , xoffset, yoffset, 5); //needle turn point
}
//--------------------------------------------------------------------------------------------------------
void show_cross_needle(float neddle_left_value, float neddle_right_value) { //show swr

  //Position of instrument:
  int xoffset_left = x_edge_left + 140;
  int xoffset_right = x_edge_right - 140;
  int yoffset = y_edge_down - 10;

  //scale drawing:
  SetRect(WHITE , x_edge_left , y_edge_up, x_edge_right, y_edge_down); //frame
  //SetLines(GREEN , x_edge_right / 2 , y_edge_down - 22, 415, 242); //swr=1,2
  ScreenText(WHITE, 20, 50, 2 , "FWD");
  ScreenText(WHITE, 420, 50, 2 , "RFL");
  //ScreenText(WHITE, 430, 235, 1 , "1.2 SWR");

  //left scale:
  for (float i = 270; i <= 355; i = i + 4) {
    int scale_xpos = (cos(i * pi_div_180) * 290) + xoffset_left;
    int scale_ypos = (sin(i * pi_div_180) * 290) + yoffset;
    SetFilledCircle(WHITE , scale_xpos, scale_ypos, 2);
  }

  //right scale:
  for (float i = 185; i <= 270; i = i + 4) {
    int scale_xpos = (cos(i * pi_div_180) * 290) + xoffset_right;
    int scale_ypos = (sin(i * pi_div_180) * 290) + yoffset;
    SetFilledCircle(WHITE , scale_xpos, scale_ypos, 2);
  }

  //turn left needle calculation:
  float alfa_left = (355 - (neddle_left_value * 0.17578125)) * pi_div_180; // pi/180=0.017453293
  int needle_left_xpos = (cos(alfa_left) * 285) + xoffset_left;
  int needle_left_ypos = (sin(alfa_left) * 285) + yoffset;
  SetLines(BLACK , xoffset_left, yoffset, old_needle_left_xpos, old_needle_left_ypos); //old needle
  old_needle_left_xpos = needle_left_xpos;
  old_needle_left_ypos = needle_left_ypos;
  SetLines(RED , xoffset_left, yoffset, needle_left_xpos, needle_left_ypos); //new needle
  SetFilledCircle(GRAY , xoffset_left, yoffset, 5); //needle turn point

  //turn right needle calculation:
  float alfa_right = ((neddle_right_value * 0.17578125) + 185) * pi_div_180; // pi/180=0.017453293
  int needle_right_xpos = (cos(alfa_right) * 285) + xoffset_right;
  int needle_right_ypos = (sin(alfa_right) * 285) + yoffset;
  SetLines(BLACK , xoffset_right, yoffset, old_needle_right_xpos, old_needle_right_ypos); //old needle
  old_needle_right_xpos = needle_right_xpos;
  old_needle_right_ypos = needle_right_ypos;
  SetLines(RED , xoffset_right, yoffset, needle_right_xpos, needle_right_ypos); //new needle
  SetFilledCircle(GRAY , xoffset_right, yoffset, 5); //needle turn point
}
//--------------------------------------------------------------------------------------------------------
void timer_interrupt() {

  update_values = true;
}
//--------------------------------------------------------------------------------------------------------
