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

int analog_fwd_Pin = A0;
int analog_rfl_Pin = A1;
int analog_batt_Pin = A2;
int band_val = 0;
float band_factor = 1;

//calculate the incomming dc-voltage from SWR-Bridge in hf-current:

// 160m > 1500W > P = I * I * 50 Ohm > I = 5,477 A > A/D = 5000mV > A/D = 1023 bit;
// 160m > 1000W > P = I * I * 50 Ohm > I = 4.472 A > A/D = 4083mV > A/D =  836 bit;
// 160m >  500W > P = I * I * 50 Ohm > I = 3.162 A > A/D = 2887mV > A/D =  591 bit;
// 160m >  100W > P = I * I * 50 Ohm > I = 1.414 A > A/D = 1291mV > A/D =  264 bit;
// 160m >   50W > P = I * I * 50 Ohm > I = 1.000 A > A/D =  913mV > A/D =  187 bit;
// 160m >   10W > P = I * I * 50 Ohm > I = 0.616 A > A/D =  562mV > A/D =  115 bit;

const float divisor_factor = 5.477 / 1023;//0.005348633;
const String band_names [8] = {"160m", " 80m", " 40m", " 30m", " 20m", " 17m", " 15m", " 10m"};
const float band_factors [8] = {1, 1.046, 1.124, 1.175, 1.225, 1.280, 1.350, 1.410}; //160m-10m > // correction of the swr-bridge

boolean update_values = false;
boolean force_update_values = false;

//hf values
int peak_value = 0;
int peak_bar = 0;
int old_peak_bar = 1;
uint16_t peak_reset = 0;
int old_band = 0;
int old_fwd = 0;
int old_rfl = 0;
float old_swr = 0;

//rotary encoder
#define encoder0PinA  2
#define encoder0PinB  3
volatile unsigned int encoder0Pos = 0;

//button
int button_Pin = 4; // Connect the Button to Ground and to button_pin
int button_status = 0;

//menue
int menue_level = 0;
boolean use_batt = false;
//#########################################################################
//#########################################################################
void setup() {

  pinMode(analog_fwd_Pin, INPUT);
  pinMode(analog_rfl_Pin, INPUT);
  pinMode(analog_batt_Pin, INPUT);
  pinMode (button_Pin, INPUT_PULLUP);
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE); // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE); // encoder pin on interrupt 1 (pin 3)

  Serial.begin(9600);
  tft.init();
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  ScreenText(WHITE, 10, 10 , 2, F("HF-Power Meter"));// Arduino IDE 1.6.11
  ScreenText(WHITE, 10, 40 , 2, F("V0.2-Beta"));
  delay(2000);
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
    hf_power();
    batterie();
    encoder_button();
    force_update_values = false;//force to update values
  }
  if (menue_level == 1) {
    menue_1();
    encoder_button();
  }
  if (menue_level == 2) {
    menue_2();
    encoder_button();
  }
  if (menue_level == 3) {
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

  force_update_values = true;
}
//--------------------------------------------------------------------------------------------------------
void hf_power() {  //show FWD / RFL / SWR / Peak-Power

  int fwd = analogRead(analog_fwd_Pin);    // read from sensor pin value:0-1024
  int rfl = analogRead(analog_rfl_Pin);    // read from sensor pin value:0-1024

  //Serial.println(String(fwd));
  //fwd = 836; // 1500W / 160m
  //rfl = 20;

  if (fwd > 0 && fwd < 145) { //145 = 0.7V Diode
    SetFilledTriangle(RED , 200, 290, 192 , 306 , 208 , 306);//291
    ScreenText(RED, 220, 291, 2 , "Not Exact !");
  }
  else {
    SetFilledRect(BLACK , 190, 291, 160, 16);
  }

  float fwd_float = float(fwd) * band_factor * divisor_factor;
  float rfl_float = float(rfl) * band_factor * divisor_factor;
  float fwd_watt = fwd_float * fwd_float * 50;
  float rfl_watt = rfl_float * rfl_float * 50;

  if (fwd == 0)peak_reset++;
  if (fwd > 0)peak_reset = 0;

  if (update_values == true || force_update_values == true) {
    update_values = false;
    if (old_fwd != fwd || force_update_values == true) {
      old_fwd = fwd;
      SetFilledRect(BLACK , 40, 70, 60, 8);
      if (fwd > 1)ScreenText(WHITE, 10, 70, 1 , "FWD: " + String (fwd_watt, 1) + " W");
      //dBm calculation
      double dBm = 30 + (10 * log10(double(fwd_watt)));
      SetFilledRect(BLACK , 70, 25, 100, 16);
      if (fwd > 1)ScreenText(WHITE, 70, 25, 2 , String (dBm, 1) + " dBm");
    }

    if (old_rfl != rfl || force_update_values == true) {
      old_rfl = rfl;
      SetFilledRect(BLACK , 40, 120, 60, 8);
      if (rfl > 1)ScreenText(WHITE, 10, 120, 1 , "RFL: " + String (rfl_watt, 1) + " W");
    }
  }

  //fwd bar:
  int fwd_bar = int(fwd_watt * 0.312);
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
  int rfl_bar = int(rfl_watt * 0.312);
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

  //SWR
  if (update_values == true || force_update_values == true) {
    float swr = (float(fwd) + float(rfl)) / (float(fwd) - float(rfl));
    if (swr > 100)swr = 100;
    if (old_swr != swr && fwd > 0) {
      old_swr = swr;
      if (swr >= 3)SetRect(RED , 10, 218, 145, 40);
      if (swr < 3)SetRect(BLACK , 10, 218, 145, 40);
      SetFilledRect(BLACK , 80, 230, 70, 16);
      ScreenText(WHITE, 20, 230, 2 , "SWR: " + String (swr, 1));
    }
  }

  //Peak Value
  if (fwd_bar > peak_bar || force_update_values == true) {
    peak_bar = fwd_bar;
    peak_value = fwd;
  }
  if (peak_reset > 150) {
    peak_reset = 0;
    peak_bar = 1;
    peak_value = 0;
  }
  if (peak_bar > 475) peak_bar = 475;
  if (old_peak_bar != peak_bar) {
    SetTriangle(BLACK , old_peak_bar, 102, old_peak_bar - 4, 108, old_peak_bar + 4, 108);
    old_peak_bar = peak_bar;
    float peak_float = float(peak_value) * band_factor * divisor_factor; //value:0-470
    float peak_watt = peak_float * peak_float * 50;
    SetFilledRect(BLACK , 230, 70, 60, 8);
    if (peak_value > 1)ScreenText(WHITE, 200, 70, 1, "PEAK: " + String (peak_watt, 1) + " W");
  }
  SetTriangle(WHITE , old_peak_bar, 102, old_peak_bar - 4, 108, old_peak_bar + 4, 108);
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
  // use for debugging - remember to comment out
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
      if (band_val > 7)band_val = 0;
    }
    if (menue_level == 1 ) {

    }
    if (menue_level == 2 ) {

    }
  }
  //---------------------------
  if (enc_direction == false) { //turn left
    if (menue_level == 0 ) {
      band_val--;
      if (band_val < 0)band_val = 7;
    }
    if (menue_level == 1 ) {

    }
    if (menue_level == 2 ) {

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
    if (menue_level == 2 ) {
      menue_level++;
      force_update_values = true;//force to update values
      tft.fillScreen(BLACK);
    }
    if (menue_level == 1 ) {
      menue_level++;
      tft.fillScreen(BLACK);
    }
    if (menue_level == 0 ) {
      menue_level++;
      tft.fillScreen(BLACK);
    }
    //todo
    button_status--;
  }
}
//-------------------------------------------------------------------------------------------------------
void menue_1() {

  ScreenText(WHITE, 10, 25, 2, "SWR Calibration Factors:");
  SetLines(WHITE, 10, 50, 290, 50);
  for (int i = 0; i < 8; i++) {
    ScreenText(WHITE, 10, 80 + (20 * i), 2, band_names[i] + ": " + String(band_factors [i], 3));
  }
  ScreenText(WHITE, 10, 270, 2, "Divisor: " + String(divisor_factor, DEC));

  SetLines(WHITE, 180, 80, 180, 235);//Coordinate y
  SetLines(WHITE, 180, 235, 479, 235);//Coordinate x
  for (int i = 0; i < 8; i++) { //scala X
    SetLines(WHITE, 180 + (i * 40), 233, 180 + (i * 40), 237);
    ScreenText(WHITE, 170 + (i * 40), 243, 1, band_names [i]);
  }
  SetLines(RED, 181, (100 * band_factors [0]), 220, 100 * band_factors [1]); //160m to 80m
  SetLines(RED, 220, (100 * band_factors [1]), 260, 100 * band_factors [2]); //80m to 40m
  SetLines(RED, 260, (100 * band_factors [2]), 300, 100 * band_factors [3]); //40m to 30m
  SetLines(RED, 300, (100 * band_factors [3]), 340, 100 * band_factors [4]); //30m to 20m
  SetLines(RED, 340, (100 * band_factors [4]), 380, 100 * band_factors [5]); //20m to 17m
  SetLines(RED, 380, (100 * band_factors [5]), 420, 100 * band_factors [6]); //17m to 15m
  SetLines(RED, 420, (100 * band_factors [6]), 460, 100 * band_factors [7]); //15m to 10m
}
//-------------------------------------------------------------------------------------------------------
void menue_2() {

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

    update_values = false;
  }
}
//--------------------------------------------------------------------------------------------------------
void timer_interrupt() {

  update_values = true;
}
//--------------------------------------------------------------------------------------------------------
