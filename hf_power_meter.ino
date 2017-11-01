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

    10 W = 0.4472 A an 50 Ohm  > P = (I*I)/R
    Die ausgekoppelte FWD Spannung ist dem HF-Strom proportional.
*/
#include <TimerOne.h>
#include <TFT_HX8357.h> // Hardware-specific library
#include "Free_Fonts.h"

TFT_HX8357 tft = TFT_HX8357();

// Assign human-readable names to some common 16-bit color values:
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
const int time_factor = 20;//microseconds > reduce the ghost pixel

const float pi = 3.14159265;

int analog_fwd_Pin = A0;
int analog_rfl_Pin = A1;
int band_val = 1;
float band_factor = 1;

int counter = 0;
boolean show_values = false;

int peak_value = 0;
int peak_bar = 0;
int old_peak_bar = 1;
uint16_t peak_reset = 0;
String old_band = "/";
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
boolean ad_values = true;
//#########################################################################
//#########################################################################
void setup() {

  pinMode(analog_fwd_Pin, INPUT);
  pinMode(analog_rfl_Pin, INPUT);
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
  ScreenText(WHITE, x_edge_left, 10 , 2, F("HF-Power Meter"));// Arduino IDE 1.6.11
  ScreenText(WHITE, x_edge_left, 40 , 2, F("V0.1-Beta"));
  delay(2000);
  tft.fillScreen(BLACK);
  scale();

  Timer1.initialize(30000000);//microseconds = 30 seconds
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
    button();
  }
  if (menue_level == 1) {
    ScreenText(WHITE, 10, 25, 2, "Menue 1:");
    button();
  }
  if (menue_level == 2) {
    ScreenText(WHITE, 10, 25, 2, "Menue 2:");
    button();
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
  delayMicroseconds(time_factor);// eleminates the ghost pixel
  tft.setCursor(xtpos, ytpos);
  delayMicroseconds(time_factor);// eleminates the ghost pixel
  tft.setTextFont(1);
  delayMicroseconds(time_factor);// eleminates the ghost pixel
  tft.setTextColor(color);
  delayMicroseconds(time_factor);// eleminates the ghost pixel
  tft.setTextSize(text_size);
  delayMicroseconds(time_factor);// eleminates the ghost pixel
  tft.println(text);
  delayMicroseconds(time_factor);// eleminates the ghost pixel
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
  delayMicroseconds(time_factor);// eleminates the ghost pixel
  SetRect(WHITE , x_edge_left, 130, x_edge_right, 10); //RFL
  delayMicroseconds(time_factor);// eleminates the ghost pixel

  for (int x = 0; x < x_edge_right; x += 32) {
    SetPoint(WHITE, x, 101);
    delayMicroseconds(time_factor);// eleminates the ghost pixel
    SetPoint(WHITE, x, 141);
    delayMicroseconds(time_factor);// eleminates the ghost pixel
  }

  ScreenText(WHITE, 10, 25, 2, "PWR:");
  delayMicroseconds(time_factor);// eleminates the ghost pixel
  ScreenText(WHITE, 300, 25, 2, "BAND: ");
  delayMicroseconds(time_factor);// eleminates the ghost pixel
}
//--------------------------------------------------------------------------------------------------------
void hf_power() {  //show FWD / RFL / SWR / Peak-Power

  counter++;
  if (counter == 100)show_values = true;

  int fwd = analogRead(analog_fwd_Pin);    // read from sensor pin value:0-1024
  int rfl = analogRead(analog_rfl_Pin);    // read from sensor pin value:0-1024
  //Serial.println(String(fwd));
  //fwd = 680; // 680 A/D = 10 W / 160m
  //rfl = 40;  // 40 A/D = 0.1 W

  // band_factor = 0.7; the value comes from the band switch > correction of the swr-bridge
  float factor_1 = 0.00324; // 0.00081; // calculate the power in watt
  float fwd_float = float(fwd) * band_factor * factor_1; //value:0-470
  float rfl_float = float(rfl) * band_factor * factor_1; //value:0-470
  float fwd_watt = fwd_float * fwd_float * 50;
  float rfl_watt = rfl_float * rfl_float * 50;

  if (fwd == 0)peak_reset++;
  if (fwd > 0)peak_reset = 0;

  if (show_values == true) {

    //Test:
    if (ad_values == true) {
      SetFilledRect(BLACK , 20, 280, 200, 16);
      delayMicroseconds(time_factor);// eleminates the ghost pixel
      ScreenText(WHITE, 20, 280, 2 , "A/D: " + String ((fwd * 4.8828125), 1) + " mV");
      SetFilledRect(BLACK , 20, 300, 200, 16);
      delayMicroseconds(time_factor);// eleminates the ghost pixel
      ScreenText(WHITE, 20, 300, 2 , "A/D: " + String ((rfl * 4.8828125), 1) + " mV");
    }
    //Test

    if (old_fwd != fwd) {
      old_fwd = fwd;
      SetFilledRect(BLACK , 40, 70, 60, 8);
      delayMicroseconds(time_factor);// eleminates the ghost pixel
      if (fwd > 1)ScreenText(WHITE, 10, 70, 1 , "FWD: " + String (fwd_watt, 1) + " W");
      delayMicroseconds(time_factor);// eleminates the ghost pixel
      //dBm calculation
      double dBm = 30 + (10 * log10(double(fwd_watt)));
      SetFilledRect(BLACK , 70, 25, 100, 16);
      delayMicroseconds(time_factor);// eleminates the ghost pixel
      if (fwd > 1)ScreenText(WHITE, 70, 25, 2 , String (dBm, 1) + " dBm");
      delayMicroseconds(time_factor);// eleminates the ghost pixel

    }
    if (old_rfl != rfl) {
      old_rfl = rfl;
      SetFilledRect(BLACK , 40, 120, 60, 8);
      delayMicroseconds(time_factor);// eleminates the ghost pixel
      if (rfl > 1)ScreenText(WHITE, 10, 120, 1 , "RFL: " + String (rfl_watt, 1) + " W");
      delayMicroseconds(time_factor);// eleminates the ghost pixel
    }
  }
  //fwd bar:  10W = 469 A/D
  int fwd_bar = float(fwd) * 0.46;
  if (fwd_bar > x_edge_right - 1)fwd_bar = x_edge_right - 1;
  uint16_t fwd_color;
  for (int i = 2; i < x_edge_right - 6; i += 6) {
    if (i < x_edge_right) fwd_color = RED;
    if (i < 355) fwd_color = ORANGE;
    if (i < 240) fwd_color = YELLOW;
    if (i < 125) fwd_color = GREEN;
    if (i > fwd_bar) fwd_color = BLACK;
    SetFilledRect(fwd_color , i, 81, 5, 18);
    delayMicroseconds(time_factor);// eleminates the ghost pixel
  }

  //rfl bar:
  int rfl_bar = float(rfl) * 0.46;
  if (rfl_bar > x_edge_right - 1)rfl_bar = x_edge_right - 1;
  uint16_t rfl_color;
  for (int y = 2; y < x_edge_right - 6; y += 6) {
    if (y < x_edge_right) rfl_color = RED;
    if (y < 355) rfl_color = ORANGE;
    if (y < 240) rfl_color = YELLOW;
    if (y < 125) rfl_color = GREEN;
    if (y > rfl_bar) rfl_color = BLACK;
    SetFilledRect(rfl_color , y, 131, 5, 8) ;
    delayMicroseconds(time_factor);// eleminates the ghost pixel
  }

  //SWR
  if (show_values == true) {
    float swr = (float(fwd) + float(rfl)) / (float(fwd) - float(rfl));
    if (swr > 100)swr = 100;
    if (old_swr != swr && fwd > 0) {
      old_swr = swr;
      if (swr >= 3)SetRect(RED , 10, 218, 145, 40);
      if (swr < 3)SetRect(BLACK , 10, 218, 145, 40);
      SetFilledRect(BLACK , 80, 230, 70, 16);
      delayMicroseconds(time_factor);// eleminates the ghost pixel
      ScreenText(WHITE, 20, 230, 2 , "SWR: " + String (swr, 1));
      delayMicroseconds(time_factor);// eleminates the ghost pixel
    }
  }

  //Peak Value
  if (fwd_bar > peak_bar) {
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
    delayMicroseconds(time_factor);// eleminates the ghost pixel
    old_peak_bar = peak_bar;
    float peak_float = float(peak_value) * band_factor * factor_1; //value:0-470
    float peak_watt = peak_float * peak_float * 50;
    SetFilledRect(BLACK , 230, 70, 60, 8);
    delayMicroseconds(time_factor);// eleminates the ghost pixel
    if (peak_value > 1)ScreenText(WHITE, 200, 70, 1, "PEAK: " + String (peak_watt, 1) + " W");
  }
  delayMicroseconds(time_factor);// eleminates the ghost pixel
  SetTriangle(WHITE , old_peak_bar, 102, old_peak_bar - 4, 108, old_peak_bar + 4, 108);

  if (counter == 100) {
    counter = 0;
    show_values = false;
  }
}
//--------------------------------------------------------------------------------------------------------
void band() { //show the used band

  String band_value;

  if (band_val == 1) {
    band_value = "160m";
    band_factor =  1; // factor for calculation of fwd and rfl for different bands
  }
  if (band_val == 2) {
    band_value = "80m";
    band_factor =  1.046;
  }
  if (band_val == 3) {
    band_value = "40m";
    band_factor =  1.124;
  }
  if (band_val == 4) {
    band_value = "30m";
    band_factor = 1.175 ;
  }
  if (band_val == 5) {
    band_value = "20m";
    band_factor =  1.225;
  }
  if (band_val == 6) {
    band_value = "17m";
    band_factor =  1.28;
  }
  if (band_val == 7) {
    band_value = "15m";
    band_factor =  1.35;
  }
  if (band_val == 8) {
    band_value = "10m";
    band_factor =  1.41;
  }

  if (old_band != band_value) {
    old_band = band_value;
    SetFilledRect(BLACK , 370, 25, 100, 16) ;
    delayMicroseconds(time_factor);// eleminates the ghost pixel
    ScreenText(WHITE, 370, 25, 2, band_value );
    delayMicroseconds(time_factor);// eleminates the ghost pixel
  }
}
//--------------------------------------------------------------------------------------------------------
void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos++;         // CW
      band_val++;
      if (band_val > 8)band_val = 1;
      Serial.println ("A:H / B:L +");
    }
    else {
      encoder0Pos--;         // CCW
      band_val--;
      if (band_val < 1)band_val = 8;
      Serial.println ("A:H / B:H -");
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos++;          // CW
      band_val++;
      if (band_val > 8)band_val = 1;
      Serial.println ("A:L / B:H +");
    }
    else {
      encoder0Pos--;          // CCW
      band_val--;
      if (band_val < 1)band_val = 8;
      Serial.println ("A:L / B:L -");
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
      encoder0Pos++;// CW
      band_val++;
      if (band_val > 8)band_val = 1;
      Serial.println ("B:H / A:H +");
    }
    else {
      encoder0Pos--;// CCW
      band_val--;
      if (band_val < 1)band_val = 8;
      Serial.println ("B:H / A:L -");
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos++;// CW
      band_val++;
      if (band_val > 8)band_val = 1;
      Serial.println ("B:L / A:L +");
    }
    else {
      encoder0Pos--;// CCW
      band_val--;
      if (band_val < 1)band_val = 8;
      Serial.println ("B:L / A:H -");
    }
  }
}
//--------------------------------------------------------------------------------------------------------
void button() { //enter the menue

  int button_val;
  button_val = digitalRead(button_Pin);
  if (button_val == LOW & button_status == 0)button_status++;
  if (button_val == HIGH & button_status == 1) {
    //todo:
    menue_level++;
    delayMicroseconds(time_factor);// eleminates the ghost pixel
    tft.fillScreen(BLACK);
    delayMicroseconds(time_factor);// eleminates the ghost pixel
    //todo
    button_status--;
  }
}
//--------------------------------------------------------------------------------------------------------
void timer_interrupt() {


}
//--------------------------------------------------------------------------------------------------------
