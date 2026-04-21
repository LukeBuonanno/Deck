#include "card_bitmaps.h"
#include <GxEPD.h>
#include <GxDEPG0150BN/GxDEPG0150BN.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include <WiFi.h>
#include "time.h"
#include "HTTPClient.h"
#include <ArduinoJson.h>
#include <AceButton.h>
#include <Adafruit_I2CDevice.h>
#include <TinyGPS++.h>
#include <ESP32Time.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSerifItalic18pt7b.h>
#include <Fonts/FreeSerifItalic12pt7b.h>
#include <Fonts/FreeSerifItalic9pt7b.h>
#include <math.h>
#include <WiFiClientSecure.h>

// ── Pin definitions ────────────────────────────────
#define GPS_RES    23
#define GPS_RX     21
#define GPS_TX     22
#define PIN_MOTOR   4
#define PIN_KEY    35
#define PWR_EN      5
#define Backlight  33
#define Bat_ADC    34
#define SPI_SCK    14
#define SPI_DIN    13
#define EPD_CS     15
#define EPD_DC      2
#define SRAM_CS    -1
#define EPD_RESET  17
#define EPD_BUSY   16

// ── Card geometry ──────────────────────────────────
#define CARD_W            150
#define CARD_H            211
#define CARD_R            15      // corner radius

// ── Layout / angle constants ───────────────────────
#define FRONT_ANGLE       -60.0f
#define PIVOT_X           218.0f
#define PIVOT_Y           152.0f
#define OFFSET_B          -9.5f
#define OFFSET_C          -16.5f
#define OFFSET_D          -22.5f
#define PIVOT_STEP        12.0f   // px shift per back card

// ── Pip layout constants ───────────────────────────
#define PIP_RADIUS        15.0f
#define ACE_RADIUS        30.0f
#define PIP_COL_OFFSET    35.0f
#define PIP_TOP_Y         44.0f
#define PIP_ROW_STEP      40.0f
#define PIP_MID7_Y        126.0f
#define CORNER_PIP_RADIUS  8.0f

// ── Shadow constants ───────────────────────────────
#define SHADOW_DX          -5      // right offset (px)
#define SHADOW_DY          -7      // down  offset (px)
#define SHADOW_SPREAD      3      // expand beyond card edges (px)
const char *portalUser = "lukebuonanno@kings.edu";
const char *portalPass = "PinkLemons123*";


// ═══════════════════════════════════════════════════
//  Pip Y lookup — card-local coordinate
// ═══════════════════════════════════════════════════
static float pipY(uint8_t key)
{
  switch(key){
    case 0: return PIP_TOP_Y;
    case 1: return PIP_TOP_Y + PIP_ROW_STEP * 0.5f;
    case 2: return PIP_TOP_Y + PIP_ROW_STEP;
    case 3: return CARD_H * 0.5f;
    case 4: return PIP_MID7_Y;
    case 5: return CARD_H - PIP_MID7_Y;
    case 6: return CARD_H - PIP_TOP_Y - PIP_ROW_STEP;
    case 7: return CARD_H - PIP_TOP_Y - PIP_ROW_STEP * 0.5f;
    case 8: return CARD_H - PIP_TOP_Y;
    default: return CARD_H * 0.5f;
  }
}

// ═══════════════════════════════════════════════════
//  Pip table — {col, yKey, flip}
//  col: -1=left  0=centre  1=right
// ═══════════════════════════════════════════════════
struct PipDef { int8_t col; uint8_t yKey; bool flip; };

static const PipDef PIP_TABLE[10][10] PROGMEM = {
  // A
  {{ 0,3,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false}}, 
  // 2
  {{ 0,0,false},{ 0,8,true },{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false}},
  // 3
  {{ 0,0,false},{ 0,3,false},{ 0,8,true },{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false}},
  // 4
  {{-1,0,false},{ 1,0,false},{-1,8,true },{ 1,8,true },{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false}},
  // 5
  {{-1,0,false},{ 1,0,false},{ 0,3,false},{-1,8,true },{ 1,8,true },{99,0,false},{99,0,false},{99,0,false},{99,0,false},{99,0,false}},
  // 6
  {{-1,0,false},{ 1,0,false},{-1,3,false},{ 1,3,false},{-1,8,true },{ 1,8,true },{99,0,false},{99,0,false},{99,0,false},{99,0,false}},
  // 7
  {{-1,0,false},{ 1,0,false},{ 0,4,false},{-1,3,false},{ 1,3,false},{-1,8,true },{ 1,8,true },{99,0,false},{99,0,false},{99,0,false}},
  // 8
  {{-1,0,false},{ 1,0,false},{ 0,4,false},{-1,3,false},{ 1,3,false},{ 0,5,true },{-1,8,true },{ 1,8,true },{99,0,false},{99,0,false}},
  // 9
  {{-1,0,false},{ 1,0,false},{-1,2,false},{ 1,2,false},{ 0,3,false},{-1,6,true },{ 1,6,true },{-1,8,true },{ 1,8,true },{99,0,false}},
  // 10
  {{-1,0,false},{ 1,0,false},{ 0,1,false},{-1,2,false},{ 1,2,false},{-1,6,true },{ 1,6,true },{ 0,7,true },{-1,8,true },{ 1,8,true }},
};

static const uint8_t PIP_COUNT[10] PROGMEM = {1,2,3,4,5,6,7,8,9,10};

// ═══════════════════════════════════════════════════
//  Globals
// ═══════════════════════════════════════════════════
using namespace ace_button;

GxIO_Class  io(SPI, EPD_CS, EPD_DC, EPD_RESET);
GxEPD_Class display(io, EPD_RESET, EPD_BUSY);
struct tm timeinfo;

const char *ntpServer1         = "pool.ntp.org";
const char *ntpServer2         = "ntp1.aliyun.com";
const long  gmtOffset_sec      = 60 * 60 * 8;
const int   daylightOffset_sec = 0;
const char *ssid     = "LeoNet";
const char *password = "";
const char *host     = "https://api.seniverse.com";
const char *apiKey   = "SOLy-aC_1gUQ-v6js";
const char *city     = "shenzhen";

HTTPClient              http_client;
String                  req, rsp;
StaticJsonDocument<500> doc;
AceButton               button(PIN_KEY);
HardwareSerial          gpsPort(Serial2);
TinyGPSPlus             gps;
ESP32Time               rtc;

bool    SleepFlag         = false;
bool    WIFIUpdataFlag    = false;
bool    GPSUpdataFlag     = false;
bool    DisplayFullUpdata = true;
bool    inMenu            = false;
uint8_t menuIndex         = 0;
uint8_t RealTime = 0, LastTime = 0;
float   Lat = 0, Long = 0, Speed = 0;
uint8_t Sat = 0;
uint8_t currentSuit = 0;
uint8_t lastHour    = 99;

// ── Forward declarations ───────────────────────────
void Task1code(void *pvParameters);
void Task2code(void *pvParameters);
void drawWatchFace();
void drawMenu();
void menuAction(uint8_t idx);
void hourToCard(uint8_t hour, char *rankStr, uint8_t &pips);
void randomiseSuit();
uint8_t getBatteryPercent();
void drawBatteryIcon(uint16_t x, uint16_t y);
void rotatePoint(float pivX, float pivY, float deg, float &x, float &y);
void drawCardShadow(float pivX, float pivY, float angleDeg);
void drawCardAtAngle(float pivX, float pivY, float angleDeg,
                     const char *rank, uint8_t rankIndex,
                     uint8_t suit, bool showPips);
void drawSuit(float cx, float cy, float r, uint8_t suit, float angleDeg);
void drawSpadeAt(float cx, float cy, float r, float angleDeg);
void drawHeartAt(float cx, float cy, float r, float angleDeg);
void drawDiamondAt(float cx, float cy, float r, float angleDeg);
void drawClubAt(float cx, float cy, float r, float angleDeg);
void drawRankBitmap(float cx, float cy, const char *rank,
                    float angleDeg, uint16_t color);

// ═══════════════════════════════════════════════════
//  Rotation helper
// ═══════════════════════════════════════════════════
void rotatePoint(float pivX, float pivY, float deg, float &x, float &y)
{
  float r = deg * (float)M_PI / 180.0f;
  float c = cosf(r), s = sinf(r);
  float dx = x - pivX, dy = y - pivY;
  x = pivX + dx * c - dy * s;
  y = pivY + dx * s + dy * c;
}

// ═══════════════════════════════════════════════════
//  Suit drawers
// ═══════════════════════════════════════════════════
void drawSpadeAt(float cx, float cy, float r, float angleDeg)
{
  float tx0=0, ty0=-r;
  float tx1=-r*0.9f, ty1=r*0.35f;
  float tx2= r*0.9f, ty2=r*0.35f;
  rotatePoint(0,0,angleDeg,tx0,ty0);
  rotatePoint(0,0,angleDeg,tx1,ty1);
  rotatePoint(0,0,angleDeg,tx2,ty2);
  display.fillTriangle(
    (int16_t)(cx+tx0),(int16_t)(cy+ty0),
    (int16_t)(cx+tx1),(int16_t)(cy+ty1),
    (int16_t)(cx+tx2),(int16_t)(cy+ty2), GxEPD_BLACK);

  float lx=-r*0.42f, ly=r*0.12f;
  rotatePoint(0,0,angleDeg,lx,ly);
  display.fillCircle((int16_t)(cx+lx),(int16_t)(cy+ly),(int16_t)(r*0.52f),GxEPD_BLACK);

  float rx2=r*0.42f, ry2=r*0.12f;
  rotatePoint(0,0,angleDeg,rx2,ry2);
  display.fillCircle((int16_t)(cx+rx2),(int16_t)(cy+ry2),(int16_t)(r*0.52f),GxEPD_BLACK);

  float s0x=-r*0.18f, s0y=r*0.38f;
  float s1x= r*0.18f, s1y=r*0.38f;
  float s2x= r*0.55f, s2y=r;
  float s3x=-r*0.55f, s3y=r;
  rotatePoint(0,0,angleDeg,s0x,s0y); rotatePoint(0,0,angleDeg,s1x,s1y);
  rotatePoint(0,0,angleDeg,s2x,s2y); rotatePoint(0,0,angleDeg,s3x,s3y);
  display.fillTriangle(
    (int16_t)(cx+s0x),(int16_t)(cy+s0y),
    (int16_t)(cx+s1x),(int16_t)(cy+s1y),
    (int16_t)(cx+s2x),(int16_t)(cy+s2y), GxEPD_BLACK);
  display.fillTriangle(
    (int16_t)(cx+s0x),(int16_t)(cy+s0y),
    (int16_t)(cx+s2x),(int16_t)(cy+s2y),
    (int16_t)(cx+s3x),(int16_t)(cy+s3y), GxEPD_BLACK);
}

void drawHeartAt(float cx, float cy, float r, float angleDeg)
{
  float lx=-r*0.5f, ly=-r*0.2f;
  float rx2= r*0.5f, ry2=-r*0.2f;
  rotatePoint(0,0,angleDeg,lx,ly);
  rotatePoint(0,0,angleDeg,rx2,ry2);
  display.fillCircle((int16_t)(cx+lx),(int16_t)(cy+ly),(int16_t)(r*0.55f),GxEPD_BLACK);
  display.fillCircle((int16_t)(cx+rx2),(int16_t)(cy+ry2),(int16_t)(r*0.55f),GxEPD_BLACK);

  float t0x=-r, t0y=r*0.1f;
  float t1x= r, t1y=r*0.1f;
  float t2x= 0, t2y=r;
  rotatePoint(0,0,angleDeg,t0x,t0y);
  rotatePoint(0,0,angleDeg,t1x,t1y);
  rotatePoint(0,0,angleDeg,t2x,t2y);
  display.fillTriangle(
    (int16_t)(cx+t0x),(int16_t)(cy+t0y),
    (int16_t)(cx+t1x),(int16_t)(cy+t1y),
    (int16_t)(cx+t2x),(int16_t)(cy+t2y), GxEPD_BLACK);
}

void drawDiamondAt(float cx, float cy, float r, float angleDeg)
{
  float t0x=0,       t0y=-r;
  float t1x=r*0.65f, t1y=0;
  float t2x=0,       t2y=r;
  float t3x=-r*0.65f,t3y=0;
  rotatePoint(0,0,angleDeg,t0x,t0y); rotatePoint(0,0,angleDeg,t1x,t1y);
  rotatePoint(0,0,angleDeg,t2x,t2y); rotatePoint(0,0,angleDeg,t3x,t3y);
  display.fillTriangle(
    (int16_t)(cx+t0x),(int16_t)(cy+t0y),
    (int16_t)(cx+t1x),(int16_t)(cy+t1y),
    (int16_t)(cx+t2x),(int16_t)(cy+t2y), GxEPD_BLACK);
  display.fillTriangle(
    (int16_t)(cx+t0x),(int16_t)(cy+t0y),
    (int16_t)(cx+t2x),(int16_t)(cy+t2y),
    (int16_t)(cx+t3x),(int16_t)(cy+t3y), GxEPD_BLACK);
}

void drawClubAt(float cx, float cy, float r, float angleDeg)
{
  float tops[3][2] = {{0,-r*0.42f},{-r*0.48f,r*0.18f},{r*0.48f,r*0.18f}};
  float rads[3]    = {r*0.44f, r*0.38f, r*0.38f};
  for(int i=0;i<3;i++){
    float bx=tops[i][0], by=tops[i][1];
    rotatePoint(0,0,angleDeg,bx,by);
    display.fillCircle((int16_t)(cx+bx),(int16_t)(cy+by),(int16_t)rads[i],GxEPD_BLACK);
  }
  float s0x=-r*0.18f, s0y=r*0.22f;
  float s1x= r*0.18f, s1y=r*0.22f;
  float s2x= r*0.55f, s2y=r;
  float s3x=-r*0.55f, s3y=r;
  rotatePoint(0,0,angleDeg,s0x,s0y); rotatePoint(0,0,angleDeg,s1x,s1y);
  rotatePoint(0,0,angleDeg,s2x,s2y); rotatePoint(0,0,angleDeg,s3x,s3y);
  display.fillTriangle(
    (int16_t)(cx+s0x),(int16_t)(cy+s0y),
    (int16_t)(cx+s1x),(int16_t)(cy+s1y),
    (int16_t)(cx+s2x),(int16_t)(cy+s2y), GxEPD_BLACK);
  display.fillTriangle(
    (int16_t)(cx+s0x),(int16_t)(cy+s0y),
    (int16_t)(cx+s2x),(int16_t)(cy+s2y),
    (int16_t)(cx+s3x),(int16_t)(cy+s3y), GxEPD_BLACK);
}

void drawSuit(float cx, float cy, float r, uint8_t suit, float angleDeg)
{
  switch(suit){
    case 0: drawSpadeAt(cx,cy,r,angleDeg);   break;
    case 1: drawHeartAt(cx,cy,r,angleDeg);   break;
    case 2: drawDiamondAt(cx,cy,r,angleDeg); break;
    case 3: drawClubAt(cx,cy,r,angleDeg);    break;
  }
}

// ═══════════════════════════════════════════════════
//  Hour → card rank
// ═══════════════════════════════════════════════════
void hourToCard(uint8_t hour, char *rankStr, uint8_t &pips)
{
  uint8_t h = hour % 12;
  if(h == 0) h = 12;
  pips = (h <= 10) ? h : 0;
  switch(h){
    case 1:  strcpy(rankStr,"A"); break;
    case 11: strcpy(rankStr,"J"); break;
    case 12: strcpy(rankStr,"Q"); break;
    default: sprintf(rankStr,"%d",h); break;
  }
}

// ═══════════════════════════════════════════════════
//  Shadow — solid black rotated rect, offset + expanded
//  Draw this BEFORE drawing the card that casts the shadow.
//  The shadow lands on whatever card was drawn behind it.
// ═══════════════════════════════════════════════════
void drawCardShadow(float pivX, float pivY, float angleDeg)
{
  const float sp = (float)SHADOW_SPREAD;
  const float CW = (float)CARD_W;
  const float CH = (float)CARD_H;

  // Compute the four unrotated corners of the shadow rect
  float corners[4][2] = {
    { pivX - CW*0.5f - sp + SHADOW_DX,  pivY - CH - sp + SHADOW_DY },
    { pivX + CW*0.5f + sp + SHADOW_DX,  pivY - CH - sp + SHADOW_DY },
    { pivX + CW*0.5f + sp + SHADOW_DX,  pivY        + sp + SHADOW_DY },
    { pivX - CW*0.5f - sp + SHADOW_DX,  pivY        + sp + SHADOW_DY },
  };
  for(int i=0;i<4;i++)
    rotatePoint(pivX, pivY, angleDeg, corners[i][0], corners[i][1]);

  // Axis-aligned bounding box of the rotated quad
  float minX = corners[0][0], maxX = corners[0][0];
  float minY = corners[0][1], maxY = corners[0][1];
  for(int i=1;i<4;i++){
    minX = min(minX, corners[i][0]); maxX = max(maxX, corners[i][0]);
    minY = min(minY, corners[i][1]); maxY = max(maxY, corners[i][1]);
  }

  // Precompute edge normals for inside-quad test
  // For each edge (a→b), a point p is inside if cross(b-a, p-a) >= 0
  auto inside = [&](float px, float py) -> bool {
    for(int i=0;i<4;i++){
      float ax = corners[i][0],       ay = corners[i][1];
      float bx = corners[(i+1)%4][0], by = corners[(i+1)%4][1];
      float cross = (bx-ax)*(py-ay) - (by-ay)*(px-ax);
      if(cross < 0) return false;
    }
    return true;
  };

  // Walk every other pixel in a checkerboard pattern
  for(int y = (int)minY; y <= (int)maxY; y++){
    for(int x = (int)minX; x <= (int)maxX; x++){
      if((x + y) & 1) continue;          // checkerboard skip
      if(x < 0 || x >= 200) continue;    // clip to display width
      if(y < 0 || y >= 200) continue;    // clip to display height
      if(inside((float)x + 0.5f, (float)y + 0.5f))
        display.drawPixel(x, y, GxEPD_BLACK);
    }
  }
}

// ═══════════════════════════════════════════════════
//  Draw one card at angle, pivoting around (pivX,pivY)
// ═══════════════════════════════════════════════════
void drawCardAtAngle(float pivX, float pivY, float angleDeg,
                     const char *rank, uint8_t rankIndex,
                     uint8_t suit, bool showPips)
{
  const float CW = CARD_W;
  const float CH = CARD_H;
  const float CX = pivX - CW * 0.5f;
  const float CY = pivY - CH;
  const float R  = (float)CARD_R;

  auto rot = [&](float &x, float &y){
    rotatePoint(pivX, pivY, angleDeg, x, y);
  };

  auto quad = [&](float x0,float y0,float x1,float y1,
                  float x2,float y2,float x3,float y3,uint16_t c){
    rot(x0,y0); rot(x1,y1); rot(x2,y2); rot(x3,y3);
    display.fillTriangle(x0,y0,x1,y1,x2,y2,c);
    display.fillTriangle(x0,y0,x2,y2,x3,y3,c);
  };

 const float BORDER_W = 2.0f;

  auto thickEdge = [&](float x0, float y0, float x1, float y1, float thickness){
    float dx = x1-x0, dy = y1-y0;
    float len = sqrtf(dx*dx + dy*dy);
    if(len < 0.001f) return;
    float nx = -dy/len * thickness*0.5f;
    float ny =  dx/len * thickness*0.5f;
    float ax=x0+nx, ay=y0+ny, bx=x1+nx, by=y1+ny;
    float cx2=x1-nx, cy2=y1-ny, dx2=x0-nx, dy2=y0-ny;
    display.fillTriangle((int16_t)ax,(int16_t)ay,(int16_t)bx,(int16_t)by,(int16_t)cx2,(int16_t)cy2, GxEPD_BLACK);
    display.fillTriangle((int16_t)ax,(int16_t)ay,(int16_t)cx2,(int16_t)cy2,(int16_t)dx2,(int16_t)dy2, GxEPD_BLACK);
  };
float cxs[4] = {CX+R,    CX+CW-R, CX+CW-R, CX+R   };
  float cys[4] = {CY+R,    CY+R,    CY+CH-R, CY+CH-R};

  // 1 — card white fill
  quad(CX+R, CY,      CX+CW-R, CY,      CX+CW-R, CY+CH,    CX+R,    CY+CH,    GxEPD_WHITE);
  quad(CX,   CY+R,    CX+R,    CY+R,    CX+R,    CY+CH-R,  CX,      CY+CH-R,  GxEPD_WHITE);
  quad(CX+CW-R,CY+R,  CX+CW,   CY+R,   CX+CW,   CY+CH-R,  CX+CW-R, CY+CH-R, GxEPD_WHITE);
  
  // Rotate and store corner positions once
  float rcxs[4], rcys[4];
  for(int i=0;i<4;i++){
    rcxs[i]=cxs[i]; rcys[i]=cys[i];
    rot(rcxs[i], rcys[i]);
    display.fillCircle((int16_t)rcxs[i],(int16_t)rcys[i],(int16_t)R, GxEPD_WHITE);
  }

  // 2 — thick border edges
  float bpts[4][2] = {{CX,CY},{CX+CW,CY},{CX+CW,CY+CH},{CX,CY+CH}};
  for(int i=0;i<4;i++){
    float rx0=bpts[i][0],      ry0=bpts[i][1];
    float rx1=bpts[(i+1)%4][0],ry1=bpts[(i+1)%4][1];
    rot(rx0,ry0); rot(rx1,ry1);
    thickEdge(rx0,ry0,rx1,ry1, BORDER_W);
  }

// 3 — corner rings then mask interior bleed with two rotated white quads per corner
  for(int i=0;i<4;i++){
    for(int t=0;t<(int)BORDER_W;t++){
      display.drawCircle((int16_t)rcxs[i], (int16_t)rcys[i], (int16_t)(R-t), GxEPD_BLACK);
    }

    float cx = cxs[i], cy = cys[i]; // use UNROTATED positions for quad()

if(i==0){ // top-left
  quad(cx+BORDER_W, cy-R+BORDER_W,  cx+R, cy-R+BORDER_W,  cx+R, cy+R,  cx+BORDER_W, cy+R,  GxEPD_WHITE);
  quad(cx-R+BORDER_W, cy+BORDER_W,  cx+R, cy+BORDER_W,    cx+R, cy+R,  cx-R+BORDER_W, cy+R, GxEPD_WHITE);
}
if(i==1){ // top-right
  quad(cx-R, cy-R+BORDER_W,  cx-BORDER_W, cy-R+BORDER_W,  cx-BORDER_W, cy+R,  cx-R, cy+R,  GxEPD_WHITE);
  quad(cx-R, cy+BORDER_W,    cx+R-BORDER_W, cy+BORDER_W,  cx+R-BORDER_W, cy+R,  cx-R, cy+R, GxEPD_WHITE);
}
if(i==2){ // bottom-right
  quad(cx-R, cy-R,  cx-BORDER_W, cy-R,  cx-BORDER_W, cy+R-BORDER_W,  cx-R, cy+R-BORDER_W,  GxEPD_WHITE);
  quad(cx-R, cy-R,  cx+R-BORDER_W, cy-R,  cx+R-BORDER_W, cy-BORDER_W,  cx-R, cy-BORDER_W,  GxEPD_WHITE);
}
if(i==3){ // bottom-left
  quad(cx+BORDER_W, cy-R,  cx+R, cy-R,  cx+R, cy+R-BORDER_W,  cx+BORDER_W, cy+R-BORDER_W,  GxEPD_WHITE);
  quad(cx-R, cy-R,  cx+R, cy-R,  cx+R, cy-BORDER_W,  cx-R, cy-BORDER_W,  GxEPD_WHITE);
}
  }

  // Corner rank bitmap (top-left)
  float tx = CX + 16.0f;
  float ty = CY + 16.0f;
  rot(tx, ty);
  drawRankBitmap(tx, ty, rank, angleDeg, GxEPD_BLACK);

  // Corner suit pip (top-left, below rank)
  uint8_t bh = 22;
  for(uint8_t i=0;i<NUM_CARD_BITMAPS;i++){
    CardBitmap cb; memcpy_P(&cb,&CARD_BITMAPS[i],sizeof(CardBitmap));
    if(strcmp(cb.label,rank)==0){ bh=cb.h; break; }
  }
  float sx = CX + 12.0f + 4.0f;
  float sy = CY + 8.0f + bh + 4.0f + CORNER_PIP_RADIUS;
  rot(sx, sy);
  drawSuit(sx, sy, CORNER_PIP_RADIUS, suit, angleDeg);

  // Corner rank bitmap (bottom-right, 180°)
  float tx2 = CX + CW - 12.0f;
  float ty2 = CY + CH - 12.0f;
  rot(tx2, ty2);
  drawRankBitmap(tx2, ty2, rank, angleDeg + 180.0f, GxEPD_BLACK);

  // Corner suit pip (bottom-right)
  float sx2 = CX + CW - 6.0f - 4.0f;
  float sy2 = CY + CH - 6.0f - bh - 4.0f - CORNER_PIP_RADIUS;
  rot(sx2, sy2);
  drawSuit(sx2, sy2, CORNER_PIP_RADIUS, suit, angleDeg + 180.0f);

  if(!showPips) return;

  // ── Pips ──────────────────────────────────────────
  const float midX = CX + CW * 0.5f;

  if(rankIndex == 0){
    // Ace — single large pip at card centre
    float px = midX;
    float py = CY + CARD_H * 0.5f;
    rot(px, py);
    drawSuit(px, py, ACE_RADIUS, suit, angleDeg);
    return;
  }

  if(rankIndex >= 10) return;   // J/Q/K — no pip body (face cards)

  uint8_t count; memcpy_P(&count, &PIP_COUNT[rankIndex], 1);

  for(uint8_t i=0;i<count;i++){
    PipDef pd; memcpy_P(&pd, &PIP_TABLE[rankIndex][i], sizeof(PipDef));
    if(pd.col == 99) break;

    float px = midX + pd.col * PIP_COL_OFFSET;
    float py = CY   + pipY(pd.yKey);
    rot(px, py);

    float pipAngle = angleDeg + (pd.flip ? 180.0f : 0.0f);
    drawSuit(px, py, PIP_RADIUS, suit, pipAngle);
  }
}

// ═══════════════════════════════════════════════════
//  Rank bitmap renderer
// ═══════════════════════════════════════════════════
void drawRankBitmap(float cx, float cy, const char* rank,
                    float angleDeg, uint16_t color)
{
  for(uint8_t i=0;i<NUM_CARD_BITMAPS;i++){
    CardBitmap cb;
    memcpy_P(&cb, &CARD_BITMAPS[i], sizeof(CardBitmap));
    if(strcmp(cb.label, rank) != 0) continue;

    const uint8_t w = cb.w, h = cb.h;
    const uint16_t rowBytes = (w + 7) / 8;

    for(uint8_t row=0;row<h;row++){
      for(uint8_t col=0;col<w;col++){
        uint8_t b;
        memcpy_P(&b, cb.bmp + row*rowBytes + col/8, 1);
        if(!(b & (0x80 >> (col & 7)))) continue;

        float px = col - w * 0.5f;
        float py = row - h * 0.5f;
        rotatePoint(0.0f, 0.0f, angleDeg, px, py);
        display.drawPixel((int16_t)(cx+px), (int16_t)(cy+py), color);
      }
    }
    return;
  }
}

// ═══════════════════════════════════════════════════
//  Suit randomiser
// ═══════════════════════════════════════════════════
void randomiseSuit(){ currentSuit = random(0,4); }

// ═══════════════════════════════════════════════════
//  Battery
// ═══════════════════════════════════════════════════
uint8_t getBatteryPercent()
{
  analogReadResolution(12);
  long bat=0;
  for(uint8_t i=0;i<20;i++) bat+=analogRead(Bat_ADC);
  bat/=20;
  float volt=bat*3.3f/4096.0f*2.0f;
  int pct=(int)((volt-3.3f)/(4.2f-3.3f)*100.0f);
  return (uint8_t)constrain(pct,0,100);
}

void drawBatteryIcon(uint16_t x, uint16_t y)
{
  uint8_t pct=getBatteryPercent();
  uint8_t fill=(uint8_t)(24.0f*pct/100.0f);
  display.drawRect(x,y,28,12,GxEPD_WHITE);
  display.drawRect(x+28,y+3,3,6,GxEPD_WHITE);
  if(fill>0) display.fillRect(x+2,y+2,fill,8,GxEPD_WHITE);
  display.setFont(NULL);
  display.setTextColor(GxEPD_WHITE);
  display.setTextSize(1);
  display.setCursor(x-26,y+3);
  display.print(pct); display.print('%');
}

// ═══════════════════════════════════════════════════
//  Watch face
// ═══════════════════════════════════════════════════
void drawWatchFace()
{
  int hrs  = rtc.getHour();
  int mins = rtc.getMinute();

  if(hrs != lastHour){ lastHour=hrs; randomiseSuit(); }

  display.fillScreen(GxEPD_BLACK);

  // Compute the 4 ranks to show (current hour + 3 previous)
  char rankA[4]; uint8_t pipsA; hourToCard(hrs, rankA, pipsA);
  char rankB[4]; uint8_t pipsB;
  char rankC[4]; uint8_t pipsC;
  char rankD[4]; uint8_t pipsD;

  uint8_t hB = (hrs==0)?11:(hrs-1)%12; if(hB==0)hB=12;
  uint8_t hC = (hrs<=1)?(hrs==0?10:11):(hrs-2)%12; if(hC==0)hC=12;
  uint8_t hD = (hrs<=2)?(hrs==0?9:(hrs==1?10:11)):(hrs-3)%12; if(hD==0)hD=12;

  hourToCard(hB, rankB, pipsB);
  hourToCard(hC, rankC, pipsC);
  hourToCard(hD, rankD, pipsD);

  // ── Draw order: back-to-front ──────────────────────
  // D is back-most — no shadow behind it
  drawCardAtAngle(PIVOT_X+PIVOT_STEP*3, PIVOT_Y, FRONT_ANGLE+OFFSET_D,
                  rankD, (pipsD==0?10:pipsD-1), currentSuit, false);

  // C's shadow lands on D, then draw C
  drawCardShadow  (PIVOT_X+PIVOT_STEP*2, PIVOT_Y, FRONT_ANGLE+OFFSET_C);
  drawCardAtAngle (PIVOT_X+PIVOT_STEP*2, PIVOT_Y, FRONT_ANGLE+OFFSET_C,
                  rankC, (pipsC==0?10:pipsC-1), currentSuit, false);

  // B's shadow lands on C, then draw B
  drawCardShadow  (PIVOT_X+PIVOT_STEP*1, PIVOT_Y, FRONT_ANGLE+OFFSET_B);
  drawCardAtAngle (PIVOT_X+PIVOT_STEP*1, PIVOT_Y, FRONT_ANGLE+OFFSET_B,
                  rankB, (pipsB==0?10:pipsB-1), currentSuit, false);

  // A's shadow lands on B, then draw A (front / current-hour card)
  drawCardShadow  (PIVOT_X, PIVOT_Y, FRONT_ANGLE);
  drawCardAtAngle (PIVOT_X, PIVOT_Y, FRONT_ANGLE,
                  rankA, (pipsA==0?10:pipsA-1), currentSuit, true);

  // Minutes display
  display.setFont(&FreeSerifItalic12pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(166, 28);
  display.printf("%02d", mins);
  display.setFont(NULL);
}

// ═══════════════════════════════════════════════════
//  Menu / Weather / Settings
// ═══════════════════════════════════════════════════
void drawMenu()
{
  display.fillScreen(GxEPD_WHITE);
  display.fillRect(0,0,200,26,GxEPD_BLACK);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(6,18);
  display.print("MENU");
  drawBatteryIcon(118,7);
  const char *items[]={"Weather","Settings","Exit"};
  for(uint8_t i=0;i<3;i++){
    uint16_t yy=38+i*38;
    if(i==menuIndex){
      display.fillRect(0,yy-2,200,32,GxEPD_BLACK);
      display.setTextColor(GxEPD_WHITE);
      display.fillTriangle(4,yy+4,4,yy+20,14,yy+12,GxEPD_WHITE);
    } else {
      display.setTextColor(GxEPD_BLACK);
    }
    display.setFont(&FreeSans9pt7b);
    display.setCursor(20,yy+18);
    display.print(items[i]);
  }
  display.setFont(NULL);
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(1);
  display.setCursor(4,188);
  display.setFont(NULL);
}


void showWeatherScreen()
{
  display.fillScreen(GxEPD_WHITE);
  display.fillRect(0,0,200,26,GxEPD_BLACK);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(6,18); display.print("Weather");
  drawBatteryIcon(118,7);
  const char *name        = doc["results"][0]["location"]["name"];
  const char *weather     = doc["results"][0]["now"]["text"];
  const char *temperature = doc["results"][0]["now"]["temperature"];
  display.setFont(&FreeSans9pt7b); display.setTextColor(GxEPD_BLACK);
  display.setCursor(6,46);  display.print(name?name:"---");
  display.setFont(&FreeSerifItalic18pt7b);
  display.setCursor(10,95); display.print(temperature?temperature:"--");
  display.setFont(&FreeSans9pt7b); display.print(" C");
  display.setCursor(6,135); display.print(weather?weather:"---");
  display.setFont(NULL); display.setTextSize(1);
  display.setCursor(4,190); display.print("DblClick to go back");
  display.update();
  while(true){ button.check(); vTaskDelay(10); if(!inMenu) return; }
}

void showSettingsScreen()
{
  display.fillScreen(GxEPD_WHITE);
  display.fillRect(0,0,200,26,GxEPD_BLACK);
  display.setFont(&FreeSans9pt7b); display.setTextColor(GxEPD_WHITE);
  display.setCursor(6,18); display.print("Settings");
  drawBatteryIcon(118,7);
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(6,50);  display.print("WiFi Sync:");
  display.setCursor(6,72);  display.print(WIFIUpdataFlag?"[ON ]":"[OFF]");
  display.setCursor(6,102); display.print("GPS:");
  display.setCursor(6,124); display.print(GPSUpdataFlag?"[ON ]":"[OFF]");
  display.setFont(NULL); display.setTextSize(1);
  display.setCursor(4,162); display.print("Click=toggle WiFi");
  display.setCursor(4,174); display.print("DblClick=toggle GPS");
  display.setCursor(4,186); display.print("LongPress=back");
  display.update();
  bool inSettings=true;
  while(inSettings){ button.check(); vTaskDelay(10); if(!inMenu) inSettings=false; }
}

void menuAction(uint8_t idx)
{
  switch(idx){
    case 0: showWeatherScreen(); break;
    case 1: showSettingsScreen(); break;
    case 2: inMenu=false; DisplayFullUpdata=true; break;
  }
}

// ═══════════════════════════════════════════════════
//  Button handler
// ═══════════════════════════════════════════════════
void handleEvent(ace_button::AceButton*, uint8_t eventType, uint8_t)
{
  switch(eventType){
    case AceButton::kEventClicked:
      if(!inMenu){
        int h = rtc.getHour();
        int m = rtc.getMinute();
        int s = rtc.getSecond();
        h = (h + 1) % 24;
        rtc.setTime(s, m, h, rtc.getDay(), rtc.getMonth(), rtc.getYear());
        DisplayFullUpdata = true;
      } else {
        menuIndex = (menuIndex + 1) % 3;
        drawMenu();
        display.update();
      }
      break;

    case AceButton::kEventLongPressed:
      if(inMenu){
        menuAction(menuIndex);
      } else {
        inMenu = true;
        menuIndex = 0;
        drawMenu();
        display.update();
      }
      break;
  }
}

// ═══════════════════════════════════════════════════
//  WiFi / HTTP
// ═══════════════════════════════════════════════════
void setUpHttpClient()
{
  req  = (String)host+"/v3/weather/now.json?key=";
  req += apiKey; req += "&location="; req += city;
  req += "&language=en&unit=c";
  if(http_client.begin(req)) Serial.println("HTTPclient setUp done!");
}

// ── Get time via HTTP since UDP 123 is blocked ─────
bool getTimeViaHTTP()
{
  HTTPClient h;
  h.begin("http://baidu.com");
  h.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  
  const char* headerKeys[] = { "Date" };   // ← named, persistent
  h.collectHeaders(headerKeys, 1);
  
  int code = h.GET();
  Serial.printf("Time HTTP code: %d\n", code);
  String date = h.header("Date");
  Serial.printf("Date header: '%s'\n", date);  // ← add this to confirm
  h.end();

  if (date.length() < 25) {
    Serial.println("Date header missing");
    return false;
  }
  // Parse: "Www, DD Mon YYYY HH:MM:SS GMT"
  struct tm t = {};
  const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4]; int d, y, hh, mm, ss;
  sscanf(date.c_str(), "%*s %d %3s %d %d:%d:%d",
         &d, mon, &y, &hh, &mm, &ss);
  t.tm_mday  = d;
  t.tm_mon   = (strstr(months, mon) - months) / 3;
  t.tm_year  = y - 1900;
  t.tm_hour  = hh;
  t.tm_min   = mm;
  t.tm_sec   = ss;
  t.tm_isdst = 0;
  time_t epoch = mktime(&t) + gmtOffset_sec;
  struct tm *lt = gmtime(&epoch);
  if (!lt) return false;
  memcpy(&timeinfo, lt, sizeof(struct tm));
  Serial.printf("HTTP Date: %s\n", date.c_str());
  return true;
}
bool authenticateCaptivePortal()
{
  // Step 1 — GET wifi.kings.edu to grab the real UniFi redirect URL
  HTTPClient h1;
  h1.begin("http://wifi.kings.edu/");
  h1.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  const char* keys[] = { "Location" };
  h1.collectHeaders(keys, 1);
  h1.GET();
  String portalURL = h1.header("Location");
  h1.end();

  Serial.println("UniFi URL: " + portalURL);
  if(portalURL.length() == 0) return false;

  // Step 2 — POST credentials to the UniFi portal URL (HTTPS)
  WiFiClientSecure sc;
  sc.setInsecure();
  HTTPClient h2;
  h2.begin(sc, portalURL);
  h2.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  h2.addHeader("Content-Type", "application/x-www-form-urlencoded");
  h2.addHeader("User-Agent",   "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
  h2.addHeader("Referer",      portalURL);

  String body = String("username=") + portalUser + "&password=" + portalPass;
  int code = h2.POST(body);

  Serial.printf("UniFi POST code: %d\n", code);
  Serial.println(h2.getString().substring(0, 200)); // first 200 chars only
  h2.end();

  // UniFi returns 200 with success page, or redirects on success
  return (code == 200 || code == 302);
}
void WIFIUpdataTime(bool EN)
{
  if(!EN) return;

  WiFi.begin(ssid, password);
  uint8_t wifiRetry = 0;
  while(WiFi.status() != WL_CONNECTED && wifiRetry < 40){
    Serial.print('.'); delay(500); wifiRetry++;
  }
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("\nWiFi failed");
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
    WIFIUpdataFlag = false;
    return;
  }
  Serial.print("\nWiFi OK: "); Serial.println(WiFi.localIP());

if(!authenticateCaptivePortal()){
  Serial.println("Portal auth failed");
} else {
  Serial.println("Portal auth OK");
}
delay(1000);
  configTime(gmtOffset_sec, daylightOffset_sec,
             "ntp1.aliyun.com", "ntp2.aliyun.com");
  delay(5000);

  bool timeOK = getLocalTime(&timeinfo);
if(!timeOK){
    Serial.println("SNTP blocked — trying HTTP time...");
    timeOK = getTimeViaHTTP();
  }

  if(timeOK){
    Serial.printf("Time OK: %02d:%02d:%02d\n",
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    rtc.setTime(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour,
                timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900);
  } else {
    Serial.println("Time sync failed — RTC unchanged");
  }

WiFiClientSecure client;
client.setInsecure();               // skip cert validation
HTTPClient http_client2;
String url = "https://api.seniverse.com/v3/weather/now.json?key=";
url += apiKey; url += "&location="; url += city;
url += "&language=en&unit=c";
http_client2.begin(client, url); 
http_client2.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);   // note: pass the secure client
int http_code = http_client2.GET();
if(http_code == HTTP_CODE_OK){
  rsp = http_client2.getString();
  Serial.println("RAW: " + rsp);      // ← add this
  DeserializationError err = deserializeJson(doc, rsp);
    if(err) Serial.printf("JSON err: %s\n", err.f_str());
    else    Serial.println("Weather OK");
  } else {
    Serial.println(http_client2.getString());
  }
  http_client2.end();

  WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  WIFIUpdataFlag = false;
}
// ═══════════════════════════════════════════════════
//  setup()
// ═══════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200); delay(100);
  WiFi.mode(WIFI_STA); delay(100);
  Serial.println(WiFi.macAddress());
  SPI.begin(SPI_SCK,-1,SPI_DIN,EPD_CS);
  pinMode(PIN_MOTOR,OUTPUT); pinMode(PWR_EN,OUTPUT);
  digitalWrite(PWR_EN,HIGH);
  pinMode(PIN_KEY,INPUT_PULLUP);
  digitalWrite(PIN_MOTOR,HIGH); delay(100);
  digitalWrite(PIN_MOTOR,LOW);  delay(50);
  digitalWrite(PIN_MOTOR,HIGH); delay(100);
  digitalWrite(PIN_MOTOR,LOW);
  gpsPort.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  display.init();
  display.setRotation(0);
  display.fillScreen(GxEPD_BLACK);
  display.update();
  randomSeed(analogRead(0));
  randomiseSuit();
  WIFIUpdataTime(true);
  ButtonConfig *buttonConfig=button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setDebounceDelay(20);
  buttonConfig->setClickDelay(200);
  xTaskCreate(Task1code,"Task1",10000,NULL,1,NULL);
  xTaskCreate(Task2code,"Task2",10000,NULL,2,NULL);
}

// ═══════════════════════════════════════════════════
//  loop()
// ═══════════════════════════════════════════════════
void loop()
{
  if(SleepFlag){
    SleepFlag=false;
    display.setFont(&FreeSans9pt7b);
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(120,188); display.print("Sleep");
    display.setFont(NULL);
    display.updateWindow(0,0,GxEPD_WIDTH,GxEPD_HEIGHT,false);
    display.powerDown();
    digitalWrite(GPS_RES,LOW);
    while(digitalRead(PIN_KEY)==LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,LOW);
    esp_deep_sleep_start();
  }
}

// ═══════════════════════════════════════════════════
//  Task 1 — button polling
// ═══════════════════════════════════════════════════
void Task1code(void *parameter)
{
  while(1){ button.check(); vTaskDelay(10); }
  vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════
//  Task 2 — display refresh
// ═══════════════════════════════════════════════════
void Task2code(void *parameter)
{
  while(1){
    if(!inMenu){
      getLocalTime(&timeinfo,10);
      WIFIUpdataTime(WIFIUpdataFlag);
      drawWatchFace();
      if(DisplayFullUpdata){
        display.update();
        DisplayFullUpdata=false;
      } else {
        display.updateWindow(0,0,GxEPD_WIDTH,GxEPD_HEIGHT,false);
      }
    }
    vTaskDelay(500);
  }
  vTaskDelete(NULL);
}
