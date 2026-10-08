#pragma once
#include <M5Unified.h>
#include "PanelConfig.h"
#include "Telemetry.h"

// Display-only: no GLIICAN, TWAI, or motor command path.
namespace panel {
constexpr uint16_t BG=0x0842, CARD=0x10A4, GRID=0x2947, WHITE=0xEF7D, MUTED=0x94B2, RED=0xF986;
constexpr uint16_t COLORS[3]={0x36DF,0x77EC,0xFD67};
const char* NAMES[3]={"Position","Velocity","Torque"};
M5Canvas chart(&M5.Display), banner(&M5.Display);
SemaphoreHandle_t mutex;
telemetry::Receiver receiver;
uint32_t badReads=0;
float measured[3][config::SAMPLES], commanded[3][config::SAMPLES];
int head=0,count=0;
uint32_t lastPlot=0,lastPaint=0,plottedSession=0;
bool ready=false,haveSession=false;

void pollWorker(void*) {
  TickType_t wake=xTaskGetTickCount();
  for (;;) {
    uint8_t packet[telemetry::SIZE];
    bool ok=M5.Ex_I2C.start(telemetry::ADDRESS,true,telemetry::CLOCK_HZ);
    if (ok) {
      ok=M5.Ex_I2C.read(packet,sizeof(packet),true);
      bool ended=M5.Ex_I2C.stop(); ok=ok && ended;
    }
    xSemaphoreTake(mutex,portMAX_DELAY);
    if (ok) {
      telemetry::Sample sample;
      if (!telemetry::unpack(packet,sizeof(packet),sample)) ++badReads;
      else receiver.accept(packet,sizeof(packet),millis());
    } else ++badReads;
    xSemaphoreGive(mutex);
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(config::POLL_MS));
  }
}
void text(M5Canvas& c,int x,int y,const char* value,uint16_t color=WHITE,int size=2) {
  c.setTextSize(size);c.setTextColor(color);c.setCursor(x,y);c.print(value);
}
int plotY(float value,config::Range r) {
  float v=fmaxf(r.lo,fminf(r.hi,value));
  return 158-int(104*(v-r.lo)/(r.hi-r.lo));
}
void drawTrace(int axis,bool command) {
  bool previous=false;int px=0,py=0;
  for(int i=0;i<count;++i) {
    int index=(head-count+i+config::SAMPLES)%config::SAMPLES;
    float v=command?commanded[axis][index]:measured[axis][index];
    int x=92+(config::SAMPLES-count+i)*1120/(config::SAMPLES-1);
    if(!isfinite(v)){previous=false;continue;}
    int y=plotY(v,config::PLOTS[axis]);
    if(previous && (!command || (i/3)%2==0)) chart.drawLine(px,py,x,y,command?WHITE:COLORS[axis]);
    px=x;py=y;previous=true;
  }
}
void drawChart(int axis,const telemetry::Receiver& r,uint32_t now) {
  chart.fillSprite(CARD);text(chart,18,13,NAMES[axis],COLORS[axis],3);
  bool fresh=r.feedbackFresh(now,config::LINK_TIMEOUT_MS,config::FEEDBACK_TIMEOUT_MS);
  char label[70];
  if(fresh) snprintf(label,sizeof(label),"%.3f %s",r.latest.measured[axis],config::UNITS[axis]);
  else snprintf(label,sizeof(label),"-- %s",config::UNITS[axis]);
  text(chart,240,13,label,WHITE,3);
  if(r.live(now,config::LINK_TIMEOUT_MS) && (r.latest.flags & telemetry::Active)) {
    snprintf(label,sizeof(label),"%s %.3f",axis==2?"Feedforward":"Command",r.latest.command[axis]);
    text(chart,660,18,label,MUTED);
  }
  for(int i=0;i<=4;++i)chart.drawFastHLine(92,54+26*i,1121,GRID);
  for(int i=0;i<=5;++i)chart.drawFastVLine(92+i*224,54,105,GRID);
  snprintf(label,sizeof(label),"%.2f",config::PLOTS[axis].hi);text(chart,8,54,label,MUTED,1);
  snprintf(label,sizeof(label),"%.2f",config::PLOTS[axis].lo);text(chart,8,150,label,MUTED,1);
  snprintf(label,sizeof(label),"-%.1f s",(config::SAMPLES-1)*config::PLOT_MS/1000.0f);
  text(chart,92,177,label,MUTED,1);text(chart,1190,177,"now",MUTED,1);
  text(chart,390,177,"Color: measured   Dashed white: command",MUTED,1);
  drawTrace(axis,false);drawTrace(axis,true);
  if(fresh && (r.latest.measured[axis]<config::PLOTS[axis].lo || r.latest.measured[axis]>config::PLOTS[axis].hi))
    text(chart,1070,18,"OFF SCALE",RED);
  chart.pushSprite(20,74+axis*212);
}
void begin() {
  auto cfg=M5.config();cfg.external_imu=false;cfg.external_rtc=false;M5.begin(cfg);
  if(M5.Display.width()<M5.Display.height())M5.Display.setRotation(M5.Display.getRotation()^1);
  if(config::FLIP_180)M5.Display.setRotation((M5.Display.getRotation()+2)%4);
  M5.Display.fillScreen(BG);M5.Display.setTextColor(WHITE);M5.Display.setTextSize(2);
  if(M5.Display.width()!=1280 || M5.Display.height()!=720){M5.Display.println("Requires Tab5 landscape 1280 x 720");return;}
  for(auto r:config::PLOTS) if(!isfinite(r.lo)||!isfinite(r.hi)||r.lo>=r.hi){M5.Display.println("Invalid plot limits");return;}
  static_assert(config::SAMPLES>=2 && config::PLOT_MS>0 && config::POLL_MS>0,"Invalid plot timing");
  mutex=xSemaphoreCreateMutex();chart.setColorDepth(16);banner.setColorDepth(16);
  if(!mutex || !chart.createSprite(1240,200) || !banner.createSprite(1280,68)){
    M5.Display.println("Buffer allocation failed; enable PSRAM");return;
  }
  // Use external bus only; internal I2C stays with touch and power management.
  if(M5.Ex_I2C.getPort()==M5.In_I2C.getPort()) {M5.Display.println("External/internal I2C conflict");return;}
  auto port=M5.Ex_I2C.getPort();
  if(!M5.Ex_I2C.begin(port,config::I2C_SDA,config::I2C_SCL)){M5.Display.println("I2C init failed");return;}
  if(xTaskCreate(pollWorker,"graphI2C",4096,nullptr,1,nullptr)!=pdPASS){M5.Display.println("I2C task failed");return;}
  lastPlot=millis();ready=true;
}
void update() {
  M5.update();if(!ready){delay(10);return;}
  xSemaphoreTake(mutex,portMAX_DELAY);
  telemetry::Receiver r=receiver;uint32_t errors=badReads;
  xSemaphoreGive(mutex);
  uint32_t now=millis();
  if(r.seen && (!haveSession || plottedSession!=r.latest.session)) {
    head=count=0;plottedSession=r.latest.session;haveSession=true;lastPlot=now;
  }
  if(now-lastPlot>=config::PLOT_MS){
    unsigned ticks=min(unsigned((now-lastPlot)/config::PLOT_MS),unsigned(config::SAMPLES));
    for(unsigned n=0;n<ticks;++n){
      bool current=n==ticks-1;
      bool fresh=current && r.feedbackFresh(now,config::LINK_TIMEOUT_MS,config::FEEDBACK_TIMEOUT_MS);
      bool active=current && r.live(now,config::LINK_TIMEOUT_MS) && (r.latest.flags & telemetry::Active);
      for(int i=0;i<3;++i){
        measured[i][head]=fresh?r.latest.measured[i]:NAN;
        commanded[i][head]=active?r.latest.command[i]:NAN;
      }
      head=(head+1)%config::SAMPLES;count=min(count+1,config::SAMPLES);
    }
    lastPlot+=((now-lastPlot)/config::PLOT_MS)*config::PLOT_MS;
  }
  if(now-lastPaint>=config::DRAW_MS){
    lastPaint=now;for(int i=0;i<3;++i)drawChart(i,r,now);
    banner.fillSprite(BG);text(banner,20,8,"MOTOR TELEMETRY",WHITE,3);
    bool live=r.live(now,config::LINK_TIMEOUT_MS);
    text(banner,430,14,live?telemetry::reasonName(r.latest.reason):"CoreS3 link offline",live?MUTED:RED);
    char label[120];
    if(live)snprintf(label,sizeof(label),"Kp %.3f   Kd %.3f   Motor status %u   CAN RX %lu   I2C errors %lu",
      r.latest.command[3],r.latest.command[4],r.latest.motorStatus,(unsigned long)r.latest.rxCount,(unsigned long)errors);
    else snprintf(label,sizeof(label),"Waiting for CoreS3 on Port A / address 0x42   I2C errors %lu",(unsigned long)errors);
    text(banner,20,46,label,MUTED,2);banner.pushSprite(0,0);
  }
  delay(1);
}
}
