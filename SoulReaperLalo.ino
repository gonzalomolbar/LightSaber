/*
FALTA:

Colores con un encoder?

Amplificador?
SetpDown?

*/

#define NUM_LEDS 31                   // Number of leds in the strip
#define BRIGHTNESS 255                // max LED brightness (0 - 255)

#define BTN_HOLD_TIMEOUT 800         // button hold delay in ms
#define BTN_TIMEOUT 20               // button delay for errors

#define MPU_TIMEOUT 500               // delay at checking the MPU

#define BUZZ_RECORDED_TIMEOUT 9000    // delay at checking buzz in recorded mode
#define BUZZ_DIRECT_TIMEOUT 3         // delay at checking buzz in direct mode

#define STRIKE_WEAK 150               // hit acceleration treshold
#define STRIKE_STRONG 320             // hard hit acceleration treshold

#define SWING_TIMEOUT 500             // timeout between swings
#define SWING_SLOW 150                // swing angle speed threshold
#define SWING_FAST 300                // fast swing angle speed threshold

#define FLASH_DELAY 80                // flash time while hit

#define PULSE_TIMEOUT 30              // delay between pulses
#define PULSE_ALLOW 1                 // blade pulsation (1 - allow, 0 - disallow)
#define PULSE_AMPL 20                 // pulse amplitude

#define BATTERY_TIMEOUT 3000          // delay at checking battery level
#define MINIMUN_BATTERY_LEVEL 10      // minimun level of battery to turn the LS on
#define BATTERY_SAFE 0                // battery monitoring (1 - allow, 0 - disallow)

#define RAINBOW_STEP 10               // step in changing the LS color

#define DEBUG 1                       // debug information in Serial (1 - allow, 0 - disallow)



#define BTN 2
#define BTN_LED 3
#define LED_PIN1 6
#define LED_PIN2 7
#define SPEAKER_PIN 10
#define CARD_PIN 4
#define CARD_GND A0
#define IMU_GND A1
#define VOLT_PIN A3


#include <SD.h>
#include <TMRpcm.h>         // audio from SD library
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <toneAC.h>         // buzz generation library
#include "FastLED.h"        // addressable LED library
#include <EEPROM.h>
#include <SPI.h>




CRGB leds1[NUM_LEDS];
CRGB leds2[NUM_LEDS];
TMRpcm tmrpcm;
MPU6050 accelgyro;


unsigned long ACC, GYR, COMPL;
int freq_f = 20;

float k = 0.2;  //Smoother

unsigned long btn_timer = 0, pulse_timer = 0, mpu_timer = 0, battery_timer = 0, swing_timer = 0, buzz_timerDirect = 0, buzz_timerRecorded = -BUZZ_RECORDED_TIMEOUT;

boolean ls_assaultMode = 1, buzz_recordMode = 1, ls_isOn = 0;
boolean btn_flag = 0, btn_flagHold = 0, swing_flag = 0, strike_flag = 0;

byte capacity = 0;
byte btn_pressCounter = 0;
byte color_now = 0, red = 0, green = 0, blue = 0;

int pulse_offSet = 0, counter = 0; //VALORAR PULSE OFFSEAT

int strike_time[16] = {270, 167, 186, 250, 252, 255, 250, 238, 779, 563, 687, 702, 673, 661, 666, 635};
int swing_time[8] = {636, 441, 772, 702, 389, 372, 360, 366};




void setup() {
  
  //COSAS DE LOS LEDS
  FastLED.addLeds<WS2812B, LED_PIN1, GRB>(leds1, NUM_LEDS).setCorrection( TypicalLEDStrip );                     //////OJO IGUAL HAY QUE QUITAR LA B y lo de typical
  FastLED.addLeds<WS2812B, LED_PIN2, GRB>(leds2, NUM_LEDS).setCorrection( TypicalLEDStrip );                     //////OJO IGUAL HAY QUE QUITAR LA B y lo de typical
  FastLED.setBrightness(BRIGHTNESS);  // ~40% of LED strip brightness
  setAll(0, 0, 0);


  Wire.begin();           //Esto es para activar el I2C y que el A4 sea SDA y el A5 sea SCL
  Serial.begin(5600);


  //PINES
  pinMode(BTN, INPUT_PULLUP);
  pinMode(IMU_GND, OUTPUT);
  pinMode(BTN_LED, OUTPUT);
  digitalWrite(IMU_GND, 0);
  digitalWrite(BTN_LED, 1);



  randomSeed(analogRead(2));




  // IMU initialization
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  if (DEBUG) {
    if (accelgyro.testConnection()) Serial.println(F("MPU6050 OK"));
    else Serial.println(F("MPU6050 fail"));
  }



  // SD initialization
  tmrpcm.speakerPin = SPEAKER_PIN;
  tmrpcm.quality(1);
  if (DEBUG) {
    if (SD.begin(CARD_PIN)) Serial.println(F("SD OK"));
    else Serial.println(F("SD fail"));
  } else {
    SD.begin(CARD_PIN);
  }



  //Leer valores grabados en EEPROMN
  if ((EEPROM.read(0) >= 0) && (EEPROM.read(0) <= 5)) {
    color_now = EEPROM.read(0);
    buzz_recordMode = EEPROM.read(1);

  } else {
    EEPROM.write(0, color_now);
    EEPROM.write(1, buzz_recordMode);
    color_now = 0;
  }


  //Mostrar nivel de bateria al encender el arduino
  setColor(color_now);
  capacity = getBatteryLevel();
  if (DEBUG) {
    Serial.print(F("Battery Leds: "));
    Serial.println(capacity);
  }
  byte ledsCapacity = map(capacity, 100, 0, (NUM_LEDS), 1);

  for (char i = 0; i <= ledsCapacity; i++) {
    setPixel(i, 255, 0, 0);
    FastLED.show();
    delay(25);
  }
  delay(2000);
  setAll(0, 0, 0);
  FastLED.setBrightness(BRIGHTNESS);
}





void loop() {
  
  btnCheck();         //
  batteryCheck();   //
    
    
  //Modo asalto activado
  if(ls_assaultMode && ls_isOn){
    
    randomPulse();    //
    buzzCheck();      //
    getDataMPU();     // Vigilar mas
    strikeCheck();    // CALIBRAR
    swingCheck();     // CALIBRAR
    
    
  //Modo vigia activado
  }else if(!ls_assaultMode){    
        
    if (Serial.available()>0){
      
      //leemos la opcion enviada
      char option = Serial.read();
      //Presencia
          
      if(option=='C') {
        
        setLsWave(0);
        
      }
      if(option=='P') {
        
        setLsWave(1);
        
      }
    }
  }
}


void setLsWave(byte function) {
  
  
  byte wave_red1, wave_green1, wave_blue1, wave_red2, wave_green2, wave_blue2;
  char charBuf[8];
  
  if(function == 1){
    char charBuf[8] = {'D','A', 'N', 'G', '.', 'w', 'a', 'v'};

    wave_red1 = 255;
    wave_green1 = 0;
    wave_blue1 = 0;
    
    wave_red2 = 0;
    wave_green2 = 255;
    wave_blue2 = 0;
    
  }else if(function == 0){
    char charBuf[8] = {'C','O', '2', 'A', '.', 'w', 'a', 'v'};

    wave_red1 = 0;
    wave_green1 = 255;
    wave_blue1 = 0;
    
    wave_red2 = 255;
    wave_green2 = 255;
    wave_blue2 = 0;
  }
  
  tmrpcm.play(charBuf);
  
  delay(200);
  
  for (int k = 0; k < 5; k++) {  
    for (int i = 0; i < NUM_LEDS; i++) {  
      
      setPixel(i - 2, 0, 0, 0);
      setPixel(NUM_LEDS - 1 - i + 2, 0, 0, 0);

      setPixel(i - 1, wave_red1, wave_green1, wave_blue1);
      setPixel(NUM_LEDS - 1 - i + 1, wave_red2, wave_green2, wave_blue2);
      
      setPixel(i, wave_red1, wave_green1, wave_blue1);
      setPixel(NUM_LEDS - 1 - i, wave_red2, wave_green2, wave_blue2);
      
      FastLED.show();
      delay(25);
    }
  setAll(0, 0, 0);
  delay(25);

  }
  
  delay(200);
}



void btnCheck() {
  
  boolean btn_state = !digitalRead(BTN);
  
  //Pulsación invididual de boton (No hace nada)
  if (btn_state && !btn_flag && ((millis() - btn_timer) > BTN_TIMEOUT)) {
    if (DEBUG) Serial.println(F("Boton Apretado"));
    
    btn_flag = 1;
    btn_pressCounter++;
    btn_timer = millis();
  }
  
  //Sueltas el boton
  if (!btn_state && btn_flag) {
    if (DEBUG) Serial.println(F("Boton Soltado"));

    btn_flag = 0;
    btn_flagHold = 0;
  }
  
  unsigned long btn_actualTimer = millis() - btn_timer;
  
  // Lo mantiene pulsado
  if (btn_flag && btn_state && (btn_actualTimer > BTN_HOLD_TIMEOUT) && !btn_flagHold && ls_assaultMode) {
    
    //Cambiar modo de sonido
    if(btn_pressCounter == 3 && ls_isOn){
      if (DEBUG){
        Serial.println(F("Se ha mantenido pulsado el boton habiendose pulsado 3 veces. Modo de sonido cambiado"));
        if(buzz_recordMode) Serial.println(F("MODO SONIDO EN DIRECTO"));
        else Serial.println(F("MODO SONIDO GRABADO"));
      }  
        buzz_recordMode = !buzz_recordMode;
        if (buzz_recordMode) {
          noToneAC();
          tmrpcm.play("HUM.wav");
        } else {
          tmrpcm.disable();
          toneAC(freq_f);
        }
        EEPROM.write(1, buzz_recordMode);
        
    //Modo aroiris
    }else if(btn_pressCounter == 5 && ls_isOn){
      if (DEBUG) Serial.println(F("Se ha mantenido pulsado el boton habiendose pulsado 5 veces. Arcoiris acttivado"));
      
      setHitFlash();
      delay(500);
      
      boolean end = 0;
      while(!end){
        end = goRainbow();
      }
      
    //Modo epilepsia
    }else if(btn_pressCounter == 7 && ls_isOn){
      if (DEBUG) Serial.println(F("Se ha mantenido pulsado el boton habiendose pulsado 7 veces. Epilepsia activacda"));
      
      setHitFlash();
      delay(500);
      
      boolean end = 0;
      while(!end){
        end = goEpilepsia();
      }
      
    //Apagar/encender sable
    }else{
      if (DEBUG){
        Serial.print(F("Se ha mantenido pulsado el boton. "));
        if(ls_isOn) Serial.println(F("APAGANDOSE"));
        else Serial.println(F("ENCENDIENDOSE"));
      }
      onOff();
      
    }
    btn_flagHold = 1;
    btn_pressCounter = 0;
  }
  
  // Si fue pulsado varias veces antes del timeout
  if ((btn_actualTimer > BTN_HOLD_TIMEOUT) && (btn_pressCounter != 0)) {
    
    //Cambiar color
    if (btn_pressCounter == 2 && ls_assaultMode && ls_isOn) {
      if (DEBUG) Serial.println(F("Boton pulsado 2 veces. Cambiando color"));

      color_now++;
      if (color_now >= 6) color_now = 0;
      setColor(color_now);
      setAll(red, green, blue);
      EEPROM.write(0, color_now);
    }
    
    //Cambiar modo vigia/asalto
    if (btn_pressCounter == 3) {
      if (DEBUG){
        Serial.println(F("Boton pulsado 3 veces. "));
        if(ls_assaultMode) Serial.println(F("MODO VIGIA ACTIVADO"));
        else Serial.println(F("MODO ASALTO ACTIVADO. BE READY"));
      }

      ls_assaultMode = !ls_assaultMode;

      if(ls_isOn){
        onOff();
      }
    }
  
    btn_pressCounter = 0;
  }
}



void onOff() {
  
    if (!ls_isOn) {
      getBatteryLevel();
      if (capacity > MINIMUN_BATTERY_LEVEL || !BATTERY_SAFE) {
        if (DEBUG) Serial.println(F("ENCENDIDO"));
        setLsOn();
        
        ls_isOn = true;
        
        if (buzz_recordMode) {
        noToneAC();
        tmrpcm.play("HUM.wav");
      } else {
        tmrpcm.disable();
        toneAC(freq_f);
      }
      
    } else {
      if (DEBUG) Serial.println(F("No hay batería ponlo a cargar so marrano"));
      for (int i = 0; i < 3; i++) {
        setAll(255, 0, 0);
        delay(400);
        setAll(0, 0, 0);
        delay(400);
      }
    }
    
  } else {
    noToneAC();
    tmrpcm.disable();
    
    setLsOff();
    
    if (DEBUG) Serial.println(F("APAGADO"));
    
    ls_isOn = false;
    
  }
}



void buzzCheck(){
  
  if (((millis() - buzz_timerRecorded) > BUZZ_RECORDED_TIMEOUT) && buzz_recordMode) {
    tmrpcm.play("HUM.wav");
    buzz_timerRecorded = millis();
    swing_flag = 1;
    strike_flag = 0;
  }
  
  if ((millis() - buzz_timerDirect > BUZZ_DIRECT_TIMEOUT) && !buzz_recordMode) {
    if (strike_flag) {
      tmrpcm.disable();
      strike_flag = 0;
    }
    toneAC(freq_f);
    buzz_timerDirect = millis();
  }
  
}



void randomPulse() {
  
  if ((millis() - pulse_timer > PULSE_TIMEOUT) && PULSE_ALLOW) {
    pulse_timer = millis();
    pulse_offSet = pulse_offSet * k + random(-PULSE_AMPL, PULSE_AMPL) * (1 - k);
    
    if (color_now == 0){
      pulse_offSet = constrain(pulse_offSet, -15, 5);
    }
    byte red_offset = constrain(red + pulse_offSet, 0, 255);
    byte green_offset = constrain(green + pulse_offSet, 0, 255);
    byte blue_offset = constrain(blue + pulse_offSet, 0, 255);
    
    setAll(red_offset, green_offset, blue_offset);
  }
  
}



void strikeCheck() {
  if (ACC >= STRIKE_WEAK) {
    char force;
    byte multStrong;
    
    if(ACC >= STRIKE_STRONG){
      Serial.println(F("Strike Fuerte"));

      force = 'S';
      multStrong=1;
    }else{
      Serial.println(F("Strike Debil"));

    
      force = 'W';
      multStrong=0;
    }
    
    if (!buzz_recordMode) noToneAC();
    
    byte number_random = random(sizeof(strike_time)/2);
    char charBuf[8] = {'S','K', force, number_random, '.', 'w', 'a', 'v'};
    tmrpcm.play(charBuf);
    
    setHitFlash();
    
     if (!buzz_recordMode)
      buzz_timerDirect = millis() + strike_time[number_random+sizeof(strike_time)*multStrong] - FLASH_DELAY;
    else
      buzz_timerRecorded = millis() - BUZZ_RECORDED_TIMEOUT + strike_time[number_random+((sizeof(strike_time)/2)*multStrong)] - FLASH_DELAY;
      
    strike_flag = 1;
  }
}



void swingCheck() {
  
  if((millis() - swing_timer > SWING_TIMEOUT) && buzz_recordMode && swing_flag && !strike_flag){                    /////////////OJO AL TIMER CHECKEAR QUE VAYA BIEN
    if (GYR >= SWING_SLOW) {
      char speed;
      byte multFast;
      
      if(GYR >= SWING_FAST){
        Serial.println(F("Swing Rapido"));

        speed = 'F';
        multFast=1;
      }else{
        Serial.println(F("Swing Lento"));

        speed = 'S';
        multFast=0;
      }
      
      byte number_random = random(sizeof(swing_time)/2);
      char charBuf[8] = {'S','W', speed, number_random, '.', 'w', 'a', 'v'};
      tmrpcm.play(charBuf);

      buzz_timerRecorded = millis() - BUZZ_RECORDED_TIMEOUT + swing_time[number_random+((sizeof(swing_time)/2)*multFast)];
      
      swing_flag = 0;
      swing_timer = millis();
    }
  }
}



void getDataMPU() {

  if (millis() - mpu_timer > MPU_TIMEOUT) {
    int16_t ax, ay, az, gx, gy, gz;
    int gyroX, gyroY, gyroZ, accelX, accelY, accelZ, freq;

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       

    // find absolute and divide on 100
    gyroX = abs(gx / 100);
    gyroY = abs(gy / 100);
    gyroZ = abs(gz / 100);
    accelX = abs(ax / 100);
    accelY = abs(ay / 100);
    accelZ = abs(az / 100);

    // vector sum
    ACC = sq((long)accelX) + sq((long)accelY) + sq((long)accelZ);
    ACC = sqrt(ACC);
    GYR = sq((long)gyroX) + sq((long)gyroY) + sq((long)gyroZ);
    GYR = sqrt((long)GYR);
    COMPL = ACC + GYR;

    freq = (long)COMPL * COMPL / 1500;                        // parabolic tone change
    freq = constrain(freq, 18, 300);                          
    freq_f = freq * k + freq_f * (1 - k);                     // smooth filter
    
    
    if (DEBUG) {
      /*
      Serial.println();
      Serial.println();
      Serial.print(F("ACC: "));
      Serial.println(ACC);
      Serial.print(F("GYR: "));
      Serial.println(GYR);
      Serial.print(F("Freq: "));
      Serial.println(freq_f);
    */
    /*
       Serial.print(F("$Giroscopio: "));
       Serial.print(gyroX);
       Serial.print(F(" "));
       Serial.print(gyroY);
       Serial.print(F(" "));
       Serial.print(gyroZ);
       Serial.print(F(":   "));
       Serial.print(GYR);
       Serial.println(F(";"));
       Serial.print(F("Acelerometro: "));
       Serial.print(accelX);
       Serial.print(F(" "));
       Serial.print(accelY);
       Serial.print(F(" "));
       Serial.print(accelZ);
       Serial.print(F(":   "));
       Serial.print(ACC);
       Serial.println(F(";"));
       Serial.print(F("Total: "));
       Serial.print(COMPL);
       Serial.println(F(";"));
       Serial.println(F(""));
       Serial.print(F("freq: "));
       Serial.print(freq);
       Serial.print(F("         freq_f: "));
       Serial.print(freq_f);
       Serial.println(F(""));
       Serial.println(F(""));
*/
    }
    mpu_timer = micros();                                                             /////WTF
  }
  
}



void setLsOn() {
  
  tmrpcm.play("ON.wav");
  delay(200);
  for (int i = 0; i < NUM_LEDS; i++) {        
    setPixel(i, red, green, blue);
    FastLED.show();
    delay(25);
  }
  delay(200);
  
}



void setLsOff() {
  
  tmrpcm.play("OFF.wav");
  delay(300);
  for (int i = NUM_LEDS - 1; i >= 0; i--) {      
    setPixel(i, 0, 0, 0);
    FastLED.show();
    delay(25);
  }
  delay(300);
  
}



void setHitFlash() {
  
  setAll(255, 255, 255);            
  delay(FLASH_DELAY);                
  setAll(red, blue, green);    
  
}



boolean goEpilepsia(){
  boolean end = !digitalRead(BTN);
  
  int counter_aux;
  for (int i = 0; i < NUM_LEDS; i++ ) {
    counter_aux = random(1530);

    setColorCounter(i, counter_aux);
  }
  FastLED.show();
  
  delay(50);
  
  return end;
}



boolean goRainbow(){
  boolean end = !digitalRead(BTN);
  
  counter = counter + RAINBOW_STEP;
  
  if(counter > 1530){
    counter = counter - 1530;
  }

  
  int counter_aux = counter;
  for (int i = 0; i < NUM_LEDS; i++ ) {
    counter_aux = counter_aux + RAINBOW_STEP;
    if(counter_aux > 1530){
      counter_aux = counter_aux - 1530;
    }
    setColorCounter(i, counter_aux);

  }
  FastLED.show();
  
  delay(10);
  
  return end;
}



void setPixel(int pixel, byte red, byte green, byte blue) {
  
  if(pixel < 0) pixel = 0;
  if(pixel >= NUM_LEDS) pixel = NUM_LEDS - 1;
  
  leds1[pixel].r = red;
  leds1[pixel].g = green;
  leds1[pixel].b = blue;
  
  leds2[pixel].r = red;
  leds2[pixel].g = green;
  leds2[pixel].b = blue;
}



void setAll(byte red, byte green, byte blue) {
  
  for (int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  FastLED.show();
  
}



void setColor(byte color) {
  
  switch (color) {
    // 0 - rojo, 1 - verde, 2 - azul, 3 - rosa, 4 - amarillo, 5 - azul verdoso
    case 0:
      red = 255;
      green = 0;
      blue = 0;
      break;
    case 1:
      red = 0;
      green = 0;
      blue = 255;
      break;
    case 2:
      red = 0;
      green = 255;
      blue = 0;
      break;
    case 3:
      red = 255;
      green = 0;
      blue = 255;
      break;
    case 4:
      red = 255;
      green = 255;
      blue = 0;
      break;
    case 5:
      red = 0;
      green = 255;
      blue = 255;
      break;
  }
  
}



void setColorCounter(int pixel, int counter){
  
  byte rw_red, rw_green, rw_blue;

  if(counter>=0 && counter<255){
    rw_red=255;
    rw_green=counter%255;
    rw_blue=0;
  }
  if(counter>=255 && counter<510){
    rw_red=255-(counter%255);
    rw_green=255;
    rw_blue=0;
  }
  if(counter>=510 && counter<765){
    rw_red=0;
    rw_green=255;
    rw_blue=counter%255;
  }
  if(counter>=765 && counter<1020){
    rw_red=0;
    rw_green=255-(counter%255);
    rw_blue=255;
  }
  if(counter>=1020 && counter<1275){
    rw_red=counter%255;
    rw_green=0;
    rw_blue=255;
  }
  if(counter>=1275 && counter<1530){
    rw_red=255;
    rw_green=0;
    rw_blue=255-(counter%255);
  }
  
  setPixel(pixel, rw_red, rw_green, rw_blue);
}



void batteryCheck() {
  
  if (millis() - battery_timer > BATTERY_TIMEOUT && BATTERY_SAFE) {
    getBatteryLevel();
    if (ls_isOn && capacity < MINIMUN_BATTERY_LEVEL) {
      onOff();
    }
    battery_timer = millis();
  }
}



byte getBatteryLevel() {
  
  float voltage = 0;
  int R1 = 2000;
  int R2 = 1000;
  
  for (int i = 0; i < 10; i++) {    
    voltage += (float)analogRead(VOLT_PIN) * 5 / 1023 * (R1 + R2) / R2;
  }
  voltage = voltage / 10; //Calcular la media 
  int volts = voltage * 100;

  if (volts > 820)
    capacity = 100;
    
  else if ((volts <= 820) && (volts > 809) )
    capacity = map(volts, 820, 809, 100, 83);
    
  else if ((volts <= 809) && (volts > 765) )
    capacity = map(volts, 809, 765, 83, 58);
    
  else if ((volts <= 765) && (volts > 746) )
    capacity = map(volts, 765, 746, 58, 41);
    
  else if ((volts <= 746) && (volts > 720) )
    capacity = map(volts, 746, 720, 41, 0);
    
  else if (volts <= 720)
    capacity = 0;
  
  /*
  Serial.println(F(""));
  Serial.print(F("BATERIA: "));
  Serial.println(capacity);
  */
  return capacity;
}
