;//Library
#include <Wire.h>                     //Library wire
#include <LiquidCrystal_I2C.h>        //Library LCD_I2C
#include <Keypad.h>                   //Library keypad
#include <Thermocouple.h>             //Library termokopel
#include <MAX6675_Thermocouple.h>     //Library MAX6675 
//Motor
#define ENC_COUNT_REV 374
#define ENC_IN         18             //Rotary Encoder
#define CLK            19             //Rotary Encoder
#define MOTOR          A0
#define relay          A3
//PID
#define KP 19.9
#define KI 15.0
#define KD 0
//Inisialisasi
#define UPDATE_TIME   50              //Time
#define SCK_PIN       10              //MAX6675
#define CS_PIN        11              //MAX6675
#define SO_PIN        12              //MAX6675
#define servo         13              //Servo
Thermocouple* thermocouple;           //Thermocouple
LiquidCrystal_I2C lcd (0x27, 20, 4);  //LCD_I2C
const byte ROWS = 4;                  //Keypad
const byte COLS = 4;                  //Keypad
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
String key_array, key_produk;
int menuu = 1;                        //Nilai awal variabel menu
byte degreeSymbol = B11011111;        //Simbol degree
int setpointT, setpointM, setpointW;  //Inisialisasi setpoint temperature, motor, waktu
//Menu
int sub_menu = 0;
int sub = 0;
int awal = 0;
int menu_key = 0; 
int del = 0; 
int i = 0, in_key = 0;
int data_temp_in = 0, data_kec_in = 0, data_waktu_in = 0;
int data_in = 0;
double celsius = 0;
int data_celcius = 0;
unsigned long data_waktu_setpoint = 0;
int interval = 1;
volatile long encoderValue = 0;
float rpm = 0; 
int a =0;
int data_kec_setpoint = 0;
//Satu kali
//Waktu
long motor_awal_Millis = 0;
long motor_sekarang_Millis = 0;
long previousMillis = 0;
long currentMillis = 0;
//PID
float eror_kec, previous_eror;
float PID_P, PID_I, PID_D, PID_TOTAL;
float time_pid;
int period_pid = 50;
int ats = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(servo, OUTPUT);                                      //Servo
  pinMode(ENC_IN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder, RISING);
  pinMode(MOTOR, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(relay, OUTPUT);
  
  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);
  lcd.init();
  lcd.backlight();                                        //Open the backlight
  lcd.begin(20, 4);                                       //Memanggil fungsi LCD
  lcd.setCursor(3, 1); lcd.print("HELLO!!");
  lcd.setCursor(3, 2); lcd.print("SELAMAT DATANG");       //Menampilkan pada LCD
  lcd.setCursor(17, 3); lcd.print("009");                 //Menampilkan pada LCD
  delay(1000);                                            //Delay
  lcd.clear();                                            //Menghapus pada LCD
  lcd.setCursor(0, 2); lcd.print(" CUSTOM");
  lcd.setCursor(0, 3); lcd.print(" PILIH PRODUK");
}
//Berulang
void loop() {
  char key = customKeypad.getKey();                      //Keypad yang ditekan
  if (menu_key == 0) {
        analogWrite(servo,convertirAngleEnPWM(0));
        delay(UPDATE_TIME);
    Serial.println(key);
    if (key != NO_KEY) {
      if (key == 'D') {
        lcd.clear();
        lcd.setCursor(0, 2); lcd.print(" CUSTOM");
        lcd.setCursor(0, 3); lcd.print(" PILIH PRODUK");
        awal = 0; sub_menu = 9; sub = 0;
        delay(100);
      }
    }
  }
  if (awal == 0) {
    lcd.setCursor(4, 0); lcd.print("PILIHAN MENU");
    lcd.setCursor(0, 1); lcd.print("--------------------");
    if (key == 'A') {
      lcd.clear();
      lcd.setCursor(0, 2); lcd.print(">CUSTOM");
      lcd.setCursor(0, 3); lcd.print(" PILIH PRODUK");
      delay(100); sub = 1;
    }
    if (key == 'B') {
      lcd.clear();
      lcd.setCursor(0, 2); lcd.print(" CUSTOM");
      lcd.setCursor(0, 3); lcd.print(">PILIH PRODUK");
      delay(100); sub = 2;
    }
    if (key == 'C') {
      if (sub == 1) { 
        awal = 1; sub_menu = 1; menu_key = 1; key = 0;
        lcd.clear(); 
      }
      if (sub == 2) {
        awal = 1; sub_menu = 2; 
        lcd.clear();
      }
    }
  }
  if (sub_menu == 1) {
      lcd.setCursor(1, 0); lcd.print("MASUKKAN SET POINT");    
      lcd.setCursor(0, 1); lcd.print("Temperatur= "); 
      lcd.setCursor(16, 1); lcd.write(degreeSymbol); 
      lcd.setCursor(17, 1); lcd.print("C"); 
      lcd.setCursor(0, 2); lcd.print("Kecepatan = ");
      lcd.setCursor(16, 2); lcd.print("RPM");
      lcd.setCursor(0, 3); lcd.print("Waktu(t)  = ");
      lcd.setCursor(16, 3); lcd.print("Mnt");
      if (menu_key == 1) {
        if (key != NO_KEY) {
          key_array += key;
          Serial.println(key);
          data_in = key_array.toInt();
          if (in_key == 0){
            data_temp_in = data_in;
            Serial.print("Data Key in Temp:"); Serial.println(data_temp_in);
            lcd.setCursor(12, 1); lcd.print(key_array);
            i ++;
            if (i == 2){
              in_key = 1; i = 0; key = 0; data_in = 0; key_array = ' ';
              if(data_celcius >= 80){
                 data_celcius  = 80;
              }
            }
          }
          if (in_key == 1){
            data_kec_in = data_in;
            Serial.print("Data Key in Kecepatan:"); Serial.println(data_kec_in);
            lcd.setCursor(11, 2); lcd.print(key_array);
            i ++;
            if (i == 4){
              in_key = 2; 
              i = 0; key = 0; 
              key_array = ' '; 
              data_in = 0;
              sub_menu = 1;
              Serial.println("rusak in_key 1");
              if(data_kec_in >= 999){
                 data_kec_in  = 999;
                Serial.println("rusak 1300");
              }
            }
          }
          if (in_key == 2){
            a ++;
            if(a>=1){
              data_waktu_in = data_in;
              Serial.print("Data Key in Waktu:"); Serial.println(data_waktu_in);
              lcd.setCursor(11, 3); lcd.print(key_array); 
            }
            if (a >= 4){
                in_key = 0; a = 0; key = 0; key_array = ' '; data_in = 0;
                menu_key = 0; sub_menu = 3; lcd.clear();
               if(data_waktu_in >= 120){
                  data_waktu_in  = 120;
              }
            }
          }
          if (key == 'D') {
            lcd.clear();
            lcd.setCursor(0, 2); lcd.print(" CUSTOM");
            lcd.setCursor(0, 3); lcd.print(" PILIH PRODUK");
            awal = 0; sub = 0; menu_key = 0; sub_menu = 0;
          }
        }
      }
    }
  if (sub_menu == 2) {
    lcd.setCursor(2, 0); lcd.print("PILIH SATU PRODUK");
    lcd.setCursor(0, 1); lcd.print("[Tekan 1] LUNAK");
    lcd.setCursor(0, 2); lcd.print("[Tekan 2] SEDANG");
    lcd.setCursor(0, 3); lcd.print("[Tekan 3] PADAT");
    if (key != NO_KEY) {
      Serial.println(key);
      if (key == '1'){
        data_celcius = 12;
        data_kec_in = 100;
        data_waktu_in = 2;
        sub_menu = 3;
      }
      if (key == '2'){
        data_celcius = 16;
        data_kec_in = 200;
        data_waktu_in = 4;
        sub_menu = 3;
      }
      if (key == '3'){
        data_celcius = 25;
        data_kec_in = 400;
        data_waktu_in = 6;
        sub_menu = 3;
      }
    }
    key = ' ';
  }
  if (sub_menu == 3){
    celsius = thermocouple->readCelsius();
    lcd.setCursor(0, 0); lcd.print("SPTemp= ");
    lcd.setCursor(13, 0); lcd.write(degreeSymbol); 
    lcd.setCursor(14, 0); lcd.print("C");
    lcd.setCursor(8, 0); lcd.print(data_temp_in);    
    lcd.setCursor(0, 1); lcd.print("Temp  = "); 
    lcd.setCursor(13, 1); lcd.write(degreeSymbol); 
    lcd.setCursor(14, 1); lcd.print("C");
    lcd.setCursor(8, 1); lcd.print(celsius);

    lcd.setCursor(0, 2); lcd.print("SPKec = ");
    lcd.setCursor(14, 2); lcd.print("RPM");
    lcd.setCursor(8, 2); lcd.print(data_kec_in);
    lcd.setCursor(0, 3); lcd.print("Waktu = ");
    lcd.setCursor(14, 3); lcd.print("Mnt");
    lcd.setCursor(8, 3); lcd.print(data_waktu_in);
    
    Serial.print("Data Temperature :"); Serial.print(celsius);
    Serial.print(" || Setpoint :"); Serial.println(data_temp_in);
    
    if (celsius < data_temp_in){
      if (ats == 0){
        for(int i=0; i<90; i++){
        analogWrite(servo,convertirAngleEnPWM(i));
        delay(UPDATE_TIME);
        Serial.println("suhu dibawah setpoint");
      }
     delay(500);
     ats = 1;
     }
    } else if(celsius > data_temp_in){
      for(int i=90; i>=0; i--){
        analogWrite(servo,convertirAngleEnPWM(i));
        delay(UPDATE_TIME);
        Serial.println("suhu diatas setpoint");
      }
     delay(500);
     data_celcius = data_temp_in;
     data_waktu_setpoint = data_waktu_in * 60000;
     data_kec_setpoint = map(data_kec_in, 0, 999, 0, 900);
     motor_awal_Millis = millis();
     previousMillis = millis();
     time_pid = millis();
     sub_menu = 4; lcd.clear();
     Serial.println("11");
    }
  }
  if(sub_menu == 4){
    motor_sekarang_Millis = millis();
    if (millis() - previousMillis >= interval){
       previousMillis = millis();
        if (rpm > 0) {
          //Serial.print("Kecepatan VALUE: "); Serial.print(data_kec_in); Serial.print('\t');
          //Serial.print(" PULSES: ");   Serial.print(encoderValue); Serial.print('\t');
          //Serial.print(" SPEED: ");
          Serial.print(rpm); //Serial.println(" RPM");
        }
      } 
      rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);
      Serial.print(data_kec_setpoint);
      eror_kec = data_kec_setpoint - rpm; 
      PID_P = KP * eror_kec;
      float kec_diference = eror_kec - previous_eror; 
      PID_D = KD * ((eror_kec - previous_eror));
      if(-5 < eror_kec && eror_kec< 5){
        PID_I = PID_I + (KI * eror_kec);
      } else {
        PID_TOTAL = PID_P + PID_I + PID_D;
        if(PID_TOTAL < 5){PID_TOTAL = 5;}
        if(PID_TOTAL > 700){PID_TOTAL = 700;}
        previous_eror = eror_kec;
        //Serial.println(PID_TOTAL);
        analogWrite(MOTOR, PID_TOTAL);
      }
      encoderValue = 0;
      
    lcd.setCursor(1, 0); lcd.print("MOTOR BERPUTAR");    
    lcd.setCursor(0, 1); lcd.print("PUL:"); 
    lcd.setCursor(5, 1); lcd.print(encoderValue); 
    lcd.setCursor(1, 2); lcd.print("RPM: ");
    lcd.setCursor(10, 2); lcd.print(rpm);
    lcd.setCursor(16, 2); lcd.print("rpm");
    lcd.setCursor(1, 3); lcd.print("Input:");
    lcd.setCursor(12, 3); lcd.print(data_kec_in);
    lcd.setCursor(16, 3); lcd.print("rpm");
    digitalWrite(relay, HIGH);
   
    if (motor_sekarang_Millis - motor_awal_Millis >= data_waktu_setpoint){
      currentMillis = millis();
      digitalWrite(relay, LOW);
      analogWrite(MOTOR, 0.0);
      lcd.clear();
      lcd.setCursor(0, 2); lcd.print(" CUSTOM");
      lcd.setCursor(0, 3); lcd.print(" PILIH PRODUK");
      awal = 0; sub = 0; menu_key = 0; sub_menu = 0;
    }
  }
}
int convertirAngleEnPWM(int ang){
  float a = 255.0/180.0;
  return (a*ang);
}
void updateEncoder(){
  encoderValue++;
}
