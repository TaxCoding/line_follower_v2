/*
  LINE FOLLOWER V2  -  Arduino Nano + L298N + 6 sensor analog lurus
  -----------------------------------------------------------------
  Wiring L298N:
    header 1 -> in3 | header 2 -> in4 | header 3 -> enb
    header 4 -> ena | header 5 -> in2 | header 6 -> in1
*/

#include <LiquidCrystal.h>
#include <EEPROM.h>

LiquidCrystal lcd(A1, A0, 2, 3, 4, 5);

// ===================== PIN =====================
const int button1 = 7;
const int button2 = 6;
const int in1 = 13, in2 = 12, ena = 11;   // motor kanan
const int in3 = 8,  in4 = 9,  enb = 10;   // motor kiri
const int pinSensor[6] = {A7, A6, A5, A4, A3, A2};  // index 0 = paling kiri

// ===================== PARAMETER =====================
// true  = garis hitam memberi nilai analog LEBIH TINGGI
// false = garis memberi nilai lebih rendah
#define GARIS_NILAI_TINGGI true

const float KP = 0.045;     // naikkan sampai mulai bergoyang, lalu turunkan sedikit
const float KD = 1.2;       // naikkan untuk meredam goyangan

const int SPEED_DASAR = 140;            // kecepatan di garis lurus (mulai rendah, naikkan bertahap)
const int SPEED_MIN   = 70;             // kecepatan terendah saat tikungan
const float FAKTOR_PERLAMBATAN = 0.03;  // makin besar = makin pelan di tikungan
const int REV_MAX     = 110;            // batas maksimum putaran mundur saat belok tajam
const int OFFSET_KIRI = 10;             // kompensasi motor kiri (dari kode lama: +10)

const int SPEED_GAP = 110;              // kecepatan melewati garis putus-putus
const int PIVOT     = 120;              // kecepatan putar saat mencari garis
const unsigned long GAP_MAX_MS    = 350;   // maksimal lurus buta saat garis hilang
const unsigned long LOST_STOP_MS  = 2500;  // garis hilang selama ini -> berhenti (finish)
const unsigned long LOOP_US       = 2500;  // periode loop PID (2.5 ms)

// ===================== DATA SENSOR =====================
const int BOBOT[6] = {-2500, -1500, -500, 500, 1500, 2500};
const int AMBANG_JUMLAH = 200; 

struct Kalib {
  byte magic;
  int mn[6];
  int mx[6];
};
Kalib kal;
const byte MAGIC = 0xA5;

int raw[6];
int nilai[6];        // hasil normalisasi 0..1000 (1000 = garis)
long posisi = 0;     // -2500 (garis di kiri) .. +2500 (garis di kanan)

// ===================== MOTOR =====================
void setMotor(int kiri, int kanan) {
  kiri  = constrain(kiri,  -REV_MAX, 255);
  kanan = constrain(kanan, -REV_MAX, 255);

  // Motor kiri: maju = in3 HIGH, in4 LOW
  if (kiri >= 0) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW); }
  else           { digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); }
  int pk = abs(kiri);
  if (pk > 0) pk = min(255, pk + OFFSET_KIRI);
  analogWrite(enb, pk);

  // Motor kanan: maju = in2 HIGH, in1 LOW
  if (kanan >= 0) { digitalWrite(in2, HIGH); digitalWrite(in1, LOW); }
  else            { digitalWrite(in2, LOW);  digitalWrite(in1, HIGH); }
  analogWrite(ena, abs(kanan));
}

void rem() {   // rem aktif L298N
  digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
  analogWrite(ena, 255);
  analogWrite(enb, 255);
}

// ===================== TOMBOL =====================
bool tekan(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(25);
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) {}
      return true;
    }
  }
  return false;
}

// ===================== SENSOR =====================
void bacaRaw() {
  for (int i = 0; i < 6; i++) raw[i] = analogRead(pinSensor[i]);
}

//Posisi di variabel global 'posisi'.
bool bacaPosisi() {
  bacaRaw();
  long sum = 0, wsum = 0;
  for (int i = 0; i < 6; i++) {
    int rentang = kal.mx[i] - kal.mn[i];
    if (rentang < 50) rentang = 50;
    long v = (long)(raw[i] - kal.mn[i]) * 1000L / rentang;
    v = constrain(v, 0, 1000);
#if !GARIS_NILAI_TINGGI
    v = 1000 - v;
#endif
    if (v < 100) v = 0;         
    nilai[i] = v;
    sum  += v;
    wsum += v * (long)BOBOT[i];
  }
  if (sum < AMBANG_JUMLAH) return false;
  posisi = wsum / sum;
  return true;
}

// ===================== KALIBRASI =====================
void simpanKalib() {
  kal.magic = MAGIC;
  EEPROM.put(0, kal);
}

void muatKalib() {
  EEPROM.get(0, kal);
  if (kal.magic != MAGIC) {
    for (int i = 0; i < 6; i++) { kal.mn[i] = 100; kal.mx[i] = 900; }
  }
}

void kalibrasi() {
  lcd.clear();
  lcd.print("KALIBRASI");
  lcd.setCursor(0, 1);
  lcd.print("B1: mulai");
  while (!tekan(button1)) {}

  for (int i = 0; i < 6; i++) { kal.mn[i] = 1023; kal.mx[i] = 0; }

  lcd.clear();
  lcd.print("GESER KIRI-KANAN");
  unsigned long t0 = millis();
  unsigned long tLcd = 0;
  while (millis() - t0 < 5000) {
    bacaRaw();
    for (int i = 0; i < 6; i++) {
      if (raw[i] < kal.mn[i]) kal.mn[i] = raw[i];
      if (raw[i] > kal.mx[i]) kal.mx[i] = raw[i];
    }
    if (millis() - tLcd > 400) {
      tLcd = millis();
      lcd.setCursor(0, 1);
      lcd.print("sisa ");
      lcd.print((5000 - (millis() - t0)) / 1000 + 1);
      lcd.print(" detik  ");
    }
  }

  bool lemah = false;
  for (int i = 0; i < 6; i++) if (kal.mx[i] - kal.mn[i] < 100) lemah = true;

  simpanKalib();
  lcd.clear();
  if (lemah) {
    lcd.print("CEK SENSOR!");
    lcd.setCursor(0, 1);
    lcd.print("ada yg lemah");
  } else {
    lcd.print("KALIBRASI OK");
  }
  delay(1500);
}

// ===================== TES SENSOR =====================
void tesSensor() {
  lcd.clear();
  while (!tekan(button2)) {
    bool ada = bacaPosisi();
    lcd.setCursor(0, 0);
    for (int i = 0; i < 6; i++) lcd.print(nilai[i] > 500 ? 1 : 0);
    lcd.print(ada ? "  ada   " : "  HILANG");
    lcd.setCursor(0, 1);
    lcd.print("pos=");
    lcd.print(ada ? posisi : 0);
    lcd.print("      ");
    delay(100);
  }
}

// ===================== PROGRAM JALAN =====================
void jalan() {
  lcd.clear();
  lcd.print("JALAN  (B2=stop)");
  delay(300);

  float lastPos = 0;
  int lastSide = 0;
  bool sedangHilang = false;
  unsigned long tHilang = 0;
  unsigned long tLoop = micros();

  while (true) {
    while (micros() - tLoop < LOOP_US) {}   // jaga periode loop tetap
    tLoop = micros();

    if (digitalRead(button2) == LOW) {      // tombol stop
      rem();
      tekan(button2);
      return;
    }

    if (bacaPosisi()) {
      if (sedangHilang) { sedangHilang = false; lastPos = posisi; }
      if (labs(posisi) > 500) lastSide = (posisi > 0) ? 1 : -1;

      float p = posisi;
      float koreksi = KP * p + KD * (p - lastPos);
      lastPos = p;

      int speed = SPEED_DASAR - (int)(fabs(p) * FAKTOR_PERLAMBATAN);
      if (speed < SPEED_MIN) speed = SPEED_MIN;

      // garis di kanan (posisi > 0) -> roda kiri lebih cepat -> belok kanan
      setMotor((int)(speed + koreksi), (int)(speed - koreksi));
    }
    else {
      if (!sedangHilang) { sedangHilang = true; tHilang = millis(); }
      unsigned long lama = millis() - tHilang;

      if (lama > LOST_STOP_MS) {            // garis habis -> finish
        rem();
        lcd.clear();
        lcd.print("GARIS HABIS");
        delay(1000);
        return;
      }

      if (fabs(lastPos) < 1200 && lama < GAP_MAX_MS) {
        setMotor(SPEED_GAP, SPEED_GAP);     // garis putus-putus
      }
      else if (lastSide <= 0) {
        setMotor(-PIVOT, PIVOT);            // garis terakhir di kiri
      }
      else {
        setMotor(PIVOT, -PIVOT);            // garis terakhir di kanan
      }
    }
  }
}

// ===================== MENU =====================
int mode = 0;
const int JUMLAH_MODE = 3;

void tampilMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (mode == 0) lcd.print("> JALAN");
  if (mode == 1) lcd.print("> KALIBRASI");
  if (mode == 2) lcd.print("> TES SENSOR");
  lcd.setCursor(0, 1);
  lcd.print("B2:ganti  B1:OK");
}

void setup() {
  // ADC prescaler 32
  ADCSRA = (ADCSRA & ~0x07) | 0x05;

  lcd.begin(16, 2);
  Serial.begin(9600);

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(ena, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); pinMode(enb, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  rem();

  muatKalib();

  lcd.setCursor(0, 0);
  lcd.print("    MAN BATAM   ");
  lcd.setCursor(0, 1);
  lcd.print(kal.magic == MAGIC ? "  kalibrasi OK  " : " BELUM KALIBRASI");
  delay(1500);
  tampilMenu();
}

void loop() {
  if (tekan(button2)) {
    mode = (mode + 1) % JUMLAH_MODE;
    tampilMenu();
  }
  if (tekan(button1)) {
    if (mode == 0) jalan();
    if (mode == 1) kalibrasi();
    if (mode == 2) tesSensor();
    tampilMenu();
  }
}
