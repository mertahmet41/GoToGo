// Rahman ve Rahim olan Allah'ın Adıyla
// Proje: GoToGo 8.0 Alpha
// =======================================================================================

#include <Wire.h>
#include <Adafruit_PN532.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// ---------------------------------------------------------------------------------------
// 1. SİSTEM AYARLARI VE SABİTLER
// ---------------------------------------------------------------------------------------
#define KAYIT_MODU 1            // 1: EEPROM, 2: SD Kart
#define ENCODER_PIN_A 2         
#define ENCODER_PIN_B 3         
const int CHIP_SELECT_PIN = 10; 
#define mainLoopDelay 50        
#define NFC_SKIP_CONTROL 1      // 1: NFC Atla (Test Modu), 0: NFC Aktif

uint8_t referenceUID[] = { 0x3A, 0x83, 0x5B, 0x06 };   

// ---------------------------------------------------------------------------------------
// 2. PİN TANIMLAMALARI
// ---------------------------------------------------------------------------------------
// İletişim ve Hata
#define SDA_PIN A4   
#define SCL_PIN A5   
#define hata_led 0  

// Sürüş Modları
#define mode_ileri 0
#define mode_geri 0
#define mode_normal 4  
#define mode_eco 5
#define mode_sport 6

// Kontak ve Güç
#define motor_kontak 7
#define ekran_kontak 8
#define buzzer 0

// Aydınlatma Sistemi (Far 1 & Far 2)
#define far_kisa 9
#define far_uzun 11 
#define far_sis 12
#define far_ileri 0
#define far_geri 0
#define far_buton 0
#define far2_ileri 0 
#define far2_geri 0
#define far2_buton 0
#define ldr A1  
#define fark 625 

// Sinyal ve Fren Grubu
#define pw 0
#define rw 0   
#define fren 0
#define solsinsw 0
#define sagsinsw 0
#define dortlusw 0
#define solsin 0
#define sagsin 0
#define stoplambasi 0
#define sinyallambasisw 0
#define sinyallambasi 0

// Motor ve EDS Kontrol
#define motor1Pin 0 
#define motor2Pin 0
#define gazpot A1 
#define solmotorgaz 5 
#define sagmotorgaz 9 
#define EDSButon 13 

// ---------------------------------------------------------------------------------------
// 3. DEĞİŞKEN TANIMLAMALARI
// ---------------------------------------------------------------------------------------

// --- Enkoder ve Direksiyon ---
Encoder myEncoder(ENCODER_PIN_A, ENCODER_PIN_B);
const float STEPS_PER_REVOLUTION = 4000.0; 
const float DEGREES_PER_STEP = 360.0 / STEPS_PER_REVOLUTION;
const int EEPROM_ADDRESS = 0; 
const char* DATA_FILENAME = "ENKODER.TXT"; 
const long WRITE_INTERVAL = 5000; 

long currentSteps = 0; 
long lastSavedSteps = 0;
float direksiyonaci = 0.0; 
unsigned long lastWriteTime = 0; 

// --- EDS (Elektronik Diferansiyel) ---
#define maxdireksiyonaci 180.0 
int dengekatsayisi = 20;   
const int MIN_GAZ_YUZDESI = 20; 
const int ANALOG_MAX_VALUE = 1023; 

// --- Hız Ölçümü ---
volatile unsigned long pulseCount1 = 0; 
volatile unsigned long pulseCount2 = 0; 
unsigned long lastMeasureTime = 0;
const unsigned long measureInterval = 1000; 
const float MOTOR_PPR_1 = 42.0; 
const float MOTOR_PPR_2 = 42.0;
const float CIRCUMFERENCE_METER_1 = 2.07; 
const float CIRCUMFERENCE_METER_2 = 2.07;
float speed_kmh_1 = 0.0;
float speed_kmh_2 = 0.0;  

// --- Zamanlama ve Debounce ---
unsigned long beklemeBaslangicZamani = 0;  
const unsigned long beklemeSuresi = 12000; 
static unsigned long sonOkumaZamani = 0; 
static unsigned long sonOkumaZamani1 = 0; 
static unsigned long sonOkumaZamani2 = 0; 
static unsigned long sonOkumaZamani3 = 0; 
const unsigned long DEBOUNCE_SURE = 200; 
const unsigned long DEBOUNCE_SURE1 = 400; 
const unsigned long DEBOUNCE_SURE2 = 500; 
const unsigned long DEBOUNCE_SURE3 = 5000; 
unsigned long oncekiMillis = 0;    
const long aralik = 500;  

// --- Durum Değişkenleri ---
int durum = 0; 
int mode = 0; 
int far = 1; 
int far2 = 0;
bool frenw = 0;
int vites = 1; 
bool sinyalsol = 0; 
bool sinyalsag = 0; 
bool dortlu = 0;
bool sinyallambasidata = 0;
int gazseviyesi = 0; 
bool EDS = 0; 
int EDS_AKTIF = 0; 
int solGazYuzdesi = 0; 
int sagGazYuzdesi = 0; 
int gazYuzdesi = 0; 

// NFC Objesi
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

// ---------------------------------------------------------------------------------------
// 4. FONKSİYONLAR
// ---------------------------------------------------------------------------------------

// NFC Kart Karşılaştırma
bool compareUID(uint8_t* uid1, uint8_t uid1Length, uint8_t* uid2, uint8_t uid2Length) {
  if (uid1Length != uid2Length) return false;
  for (uint8_t i = 0; i < uid1Length; i++) {
    if (uid1[i] != uid2[i]) return false;
  }
  return true;
}

// Hız Ölçümü Kesmeleri
void countPulse1() { pulseCount1++; }
void countPulse2() { pulseCount2++; }

// SD Karttan Adım Verisi Okuma
long readStepsFromSD() {
  File dataFile = SD.open(DATA_FILENAME, FILE_READ);
  long loadedSteps = 0;
  if (dataFile) {
    String dataString = dataFile.readStringUntil('\n');
    dataFile.close();
    if (dataString.length() > 0) {
      dataString.trim();
      loadedSteps = dataString.toInt();
    }
  } else {
    if (KAYIT_MODU == 2) Serial.println("1205"); // SD Hata Kodu
  }
  return loadedSteps;
}

// SD Karta Adım Verisi Yazma
void writeStepsToSD(long steps) {
  if (SD.exists(DATA_FILENAME)) SD.remove(DATA_FILENAME);
  File dataFile = SD.open(DATA_FILENAME, FILE_WRITE);
  if (dataFile) {
    dataFile.println(steps);
    dataFile.close();
  }
}

// ---------------------------------------------------------------------------------------
// 5. SETUP (BAŞLANGIÇ AYARLARI)
// ---------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);   
  pinMode(hata_led, OUTPUT);

  // Pin Modu Tanımlamaları (Girişler/Çıkışlar)
  pinMode(mode_ileri, INPUT); pinMode(mode_geri, INPUT);
  pinMode(mode_normal, OUTPUT); pinMode(mode_eco, OUTPUT); pinMode(mode_sport, OUTPUT);
  pinMode(far_ileri, INPUT); pinMode(far_geri, INPUT); pinMode(far_buton, INPUT);
  pinMode(far_uzun, OUTPUT); pinMode(far_kisa, OUTPUT);
  pinMode(pw, INPUT); pinMode(rw, INPUT); pinMode(fren, INPUT);
  pinMode(dortlusw, INPUT); pinMode(solsin, OUTPUT); pinMode(sagsin, OUTPUT);
  pinMode(solsinsw, INPUT); pinMode(sagsinsw, INPUT);
  pinMode(sinyallambasisw, INPUT); pinMode(sinyallambasi, OUTPUT);
  pinMode(motor1Pin, INPUT); pinMode(motor2Pin, INPUT);
  pinMode(gazpot, INPUT); pinMode(solmotorgaz, OUTPUT); pinMode(sagmotorgaz, OUTPUT);
  pinMode(EDSButon, INPUT); pinMode(ekran_kontak, OUTPUT); pinMode(motor_kontak, OUTPUT);

  // Veri Kayıt Sistemini Başlat (EEPROM veya SD)
  if (KAYIT_MODU == 1) { 
    EEPROM.get(EEPROM_ADDRESS, currentSteps);
  } else if (KAYIT_MODU == 2) { 
    if (!SD.begin(CHIP_SELECT_PIN)) Serial.println("1205"); 
    else currentSteps = readStepsFromSD();
  }
  
  myEncoder.write(currentSteps);
  lastSavedSteps = currentSteps; 
  direksiyonaci = currentSteps * DEGREES_PER_STEP;
  lastWriteTime = millis();
  
  // Kesme (Interrupt) Ayarları
  attachInterrupt(digitalPinToInterrupt(motor1Pin), countPulse1, RISING); 
  attachInterrupt(digitalPinToInterrupt(motor2Pin), countPulse2, RISING); 
  lastMeasureTime = millis();

  // NFC Modül Kontrolü
#if NFC_SKIP_CONTROL == 0
  nfc.begin();       
  uint32_t versiondata = nfc.getFirmwareVersion();  
  if (!versiondata) {                              
    Serial.println("1100");                        
    digitalWrite(hata_led, 1);                     
    while (1);             
  }
  nfc.SAMConfig();          
  Serial.println("2200");
#else
  Serial.println("NFC kontrolü devre disi - Test modu");
#endif
}

// ---------------------------------------------------------------------------------------
// 6. ANA DÖNGÜ (LOOP)
// ---------------------------------------------------------------------------------------
void loop() {
  
  // --- Enkoder ve Direksiyon Açısı Güncelleme ---
  long newSteps = myEncoder.read();
  if (newSteps != currentSteps) {
    currentSteps = newSteps;
    direksiyonaci = currentSteps * DEGREES_PER_STEP;
  }

  unsigned long mevcutMillis = millis();

  // --- Sürüş Modu Seçimi (Debounce ile) ---
  if(mevcutMillis - sonOkumaZamani > DEBOUNCE_SURE) {
    if(digitalRead(mode_ileri) == 1) {
      if(!(mode >= 2)) {mode = mode + 1;} else {mode = 0;}
      sonOkumaZamani = mevcutMillis;
    } else if(digitalRead(mode_geri) == 1) {
        if(!(mode <= 0)) {mode = mode - 1;} else {mode = 2;}
        sonOkumaZamani = mevcutMillis;
    }
  }

  // Sürüş Modu Pin Çıkışları
  if(mode == 0){ // ECO
    digitalWrite(mode_eco, 1); digitalWrite(mode_normal, 0); digitalWrite(mode_sport, 0);
  }else if(mode == 1){ // NORMAL
    digitalWrite(mode_eco, 0); digitalWrite(mode_normal, 1); digitalWrite(mode_sport, 0); 
  }else if(mode == 2){ // SPORT
    digitalWrite(mode_eco, 0); digitalWrite(mode_normal, 0); digitalWrite(mode_sport, 1);
  }

  // --- Sistem Durumu ve Kontak Kontrolü ---
  if(durum == 0){ // Kapalı
    digitalWrite(ekran_kontak, 0); digitalWrite(motor_kontak, 0); 
  }else if(durum == 1){ // Tam Açık
    digitalWrite(ekran_kontak, 1); digitalWrite(motor_kontak, 1); 
  }else if(durum == 2){ // Sadece Ekran
    digitalWrite(ekran_kontak, 1); digitalWrite(motor_kontak, 0); 
  }

  // --- Far Sistemi (Kısa / Otomatik) ---
  if(mevcutMillis - sonOkumaZamani1 > DEBOUNCE_SURE1) {
    if(digitalRead(far_ileri) == 1) {
      if(!(far >= 2)) {far = far + 1;} else {far = 1;}
      sonOkumaZamani1 = mevcutMillis;
    } else if(digitalRead(far_geri) == 1) {
        if(!(far <= 1)) {far = far - 1;} else {far = 2;}
        sonOkumaZamani1 = mevcutMillis;
    } else if(digitalRead(far_buton) == 1){
        far = 0;
        sonOkumaZamani1 = mevcutMillis;
    }
  } 

  if(far == 0){ 
    digitalWrite(far_kisa, 0); digitalWrite(stoplambasi, 0); 
  }else if(far == 1){ // Otomatik Mod (LDR)
    if(analogRead(ldr) >= fark) { 
      digitalWrite(far_kisa, 1); digitalWrite(stoplambasi, 1);
    }else {
      digitalWrite(far_kisa, 0); digitalWrite(stoplambasi, 0);
    }
  }else if(far == 2){ // Manuel Açık
    digitalWrite(far_kisa, 1); digitalWrite(stoplambasi, 1);
  }

  // --- Far 2 Sistemi (Uzun / Sis) ---
  if(mevcutMillis - sonOkumaZamani2 > DEBOUNCE_SURE2) {
    if(digitalRead(far2_ileri) == 1) {
      if(!(far2 >= 2)) {far2 = far2 + 1;} else {far2 = 1;}
      sonOkumaZamani2 = mevcutMillis;
    } else if(digitalRead(far2_geri) == 1) {
        if(!(far2 <= 1)) {far2 = far2 - 1;} else {far2 = 2;}
        sonOkumaZamani2 = mevcutMillis;
    } else if(digitalRead(far2_buton) == 1){
        far2 = 0;
        sonOkumaZamani2 = mevcutMillis;
    }
  } 

  if(far2 == 0){ 
    digitalWrite(far_uzun, 0); digitalWrite(far_sis, 0); 
  }else if(far2 == 1){ 
    digitalWrite(far_uzun, 1); digitalWrite(far_sis, 0);
  }else if(far2 == 2){ 
    digitalWrite(far_uzun, 0); digitalWrite(far_sis, 1);
  }

  // --- Vites Kontrolü ---
  if (digitalRead(pw) == HIGH) vites = 1;
  else if (digitalRead(rw) == HIGH) vites = 2; 
  else vites = 0;

  // --- Sinyalizasyon Sistemi (Dörtlü / Sağ / Sol) ---
  if(digitalRead(dortlusw) == 1) {
    if (dortlu == 0) { dortlu = 1; } else { dortlu = 0; }  
  }
  
  if(dortlu == 1){
      if (mevcutMillis - oncekiMillis >= aralik) {    
      oncekiMillis = mevcutMillis;
      if (sinyalsag == 0) { sinyalsag = 1; sinyalsol = 1; } 
      else { sinyalsol = 0; sinyalsag = 0; }
  }}else if(digitalRead(solsinsw) == 1) {
    if (mevcutMillis - oncekiMillis >= aralik) {    
      oncekiMillis = mevcutMillis;
      if (sinyalsol == 0) { sinyalsol = 1; } else { sinyalsol = 0; }
  }}else if(digitalRead(sagsinsw) == 1){
     if (mevcutMillis - oncekiMillis >= aralik) {    
      oncekiMillis = mevcutMillis;
      if (sinyalsag == 0) { sinyalsag = 1; } else { sinyalsag = 0; } 
  }}else{
    sinyalsag = 0; sinyalsol = 0;
  }

  // Sinyal Çıkışları
  if(dortlu == 1) {
    digitalWrite(solsin, 1); digitalWrite(sagsin, 1);
  }else if(sinyalsag == 1) {
    digitalWrite(sagsin, 1); digitalWrite(solsin, 0);
  }else if(sinyalsol == 1) {
    digitalWrite(solsin, 1); digitalWrite(sagsin, 0);
  }else {
    digitalWrite(solsin, 0); digitalWrite(sagsin, 1);
  }

  // Sinyal Lambası (Ek Fonksiyon)
  if(digitalRead(sinyallambasisw) == 1){
    sinyallambasidata = !sinyallambasidata;
  }
  digitalWrite(sinyallambasi, sinyallambasidata);

  // --- Hız Ölçümü Hesaplamaları ---
  if (mevcutMillis - lastMeasureTime >= measureInterval) {
      noInterrupts(); 
      unsigned long currentCount1 = pulseCount1;
      unsigned long currentCount2 = pulseCount2;
      pulseCount1 = 0; pulseCount2 = 0; 
      interrupts();
      speed_kmh_1 = (currentCount1 * CIRCUMFERENCE_METER_1 * 3.6) / MOTOR_PPR_1;
      speed_kmh_2 = (currentCount2 * CIRCUMFERENCE_METER_2 * 3.6) / MOTOR_PPR_2;
      lastMeasureTime = mevcutMillis;
  }

  // --- Gaz Potansiyometresi Okuma ---
  gazseviyesi = analogRead(gazpot); 
  gazYuzdesi = map(gazseviyesi, 0, ANALOG_MAX_VALUE, 0, 100);

  // --- EDS (Elektronik Diferansiyel Sistemi) Kontrolü ---
  if(digitalRead(EDSButon) == HIGH) {
    EDS = !EDS;
  }

  int gazSol = gazseviyesi;
  int gazSag = gazseviyesi;
  EDS_AKTIF = 0; 

  // EDS Aktivasyon Şartı: Buton açık VE Gaz %20'den fazla
  if (EDS == 1 && gazYuzdesi >= MIN_GAZ_YUZDESI) {
      EDS_AKTIF = 1; 
      float oran = abs(direksiyonaci) / maxdireksiyonaci;
      int uygulamaFarki = (int)(oran * dengekatsayisi);

      if(direksiyonaci > 0){ // Sağa dönüş
          gazSol = gazseviyesi + uygulamaFarki;
          gazSag = gazseviyesi - uygulamaFarki;
      }else if(direksiyonaci < 0){ // Sola dönüş
          gazSol = gazseviyesi - uygulamaFarki;
          gazSag = gazseviyesi + uygulamaFarki;
      }
      gazSol = constrain(gazSol, 0, ANALOG_MAX_VALUE); 
      gazSag = constrain(gazSag, 0, ANALOG_MAX_VALUE);
  } 

  // Motorlara PWM Çıkışı
  analogWrite(solmotorgaz, gazSol);
  analogWrite(sagmotorgaz, gazSag);

  // Yüzdesel Geri Bildirim Hesaplama
  solGazYuzdesi = map(gazSol, 0, ANALOG_MAX_VALUE, 0, 100);
  sagGazYuzdesi = map(gazSag, 0, ANALOG_MAX_VALUE, 0, 100);

  // --- Veri Kayıt İşlemi (EEPROM/SD) ---
  if (mevcutMillis - lastWriteTime >= WRITE_INTERVAL) {
    if (currentSteps != lastSavedSteps) {
      if (KAYIT_MODU == 1) EEPROM.put(EEPROM_ADDRESS, currentSteps);
      else if (KAYIT_MODU == 2) writeStepsToSD(currentSteps);
      lastWriteTime = mevcutMillis;
      lastSavedSteps = currentSteps;
    }
  }

  // --- Seri Port Veri Paketi Çıkışı ---
  Serial.print(mode); Serial.print("/");
  Serial.print(far); Serial.print("/");
  Serial.print(durum); Serial.print("/");
  Serial.print(far2); Serial.print("/");
  Serial.print(vites); Serial.print("/");
  Serial.print(sinyallambasidata); Serial.print("/");
  Serial.print(dortlu); Serial.print("/");
  Serial.print(sinyalsol); Serial.print("/");
  Serial.print(sinyalsag); Serial.print("/");
  Serial.print(speed_kmh_1); Serial.print("/");
  Serial.print(speed_kmh_2); Serial.print("/");
  Serial.print(direksiyonaci, 1); Serial.print("/");
  Serial.print(solGazYuzdesi); Serial.print("/");
  Serial.print(sagGazYuzdesi); Serial.print("/");
  Serial.println(EDS_AKTIF); 

  delay(mainLoopDelay);
}