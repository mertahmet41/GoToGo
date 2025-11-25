// Rahman Ve Rahim Olan Allah'ın Adıyla
// GoToGo 5.0(BETA)

#include <Wire.h>
#include <Adafruit_PN532.h>


uint8_t referenceUID[] = { 0x3A, 0x83, 0x5B, 0x06 };   // Yetkili kart (4 byte)

//-------------------------------------------// Pin Tanımlamaları
#define SDA_PIN A4   // NFC kart SDA pin
#define SCL_PIN A5   // NFC kart SCL pin
#define hata_led 0  // hata ledi
#define mode_ileri 0// Mod Tuşu ileri
#define mode_geri 0// Mod Tuşu Geri
#define mode_normal 3  
#define mode_eco 2
#define mode_sport 6
#define motor_kontak 7
#define ekran_kontak 8
#define buzzer 0
#define far_kisa 0
#define far_uzun 0
#define far_sis 0
#define far_ileri 0
#define far_geri 0
#define far_buton 0
#define far2_ileri 4 //Far2 2. far ayarı tekerleği dir
#define far2_geri 5
#define far2_buton 9


//-------------------------------------------// Tanımlamalar
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

//-------------------------------------------// Değişken Tanımlamaları
//DİĞER DEĞİŞKENLER
int a1 = 0;             // Bekleme modu Değişkeni
unsigned long beklemeBaslangicZamani = 0;  //NFC Milis Değişkenleri
const unsigned long beklemeSuresi = 12000; // 13 saniye
static unsigned long sonOkumaZamani3 = 0; // NCD Bekleme Değişkenleri
const unsigned long DEBOUNCE_SURE3 = 5000; // Beklemeye Geçiş gecikme ms
static unsigned long sonOkumaZamani = 0; // Mod Milis Değişkenleri
const unsigned long DEBOUNCE_SURE = 200; // mode geçiş gecikme ms
static unsigned long sonOkumaZamani1 = 0; // far Milis Değişkenleri
const unsigned long DEBOUNCE_SURE1 = 400; // far geçiş gecikme ms
static unsigned long sonOkumaZamani2 = 0; // far2 Milis Değişkenleri
const unsigned long DEBOUNCE_SURE2 = 500; // far2 geçiş gecikme ms
//DATA DEĞİŞKENLERİ
int durum = 0; // 0 KAPALI 1 AÇIK 3 YETKİSİZ KART
int mode = 0; // 0 eco 1 Normal 2 Sport
int far = 1; //0 Kapalı 1 Aouto 2 Kısa
int far2 = 0;


// compareUID fonksiyonu
bool compareUID(uint8_t* uid1, uint8_t uid1Length, uint8_t* uid2, uint8_t uid2Length) {
  if (uid1Length != uid2Length) return false;
  for (uint8_t i = 0; i < uid1Length; i++) {
    if (uid1[i] != uid2[i]) return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);   // Seri Haberleşme Başlatılıyor
  pinMode(hata_led, OUTPUT);

  //pin Mode Tanımlamaları
    pinMode(mode_ileri, INPUT);
    pinMode(mode_geri, INPUT);
    pinMode(mode_normal, OUTPUT);
    pinMode(mode_eco, OUTPUT);
    pinMode(mode_sport, OUTPUT);
    pinMode(far_ileri, INPUT);
    pinMode(far_geri, INPUT);
    pinMode(far_buton, INPUT);
    pinMode(far_uzun, OUTPUT);
    pinMode(far_kisa, OUTPUT);


  nfc.begin();       // nfc okuma başlatılıyor
  uint32_t versiondata = nfc.getFirmwareVersion();  
  if (!versiondata) {                               // Modül kontrolü yapılıyor
    Serial.println("1100");                         // nfc haberleşme hatası olan 1100 hatası veriliyor
    digitalWrite(hata_led, 1);                      // hata ledini açıyor
    while (1);              // Sistem Durduruluyor
  }
  nfc.SAMConfig();            // nfc okuma başlıyor
  Serial.println("Sistem başlatıldı. Kart bekleniyor...");
}

void loop() {
  // NFC Kartın Okunması Ve Tepki Verilmesi
  //--------------------------------------------//  
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  uint8_t uidLength;
  
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100); // 100ms timeout eklendi
  
  if (success) {
    if (compareUID(uid, uidLength, referenceUID, sizeof(referenceUID))) {      
      a1 = 1;
      beklemeBaslangicZamani = millis(); // Zamanlayıcıyı sıfırla
      durum = 1; // AÇIK DURUM                 
    } else {
      durum = 3;
    }
  } else {
    // Kart okunamadığı durum
    if (a1 == 1) {
      if(millis() - sonOkumaZamani3 > DEBOUNCE_SURE3) {
      durum = 2; // BEKLEME DURUMU
     sonOkumaZamani1 = millis();
      }      // 5 saniye bekleme kontrolü
      if (millis() - beklemeBaslangicZamani >= beklemeSuresi) {
        a1 = 0;
        durum = 0; // KAPALI DURUM
      }
    } else {        
      if(a1 == 0){
        // Sadece durum değiştiğinde yazdır (spam önlemek için)
        static unsigned long sonYazdirma = 0;
        if (millis() - sonYazdirma >= 2000) { // Her 2 saniyede bir yazdır
          durum = 0; // KAPALI DURUM
          sonYazdirma = millis();
        }
      }
    }
  }
 
//---------------------------------------------------------------------------// Mod Değişimleri Atama

if(millis() - sonOkumaZamani > DEBOUNCE_SURE) {
  if(digitalRead(mode_ileri) == 1) {
    if(!(mode >= 2)) {mode = mode + 1;} else {mode = 0;}
    sonOkumaZamani = millis();
  } else {
    if(digitalRead(mode_geri) == 1) {
      if(!(mode <= 0)) {mode = mode - 1;} else {mode = 2;}
      sonOkumaZamani = millis();
    }
  }
}
//-------------------------------------------------------------// Mode Değişimleri Pin Tepkimesi
if(mode == 0){
  digitalWrite(mode_eco, 1); 
  digitalWrite(mode_normal, 0); 
  digitalWrite(mode_sport, 0);
}else if(mode == 1){
  digitalWrite(mode_eco, 0); 
  digitalWrite(mode_normal, 1); 
  digitalWrite(mode_sport, 0); 
}else if(mode == 2){
  digitalWrite(mode_eco, 0); 
  digitalWrite(mode_normal, 0); 
  digitalWrite(mode_sport, 1);
}
//---------------------------------------------------------------// Drum Pin Tepkimesi
if(durum == 0){
  digitalWrite(ekran_kontak, 0); 
  digitalWrite(motor_kontak, 0); 
}else if(durum == 1){
  digitalWrite(ekran_kontak, 1); 
  digitalWrite(motor_kontak, 1); 
}else if(durum == 2){
  digitalWrite(ekran_kontak, 1); 
  digitalWrite(motor_kontak, 0); 
}
//------------------------------------------------------// Far Değişkeni Atama
if(millis() - sonOkumaZamani1 > DEBOUNCE_SURE1) {
  if(digitalRead(far_ileri) == 1) {
    if(!(far >= 2)) {far = far + 1;} else {far = 1;}
    sonOkumaZamani1 = millis();
  } else {
    if(digitalRead(far_geri) == 1) {
      if(!(far <= 1)) {far = far - 1;} else {far = 2;}
      sonOkumaZamani1 = millis();
    }else{
    if(digitalRead(far_buton) == 1){
      far = 0;
    }
    }
  }
} 
//----------------------------------------------------------------------// Far Değişkeni pin Tepkimesi
if(far == 0){ 
  digitalWrite(far_kisa, 0); 
}else if(far == 1){ 
  //Far Auto Kısmı
}else if(far == 2){ 
  digitalWrite(far_kisa, 1);
}
//----------------------------------------------------------------------// Far2 Değişkeni atama
if(millis() - sonOkumaZamani2 > DEBOUNCE_SURE2) {
  if(digitalRead(far2_ileri) == 1) {
    if(!(far2 >= 2)) {far2 = far2 + 1;} else {far2 = 1;}
    sonOkumaZamani2 = millis();
  } else {
    if(digitalRead(far2_geri) == 1) {
      if(!(far2 <= 1)) {far2 = far2 - 1;} else {far2 = 2;}
      sonOkumaZamani2 = millis();
    }else{
    if(digitalRead(far2_buton) == 1){
      far2 = 0;
    }
    }
  }
} 
//-------------------------------------------------------------------// Far2 Değişken Tepkimesi
if(far2 == 0){ 
  digitalWrite(far_uzun, 0); 
  digitalWrite(far_sis, 0); 
}else if(far2 == 1){ 
  digitalWrite(far_uzun, 1); 
  digitalWrite(far_sis, 0);
}else if(far2 == 2){ 
  digitalWrite(far_uzun, 0); 
  digitalWrite(far_sis, 1);
}







Serial.print(mode); Serial.print("/"); Serial.print(far); Serial.print("/"); Serial.print(durum);Serial.print("/");
 Serial.print(far2); Serial.println("/");  //Seri Yazdırma


  }
