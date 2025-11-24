#include <Wire.h>
#include <Adafruit_PN532.h>

// Rahman Ve Rahim Olan Allah'ın Adıyla
// GoToGo 4.0(BETA)

uint8_t referenceUID[] = { 0x3A, 0x83, 0x5B, 0x06 };   // Yetkili kart (4 byte)

//-------------------------------------------// Pin Tanımlamaları
#define SDA_PIN 4   // NFC kart SDA pin
#define SCL_PIN 5   // NFC kart SCL pin
#define hata_led 0  // hata ledi

//-------------------------------------------// Tanımlamalar
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

//-------------------------------------------// Değişken Tanımlamaları
int a1 = 0;             // Bekleme modu Değişkeni
unsigned long beklemeBaslangicZamani = 0;
const unsigned long beklemeSuresi = 5000; // 5 saniye

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
//-----------------------------------------------------------------------------// NFC KART MODÜLÜ
  nfc.begin();       // nfc okuma başlatılıyor
  uint32_t versiondata = nfc.getFirmwareVersion();  
  if (!versiondata) {                               // Modül kontrolü yapılıyor
    Serial.println("1100");                         // nfc haberleşme hatası olan 1100 hatası veriliyor
    digitalWrite(hata_led, 1);                      // hata ledini açıyor
    while (1);              // Sistem Durduruluyor
  }
  nfc.SAMConfig();            // nfc okuma başlıyor
//-----------------------------------------------------------------------------// NFC KART MODÜLÜ
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
      Serial.println("AÇIK DURUM"); // AÇIK DURUM                 
    } else {
      Serial.println("Yetkisiz Kart!");
    }
  } else {
    // Kart okunamadığı durum
    if (a1 == 1) {
      Serial.println("BEKLEME DURUMU"); // BEKLEME DURUMU
      
      // 5 saniye bekleme kontrolü
      if (millis() - beklemeBaslangicZamani >= beklemeSuresi) {
        a1 = 0;
        Serial.println("KAPALI DURUM"); // KAPALI DURUM
      }
    } else {        
      if(a1 == 0){
        // Sadece durum değiştiğinde yazdır (spam önlemek için)
        static unsigned long sonYazdirma = 0;
        if (millis() - sonYazdirma >= 2000) { // Her 2 saniyede bir yazdır
          Serial.println("KAPALI DURUM"); // KAPALI DURUM
          sonYazdirma = millis();
        }
      }
    }
  }
//--------------------------------------------------------------//













}
