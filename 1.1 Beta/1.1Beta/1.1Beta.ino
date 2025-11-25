//Rahman Ve Rahim Olan ALLAH'ın Adıyla
//GoToGo 1.1 Beta

#include <SPI.h>
#include <MFRC522.h>

// RFID için tanımlamalar
#define RST_PIN 0
#define SS_PIN 0
MFRC522 rfid(SS_PIN, RST_PIN);

// Yetkili kart ID'si (Örnek: kendi kartınızın ID'sini yazın)
byte authorizedID[4] = {0x12, 0x34, 0x56, 0x78};

// Zaman sabitleri
const unsigned long KART_TIMEOUT = 300000;    // 5 dakika (milisaniye)
const unsigned long SUREKLI_OKUMA_SURE = 3000; // 3 saniye
const unsigned long MOTOR_TIMEOUT = 15000;    // 15 saniye
#define sinyalsn 200   //ms cinsinden

//Pin Tanımlamaları  
// Motor ve Sensörler
#define SolMotorHiz        A0
#define SagMotorHiz        A1
#define AnlikTuketim       A2
//#define AnlikVoltaj        A3
#define BataryaVoltaj    A4

// Sinyaller ve Işıklar (Giriş + Çıkış)
#define hataLed            2
#define SolSinyalButon     0
#define SolSinyalLed       0

#define SagSinyalButon     0
#define SagSinyalLed       0

#define StopButon          0
#define StopLed            0

#define OnFarButon         0
#define OnFarLed           0

#define DortluButon        0
#define sinyallambabuton   0
#define sinyallamba        0

// Sıcaklık Sensörleri
#define OrtamSicaklik      0
#define SolMotorSicaklik   0
#define SagMotorSicaklik   0

// Vites Konumu
#define VitesD             0
#define VitesP             0
#define VitesR             0

// Multimedya Kontrolleri
#define MultimediaSag      0
#define MultimediaSol      0
#define MultimediaYukari   0
#define MultimediaAsagi    0
#define MultimediaOk       0

// Diğer
#define AcilDurum          0
#define Enlem              0
#define Boylam             0
#define EkranKontak        9  // Pikontak pini
#define MotorKontak        8
#define frenswitch  

// Değişkenler
unsigned long sonOkumaZamani = 0;
unsigned long surekliOkumaBaslangic = 0;
bool kartSurekliOkunuyor = false;
bool motorAcik = false;
bool pinkontakAcik = false;
int Solsinyaldata = 0;
int Sagsinyaldata = 0;

void setup() {
  //Pin Tanımlamaları
  // Giriş pinleri
  pinMode(SolSinyalButon, INPUT);
  pinMode(SagSinyalButon, INPUT);
  pinMode(StopButon, INPUT);
  pinMode(OnFarButon, INPUT);
  pinMode(DortluButon, INPUT);
  pinMode(VitesD, INPUT);
  pinMode(VitesP, INPUT);
  pinMode(VitesR, INPUT);
  pinMode(MultimediaSag, INPUT);
  pinMode(MultimediaSol, INPUT);
  pinMode(MultimediaYukari, INPUT);
  pinMode(MultimediaAsagi, INPUT);
  pinMode(MultimediaOk, INPUT);
  pinMode(AcilDurum, INPUT);

  // Çıkış pinleri
  pinMode(SolSinyalLed, OUTPUT);
  pinMode(SagSinyalLed, OUTPUT);
  pinMode(StopLed, OUTPUT);
  pinMode(OnFarLed, OUTPUT);
  pinMode(hataLed, OUTPUT);
  pinMode(EkranKontak, OUTPUT); // Pikontak pini
  pinMode(MotorKontak, OUTPUT); // Motorkontak pini
  
  // Başlangıçta tüm kontaklar kapalı
  digitalWrite(EkranKontak, LOW);
  digitalWrite(MotorKontak, LOW);
  
  // RFID başlatma
  SPI.begin();
  rfid.PCD_Init();
}

void loop() {

//RFID Kart okunması
  unsigned long simdi = millis();
  bool yeniKartOkundu = false;

  // RFID okuma
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    yeniKartOkundu = true;
    
    // Kart yetkili mi kontrol et
    bool yetkiliKart = true;
    for (byte i = 0; i < 4; i++) {
      if (rfid.uid.uidByte[i] != authorizedID[i]) {
        yetkiliKart = false;
        break;
      }
    }

    if (yetkiliKart) {
      sonOkumaZamani = simdi;
      
      // İlk okumada pinkontak aç
      if (!pinkontakAcik) {
        digitalWrite(EkranKontak, HIGH);
        pinkontakAcik = true;
      }

      // Sürekli okuma kontrolü
      if (!kartSurekliOkunuyor) {
        surekliOkumaBaslangic = simdi;
        kartSurekliOkunuyor = true;
      } else {
        // 3 saniye boyunca sürekli okunuyorsa motoru aç
        if (simdi - surekliOkumaBaslangic >= SUREKLI_OKUMA_SURE && !motorAcik) {
          digitalWrite(MotorKontak, HIGH);
          motorAcik = true;
        }
      }
    }
    
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }

  // Kart okunmadıysa sürekli okuma durumunu sıfırla
  if (!yeniKartOkundu && kartSurekliOkunuyor) {
    kartSurekliOkunuyor = false;
  }

  // Zaman aşımlarını kontrol et
  if (pinkontakAcik) {
    // 5 dakika içinde kart okunmazsa pinkontak kapat
    if (simdi - sonOkumaZamani >= KART_TIMEOUT) {
      digitalWrite(EkranKontak, LOW);
      digitalWrite(MotorKontak, LOW);
      pinkontakAcik = false;
      motorAcik = false;
      kartSurekliOkunuyor = false;
    }
  }

  if (motorAcik && !kartSurekliOkunuyor) {
    // Motor açık ve kart sürekli okunmuyorsa 15 sn sonra motoru kapat
    if (simdi - sonOkumaZamani >= MOTOR_TIMEOUT) {
      digitalWrite(MotorKontak, LOW);
      motorAcik = false;
    }
  }



if (pinkontakAcik == 1) {
//Sinayllerin kontrolü
  if(digitalRead(SolSinyalButon) == HIGH) {
      delay(sinyalsn);
      digitalWrite(SolSinyalLed,1 );
      Solsinyaldata = 1;  
      delay(sinyalsn);
      digitalWrite(SolSinyalLed,0 );
      Solsinyaldata = 0; 
    }else {
      if(digitalRead(SagSinyalButon) == HIGH) {
      delay(sinyalsn);
      digitalWrite(SagSinyalLed,1 );
      Sagsinyaldata = 1;  
      delay(sinyalsn);
      digitalWrite(SagSinyalLed,0 );
      Sagsinyaldata = 0; 

      }else {
        if(digitalRead(DortluButon) == HIGH) {
          delay(sinyalsn);
          digitalWrite(SolSinyalLed,1 );
          Solsinyaldata = 1;
          digitalWrite(SagSinyalLed,1 );
          Sagsinyaldata = 1;    
          delay(sinyalsn);
          digitalWrite(SolSinyalLed,0 );
          Solsinyaldata = 0; 
          digitalWrite(SagSinyalLed,0 );
          Sagsinyaldata = 0; 

        }else{
          digitalWrite(SolSinyalLed,0 );
          digitalWrite(SagSinyalLed,0 );
          Sagsinyaldata = 0;
          Solsinyaldata = 0;
          }
      }
    }
  
  
  
  
  
  
 }
}