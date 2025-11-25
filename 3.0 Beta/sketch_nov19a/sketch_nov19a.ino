//Rahman Ve Rahim Olan ALLAH'ın Adıyla
//GoToGo 1.1 Beta

#include <Wire.h>
#include <Adafruit_PN532.h>

#define SDA_PIN A4
#define SCL_PIN A5

Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);
byte authorizedID[4] = {0x3A 0x83 0x5B 0x6};

// T ÜM PİN TANIMLAMALARI - KENDİNE GÖRE AYARLA!
#define SolSinyalButon     3
#define SolSinyalLed       4
#define SagSinyalButon     5
#define SagSinyalLed       6
#define StopButon          7
#define StopLed            8
#define OnFarButon         9
#define OnFarLed           10
#define DortluButon        11
#define sinyallambabuton   12
#define sinyallamba        13
#define EkranKontak        26
#define MotorKontak        27
#define frenswitch         23

// MULTIMEDYA ve VİTES PİNLERİ - SADECE GİRİŞ OLARAK KULLANILACAK
#define MultimediaSol      14
#define MultimediaSag      15
#define MultimediaYukari   16
#define MultimediaAsagi    17
#define MultimediaOk       18
#define VitesD             19
#define VitesP             20
#define VitesR             21
#define AcilDurum          22

// SENSÖR PİNLERİ
#define SolMotorHiz        A0
#define SagMotorHiz        A1
#define AnlikTuketim       A2
#define BataryaVoltaj      A3
#define OrtamSicaklik      A4
#define SolMotorSicaklik   A5
#define SagMotorSicaklik   A6

// Tüm Data Değişkenleri
int Solsinyaldata = 0, Sagsinyaldata = 0, Stopdata = 0, OnFardata = 0;
int Dortludata = 0, EkranKontakdata = 0, MotorKontakdata = 0;
int VitesDdata = 0, VitesPdata = 0, VitesRdata = 0;
int MultimediaSagdata = 0, MultimediaSoldata = 0;
int MultimediaYukaridata = 0, MultimediaAsagidata = 0, MultimediaOkdata = 0;
int AcilDurumdata = 0, sinyallambadata = 0;
float BataryaVoltajdata = 0, AnlikAkimdata = 0, ToplamGucdata = 0;
float OrtamSicaklikdata = 0, SolMotorSicaklikdata = 0, SagMotorSicaklikdata = 0;

// Sistem Değişkenleri
bool motorAcik = false, pinkontakAcik = false;
unsigned long previousSinyalMillis = 0;
bool sinyalDurum = false;
int aktifSinyal = 0;
bool sinyalToggle[4] = {false, false, false, false};

void setup() {
  // GİRİŞ PİNLERİ
  pinMode(SolSinyalButon, INPUT);
  pinMode(SagSinyalButon, INPUT);
  pinMode(StopButon, INPUT);
  pinMode(OnFarButon, INPUT);
  pinMode(DortluButon, INPUT);
  pinMode(sinyallambabuton, INPUT);
  pinMode(frenswitch, INPUT);
  
  // MULTIMEDYA ve VİTES GİRİŞLERİ - SADECE OKUNACAK
  pinMode(MultimediaSol, INPUT);
  pinMode(MultimediaSag, INPUT);
  pinMode(MultimediaYukari, INPUT);
  pinMode(MultimediaAsagi, INPUT);
  pinMode(MultimediaOk, INPUT);
  pinMode(VitesD, INPUT);
  pinMode(VitesP, INPUT);
  pinMode(VitesR, INPUT);
  pinMode(AcilDurum, INPUT);
  
  // ÇIKIŞ PİNLERİ
  pinMode(SolSinyalLed, OUTPUT);
  pinMode(SagSinyalLed, OUTPUT);
  pinMode(StopLed, OUTPUT);
  pinMode(OnFarLed, OUTPUT);
  pinMode(sinyallamba, OUTPUT);
  pinMode(EkranKontak, OUTPUT);
  pinMode(MotorKontak, OUTPUT);
  
  // Başlangıçta tüm çıkışlar kapalı
  digitalWrite(SolSinyalLed, LOW);
  digitalWrite(SagSinyalLed, LOW);
  digitalWrite(StopLed, LOW);
  digitalWrite(OnFarLed, LOW);
  digitalWrite(sinyallamba, LOW);
  digitalWrite(EkranKontak, LOW);
  digitalWrite(MotorKontak, LOW);
  
  // RFID ve Seri İletişim
  nfc.begin();
   nfc.SAMConfig();
  //SPI.begin();
  //rfid.PCD_Init();
  Serial.begin(115200);
}

void loop() {
  // RFID okuma
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  if (success) {
    bool yetkiliKart = true;
    for (byte i = 0; i < 4; i++) {
      if (rfid.uid.uidByte[i] != authorizedID[i]) {
        yetkiliKart = false;
        break;
      }
    }
    if (yetkiliKart) {
      pinkontakAcik = true;
      digitalWrite(EkranKontak, HIGH);
      EkranKontakdata = 1;
    }
    //rfid.PICC_HaltA();
    //rfid.PCD_StopCrypto1();
  }

  // Kontak durumlarını data değişkenlerine ata
  EkranKontakdata = pinkontakAcik;
  MotorKontakdata = motorAcik;

  // Tüm kontroller
  if (pinkontakAcik) {
    sinyalKontrol();
    stopKontrol();
    farKontrol();
    sinyallambaKontrol();
    multimediaKontrol();    // SADECE OKUMA - ÇIKIŞ YOK
    vitesKontrol();         // SADECE OKUMA - ÇIKIŞ YOK
    sensorOku();
    acilDurumKontrol();     // SADECE OKUMA - ÇIKIŞ YOK
  } else {
    // Pinkontak kapalıysa tüm çıkışları kapat ve data'ları sıfırla
    digitalWrite(SolSinyalLed, LOW);
    digitalWrite(SagSinyalLed, LOW);
    digitalWrite(StopLed, LOW);
    digitalWrite(OnFarLed, LOW);
    digitalWrite(sinyallamba, LOW);
    
    Solsinyaldata = 0;
    Sagsinyaldata = 0;
    Stopdata = 0;
    OnFardata = 0;
    Dortludata = 0;
    sinyallambadata = 0;
    
    // Toggle durumlarını sıfırla
    for(int i = 0; i < 4; i++) sinyalToggle[i] = false;
    aktifSinyal = 0;
  }

  // TÜM DATALARI SERİ PORTA GÖNDER
  serialDataGonder();
  delay(50);
}

void sinyalKontrol() {
  unsigned long currentMillis = millis();
  
  // Buton kontrolleri - toggle mantığı
  if (digitalRead(DortluButon) == HIGH) {
    delay(50);
    if (digitalRead(DortluButon) == HIGH) {
      sinyalToggle[3] = !sinyalToggle[3];
      if (sinyalToggle[3]) {
        sinyalToggle[1] = false;
        sinyalToggle[2] = false;
      }
      while(digitalRead(DortluButon) == HIGH);
    }
  }
  
  if (digitalRead(SolSinyalButon) == HIGH) {
    delay(50);
    if (digitalRead(SolSinyalButon) == HIGH) {
      sinyalToggle[1] = !sinyalToggle[1];
      if (sinyalToggle[1]) {
        sinyalToggle[2] = false;
        sinyalToggle[3] = false;
      }
      while(digitalRead(SolSinyalButon) == HIGH);
    }
  }
  
  if (digitalRead(SagSinyalButon) == HIGH) {
    delay(50);
    if (digitalRead(SagSinyalButon) == HIGH) {
      sinyalToggle[2] = !sinyalToggle[2];
      if (sinyalToggle[2]) {
        sinyalToggle[1] = false;
        sinyalToggle[3] = false;
      }
      while(digitalRead(SagSinyalButon) == HIGH);
    }
  }

  // Aktif sinyal
  if (sinyalToggle[3]) aktifSinyal = 3;
  else if (sinyalToggle[1]) aktifSinyal = 1;
  else if (sinyalToggle[2]) aktifSinyal = 2;
  else aktifSinyal = 0;

  // Sinyal yanıp sönme
  if (currentMillis - previousSinyalMillis >= 200) {
    previousSinyalMillis = currentMillis;
    sinyalDurum = !sinyalDurum;
    
    switch (aktifSinyal) {
      case 1: // Sol sinyal
        digitalWrite(SolSinyalLed, sinyalDurum);
        digitalWrite(SagSinyalLed, LOW);
        Solsinyaldata = sinyalDurum;
        Sagsinyaldata = 0;
        Dortludata = 0;
        break;
      case 2: // Sağ sinyal
        digitalWrite(SagSinyalLed, sinyalDurum);
        digitalWrite(SolSinyalLed, LOW);
        Sagsinyaldata = sinyalDurum;
        Solsinyaldata = 0;
        Dortludata = 0;
        break;
      case 3: // Dörtlü sinyal
        digitalWrite(SolSinyalLed, sinyalDurum);
        digitalWrite(SagSinyalLed, sinyalDurum);
        Solsinyaldata = sinyalDurum;
        Sagsinyaldata = sinyalDurum;
        Dortludata = 1;
        break;
      default: // Sinyal yok
        digitalWrite(SolSinyalLed, LOW);
        digitalWrite(SagSinyalLed, LOW);
        Solsinyaldata = 0;
        Sagsinyaldata = 0;
        Dortludata = 0;
        break;
    }
  }
}

void stopKontrol() {
  Stopdata = (digitalRead(StopButon) == HIGH || digitalRead(frenswitch) == HIGH);
  digitalWrite(StopLed, Stopdata);
}

void farKontrol() {
  static unsigned long sonFarBasma = 0;
  if (digitalRead(OnFarButon) == HIGH && millis() - sonFarBasma > 300) {
    OnFardata = !OnFardata;
    digitalWrite(OnFarLed, OnFardata);
    sonFarBasma = millis();
    delay(300);
  }
}

void sinyallambaKontrol() {
  static unsigned long sonSinyalLambaBasma = 0;
  if (digitalRead(sinyallambabuton) == HIGH && millis() - sonSinyalLambaBasma > 300) {
    sinyallambadata = !sinyallambadata;
    digitalWrite(sinyallamba, sinyallambadata);
    sonSinyalLambaBasma = millis();
    delay(300);
  }
}

// MULTIMEDYA KONTROL - SADECE OKUMA, ÇIKIŞ YOK
void multimediaKontrol() {
  MultimediaSoldata = digitalRead(MultimediaSol);
  MultimediaSagdata = digitalRead(MultimediaSag);
  MultimediaYukaridata = digitalRead(MultimediaYukari);
  MultimediaAsagidata = digitalRead(MultimediaAsagi);
  MultimediaOkdata = digitalRead(MultimediaOk);
}

// VİTES KONTROL - SADECE OKUMA, ÇIKIŞ YOK
void vitesKontrol() {
  VitesDdata = digitalRead(VitesD);
  VitesPdata = digitalRead(VitesP);
  VitesRdata = digitalRead(VitesR);
}

void sensorOku() {
  // Sensör okumaları - örnek değerler, gerçek sensörleri bağlayacaksın
  BataryaVoltajdata = 48.5;
  AnlikAkimdata = 12.3;
  ToplamGucdata = BataryaVoltajdata * AnlikAkimdata;
  OrtamSicaklikdata = 25.0;
  SolMotorSicaklikdata = 45.0;
  SagMotorSicaklikdata = 47.0;
}

// ACİL DURUM KONTROL - SADECE OKUMA, ÇIKIŞ YOK
void acilDurumKontrol() {
  AcilDurumdata = digitalRead(AcilDurum);
}

// TÜM DATALARI SERİ PORTA GÖNDER - Raspberry Pi alacak
void serialDataGonder() {
  Serial.print(Solsinyaldata); Serial.print("/");
  Serial.print(Sagsinyaldata); Serial.print("/");
  Serial.print(Stopdata); Serial.print("/");
  Serial.print(OnFardata); Serial.print("/");
  Serial.print(Dortludata); Serial.print("/");
  Serial.print(sinyallambadata); Serial.print("/");
  Serial.print(EkranKontakdata); Serial.print("/");
  Serial.print(MotorKontakdata); Serial.print("/");
  Serial.print(VitesDdata); Serial.print("/");
  Serial.print(VitesPdata); Serial.print("/");
  Serial.print(VitesRdata); Serial.print("/");
  Serial.print(MultimediaSagdata); Serial.print("/");
  Serial.print(MultimediaSoldata); Serial.print("/");
  Serial.print(MultimediaYukaridata); Serial.print("/");
  Serial.print(MultimediaAsagidata); Serial.print("/");
  Serial.print(MultimediaOkdata); Serial.print("/");
  Serial.print(AcilDurumdata); Serial.print("/");
  Serial.print(BataryaVoltajdata); Serial.print("/");
  Serial.print(AnlikAkimdata); Serial.print("/");
  Serial.print(ToplamGucdata); Serial.print("/");
  Serial.print(OrtamSicaklikdata); Serial.print("/");
  Serial.print(SolMotorSicaklikdata); Serial.print("/");
  Serial.print(SagMotorSicaklikdata); Serial.print("/");
  Serial.println();
}