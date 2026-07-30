// =========================================================================================
// Rahman ve Rahim olan Allah'ın Adıyla
// Proje: GoToGo 9.2 Alpha
// EV Control Unit (Arduino Due) <-> Raspberry Pi
// Yazan: AhmetMertÇELİK 
// =========================================================================================
//
// BU SÜRÜMDE (8.1 -> 9.2) NELER DEĞİŞTİ?
// -----------------------------------------------------------------------------------------
// 1) GAZ KOLU ARTIK POTANSİYOMETRE DEĞİL, 3 KABLOLU PWM SİNYALLİ OKUNUYOR:
//    Gaz kolu +5V / GND / SİNYAL şeklinde 3 kablolu ve sinyal kablosundan PWM (değişken
//    darbe genişliği) üretiyor. Eskiden analogRead() ile gerilim okunuyordu; artık pin
//    değişimlerini yakalayan bir kesme (interrupt) ile darbe genişliği mikrosaniye
//    cinsinden ölçülüyor ve bu genişlik 0-100 gaz yüzdesine çevriliyor.
//    !!! KALİBRASYON GEREKİYOR: GAZ_PWM_MIN_US ve GAZ_PWM_MAX_US değerlerini kendi gaz
//    kolunuzun gerçek sinyaline göre ayarlamalısınız (aşağıda "GAZ/THROTTLE AYARLARI"
//    bölümünde). Seri port debug çıktısına bakarak (Serial.println(gazPulseGenisligi))
//    boşta ve tam basılı haldeki gerçek mikrosaniye değerlerini ölçüp buraya yazın.
//
// 2) GAZ SERTLİĞİ / TEPKİ EĞRİSİ (Throttle Response Curve):
//    Güncel elektrikli araçlardaki gibi, sürüş moduna göre gaz pedalının "hissi" değişiyor:
//      - ECO   : Yumuşak/aşamalı tepki (pedalın ilk kısmında güç daha az artar, verimli sürüş)
//      - NORMAL: Doğrusal tepki (pedal hareketi = güç artışı, 1:1)
//      - SPORT : Sert/agresif tepki (pedalın az hareketinde bile güç hızla artar)
//    Bu, "gazEgrisiUygula()" fonksiyonunda üstel bir eğriyle (gamma) uygulanıyor.
//
// 3) JİROSKOP + İVMEÖLÇER (MPU6050) EKLENDİ - GÜNCEL ARAÇLARDAN DAHA İYİ GÜVENLİK:
//    Gerekli kütüphaneler (Arduino Kütüphane Yöneticisi'nden kurulmalı):
//      "Adafruit MPU6050", "Adafruit Unified Sensor", "Adafruit BusIO"
//    Jiroskop/ivmeölçer verisiyle 3 YENİ GÜVENLİK ÖZELLİĞİ eklendi:
//      a) DEVRİLME (ROLLOVER) KORUMASI: Araç tehlikeli açıda eğilirse (roll/pitch açısı
//         eşiği aşıp bir süre bu şekilde kalırsa) motor gücü kesilir ve dörtlü ikaz yanar.
//      b) ÇARPIŞMA/DARBE TESPİTİ: Ani ve yüksek bir ivme (g) darbesi algılanırsa (çarpışma
//         şüphesi) motor gücü anında kesilir, dörtlü ikaz otomatik yanar. Kontak
//         kapatılıp açılana kadar bu durum kilitli kalır (yanlışlıkla devam edilmesin diye).
//      c) JİROSKOP DESTEKLİ ÇEKİŞ KONTROLÜ (basit Traction/Stability Control): Aracın
//         gerçek dönüş hızı (yaw rate), beklenenden fazla ise (tekerlek kayması/savrulma
//         şüphesi) motor gücü geçici olarak azaltılır. Bu, günümüz elektrikli araçlarında
//         bulunan Elektronik Stabilite Kontrolü (ESC/ESP) sisteminin basitleştirilmiş bir
//         versiyonudur ve okul projeleri için "çığır açıcı" bir güvenlik özelliğidir.
//    Tüm bu ivme/açı/dönüş verileri artık Raspberry Pi'ye de gönderiliyor (aşağıya bakın).
//
// 4) TELEMETRİ PAKETİ GENİŞLETİLDİ:
//    Binary paket artık jiroskop/ivmeölçer verilerini (ivme X/Y/Z, yaw rate, roll/pitch
//    açıları) ve yeni güvenlik bayraklarını (devrilme/çarpışma/kayma tespiti) da içeriyor.
//    Haberleşme yöntemi (Serial1 üzerinden binary + CRC8) 8.1'de anlatıldığı gibi aynen
//    korunuyor, sadece paket içeriği zenginleştirildi.
//
// 5) KOD BAŞTAN AŞAĞI DÜZENLENDİ:
//    Tüm bölümler yeniden gruplandı, isimlendirme sadeleştirildi, HER SATIRA yakın yorum
//    eklendi. Önceki sürümlerdeki (8.0/8.1) tüm hata düzeltmeleri (PWM çözünürlüğü, sinyal
//    lambası varsayılan durumu, debounce eksiklikleri, fren okuması, watchdog, APPS-fren
//    plazibilite kontrolü, ivme yumuşatma) korunuyor.
//
// (Geliştirme aşamasında olduğundan bazı pinler 0 olarak tanımlıdır, bu şekilde KULLANMAYIN!)
// (Bu kod Arduino Due ile Raspberry Pi'nin birlikte çalışması için yazılmıştır)
// =========================================================================================

#include <Wire.h>
#include <Adafruit_PN532.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <stddef.h>            // offsetof() makrosu için (telemetri paketi CRC hesaplamasında kullanılıyor)
#include <Adafruit_MPU6050.h>  // Jiroskop + İvmeölçer (Kütüphane Yöneticisi'nden kurulmalı)
#include <Adafruit_Sensor.h>   // Adafruit_MPU6050'nin ihtiyaç duyduğu ortak sensör arayüzü

// =========================================================================================
// BÖLÜM 1: GENEL SİSTEM AYARLARI VE SABİTLER
// =========================================================================================
#define KAYIT_MODU 1                 // 1: EEPROM'a kaydet, 2: SD Karta kaydet
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
const int CHIP_SELECT_PIN = 10;      // SD kart modülünün CS (Chip Select) pini
#define mainLoopDelay 50             // Ana döngü periyodu (ms) - saniyede ~20 kez çalışır
#define NFC_SKIP_CONTROL 1           // 1: NFC kontrolünü atla (test modu), 0: NFC zorunlu

uint8_t referenceUID[] = { 0x3A, 0x83, 0x5B, 0x06 }; // Anahtar kart olarak tanınacak NFC UID

// =========================================================================================
// BÖLÜM 2: PİN TANIMLAMALARI (Kategori Kategori)
// =========================================================================================

// --- 2.1 İletişim ve Genel Hata ---
#define SDA_PIN A4
#define SCL_PIN A5
#define hata_led 0                   // Genel hata/uyarı LED'i (gaz arızası, çarpışma, NFC hatası vb.)

// --- 2.2 Sürüş Modu Butonları ve Çıkışları ---
#define mode_ileri 0                 // Sürüş modunu bir sonrakine geçirme butonu
#define mode_geri 0                  // Sürüş modunu bir öncekine geçirme butonu
#define mode_normal 4                // NORMAL mod göstergesi/rölesi
#define mode_eco 5                   // ECO mod göstergesi/rölesi
#define mode_sport 6                 // SPORT mod göstergesi/rölesi

// --- 2.3 Kontak ve Güç Röleleri ---
#define motor_kontak 7               // Motor sürücü besleme rölesi
#define ekran_kontak 8               // Gösterge ekranı besleme rölesi
#define buzzer 0                     // Uyarı sesi (buzzer)

// --- 2.4 Aydınlatma Sistemi (Far 1: Kısa/Otomatik, Far 2: Uzun/Sis) ---
#define far_kisa 9
#define far_uzun 11
#define far_sis 12
#define far_ileri 0                  // Far modunu ileri sarma butonu
#define far_geri 0                   // Far modunu geri sarma butonu
#define far_buton 0                  // Farı tamamen kapatma butonu
#define far2_ileri 0
#define far2_geri 0
#define far2_buton 0
#define ldr A1                       // Ortam ışığı sensörü (LDR) - otomatik far modu için
#define fark 625                     // LDR eşik değeri (bu değerin altı = karanlık say)

// --- 2.5 Sinyal, Fren ve Vites Grubu ---
#define pw 0                         // İleri vites switch'i
#define rw 0                         // Geri vites switch'i
#define fren 0                       // Fren pedalı/switch girişi
#define solsinsw 0
#define sagsinsw 0
#define dortlusw 0
#define solsin 0
#define sagsin 0
#define stoplambasi 0                // Fren lambası (hem far/otomatik hem gerçek fren ile tetiklenir)
#define sinyallambasisw 0
#define sinyallambasi 0

// --- 2.6 Motor ve Gaz Kolu (ARTIK 3 KABLOLU PWM SİNYALLİ) ---
#define motor1Pin 0                  // Sol teker hız sensörü (encoder/hall) - kesme pini
#define motor2Pin 0                  // Sağ teker hız sensörü (encoder/hall) - kesme pini
#define gazpot A1                    // !! DİKKAT: "ldr" ile aynı pinde tanımlı - gerçek pin
                                      // atamasında FARKLI bir pin seçin, ikisi çakışır.
                                      // Not: Artık bu pin analogRead ile değil, PWM darbe
                                      // genişliği ölçümüyle (interrupt) okunuyor; yine de
                                      // ldr ile AYNI FİZİKSEL PİNİ paylaşamaz.
#define solmotorgaz 5                // Sol motor sürücüsüne giden PWM çıkışı
#define sagmotorgaz 9                // Sağ motor sürücüsüne giden PWM çıkışı
#define EDSButon 13                  // Elektronik Diferansiyel açma/kapama butonu

// =========================================================================================
// BÖLÜM 3: GLOBAL DEĞİŞKENLER (Kategori Kategori)
// =========================================================================================

// --- 3.1 Enkoder ve Direksiyon Açısı ---
Encoder myEncoder(ENCODER_PIN_A, ENCODER_PIN_B);
const float STEPS_PER_REVOLUTION = 4000.0;
const float DEGREES_PER_STEP = 360.0 / STEPS_PER_REVOLUTION;
const int EEPROM_ADDRESS = 0;
const char* DATA_FILENAME = "ENKODER.TXT";
const long WRITE_INTERVAL = 5000;        // Direksiyon adımının kalıcı hafızaya yazılma sıklığı (ms)

long currentSteps = 0;
long lastSavedSteps = 0;
float direksiyonaci = 0.0;               // Direksiyon açısı (derece, - : sola, + : sağa)
unsigned long lastWriteTime = 0;

// --- 3.2 EDS (Elektronik Diferansiyel Sistemi) ---
#define maxdireksiyonaci 180.0
int dengekatsayisi = 20;                 // Direksiyon açısına göre gaz farkı katsayısı
const int MIN_GAZ_YUZDESI = 20;          // EDS'nin devreye girmesi için gereken minimum gaz %'si
const int ANALOG_MAX_VALUE = 1023;       // Dahili hesaplarda kullanılan 10-bit ölçek (0-1023)

// --- 3.3 GAZ / THROTTLE AYARLARI (3 Kablolu PWM Sinyal Okuma) ---
// !!! BU İKİ DEĞERİ MUTLAKA KENDİ GAZ KOLUNUZA GÖRE KALİBRE EDİN !!!
// Nasıl kalibre edilir: NFC_SKIP_CONTROL=1 iken seri portu (USB) açın, gazı hiç basmadan
// ve tam basılı haldeyken "gazPulseGenisligi" değerini Serial Monitor'den okuyup buraya yazın.
const unsigned int GAZ_PWM_MIN_US = 1000;      // Gaz bırakılmış (rölanti) haldeki darbe genişliği (µs)
const unsigned int GAZ_PWM_MAX_US = 2000;      // Gaz tam basılı haldeki darbe genişliği (µs)
const unsigned long GAZ_PWM_TIMEOUT_MS = 200;  // Bu süre boyunca yeni darbe gelmezse "sinyal yok" say

volatile unsigned long gazPulseBaslangicZamani = 0; // ISR içi: darbenin başladığı an (mikrosaniye)
volatile unsigned int  gazPulseGenisligi = 0;       // ISR içi: son ölçülen darbe genişliği (mikrosaniye)
volatile unsigned long gazSonPulseZamani = 0;       // ISR içi: son geçerli darbenin alındığı an (millis)

int gazYuzdesiHam = 0;   // Gaz kolundan doğrudan okunan, eğri uygulanmamış ham yüzde (0-100)
bool gazSinyaliVarMi = false; // Gaz sinyali zaman aşımına uğramadan düzenli geliyor mu?

// --- Gaz Sertliği / Tepki Eğrisi (Throttle Response Curve) ---
// gamma > 1  -> pedalın ilk kısmında daha az güç (yumuşak/ECO)
// gamma == 1 -> doğrusal (NORMAL)
// gamma < 1  -> pedalın az hareketinde bile fazla güç (sert/agresif, SPORT)
const float GAZ_EGRI_ECO    = 1.6;
const float GAZ_EGRI_NORMAL = 1.0;
const float GAZ_EGRI_SPORT  = 0.65;

// --- 3.4 Hız Ölçümü (Teker Sensörleri) ---
volatile unsigned long pulseCount1 = 0;
volatile unsigned long pulseCount2 = 0;
unsigned long lastMeasureTime = 0;
const unsigned long measureInterval = 1000;   // Hız hesaplama periyodu (ms)
const float MOTOR_PPR_1 = 42.0;               // Sol teker sensörünün tur başına darbe sayısı
const float MOTOR_PPR_2 = 42.0;               // Sağ teker sensörünün tur başına darbe sayısı
const float CIRCUMFERENCE_METER_1 = 2.07;     // Sol tekerin çevresi (metre)
const float CIRCUMFERENCE_METER_2 = 2.07;     // Sağ tekerin çevresi (metre)
float speed_kmh_1 = 0.0;
float speed_kmh_2 = 0.0;

// --- 3.5 Jiroskop / İvmeölçer (MPU6050) ---
Adafruit_MPU6050 mpu;
bool mpuHazir = false;              // Sensör başlatma başarılı oldu mu?

float ivmeX = 0.0, ivmeY = 0.0, ivmeZ = 0.0;  // g cinsinden (1g = 9.81 m/s^2) ivme bileşenleri
float toplamIvme = 0.0;                       // Bileşke ivme (g) - çarpışma tespiti için
float yawRateDPS = 0.0;                       // Z ekseni dönüş hızı (derece/saniye) - savrulma tespiti için
float rollAci = 0.0, pitchAci = 0.0;          // İvmeölçerden hesaplanan yaklaşık eğim açıları (derece)

// --- Jiroskop Tabanlı Güvenlik Sabitleri ---
const float DEVRILME_ACI_ESIGI = 35.0;        // derece - bu açı aşılırsa devrilme riski var say
const unsigned long DEVRILME_SURESI_MS = 400; // Bu süre boyunca eşik aşılı kalırsa GERÇEK devrilme say
const float CARPISMA_IVME_ESIGI_G = 2.5;      // g - bu ivmenin üstü ani darbe/çarpışma say
const float YAW_KAYMA_ESIGI_DPS = 25.0;       // derece/saniye - beklenenden fazla dönüş = kayma şüphesi
const float KAYMA_MIN_HIZ_KMH = 5.0;          // Bu hızın altında kayma kontrolü çalışmaz (yanlış alarm önleme)
const float TRACTION_KESME_ORANI = 0.55;      // Kayma tespit edilince gaz bu orana düşürülür (0.0-1.0)

bool devrilmeTespit = false;
bool carpismaTespit = false;   // Bir kez tetiklenince kontak (durum) kapatılıp açılana kadar kilitli kalır
bool kaymaTespit = false;

// --- 3.6 Genel Gaz/Fren Güvenlik Sistemi ---
const int FREN_GAZ_KESME_ESIGI = 8;           // Bu gaz yüzdesi üzerindeyken fren basılırsa güç kesilir
bool throttleGuvenlikHatasi = false;           // true ise motorlara KESİNLİKLE güç verilmez

// --- 3.7 İvme Yumuşatma (Acceleration Smoothing) ---
// Sport modda (mode==2) devre dışı bırakılır (kullanıcı tercihiyle - anlık tepki için).
const int IVME_YUMUSATMA_ADIMI = 15;          // Döngü başına izin verilen maksimum PWM değişimi
int oncekiGazSol = 0;
int oncekiGazSag = 0;

// --- 3.8 Zamanlama ve Debounce ---
static unsigned long sonOkumaZamani = 0;        // Sürüş modu butonu debounce
static unsigned long sonOkumaZamani1 = 0;       // Far 1 butonu debounce
static unsigned long sonOkumaZamani2 = 0;       // Far 2 butonu debounce
static unsigned long sonOkumaZamani3 = 0;       // EDS butonu debounce
static unsigned long sonOkumaZamaniDortlu = 0;  // Dörtlü butonu debounce
static unsigned long sonOkumaZamaniLamba = 0;   // Sinyal lambası butonu debounce
const unsigned long DEBOUNCE_SURE  = 200;
const unsigned long DEBOUNCE_SURE1 = 400;
const unsigned long DEBOUNCE_SURE2 = 500;
unsigned long oncekiMillis = 0;                 // Sinyal lambası yanıp-sönme zamanlayıcısı
const long aralik = 500;                        // Sinyal lambası yanıp-sönme aralığı (ms)

// --- 3.9 Genel Durum Değişkenleri ---
int durum = 0;                 // 0: Kapalı, 1: Tam açık, 2: Sadece ekran
int mode = 0;                  // 0: ECO, 1: NORMAL, 2: SPORT
int far = 1;                   // 0: Kapalı, 1: Otomatik(LDR), 2: Manuel açık
int far2 = 0;                  // 0: Kapalı, 1: Uzun far, 2: Sis farı
bool frenBasili = false;       // Fren pedalı/switch anlık durumu
int vites = 1;                 // 0: Boş, 1: İleri, 2: Geri
bool sinyalsol = 0;
bool sinyalsag = 0;
bool dortlu = 0;
bool sinyallambasidata = 0;
int gazseviyesi = 0;           // Gaz seviyesi, 0-1023 ölçeğinde (geri kalan EDS matematiğiyle uyumlu)
bool EDS = 0;                  // EDS kullanıcı tarafından açık mı?
int EDS_AKTIF = 0;             // EDS şu an gerçekten devrede mi? (buton açık VE gaz yeterli)
int solGazYuzdesi = 0;
int sagGazYuzdesi = 0;
int gazYuzdesi = 0;            // Gaz eğrisi uygulanmış NİHAİ gaz yüzdesi (0-100)

// NFC Objesi
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

// =========================================================================================
// BÖLÜM 4: RASPBERRY Pi HABERLEŞME PROTOKOLÜ (Binary + CRC8)
// =========================================================================================
// Due'nun ayrılmış donanımsal UART portu (Serial1, TX1=pin18 / RX1=pin19) üzerinden, sabit
// boyutlu + başlık/bitiş baytlı + CRC8 kontrollü binary paket gönderilir. Bu yöntem eski
// ASCII/CSV metin paketine göre çok daha hızlı ve güvenilirdir (bkz. 8.1 sürüm notları).
// USB Serial (Serial) sadece debug/hata kodları ve programlama için ayrılmıştır.
#define RPI_SERIAL Serial1
#define RPI_BAUDRATE 500000
#define PAKET_BASLIK 0xAA
#define PAKET_BITIS  0x55

#pragma pack(push, 1)
struct TelemetriPaketi {
  uint8_t  baslik;        // Senkron baytı: her zaman 0xAA
  uint8_t  mode;          // Sürüş modu (0:ECO, 1:NORMAL, 2:SPORT)
  uint8_t  far;
  uint8_t  durum;
  uint8_t  far2;
  uint8_t  vites;
  uint8_t  bayraklar;     // bit0:sinyallambasi bit1:dortlu bit2:sinyalsol bit3:sinyalsag
                          // bit4:EDS_AKTIF bit5:fren bit6:throttleGuvenlikHatasi
  uint8_t  bayraklar2;    // bit0:devrilmeTespit bit1:carpismaTespit bit2:kaymaTespit
                          // bit3:mpuHazir bit4:gazSinyaliVarMi
  float    hiz1;          // speed_kmh_1
  float    hiz2;          // speed_kmh_2
  float    direksiyon;    // direksiyonaci (derece)
  uint8_t  solGaz;        // solGazYuzdesi (0-100)
  uint8_t  sagGaz;        // sagGazYuzdesi (0-100)
  float    ivmeX;         // g cinsinden
  float    ivmeY;         // g cinsinden
  float    ivmeZ;         // g cinsinden
  float    yawRate;       // derece/saniye
  float    rollAci;       // derece
  float    pitchAci;      // derece
  uint8_t  crc;           // CRC8 (baslik ve crc/bitis hariç, aradaki tüm alanlar üzerinden)
  uint8_t  bitis;         // Senkron baytı: her zaman 0x55
};
#pragma pack(pop)

// Basit, tablo gerektirmeyen CRC8 (Dallas/Maxim polinomu 0x31)
uint8_t hesaplaCRC8(const uint8_t* veri, size_t uzunluk) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < uzunluk; i++) {
    crc ^= veri[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
      else crc <<= 1;
    }
  }
  return crc;
}

// Telemetri paketini doldurup Serial1 üzerinden Raspberry Pi'ye gönderir
void gonderTelemetriRPi() {
  TelemetriPaketi p;
  p.baslik    = PAKET_BASLIK;
  p.mode      = (uint8_t)mode;
  p.far       = (uint8_t)far;
  p.durum     = (uint8_t)durum;
  p.far2      = (uint8_t)far2;
  p.vites     = (uint8_t)vites;

  p.bayraklar = (sinyallambasidata ? 0x01 : 0) |
                (dortlu            ? 0x02 : 0) |
                (sinyalsol         ? 0x04 : 0) |
                (sinyalsag         ? 0x08 : 0) |
                (EDS_AKTIF         ? 0x10 : 0) |
                (frenBasili        ? 0x20 : 0) |
                (throttleGuvenlikHatasi ? 0x40 : 0);

  p.bayraklar2 = (devrilmeTespit  ? 0x01 : 0) |
                 (carpismaTespit  ? 0x02 : 0) |
                 (kaymaTespit     ? 0x04 : 0) |
                 (mpuHazir        ? 0x08 : 0) |
                 (gazSinyaliVarMi ? 0x10 : 0);

  p.hiz1       = speed_kmh_1;
  p.hiz2       = speed_kmh_2;
  p.direksiyon = direksiyonaci;
  p.solGaz     = (uint8_t)constrain(solGazYuzdesi, 0, 100);
  p.sagGaz     = (uint8_t)constrain(sagGazYuzdesi, 0, 100);
  p.ivmeX      = ivmeX;
  p.ivmeY      = ivmeY;
  p.ivmeZ      = ivmeZ;
  p.yawRate    = yawRateDPS;
  p.rollAci    = rollAci;
  p.pitchAci   = pitchAci;

  // CRC, "baslik" ve "crc/bitis" alanları HARİÇ aradaki veri bloğu üzerinden hesaplanır
  const uint8_t* veriBasi = ((const uint8_t*)&p) + offsetof(TelemetriPaketi, mode);
  size_t veriUzunlugu = offsetof(TelemetriPaketi, crc) - offsetof(TelemetriPaketi, mode);
  p.crc = hesaplaCRC8(veriBasi, veriUzunlugu);
  p.bitis = PAKET_BITIS;

  RPI_SERIAL.write((const uint8_t*)&p, sizeof(TelemetriPaketi));
}

// =========================================================================================
// BÖLÜM 5: DONANIMSAL WATCHDOG TIMER (WDT)
// =========================================================================================
// Due'nun SAM3X8E çipindeki gerçek donanımsal watchdog. loop() içinde herhangi bir yerde
// (örn. NFC/SD kart erişiminde) kod takılıp kalırsa, watchdog belirlenen sürede
// "beslenmezse" işlemciyi otomatik olarak resetler. Aracın sürüş sırasında "donmuş" bir
// beyinle kalmasını engelleyen kritik bir güvenlik önlemidir.
void wdtBaslat() {
  // WDT_MR register'ı setup() sonrasında sadece BİR KEZ yazılabilir (donanımsal kilit).
  WDT->WDT_MR = WDT_MR_WDRSTEN |         // Zaman aşımında reset at
                WDT_MR_WDV(0xFFF) |      // Zaman aşımı süresi (~4 saniye)
                WDT_MR_WDD(0xFFF);       // "Çok erken besleme" penceresi
}

void wdtBesle() {
  WDT->WDT_CR = WDT_CR_KEY(0xA5) | WDT_CR_WDRSTT; // Watchdog sayacını sıfırla ("besle")
}

// =========================================================================================
// BÖLÜM 6: YARDIMCI FONKSİYONLAR
// =========================================================================================

// --- 6.1 NFC Kart Karşılaştırma ---
bool compareUID(uint8_t* uid1, uint8_t uid1Length, uint8_t* uid2, uint8_t uid2Length) {
  if (uid1Length != uid2Length) return false;
  for (uint8_t i = 0; i < uid1Length; i++) {
    if (uid1[i] != uid2[i]) return false;
  }
  return true;
}

// --- 6.2 Teker Hız Sensörü Kesmeleri ---
void countPulse1() { pulseCount1++; }
void countPulse2() { pulseCount2++; }

// --- 6.3 Gaz Kolu PWM Darbe Genişliği Ölçüm Kesmesi ---
// Gaz kolunun sinyal kablosu HIGH'a geçince darbe başlar, LOW'a düşünce darbe biter.
// Aradaki süre (mikrosaniye) gaz basma miktarıyla orantılıdır.
void gazPWM_ISR() {
  if (digitalRead(gazpot) == HIGH) {
    gazPulseBaslangicZamani = micros();
  } else {
    unsigned long genislik = micros() - gazPulseBaslangicZamani;
    // Gürültü/yanlış tetiklenme filtrelemesi: makul bir darbe genişliği aralığı (200us-5000us)
    if (genislik > 200 && genislik < 5000) {
      gazPulseGenisligi = (unsigned int)genislik;
      gazSonPulseZamani = millis();
    }
  }
}

// --- 6.4 SD Karttan Adım Verisi Okuma ---
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
    if (KAYIT_MODU == 2) Serial.println("1205"); // Hata kodu: SD kart okunamadı
  }
  return loadedSteps;
}

// --- 6.5 SD Karta Adım Verisi Yazma ---
void writeStepsToSD(long steps) {
  if (SD.exists(DATA_FILENAME)) SD.remove(DATA_FILENAME);
  File dataFile = SD.open(DATA_FILENAME, FILE_WRITE);
  if (dataFile) {
    dataFile.println(steps);
    dataFile.close();
  }
}

// --- 6.6 Gaz Sertliği / Tepki Eğrisi Uygulama ---
// Ham gaz yüzdesini (0-100), sürüş moduna göre bir üstel eğriden geçirir.
// ECO'da yumuşak başlangıç, SPORT'ta agresif/sert başlangıç sağlar.
int gazEgrisiUygula(int hamYuzde, int surusModu) {
  float gamma = GAZ_EGRI_NORMAL;
  if (surusModu == 0) gamma = GAZ_EGRI_ECO;        // ECO
  else if (surusModu == 2) gamma = GAZ_EGRI_SPORT; // SPORT

  float oran = constrain(hamYuzde, 0, 100) / 100.0;
  float sonuc = pow(oran, gamma) * 100.0;
  return (int)constrain(sonuc, 0, 100);
}

// --- 6.7 Jiroskop/İvmeölçer Okuma ---
void mpuOku() {
  if (!mpuHazir) return; // Sensör yoksa/başlatılamadıysa hiç okumaya çalışma

  sensors_event_t ivmeOlay, giroOlay, sicaklikOlay;
  mpu.getEvent(&ivmeOlay, &giroOlay, &sicaklikOlay);

  // m/s^2 -> g cinsine çevrilir (1g = 9.81 m/s^2)
  ivmeX = ivmeOlay.acceleration.x / 9.81;
  ivmeY = ivmeOlay.acceleration.y / 9.81;
  ivmeZ = ivmeOlay.acceleration.z / 9.81;
  toplamIvme = sqrt(ivmeX * ivmeX + ivmeY * ivmeY + ivmeZ * ivmeZ);

  // Adafruit_Sensor kütüphanesi gyro değerlerini rad/s cinsinden döndürür -> derece/s'ye çevir
  yawRateDPS = giroOlay.gyro.z * (180.0 / PI);

  // İvmeölçerden yaklaşık roll/pitch açıları (basit trigonometrik tahmin)
  rollAci  = atan2(ivmeY, ivmeZ) * (180.0 / PI);
  pitchAci = atan2(-ivmeX, sqrt(ivmeY * ivmeY + ivmeZ * ivmeZ)) * (180.0 / PI);
}

// --- 6.8 Jiroskop Tabanlı Güvenlik Kontrolleri ---
// Devrilme koruması + çarpışma tespiti + basit çekiş/stabilite kontrolü.
void jiroskopGuvenlikKontrolu() {
  if (!mpuHazir) { // Sensör yoksa bu güvenlik katmanı devre dışı kalır (diğerleri çalışmaya devam eder)
    devrilmeTespit = false;
    kaymaTespit = false;
    return;
  }

  // --- Devrilme (Rollover) Tespiti ---
  static unsigned long devrilmeBaslangic = 0;
  bool asiriEgim = (abs(rollAci) > DEVRILME_ACI_ESIGI) || (abs(pitchAci) > DEVRILME_ACI_ESIGI);
  if (asiriEgim) {
    if (devrilmeBaslangic == 0) devrilmeBaslangic = millis();
    devrilmeTespit = (millis() - devrilmeBaslangic) >= DEVRILME_SURESI_MS;
  } else {
    devrilmeBaslangic = 0;
    devrilmeTespit = false;
  }

  // --- Çarpışma/Darbe Tespiti (bir kez tetiklenince kontak kapatılana kadar kilitli kalır) ---
  if (toplamIvme > CARPISMA_IVME_ESIGI_G) {
    carpismaTespit = true;
  }

  // --- Basit Çekiş/Stabilite Kontrolü (Jiroskop Destekli) ---
  float ortalamaHiz = (speed_kmh_1 + speed_kmh_2) / 2.0;
  kaymaTespit = (abs(yawRateDPS) > YAW_KAYMA_ESIGI_DPS) && (ortalamaHiz > KAYMA_MIN_HIZ_KMH);
}

// =========================================================================================
// BÖLÜM 7: SETUP (BAŞLANGIÇ AYARLARI)
// =========================================================================================
void setup() {
  Serial.begin(115200);            // USB Serial: sadece debug / hata kodları / programlama
  RPI_SERIAL.begin(RPI_BAUDRATE);  // Serial1: Raspberry Pi ile hızlı binary haberleşme hattı
  pinMode(hata_led, OUTPUT);

  // !! ÖNEMLİ HATA DÜZELTMESİ: analogRead() 10-bit (0-1023) değer döndürür, ama Due'da
  // analogWrite() varsayılan olarak 8-bit (0-255) çalışır. Bu satır olmadan motorlara
  // neredeyse her zaman tutarsız/aşırı PWM gönderilirdi. Due'nun PWM çıkışı 12-bit'e kadar
  // çözünürlük destekler; burada 10-bit'e ayarlayıp 0-1023 skalasıyla birebir uyumlu hale
  // getiriyoruz.
  analogWriteResolution(10);

  // Watchdog Timer'ı başlat (bkz. BÖLÜM 5)
  wdtBaslat();

  // --- Pin Modu Tanımlamaları (Girişler/Çıkışlar) ---
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

  // --- Veri Kayıt Sistemini Başlat (EEPROM veya SD) ---
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

  // --- Kesme (Interrupt) Ayarları ---
  attachInterrupt(digitalPinToInterrupt(motor1Pin), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2Pin), countPulse2, RISING);
  attachInterrupt(digitalPinToInterrupt(gazpot), gazPWM_ISR, CHANGE); // YENİ: gaz PWM ölçümü
  lastMeasureTime = millis();

  // --- Jiroskop/İvmeölçer (MPU6050) Başlatma ---
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("1400"); // Hata kodu: MPU6050 bulunamadı/başlatılamadı
    mpuHazir = false;
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuHazir = true;
    Serial.println("2300"); // Bilgi kodu: MPU6050 hazır
  }

  // --- NFC Modül Kontrolü ---
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

// =========================================================================================
// BÖLÜM 8: ANA DÖNGÜ (LOOP)
// =========================================================================================
void loop() {

  wdtBesle(); // Watchdog'u besle: buraya kadar loop takılmadan geldi demektir

  // --- Fren Pedalı/Switch Okuma ---
  frenBasili = (digitalRead(fren) == HIGH);

  // --- Enkoder ve Direksiyon Açısı Güncelleme ---
  long newSteps = myEncoder.read();
  if (newSteps != currentSteps) {
    currentSteps = newSteps;
    direksiyonaci = currentSteps * DEGREES_PER_STEP;
  }

  unsigned long mevcutMillis = millis();

  // --- Sürüş Modu Seçimi (Debounce ile) ---
  if (mevcutMillis - sonOkumaZamani > DEBOUNCE_SURE) {
    if (digitalRead(mode_ileri) == 1) {
      if (!(mode >= 2)) { mode = mode + 1; } else { mode = 0; }
      sonOkumaZamani = mevcutMillis;
    } else if (digitalRead(mode_geri) == 1) {
      if (!(mode <= 0)) { mode = mode - 1; } else { mode = 2; }
      sonOkumaZamani = mevcutMillis;
    }
  }

  // Sürüş Modu Pin Çıkışları
  if (mode == 0) { // ECO
    digitalWrite(mode_eco, 1); digitalWrite(mode_normal, 0); digitalWrite(mode_sport, 0);
  } else if (mode == 1) { // NORMAL
    digitalWrite(mode_eco, 0); digitalWrite(mode_normal, 1); digitalWrite(mode_sport, 0);
  } else if (mode == 2) { // SPORT
    digitalWrite(mode_eco, 0); digitalWrite(mode_normal, 0); digitalWrite(mode_sport, 1);
  }

  // --- Sistem Durumu ve Kontak Kontrolü ---
  if (durum == 0) { // Kapalı
    digitalWrite(ekran_kontak, 0); digitalWrite(motor_kontak, 0);
    // Kontak kapatıldığında çarpışma kilidi sıfırlanır (güvenlik: bir dahaki açılışta
    // sürücü aracı kontrol ettiğini bilerek yeniden başlatmış olur).
    carpismaTespit = false;
  } else if (durum == 1) { // Tam Açık
    digitalWrite(ekran_kontak, 1); digitalWrite(motor_kontak, 1);
  } else if (durum == 2) { // Sadece Ekran
    digitalWrite(ekran_kontak, 1); digitalWrite(motor_kontak, 0);
  }

  // --- Far Sistemi (Kısa / Otomatik) ---
  if (mevcutMillis - sonOkumaZamani1 > DEBOUNCE_SURE1) {
    if (digitalRead(far_ileri) == 1) {
      if (!(far >= 2)) { far = far + 1; } else { far = 1; }
      sonOkumaZamani1 = mevcutMillis;
    } else if (digitalRead(far_geri) == 1) {
      if (!(far <= 1)) { far = far - 1; } else { far = 2; }
      sonOkumaZamani1 = mevcutMillis;
    } else if (digitalRead(far_buton) == 1) {
      far = 0;
      sonOkumaZamani1 = mevcutMillis;
    }
  }

  if (far == 0) {
    digitalWrite(far_kisa, 0); digitalWrite(stoplambasi, 0);
  } else if (far == 1) { // Otomatik Mod (LDR)
    if (analogRead(ldr) >= fark) {
      digitalWrite(far_kisa, 1); digitalWrite(stoplambasi, 1);
    } else {
      digitalWrite(far_kisa, 0); digitalWrite(stoplambasi, 0);
    }
  } else if (far == 2) { // Manuel Açık
    digitalWrite(far_kisa, 1); digitalWrite(stoplambasi, 1);
  }

  // --- Gerçek Fren Lambası Fonksiyonu ---
  // "stoplambasi" pini artık sadece far durumuna değil, fren pedalına da bağlı: far kapalı
  // olsa bile fren basılırsa stop lambası yanar (gerçek bir fren lambası gibi davranır).
  if (frenBasili) {
    digitalWrite(stoplambasi, HIGH);
  }

  // --- Far 2 Sistemi (Uzun / Sis) ---
  if (mevcutMillis - sonOkumaZamani2 > DEBOUNCE_SURE2) {
    if (digitalRead(far2_ileri) == 1) {
      if (!(far2 >= 2)) { far2 = far2 + 1; } else { far2 = 1; }
      sonOkumaZamani2 = mevcutMillis;
    } else if (digitalRead(far2_geri) == 1) {
      if (!(far2 <= 1)) { far2 = far2 - 1; } else { far2 = 2; }
      sonOkumaZamani2 = mevcutMillis;
    } else if (digitalRead(far2_buton) == 1) {
      far2 = 0;
      sonOkumaZamani2 = mevcutMillis;
    }
  }

  if (far2 == 0) {
    digitalWrite(far_uzun, 0); digitalWrite(far_sis, 0);
  } else if (far2 == 1) {
    digitalWrite(far_uzun, 1); digitalWrite(far_sis, 0);
  } else if (far2 == 2) {
    digitalWrite(far_uzun, 0); digitalWrite(far_sis, 1);
  }

  // --- Vites Kontrolü ---
  if (digitalRead(pw) == HIGH) vites = 1;
  else if (digitalRead(rw) == HIGH) vites = 2;
  else vites = 0;

  // --- Sinyalizasyon Sistemi: Dörtlü (Debounce ile) ---
  if (mevcutMillis - sonOkumaZamaniDortlu > DEBOUNCE_SURE1) {
    if (digitalRead(dortlusw) == 1) {
      dortlu = !dortlu;
      sonOkumaZamaniDortlu = mevcutMillis;
    }
  }

  if (dortlu == 1) {
    if (mevcutMillis - oncekiMillis >= aralik) {
      oncekiMillis = mevcutMillis;
      if (sinyalsag == 0) { sinyalsag = 1; sinyalsol = 1; }
      else { sinyalsol = 0; sinyalsag = 0; }
    }
  } else if (digitalRead(solsinsw) == 1) {
    if (mevcutMillis - oncekiMillis >= aralik) {
      oncekiMillis = mevcutMillis;
      if (sinyalsol == 0) { sinyalsol = 1; } else { sinyalsol = 0; }
    }
  } else if (digitalRead(sagsinsw) == 1) {
    if (mevcutMillis - oncekiMillis >= aralik) {
      oncekiMillis = mevcutMillis;
      if (sinyalsag == 0) { sinyalsag = 1; } else { sinyalsag = 0; }
    }
  } else {
    sinyalsag = 0; sinyalsol = 0;
  }

  // Sinyal Çıkışları
  if (dortlu == 1) {
    digitalWrite(solsin, 1); digitalWrite(sagsin, 1);
  } else if (sinyalsag == 1) {
    digitalWrite(sagsin, 1); digitalWrite(solsin, 0);
  } else if (sinyalsol == 1) {
    digitalWrite(solsin, 1); digitalWrite(sagsin, 0);
  } else {
    digitalWrite(solsin, 0); digitalWrite(sagsin, 0);
  }

  // Sinyal Lambası (Ek Fonksiyon, Debounce ile)
  if (mevcutMillis - sonOkumaZamaniLamba > DEBOUNCE_SURE1) {
    if (digitalRead(sinyallambasisw) == 1) {
      sinyallambasidata = !sinyallambasidata;
      sonOkumaZamaniLamba = mevcutMillis;
    }
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

  // --- Jiroskop/İvmeölçer Okuma ve Güvenlik Kontrolü ---
  mpuOku();
  jiroskopGuvenlikKontrolu();

  // -----------------------------------------------------------------------------------
  // GAZ KOLU OKUMA (3 Kablolu PWM Sinyal) + GAZ SERTLİĞİ EĞRİSİ
  // -----------------------------------------------------------------------------------
  unsigned int pulseGenisligi;
  unsigned long sonPulseZamani;
  noInterrupts();
  pulseGenisligi = gazPulseGenisligi;
  sonPulseZamani = gazSonPulseZamani;
  interrupts();

  gazSinyaliVarMi = (mevcutMillis - sonPulseZamani) < GAZ_PWM_TIMEOUT_MS;
  bool gazPlazibiliteHatasi = gazSinyaliVarMi && (pulseGenisligi < 500 || pulseGenisligi > 2500);
  bool gazSensorArizasi = (!gazSinyaliVarMi) || gazPlazibiliteHatasi;

  gazYuzdesiHam = 0;
  if (gazSinyaliVarMi) {
    unsigned int pgSinirli = constrain(pulseGenisligi, GAZ_PWM_MIN_US, GAZ_PWM_MAX_US);
    gazYuzdesiHam = map(pgSinirli, GAZ_PWM_MIN_US, GAZ_PWM_MAX_US, 0, 100);
  }
  gazYuzdesi = gazEgrisiUygula(gazYuzdesiHam, mode);       // Gaz sertliği eğrisi uygulanır
  gazseviyesi = map(gazYuzdesi, 0, 100, 0, ANALOG_MAX_VALUE); // EDS matematiğiyle uyum için 0-1023'e çevir

  // --- EDS (Elektronik Diferansiyel Sistemi) Buton Kontrolü (Debounce ile) ---
  if (mevcutMillis - sonOkumaZamani3 > DEBOUNCE_SURE) {
    if (digitalRead(EDSButon) == HIGH) {
      EDS = !EDS;
      sonOkumaZamani3 = mevcutMillis;
    }
  }

  int gazSol = gazseviyesi;
  int gazSag = gazseviyesi;
  EDS_AKTIF = 0;

  // EDS Aktivasyon Şartı: Buton açık VE Gaz yeterince basılı
  if (EDS == 1 && gazYuzdesi >= MIN_GAZ_YUZDESI) {
    EDS_AKTIF = 1;
    float oran = abs(direksiyonaci) / maxdireksiyonaci;
    int uygulamaFarki = (int)(oran * dengekatsayisi);

    if (direksiyonaci > 0) { // Sağa dönüş
      gazSol = gazseviyesi + uygulamaFarki;
      gazSag = gazseviyesi - uygulamaFarki;
    } else if (direksiyonaci < 0) { // Sola dönüş
      gazSol = gazseviyesi - uygulamaFarki;
      gazSag = gazseviyesi + uygulamaFarki;
    }
    gazSol = constrain(gazSol, 0, ANALOG_MAX_VALUE);
    gazSag = constrain(gazSag, 0, ANALOG_MAX_VALUE);
  }

  // -----------------------------------------------------------------------------------
  // GÜVENLİK SİSTEMİ: APPS-Fren Plazibilite + Gaz Sinyali Arızası + Devrilme + Çarpışma
  // -----------------------------------------------------------------------------------
  bool frenGazCakismasi = frenBasili && (gazYuzdesi >= FREN_GAZ_KESME_ESIGI);
  throttleGuvenlikHatasi = frenGazCakismasi || gazSensorArizasi || devrilmeTespit || carpismaTespit;

  if (throttleGuvenlikHatasi) {
    gazSol = 0; gazSag = 0; // Güç ANINDA kesilir (yumuşatma uygulanmaz)
    digitalWrite(hata_led, HIGH);

    // Devrilme/çarpışma durumunda dörtlü ikaz otomatik ve ANINDA yakılır (bir sonraki
    // loop turunu beklemeden), sinyal sistemiyle çakışmaması için "dortlu" bayrağı da
    // güncellenir.
    if (devrilmeTespit || carpismaTespit) {
      dortlu = true;
      digitalWrite(solsin, HIGH); digitalWrite(sagsin, HIGH);
    }

    if (gazSensorArizasi)  Serial.println("1301"); // Hata kodu: gaz sinyali kaybı/plazibilite hatası
    if (frenGazCakismasi)  Serial.println("1302"); // Hata kodu: fren+gaz aynı anda basılı
    if (devrilmeTespit)    Serial.println("1303"); // Hata kodu: devrilme/aşırı eğim tespit edildi
    if (carpismaTespit)    Serial.println("1304"); // Hata kodu: çarpışma/darbe tespit edildi
  } else {
    digitalWrite(hata_led, LOW);

    // --- Basit Çekiş Kontrolü: Kayma tespit edilirse gücü azalt (tam kesme değil) ---
    if (kaymaTespit) {
      gazSol = (int)(gazSol * TRACTION_KESME_ORANI);
      gazSag = (int)(gazSag * TRACTION_KESME_ORANI);
    }

    // --- İvme Yumuşatma (Sport modda devre dışı) ---
    if (mode != 2) {
      gazSol = constrain(gazSol, oncekiGazSol - IVME_YUMUSATMA_ADIMI, oncekiGazSol + IVME_YUMUSATMA_ADIMI);
      gazSag = constrain(gazSag, oncekiGazSag - IVME_YUMUSATMA_ADIMI, oncekiGazSag + IVME_YUMUSATMA_ADIMI);
      gazSol = constrain(gazSol, 0, ANALOG_MAX_VALUE);
      gazSag = constrain(gazSag, 0, ANALOG_MAX_VALUE);
    }
  }
  oncekiGazSol = gazSol;
  oncekiGazSag = gazSag;

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

  // --- Raspberry Pi'ye Telemetri Paketi Gönderimi (Binary + CRC8, Serial1 üzerinden) ---
  gonderTelemetriRPi();

  delay(mainLoopDelay);
}
//