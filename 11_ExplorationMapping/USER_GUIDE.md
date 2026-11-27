# Exploration Mapping — Kullanıcı Kılavuzu (Kobuki + tek HC-SR04, 4 yöntem)

Robot laboratuvar alanını haritayı bilmeden dolaşır. Her hücre için P(Dolu) olasılığını Gauss sensör modeli ve Bayes kuralıyla hesaplar, sonunda haritayı A* (10) için 0/1 txt olarak kaydeder.

Bunu dört farklı keşif yöntemiyle yapar: **11_01 Frontier, 11_02 BFS, 11_03 Random, 11_04 TSP**. Hepsi tek bir Windows uygulamasından (**ExploreLauncher**) seçilir, çalıştırılır ve karşılaştırılır.

Genel bakış ve sonuçlar: [README.md](README.md). Teknik ayrıntılar: [ExplorationMapping.md](ExplorationMapping.md).

## 1. Başlatma

En pratik yollar (hepsi gerekirse önce derler, sonra launcher'ı açar):

| Yol | Nasıl | Not |
|---|---|---|
| **Çift tıklama** | Dosya Gezgini'nde `11_ExplorationMapping\ExploreLauncher.cmd` | Release derler ve açar. Hata olursa pencere açık kalır. |
| **Masaüstü kısayolu** | Bir kez: `.\run.ps1 launcher -Config release -Shortcut` | Masaüstüne **11 Exploration Launcher** koyar. Sonra kısayola çift tıklamak yeter (derlemez; kod değişince komutu tekrar çalıştırın). |
| **PowerShell** | Depo kökünde `.\run.ps1 launcher` (debug) ya da `.\run.ps1 launcher -Config release` | Önce çalışan robot programlarını kapatır, COM portu boşalır. |
| **VS Code** | **Terminal → Run Task → "Robot: 11 Exploration Launcher"** | |
| **Doğrudan** | `out\build\x64-release\11_ExplorationMapping\bin\ExploreLauncher.exe` | Derlenmiş olmalı. |

- `ExploreLauncher.cmd -NoBuild` derlemeden son derlenmiş launcher'ı açar.
- Launcher'ı derlemek dört `Explore_*.exe`'yi de derler. Launcher onları aynı klasörden başlatır.
- Laboratuvarda tipik sıra: robotu ve Arduino'yu bağla → launcher → *Gerçek robot* → **Portları tara** (iki yeşil ✓) → **3. adım: 40 cm kalibrasyonu** (sensör değişmediyse günde bir kez yeter) → yöntem + harita → **Robotta başlat**.

## 2. Launcher penceresi

Tek pencere, dört sekme:

| Sekme | Ne için |
|---|---|
| **Çalıştır** | Dört adım (solda) ve canlı harita (sağda). Gerçek robotta sağ taraf gerektiğinde bir panele dönüşür: kalibrasyon ([§2.1](#21-sensör-kalibrasyonu-3-adım)), hazırlık kontrolü ([§2.2](#22-hazırlık-kontrolü-robotta-başlat)), elle sürüş ([§2.3](#23-elle-sürüş-ve-sensör-testi)). |
| **Karşılaştırma** | Benchmark ve üç ölçütle sıralama. |
| **Geçmiş** | Yapılmış koşular; launcher'dan başlatılanlar kare kare yeniden izlenir ([§2.4](#24-geçmiş-koşu-geçmişi-ve-tekrar-izleme)). |
| **Haritalar** | Engel haritası düzenleyici ([§2.5](#25-haritalar-engel-haritası-düzenleyici)). |

```text
┌──────────────────────────┐ ┌ KEŞİF ┐┌ DOĞRULUK ┐┌ BAŞARI ORANI ┐┌ SÜRE ┐┌ YOL ┐
│ 1 Çalışma modu           │ │ %68   ││ %95      ││ %64          ││ 77 s ││ 3.8 m│
│ [ Simülasyon | Gerçek ]  │ └───────┘└──────────┘└──────────────┘└──────┘└──────┘
├──────────────────────────┤ ┌ Canlı harita ─────────── Sürüş ┐ ┌ Yöntem ──────────┐
│ 2 Bağlantı               │ │  %99 %99 %99 %50 ...            │ │ 11_01 Frontier    │
│ Kobuki  [COM6 ▾] ✓ Bağlı │ │  %99 %1  %1  %50   (P(Dolu) %)  │ │ kural + bilgiler  │
│ Arduino [COM14▾] ✓ Bağlı │ │  ... robot, sonar, yol,         │ ├ Olaylar ──────────┤
│ [Test et] [Portları tara]│ │      frontier, hedef, tur,      │ │ [10 s] frontier.. │
├──────────────────────────┤ │      yanlış hücre ✗             │ │ [21 s] ...        │
│ 3 Sensör kalibrasyonu [Aç]│ │                                  │ │                   │
│ Trig D11 · Echo D12      │ │                                  │ │                   │
│ ✓ 40 cm = 2332 µs ×0.98  │ │                                  │ │                   │
├──────────────────────────┤ │                                  │ │                   │
│ 4 Algoritma ve harita    │ │                                  │ │                   │
│ [Frontier][BFS][Rnd][TSP]│ │                                  │ │                   │
│ [map1][map2][map3][map4] │ └──────────────────────────────────┘ └───────────────────┘
│ [▶ Robotta başlat]       │
└──────────────────────────┘
```

### Çalıştır

1. **Çalışma modu:** *Simülasyon* ya da *Gerçek robot*.
2. **Bağlantı** (gerçek robot). Simülasyonda bu kartın adı **Simülatör**dür; iki anahtar vardır: **Sensör gürültüsü** ve **Hızlı (beklemeden)**. Hızlı kapalıyken simülasyon izlenebilir hızda (~2,5 × gerçek zaman) akar.
   - **Port seçimi:** Kobuki ve Arduino için portu kutudan seçin. Seçilen port **hemen test edilir**.
   - **Test:** Port ~3,5 s dinlenir (robota komut gönderilmez, robot hareket etmez):

     | Rozet | Anlamı |
     |---|---|
     | ✓ Bağlı (Kobuki) | Checksum'ı doğru Kobuki paketleri geliyor; batarya voltajı da yazar (13,5 V altı: "şarj edin"). |
     | ✓ Bağlı (Arduino) | `sensor.ino`'nun JSON satırları geliyor; ön sensörün o anki mesafesi ve satır hızı (~10 Hz) yazar. |
     | ✗ Bu port Arduino / Kobuki | Portlar ters seçilmiş. |
     | ! Yanlış program | Arduino veri gönderiyor ama JSON değil (başka bir sketch yüklü). `sensor.ino`'yu yükleyin. |
     | ✗ Hata: Veri gelmiyor | Port açıldı ama sessiz: güç, kablo ya da kart. |
     | ✗ Hata: Port başka programda açık | Arduino IDE / seri monitör / başka bir program portu tutuyor. |

   - **Bağlantıyı test et:** İki portu yeniden dinler.
   - **Portları tara:** Bütün portları (Bluetooth hariç) aynı anda dinler; Kobuki'yi ve Arduino'yu gönderdikleri veriden bulup kendiliğinden atar. Gerçek robot moduna ilk geçişte port seçili değilse tarama kendiliğinden başlar.
   - **Elle sürüş ›:** Robotu elle sürme ve sensör testi panelini açar ([§2.3](#23-elle-sürüş-ve-sensör-testi)).
3. **Sensör kalibrasyonu** (gerçek robot; ayrıntı: [§2.1](#21-sensör-kalibrasyonu-3-adım)):
   - Kart özeti gösterir: pinler, son kalibrasyon (`✓ 40 cm = 2332 µs · ham × 0.9786`) ya da `! Kalibre edilmedi`.
   - **Aç ›** sağdaki paneli açar. Bağlantı ilk kez doğrulandığında sensör kalibre değilse panel kendiliğinden açılır.
   - Kalibrasyon bir günden eskiyse ya da pinler kalibrasyondan sonra değiştiyse kart sarı olur.
   - Kalibrasyon yoksa hazırlık kontrolü uyarır (ham mesafelerle de çalışılabilir).
4. **Algoritma ve harita:**
   - Dört yöntemden birini ve map1..map4'ten birini seçin. Sağda seçilen engel haritası (referans) koordinatlarıyla görünür.
   - Simülasyonda bu harita simülatörün dünyasıdır. Gerçek robotta çıkan harita bu referansla karşılaştırılır.
5. **▶ Başlat:**
   - Gerçek robotta önce **hazırlık kontrolü** açılır ([§2.2](#22-hazırlık-kontrolü-robotta-başlat)). Oradaki **Robotta başlat** koşuyu başlatır; robot bağlandıktan 3 s sonra hareket eder.
   - Açık panel kapanır ve portlar programa bırakılır.
   - Program ayrı pencere açmaz. Durumunu launcher'a akıtır (`--ipc`) ve harita **canlı** çizilir:
     - **Hücreler:** P(Dolu) değeri % olarak (`%1` boş … `%99` dolu, `%50` bilinmiyor).
     - **Çizimler:** Robot, sonar ışını, yol, frontier, hedef, yöntemin katmanı (sıra numaraları, TSP turu).
     - **Yanlış hücreler:** Referansa göre yanlış karar verilen hücreler kırmızı ✗ ile işaretlenir.
   - **Göstergeler** (görülebilir hücrelere göre):

     | Gösterge | Tanım |
     |---|---|
     | **Keşif** | Karar verilmiş / görülebilir hücre |
     | **Doğruluk** | Doğru karar / karar verilmiş hücre |
     | **Başarı oranı** | Doğru karar / görülebilir hücre (2. karşılaştırma ölçütü, results.csv ile aynı) |
     | **Süre** | Harita tamamlanınca 1. ölçüt (hız) |
     | **Yol** | Sürülen mesafe |

   - **Panel:** Sağda yöntemin bilgileri (ör. TSP tur uzunluğu) ve olaylar akar.
6. **■ Durdur:**
   - Programa "stop" gönderir; program robotu durdurur, o ana kadarki haritayı kaydeder ve kapanır.
   - 10 s içinde kapanmazsa zorla kapatmak için onay sorar.
   - Pencereyi kapatmak da aynı şekilde önce durdurur.

Bitince durum rozeti **✓ Harita tamamlandı** ya da **Durduruldu** olur; **Resmi aç** kaydedilen PNG'yi açar. Koşu `results/results.csv` dosyasına eklenir ve kare kaydı haritanın yanına `<harita>.replay` olarak yazılır (Geçmiş sekmesi).

Dörtten fazla harita varsa 4. adımdaki `‹ ›` düğmeleri sayfalar arasında geçer.

### 2.1 Sensör kalibrasyonu (3. adım)

Panel açıkken launcher Arduino portunu açık tutar (robota hiçbir şey gönderilmez). Üç bölüm vardır:

**Pin bağlantısı**
- **Trig** ve **Echo** kutularından HC-SR04'ün bağlı olduğu dijital pini seçin (D2..D13; D0/D1 USB seri hattıdır). Varsayılan Trig **D11**, Echo **D12**.
- **Karta yaz** seçimi Arduino'ya gönderir (`PINS 7 8`). Kart pinleri EEPROM'a yazar; kart yeniden başlasa da kalır. Kartın yanıtı gelince `✓ Pinler karta yazıldı` görünür.
- Panel açılınca kart sorgulanır (`?`). Kutular karttaki pinleri gösterir: `✓ Kartta kayıtlı: Trig D11 · Echo D12 (sensor.ino v2)`.
- **Varsayılana al** D11/D12'yi seçer. Yazmak için yine **Karta yaz**'a basın.
- Kart yanıt vermezse eski `sensor.ino` yüklüdür: `! Kart pin komutlarını tanımıyor`. Okuma ve kalibrasyon yine çalışır, yalnız pin değiştirilemez. Yeni sketch'i yükleyin ([§3](#3-donanım)).

**Anlık okuma** (~10 Hz)

| Kutu | Anlamı |
|---|---|
| **HAM DEĞER** | Yankı süresi, µs (sensörün ölçtüğü asıl değer). Eski sketch bunu göndermez. |
| **HAM MESAFE** | µs × 0,01715 (343 m/s, gidiş-dönüş) = cm. |
| **KALİBRELİ** | Ham mesafe × ölçek. Kalibrasyondan sonra görünür. |

Altındaki grafik son ~12 s'yi çizer (kalibrasyon varsa kalibreli cm). Kesikli çizgi 40 cm'dir. Ani düşüşler hayalet yankılardır.

**40 cm kalibrasyonu**
1. Robotu düz zemine koyun; sensör yere paralel ve tam ileri baksın.
2. 40 × 40 cm bloğu, düz yüzü sensöre dik gelecek şekilde, **sensörün ön yüzünden bloğun yüzüne tam 40 cm** olacak biçimde koyun (bir hücre).
3. Grafik 40 cm civarında sabitlenince **▶ 40 cm kalibrasyonunu başlat**'a basın. Yankılı 50 ölçüm alınır (~5 s).
4. Hesap:
   - Ortadan (medyandan) 3 cm'den uzak okumalar atılır (hayalet yankı).
   - Kalanların ortalaması alınır: ölçek = 40 / ortalama.
5. Sonuç kutusu bu değeri cm karşılığıyla gösterir: **40 cm = 2332 µs**. Ayrıca ham ortalamayı, ölçeği, bunun karşılık geldiği ses hızını (ör. 336 m/s; 20 °C'de 343), yayılımı (σ) ve atılan ölçüm sayısını yazar.
6. Sonuç `11_ExplorationMapping\sensor_calibration.txt` dosyasına yazılır. **Dört yöntem de** gerçek robotta okumaları `ham cm × ölçek` olarak kullanır (`--range-cal`).

Kalibrasyon şu durumlarda reddedilir ve önceki kalibrasyon korunur:

| Mesaj | Ne yapmalı |
|---|---|
| Blok 40 cm'de görünmüyor (okumaların ortası 40 ± 15 cm dışında) | Bloğun yerini ve sensörün yönünü kontrol edin. |
| Okumalar çok dağınık | Blok sabit ve sensöre dik mi? |
| Yankı alınamıyor | Blok çok uzak / eğik mi, sensör bağlı mı? |
| Ölçek 0,85 – 1,15 dışında | Ölçüm şüpheli; tekrar deneyin. |

Tek nokta tek parametre belirler. HC-SR04'ün hatası çoğunlukla ses hızından (sıcaklık) gelen bir ölçek hatasıdır, bu yüzden 40 cm noktası ölçek için kullanılır. Gauss gürültü modeli (σ(d), P(z|E)) ayrı ve daha ayrıntılı bir kalibrasyondur ([§4](#4-kalibrasyon)).

**Sketch yükleme** (Pin bağlantısı kartının alt yarısı)
- **Kart** kutusundan kart türünü seçin (Uno, Nano, Nano eski bootloader, Mega 2560).
- **Sketch'i yükle:** `arduino/sensor/sensor.ino` arduino-cli ile derlenir ve Arduino portuna yazılır. Çıktı, anlık okuma kartının yerinde görünür. Bitince port yeniden test edilir ve kart sorgulanır.
- Launcher yükleme sırasında portu bırakır; Arduino IDE'nin seri monitörü açıksa kapatın.
- arduino-cli yoksa düğme **arduino-cli'yi kur** olur. Onay verince `arduino/setup-arduino-cli.ps1` çalışır:
  - arduino-cli'yi depodaki `.tools\arduino-cli` klasörüne indirir (gitignore'da).
  - AVR çekirdeğini kurar (~180 MB, Arduino IDE ile aynı `%LOCALAPPDATA%\Arduino15` klasörüne).
- Arduino IDE 2 kuruluysa onun arduino-cli'si kendiliğinden bulunur.
- Kartta başka bir program ya da eski sketch varsa düğme mavi olur.

**Kalibrasyonun yaşı ve doğrulaması**
- Sonuç kutusu kalibrasyonun ne zaman yapıldığını yazar ("2 saat önce"). Bir günden eskiyse ya da karttaki pinler kalibrasyondakinden farklıysa kutu sarı olur.
- **Kontrol:** Bloğu 40 ya da 80 cm'ye koyun, mesafeyi seçin, **Bloğu koyup kontrol et**'e basın. 20 ölçüm alınır ve sapma gösterilir: `✓ 80 cm'de 80.6 cm · sapma +0.6 cm · geçerli`. Kayıt değişmez.
- Sapma 40 cm'de 1 cm'yi, 80 cm'de 2 cm'yi aşarsa `✗ … yeniden kalibre edin` yazar.
- 80 cm kontrolü özellikle yararlıdır: kalibrasyon tek noktadan (40 cm) yapıldığı için ikinci bir mesafe ölçeğin doğru olduğunu gösterir.

### 2.2 Hazırlık kontrolü (Robotta başlat)

Gerçek robotta **Robotta başlat** önce bu paneli açar. İki port yeniden dinlenir (robota komut gitmez) ve koşuyu yarıda bırakabilecek her şey tek listede görünür:

| Satır | ✓ | ! uyarı | ✗ sorun |
|---|---|---|---|
| Program | exe derlenmiş | | derlenmemiş |
| Kobuki bağlantısı | paketler geliyor | | port yok / veri yok / başka cihaz |
| Batarya | ≥ 14,5 V | 13,5 – 14,5 V | < 13,5 V |
| Tampon ve tekerlekler | serbest, yerde | zemin sensörü boşluk görüyor | tampon basılı, tekerlek havada |
| Şarj | | adaptör / istasyon takılı | |
| Arduino bağlantısı | sensör satırları geliyor (sketch sürümüyle) | | port yok / veri yok / yanlış program |
| Ön sensör | okuyor | yankı yok, ya da önü kapalı (< 12 cm) | |
| Sensör kalibrasyonu | var, bir günden yeni | yok ya da eski | |

- Altta yerleştirme hatırlatması durur: robot (1,1)'de, +y'ye bakıyor.
- **▶ Robotta başlat** koşuyu başlatır. Sorun (✗) varsa düğme kırmızı **Yine de başlat** olur.
- **Yeniden kontrol et** portları bir daha dinler; **Vazgeç** haritaya döner.

### 2.3 Elle sürüş ve sensör testi

2. adımdaki **Elle sürüş ›** paneli açar; launcher Kobuki'ye bağlanır (Arduino portu seçiliyse ön sensörü de gösterir).

**Sürüş**
- **Ok tuşları** ya da **W A S D**: basılı tuttukça robot hareket eder, bırakınca durur. Ekrandaki ▲ ◀ ▶ ▼ düğmeleri de fareyle basılı tutularak kullanılır.
- **Boşluk**, **Esc** ya da kırmızı **DUR**: hemen durdurur.
- **Hız:** Yavaş (0,08 m/s) / Normal (0,15 m/s).
- **Sabit hareketler:** İleri 40 cm (1 hücre), Geri 40 cm, Sola 90°, Sağa 90°. Robotun bir hücreyi ve dik açıyı ne kadar doğru gittiğini cetvelle ölçmek için.

**Güvenlik**
- Tampon basılınca ileri hareket sürücüde kesilir.
- Komut 0,3 s yenilenmezse robot durur: tuş bırakılınca, pencere odağı kaybedince ya da program takılınca.
- Sabit hareketler tampon, tekerlek / zemin sensörü ya da 12 s zaman aşımında durur.
- Yine de robotun yanında durun ve çevresini boş tutun.

**Sensörler**
- Batarya voltajı, tekerlek ve zemin sensörleri, yön (jiroskop), gidilen yol, x-y konumu.
- **Tampon testi:** Sol, orta ve sağ tampona elle birer kez basın; her biri `✓ denendi` olur. Üçü de algılanınca test tamamdır.
- **Ön sensör:** Arduino'nun o anki okuması (kalibrasyon varsa kalibreli cm).

### Karşılaştırma

- **Benchmark:**
  - Yöntem ve harita seçin, tekrar sayısını ve başarı eşiğini ayarlayın.
  - **Benchmark başlat:** İşaretli yöntemler × haritalar × seed 1..N simülasyonda, pencere açmadan art arda koşar. 4 × 4 × 3 = 48 koşu yaklaşık yarım dakika sürer.
  - Günlükler `results/logs/` klasörüne yazılır.
- **Sıralama:**
  - Önce başarılı harita sayısı, sonra başarı oranı, sonra hız.
  - Sütunlar: 1) hız (ort. süre), 2) başarı oranı, 3) başarılı engel haritası.
  - **Filtre:** Simülasyon / Gerçek robot / Tümü.
- **Yöntem × engel haritası:** Her hücrede ortalama süre ve başarı oranı yazar, ayrıca eşiği geçen koşu sayısı ve ✓ işareti. Yeşil = o harita başarılı.
- **Rapor kaydet (.md):** `results/comparison_<tarih>.md` yazar. Komut satırından: `ExploreLauncher.exe --report`.
- **Sonuçları arşivle:** `results.csv` silinmez, `results_<tarih>.csv` olarak saklanır ve boş bir tabloyla başlanır.

Seçimler `results/launcher.ini` dosyasında saklanır.

### 2.4 Geçmiş: koşu geçmişi ve tekrar izleme

- Solda `results.csv`'deki koşular en yeniden eskiye listelenir (**Tümü / Simülasyon / Gerçek robot**). Fare tekerleği listeyi kaydırır.
- Bir koşuya tıklayınca sağda aynı göstergeler ve harita görünür.
- **▶ kayıt** işaretli koşular kare kare yeniden izlenir:
  - **▶ Oynat / ⏸ Duraklat** (Boşluk tuşu da olur), hız **1× 4× 16× 64×**.
  - Zaman çubuğuna tıklayarak ya da sürükleyerek istenen ana gidilir.
  - Robotun yolu, hedefi, frontier, yöntemin sıra numaraları ve olaylar o andaki haliyle çizilir.
- Kare kaydı (`<harita>.replay`) yalnız launcher'ın **Çalıştır** sekmesinden başlatılan koşularda tutulur. Benchmark ve komut satırı koşularında yalnız son harita gösterilir.
- **Resmi aç** kaydedilen PNG'yi, **Klasörü aç** koşunun dosyalarını açar.

### 2.5 Haritalar: engel haritası düzenleyici

- map1 – map4 dersin referans haritalarıdır (görselleriyle); salt okunurdur.
- **+ Yeni harita:** Seçili haritanın kopyasını sıradaki boş adla (map5 … map8) açar.
- Hücreye tıklayınca engel eklenir / kaldırılır; basılı tutup sürükleyince boyanır. Kenar duvarı ve başlangıç hücresi sabittir.
- Sağdaki özet iç engel, boş hücre ve görülebilir hücre sayılarını verir. Robotun başlangıçtan gidemeyeceği boş hücreler sarı gösterilir; böyle hücreler başarı oranını düşürür.
- **Kaydet** `obstacleMapReferences\mapN\mapN.txt` dosyasını yazar. Harita hemen Çalıştır ve Karşılaştırma sekmelerinde seçilebilir.
- Üzerinde kayıtlı koşu olan bir harita değiştirilirse launcher uyarır: eski koşuların başarı oranı eski haritaya göredir.
- **Bu haritayla çalıştır** haritayı seçip Çalıştır sekmesine geçer. **Sil** yalnız kendi haritalarınızda çalışır.
- En çok 8 harita olabilir.

## 3. Donanım

| Sensör | Yön | Trig | Echo | Güç |
|---|---|---|---|---|
| HC-SR04 | tam ileri | D11 (varsayılan) | D12 (varsayılan) | VCC → 5V, GND → GND |

- Sensör robot merkezinden yaklaşık 11 cm önde ve yere paralel olmalı. Robot etrafı yerinde 360° dönerek ölçer.
- **Arduino kodu:** [arduino/sensor/sensor.ino](arduino/sensor/sensor.ino) (v2). Launcher'dan yükleyin (3. adım → **Sketch'i yükle**, [§2.1](#21-sensör-kalibrasyonu-3-adım)) ya da Arduino IDE'de kartı (Uno) ve portu seçip yükleyin.
- **Seri çıktı:** Ham ölçüm gönderir, ~10 Hz, 115200 baud: `{"l":-1,"f":40,"r":-1,"us":2332}`.
  - `f`: cm, 343 m/s ile, yuvarlanmış.
  - `us`: yankı süresi (µs).
  - Yankı yoksa ikisi de -1 olur.
  - Eski programlar yalnız `f`'yi okur; ek alan onları etkilemez.
- **Seri komutlar** (launcher 3. adım bunları kullanır; Arduino IDE seri monitöründen de yazılabilir, satır sonu "Newline"):

  | Komut | Yanıt |
  |---|---|
  | `?` | `{"ver":2,"trig":11,"echo":12}` |
  | `PINS 7 8` | Trig D7, Echo D8 olur, EEPROM'a yazılır; yanıt yeni ayar satırıdır. Geçersizse `{"err":"pins"}`. |

- Pin farklıysa sketch'i değiştirmeye gerek yok: launcher'dan **Karta yaz** yeterli.
- Montaj farklıysa [common/ExploreApp.h](common/ExploreApp.h) başındaki `SensorOffsetM` (0.11) sabitini güncelleyin.

## 4. Kalibrasyon

İki ayrı kalibrasyon vardır. İkisi de donanıma aittir, yönteme değil; dört yöntem de aynı dosyaları okur.

| | 40 cm mesafe kalibrasyonu | Gauss sensör modeli |
|---|---|---|
| **Nerede** | Launcher, 3. adım ([§2.1](#21-sensör-kalibrasyonu-3-adım)) | Komut satırı |
| **Ne bulur** | Ölçek: ham cm → gerçek cm | σ(d), sapma, P(algılama), P(z\|E) |
| **Dosya** | `sensor_calibration.txt` | `sensor_model.txt` |
| **Ne zaman** | Önce; sensör / pin / ortam sıcaklığı değişince | 40 cm kalibrasyonundan sonra, bir kez |

Gauss modeli 40 cm ölçeği uygulanmış okumalarla ölçülür (alan 7 hücre uzunluğunda olduğu için hücreler `1,2,3,5,6`):

```powershell
.\run.ps1 11 -Calibrate -ArduinoPort COM14 --cal-cells 1,2,3,5,6
.\run.ps1 11 -Sim -Calibrate                # donanımsız deneme
```

1. Robotu bir hücrenin ortasına koyun; sensör ızgara çizgilerine dik baksın.
2. **Gauss (Method B):** Program kutuyu sırayla 1, 2, 3, 5, 6 hücre öteye koymanızı ister.
   - Kutunun düz yüzü robota dönük ve hücrenin yakın kenarında olmalı.
   - Her konumda konsolda Enter'a basın; 100 ölçüm alınır.
3. **Boş hücre (Method A):** Kutuyu kaldırın, Enter'a basın; 100 ölçüm daha alınır.
4. **Sonuç:** `11_ExplorationMapping\sensor_model.txt` dosyasına yazılır.

## 5. Komut satırı (launcher olmadan)

```powershell
.\run.ps1 11 -Sim                                  # 11_01 Frontier, map3, simülatör
.\run.ps1 11 -Algo bfs -Sim -ObstacleMap map2      # 11_02 BFS, map2
.\run.ps1 random -Sim -ObstacleMap map1 --seed 4   # 11_03 Random, seed 4
.\run.ps1 tsp -ObstacleMap map3                    # 11_04 TSP, gerçek robot (portlar otomatik), map3 referans
.\run.ps1 11 -Algo tsp -Port COM6 -ArduinoPort COM14 -ObstacleMap map3 --go
```

Programlar doğrudan da çalışır:

```text
Explore_<Frontier|BFS|Random|TSP>.exe --sim [seçenekler]
Explore_<...>.exe <Kobuki COM> <Arduino COM> [seçenekler]
Explore_<...>.exe --calibrate <Arduino COM> [--trials N] [--cal-cells 1,2,3,5,10]
Explore_<...>.exe --calibrate --sim
ExploreLauncher.exe [--report]
ExploreLauncher.cmd [-Shortcut] [-NoBuild]      # 11_ExplorationMapping klasöründe, çift tıklanabilir
```

## 6. Program seçenekleri

| Seçenek | Etki |
|---|---|
| `--map txt` | Engel haritası. `--sim`'de simülatörün dünyası (varsayılan map3), robotta karşılaştırma referansı (verilmezse puan yok). `--truth` eski adıdır. |
| `--arena txt` | Alan dosyası. Varsayılan `maps/arena.txt`. |
| `--model txt` | Sensör modeli. Varsayılan `sensor_model.txt`; yoksa σ(4 m) = 10 cm, P(z\|E) = 0,20. |
| `--range-cal txt` | 40 cm mesafe kalibrasyonu (launcher 3. adım yazar). Varsayılan `sensor_calibration.txt`; yoksa ham mesafe (ölçek 1). Yalnız gerçek sensörde. |
| `--seed n` | Simülatör gürültüsü ve Random yönteminin tohumu (varsayılan 2026). |
| `--result csv` / `--no-result` | Sonuç satırının ekleneceği dosya (varsayılan `results/results.csv`) / ekleme. |
| `--no-verify` | Yöntem bitince dolu hücreleri doğrulama (daha hızlı, daha az güvenilir). |
| `--go` | Gerçek robotta Enter beklemeden 3 s sonra başla. |
| `--time s` | Azami süre (varsayılan 1500 s). |
| `--snapshot s` | Bu saniyede pencerenin resmini de kaydet (belge / kontrol için). Birden çok kez verilebilir. |
| `--ipc` | Pencere yok; durum stdout'a satır satır yazılır (launcher'ın canlı haritası, [common/Ipc.h](common/Ipc.h)), stdin'e `stop` yazılınca durur. |
| `--no-noise` / `--no-gui` / `--fast` / `--out klasör` | Simülatörde gürültü yok / pencere yok / beklemeden koştur / kayıt klasörü. |

## 7. Ekran (her yöntemde aynı)

**Pencere:** Her hücrede P(Dolu) değeri yazar. Renkler:

| Renk / çerçeve | Anlamı |
|---|---|
| Beyaz | boş (P < 0,3) |
| Gri | kararsız |
| Koyu kırmızı | dolu (P > 0,8) |
| Turkuaz çerçeve | frontier |
| Turuncu çerçeve | hedef |
| Mavi çizgi | yol |
| Yeşil | robot ve sonar ışını |
| Mor | yöntemin kendi katmanı: sıra numaraları, TSP turu, BFS kökü, rastgele hedef |
| Kırmızı çarpı | engel haritasına göre YANLIŞ karar verilmiş hücre (canlı) |

Sağ panelde şunlar görünür:
- yöntemin kuralı ve kendi bilgileri (ör. TSP tur uzunluğu, BFS kuyruğu);
- ızgara sayıları;
- engel haritasına göre canlı başarı oranı;
- keşif sayaçları.

`Q` / Esc / Ctrl+C / pencereyi kapatmak robotu durdurur ve o ana kadarki haritayı kaydeder.

**Kaydedilenler** (`11_ExplorationMapping\output\<yöntem>\`):
- `explored_<yöntem>_<harita>_s<seed>_<zaman>.txt`: 0/1 harita (A* için).
- `..._prob.txt`: her hücrenin P(Dolu) değeri.
- `.png`: pencerenin resmi.

**Haritayı A* ile kullanma:**
1. txt'de hedef hücreye `/` yazın.
2. `.\run.ps1 10 -Map 11_ExplorationMapping\output\tsp\explored_....txt` çalıştırın.

## 8. Sorun giderme

| Belirti | Çözüm |
|---|---|
| Launcher'da yöntemin yanında `(derlenmemiş)` | `.\run.ps1 launcher` ya da `cmake --build --preset x64-release --target ExploreLauncher` |
| `Sensor model: cannot open ...` | Gauss kalibrasyonu yapılmamış; varsayılan model kullanılır. Bkz. §4. |
| `!! Range calibration: ... missing` | 40 cm kalibrasyonu yapılmamış; ham mesafe kullanılır. Launcher 3. adım. |
| 3. adım: `! Kart pin komutlarını tanımıyor` | Kartta eski `sensor.ino` var. Aynı kartta **Sketch'i yükle**. |
| Sketch yükleme `✗ Başarısız` | Kart türü doğru mu (klon Nano'larda "eski bootloader")? Port Arduino IDE'de açık mı? Çıktının son satırları sebebi yazar. |
| Elle sürüş: `✗ Port açılamadı` / `Kobuki veri göndermiyor` | Port başka programda açık, robot kapalı ya da yanlış port. 2. adımda **Test et**. |
| Elle sürüşte tuşlar çalışmıyor | Pencere odakta olmalı; pencereye bir kez tıklayın. |
| 3. adım: `✗ Kart yanıt vermedi` / `Veri gelmiyor` | Panel portu açık tutar; Arduino IDE seri monitörünü kapatın, **Yeniden bağlan**. |
| 3. adım: okuma hep `yankı yok` | Pinler yanlış (Trig/Echo ters?) ya da sensör bağlı değil. Kablolamaya göre pinleri seçip **Karta yaz**. |
| Kalibrasyon: `the box was never detected` | Kutu sensörün tam önünde değil ya da çok küçük; düz ve geniş bir yüzey kullanın. |
| `No sensor data on COMx` | Arduino kodu yüklü değil, port başka programda açık ya da kablo sorunu var. Arduino'yu başka bir USB girişine takınca COM numarası değişir; launcher'da **Portları tara**. Ayrıntı: [README.md](README.md) §8. |
| `sensor stream lost (Arduino)` | Arduino 1,5 s veri göndermedi. |
| Boş hücreler dolu çıkıyor | Sensör yere paralel ve tam ileri mi? Kalibrasyonu yenileyin; doğrulamayı kapatmayın. |
| Bazı hücreler hep 0,50 | Hiç görülemeyen yerler (duvar arkası, dış köşeler); txt'ye 1 yazılır, başarı oranına sayılmaz. |
| Yeniden derlerken `LNK1104` | Bir `Explore_*.exe` hâlâ açık (exe kilitli). Kapatın ya da `.\run.ps1 stop`. |
