# 11_ExplorationMapping — dört keşif yöntemi ve karşılaştırma

Kobuki, laboratuvardaki 7 × 9 hücrelik alanı **haritayı bilmeden** dolaşır. Robotun önünde tek bir HC-SR04 sonar var.

- **Ölçüm:** Robot yerinde 360° döner ya da bir hücreye karşıdan bakar.
- **Harita:** Her hücrenin P(Dolu) olasılığı Gauss sensör modeli ve Bayes kuralıyla güncellenir.
- **Yöntemler:** "Sıradaki hareket nereye?" sorusunu **dört farklı yöntem** cevaplar. Bu yöntemler aynı haritalarda karşılaştırılır.

| Klasör | Program | Yöntem |
|---|---|---|
| [11_01_Frontier](11_01_Frontier/README.md) | `Explore_Frontier.exe` | En yakın frontier (A* yol maliyeti). Önceki sürümdeki yöntem, davranışı birebir aynı. |
| [11_02_BFS](11_02_BFS/README.md) | `Explore_BFS.exe` | Başlangıç hücresinden genişlik öncelikli (FIFO) sıra. |
| [11_03_Random](11_03_Random/README.md) | `Explore_Random.exe` | Rastgele ulaşılabilir hücre + 360° tarama (temel karşılaştırma). |
| [11_04_TSP](11_04_TSP/README.md) | `Explore_TSP.exe` | Tüm frontier hücrelerinden geçen en kısa tur (gezgin satıcı). |
| [Launcher](Launcher/src/main.cpp) | `ExploreLauncher.exe` | Windows uygulaması: 1) simülasyon / gerçek robot, 2) Kobuki + Arduino portları ve bağlantı kontrolü, 3) sensör: Trig/Echo pinleri, sketch yükleme, anlık okuma, 40 cm kalibrasyonu ve doğrulaması, 4) algoritma + harita; hazırlık kontrolü, elle sürüş ve sensör testi; canlı harita ve %'ler; benchmark ve karşılaştırma; koşu geçmişi ve tekrar izleme; harita düzenleyici. Çift tıkla: `ExploreLauncher.cmd`. |

Diğer belgeler:
- Kullanım: [USER_GUIDE.md](USER_GUIDE.md)
- Ortak teknik ayrıntı (sensör modeli, Bayes, ortak keşif döngüsü): [ExplorationMapping.md](ExplorationMapping.md)
- A* örneğinin özeti: [../10_AStarPathPlanning/OZET.md](../10_AStarPathPlanning/OZET.md)

Son güncelleme: **24.11.2026**.

---

## 1. Yapı: ortak çekirdek + değişen tek parça

Ders notundaki algoritmanın 4 adımından yalnızca **3. adım ("Pick a next move")** yöntemden yönteme değişir. Geri kalan her şey ortaktır, böylece karşılaştırma yalnızca karar kuralını ölçer.

```text
11_ExplorationMapping/
├─ common/                    ortak çekirdek (header-only, CMake: ExploreCore)
│  ├─ Strategy.h              ARAYÜZ: Strategy::next(PlanContext) -> Goal  + ortak yardımcılar (A*, BFS yolu, en yakın hedef)
│  ├─ Explorer.h              keşif döngüsü: tarama, sürüş, karşıdan ölçüm, doğrulama, tampon, süre sınırı
│  ├─ BayesGrid.h             Bayes doluluk ızgarası, Gauss sensör modeli, ışın işleme, frontier
│  ├─ ExploreRobot.h          gerçek Kobuki + Arduino, simülatör (seed'li gürültü)
│  ├─ ExploreView.h           canlı pencere / PNG (her yöntemin kendi çizim katmanıyla)
│  ├─ Calibration.h           sensör kalibrasyonu (Method A + B) - dört yöntem aynı sensor_model.txt'yi kullanır
│  ├─ RangeCalibration.h      40 cm mesafe kalibrasyonu (launcher yazar, programlar okur): sensor_calibration.txt
│  ├─ Metrics.h               başarı oranı, results.csv satırı
│  ├─ ExploreApp.h            program: argümanlar, döngü, kayıt, özet  ->  app::run(argc, argv, strateji)
│  ├─ Ipc.h                   --ipc: durum stdout'a satır satır (launcher'ın canlı haritası), stdin "stop"
│  └─ Console.h
├─ 11_01_Frontier/src/        FrontierStrategy.h + main.cpp (tek satır)
├─ 11_02_BFS/src/             BfsStrategy.h + main.cpp
├─ 11_03_Random/src/          RandomStrategy.h + main.cpp
├─ 11_04_TSP/src/             TspStrategy.h + TspSolver.h (Held-Karp / 2-opt) + main.cpp
├─ Launcher/src/              ExploreLauncher: main.cpp (arayüz), Theme.h (GDI+ tema), Probe.h (bağlantı kontrolü),
│                             LiveRun.h (canlı koşu + kayıt), SensorLink.h (3. adım: pinler, anlık okuma),
│                             Panels.h (kalibrasyon, elle sürüş, hazırlık kontrolü), Tabs.h (geçmiş, haritalar),
│                             Teleop.h (elle sürüş), ArduinoCli.h (sketch yükleme), Replay.h (tekrar izleme),
│                             MapFile.h (harita kaydı), ComPorts.h (SetupAPI), Results.h (karşılaştırma / rapor)
├─ ExploreLauncher.cmd        çift tıkla: derler (release) ve launcher'ı açar; -Shortcut masaüstü kısayolu
├─ obstacleMapReferences/     map1..map4: görsel (png/jpg) + 0/1 txt (simülatörün dünyası, gerçek koşunun referansı)
├─ maps/arena.txt             robotun bildiği tek şey: alan boyutu, başlangıç, yön
├─ arduino/sensor/sensor.ino  tek HC-SR04 (v2: pinler seri komutla / EEPROM, varsayılan D11/D12), JSON + yankı süresi
├─ arduino/setup-arduino-cli.ps1  arduino-cli + AVR çekirdeği (launcher'daki "Sketch'i yükle" için)
├─ sensor_calibration.txt     40 cm kalibrasyonu (launcher 3. adım yazar)
├─ output/<yöntem>/           her koşunun haritası (txt, olasılık, png, launcher koşularında .replay) - git'e girmez
└─ results/                   results.csv, logs/, comparison_*.md, launcher.ini - git'e girmez
```

**Yeni yöntem eklemek:** `11_05_X/src/XStrategy.h` içinde `explore::Strategy`'den türeyen bir sınıf yazın (`id`, `name`, `title`, `rule`, `next`). `main.cpp` tek satırdır: `app::run(argc, argv, [](auto&){ return std::make_unique<XStrategy>(); })`. CMake tarafında `add_explore_method(Explore_X src/main.cpp src/XStrategy.h)` yeterli. Son olarak launcher'daki `Methods` listesine bir satır ekleyin.

**Derleme esnekliği:**
- Her yöntem kapatılabilir: `cmake --preset x64-release -DEXPLORE_BUILD_RANDOM=OFF`. Ayrıca `EXPLORE_BUILD_FRONTIER`, `_BFS`, `_TSP` ve `_LAUNCHER` seçenekleri var.
- Tek bir yöntem derlenebilir: `--target Explore_BFS`.
- Tüm exe'ler tek klasöre çıkar: `out/build/<preset>/11_ExplorationMapping/bin/`. `ExploreLauncher` derlenince dört yöntem de derlenir.

## 2. Yöntemler bir bakışta

| | 11_01 Frontier | 11_02 BFS | 11_03 Random | 11_04 TSP |
|---|---|---|---|---|
| Sıradaki hedef | Robota A* yolu en kısa frontier | Başlangıçtan BFS (FIFO) sırasında ilk kararsız hücre | Ulaşılabilir boş hücrelerden rastgele biri | Tüm frontier'lardan geçen en kısa açık turun ilk durağı |
| Hedefte | Hücreye dön, karşıdan ölç | Hücreye BFS ebeveyninden dön, karşıdan ölç | Yerinde 360° tara | Hücreye dön, karşıdan ölç |
| Yol | A* (8 komşu) | BFS (4 komşu, en az hücre) | A* | A* |
| Bitiş | Ulaşılabilir frontier kalmadı | Kuyrukta ulaşılabilir hedef kalmadı | Frontier kalmadı ya da 6 taramada yeni hücre yok | Ulaşılabilir frontier kalmadı |
| Pencerede | 1, 2, 3: maliyet sırası | 1, 2, 3: BFS sırası, daire: kök | Numara: rastgele hedef | Mor çizgi: TSP turu, numaralar: tur sırası |

Hepsinde ortak:
- 360° başlangıç taraması.
- Her pakette Bayes güncellemesi.
- 3 ziyarette karar verilemeyen hücreden vazgeçme.
- **Doğrulama:** Yöntem bitince, boş alana komşu ve hiç karşıdan ölçülmemiş dolu hücreler yeniden ölçülür (`--no-verify` ile kapatılır).
- Tampon / uçurum güvenliği ve 1500 s süre sınırı.

## 3. Karşılaştırma ölçütleri

Her koşu, bittiğinde `results/results.csv` dosyasına bir satır ekler ([common/Metrics.h](common/Metrics.h)). Launcher bu tabloyu okur.

1. **Hız:** Haritayı kendiliğinden tamamlayan koşunun süresi (`time_s`). Simülasyonda simülasyon saniyesi, robotta gerçek süredir. Süre sınırına takılan ya da elle durdurulan koşu "tamamlanmadı" sayılır.
2. **Başarı oranı:** Doğru karar verilen hücre / görülebilir hücre (`success_ratio`), engel haritasına göre hesaplanır.
   - **Görülebilir hücre:** Boş hücre ya da boş komşusu olan engel hücresi.
   - Dört dış köşe ve kalın duvarın içi hiçbir sonardan görülemez. Bunları saymak her yöntemi aynı miktarda düşürürdü.
   - Tüm hücrelere göre doğruluk da `accuracy_all` sütununda yazılır. Kararsız hücre bu hesapta yanlış sayılır.
3. **Engel haritası sayısı:** Kaç farklı haritada başarılı olduğu.
   - Bir harita, o yöntemin koşularının en az yarısı **kendiliğinden bitip** başarı oranı ≥ eşik (varsayılan %95) olduğunda "geçti" sayılır.

Gerçek robotta da engel haritası seçilir (launcher bunu her zaman verir). Çıkan harita o referansla karşılaştırılır ve yanlış hücreler pencerede canlı olarak kırmızı çarpıyla gösterilir.

## 4. Simülasyon karşılaştırması (24.11.2026)

Launcher → Benchmark: 4 yöntem × 4 harita × 3 seed = **48 koşu**. Gürültülü sensör kullanıldı (koni, mesafeyle büyüyen gürültü, %4 kaçan yankı, %2 hayalet okuma), varsayılan sensör modeliyle. Toplam ~30 s sürdü.

| Sıra | Yöntem | 1) Hız: ort. süre | 2) Başarı oranı | 3) Başarılı harita | Ort. yol |
|---:|---|---:|---:|---:|---:|
| 1 | 11_04 TSP | **323 s** | 100,0 % | 4 / 4 | 14,8 m |
| 2 | 11_01 Frontier | 330 s | 100,0 % | 4 / 4 | 15,5 m |
| 3 | 11_02 BFS | 358 s | 100,0 % | 4 / 4 | 16,7 m |
| 4 | 11_03 Random | 639 s | 98,8 % | 4 / 4 | 26,4 m |

| Yöntem | map1 | map2 | map3 | map4 |
|---|---|---|---|---|
| 11_01 Frontier | 301 s / 100 % | 335 s / 100 % | 345 s / 100 % | 337 s / 100 % |
| 11_02 BFS | 358 s / 100 % | 397 s / 100 % | 352 s / 100 % | 325 s / 100 % |
| 11_03 Random | 568 s / 100 % | 674 s / 97,1 % | 668 s / 99,4 % | 645 s / 98,7 % |
| 11_04 TSP | 299 s / 100 % | 340 s / 100 % | 332 s / 100 % | 321 s / 100 % |

- **TSP ile Frontier:** TSP, haritaların üçünde Frontier'dan kısa sürdü. Yalnız map2'de (340 s / 335 s) biraz uzun sürdü. Tur bazen en yakın hedef yerine, bir köşeyi sonraya bırakmayacak bir hedefi önce seçer. Frontier yerelde en yakını seçtiği için o köşeye sonradan geri dönmek zorunda kalır.
- **BFS:** Başlangıçtan katman katman ilerlediği için robot ileri geri gidiyor. Ortalama ~%9 daha yavaş ve daha uzun yol kat ediyor.
- **Random:** Hiç yanlış hücre vermedi, ama yaklaşık iki kat yavaş. 12 koşunun 3'ü durgunlukla (6 taramada yeni hücre yok) bitti ve bazı hücreler kararsız kaldı (%96–99).
- **Hata ve çarpma:** 48 koşunun hiçbirinde görülebilir hücrede yanlış karar ve tampon teması olmadı.
- **Fark nerede çıkacak:** Simülasyonda başarı oranı neredeyse hep %100. Asıl fark gerçek robotta (sensör eğimi, odometri kayması) ortaya çıkacak; bkz. §7.

Raporu yeniden üretmek: launcher'da **Rapor kaydet**, ya da `ExploreLauncher.exe --report` (`results/comparison_<tarih>.md`).

## 5. Alan ve engel haritaları

- 7 sütun × 9 satır, hücreler 40 cm.
- Koordinatlar **(x, y)**, 0'dan başlar. x sağa, **y yukarı** artar: alt satır y = 0, üst satır y = 8.
- Başlangıç **(1,1)**, robot **+y** yönüne (heading 90°) bakar.
- [maps/arena.txt](maps/arena.txt): Robotun bildiği tek şey (boyut, başlangıç, yön). Bütün hücreler `?`.

| Harita | İç engel hücreleri | Görsel | txt |
|---|---|---|---|
| map1 | yok (yalnız dış duvar) | [map1.png](obstacleMapReferences/map1/map1.png) | [map1.txt](obstacleMapReferences/map1/map1.txt) |
| map2 | (1,4) (2,4) | [map2.png](obstacleMapReferences/map2/map2.png) | [map2.txt](obstacleMapReferences/map2/map2.txt) |
| map3 | (1,4) (2,4) (5,6) (5,7) — önceki `maps/truth.txt` | [map3.jpg](obstacleMapReferences/map3/map3.jpg) | [map3.txt](obstacleMapReferences/map3/map3.txt) |
| map4 | map3 + (4,1) (5,1) | [map4.png](obstacleMapReferences/map4/map4.png) | [map4.txt](obstacleMapReferences/map4/map4.txt) |

```text
 map4 (en dolu)              map3 (laboratuvarda kurulan)
 y                            y
 8   1 1 1 1 1 1 1            8   1 1 1 1 1 1 1
 7   1 0 0 0 0 1 1            7   1 0 0 0 0 1 1
 6   1 0 0 0 0 1 1            6   1 0 0 0 0 1 1
 5   1 0 0 0 0 0 1            5   1 0 0 0 0 0 1
 4   1 1 1 0 0 0 1            4   1 1 1 0 0 0 1
 3   1 0 0 0 0 0 1            3   1 0 0 0 0 0 1
 2   1 0 0 0 0 0 1            2   1 0 0 0 0 0 1
 1   1 * 0 0 1 1 1            1   1 * 0 0 0 0 1
 0   1 1 1 1 1 1 1            0   1 1 1 1 1 1 1
     0 1 2 3 4 5 6  x             0 1 2 3 4 5 6  x
```

**Yeni harita eklemek:** Launcher → **Haritalar** sekmesi: bir haritayı kopyalayıp hücrelere tıklayarak düzenleyin ve kaydedin (map5 … map8). Elle de olur: `obstacleMapReferences/map5/` klasörüne görseli ve aynı biçimde `map5.txt` dosyasını koyun (`name map5`, `coords x-y 0`, `cell 0.40`, `border wall`, `heading 90`, `*` başlangıç). Launcher klasörü kendiliğinden bulur.

- **Kontrol:** Robot (1,1)'de +y'ye bakarken ön sensör map3/map4'te **84–86 cm** okur. Hesaplanan değer 89 cm'dir: (1,4) duvarı 1,6 m'de, robot merkezi 0,6 m'de, sensör merkezden 11 cm önde.
- map1'de önü 3,2 m boştur.

## 6. Donanım ve portlar

| Parça | Port | Not |
|---|---|---|
| Kobuki | laboratuvarda **COM6** | FTDI, `FTDIBUS\VID_0403+PID_6001+KOBUKI_AH02IGLCA` |
| Arduino Uno (CH340) | laboratuvarda **COM14**, başka USB girişinde **COM15** | `VID_1A86&PID_7523`. COM numarası takılan USB girişine göre değişir. |
| HC-SR04 | Trig **D11**, Echo **D12** (varsayılan; launcher'dan değiştirilebilir), VCC 5V, GND GND | Robot merkezinden ~11 cm önde, yere paralel, tam ileri. |

- **COM numaraları bilgisayara göre değişir.** Launcher'da port seçilince port dinlenir: Kobuki paketleri (batarya voltajıyla) ya da Arduino JSON satırları (ön mesafe, Hz) tanınır; ters seçim, yanlış sketch, sessiz ya da meşgul port ayrı ayrı bildirilir. **Portları tara** ikisini de gönderdikleri veriden bulur ([Launcher/src/Probe.h](Launcher/src/Probe.h)).
- Başka bir bilgisayarda COM5/COM6 com0com gibi sanal portlar olabilir. Elle port vermeden önce listeye bakın.

**Arduino programı:** [arduino/sensor/sensor.ino](arduino/sensor/sensor.ino) (v2). Filtresiz, ~10 Hz, 115200 baud gönderir:

```text
{"l":-1,"f":86,"r":-1,"us":5015}
```

- `f`: cm. `us`: yankı süresi (µs), 40 cm kalibrasyonu bunu kullanır.
- `?` ve `PINS <trig> <echo>` komutlarını tanır. Pinler EEPROM'da saklanır; launcher 3. adımda **Karta yaz** bunu yapar.
- Eski sketch (yalnız `f`) ile de her şey çalışır; yalnız pin değiştirilemez ve kalibrasyon tam cm'lerle yapılır.
- Ayrıntı: [USER_GUIDE.md](USER_GUIDE.md) §2.1 ve §3.

Karta başka bir program yüklüyse (örneğin düz `84` gibi sayılar geliyorsa) program "No sensor data" der.

**Komut satırından yükleme:**
- arduino-cli laboratuvar bilgisayarında `.tools\arduino-cli\arduino-cli.exe` konumunda; `.tools/` gitignore'dadır.
- Başka bilgisayarda arduino-cli'yi kurun ya da Arduino IDE'yi kullanın.

Git Bash'te:

```bash
./.tools/arduino-cli/arduino-cli.exe compile --fqbn arduino:avr:uno 11_ExplorationMapping/arduino/sensor
./.tools/arduino-cli/arduino-cli.exe upload --verify -p COM14 --fqbn arduino:avr:uno 11_ExplorationMapping/arduino/sensor
```

**Portları ve sensör akışını kontrol etme** (PowerShell):

```powershell
Get-PnpDevice -Class Ports -PresentOnly | Select Status, FriendlyName, InstanceId
$p = New-Object System.IO.Ports.SerialPort 'COM14',115200,'None',8,'One'; $p.DtrEnable=$true; $p.Open()
Start-Sleep 3; $p.ReadExisting(); $p.Close()
```

## 7. Gerçek robot denemeleri (23.11.2026, yöntem: 11_01 Frontier)

Her iki denemede de robot (1,1)'deydi ve +y'ye bakıyordu; alan map3 idi. Varsayılan sensör modeli kullanıldı (kalibrasyon yok).

### Deneme 1: 246 s, 7,8 m, 0 tampon (elle durduruldu)

```text
           0     1     2     3     4     5     6
   8   0.50? 0.51? 0.99# 0.99# 0.91# 0.50? 0.50?
   7   0.92# 0.99# 0.99# 0.99# 0.50? 0.50? 0.50?
   6   0.99# 0.01  0.01  0.01  0.99# 0.99# 0.50?
   5   0.99# 0.01  0.01  0.83# 0.01  0.96# 0.52?
   4   0.83# 0.99# 0.99# 0.99# 0.01  0.46? 0.93#
   3   0.99# 0.01  0.99# 0.01  0.01  0.01  0.99#
   2   0.94# 0.01  0.01  0.01  0.01  0.01  0.99#
   1   0.99# 0.01  0.01  0.96# 0.99# 0.94# 0.99#
   0   0.96# 0.99# 0.99# 0.99# 0.86# 0.50? 0.50?
```

- **Yanlış "dolu" çıkanlar:** (1..3,7), (4,6), (3,5), (5,5), (3,4), (2,3), (3..5,1).
- 7. satır ve (3,4) geçidi sürekli dolu ↔ kararsız arasında gidip geldi; robot hep yeniden planladı.
- 225. saniyede robot gerçekte duvar olan (2,4)'e girmek üzereydi; o anda durduruldu.

### Deneme 2: 412 s, 12,9 m, 7 tampon (elle durduruldu)

```text
           0     1     2     3     4     5     6
   8   0.95# 0.99# 0.99# 0.99# 0.99# 0.50? 0.50?
   7   0.99# 0.01  0.01  0.01  0.99# 0.80? 0.50?
   6   0.86# 0.01  0.01  0.01  0.01  0.99# 0.50?
   5   0.93# 0.99# 0.01  0.05  0.01  0.99# 0.67?
   4   0.51? 0.99# 0.99# 0.05  0.02  0.99# 0.67?
   3   0.99# 0.01  0.01  0.01  0.01  0.99# 0.99#
   2   0.99# 0.01  0.81# 0.01  0.01  0.01  0.99#
   1   0.99# 0.99# 0.99# 0.99# 0.99# 0.99# 0.98#
   0   0.94# 0.99# 0.99# 0.71? 0.50? 0.50? 0.50?
```

- **Yanlış "dolu" çıkanlar:** 1. satırın tamamı (1..5,1), başlangıç hücresi dahil; 5. sütun (5,3), (5,4), (5,5); (1,5), (4,7), (2,2).
- **Doğru çıkanlar:** Üst bölge (7. satır bu sefer doğru), geçit (3,4)–(4,4), dış duvarlar, (1,4) ve (2,4). (2,4)'ü sonar kaçırdı, tampon yakaladı.
- **Tampon temasları:**
  - (5,6) duvarına 2 kez: (4,6) duvara çok yakın, yanda ~2,5 cm boşluk var.
  - (2,4)'e 1 kez.
  - (2,2) yönüne 3 kez. Resme göre (2,2) boş; sahada orada bir nesne olup olmadığı doğrulanmadı.
- Bu denemeden sonra tampon hatası düzeltildi: tampon hücreyi doğrudan **P ≥ 0,90**'a çeker (`BayesGrid::markBump`). Düzeltme gerçek robotta henüz denenmedi.

### Gözlenen örüntü ve olası sebepler (henüz doğrulanmadı)

İki denemede de alt duvar (y = 0) ve sağ duvar (x = 6) robota yaklaşık bir hücre yakın göründü; 1. satır ve 5. sütun "dolu" çıktı. Simülasyon aynı alanda 0 hata veriyor, yani fark gerçek ortamdan ya da sensörden geliyor. Kontrol edilecekler:

1. **Sensör aşağı eğik mi?** Eğikse zemin yankısı alır ve robotun 1–2 hücre önünü dolu gösterir.
2. **Duvarlar:** Gerçekte 40 cm'lik hücreyi dolduran kutular mı, ince panolar mı? Yerleri resimdeki gibi mi?
3. **Geniş koni:** 360° taramada ±12°'lik koni yan duvarları ortadaki hücrelere yazıyor olabilir. Gerekirse `hit_range` (1,0 m) ya da `cone_half_deg` küçültülebilir; ikisi de `sensor_model.txt` ile ayarlanabilir.
4. **Odometri kayması:** Çok sayıda dönüş ve geri çekilmeden sonra birikebilir. Deneme 1'de robot kendini bir hücre kaymış sanıyordu.
5. **Kalibrasyon yapılmadı.**

## 8. Açık sorun: Arduino (23.11.2026, deneme 2'den sonra)

O oturumun sonunda gerçek robot başlatılamadı.

1. Gün içinde karta iki kez başka bir program yüklenmişti; JSON yerine düz sayı (`18`, `84`) geliyordu. İkisinde de `sensor.ino` yeniden yüklendi ve JSON geldi.
2. Deneme 2'den sonra kart **tamamen sessizleşti**: hiçbir baud hızında tek bayt gelmedi.
3. Bundan sonra her yükleme `flash verification mismatch` hatasıyla bitti: `device 0x62 != input 0x0c at addr 0x0000`. Her bayt `0x62` okundu. `--verify` olmadan yükleme "başarılı" görünüyor ama program çalışmıyor.
4. USB'yi çıkarıp takmak sonucu değiştirmedi. Kart başka girişe takılınca COM15 oldu, sonuç aynı.
5. Son kontrolde bilgisayar hiçbir CH340 görmüyordu (COM14 ve COM15 "Unknown").

Sırayla bakılacaklar:
1. **Bağlantı:** Launcher → Gerçek robot → **Portları tara**: Arduino görünüyor mu, rozet ne diyor? (Ya da `Get-PnpDevice -Class Ports -PresentOnly`.)
   - Görünmüyorsa: kablo ya da USB girişi. Hub yerine doğrudan bilgisayara takın.
2. **Akış:** Bağlıysa önce yükleme yapmadan 3 s dinleyin (§6).
3. **Yükleme:** Her zaman `--verify` ile yapın. Yine `0x62` çıkarsa:
   - Sensörün 4 kablosunu söküp yalnızca USB ile deneyin.
   - Olmazsa ikinci bir Arduino ile ArduinoISP üzerinden chip erase ve bootloader yazımı yapın.

## 9. Sonraki adımlar

1. Arduino bağlantısını çözün (§8). JSON akışını ve 84–86 cm okumayı doğrulayın.
2. **Yeni sensor.ino'yu (v2) yükleyin** (launcher → 3. adım → **Sketch'i yükle**), pinleri doğrulayın ve **40 cm kalibrasyonunu** yapın (`sensor_calibration.txt`); 80 cm'de **Kontrol et** ile doğrulayın.
   - Önce **Elle sürüş** panelinde tampon testini ve "İleri 40 cm" / "Sola 90°" hareketlerini deneyin.
3. **Gauss kalibrasyonu** (komut satırından), hücreler `1,2,3,5,6`.
   - `.\run.ps1 11 -Calibrate -ArduinoPort COM14 --cal-cells 1,2,3,5,6`.
   - Sonuç `sensor_model.txt` dosyasına yazılır; **dört yöntem de** iki dosyayı da kullanır.
4. Sensörün yere paralel olduğunu kontrol edin.
5. **Gerçek robot karşılaştırması:** Launcher'da *Gerçek robot*, iki yeşil ✓, harita: laboratuvarda kurulan düzen (map1..map4). Her yöntemi her haritada en az bir kez çalıştırın. Sonuçlar *Gerçek robot koşuları* filtresinde aynı üç ölçütle karşılaştırılır.
6. 1. satır ve 5. sütun yine dolu çıkarsa §7'deki 1–4. maddelere bakın.
7. Başarılı haritayı A*'a verin: `.\run.ps1 10 -Map 11_ExplorationMapping\output\<yöntem>\explored_....txt`.
