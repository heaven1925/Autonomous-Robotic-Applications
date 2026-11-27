# Autonomous Robotic Applications — Kobuki (Windows, C++20, ROS'suz)

Kobuki mobil robotu için ders örnekleri: temel hareketten engelden kaçınmaya, haritalamaya, yol planlamaya ve keşfe. Her şey doğrudan Windows'ta çalışır; ROS yoktur. Robotla seri porttan Kobuki protokolüyle konuşulur, mesafe sensörleri (HC-SR04) bir Arduino üzerinden okunur.

Güncel çalışma **[11_ExplorationMapping](11_ExplorationMapping/README.md)** üzerindedir: dört keşif yöntemi, bunları seçip çalıştıran ve karşılaştıran bir Windows uygulaması (ExploreLauncher). Ayrıntı için [aşağıdaki bölüme](#11-exploration-mapping-güncel-çalışma) bakın.

## Hızlı başlangıç

Gerekenler: Windows 10/11, Visual Studio (C++ araçları, MSVC) ve CMake. Depo kökünde PowerShell açın:

```powershell
.\run.ps1 launcher            # 11: ExploreLauncher'ı derler ve açar (donanım gerekmez, simülasyon var)
.\run.ps1 11 -Sim             # 11_01 Frontier, simülatörde
.\run.ps1 10 -Sim             # A* yol planlama, simülatörde
.\run.ps1 1                   # Example01 (kare), gerçek robot; Kobuki portu kendiliğinden bulunur
.\run.ps1 stop                # çalışan robot programını durdur
.\run.ps1 -ListPorts          # COM portlarını listele
```

- `run.ps1` hedefi derler, önceki robot programını kapatır (COM portu boşalır) ve programı çalıştırır. Bütün seçenekler dosyanın başında yazılıdır.
- Derleme: CMake ön ayarları `x64-debug` ve `x64-release` (Ninja + MSVC). Çıktılar `out/build/<ön ayar>/` altındadır.
- VS Code: **Terminal → Run Task** altında "Robot: Build & Run", "Robot: 11 Exploration Launcher", "Robot: List COM ports".
- Launcher'ı çift tıklayarak açmak için: `11_ExplorationMapping\ExploreLauncher.cmd`.

## Örnekler

| Klasör | `run.ps1` | Konu | Belge |
|---|---|---|---|
| [01_Sample](01_Sample/README.md) | `sample` | Sürücü kütüphanesi (KobukiDriver) ve klavyeyle sürüş örneği. Diğer örneklerin ortak başlıkları burada: `01_Sample/KobukiNativeVS/include`. | [README](01_Sample/README.md) |
| 02_Example01 | `1` | Kare çizme | |
| 03_Example02 | `2` | Git-gel ve yerinde dönüş | |
| 04_Example03 | `3` | Ara noktalarla sürüş | |
| 05_Example04 | `4` | Ara noktalar + tamponla engelden kaçınma | |
| 06_FollowTheGap | `6` | Follow the Gap (HC-SR04 + Arduino) | [mimari](06_FollowTheGap/Kobuki_Follow_the_Gap_Sensor_Mimarisi.md) |
| 07_Bug0Algorithm | `7` | Bug0 (3 × HC-SR04) | [Bug0.md](07_Bug0Algorithm/Bug0.md) |
| 08_TangentBugAlgorithm | `8` | TangentBug; simülatör ve sensör testi var | [TangentBug.md](08_TangentBugAlgorithm/TangentBug.md), [kılavuz](08_TangentBugAlgorithm/USER_GUIDE.md) |
| 09_Mapping | `9` | Doluluk ızgarası haritalama (3 × HC-SR04) | [Mapping.md](09_Mapping/Mapping.md), [kılavuz](09_Mapping/USER_GUIDE.md) |
| 10_AStarPathPlanning | `10` | A* yol planlama ve yolu sürme | [AStar.md](10_AStarPathPlanning/AStar.md), [özet](10_AStarPathPlanning/OZET.md), [kılavuz](10_AStarPathPlanning/USER_GUIDE.md) |
| **11_ExplorationMapping** | `launcher`, `11` | Bayes doluluk ızgarasıyla keşif: 4 yöntem + launcher | [README](11_ExplorationMapping/README.md), [kılavuz](11_ExplorationMapping/USER_GUIDE.md), [teknik](11_ExplorationMapping/ExplorationMapping.md) |

## Donanım

| Parça | Bağlantı | Not |
|---|---|---|
| Kobuki | USB seri (FTDI), 115200 baud | Laboratuvarda COM6. Geri bildirim paketleri ~50 Hz. |
| Arduino Uno (CH340) | USB seri, 115200 baud | Laboratuvarda COM14 / COM15. COM numarası takılan USB girişine göre değişir. |
| HC-SR04 | Arduino dijital pinleri | Örnek 11'de tek sensör, önde; varsayılan Trig D11, Echo D12. |

COM numaraları bilgisayara göre değişir. Bazı bilgisayarlarda sanal (com0com) ve Bluetooth portları da listede görünür; bunlar robot değildir. Örnek 11'in launcher'ı portları gönderdikleri veriden tanır.

---

## 11 Exploration Mapping (güncel çalışma)

Robot, 7 × 9 hücrelik (hücre 40 cm, bloklar 40 × 40 cm) laboratuvar alanını haritayı bilmeden dolaşır. Önündeki tek HC-SR04 ile ölçer; her hücrenin dolu olma olasılığını Gauss sensör modeli ve Bayes kuralıyla günceller. Sonuçta çıkan harita A* örneğine (10) verilebilir.

### Dört yöntem, tek çekirdek

Keşif döngüsü (tarama, sürüş, karşıdan ölçüm, doğrulama, tampon) ortaktır: [common/](11_ExplorationMapping/common/). Yöntemler yalnızca "sıradaki hareket nereye?" sorusunu farklı cevaplar ([common/Strategy.h](11_ExplorationMapping/common/Strategy.h)).

| Yöntem | Program | Kural |
|---|---|---|
| [11_01 Frontier](11_ExplorationMapping/11_01_Frontier/README.md) | `Explore_Frontier.exe` | A* yolu en kısa olan frontier hücresi |
| [11_02 BFS](11_ExplorationMapping/11_02_BFS/README.md) | `Explore_BFS.exe` | Başlangıçtan genişlik öncelikli sıra |
| [11_03 Random](11_ExplorationMapping/11_03_Random/README.md) | `Explore_Random.exe` | Rastgele ulaşılabilir hücre + 360° tarama (temel karşılaştırma) |
| [11_04 TSP](11_ExplorationMapping/11_04_TSP/README.md) | `Explore_TSP.exe` | Bütün frontier hücrelerinden geçen en kısa tur |

### Karşılaştırma ölçütleri

1. **Hız:** haritayı tamamlama süresi.
2. **Başarı oranı:** doğru karar verilen hücre / görülebilir hücre.
3. **Engel haritası sayısı:** kaç farklı haritada başarılı olduğu (map1 – map4, [obstacleMapReferences/](11_ExplorationMapping/obstacleMapReferences/)).

Simülasyon sonucu (24.11.2026; 4 yöntem × 4 harita × 3 tekrar = 48 koşu, gürültülü sensör):

| Sıra | Yöntem | Ort. süre | Başarı oranı | Başarılı harita |
|---:|---|---:|---:|---:|
| 1 | 11_04 TSP | 323 s | 100,0 % | 4 / 4 |
| 2 | 11_01 Frontier | 330 s | 100,0 % | 4 / 4 |
| 3 | 11_02 BFS | 358 s | 100,0 % | 4 / 4 |
| 4 | 11_03 Random | 639 s | 98,8 % | 4 / 4 |

Gerçek robotta dört yöntemin karşılaştırması **henüz yapılmadı**. Ayrıntı ve harita bazında tablo: [11_ExplorationMapping/README.md](11_ExplorationMapping/README.md) §4.

### ExploreLauncher

Her şey tek pencereden yönetilir. Açmak için `.\run.ps1 launcher` ya da `11_ExplorationMapping\ExploreLauncher.cmd` (çift tıklama).

| Sekme | İçerik |
|---|---|
| **Çalıştır** | Dört adım: 1) simülasyon / gerçek robot, 2) Kobuki ve Arduino portları + bağlantı testi, 3) sensör kalibrasyonu, 4) algoritma ve harita. Koşu sırasında harita canlı çizilir (hücrelerde P(Dolu) %; keşif, doğruluk, başarı oranı göstergeleri). |
| **Karşılaştırma** | Simülasyonda toplu koşu (benchmark) ve üç ölçütle sıralama; rapor kaydı. |
| **Geçmiş** | Yapılmış koşular. Launcher'dan başlatılanlar kare kare yeniden izlenir. |
| **Haritalar** | Engel haritası düzenleyici (map5 – map8; map1 – map4 salt okunur). |

Gerçek robot için ek paneller (Çalıştır sekmesinin sağ tarafında açılır):

| Panel | Ne yapar |
|---|---|
| **Sensör kalibrasyonu** (3. adım) | HC-SR04'ün Trig / Echo pinlerini seçip karta yazar; anlık okumayı gösterir (yankı süresi µs, ham cm, kalibreli cm); bloğu 40 cm'ye koyarak mesafe ölçeğini kalibre eder; 40 / 80 cm'de doğrular; `sensor.ino`'yu karta yükler. |
| **Hazırlık kontrolü** | "Robotta başlat" önce bunu açar: bağlantılar, batarya, tampon, sensör akışı ve kalibrasyon tek listede. |
| **Elle sürüş ve sensör testi** | Ok tuşları / WASD ile sürüş, 40 cm ve 90° sabit hareketler, Kobuki sensörleri, tampon testi. |

Adım adım kullanım: [11_ExplorationMapping/USER_GUIDE.md](11_ExplorationMapping/USER_GUIDE.md).

### Laboratuvarda tipik sıra

1. Robotu ve Arduino'yu bağlayın, launcher'ı açın, **Gerçek robot**'u seçin.
2. **Portları tara**: iki yeşil ✓ görünmeli.
3. Kartta yeni `sensor.ino` (v2) yoksa 3. adımda **Sketch'i yükle**.
4. **Elle sürüş**: tampon testini ve "İleri 40 cm" / "Sola 90°" hareketlerini deneyin.
5. 3. adım: bloğu sensörün önüne 40 cm'ye koyup kalibre edin; 80 cm'de **Kontrol et**.
6. Yöntemi ve haritayı seçin, blokları haritadaki düzende kurun, robotu (1,1)'e +y yönüne bakacak şekilde koyun.
7. **Robotta başlat** → hazırlık kontrolü → başlat.

### Önemli dosyalar

| Dosya | İçerik |
|---|---|
| [11_ExplorationMapping/common/](11_ExplorationMapping/common/) | Ortak çekirdek: `Strategy.h` (yöntem arayüzü), `Explorer.h` (keşif döngüsü), `BayesGrid.h`, `ExploreRobot.h` (gerçek robot + simülatör), `ExploreApp.h` (program gövdesi), `Ipc.h` (launcher'a canlı akış), `Metrics.h`, `Calibration.h`, `RangeCalibration.h` |
| [11_ExplorationMapping/Launcher/src/](11_ExplorationMapping/Launcher/src/) | `main.cpp` (pencere, Çalıştır ve Karşılaştırma), `Panels.h` (kalibrasyon, elle sürüş, hazırlık kontrolü), `Tabs.h` (geçmiş, haritalar), `Probe.h` (port testi), `SensorLink.h`, `Teleop.h`, `ArduinoCli.h`, `LiveRun.h`, `Replay.h`, `MapFile.h`, `Results.h`, `Theme.h` |
| [11_ExplorationMapping/arduino/sensor/sensor.ino](11_ExplorationMapping/arduino/sensor/sensor.ino) | Sensör sketch'i (v2): JSON + yankı süresi, `?` ve `PINS t e` komutları, pinler EEPROM'da |
| `11_ExplorationMapping/sensor_calibration.txt` | 40 cm mesafe ölçeği (launcher yazar; dört yöntem gerçek robotta okur) |
| `11_ExplorationMapping/sensor_model.txt` | Gauss sensör modeli (`.\run.ps1 11 -Calibrate` yazar) |
| `11_ExplorationMapping/results/`, `output/` | Sonuç tablosu, günlükler, haritalar, koşu kayıtları. Git'e girmez. |

### Durum (27.11.2026)

**Doğrulananlar** (simülasyonda ve bu bilgisayardaki sanal port çiftinde, sahte cihazlarla):
- Dört yöntem dört haritada; 11_01 Frontier'ın davranışı önceki sürümle birebir aynı (map3, seed 2026: 376,7 s, 57 doğru / 0 yanlış / 6 kararsız).
- Launcher'ın bütün sekmeleri ve panelleri: port tanıma, canlı harita, durdurma, benchmark, 40 cm kalibrasyonu ve doğrulaması, pin yazma, hazırlık kontrolü, elle sürüş komutları, koşu kaydı ve tekrar izleme, harita düzenleme.
- Release ve debug derlemeleri uyarısız.

**Henüz doğrulanmayanlar:**
- Yeni launcher ve yeni `sensor.ino` (v2) **gerçek Kobuki ve Arduino ile denenmedi**.
- `sensor.ino` gerçek AVR derleyicisiyle derlenmedi; launcher'daki yükleme akışı yalnız sahte bir arduino-cli ile denendi. arduino-cli kurulumu: `11_ExplorationMapping\arduino\setup-arduino-cli.ps1` ya da launcher'daki **arduino-cli'yi kur** düğmesi.
- Elle sürüşün güvenlik davranışı (tuş bırakınca durma, tamponla kesme) gerçek robotta denenmedi; ilk denemede robotun yanında durun.

**Açık konular** (ayrıntı: [11_ExplorationMapping/README.md](11_ExplorationMapping/README.md) §7 – §9):
- Arduino'ya yükleme sorunu (23.11.2026'da görülen doğrulama hatası) çözülmedi.
- 40 cm kalibrasyonu ve Gauss sensör kalibrasyonu gerçek sensörle yapılmadı.
- Gerçek robot denemelerinde 1. satır ve 5. sütun yanlışlıkla dolu çıkıyordu; sebebi doğrulanmadı.
- Tampon düzeltmesi robotta denenmedi.

**Sıradaki adımlar:**
1. arduino-cli'yi kurup `sensor.ino` v2'yi karta yüklemek.
2. Gerçek robotta launcher akışını baştan sona denemek (bağlantı, elle sürüş, 40 cm kalibrasyonu, 80 cm kontrolü).
3. Gauss sensör kalibrasyonunu yapmak.
4. Dört yöntemi dört haritada gerçek robotta koşturup üç ölçütle karşılaştırmak.

## Depo düzeni

```text
01_Sample/KobukiNativeVS/     KobukiDriver kütüphanesi + ortak başlıklar (include/)
02_Example01 … 05_Example04   temel hareket örnekleri
06_FollowTheGap … 10_AStarPathPlanning   engelden kaçınma, haritalama, yol planlama
11_ExplorationMapping/        4 keşif yöntemi + ExploreLauncher
run.ps1                       derle + çalıştır
CMakeLists.txt, CMakePresets.json
.vscode/tasks.json
```

- Belgeler Türkçedir; kod içi açıklamalar çoğunlukla İngilizcedir.
- `out/`, `.tools/` ve örneklerin `output/`, `results/` klasörleri git'e girmez.
