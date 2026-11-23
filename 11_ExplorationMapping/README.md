# 11_ExplorationMapping — README (çalışma özeti ve devam notları)

Kobuki, laboratuvardaki alanı **haritayı bilmeden** dolaşır. Robotun önünde tek bir HC-SR04 sonar var.

- **Ölçüm:** her durakta yerinde 360° döner.
- **Harita:** her hücre için P(Dolu) olasılığını **Gauss sensör modeli + Bayes kuralı** ile günceller.
- **Gezinme:** frontier hücrelerine **A\*** ile gider; frontier'a döner ve tekrar ölçer.
- **Çıktı:** sonucu 10_AStarPathPlanning'in okuduğu 0/1 txt haritası olarak kaydeder.

Diğer belgeler:
- Teknik ayrıntı: [ExplorationMapping.md](ExplorationMapping.md)
- Kullanım: [USER_GUIDE.md](USER_GUIDE.md)
- A* örneğinin özeti: [../10_AStarPathPlanning/OZET.md](../10_AStarPathPlanning/OZET.md)

Son güncelleme: **23.11.2026**. Branch: `exploration-11`.

---

## 1. Kısa durum

| Konu | Durum |
|---|---|
| Algoritma (Bayes ızgarası, Gauss model, frontier, A\*, doğrulama) | tamam |
| Simülasyon, yeni 7 × 9 alan | **57 doğru, 0 yanlış**, 6 kararsız (görülemeyen köşeler), ~6 dk, 0 çarpma |
| Simülasyon, eski 8 × 11 labirent | 73 doğru, 0 yanlış |
| Gerçek robot, deneme 1 | 246 s; iç bölgenin çoğu doğru, 1. satır ve sağ taraf yanlış "dolu" (§6) |
| Gerçek robot, deneme 2 | 412 s; 20 boş hücre doğru, 1. satır tamamen ve 5. sütun yanlış "dolu", 7 tampon teması (§6) |
| Tampon hatası | düzeltildi, derlendi, **gerçek robotta henüz denenmedi** (§5) |
| Kalibrasyon (`sensor_model.txt`) | **yapılmadı**; varsayılan model kullanılıyor (§8) |
| Arduino | **sorunlu**: yükleme doğrulanamıyor, kart USB'den düşüyor (§7) |

---

## 2. Alan (laboratuvar)

- 7 sütun × 9 satır, hücreler 40 cm.
- Koordinatlar **(x, y)**, 0'dan başlar. x sağa, **y yukarı** artar: alt satır y = 0, üst satır y = 8.
- Kırmızı = engel (`1`), beyaz = boş (`0`).
- Başlangıç **(1,1)**, robot **+y yönüne** (8. satıra doğru, heading 90°) bakıyor.

```text
 y
 8   1 1 1 1 1 1 1
 7   1 0 0 0 0 1 1
 6   1 0 0 0 0 1 1
 5   1 0 0 0 0 0 1
 4   1 1 1 0 0 0 1
 3   1 0 0 0 0 0 1
 2   1 0 0 0 0 0 1
 1   1 * 0 0 0 0 1      * = başlangıç (1,1)
 0   1 1 1 1 1 1 1
     0 1 2 3 4 5 6  x
```

- 31 boş, 32 dolu hücre.
- [maps/arena.txt](maps/arena.txt): robotun bildiği tek şey alanın boyutu, başlangıç ve yön. Hücreler `?`.
- [maps/truth.txt](maps/truth.txt): yukarıdaki gerçek harita. **Yalnızca simülatör** kullanır.
- Kontrol: robot (1,1)'de +y'ye bakarken ön sensör **84–86 cm** okur. Hesaplanan değer 89 cm: (1,4) duvarı 1,6 m'de, robot merkezi 0,6 m'de, sensör merkezden 11 cm önde.

---

## 3. Donanım ve portlar

| Parça | Port | Not |
|---|---|---|
| Kobuki | **COM6** | FTDI, `FTDIBUS\VID_0403+PID_6001+KOBUKI_AH02IGLCA` |
| Arduino Uno (CH340) | **COM14**, başka USB girişinde **COM15** | `VID_1A86&PID_7523`. COM numarası takılan USB girişine göre değişir. |
| HC-SR04 | Trig **D11**, Echo **D12**, VCC 5V, GND GND | robot merkezinden ~11 cm önde, yere paralel, tam ileri |

**Arduino programı:** [arduino/sensor/sensor.ino](arduino/sensor/sensor.ino). Filtresiz, ~10 Hz, 115200 baud gönderir:

```text
{"l":-1,"f":86,"r":-1}
```

Karta başka bir program yüklüyse (örneğin düz `84` gibi sayılar geliyorsa) Explore "No sensor data" der.

**Komut satırından yükleme.** arduino-cli bu bilgisayarda `.tools\arduino-cli\arduino-cli.exe` konumunda, `arduino:avr` 1.8.8 kurulu. `.tools/` gitignore'da olduğu için başka bir bilgisayarda yoktur; orada arduino-cli'yi kurun ya da Arduino IDE'yi kullanın. Git Bash'te:

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

---

## 4. Derleme ve çalıştırma

```powershell
# derleme (x64-release)
cmd /c '"C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul && cmake --preset x64-release && cmake --build out/build/x64-release --target Explore AStar'

# simülasyon (pencereli, gerçek zamanlı)
.\out\build\x64-release\11_ExplorationMapping\Explore.exe --sim
# simülasyon, hızlı ve penceresiz
.\out\build\x64-release\11_ExplorationMapping\Explore.exe --sim --no-gui --fast

# gerçek robot: Kobuki COM6, Arduino COM14; Enter beklemeden 3 s sonra başlar
.\out\build\x64-release\11_ExplorationMapping\Explore.exe COM6 COM14 --go

# run.ps1 ile aynısı (derler, portları otomatik bulur)
.\run.ps1 11 -Sim
.\run.ps1 11 -Port COM6 -ArduinoPort COM14 --go
```

- **Gerçek robot ve simülasyon birlikte:** iki `Explore.exe` aynı anda çalışabilir. Ancak biri açıkken yeniden derleme `LNK1104` (exe kilitli) hatası verir.
- **Robotu durdurma:** pencerede `Q` ya da Esc'e basın veya pencereyi kapatın; robot durur ve harita kaydedilir. Uzaktan durdurmak için:
  ```powershell
  (Get-Process -Id <pid>).CloseMainWindow()
  ```
  Exit code 2 = "window closed"; bu bir hata değil.
- **Çıktılar** `11_ExplorationMapping\output\` klasörüne yazılır (git'e girmez):
  - `explored_<zaman>.txt`: 0/1 harita;
  - `_prob.txt`: her hücrenin P(Dolu) değeri;
  - `.png`: pencerenin resmi.
- **Haritayı A\* ile kullanma:** txt'de hedef hücreye `/` yazın, sonra `.\run.ps1 10 -Map 11_ExplorationMapping\output\explored_<zaman>.txt` çalıştırın.

---

## 5. Bu çalışmada yapılanlar

### Önceki oturumlarda
- **10_AStarPathPlanning:** lab labirenti (8 × 11, başlangıç (1,7)), `*` başlangıç, `/` hedef, 15 cm robot payı. Gerçek robotla hedefe ulaşıldı. [OZET.md](../10_AStarPathPlanning/OZET.md) eklendi.
- **Ortak başlıklar** `01_Sample/KobukiNativeVS/include/` altına taşındı:
  - `NavCommon.h` (Pose, açı yardımcıları);
  - `PathFollower.h` (10'dan taşındı);
  - `AStarPlanner.h` (harita okuyucu; `?` = bilinmiyor).
- **11_ExplorationMapping** sıfırdan yazıldı:
  - `BayesGrid.h`: Gauss sensör modeli (Method B), P(z|E) deneyi (Method A), Bayes güncellemesi, koni/bölge modeli, frontier.
  - `Explorer.h`: 360° tarama, frontier seçimi, A\*, karşıdan ölçüm, doğrulama, tampon ve geri çekilme.
  - `ExploreRobot.h`: gerçek Kobuki + Arduino ve simülatör.
  - `ExploreView.h`: pencere ve PNG.
  - `main.cpp`: kalibrasyon modu, keşif döngüsü, kayıt.
- **Önce 3 sensörle** (sol 45°, ön, sağ 45°) başlandı, sonra kullanıcının isteğiyle **tek sensöre ve 360° taramaya** geçildi.
- **Slayttaki 4 m örneği:** P(z|O) = 0,649, P(z|E) = 0,20. Tek ölçümden sonra isabet → **0,76**, isabet yok → **0,30**. Program bu tabloyu her başlangıçta yazar.

### Bu oturumda (23.11.2026)
1. **Yeni alan:** 7 × 9, başlangıç (1,1), heading 90°. [maps/arena.txt](maps/arena.txt) yeniden yazıldı, [maps/truth.txt](maps/truth.txt) eklendi. Simülatörün varsayılan gerçek haritası artık `maps/truth.txt` (`CMakeLists.txt` → `LAB_MAP`).
2. **Yeni koordinat biçimi `coords x-y 0`:** sütun önce, y alttan yukarı sayılır.
   - `AStarPlanner.h` → `CoordStyle`: `yUp`, `rows`, `colLabel()`, `rowLabel()`, `keyword()` eklendi.
   - Pencere, txt çıktısı, olasılık tablosu ve 10'un görünümü bu etiketleri kullanıyor. Eski `row-col` / `col-row` dosyaları aynen çalışır.
3. **Sığ açı kuralı** (`BayesGrid.h`, `grazingDeg = 50°`):
   - Sorun: simülasyonda (6,4) duvarı yanlışlıkla "boş" çıkıyordu. Işın duvara sığ açıyla gelince yankı dönmüyordu ve program hücreyi "ışın içinden geçti" sayıyordu.
   - Kural: ışın bir hücreye yüzüne 50°'den sığ açıyla giriyorsa orada durur.
   - Sonuç: yeni alanda 0 yanlış; eski labirentte de 0 yanlış kaldı.
4. **Tampon düzeltmesi** (`BayesGrid::markBump`):
   - Sorun: tampon değdiği hücreyi tek bir Bayes adımıyla işliyordu. Sonar hücreyi P = 0,01'e indirmişse P yalnızca 0,16'ya çıkıyordu; hücre "boş" kalıyor ve robot aynı yere tekrar çarpıyordu (deneme 2'de (2,4) ve (2,2)).
   - Şimdi: tampon hücreyi doğrudan **P ≥ 0,90**'a çeker. Karşıdan ölçüm hücreyi gerekirse yine boşa çevirebilir.
5. Belgeler güncellendi: USER_GUIDE.md, ExplorationMapping.md ve bu README.

---

## 6. Gerçek robot denemeleri (23.11.2026)

Her iki denemede de robot (1,1)'deydi ve +y'ye bakıyordu. Varsayılan sensör modeli kullanıldı (kalibrasyon yok).

### Deneme 1: 246 s, 7,8 m, 0 tampon (ben durdurdum)

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
- 225. saniyede robot gerçekte duvar olan (2,4)'e girmek üzereydi; o anda durdurdum.

### Deneme 2: 412 s, 12,9 m, 7 tampon (ben durdurdum)

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
- **Doğru çıkanlar:** üst bölge (7. satır bu sefer doğru), geçit (3,4)–(4,4), dış duvarlar, (1,4) ve (2,4). (2,4)'ü sonar kaçırdı, tampon yakaladı.
- **Tampon temasları:**
  - (5,6) duvarına 2 kez: (4,6) duvara çok yakın, yanda ~2,5 cm boşluk var.
  - (2,4)'e 1 kez.
  - **(2,2) yönüne 3 kez.** Resme göre (2,2) boş; sahada orada bir nesne olup olmadığı sorulmadı ya da cevaplanmadı.
- Tampon hatası (§5.4) bu denemeden sonra düzeltildi.

### Gözlenen örüntü ve olası sebepler (henüz doğrulanmadı)

İki denemede de **alt duvar (y = 0) ve sağ duvar (x = 6) robota yaklaşık bir hücre yakın** göründü. 1. satır ve 5. sütun "dolu" çıktı. Simülasyon aynı alanda 0 hata veriyor, yani fark gerçek ortamdan ya da sensörden geliyor. Kontrol edilecekler:

1. **Sensör aşağı eğik mi?** Eğikse zemin yankısı alır ve robotun 1–2 hücre önünü dolu gösterir.
2. **Duvarlar** gerçekte 40 cm'lik hücreyi dolduran kutular mı, ince panolar mı? Yerleri resimdeki gibi mi?
3. **Geniş koni:** 360° taramada ±12°'lik koni yan duvarları ortadaki hücrelere yazıyor olabilir. Gerekirse `hitRange` (1,0 m) ya da `coneHalfDeg` küçültülebilir.
4. **Odometri kayması:** çok sayıda dönüş ve geri çekilmeden sonra birikebilir. Deneme 1'de robot kendini bir hücre kaymış sanıyordu.
5. **Kalibrasyon yapılmadı** (§8).

---

## 7. AÇIK SORUN: Arduino (deneme 2'den sonra)

Bu oturumun sonunda gerçek robot **başlatılamadı**.

**Zaman sırasıyla:**
1. Bugün karta iki kez başka bir program yüklenmiş; JSON yerine düz sayı (`18`, `84`) geliyordu. İkisinde de `sensor.ino` başarıyla yeniden yüklendi ve JSON geldi.
2. Deneme 2'den sonra kart **tamamen sessizleşti**: hiçbir baud hızında tek bayt gelmedi.
3. Bundan sonra her yükleme şu hatayla bitti:
   ```text
   Device signature = 1E 95 0F (ATmega328P, ATA6614Q, LGT8F328P)
   Warning: flash verification mismatch
     device 0x62 != input 0x0c at addr 0x0000 (error)
   ```
   Her bayt `0x62` okundu. `--verify` olmadan yükleme "başarılı" görünüyor ama program çalışmıyor; pinlere hiç dokunmayan "hello" test programı bile sessiz kaldı.
4. USB çıkarıp takmak sonucu değiştirmedi. Kart başka girişe takılınca **COM15** oldu, sonuç aynı.
5. Son kontrolde **bilgisayar hiçbir CH340 görmüyordu**: COM14 ve COM15 "Unknown" (bağlı değil) durumundaydı. 15 dakika boyunca da görünmedi.

**Kullanıcı kartın bozuk olmadığını söylüyor.** Bir sonraki oturumda sırayla şunlara bakın:

1. `Get-PnpDevice -Class Ports -PresentOnly` ile Arduino gerçekten bağlı mı, hangi COM'da?
   - Görünmüyorsa: kablo veya USB girişi. Başka kablo deneyin, hub yerine doğrudan bilgisayara takın.
   - Arduino robotun üstündeyse kablo hareketle kopuyor olabilir.
2. Bağlıysa önce yüklemeden, 3 s dinleyin (§3). Akış zaten geliyor olabilir.
3. Yükleme her zaman `--verify` ile yapılsın.
   - Yine `0x62` çıkarsa: temassız kablo, USB güç sorunu ya da sensör kablajında kısa devre olabilir. Sensörün 4 kablosunu söküp yalnızca USB ile deneyin.
   - Hâlâ olmazsa: kilit bitleri değişmiş olabilir. İkinci bir Arduino ile ArduinoISP üzerinden chip erase ve bootloader yazımı gerekir.
4. Karta Arduino IDE'den ya da başka bir bilgisayardan başka program yüklenmesin.

---

## 8. Sonraki adımlar

1. Arduino bağlantısını çözün (§7), JSON akışını ve 84–86 cm okumayı doğrulayın.
2. **Kalibrasyon.** Alan yalnızca 7 hücre uzunluğunda olduğu için:
   ```powershell
   .\run.ps1 11 -Calibrate -ArduinoPort COM14 --cal-cells 1,2,3,5,6
   ```
   Kutu sırayla 1, 2, 3, 5 ve 6 hücre öteye konur, ardından boş ölçüm alınır. Sonuç `sensor_model.txt` dosyasına yazılır.
3. Sensörün yere paralel olduğunu kontrol edin; (2,2) çevresinde yerde nesne olmasın.
4. Gerçek robotu (1,1)'den, +y'ye bakarken tekrar çalıştırın. Tampon düzeltmesi artık devrede.
   ```powershell
   .\out\build\x64-release\11_ExplorationMapping\Explore.exe COM6 COM14 --go
   ```
5. 1. satır ve 5. sütun yine dolu çıkarsa §6'daki 1–4. maddeler: koni ve `hitRange` ayarı, sensör eğimi, odometri.
6. Başarılı haritayı `.\run.ps1 10 -Map ...` ile A\*'a verin.

---

## 9. Dosyalar

| Dosya | İçerik |
|---|---|
| [maps/arena.txt](maps/arena.txt) | Keşif alanı: 7 × 9, `coords x-y 0`, `*` (1,1), heading 90, hücreler `?` |
| [maps/truth.txt](maps/truth.txt) | Simülatör için gerçek harita |
| [arduino/sensor/sensor.ino](arduino/sensor/sensor.ino) | Tek HC-SR04 (D11/D12), ham ölçüm, JSON |
| [src/BayesGrid.h](src/BayesGrid.h) | Sensör modeli, Bayes, ışın işleme, sığ açı kuralı, tampon, frontier |
| [src/Explorer.h](src/Explorer.h) | Keşif durum makinesi |
| [src/ExploreRobot.h](src/ExploreRobot.h) | Gerçek robot ve simülatör |
| [src/ExploreView.h](src/ExploreView.h) | Pencere ve PNG |
| [src/main.cpp](src/main.cpp) | Seçenekler, kalibrasyon, keşif, kayıt |
| `../01_Sample/KobukiNativeVS/include/AStarPlanner.h` | Harita okuyucu (`coords x-y`), A\* |
| `../01_Sample/KobukiNativeVS/include/PathFollower.h`, `NavCommon.h` | Yol takibi, ortak tipler (10 ile ortak) |
| `../.tools/arduino-cli/` | Komut satırından Arduino derleme ve yükleme |
