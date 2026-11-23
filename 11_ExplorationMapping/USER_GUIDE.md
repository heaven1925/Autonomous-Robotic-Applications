# Exploration Mapping — Kullanıcı Kılavuzu (Kobuki + tek HC-SR04)

Robot, laboratuvar alanını haritayı bilmeden dolaşır. Her hücre için P(Dolu) olasılığını Gauss sensör modeli ve Bayes kuralıyla hesaplar. Sonunda haritayı A* (10) için 0/1 txt olarak kaydeder. Teknik ayrıntılar: [ExplorationMapping.md](ExplorationMapping.md).

## 1. Donanım

| Sensör | Yön | Trig | Echo | Güç |
|---|---|---|---|---|
| HC-SR04 | tam ileri | D11 | D12 | VCC → 5V, GND → GND |

- Sensör robot merkezinden yaklaşık 11 cm önde ve yere paralel olmalı. Robot etrafı yerinde 360° dönerek ölçer.
- Pinler 07/08'deki kartın ön kanalıyla aynıdır.
- **Arduino kodu:** [arduino/sensor/sensor.ino](arduino/sensor/sensor.ino). Arduino IDE'de kartı (Uno) ve portu seçip yükleyin. Kod filtresiz (ham) ölçüm gönderir: `{"l":-1,"f":40,"r":-1}`, ~10 Hz, 115200 baud.
- Montaj farklıysa [src/main.cpp](src/main.cpp) başındaki `SensorOffsetM` (0.11) sabitini güncelleyin.

## 2. Adım adım

Depo kökünde PowerShell açın.

### Adım 1: Kalibrasyon (bir kez; sensör ya da zemin değişince tekrar)

```powershell
.\run.ps1 11 -Calibrate                     # yalnızca Arduino; Kobuki kapalı olabilir
.\run.ps1 11 -Calibrate -ArduinoPort COM8   # port otomatik bulunmazsa
.\run.ps1 11 -Sim -Calibrate                # donanımsız deneme
```

1. Robotu bir hücrenin ortasına koyun; sensör ızgara çizgilerine dik baksın.
2. **Gauss (Method B):** program kutuyu sırayla **1, 2, 3, 5 ve 10 hücre** öteye koymanızı ister. Kutunun düz yüzü robota dönük ve hücrenin yakın kenarında olmalı. Her konumda Enter'a basın; 100 ölçüm alınır. 10 hücre yaklaşık 4 m'dir. Alan yetmezse `--cal-cells 1,2,3,5,8` verin.
3. **Boş hücre (Method A):** kutuyu kaldırın, önü boş kalsın, Enter'a basın; 100 ölçüm daha alınır.
4. Program σ(d)'yi, algılama oranını ve P(z|E)'yi hesaplar, 0,5–4 m tablosunu ve slayttaki 4 m örneğini yazar. Sonuç `11_ExplorationMapping\sensor_model.txt` dosyasına kaydedilir.

### Adım 2: Keşif

```powershell
.\run.ps1 11 -Sim       # önce simülatörde (gerçek harita: maps/truth.txt)
.\run.ps1 11            # gerçek robot: Kobuki + Arduino portları otomatik bulunur
.\run.ps1 11 -Port COM6 -ArduinoPort COM14
```

1. Robotun merkezini (1,1) hücresinin ortasına koyun ve +y yönüne (8. satıra doğru, yukarı) baktırın. Koordinatlar (x, y), 0'dan, y yukarı (`coords x-y 0`). Başlangıç ve yön [maps/arena.txt](maps/arena.txt)'de yazar. Doğru duruşta ön sensör ~86 cm okur.
2. Enter'a basın. Beklemeden başlatmak için `--go` ekleyin.
3. Robot 360° döner, frontier seçer, oraya gider, frontier'a dönüp tekrar ölçer; tüm hücreler karar verilene kadar devam eder.
4. Bitince harita kaydedilir.

7 × 9'luk alan simülatörde yaklaşık 6 dakika sürüyor. Güncel durum ve gerçek robot denemeleri: [README.md](README.md).

### Adım 3: Haritayı A* ile kullanma

1. `11_ExplorationMapping\output\explored_<zaman>.txt` dosyasında hedef hücreye `/` yazın.
2. Şunu çalıştırın:
   ```powershell
   .\run.ps1 10 -Map 11_ExplorationMapping\output\explored_<zaman>.txt
   ```

## 3. Ekran

**Pencere:** her hücrede P(Dolu) değeri yazar. Renkler:

| Renk / çerçeve | Anlamı |
|---|---|
| Beyaz | boş (P < 0,3) |
| Gri | kararsız |
| Koyu kırmızı | dolu (P > 0,8) |
| Turkuaz çerçeve | frontier |
| Turuncu çerçeve | hedef |
| Mavi çizgi | A* yolu |
| Yeşil | robot ve sonar ışını |

Sağ panelde sensör modeli (σ(1 m), σ(4 m), P(z|O), P(z|E)), hücre sayıları ve ön sensör değeri görünür.

**Konsolda** başta sensör modeli tablosu, sonra olaylar yazılır, örneğin:
- `frontier (4,4) -> going to (3,4)`
- `verify occupied (7,5) face-on from (6,5)`

`Q` / Esc / Ctrl+C / pencereyi kapatmak robotu durdurur ve o ana kadarki haritayı kaydeder.

**Kaydedilenler** (`11_ExplorationMapping\output\`):
- `explored_<zaman>.txt`: 0/1 harita;
- `explored_<zaman>_prob.txt`: her hücrenin P(Dolu) değeri;
- `explored_<zaman>.png`: pencerenin resmi.

## 4. Program seçenekleri

```text
Explore.exe --calibrate <Arduino COM> [--trials N] [--cal-cells 1,2,3,5,10]
Explore.exe --calibrate --sim
Explore.exe --sim [seçenekler]
Explore.exe <Kobuki COM> <Arduino COM> [seçenekler]
```

| Seçenek | Etki |
|---|---|
| `--arena txt` | Alan dosyası. Varsayılan `maps/arena.txt`. |
| `--model txt` | Sensör modeli. Varsayılan `sensor_model.txt`; yoksa σ(4 m) = 10 cm, P(z\|E) = 0,20. |
| `--truth txt` | Yalnızca `--sim`: gerçek harita. Varsayılan `maps/truth.txt`. |
| `--no-verify` | Frontier bitince dolu hücreleri doğrulama (daha hızlı, daha az güvenilir) |
| `--go` | Gerçek robotta Enter beklemeden 3 s sonra başla |
| `--time s` | Azami süre (varsayılan 1500 s) |
| `--no-noise` / `--no-gui` / `--fast` / `--out klasör` | Simülatörde gürültü yok / pencere yok / beklemeden koştur / kayıt klasörü |

`run.ps1` ile bu seçenekler komutun sonuna eklenir, örneğin `.\run.ps1 11 -Sim --no-verify`.

## 5. Sorun giderme

| Belirti | Çözüm |
|---|---|
| `Sensor model: cannot open ...` | Kalibrasyon yapılmamış; varsayılan model kullanılır. `.\run.ps1 11 -Calibrate`. |
| Kalibrasyon: `the box was never detected` | Kutu sensörün tam önünde değil ya da çok küçük; düz ve geniş bir yüzey kullanın. |
| `No sensor data on COMx` | Arduino kodu yüklü değil, port başka programda açık ya da kablo sorunu var. Arduino'yu başka bir USB girişine takınca COM numarası değişir (COM14 → COM15); `Get-PnpDevice -Class Ports -PresentOnly` ile bakın. Ayrıntı: [README.md](README.md) §7. |
| `sensor stream lost (Arduino)` | Arduino 1,5 s veri göndermedi. |
| Boş hücreler dolu çıkıyor | Sensör yere paralel ve tam ileri mi? Kalibrasyonu yenileyin; doğrulamayı kapatmayın. |
| Bazı hücreler hep 0,50 | Hiç görülemeyen yerler (duvar arkası); txt'ye 1 yazılır. |
