# Örnek 11: Exploration Mapping (Gaussian + Bayes doluluk ızgarası) — ortak teknik ayrıntı

Kobuki, laboratuvar zeminindeki 7 × 9 hücrelik alanı (40 cm hücreler, başlangıç (1,1)) **haritayı bilmeden** dolaşır. Güncel durum ve yöntem karşılaştırması: [README.md](README.md).
- **Sensör:** Robotun önünde **tek** bir HC-SR04 var. Robot yerinde 360° dönerek ya da bir hücreye karşıdan bakarak ölçer.
- **Güncelleme:** Her hücrenin P(Dolu) olasılığı Bayes kuralıyla güncellenir. Ölçüm olasılıkları Gauss gürültü modelinden (Method B) ve deneyden (Method A) gelir.
- **Hareket:** Sıradaki hareketi dört yöntemden biri seçer: [11_01 Frontier](11_01_Frontier/README.md), [11_02 BFS](11_02_BFS/README.md), [11_03 Random](11_03_Random/README.md), [11_04 TSP](11_04_TSP/README.md).
- **Sonuç:** Harita sonunda 10_AStarPathPlanning'in okuduğu 0/1 txt olarak kaydedilir.

Bu belge dört yöntemin **ortak** kısmını anlatır: sensör modeli, Bayes güncellemesi, kalibrasyon, keşif döngüsü ve metrikler (`common/`). Kullanım: [USER_GUIDE.md](USER_GUIDE.md).

## 1. Algoritma (ders notu)

| Adım | Uygulamada | Kod |
|---|---|---|
| 1. Initialize grid | Her hücre P(Dolu) = 0,5. | `BayesGrid` (ortak) |
| 2. Update the grid | Robotun hücresi boş işaretlenir. Her sonar ölçümü Bayes kuralıyla hücreleri günceller. | `Explorer::observe`, `BayesGrid::integrate` (ortak) |
| 3. Pick a next move | **Yönteme göre değişir:** en yakın frontier / BFS sırası / rastgele hücre / TSP turu. Ölçülecek hücreye dönülür ve **karşıdan ölçülür**; hâlâ kararsızsa 360° dönülür (Random: hedefte 360° tarama). | `Strategy::next` (her yöntemin klasöründe) |
| 4. Loop | Tüm hücreler P < 0,3 (boş) ya da P > 0,8 (dolu) olunca ya da yöntemin hedefi kalmayınca biter. Ardından doğrulama yapılır. Hiç görülemeyen hücreler kararsız kalır. | `Explorer` (ortak) |

**Bayes güncelleme kuralı:**

```text
P(O|sₙ) = P(sₙ|O)·P(O|sₙ₋₁) / [ P(sₙ|O)·P(O|sₙ₋₁) + P(sₙ|E)·P(E|sₙ₋₁) ]
```

## 2. Olasılıksal sensör modeli

Ders notundaki akış: **ölçüm z → sensör modeli P(z|O), P(z|E) → Bayes güncellemesi P(O|z).**

### Method B: Gauss gürültü modeli → P(z|O)

d metredeki bir yüzey:
- p_detect olasılıkla algılanır;
- algılanınca okuma `z ~ N(d + bias, σ(d))` olur;
- `σ(d) = σ₀ + k·d`, yani gürültü mesafeyle büyür.

Bir olayın olasılığı, okumanın bir aralığa düşmesidir ve Gauss eğrisinin o aralık üzerindeki integraliyle bulunur:

```text
P(z ∈ [w₀, w₁] | O) = p_detect · ( Φ((w₁ − d − bias)/σ(d)) − Φ((w₀ − d − bias)/σ(d)) )
```

Hücre için olay "okuma bu hücreye düştü"dür. Ön yüzü s metrede, derinliği L (40 cm) olan bir hücrenin yüzey okumaları s etrafında dağılır. Bu yüzden aralık `[s − 2σ, s + L − 2σ]` alınır.

### Method A: deney → P(z|E)

Hücre **boşken** okumaların ne kadarının yine de o hücreye düştüğü sayılır: yanlış okuma, çoklu yansıma. Sonuç P(olay|Boş) olur.

### Slayttaki 4 m örneği

Hücre 4,0 m'de, olay "okuma [3,9; 4,1] m'de". Varsayılan modelle (σ(4 m) = 10 cm, p_detect 0,95, P(z|E) = 0,20):

```text
P(z|O) = 0.95 · (Φ(1) − Φ(−1)) = 0.649      P(z|E) = 0.200
Bayes, P = 0.5'ten:   isabet    → P(O|z) = 0.649·0.5 / (0.649·0.5 + 0.2·0.5) = 0.76  (≈ 0.8, dolu tarafı)
                      isabet yok → P(O|z) = 0.351·0.5 / (0.351·0.5 + 0.8·0.5) = 0.30  (≈ 0.3, boş tarafı)
```

Yani tek bir ölçüm hücreyi eşiklerin sınırına getirir; karar için birkaç ölçüm gerekir. Program bu tabloyu kalibrasyonda ve her keşfin başında 0,5–4 m için yazar.

## 3. Kalibrasyon (`.\run.ps1 11 -Calibrate`)

**Önce mesafe ölçeği (launcher, 3. adım):**
- 40 × 40 cm blok, sensörün ön yüzünden tam 40 cm uzağa konur.
- Yankılı 50 ölçüm alınır; medyandan 3 cm'den uzak olanlar atılır.
- Kalanlardan ölçek bulunur: ölçek = 40 / ortalama ham cm. Ham cm = yankı süresi (µs) × 0,01715 (343 m/s varsayılır).
- Tek nokta tek parametre verir. HC-SR04'ün asıl hatası ses hızından (sıcaklıktan) gelen bir ölçek hatası olduğu için bu nokta ölçeğe kullanılır.
- Sonuç `sensor_calibration.txt` dosyasına yazılır ([common/RangeCalibration.h](common/RangeCalibration.h)).
- Gerçek robotta `SensorBoard` her yankılı okumayı bu ölçekle çarpar. Aşağıdaki Gauss modeli bu ölçeklenmiş okumalarla ölçülür.

Gauss modeli için yalnızca Arduino gerekir; robot sabit durur. Robot hücre ortasında ve ızgaraya dik bakar.

1. **Method B:** kutu sırayla 1, 2, 3, 5, 10 hücre öteye konur. 10 hücre, robot merkezinden yaklaşık 4 m demektir. Her konumda 100 ölçüm alınır ve şunlar hesaplanır:
   - ortalama (bias);
   - σ;
   - algılama oranı (kutunun ±25 cm'sindeki okumaların payı).

   Ardından `σ(d) = σ₀ + k·d` doğrusu en küçük kareler yöntemiyle uydurulur.
2. **Method A:** kutu kaldırılır ve 100 ölçüm alınır. Önündeki hücreye düşen okumaların payı P(z|E) olur.
3. Sonuç `sensor_model.txt` dosyasına yazılır (σ₀, k, bias, p_detect, P(z|E)) ve 0,5–4 m tablosu ile 4 m örneği basılır.

Simülatörde (`.\run.ps1 11 -Sim -Calibrate`) ölçülenler: σ, 0,5 m'de 1,3 cm'den 3,7 m'de 7,6 cm'ye çıkıyor; uydurulan doğru σ(d) = 1,6 cm + 1,6 cm/m.

## 4. Ölçümün hücrelere işlenmesi

Kod: [common/BayesGrid.h](common/BayesGrid.h), [common/Explorer.h](common/Explorer.h).

**a) Karşıdan ölçüm.** Robot hedefin komşusunda durup ona dönünce düzen kalibrasyonla aynıdır. Her okuma doğrudan olaydır:
- **isabet:** P(z|O) Gauss'tan hedefin mesafesinde, P(z|E) deneyden;
- **isabet yok:** ikisinin tümleyeni.

En az 3, en çok 12 okuma alınır; son 3 okuma kararla tutarlıysa erken durulur.

**b) 360° tarama ve sürüş sırasındaki okumalar** (ders notundaki bölgeler). HC-SR04 yaklaşık ±12°'lik koninin içindeki en yakın yüzeyi ölçer.
- **Bölge 1 (boş):** koninin iç kısmının (±6°) ölçülen mesafeden önce geçtiği hücreler → "geçti". Olasılıklar: P(geçti|O) = 1 − P(z|O), P(geçti|E) = 1 − P(z|E).
- **Bölge 2 (dolu):** ölçülen mesafedeki yay üzerindeki hücreler. İsabet bu hücreler arasında olasılık oranlarına göre paylaştırılır. Yay dış duvara ya da dolu bilinen bir hücreye değiyorsa ölçüm onunla açıklanır.
- **Bölge 3:** mesafenin arkası, değişmez.

Ek kurallar:
- Koni okumaları karşıdan ölçümden daha az güvenilirdir; en fazla 0,80 / 0,20 güvenle kullanılır.
- 1 m'den uzak isabetler kullanılmaz, ama boş kısım 4 m'ye kadar kullanılır.
- **Ses duvarın içinden geçmez:** ışın dolu bilinen hücrede durur. Bilinen bir duvarın içinden geçmesi gereken okuma isabet sayılmaz.
- "Yankı yok" zayıf kanıttır (0,8 m'ye kadar).
- **Sığ açı:** ışın bir hücreye yüzüne 50°'den sığ açıyla giriyorsa (`grazingDeg`) o hücre duvar olsa bile yankı vermeyebilir; ışın orada durur, o hücre ve arkası "geçti" sayılmaz.
- Aynı sensör okuması ancak robot 6° döndüyse, 4 cm ilerlediyse ya da 2 s geçtiyse yeniden işlenir.

## 5. Keşif döngüsü (ortak) ve Strategy arayüzü

Kod: [common/Explorer.h](common/Explorer.h), [common/Strategy.h](common/Strategy.h).

1. Başlangıçta yerinde 360° tarama.
2. **Plan:** Explorer, yönteme bir `PlanContext` verir:
   - ızgara;
   - yalnızca boş bilinen hücrelerin geçilebildiği maliyet haritası;
   - robotun ve başlangıcın hücresi;
   - frontier listesi;
   - vazgeçilen hücreler.

   Yöntem bir `Goal` döndürür:
   - `Measure`: hedef hücre, durulacak komşu ve oraya giden hücre yolu;
   - `LookAround`: gidilecek hücre, orada 360° tarama.
3. **Sürüş:** Yolun köşe hücreleri `PathFollower` ile önce yerinde dönülerek, sonra düz gidilerek sürülür. Yol üzerindeki bir hücre yeni ölçümle boş olmaktan çıkarsa yeniden planlanır.
4. **Ölçüm:** `Measure` hedefine dönülür ve karşıdan ölçülür. En az 3, en çok 12 okuma alınır (kalibrasyon düzeniyle aynı). Hâlâ kararsızsa 360° taranır. `LookAround` hedefinde 360° taranır.
5. **Vazgeçme:** 3 ziyarette karar verilemeyen hedeften vazgeçilir; yöntem bu hücreyi `givenUp` listesinde görür.
6. **Yöntem bitince doğrulama:** Boş alana komşu, henüz karşıdan ölçülmemiş dolu hücreler en yakından başlanarak karşıdan ölçülür. Yanlış "dolu" bir hücre arkasında keşfedilecek yer saklayabilir. Bu adım `--no-verify` ile kapatılır. Doğrulama yeni frontier açarsa yöntem yeniden devreye girer.
7. **Bitiş:**
   - Tüm hücreler karar verilince ya da ne yöntemin ne doğrulamanın hedefi kalınca durulur ve harita kaydedilir.
   - Bitiş nedeni: `all_decided`, `no_frontier`, `stagnated` (Random) ya da `time_limit` / `too_many_bumps`.
8. **Çarpma:** Bilinen engelse konum düzeltilir. Bilinmeyen bir şeyse o hücre doğrudan dolu (P ≥ 0,90) işaretlenir. Ardından robot 10 cm geri çekilir ve yeniden planlar. Uçurum sinyali ya da sensör akışı 1,5 s'den uzun kesilirse robot durur.

Ortak yardımcılar (Strategy.h):
- `routeAStar` (8 komşu, köşe kesmeden);
- `routeBfs` (4 komşu, en az hücre);
- `nearestTarget` / `rankTargets`: her hedef için A* yolu en kısa boş komşu.

## 6. Çıktı ve metrikler

Dosyalar `output/<yöntem>/` klasörüne yazılır:
- `explored_<yöntem>_<harita>_s<seed>_<zaman>.txt`: 10'un harita biçimi. 0 = boş, 1 = dolu ya da kararsız, `*` = başlangıç. Hedefe `/` yazıp `.\run.ps1 10 -Map ...` ile A* çalıştırılır.
- `..._prob.txt`: her hücrenin P(Dolu) değeri.
- `....png`: pencere resmi.
- `results/results.csv`: her koşuya bir satır ([common/Metrics.h](common/Metrics.h)). Sütunlar:
  - `time_s`, `driven_m`, `finish`, `completed`;
  - `success_ratio` = `correct_obs / observable`, `accuracy_all`;
  - `wrong`, `undecided`, `bumps`, `goals` ...

**Görülebilir hücre:** Engel haritasında boş olan, ya da boş bir 4-komşusu olan hücre. Dış köşeler ve kalın duvarın içi hiçbir konumdan görülemez; başarı oranına sayılmaz.

## 7. Simülasyon sonuçları (tek sensör)

**Dört yöntem, dört engel haritası:** [README.md](README.md) §4. Özetle, 48 gürültülü koşuda (seed 1–3):

| Yöntem | Ort. süre | Başarı oranı |
|---|---:|---:|
| TSP | 323 s | %100 |
| Frontier | 330 s | %100 |
| BFS | 358 s | %100 |
| Random | 639 s | %98,8 |

**11_01 Frontier, map3** (önceki sürümün sonuçlarıyla birebir aynı, seed 2026):
- Gürültülü: 376,7 s, 19,0 m, 57 doğru, **0 yanlış**, 6 kararsız (görülemeyen köşeler).
- Gürültüsüz: 358,5 s, 17,4 m, 0 çarpma.

**Önceki alan** (aşağıdaki tablo, 11_01 Frontier): gerçek harita [10_AStarPathPlanning/maps/lab.txt](../10_AStarPathPlanning/maps/lab.txt), başlangıç (1,7). Simülatördeki sensör:
- ±12° koni;
- 55°'den sığ açıyla gelen ışında yankı yok;
- mesafeyle büyüyen gürültü (4 m'de ~8 cm);
- %4 kaçan yankı, %2 hayalet okuma.

| Sensör modeli | Gürültü | Süre | Sonuç |
|---|---|---|---|
| varsayılan: σ(4 m) 10 cm, P(z\|E) 0,20 | var | 506 s, 27,0 m | 76 doğru, **0 yanlış**, 12 kararsız |
| varsayılan | yok | 488 s, 27,0 m | 75 doğru, **0 yanlış**, 13 kararsız |
| simülatör kalibrasyonu: σ(d) = 1,6 + 1,6·d cm, P(z\|E) 0,01 | var | 492 s, 25,7 m | 76 doğru, **0 yanlış**, 12 kararsız |
| kötü sensör: σ(d) = 2 + 3·d cm, p_detect 0,75, P(z\|E) 0,30 | var | 513 s, 25,8 m | 74 doğru, **0 yanlış**, 14 kararsız |

- Tüm denemelerde 44 boş hücrenin 44'ü bulundu ve hiç çarpma olmadı.
- Kararsız kalanlar gerçekten görülemeyen yerler: 11. satır, (2..4, 8) ve köşeler.
- Bulunan harita `lab.txt` ile aynı; A*'a verilince robot (1,7)'den (9,7)'ye gidiyor.

Gerçek robot denemeleri ve açık sorunlar: [README.md](README.md).

## 8. Dosyalar

| Dosya | İçerik |
|---|---|
| [maps/arena.txt](maps/arena.txt) | Alan: 7 × 9, 40 cm, `coords x-y 0`, başlangıç `*` (1,1), yön 90°, kenarlar duvar; içerik bilinmiyor (`?`) |
| [obstacleMapReferences/](obstacleMapReferences/) | map1..map4: görsel + 0/1 txt. Simülatörün dünyası, gerçek koşunun referansı (map3 = önceki `maps/truth.txt`) |
| `sensor_model.txt` | Kalibrasyon sonucu; `-Calibrate` oluşturur, dört yöntem de kullanır |
| `sensor_calibration.txt` | 40 cm mesafe ölçeği; launcher 3. adım oluşturur, dört yöntem de gerçek robotta kullanır (`--range-cal`) |
| [arduino/sensor/sensor.ino](arduino/sensor/sensor.ino) | Tek HC-SR04 (v2: varsayılan D11/D12, `PINS` komutu + EEPROM), ham ölçüm, 07/08 ile aynı JSON + `us` (yankı süresi) |
| [common/Strategy.h](common/Strategy.h) | Yöntem arayüzü (`next() -> Goal`), ortak yol / hedef yardımcıları |
| [common/Explorer.h](common/Explorer.h) | Ortak keşif döngüsü: tarama, sürüş, karşıdan ölçüm, doğrulama, çarpma |
| [common/BayesGrid.h](common/BayesGrid.h) | Gauss sensör modeli, Bayes kuralı, ışın işleme, frontier |
| [common/ExploreRobot.h](common/ExploreRobot.h) | Gerçek Kobuki + Arduino, simülatör (seed'li gürültü) |
| [common/ExploreView.h](common/ExploreView.h) | Pencere / PNG, yöntem katmanı, yanlış hücre işareti |
| [common/Calibration.h](common/Calibration.h) | Kalibrasyon (Method A + B) |
| [common/RangeCalibration.h](common/RangeCalibration.h) | 40 cm mesafe ölçeği: dosya okuma / yazma |
| [common/Metrics.h](common/Metrics.h) | Başarı oranı, results.csv |
| [common/ExploreApp.h](common/ExploreApp.h) | Program gövdesi: seçenekler, döngü, kayıt, özet |
| `11_0X_*/src/*Strategy.h` | Dört yöntem (her birinin README'si kendi klasöründe) |
| [Launcher/src/](Launcher/src/) | ExploreLauncher (Windows uygulaması), COM listesi, karşılaştırma / rapor |
| 01_Sample/.../AStarPlanner.h, PathFollower.h, NavCommon.h | A* ve yol takipçisi (10 ile ortak) |
