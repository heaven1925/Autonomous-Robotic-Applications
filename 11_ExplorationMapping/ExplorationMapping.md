# Örnek 11: Exploration Mapping (Gaussian + Bayes doluluk ızgarası, frontier keşfi)

Kobuki, laboratuvar zeminindeki 7 × 9 hücrelik alanı (40 cm hücreler, başlangıç (1,1)) **haritayı bilmeden** dolaşır. Güncel durum: [README.md](README.md).
- **Sensör:** robotun önünde **tek** bir HC-SR04 var; robot her durakta yerinde 360° dönerek ölçer.
- **Güncelleme:** her hücrenin P(Dolu) olasılığı Bayes kuralıyla güncellenir. Ölçüm olasılıkları Gauss gürültü modelinden (Method B) ve deneyden (Method A) gelir.
- **Sonuç:** frontier'lara A* ile gidilir; sonunda harita 10_AStarPathPlanning'in okuduğu 0/1 txt olarak kaydedilir.

Kullanım: [USER_GUIDE.md](USER_GUIDE.md).

## 1. Algoritma (ders notu)

| Adım | Uygulamada |
|---|---|
| 1. Initialize grid | Her hücre P(Dolu) = 0,5. |
| 2. Update the grid | Robotun hücresi boş işaretlenir. Her sonar ölçümü Bayes kuralıyla hücreleri günceller. |
| 3. Pick a next move | Frontier = karar verilmemiş ve boş komşusu olan hücre. Boş komşusuna giden A* yolu en kısa olan frontier seçilir. Robot oraya gider, **frontier'a döner ve tekrar ölçer**; hâlâ kararsızsa 360° döner. |
| 4. Loop | Tüm hücreler P < 0,3 (boş) ya da P > 0,8 (dolu) olunca biter. Hiç görülemeyen hücreler kalırsa ulaşılabilir hedef bitince durur. |

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

Yalnızca Arduino gerekir; robot sabit durur. Robot hücre ortasında ve ızgaraya dik bakar.

1. **Method B:** kutu sırayla 1, 2, 3, 5, 10 hücre öteye konur. 10 hücre, robot merkezinden yaklaşık 4 m demektir. Her konumda 100 ölçüm alınır ve şunlar hesaplanır:
   - ortalama (bias);
   - σ;
   - algılama oranı (kutunun ±25 cm'sindeki okumaların payı).

   Ardından `σ(d) = σ₀ + k·d` doğrusu en küçük kareler yöntemiyle uydurulur.
2. **Method A:** kutu kaldırılır ve 100 ölçüm alınır. Önündeki hücreye düşen okumaların payı P(z|E) olur.
3. Sonuç `sensor_model.txt` dosyasına yazılır (σ₀, k, bias, p_detect, P(z|E)) ve 0,5–4 m tablosu ile 4 m örneği basılır.

Simülatörde (`.\run.ps1 11 -Sim -Calibrate`) ölçülenler: σ, 0,5 m'de 1,3 cm'den 3,7 m'de 7,6 cm'ye çıkıyor; uydurulan doğru σ(d) = 1,6 cm + 1,6 cm/m.

## 4. Ölçümün hücrelere işlenmesi

Kod: [src/BayesGrid.h](src/BayesGrid.h), [src/Explorer.h](src/Explorer.h).

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

## 5. Keşif adımları

1. Başlangıçta yerinde 360° tarama.
2. En yakın frontier'ın boş komşusuna A* ile git. Yalnızca boş bilinen hücrelerden geçilir. Frontier'a dön, karşıdan ölç; hâlâ kararsızsa 360° tara.
3. Ulaşılabilir frontier kalmayınca **doğrulama:** boş alana komşu, henüz karşıdan ölçülmemiş dolu hücreleri karşıdan ölç. Yanlış "dolu" bir hücre arkasında keşfedilecek yer saklayabilir. `--no-verify` ile kapatılır.
4. Tüm hücreler karar verilince ya da ulaşılabilir hedef kalmayınca dur ve haritayı kaydet.
5. **Çarpma:** bilinen engelse konum düzeltilir; bilinmeyen bir şeyse o hücre doğrudan dolu (P ≥ 0,90) işaretlenir. Ardından robot 10 cm geri çekilir ve yeniden planlar. Uçurum sinyali ya da sensör akışı 1,5 s'den uzun kesilirse durur.

## 6. Çıktı

- `output/explored_<zaman>.txt`: 10'un harita biçimi. 0 = boş, 1 = dolu ya da kararsız, `*` = başlangıç. Hedefe `/` yazıp `.\run.ps1 10 -Map ...` ile A* çalıştırılır.
- `output/explored_<zaman>_prob.txt`: her hücrenin P(Dolu) değeri.
- `output/explored_<zaman>.png`: pencere resmi.

## 7. Simülasyon sonuçları (tek sensör)

**Güncel alan** ([maps/truth.txt](maps/truth.txt), 7 × 9, başlangıç (1,1)): gürültülü ve gürültüsüz simülasyonda 57 doğru, **0 yanlış**, 6 kararsız (görülemeyen köşeler), ~6 dakika, 0 çarpma.

**Önceki alan** (aşağıdaki tablo): gerçek harita [10_AStarPathPlanning/maps/lab.txt](../10_AStarPathPlanning/maps/lab.txt), başlangıç (1,7). Simülatördeki sensör:
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
| [maps/truth.txt](maps/truth.txt) | Yalnızca simülatör: laboratuvarda kurulan alanın gerçek haritası |
| `sensor_model.txt` | Kalibrasyon sonucu; `-Calibrate` oluşturur |
| [arduino/sensor/sensor.ino](arduino/sensor/sensor.ino) | Tek HC-SR04 (D11/D12), ham ölçüm, 07/08 ile aynı JSON |
| [src/BayesGrid.h](src/BayesGrid.h) | Gauss sensör modeli, Bayes kuralı, ışın işleme, frontier |
| [src/Explorer.h](src/Explorer.h) | 360° tarama, frontier seçimi, A*, karşıdan ölçüm, doğrulama, çarpma |
| [src/ExploreRobot.h](src/ExploreRobot.h) | Gerçek Kobuki + Arduino, simülatör |
| [src/ExploreView.h](src/ExploreView.h) | Pencere / PNG |
| [src/main.cpp](src/main.cpp) | Kalibrasyon, keşif döngüsü, kayıt |
| 01_Sample/.../AStarPlanner.h, PathFollower.h, NavCommon.h | A* ve yol takipçisi (10 ile ortak) |
