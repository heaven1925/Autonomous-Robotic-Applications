# Örnek 10: A* ile yol planlama (Kobuki, laboratuvar labirenti)

Robot, laboratuvar zeminindeki 8 × 11 hücrelik labirentte başlangıçtan hedefe **A\*** ile en kısa yolu bulur. Robot yarıçapı (15 cm) kadar pay bırakıldığı için engellere değmez. Ardından Kobuki bu yolu sürer.

- Labirent [maps/lab.txt](maps/lab.txt) dosyasında 0/1 olarak tanımlıdır.
- Sensör kartı (HC-SR04 / Arduino) kullanılmaz. Robot yalnızca haritaya, jiroskopa ve tekerlek enkoderlerine göre gider.
- Donanım olmadan da çalışır: yerleşik simülatör aynı haritayı kullanır.

Kullanım: [USER_GUIDE.md](USER_GUIDE.md).

## 1. Harita dosyası

```text
name lab
cell 0.43            # hücre kenarı, metre (zeminde ölçülmeli)
coords row-col 1     # hücreler (satır, sütun) yazılır, 1'den sayılır
border wall          # alanın dört yanı kapalı
heading 270          # robotun ilk yönü: 270 = aşağı (11. satıra doğru)
1 0 0 0 0 0 * 1      # satır 1
1 0 0 0 0 0 1 1
1 0 0 0 1 1 1 1
1 0 0 0 0 0 1 1
1 1 1 0 0 0 0 1
1 0 0 0 0 0 0 1
1 0 0 0 1 1 0 1
1 0 0 0 1 0 0 1
1 0 0 0 0 0 / 1      # satır 9
1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1      # satır 11
```

| İşaret | Anlamı |
|---|---|
| `0` | boş |
| `1` | engel |
| `*` | robotun başlangıç hücresi: **(1,7)** |
| `/` | hedef hücresi: **(9,7)** |

- **Koordinatlar zemindeki numaralar gibidir:** (satır, sütun), 1'den başlar. Satır 1 kapıya yakın üst kenar, sütun 8 sağdaki uzun beyaz panel. Ekrandaki mesajlar, resimdeki numaralar ve `-Goal`, `-Start`, `--block` de bu yazımı kullanır.
- **Engel ya da hedef değişirse** yalnızca ızgaradaki 0 / 1 / `*` / `/` değerlerini değiştirin. Hücreler boşlukla, virgülle ya da bitişik yazılabilir; `{0,1,0}` yazımı da olur.
- `-Goal satır,sütun` (komut satırı) dosyadaki `/`'ın, `-Start` de `*`'ın yerine geçer.
- Hatalı dosyada program satır numarasıyla birlikte hatayı söyler. Örnekler: `line 22: this row has 7 cells, the first row has 8`, `more than one goal (/) in the grid`.
- 09 Mapping'in kaydettiği `_grid.txt` da aynı biçimdedir ve buraya verilebilir.

## 2. Robot yarıçapı kadar pay: konfigürasyon uzayı

Robot bir nokta değil, 15 cm yarıçaplı bir dairedir. Bu yüzden engeller robot yarıçapı kadar **şişirilir** ve robot bir nokta gibi planlanır:

> Bir boş hücrenin **merkezi**, herhangi bir engel hücresinin (kare) en yakın noktasına yarıçaptan yakınsa o hücre yasak sayılır. Robotun merkezi oraya giremez.

Labirentte hücre 43 cm, yani bir hücre merkezinden komşu engelin kenarına uzaklık 21,5 cm, çapraz köşeye 30 cm. Bu 15 cm'den büyük olduğu için hiçbir boş hücre yasaklanmıyor; Kobuki'nin gerçek yarıçapı olan yaklaşık 17,5 cm için de durum aynı. Gövde (35 cm) hücreye (43 cm) sığar ve her yanda yaklaşık 4 cm pay kalır.

Planlayıcı yine de her yolu kontrol eder: rota 1 cm aralıkla örneklenir ve robot merkezinin engellere en yakın mesafesi (`min clearance`) yazılır. Labirentte bu değer 21,5 cm'dir.

Ayarlar:
- `border wall`: harita kenarı da duvar sayılır.
- `--radius 0.18`: Kobuki'nin gerçek yarıçapıyla planlar.
- `--margin 0.05`: odometri hatasına karşı ek pay ekler.
- Hücre küçük olsaydı (örneğin 20 cm), engelin çevresindeki hücreler yasaklanırdı. Konsolda bunlar `+`, resimde açık kırmızı görünür.

## 3. A*

Kod: [AStarPlanner.h](../01_Sample/KobukiNativeVS/include/AStarPlanner.h) (`astar::search`).

- **Düğüm:** hücre. **Komşular:** 8 yön (`--4` ile yalnızca 4 düz yön).
- **g:** başlangıçtan gelinen yol uzunluğu. Düz adım 1 hücre, çapraz adım √2 hücre.
- **h:** hedefe tahmini mesafe. 8-komşulukta *octile* `max(dx,dy) + (√2−1)·min(dx,dy)`, 4-komşulukta Manhattan. İkisi de kabul edilebilir ve tutarlı olduğu için A* en kısa yolu bulur ve her hücreyi bir kez kapatır.
- **f = g + h** en küçük olan açık hücre genişletilir. Eşit f'de hedefe yakın olan (g'si büyük olan) önce gelir.
- **Köşe kesme yok:** çapraz adımda yanındaki iki hücre de boş olmalı. Örneğin (1,7)'den (2,6)'ya çapraz gidilemez, çünkü (2,7) engeldir.
- Açık liste `priority_queue` ile tutulur. Daha kısa yol bulununca yeni kayıt eklenir, eskisi çekildiğinde atlanır.
- Karşılaştırma için aynı arama h = 0 ile (Dijkstra) de yapılır ve iki algoritmanın açtığı hücre sayıları yazılır.

Yol, yön değiştirdiği **köşe hücrelerine** indirgenir; robot iki köşe arasını düz gider. `--smooth` verilirse yol "görüş hattı" ile kısaltılır ve bacak sayısı azalır.

## 4. Sürüş

Kod: [01_Sample/.../PathFollower.h](../01_Sample/KobukiNativeVS/include/PathFollower.h). Konum Kobuki'nin jiroskopundan (yön) ve tekerlek enkoderlerinden (mesafe) gelir; bunlar haritadaki başlangıç pozuna göre harita çerçevesine çevrilir.

1. **Dön:** robot sıradaki köşeye dönük olana kadar yerinde döner (tolerans 3°).
2. **Git:** o köşeye düz gider. Çizgiden saparsa yönünü düzeltip çizgiye geri döner. Hız 0,15 m/s'dir, köşeye yaklaşınca düşer; 1 cm kala bir sonraki bacağa geçer.

**Tampon:** haritada olmayan bir şeye çarparsa robot şunları yapar:
1. Durur.
2. Çarptığı hücreyi haritaya engel olarak ekler.
3. 10 cm geri çekilir.
4. Bulunduğu hücreden **A\*'ı yeniden çalıştırır** (en fazla 10 kez).

**Güvenlik:** uçurum sensörü, `Q`, Ctrl+C ya da pencereyi kapatmak robotu durdurur. Ayrıca planlanan sürenin yaklaşık 20 katında süre sınırı vardır.

## 5. Sonuçlar (simülatör)

| Senaryo | A* yolu | Açılan hücre (A* / Dijkstra) | Sürüş | Hedef hatası |
|---|---|---|---|---|
| (1,7) → (9,7), 8 komşu | 12 hücre, 5,26 m, 7 bacak | 17 / 42 | 65,5 s, çarpma yok | 1,0 cm |
| aynı, `--4` | 15 hücre, 6,02 m, 8 bacak | – | – | – |
| haritada olmayan engel `--block 6,6` | çarptı, (5,5)'ten yeniden planladı: 7 hücre, 2,58 m | – | 67,0 s, 1 çarpma, 1 yeniden plan | 1,0 cm |

```text
      1 2 3 4 5 6 7 8
   1  # . . . . o * #        *  başlangıç (1,7)
   2  # . . o o . # #        /  hedef (9,7)
   3  # . . o # # # #        o  A* yolu
   4  # . . o . . # #        #  engel
   5  # # # . o . . #
   6  # . . . . o o #
   7  # . . . # # o #
   8  # . . . # . o #
   9  # . . . . . / #
  10  # # # # # # # #
  11  # # # # # # # #
```

- Robot merkezi engellere en az 21,5 cm uzak kalıyor; gereken 15 cm.
- Planlama süresi 0,1 ms'nin altında.
- Labirentteki tüm boş hücrelere yol bulunuyor; engel üstündeki ya da harita dışındaki hedef reddediliyor.
- Gerçek robotta henüz denenmedi.

Resimde:
- koyu kırmızı = engel;
- açık mavi = A*'ın açtığı hücreler;
- mavi çizgi = A* yolu;
- yeşil çerçeve = robot gövdesinin kapladığı alan;
- koyu yeşil = robotun gittiği yol.

Resim her çalıştırmada `output/` klasörüne kaydedilir.

## 6. Dosyalar

| Dosya | İçerik |
|---|---|
| [maps/lab.txt](maps/lab.txt) | Laboratuvar labirenti |
| [01_Sample/.../AStarPlanner.h](../01_Sample/KobukiNativeVS/include/AStarPlanner.h) | Harita okuma, şişirme, A*, yol kısaltma, açıklık kontrolü (donanımdan bağımsız) |
| [01_Sample/.../GdiCanvas.h](../01_Sample/KobukiNativeVS/include/GdiCanvas.h) | Çizim tuvali, pencere, PNG (09 ile ortak) |
| [src/main.cpp](src/main.cpp) | Argümanlar, doğrulama, plan, sürüş döngüsü, kayıt |
| [src/Robot.h](src/Robot.h) | Gerçek Kobuki ve simülatör |
| [01_Sample/.../PathFollower.h](../01_Sample/KobukiNativeVS/include/PathFollower.h) | Dön-git yol takipçisi |
| [src/PlanView.h](src/PlanView.h) | Harita resmi |
