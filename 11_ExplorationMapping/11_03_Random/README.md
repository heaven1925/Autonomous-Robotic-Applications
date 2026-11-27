# 11_03 Random — rastgele keşif (temel karşılaştırma)

"Bilinmeyen nerede?" sorusunu hiç sormayan yöntem. Diğer yöntemlerin ne kadar kazandırdığını göstermek için temel çizgidir (baseline).

Program: `Explore_Random.exe`. Kod: [src/RandomStrategy.h](src/RandomStrategy.h).

## Karar kuralı

```text
her plan adımında:
    frontier kalmadıysa                         -> bitti (keşfedilecek yer yok)
    son 6 taramada hiç yeni hücre kararlaşmadıysa -> bitti ("stagnated")
    hedef = robotun ulaşabildiği BOŞ hücrelerden düzgün dağılımla rastgele biri
    A* ile oraya git, yerinde 360° tara
```

- **Ölçüm biçimi:** Hiçbir hücreyi hedef alıp karşıdan ölçmez; yalnızca 360° taramalar ve sürüş sırasındaki okumalarla öğrenir. Bu yüzden yalnızca karşıdan ölçümle kararlaşan hücreler kararsız kalabilir. Başarı oranı bu bedeli gösterir.
- **Tekrarlanabilirlik:** Rastgelelik kendi tohumuyla (`--seed`) üretilir; aynı seed aynı yürüyüşü verir.
- **Ortak doğrulama:** Durgunlukla bitse de çalışır ve dolu hücreleri karşıdan ölçer. Durgunluk kararı bir kez verilince yöntem yeniden devreye girmez.
- **Pencerede:** Numara, kaçıncı rastgele hedef olduğunu gösterir. Panelde aday sayısı ve yeni hücre vermeyen tarama sayacı (x / 6) yazar.

## Artı / eksi

| + | − |
|---|---|
| Hiç plan yok, harita bilgisi gerekmez. | Çok yavaş: aynı yerlere tekrar tekrar gider. |
| Her yerden 360° bakış, birçok açıdan ölçüm verir. | Bitişi belirsiz: durgunluk eşiğine bağlı. Bazı hücreler kararsız kalabilir. |

## Simülasyon (4 harita × 3 seed, gürültülü)

| | map1 | map2 | map3 | map4 | Ortalama |
|---|---:|---:|---:|---:|---:|
| Süre | 568 s | 674 s | 668 s | 645 s | **639 s** |
| Başarı oranı | 100 % | 97,1 % | 99,4 % | 98,7 % | **98,8 %** |
| Yol | | | | | 26,4 m |

- Diğerlerinin yaklaşık **2 katı süre**, 1,7 katı yol.
- 12 koşunun 3'ü durgunlukla bitti.
- Yanlış karar hiç yok, ama bazı görülebilir hücreler kararsız kaldı.
- %95 eşiğiyle 4/4 harita başarılı.

**Ayar:** Durgunluk eşiği `RandomStrategy(seed, maxIdle = 6)` ile değiştirilir ([src/main.cpp](src/main.cpp)).

## Çalıştırma

```powershell
.\run.ps1 random -Sim -ObstacleMap map1 --seed 4
.\run.ps1 random -ObstacleMap map3                  # gerçek robot
```
