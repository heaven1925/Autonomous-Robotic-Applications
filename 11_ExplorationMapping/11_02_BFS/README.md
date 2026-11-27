# 11_02 BFS — genişlik öncelikli keşif

Izgara bir graf olarak düşünülür: hücreler düğüm, 4-komşuluk kenardır. Frontier yönteminde "robota en yakın" hedef seçilir. Burada ise hücreler **başlangıç hücresinden** genişlik öncelikli arama (BFS) sırasıyla, halka halka (katman katman) keşfedilir.

Program: `Explore_BFS.exe`. Kod: [src/BfsStrategy.h](src/BfsStrategy.h).

## Karar kuralı

```text
kuyruk <- başlangıç hücresi                    (FIFO)
kuyruk boşalana kadar:
    c <- kuyruğun başı
    c'nin görülmemiş her 4-komşusu n için:
        n BOŞ       -> kuyruğa ekle (bir katman derinde, sonra genişletilir)
        n KARARSIZ  -> HEDEF: ilk rastlanan ölçülür; robot c'ye (n'nin BFS ebeveyni) gider, n'ye dönüp karşıdan ölçer
        n DOLU      -> duvar, genişletilmez
```

- **Yol:** Robot c'ye **BFS en kısa yoluyla** gider (4 komşu, en az hücre; A* değil).
- **Yeniden arama:** Her ölçümden sonra BFS güncel ızgarada baştan yapılır. Böylece boşa dönen hücre genişletilir ve sıra başlangıçtan genişlik öncelikli kalır. Gürültü bir hücrenin durumunu değiştirse bile kuyruk bozulmaz.
- **Bitiş:** Kuyrukta ulaşılabilir kararsız hücre kalmayınca durulur (ardından ortak doğrulama).
- **Pencerede:**
  - 1, 2, 3 … kararsız hücrelerin BFS (FIFO) sırasıdır;
  - mor daire BFS köküdür (başlangıç);
  - panelde genişletilen hücre sayısı ve sıradaki hedefin derinliği yazar.

## Artı / eksi

| + | − |
|---|---|
| Sistematik ve eksiksiz: başlangıca yakın her yer önce bitirilir; sıra öngörülebilir. | Sıra robotun nerede olduğunu dikkate almaz: uzak bir hedeften sonra sıradaki yine başlangıç yakınında olabilir. Daha çok ileri geri sürüş. |
| BFS yolu en az hücreli yoldur. | 4-komşu yol çaprazda merdiven çizer; daha çok yerinde dönüş. |

## Simülasyon (4 harita × 3 seed, gürültülü)

| | map1 | map2 | map3 | map4 | Ortalama |
|---|---:|---:|---:|---:|---:|
| Süre | 358 s | 397 s | 352 s | 325 s | **358 s** |
| Başarı oranı | 100 % | 100 % | 100 % | 100 % | **100 %** |
| Yol | | | | | 16,7 m |

- Frontier'dan ortalama ~%9 daha yavaş; 4/4 harita başarılı.
- En iyi sonucunu en dolu harita olan map4'te verdi (325 s); bu haritada Frontier'dan da hızlı.

## Çalıştırma

```powershell
.\run.ps1 11 -Algo bfs -Sim -ObstacleMap map2      # ya da: .\run.ps1 bfs -Sim ...
.\run.ps1 bfs -ObstacleMap map3                    # gerçek robot
```
