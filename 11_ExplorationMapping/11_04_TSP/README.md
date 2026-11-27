# 11_04 TSP — gezgin satıcı turu ile keşif

Frontier yöntemi her seferinde **en yakın** hedefi seçer (açgözlü). Burada her plan adımında ulaşılabilir **bütün** frontier hücreleri birlikte düşünülür. Robotun bulunduğu yerden başlayıp hepsini birer kez ziyaret eden en kısa tur (açık gezgin satıcı yolu, TSP) hesaplanır. Robot turun **ilk** durağına gider, ölçer, sonra tur yeni haritayla yeniden hesaplanır (receding-horizon TSP; Zlot 2002, Kulich 2011).

Program: `Explore_TSP.exe`. Kod: [src/TspStrategy.h](src/TspStrategy.h), çözücü [src/TspSolver.h](src/TspSolver.h).

## Karar kuralı

```text
düğümler : her ulaşılabilir frontier hücresi f, durulacak yer n(f) = A* yolu en kısa boş komşusu (11_01 ile aynı)
maliyet  : robot -> n(f) ve n(f) -> n(g) arası A* yol uzunluğu (aynı yerden ölçülen hedefler arası 0)
tur      : robottan başlayıp tüm düğümlerden geçen en kısa AÇIK yol (dönüş yok)
            n <= 13 : Held-Karp dinamik programlama, kesin en iyi   O(2^n · n²)
            n >  13 : en yakın komşu + 2-opt iyileştirme
hareket  : turun ilk durağına git, f'ye dön, karşıdan ölç; sonra yeniden planla
```

Held-Karp:

```text
dp[S][j] = robottan çıkıp S kümesindeki düğümlerin hepsinden geçerek j'de biten en kısa yol
dp[{j}][j] = d0[j]
dp[S][j]   = min_{i ∈ S−{j}}  dp[S−{j}][i] + d[i][j]
tur        = min_j dp[tümü][j]   (geri izleme ile sıra)
```

**Pencerede:**
- Mor çizgi turun tamamıdır; 1, 2, 3 … tur sırasıdır.
- Panelde şunlar yazar: tur uzunluğu, çözücü (Held-Karp kesin / 2-opt), ilk durağın en yakın frontier olup olmadığı ("first stop is NOT the nearest frontier" = TSP'nin açgözlüden farklı karar verdiği an).

## Artı / eksi

| + | − |
|---|---|
| Küresel sıra: köşeleri geride bırakıp geri dönmeyi azaltır; toplam yol daha kısa. | Hesap daha pahalı: n düğüm için n² A* + Held-Karp (bu 7 × 9 alanda simülasyon hızını fark edilir biçimde etkilemiyor). 13 düğümden fazlasında sezgisel çözücüye düşer, en iyi tur garanti değildir. |
| Aynı yerden görülen hedefleri art arda toplar. | Tur, henüz görülmemiş alanı bilmez; harita değiştikçe plan da değişir (bu yüzden her adımda yeniden hesaplanır). |

## Simülasyon (4 harita × 3 seed, gürültülü)

| | map1 | map2 | map3 | map4 | Ortalama |
|---|---:|---:|---:|---:|---:|
| Süre | 299 s | 340 s | 332 s | 321 s | **323 s** |
| Başarı oranı | 100 % | 100 % | 100 % | 100 % | **100 %** |
| Yol | | | | | 14,8 m |

- **En hızlı yöntem** ve en kısa yol; 4/4 harita başarılı.
- Frontier'dan farkı küçük (%2). Alan küçük olduğu için en yakın hedef çoğu zaman turun da ilk durağıdır. Fark, frontier sayısı arttıkça ve alan büyüdükçe artar.

## Çalıştırma

```powershell
.\run.ps1 tsp -Sim -ObstacleMap map4
.\run.ps1 tsp -ObstacleMap map3                     # gerçek robot
```
