# 11_01 Frontier — en yakın frontier ile keşif

Ders notundaki yöntem (Yamauchi, 1997), example 11'in ilk sürümüyle **birebir aynı davranış**. Yeniden düzenlemeden sonra map3'te, aynı seed ile, olay sırası ve son olasılık ızgarası eski sürümle aynı çıktı.

Program: `Explore_Frontier.exe`. Kod: [src/FrontierStrategy.h](src/FrontierStrategy.h). Geri kalan her şey ortaktır ([../common](../common)).

## Karar kuralı

```text
frontier hücresi = kararsız (0.3 <= P <= 0.8) ve en az bir 4-komşusu BOŞ bilinen hücre
her plan adımında:
    her frontier f için: boş komşularından robota A* yolu en kısa olan n(f)
    hedef = maliyet(robot -> n(f)) en küçük olan f     (eşitlikte robota düz çizgide yakın olan)
    n(f)'ye git, f'ye dön, karşıdan ölç (hâlâ kararsızsa 360° tara)
ulaşılabilir frontier kalmayınca dur  (ardından ortak doğrulama)
```

- **Açgözlü (greedy):** Her ölçümden sonra karar yeniden verilir; hep "şu an en yakın" olana gidilir.
- **Pencerede:** Frontier hücrelerindeki 1, 2, 3 … sayıları A* maliyet sırasıdır (1 = seçilen).

## Artı / eksi

| + | − |
|---|---|
| Basit, hesabı ucuz; bilinmeyenin sınırına doğrudan gider. | Yerel düşünür: bir köşeyi sonraya bırakıp ona geri dönebilir. Toplam yol optimal değildir (bkz. [11_04_TSP](../11_04_TSP/README.md)). |
| Sık yeniden planlama: yeni bilgiye hemen uyar. | Eşit maliyetli hedefler arasında robot zikzak yapabilir. |

## Simülasyon (4 harita × 3 seed, gürültülü)

| | map1 | map2 | map3 | map4 | Ortalama |
|---|---:|---:|---:|---:|---:|
| Süre | 301 s | 335 s | 345 s | 337 s | **330 s** |
| Başarı oranı | 100 % | 100 % | 100 % | 100 % | **100 %** |
| Yol | | | | | 15,5 m |

TSP'den sonra ikinci hızlı yöntem; 4/4 harita başarılı. Ayrıntı: [../README.md](../README.md) §4.

## Çalıştırma

```powershell
.\run.ps1 11 -Sim -ObstacleMap map2          # ya da launcher'da "11_01 Frontier"
.\run.ps1 11 -ObstacleMap map3               # gerçek robot
```
