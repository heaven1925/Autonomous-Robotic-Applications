# A* Yol Planlama — Kullanıcı Kılavuzu (Kobuki)

Haritayı bir `.txt` dosyasına 0/1 ile yazarsınız. Program, robot yarıçapı kadar pay bırakarak A* ile en kısa yolu bulur, gösterir, kaydeder ve isterseniz Kobuki'yi o yolda sürer. Teknik ayrıntılar: [AStar.md](AStar.md).

## 1. Hızlı başlangıç

Depo kökünde PowerShell açın:

```powershell
.\run.ps1 10 -PlanOnly          # Yalnızca planla ve göster (robot yok)
.\run.ps1 10 -Sim               # Planla ve simülatörde sür
.\run.ps1 10                    # Planla ve gerçek Kobuki'yi sür (port otomatik bulunur)
.\run.ps1 10 -Port COM6         # Kobuki portunu elle verin (Arduino gerekmez)

# Başka harita / başlangıç / hedef:
.\run.ps1 10 -Sim -Map 10_AStarPathPlanning\maps\example_20x10.txt
.\run.ps1 10 -Sim -Start 0,0 -Goal 18,5 -Map 10_AStarPathPlanning\maps\example_20x10.txt

# Diğer seçenekler olduğu gibi programa aktarılır:
.\run.ps1 10 -Sim --smooth --block 10,0
```

Harita verilmezse [maps/assignment.txt](maps/assignment.txt) (ödev görseli) kullanılır.

## 2. Kendi haritanızı yazmak

```text
# 2 m x 2 m oda, ortada 40 cm'lik kutu
cell 0.20
goal 9 2
heading 0
* 0 0 0 0 0 0 0 0 0
0 0 0 1 1 0 0 0 0 0
0 0 0 1 1 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0
...
```

- `0` = boş, `1` = engel, `*` = başlangıç (robotun durduğu hücre). Her hücre `cell` metredir (varsayılan 20 cm). 10 hücre × 20 cm = 2 m.
- Koordinatlar **(sütun, satır)** şeklindedir: sütun soldan, satır **üstten** (dosyanın ilk ızgara satırı = 0) sayılır.
- `heading`: robotun başlangıçtaki yönü. 0 = sağa (sütun artan), 90 = yukarı (satır 0'a doğru), 180 = sola, 270 = aşağı.
- Hedefi `goal` satırı yerine ızgaraya `G` yazarak da verebilirsiniz. Başlangıç için `*` yerine `start 0 0` satırı da kabul edilir.
- 40 cm'lik bir engel 2×2 hücre `1` demektir.

## 3. Ekran

**Pencere:**
- Koyu kırmızı = engel.
- Açık kırmızı = robotun **merkezinin** giremeyeceği hücreler (engele 15 cm'den yakın).
- Açık mavi = A*'ın incelediği hücreler.
- Mavi çizgi = A* yolu.
- Yeşil çerçeve = robot gövdesinin kaplayacağı alan.
- Sürüş sırasında robot ve geçtiği iz de çizilir.
- Sağ panelde sayılar ve yol noktaları listelenir.

**Konsol:** haritanın metin hali yazılır.

```text
# engel   + robot merkezi giremez   . boş   * başlangıç   o A* yolu   G hedef   X tamponla bulunan engel   R robot (sürüş sonunda)
```

| Tuş (pencere ya da konsol) | Etki |
|---|---|
| `Enter` | Gerçek robotta sürüşü başlat |
| `Boşluk` | Duraklat / devam |
| `Q` / Esc / Ctrl+C / pencereyi kapat | Dur |

Sonuçlar [`10_AStarPathPlanning/output/`](output/) klasörüne kaydedilir:
- `<harita>_<zaman>_plan.png` / `_plan.txt`: plan;
- `<harita>_<zaman>_run.png` / `_run.txt`: sürüşten sonraki durum.

Metin dosyasında harita, yol, hücre listesi ve metre cinsinden yol noktaları bulunur.

## 4. Gerçek robotla çalıştırma

1. Haritayı zemine ölçerek uygulayın: her hücre `cell` (20 cm). Haritanın sol-üst köşesi (0,0) hücresinin köşesidir.
2. `.\run.ps1 10` (ya da `-Map ...`) çalıştırın. Program planı gösterir ve Kobuki'ye bağlanır.
3. Robotun **merkezini** başlangıç hücresinin merkezine koyun ve `heading` yönüne çevirin. Varsayılan yön sağdır, yani artan sütun yönü.
4. `Enter`'a basın. Robot her köşede yerinde döner ve bir sonraki köşeye düz gider.

**Dikkat:**
- Konum yalnızca odometriden (jiroskop + tekerlekler) gelir. Uzun yollarda birkaç cm kayma olabilir. Gerekirse `--margin 0.05` ile ek pay verin.
- Kobuki'nin gerçek yarıçapı yaklaşık 17,5 cm'dir. 20 cm hücrede 15 cm ile aynı sonucu verir; daha küçük hücre kullanıyorsanız `--radius 0.18` verin.
- Harita kenarı duvar sayılmaz (robot kenar hücrelerden geçebilir). Alan duvarla çevriliyse `--walled` kullanın.
- Haritada olmayan bir şeye çarparsa robot durur, 10 cm geri çekilir, o hücreyi engel olarak ekler ve yeniden planlar.

## 5. Program seçenekleri

```text
AStar.exe [harita.txt] [seçenekler]                yalnızca plan
AStar.exe [harita.txt] --sim [seçenekler]          plan + simülatör
AStar.exe [harita.txt] <Kobuki COM> [seçenekler]   plan + gerçek robot
```

| Seçenek | Etki |
|---|---|
| `--start c,r` / `--goal c,r` | Başlangıç / hedef hücresi (dosyadakini ezer) |
| `--heading d` | Başlangıç yönü, derece |
| `--cell m` | Hücre boyu (m) |
| `--radius m` | Robot yarıçapı, varsayılan 0.15 |
| `--margin m` | Yarıçapa ek güvenlik payı |
| `--4` | 4 komşulu arama (çapraz hareket yok) |
| `--smooth` | Yolu görüş hattı ile kısalt (daha az dönüş) |
| `--walled` | Harita kenarı da duvar |
| `--strict` | Hedef engele çok yakınsa kaydırma, dur |
| `--block c,r` | Yalnızca `--sim`: haritada **olmayan** bir engel ekle (yeniden planlamayı dener) |
| `--no-gui` / `--fast` / `--out klasör` | Pencere yok / simülasyonu beklemeden koştur / kayıt klasörü |

## 6. Sorun giderme

| Mesaj | Anlamı / çözüm |
|---|---|
| `MAP ERROR: ... line N: ...` | Dosyanın N. satırı hatalı: 0/1 dışında karakter, satır uzunlukları farklı ya da bilinmeyen anahtar kelime. |
| `GOAL: the centre of (c,r) is only X cm from an obstacle` | Hedef engele yarıçaptan yakın. Program en yakın güvenli hücreye kaydırır (`--strict` ile kaydırmaz). |
| `A*: NO PATH` | Hedefe giden her yol kapalı ya da robottan dar. Tek hücrelik (20 cm) koridorlardan 30 cm çaplı robot geçemez. |
| `the start cell ... is closer than 15 cm` | Robot başlangıçta engele değiyor olabilir; A* onu oradan çıkarır. |
| `Bumper hit ... added to the map` | Haritada olmayan engel; robot yeniden planladı. |
| `too many bumps` / `no path left` | Haritanın gerçek alanla uyuşmadığı çok yer var. Haritayı düzeltin. |
