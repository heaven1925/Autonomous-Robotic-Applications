# A* Yol Planlama — Kullanıcı Kılavuzu (Kobuki, laboratuvar labirenti)

Labirent [maps/lab.txt](maps/lab.txt) dosyasında 0/1 ile tanımlıdır. Program, robot yarıçapı kadar pay bırakarak A* ile en kısa yolu bulur, gösterir, kaydeder ve Kobuki'yi o yolda sürer. Arduino / HC-SR04 gerekmez. Teknik ayrıntılar: [AStar.md](AStar.md). Kısa özet ve build/run adımları: [OZET.md](OZET.md).

## 1. Hızlı başlangıç

Depo kökünde PowerShell açın:

```powershell
.\run.ps1 10 -PlanOnly          # Yalnızca planla ve göster (robot yok)
.\run.ps1 10 -Sim               # Planla ve simülatörde sür
.\run.ps1 10                    # Planla ve gerçek Kobuki'yi sür (port otomatik bulunur)
.\run.ps1 10 -Port COM6         # Kobuki portunu elle verin
.\run.ps1 10 -Goal 6,2          # Dosyadaki / yerine başka hedef: 6. satır, 2. sütun
.\run.ps1 10 -Sim --block 6,6   # Simülatörde haritada olmayan bir engel (yeniden planlamayı dener)
```

## 2. Labirent dosyası

```text
cell 0.43            # hücre kenarı (m)  <- ZEMİNDE ÖLÇÜP DÜZELTİN
coords row-col 1     # (satır, sütun), 1'den
border wall          # alanın dört yanı kapalı
heading 270          # robot başlangıçta 11. satıra doğru (aşağı) bakar
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

- `0` boş, `1` engel, `*` başlangıç (1,7), `/` hedef (9,7).
- Koordinatlar zemindeki numaralar gibi **(satır, sütun)** ve **1'den** başlar.
- Engeller değişirse ilgili hücreyi `1` ya da `0` yapın. Hedefi taşımak için `/`'ı, başlangıcı taşımak için `*`'ı başka hücreye yazın.
- `heading`: 0 = 8. sütuna doğru (sağa), 90 = 1. satıra doğru (yukarı), 180 = 1. sütuna doğru (sola), 270 = 11. satıra doğru (aşağı).

## 3. Gerçek robotla çalıştırma

1. **Hücre boyunu ölçün** (iki kırmızı çizgi arası) ve `lab.txt`'deki `cell` değerini metre olarak yazın. Şu anki 0.43 fotoğraftan tahmindir.
2. Kobuki'yi açın ve USB'yi takın. `.\run.ps1 -ListPorts` ile `IsFtdi = True` olan portu görün.
3. `.\run.ps1 10` çalıştırın. Program planı gösterip Kobuki'ye bağlanır.
4. Robotun **merkezini** (1,7) hücresinin ortasına koyun ve `heading` yönüne (varsayılan: 11. satıra doğru) çevirin.
5. `Enter`'a basın. Robot her köşede yerinde döner ve bir sonraki köşeye düz gider.
6. Robot durunca hedef hücrenin ortasına olan uzaklığı ölçüp ekrandaki "Goal error" ile karşılaştırın.

**Dikkat:**
- Konum yalnızca odometriden (jiroskop + tekerlekler) gelir. Yol uzadıkça birkaç cm kayma olabilir.
- Haritada olmayan bir şeye çarparsa robot durur, 10 cm geri çekilir, o hücreyi engel olarak ekler ve yeniden planlar.
- İlk denemede robotun yanında durun.

| Tuş (pencere ya da konsol) | Etki |
|---|---|
| `Enter` | Sürüşü başlat |
| `Boşluk` | Duraklat / devam |
| `Q` / Esc / Ctrl+C / pencereyi kapat | Dur |

## 4. Ekran ve çıktılar

**Pencere:**
- Koyu kırmızı = engel.
- Açık mavi = A*'ın açtığı hücreler.
- Mavi çizgi = A* yolu.
- Yeşil çerçeve = robot gövdesinin kaplayacağı alan.
- Sürüş sırasında robot ve geçtiği iz de çizilir.
- Sağ panelde sayılar ve yol noktaları listelenir.

**Konsol:** haritanın metin hali yazılır.

```text
# engel   + robot merkezi giremez   . boş   * başlangıç   / hedef   o A* yolu   X tamponla bulunan engel   R robot (sürüş sonunda)
```

Sonuçlar [`10_AStarPathPlanning/output/`](output/) klasörüne kaydedilir:
- `lab_<zaman>_plan.png` / `_plan.txt`: plan;
- `lab_<zaman>_run.png` / `_run.txt`: sürüşten sonraki durum.

## 5. Program seçenekleri

```text
AStar.exe [harita.txt] [seçenekler]                yalnızca plan
AStar.exe [harita.txt] --sim [seçenekler]          plan + simülatör
AStar.exe [harita.txt] <Kobuki COM> [seçenekler]   plan + gerçek robot
```

Harita verilmezse `maps/lab.txt` kullanılır. `run.ps1` ile başka dosya: `-Map yol\harita.txt`.

| Seçenek | Etki |
|---|---|
| `--start r,c` / `--goal r,c` | Başlangıç / hedef hücresi, dosyadaki `*` / `/` yerine (`run.ps1`: `-Start`, `-Goal`) |
| `--heading d` | Başlangıç yönü, derece |
| `--cell m` | Hücre boyu (m) |
| `--radius m` | Robot yarıçapı, varsayılan 0.15 |
| `--margin m` | Yarıçapa ek güvenlik payı |
| `--4` | 4 komşulu arama (çapraz hareket yok) |
| `--smooth` | Yolu görüş hattı ile kısalt (daha az dönüş) |
| `--strict` | Hedef engele çok yakınsa kaydırma, dur |
| `--block r,c` | Yalnızca `--sim`: haritada **olmayan** bir engel ekle |
| `--no-gui` / `--fast` / `--out klasör` | Pencere yok / simülasyonu beklemeden koştur / kayıt klasörü |

## 6. Sorun giderme

| Mesaj | Anlamı / çözüm |
|---|---|
| `MAP ERROR: ... line N: ...` | Dosyanın N. satırı hatalı: 0/1/`*`/`/` dışında karakter, satır uzunlukları farklı ya da iki `*` / iki `/`. |
| `the goal cell (r,c) is an obstacle` | Hedef bir engelin üstünde. Başka hücre seçin. |
| `the goal cell (r,c) is outside the map` | Labirent 11 satır × 8 sütun. |
| `A*: NO PATH` | Hedefe giden her yol kapalı. |
| `Bumper hit ... added to the map` | Haritada olmayan engel; robot yeniden planladı. |
| `too many bumps` / `no path left` | Harita zeminle uyuşmuyor; `lab.txt`'yi düzeltin. |
