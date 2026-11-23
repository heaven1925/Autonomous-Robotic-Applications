# A* Yol Planlama — Özet, Build ve Run (Örnek 10)

## Ne yapıyor

Kobuki, zemindeki 8 sütun × 11 satırlık labirentte `*` başlangıçtan `/` hedefe A* ile en kısa yolu bulup sürer. Sensör kartı (HC-SR04) kullanılmaz: robot yalnızca haritaya, jiroskopa (yön) ve tekerlek ölçümlerine (mesafe) göre gider.

### Harita ([maps/lab.txt](maps/lab.txt))

```text
cell 0.40            # hücre kenarı 40 cm (ölçüldü)
coords row-col 1     # (satır, sütun), 1'den – zemindeki numaralar
border wall          # alanın dört yanı kapalı
heading 270          # robot 11. satıra doğru bakar
1 0 0 0 0 0 * 1      # 0 boş, 1 engel, * başlangıç (1,7)
...
1 0 0 0 0 0 / 1      # / hedef (9,7)
```

### Programın adımları

1. Haritayı okur; hatalı satır varsa numarasıyla söyler.
2. Merkezi bir engele robot yarıçapından (15 cm) yakın olan hücreleri robot merkezine yasaklar. 40 cm hücrede hiçbir hücre yasaklanmaz; robot merkezi engellere en az 20 cm uzak kalır.
3. A* ile en kısa yolu bulur: 8 yön, köşe kesme yok, tahmini mesafe octile. Bu labirentte yol 12 hücre, 4,90 m, 7 düz bacaktır.
4. Robot her köşede yerinde döner ve bir sonraki köşeye düz gider.
5. **Çarparsa:**
   - Bilinen bir engele (panel, duvar) çarptıysa konum tahminini düzeltir.
   - Bilinmeyen bir engele çarptıysa o hücreyi haritaya ekler, 10 cm geri çekilir ve yeniden planlar.
6. **Uçurum sinyali gelirse** hemen durur; sinyal 1,5 saniyeden uzun sürerse programı bitirir.

### Gerçek robot denemeleri (11 Kasım 2026)

| Deneme | Sonuç |
|---|---|
| 1 | Panele çarpma döngüsü, sonra uçurum yanlış alarmı. Bunun üzerine iki düzeltme yapıldı: çarpmayla konum düzeltme ve uçurum sinyalinde bekleme. |
| 2 | Çarpmadan hedefin yaklaşık 2 cm yakınına geldi; yeni deneme için yarıda kapatıldı. |
| 3 | Hedefe ulaştı: 82,5 s, 4,93 m, çarpma yok, hedef hatası 1,0 cm. |

Hedef hatası robotun kendi konum tahminine göredir; zeminde ölçülmemiştir.

---

## Build ve run

Depo kökünde (`AutonRobotSysWS`) PowerShell açın. `run.ps1` her seferinde önce derler, sonra çalıştırır; ayrı bir derleme komutu gerekmez.

### 1. Robotu hazırlayın

- Kobuki'yi açın ve USB'yi takın.
- `.\run.ps1 -ListPorts` çalıştırın; `IsFtdi = True` olan port Kobuki'dir (örneğin COM7).
- Robotun merkezini (1,7) hücresinin ortasına koyun ve 11. satıra doğru baktırın.

### 2. Çalıştırın

```powershell
.\run.ps1 10                 # derle + gerçek robot; plan gösterilir, Enter'a basınca sürer
.\run.ps1 10 -Port COM7      # port otomatik bulunmazsa elle verin
.\run.ps1 10 -Goal 6,2       # başka hedef (satır,sütun); dosyadaki /'ın yerine geçer
.\run.ps1 10 --go            # Enter beklemeden, 3 s geri sayımla başlar
```

**Robot olmadan:**

```powershell
.\run.ps1 10 -PlanOnly          # yalnızca planı gösterir ve kaydeder
.\run.ps1 10 -Sim               # simülatörde sürer
.\run.ps1 10 -Sim --block 6,6   # haritada olmayan bir engelle yeniden planlamayı dener
```

**Sürüş sırasında:** `Boşluk` duraklatır, `Q` ya da pencereyi kapatmak durdurur.

VS Code'da `Ctrl+Shift+B` → hedef `10` aynı `run.ps1`'i çalıştırır.

### 3. Sonuçlar

`10_AStarPathPlanning\output\` klasörüne kaydedilir:

- `lab_<zaman>_plan.png` / `.txt`: plan;
- `lab_<zaman>_run.png` / `.txt`: sürüş sonucu (hedef hatası, süre, çarpma sayısı).

### 4. Labirent değişirse

Yalnızca `lab.txt`'yi düzenleyin:

- engel `1`, boş `0`;
- başlangıç `*`, hedef `/`;
- hücre boyu değişirse `cell` satırı;
- robotun yönü değişirse `heading` (0 sağa, 90 yukarı, 180 sola, 270 aşağı).

Kod değişmediği sürece başka bir şeye dokunmanız gerekmez.

### Sorun çıkarsa

| Belirti | Çözüm |
|---|---|
| `Could not auto-detect the robot COM port` | Kobuki kapalı ya da takılı değil; `.\run.ps1 -ListPorts` ile bakın, gerekirse `-Port COMx` verin. |
| `MAP ERROR: ... line N` | `lab.txt`'nin N. satırı hatalı (0/1/`*`/`/` dışında karakter, eksik ya da fazla hücre, iki `*` ya da iki `/`). |
| Robot bir engele sürtüyor | `.\run.ps1 10 --margin 0.02` ile ek pay verin. 40 cm hücrede her yanda yalnızca 2,5 cm boşluk var. |
| `cliff detected` ile duruyor | Koyu ya da parlak zeminde uçurum sensörü yanlış alarm verebilir; robotu bulunduğu yerden alıp tekrar başlatın. |

Daha ayrıntılı anlatım: [USER_GUIDE.md](USER_GUIDE.md). Algoritma: [AStar.md](AStar.md).
