# Haritalama — Kullanıcı Kılavuzu (Kobuki + 3×HC-SR04)

Robot bilinmeyen bir odayı kendi başına dolaşır, haritasını çıkarır, başladığı yere döner ve haritayı kaydeder. Teknik ayrıntılar ve test sonuçları: [Mapping.md](Mapping.md).

## 1. Hızlı başlangıç

Depo kökünde PowerShell açın:

```powershell
.\run.ps1 9 -Sim            # Donanım OLMADAN: ödev görselindeki planın simülatörü
.\run.ps1 9 -Sim -Manual    # Simülatörde robotu klavyeyle sürün
.\run.ps1 9                 # Gerçek robot: derler, Kobuki ve Arduino portlarını bulur, başlatır
.\run.ps1 9 -Manual         # Gerçek robot, siz sürersiniz; harita yine çıkarılır
.\run.ps1 9 -Port COM6 -ArduinoPort COM8   # portları elle verin
```

Bir pencere açılır:

- **Sol panel:** 5 cm'lik olasılık haritası. Beyaz = boş, gri = bilinmiyor, koyu kırmızı = dolu. Ayrıca robot (yeşil), planlanan yol (mavi), frontier (turkuaz) ve hedef (turuncu ×) gösterilir.
- **Sağ panel:** 0.5 m'lik grid map, ödev görselindeki biçimde.

| Tuş (pencere ya da konsol) | Etki |
|---|---|
| `Q` / Ctrl+C / pencereyi kapat | Dur, haritayı kaydet |
| `R` | Resmi 90° döndür |
| `W A S D` / ok tuşları, `X`/Boşluk | Yalnız `-Manual`: ileri, sol, geri, sağ, dur |

İş bitince haritalar [`09_Mapping/output/`](output/) klasörüne kaydedilir: tam resim (`.png`), yalnız grid map (`_grid.png`) ve 0/1 metin haritası (`_grid.txt`: 0 boş, 1 dolu ya da bilinmiyor; 10_AStarPathPlanning'e doğrudan verilebilir). Pencere siz kapatana (ya da `Q`/Enter'a basana) kadar açık kalır.

## 2. Donanım

08 (Tangent Bug) ile **aynı** sensör kartı ve montaj:

| Sensör | Yön | Trig | Echo |
|---|---|---|---|
| Sol-ön | 60° sola | D9 | D10 |
| Ön | ileri | D11 | D12 |
| Sağ-ön | 60° sağa | D7 | D8 |

- Sensörler robot merkezinden ~11 cm uzakta, yere paralel takılıdır.
- Sketch: 08 ile ortak, [`08_TangentBugAlgorithm/arduino/sensor/sensor.ino`](../08_TangentBugAlgorithm/arduino/sensor/sensor.ino). 08'deki sketch zaten yüklüyse yeniden yüklemeye gerek yoktur.
- Montaj farklıysa [`src/main.cpp`](src/main.cpp) başındaki `SensorOffsetM`, `SideBeamDeg` ve `RobotRadiusM` sabitlerini güncelleyin.
- Sensörleri kontrol etmek için: `.\run.ps1 8 -SensorTest`.

## 3. Program seçenekleri

`run.ps1` dışında doğrudan da çalıştırılabilir:

```text
Mapping.exe <Kobuki COM> <Arduino COM> [seçenekler]
Mapping.exe --sim [seçenekler]
```

| Seçenek | Etki |
|---|---|
| `--manual` | Klavyeyle sürüş |
| `--no-gui` | Pencere yok (dosyalar yine kaydedilir) |
| `--fast` | Simülasyonu beklemeden koştur (birkaç saniye sürer) |
| `--noise` | Simülasyonda sensör gürültüsü ve kaçan yankılar |
| `--grid 0.25` | Grid map hücre boyutu (m), varsayılan 0.50 |
| `--radius 5` | Başlangıçtan bu kadar uzaktaki frontier'lar yok sayılır (gerçek robotta varsayılan 8 m) |
| `--time 600` | Azami keşif süresi (s), gerçek robotta varsayılan 1800 |
| `--out D:\maps` | Kayıt klasörü |

## 4. Ekrandaki durum satırı

```text
  DRIVE     L  91 F 164 R 238 cm | (2.40, -1.10)  -35 deg | explored 18.2 m2 | t 74.0 s
```

- **Durum:** `SCAN` (yerinde tam tur atıp etrafa bakıyor), `DRIVE` (frontier'a gidiyor), `BACK_OFF` (geri çekiliyor), `ALIGN` (başlangıçta ilk yönüne dönüyor), `DONE`.
- **L/F/R:** Sensörlerin ölçtüğü mesafeler (cm). `-1` = yankı yok, `-2` = bozuk paket.
- Ardından robotun konumu ve yönü, keşfedilen boş alan ve geçen süre gelir.

## 5. Güvenlik ve sorun giderme

- İlk denemeyi **küçük, kapalı bir alanda ve robotun yanında** yapın; açık kapılı odalarda robot dışarıyı da keşfetmeye çalışır (`--radius` ile sınırlayın).
- Hızlar: sürüş 0.18 m/s, etrafa bakma 0.5 rad/s. Robot önünde 38 cm'den yakın bir şey görürse ileri gitmez.
- **Tampon:** Robot durur, çarptığı yeri haritaya duvar olarak yazar, 12 cm geri çekilir ve yeniden planlar.
- **Uçurum** ya da 1.5 s **sensör kesintisi:** Robot durur ve program o ana kadarki haritayı kaydedip çıkar.

| Belirti | Çözüm |
|---|---|
| Portlar bulunamıyor | `.\run.ps1 9 -ListPorts`, sonra `-Port` / `-ArduinoPort` verin. |
| Duvarlar çift / kaymış görünüyor | Odometri kayması. Daha kısa keşif yapın (`--time`), robotu kaygan zeminde kullanmayın. |
| Cam / parlak duvar haritada yok | Sonar sığ açıda yankı almaz. Robotu duvara daha dik bakacak şekilde elle sürün (`-Manual`). |
| `STOPPED (robot got stuck too often)` | Alan robot için çok dar ya da çok dağınık. Engelleri azaltın. |
| `STOPPED (time limit reached)` | Alan büyük. `--time` artırın ya da `--radius` küçültün. |
