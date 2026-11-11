# Tangent Bug — Kullanıcı Kılavuzu (Kobuki + 3×HC-SR04)

Kobuki robotu, üç ultrasonik sensörle engellerin etrafından dolaşarak klavyeden girilen `(x, y)` hedeflerine götüren uygulama.

---

## 1. Tangent Bug nedir?

Engel haritası **olmadan**, yalnızca kendi konumunu (odometri) ve yakın çevresindeki mesafeleri bilen bir robotun hedefe gitmesini sağlayan bir yol bulma algoritmasıdır. İki davranış arasında geçiş yapar:

1. **Hedefe git:** Hedef yönü açıksa düz gider. Bir engel yolu kesiyorsa, sensörlerin gördüğü **engel uçlarından** (kenarlardan) toplam yolu en kısa tahmin edileni seçer:
   `robot → uç + uç → hedef`
2. **Sınırı takip et:** Bu tahmin artık kısalmıyorsa (robot bir "çukura" girdiyse) engelin kenarını izler. Kenardan görünen boş alanda, şimdiye kadar takip edilen kenardan **hedefe daha yakın** bir nokta görünce kenarı bırakıp hedefe döner.

Robot engelin çevresini **iki yönde de** dolaşıp hiçbir geçit bulamazsa *"hedef erişilemez"* der ve durur.

**Bu uygulamada hareket biçimi:**
- **Tarama yok:** Robot durup etrafa bakmaz; kararları sürüş sırasında üç sensörün gördüğüyle verir.
- **Yumuşak yaylar:** Yön değişimleri "yerinde dön, sonra düz git" değil, yay şeklindedir.
- **Kısa algılama mesafesi:** 1.2 m'den uzaktaki yankıları "boş" sayar; böylece sensör konisi uzaktaki bir labirent boşluğunu duvar gibi göstermez. Önündeki engelde ~0.65 m'den yavaşlar, ~0.4 m'de durur ya da döner. Kenarı ~0.4 m mesafeden izler, bu da 0.8 m'lik bir koridorun ortasıdır. Gövdesinin (Ø 0.36 m) sığdığı ~0.5–0.6 m'lik boşluklardan geçmeye çalışır.
- **Açık yolda hemen hareket:** Sıkışıp dönerken açık bir yol görünce dönmeyi bitirmeyi beklemeden o yöne gider.

**Bug0'dan farkı:** Bug0 yalnızca "hedef yönü açık mı?" diye bakar ve U şeklindeki engellerde döngüye girebilir. Tangent Bug mesafe bilgisini kullanarak daha kısa yollar seçer ve erişilemeyen hedefi tanır.

**Ekranda göreceğiniz durumlar:**

| Durum | Anlamı |
|---|---|
| `MOTION_TO_GOAL` | Hedefe dönüyor / düz gidiyor |
| `MOVE_TO_ENDPOINT` | Seçtiği engel ucuna gidiyor |
| `FOLLOW_BOUNDARY` | Engel kenarını izliyor (`wall on left/right`) |
| `RECOVERY` | Sıkıştı: boş tarafa dönüyor, açık yön görünce hemen devam ediyor |
| `GOAL_REACHED` / `FAULT` | Hedefe ulaştı / gidemedi (erişilemez ya da güvenli yol yok) |

> Ayrıntılı teknik açıklama ve test sonuçları: [TangentBug.md](TangentBug.md)

---

## 2. Donanım ve sensör yerleşimi

```text
              ön (+x, robotun başlangıçtaki burnu)
                          ▲
           SOL-ÖN         │ ÖN            SAĞ-ÖN
            +60° ╲        │ 0°         ╱ −60°
                  ╲       │          ╱
                   ┌──────┴──────┐
       sol (+y) ◄  │   KOBUKI    │
                   │  (üstten)   │
                   └─────────────┘
```

| Sensör | Bakış yönü | Trig | Echo |
|---|---|---|---|
| Sol-ön | Önden **60° sola** | D9 | D10 |
| Ön | Tam **ileri** | D11 | D12 |
| Sağ-ön | Önden **60° sağa** | D7 | D8 |

- Sensörler robot merkezinden **~11 cm** uzakta, yere paralel ve **aynı yükseklikte** takılı. Zemini ya da robotun kendi gövdesini görmemeleri gerekir.
- VCC → 5 V, GND → GND (üçü ortak).
- Program her ölçüme bu 11 cm'yi ekleyip mesafeleri **robot merkezinden** hesaplar. Montaj değişirse [`src/main.cpp`](src/main.cpp) başındaki şu sabitleri güncelleyin:
  - `SensorOffsetM`: merkezden sensörlere uzaklık (m),
  - `SideBeamDeg`: yan sensör açısı (derece),
  - `RobotRadiusM`: gövde yarıçapı (m).
- **Arduino Uno** (CH340): USB ile PC'ye. **Kobuki**: kendi USB (FTDI) kablosuyla PC'ye.

### Arduino kodunu yükleme

1. Arduino IDE ile [`arduino/sensor/sensor.ino`](arduino/sensor/sensor.ino) dosyasını açın.
2. *Board:* **Arduino Uno**, *Port:* Arduino'nun COM portu → **Upload**.
3. Kontrol: *Serial Monitor* (115200 baud) ekranında saniyede ~10 satır `{"l":32,"f":40,"r":27}` görünmeli. `-1` = yankı yok (menzil dışı). Sonra Serial Monitor'ü **kapatın**; port aynı anda tek programa açılabilir.

### Sensör testi (yalnız Arduino, Kobuki gerekmez)

```powershell
.\run.ps1 8 -SensorTest
```

```text
  raw  L   -1  F   32  R   51 cm   | centre  L  free  F 0.43  R 0.62 m   | 10.1 Hz
```

- **raw:** sensörlerin ölçtüğü değer (cm).
- **centre:** planlayıcının kullandığı, robot merkezinden mesafe (m).
- `free` = yankı yok, o yön boş sayılır.
- Hız **~10 Hz** olmalı.

Her sensörün önüne elinizi tek tek tutun; yalnızca **kendi** sütunu değişmeli. Çıkmak için **Ctrl+C**.

---

## 3. Gereksinimler

- Windows 10/11, **Windows PowerShell 5.1** (yerleşik) veya PowerShell 7.
- **Visual Studio 2022+** veya **Build Tools**, *"Desktop development with C++"* iş yüküyle. CMake ve Ninja bu iş yüküyle birlikte gelir. Betik derleyiciyi kendisi bulur.

---

## 4. Derleme ve çalıştırma (PowerShell)

Depo kök klasöründe (`AutonRobotSysWS`) PowerShell açın:

```powershell
.\run.ps1 8 -Sim          # Donanım OLMADAN dene: dahili simülatör
.\run.ps1 8               # Gerçek robot: derler, portları bulur, çalıştırır
```

`run.ps1` her seferinde şunları yapar:
1. Açık robot programlarını kapatır (COM portları serbest kalsın diye).
2. Projeyi derler.
3. Kobuki (FTDI) ve Arduino (CH340) portlarını otomatik bulur.
4. Programı başlatır.

| Seçenek | Ne işe yarar |
|---|---|
| `-Sim` | Simülatör modu; COM portu gerekmez |
| `-SensorTest` | Yalnız Arduino ile sensör testi (Kobuki gerekmez) |
| `-Port COM6 -ArduinoPort COM8` | Portları elle ver (otomatik bulma yanlışsa) |
| `-ListPorts` | Bağlı COM portlarını listele |
| `-NoBuild` | Derlemeden son derlenen programı çalıştır |
| `-Config release` | Optimize (Release) derleme |
| `.\run.ps1 stop` | Çalışan robot programını durdur |

Betik çalışmazsa (*"running scripts is disabled"*) bir kez şunu çalıştırın:
`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`

**Elle derleme (isteğe bağlı)**, *Developer PowerShell for VS* içinde:

```powershell
cmake --preset x64-debug
cmake --build --preset x64-debug --target TangentBug
.\out\build\x64-debug\08_TangentBugAlgorithm\TangentBug.exe --sim
.\out\build\x64-debug\08_TangentBugAlgorithm\TangentBug.exe --sensors COM13              # yalnız Arduino
.\out\build\x64-debug\08_TangentBugAlgorithm\TangentBug.exe COM6 COM8   # Kobuki, Arduino
```

---

## 5. Kullanım

- Program açıldığı anda robotun durduğu yer **(0, 0)** olur:
  - **+x** robotun o anki baktığı yön,
  - **+y** robotun solu.
- Hedefleri metre cinsinden `x y` olarak yazın. Robot gider ve bir sonraki hedefi sorar. Koordinatlar hep **başlangıç noktasına** göredir.
- Hedefi engellerden **en az 0.35 m** uzakta seçin; robot gövdesi (Ø 0.36 m) ve güvenlik payı nedeniyle daha yakın noktalara varılamaz.

```text
Robot at (0.00, 0.00).  Target x y > 2 0
  Going to (2.00, 0.00)...
  [MOTION_TO_GOAL -> FOLLOW_BOUNDARY] wall on right
  FOLLOW_BOUNDARY  L  51 F 169 R  44 cm | (1.10, 0.70)  35 deg | goal 1.12 m
  ...
  Result      : GOAL REACHED
Robot at (1.98, 0.02).  Target x y > q
```

| Giriş | Etki |
|---|---|
| `2 0`, `1.5 -0.5`, `1,5 -0,5`, `2, 0` | Hedef (ondalık nokta veya virgül) |
| `q` | Çıkış (robot durur) |
| **Ctrl+C** (hareket sırasında) | Mevcut hedefi iptal et; yeni hedef sorulur |

Canlı satırda `L/F/R` sol-ön, ön ve sağ-ön sensörlerin ölçtüğü mesafelerdir (cm; `-1` = yankı yok). Ardından robotun konumu, yönü ve hedefe kalan mesafe gelir.

**Simülatör odası** (`-Sim`, 8×8 m):
- Önde bir kutu (x 1.2–1.8),
- x = 3.2'de uzun bir duvar,
- (3.8–5.0, 1.8–3.2)'de sola açık bir cep.

Deneyin: `4 0`, `4.5 -1`, `4.4 2.5`, `-1 3`.

---

## 6. Güvenlik ve sorun giderme

- İlk denemeleri **açık alanda, kısa hedeflerle (1–2 m) ve robotun yanında** yapın.
- Robotun **tamponu** veya **uçurum sensörü** tetiklenirse ya da sensör verisi 1.5 s kesilirse robot durur ve program kapanır.
- Hızlar:
  - düz gidiş 0.15 m/s,
  - kenar takibi 0.08 m/s,
  - dönüş 0.4 rad/s (sensörlerin 10 Hz hızına göre seçildi).

  Karmaşık ortamlarda bir hedef birkaç dakika sürebilir.

| Belirti | Çözüm |
|---|---|
| `Could not auto-detect ... COM port` | Kabloyu ve Kobuki gücünü kontrol edin. `-ListPorts` ile bakıp `-Port`/`-ArduinoPort` verin. |
| `No sensor data on COMx` | Sketch yüklü mü? Serial Monitor kapalı mı? Arduino portu doğru mu? |
| `ERROR opening sensor port` / port meşgul | `.\run.ps1 stop`, Arduino IDE'yi kapatın. |
| Robot engelleri geç görüyor / duvara fazla yaklaşıyor | `-SensorTest` ile değerleri kontrol edin. Sensör açısı ve uzaklığı `SideBeamDeg` / `SensorOffsetM` ile aynı olmalı. |
| `GOAL UNREACHABLE` | Hedef kapalı bir alanda ya da geçitler robot için dar (pratikte ≲ 1.1 m geçit güvenilir bulunamayabilir). |
| `GOAL TOO CLOSE TO AN OBSTACLE` | Hedef bir engele yapışık; robotun güvenlik payıyla oraya varılamaz. Hedefi engellerden **≥ 0.4 m** uzakta seçin. |
| `GAVE UP (time limit ...)` | Bir hedef için 10 dakikalık süre doldu (çoğunlukla engele çok yakın hedef). Süre `cfg.maxGoalTimeS` ile ayarlanır. |
| `STOPPED (no safe way forward)` | Robot sıkıştı veya sensör verisi çok gürültülü. Robotu açık alana alıp yeniden deneyin. |
| Konum kayıyor | Odometri zamanla kayar. Uzun görevlerde programı yeniden başlatarak (0,0)'ı tazeleyin. |

**Bilinen kısıt:** Üç sensör arasında kör açılar vardır. Robota ucuyla bakan ince nesneler (masa/sandalye ayağı, ince bölme ucu) geç görülebilir.
