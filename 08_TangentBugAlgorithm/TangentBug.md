# Kobuki Robot ile Tangent Bug Navigasyonu (3×HC-SR04)

## 1. Amaç

Bu örnek, Kobuki diferansiyel sürüşlü robot üzerinde **Tangent Bug** hedefe gitme / engelden kaçınma algoritmasının, Bug0 örneğiyle (07) **aynı donanımla** — Arduino'ya bağlı sol-ön, ön ve sağ-ön üç HC-SR04 — çalışan yaklaşık bir uygulamasıdır.

| Dosya | Görev |
|---|---|
| [`TangentBugPlanner.h`](../01_Sample/KobukiNativeVS/include/TangentBugPlanner.h) | Donanımdan bağımsız karar çekirdeği (durum makinesi, sezgisel, nokta hafızası). |
| [`src/main.cpp`](src/main.cpp) | Uygulama: hedefleri klavyeden sırayla alır, her hedef için planlayıcıyı çalıştırır, canlı durum satırı ve güvenlik katmanı. |
| [`src/RobotIO.h`](src/RobotIO.h) | Robot arayüzü ve iki arka uç: gerçek robot (Kobuki + Arduino) ve dahili simülatör (`--sim`). |
| [`arduino/sensor/sensor.ino`](arduino/sensor/sensor.ino) | 3 sensörü sırayla okur, kanal başına 3'lü medyan filtre uygular, `{"l":..,"f":..,"r":..}` gönderir. |

Kurulum, sensör montajı, derleme ve kullanım için: [USER_GUIDE.md](USER_GUIDE.md). Bu belge algoritmanın teknik ayrıntılarını ve doğrulama sonuçlarını içerir.

## 2. Klasik Tangent Bug

Tangent Bug, sonlu menzilli (R) bir mesafe sensörü kullanan Bug ailesi algoritmasıdır. İki davranış arasında geçiş yapar:

1. **Hedefe hareket (motion-to-goal):** Hedef ışını R içinde açıksa doğrudan hedefe gidilir. Engel ışını kesiyorsa, menzil profilindeki **süreksizlik noktaları** (engel uçları, `O_i`) arasından

   \[ h(O_i) = d(x, O_i) + d(O_i, q_{goal}) \]

   sezgiselini en küçükleyen uca doğru ilerlenir. Bu en küçük değer azalmayı bıraktığında (yerel minimum) sınır takibine geçilir.
2. **Sınır takibi (boundary following):** Robot engelin kenarını izlerken iki değer tutar:
   - `d_followed` — takip edilen sınırda o ana kadar algılanan noktaların hedefe olan en küçük uzaklığı,
   - `d_reach` — şu anda görünen serbest alandaki noktaların hedefe olan en küçük uzaklığı.

   `d_reach < d_followed` olduğunda sınırdan ayrılır ve **d_reach noktasına** doğru hedefe hareketi sürdürür. Robot sınır boyunca tam tur atıp başladığı noktaya dönerse **hedef erişilemezdir**.

Bug0'dan farkı: Bug0 yalnızca "hedef yönü açık mı?" diye bakar ve içbükey engellerde sonsuz döngüye girebilir; Tangent Bug menzil bilgisini kullanarak daha kısa yollar seçer ve erişilemeyen hedefi raporlayabilir.

## 3. Üç sensörle yaklaşık uygulama

Üç sabit HC-SR04 sürekli bir menzil profili vermez; bu yüzden klasik kavramlar şöyle yaklaşıklanır (`Config` alanları parantez içinde):

| Kavram | Bu uygulamadaki karşılığı |
|---|---|
| Sensör menzili R | `sensingRange` = 2.0 m; bundan uzun okumalar "bir şey görülmedi" sayılır. |
| Hedef ışını açık | Robot önce hedefe döner; ön ışın `min(R, d_goal)` kadar boş **ve** gövde koridoru hafızadaki noktalardan temiz olmalıdır. |
| Süreksizlik `O_i` | Ön ışın `d_f`'de engel görürken yan ışın düz bir duvarın vereceği `d_f / cos(beamAngle)` değerinden `jumpThreshold` (0.30 m) daha uzunsa ya da hiçbir şey görmüyorsa, engel o tarafta iki ışın arasında biter. Uç noktası tahmini, yüzeyin düz devam etseydi yan ışını keseceği noktadır (temkinli, dış tahmin). |
| Uca doğru ilerleme | Yan ışın yönünde en fazla `endpointMaxStep` (0.8 m) gidilir, sonra hedefe dönülüp yeniden değerlendirilir (`MOVE_TO_ENDPOINT`). Yeni `h`, en iyi `h`'den `heuristicEps` kadar küçük değilse yerel minimumdur → sınır takibi. |
| Taraf histerezisi | Seçilmiş taraf, diğer taraf `sideSwitchMargin` (0.3 m) kadar daha iyi değilse değiştirilmez. |
| `d_followed` | Takip edilen sınıra ait (robota `wallSeenRange` = 1 m'den yakın) yankıların hedefe uzaklığının minimumu. Uzaktaki başka bir engelin yankısı sayılmaz; sayılsaydı ayrılma koşulu hiç sağlanamazdı. |
| `d_reach` | Her ışının serbest kısmı (gövde + pay kadar kısaltılmış, hafızadaki noktalarla kırpılmış) üzerinde hedefe en yakın nokta. Ayrılınca robot doğrudan hedefe değil **bu noktaya** gider. |
| Tarama yok | Robot hiçbir zaman durup sensörleri etrafa çevirmez; tüm kararlar sürüş sırasında ışınların gördüğüyle verilir. `d_reach` her yeni pakette üç ışından hesaplanır. Yerinde dönüş yalnızca yakın bir engelden kaçarken, Recovery'de ve çıkmazdan U dönüşünde yapılır; üçü de boş bir yön görüldüğü **anda** biter. |
| Yumuşak (yay) hareket | Yön değişimleri "yerinde dön, sonra düz git" yerine yay olarak sürülür (`arcTo`): ileri hız = v<sub>max</sub>·cos(yön hatası). Küçük düzeltmeler geniş, büyük düzeltmeler dar yaylar olur; yalnızca 90°'den fazla ters yöndeki hedefe yerinde dönülür. Ön ışın `d_stop` içinde engel görürse ileri gidilmez. |
| Erken tepki | Takipte önde bir engel (iç köşe, sonraki duvar) `d_stopFollow + followReact` (0.4 + 0.5 = 0.9 m) mesafeden itibaren görülünce dönüş yumuşakça karıştırılır. Robot engelin dibine girip yerinde dönmez; sensörler HC-SR04'ün güvenilmez yakın menziline (< ~25 cm) girmez. |
| Erişilemez hedef | Aşağıdaki "Döngü kapanışı" bölümüne bakın. |

### Döngü kapanışı ("hedef erişilemez" kararı)

Kusurlu algılamada en tehlikeli hata, erişilebilir bir hedefe "erişilemez" demektir. Bu nedenle karar şu şartların **hepsine** bağlıdır:

1. Referans noktası, robot sınıra gerçekten oturduğunda (duvara mesafe `d_follow` ± 0.1 m) alınır. Takibe başlanan nokta duvardan uzak olabilir.
2. Referansta yola dik ±`loopGate` (0.6 m) genişliğinde bir **geçiş çizgisi** kurulur. Robot en az `loopMinTravel` yol aldıktan sonra bu çizgiyi **aynı yönde** yeniden geçerse tur kapanır. Yay şeklindeki turlar aynı izden geçmediği için tek bir noktaya yakınlık yerine çizgi kullanılır. Dar bir çıkmazdan U dönüşüyle geri çıkarken çizgi ters yönde geçilir; bu kapanış sayılmaz.
3. Toplam yön değişimi |Σ dönüş| > 1.5π olmalıdır. Gerçek bir tur ≈ ±2π, cepten geri çıkış ≈ ±π eder.
4. **Sarım sayısı kontrolü:** Robottan hedefe olan yönün tur boyunca dönüşü ölçülür.
   - Bir engelin **dışından** dönülüyorsa hedef turun **içinde** olmalıdır.
   - Bir alanın **içinden** dönülüyorsa hedef turun **dışında** olmalıdır.
   - Tutarsız bir tur (örneğin boş alan etrafında yay) erişilebilirlik hakkında bilgi vermez; robot tarafı değiştirip yeniden plan yapar.
5. **Teyit turu:** İlk geçerli kapanıştan sonra robot yerinde döner ve sınırı **ters yönde** bir kez daha dolaşır. Bir taraftan kaçırılan kapı çoğu zaman diğer pervazdan bulunur. `FAULT (GoalUnreachable)` ancak bu tur da kapanırsa verilir.

### Dar geçitler, çıkmazlar ve kilitlenme önlemleri

- **Kademeli yan güvenlik:** Bir yan ışın `robotRadius + hardSideMargin` (0.32 m) altındaysa gerçekten çok yakındır; robot yerinde döner ya da Recovery'ye geçer. `robotRadius + sideEmergencyMargin` (0.40 m) altında ama bunun üstündeyse robot durmaz; en düşük hızla o taraftan uzaklaşarak ilerler.
- **Dar ama geçilebilir geçit:** İki yan da sınırdaysa ve önü açıksa robot iki taraf arasında ortalanarak sürünür. Bu, hedefe hareket, uca hareket ve sınır takibi durumlarının üçünde de aynıdır.
- **Kaçış kilidi:** Sınır takibinde önü ya da duvar tarafı kapanınca robot, ikisi de açılana kadar aynı yöne döner. Böylece sağa-sola sırayla acil dönüşler birbirini kilitlemez; dar bir çıkmazda bu yerinde bir U dönüşüne dönüşür.
- **Takılma algılama:** `stallTimeS` (20 s) içinde `stallDist` (0.3 m) dışına çıkılamamış ve net dönüş < 0.75π ise robot yerinde döner ve takibe devam eder. Dönüş en az 90°'den sonra önü açılınca hemen biter; en fazla 180° sürer.
- **Recovery:** Robot daha boş olan tarafa doğru yerinde döner. Önü `recoveryClear` (0.9 m) kadar boş ve yanlar güvenliyse **hemen** yola çıkar; tam tur (2π) dönüp boş yön bulamazsa `FAULT (NoSafeMotion)`.
- **Recovery sayacı** yalnızca robot aynı yerde kaldıkça artar. Son Recovery noktasından `progressReset` (1 m) uzaklaşınca sıfırlanır. Derin bir fincanın etrafı gibi önce hedeften uzaklaşmayı gerektiren yollar bu yüzden arızaya yol açmaz.
- **Duvar kaybı** süreyle değil, duvarı görmeden dönülen açıyla ölçülür (`wallLostTurn` = 1.75π). İnce bir duvarın ucunu dönmek ~π gerektirir ve yavaş takip hızında uzun sürer.
- **Dış köşeler:** Yan ışın duvarı kaybedince kör bir yay yerine hafızadaki en yakın sınır noktasının (≤ `followMemoryRange` = 0.8 m) etrafında `d_follow` yarıçapıyla dönülür. Düz duvarlarda ışın kontrolü kullanılır, çünkü koni şeklindeki gerçek ışınlarda hafızadaki noktalar bulanıklaşır.

### Nokta hafızası (neden gerekli?)

Tek bir ultrasonik ışın, gövdenin (Ø ≈ 0.36 m) bir engel köşesinin yanından geçip geçemeyeceğini söyleyemez: simülasyonda hedef ışını köşeyi milimetrik sıyırdığında ışın "açık" görüyor ama gövde köşeye çarpıyordu. Bu nedenle planlayıcı, `memoryMaxRange` (1.5 m) içindeki her yankıyı odometri çerçevesinde `memoryTimeS` (60 s) boyunca saklar. Her düz hareket kararı (hedefe, uca, d_reach noktasına) gövde yarıçapı + `bodyMargin` genişliğindeki koridorun bu noktalardan temiz olmasını da şart koşar. Yerinde dönüş, yuvarlak Kobuki için her zaman serbesttir.

Not: Hafıza odometriye dayanır; uzun sürüşte kayma (drift) hafızadaki noktaları kaydırır. Süre sınırı bu etkiyi sınırlar.

## 4. Durumlar

| Durum | Davranış | Sonraki |
|---|---|---|
| `INIT` | Sayaçları sıfırlar. | `MOTION_TO_GOAL` |
| `MOTION_TO_GOAL` | Hedefe doğru **yay çizerek** döner; hizalanınca (±10°) hedef ışını açıksa ilerler. Engel teyit edilirse uç adaylarını değerlendirir. | `MOVE_TO_ENDPOINT`, `FOLLOW_BOUNDARY`, `RECOVERY` |
| `MOVE_TO_ENDPOINT` | Seçilen uca ya da d_reach noktasına **yay çizerek** gider. Her pakette hedef ışınını yoklar; açılırsa hemen hedefe yönelir. Gidiş yönünde önü veya duvar tarafı yakınsa sınır takibine geçer. | `MOTION_TO_GOAL`, `FOLLOW_BOUNDARY`, `RECOVERY` |
| `FOLLOW_BOUNDARY` | Duvarı seçilen tarafta `d_follow` (yanal 0.5 m) mesafede izler. İç köşeleri erken tepkiyle, dış köşeleri hafızadaki köşe noktası etrafında yayla döner. `d_followed`/`d_reach` günceller, döngü kapanışını ve takılmayı izler. | `MOVE_TO_ENDPOINT`/`MOTION_TO_GOAL` (ayrılma), `RECOVERY`, `FAULT` |
| `RECOVERY` | Boş tarafa doğru yerinde döner; boş bir yön görünce **hemen** yeniden planlar. Tam turda yön bulunamazsa ya da `maxRecoveryAttempts` aşılırsa arıza. | `MOTION_TO_GOAL`, `FAULT` |
| `GOAL_REACHED` | Sıfır hız. | Terminal |
| `FAULT` | Sıfır hız. Neden şunlardan biri: `GoalUnreachable` (sınır iki yönde de dolaşıldı), `GoalBlocked` (hedefe yakınken ön ışın hedefin hemen arkasında engel görüyor, toplam `goalBlockedTimeS` = 30 s), `Timeout` (`maxGoalTimeS`; uygulamada 600 s) veya `NoSafeMotion`. | Terminal |

Bumper/cliff olayları, sensör zaman aşımı (1.5 s) ve Ctrl+C, planlayıcıdan **bağımsız** olarak `main.cpp` içinde robotu durdurur.

## 5. Zaman tabanlı histerezis

Tüm teyit süreleri saniye cinsindendir (`confirmTimeS` = 0.25 s, `stallTimeS`, `wallLostTimeS` …) ve `main.cpp` planlayıcıya **gerçekte geçen süreyi** (`dt`) verir. Böylece davranış 100 Hz kontrol döngüsünde, ~10 Hz sensör akışında ve Windows'un düzensiz `sleep` süresinde aynı kalır. (Bug0Planner'daki tick sayısına dayalı sayaçlar döngü hızına bağlıdır; burada bilerek kullanılmadı.)

## 6. Doğrulama

Planlayıcı iki ışın izleme simülatörüyle test edildi.

**Temel set:** 17 senaryo, 20 Hz ve 100 Hz adım hızlarında; **tamamı geçti**, çarpışma yok. Kapsam:

- boş ortam, önde/kaymış/geniş duvar, kutu, koridor, zikzak, L köşe;
- arkası açık U tuzağı, duvarın hemen önündeki hedef;
- gürültü, veri kaybı ve konik ışınla çalışan varyantlar;
- kapalı kutunun içindeki hedef (`GoalUnreachable`).

**Gerçek sensör zamanlaması:** Simülatör Arduino'yu birebir taklit eder: paketler 10 Hz'de gelir ve her biri son 3 ölçümün ortancasıdır. Planlayıcı da uygulamadaki gibi yalnızca yeni paket geldiğinde çalışır. Varsayılanlarla (yan sensör 60°, dönüş 0.4 rad/s, `d_follow` 0.5 m) aşağıdaki 95 senaryonun (10 demo odası hedefi dahil) **95**'i de geçer:
- çarpışma yok;
- koni modelinde en küçük merkez-duvar mesafesi 0.29 m;
- yerinde dönme süresinin ortalama payı %6.6.

Aşağıdaki ayrıntılı tablo eski, tarama yapan (peek) sürüme ait ideal-model sonuçlarıdır. Taramasız sürüm ideal modelde 89/95 geçer; bu model gerçek uygulamada kullanılmaz.

Uygulamada planlayıcı her yeni pakette bir kez çalışır. Aynı eski okumayı 100 Hz'lik döngüde tekrar tekrar işlemek nokta hafızasını bulandırıyordu: bu şekilde 6 hata vardı, yalnızca yeni pakette çalışınca 3'e indi, dönüş hızı 0.4 rad/s'ye düşünce 2 kaldı.

Mesafeler planlayıcıya robot **merkezinden** verilir: `RobotIO` her yankıya sensör montaj ofsetini (11 cm) ekler. Yön jiroskoptan alınır; konum, enkoder mesafesinin jiroskop yönüyle entegre edilmesiyle hesaplanır.

**Labirent / karmaşık set:** Her senaryonun gerçek cevabı, robot yarıçapıyla şişirilmiş bir ızgarada BFS ile bağımsız olarak hesaplandı:

- **Erişilebilir:** ≥ 0.84 m'lik geçitlerle (yarıçap 0.42 m) yol var → robot hedefe ulaşmalı.
- **Erişilemez:** İnce bir gövde (0.19 m) bile geçemiyor → `GoalUnreachable` beklenir.
- **Arası:** Yalnızca çarpışmasızlık beklenir.

Pratikte erişilebilir bir hedefe "erişilemez" demek başarısızlık sayıldı. Sensör modeli varsayılan olarak HC-SR04 benzeri ±10° konidir.

| Grup | Sonuç |
|---|---|
| Elle tasarlanmış (fincan, derin fincan, kapılı odalar, S yolu, spiral, cep dizisi, kapalı iç oda / kutu / başlangıç odası, gövdeden dar tek çıkış), 2 adım hızı | 20/22 |
| Rastgele "mükemmel" labirent 3×3, 4×4, 5×5 (1.4 m hücre, 6'şar tohum) | **18/18** |
| Döngülü 5×5 labirent | **6/6** |
| Dağınık kutu alanı 6×6 m (20 tohum) | 17/20 |
| 4×4 labirent, ±15° koni + ±2 cm gürültü + her 9 adımda veri kaybı | 2/4 |
| Stres: ideal tek ışın (koni yok) | 14/15 |
| **Toplam** | **77/85** |

- Gerçekten erişilemez olan 8 vakanın (2 adım hızı × 4 senaryo) **hepsi** doğru şekilde `GoalUnreachable` raporladı.
- Koni modelinde **hiç çarpışma yok**; en küçük gövde merkezi-duvar mesafesi 0.33 m (gövde kenarına ~15 cm).
- Kalan başarısızlıkların hepsi incelendi:
  - **Cep dizisi (2):** 0.8 m genişliğinde, robotu iki tarafından da sıkıştıran ceplerde ilerleme yok. Robot güvenli kalıyor ama zaman aşımına uğruyor.
  - **Dağınık alan (3):** Hedef yalnızca 0.84–1.1 m'lik dar geçitlerden erişilebilir. Robot bu geçitleri bulamayıp iki turdan sonra "erişilemez" diyor. Geçit ≥ 1.1 m sayıldığında bu üç vaka beklenen sonuca dönüşüyor.
  - **Ağır gürültü (2):** Sık veri kaybı nedeniyle tekrarlanan Recovery sonrası `FAULT (NoSafeMotion)` ile güvenli duruş.
  - **Tek ışın spiral (1):** Bkz. bilinen kısıtlar, "ışınlar arası kör nokta".

## 7. Bilinen kısıtlar ve saha testi

- Üç ışın sürekli profil değildir: uç noktaları tahmindir, ışınların arasında kalan ince engeller (masa ayağı vb.) görülmeyebilir. İlk denemeleri düşük hızda ve gözetim altında yapın.
- Yan sensör açısı (60°) ve montaj ofseti (11 cm) gerçek montajla eşleşmelidir ([`src/main.cpp`](src/main.cpp): `SideBeamDeg`, `SensorOffsetM`). Yanlış değerler uç tahminlerini ve duvar takibini bozar.
- Speküler yansıma veya yumuşak yüzeyler kısa/uzun sahte okumalar üretebilir; sketch'teki medyan filtre tek seferlik hataları bastırır ama sürekli hatalı yüzeyleri düzeltemez.
- Hafıza odometri kaymasından etkilenir; çok uzun görevlerde `memoryTimeS` kısaltılabilir.
- **Işınlar arası kör nokta:** Robota uç kısmıyla bakan ince bir duvar (ör. bölme ucu) tam ön ışın ile yan ışın arasında kalırsa görülmeyebilir. İdeal tek ışın simülasyonunda bu bir çarpışmaya yol açtı. Gerçek HC-SR04'ün ~±15°'lik konisi bu ucu ön ışınla görür (±10° koni modelinde çarpışma yok), ama ince ve açılı yüzeylerde yine de dikkatli olun.
- **`GoalUnreachable` anlamı:** "Sınır iki yönde de dolaşıldı ve robotun güvenle geçebileceği (pratikte ≳ 1.1 m) bir açıklık bulunamadı." Hedef 0.85–1.1 m arası geçitlerin arkasındaysa bu rapor yanlış olabilir.
- **Dar cepler:** Genişliği ≈ 2·`d_follow` (1.0 m) altındaki çıkmaz ceplerde sınır takibi ilerleyemeyebilir. Robot güvenli kalır ama hedefe ulaşmayabilir.
- **Uzun görevler:** Karmaşık ortamlarda sınır takibi onlarca metre ve dakikalarca sürebilir (0.08 m/s). `maxFollowTimeS` (900 s) bir takip bölümünün üst sınırıdır.
- Gövdeden dar geçitler (bir yan ışın `robotRadius + hardSideMargin` altında) güvenlik gereği reddedilir.

Önerilen saha sırası: (1) boş zemin, 1–2 m hedef; (2) tek kutu; (3) duvar köşesi; (4) U engel; (5) kapalı alan içinde hedef (erişilemez raporu).
