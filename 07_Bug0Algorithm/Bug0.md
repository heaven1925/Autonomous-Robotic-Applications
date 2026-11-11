# Kobuki Robot ile Bug0 Engel Kaçınma Projesi

## 1. Proje amacı

Bu projede **Kobuki diferansiyel sürüşlü mobil robot** üzerinde **Bug0 (Bug Zero)** engelden kaçınma algoritması geliştirilecektir. Engel algılama için **Arduino'ya bağlanan üç adet HC-SR04 ultrasonik mesafe sensörü** kullanılacaktır. Sensörler robotun **sol ön, ön ve sağ ön** bölgelerini izleyecek; Arduino ölçümleri işleyecek ve robotun hareket kararlarını veren yazılıma aktaracaktır. Bug0 karar katmanı, hedefe doğrudan ilerlemek mümkünse hedefe yönelecek, engel hareketi kısıtlıyorsa seçilen engel kenarını takip edecek ve hedef yönündeki geçiş tekrar güvenli olduğunda hedefe ilerlemeye dönecektir.

**Hedef:** Donanım arayüzlerini algoritmadan ayıran, test edilebilir ve platform bağımsız bir Bug0 karar yazılımı geliştirmek. Kobuki ile haberleşme ve Arduino verisinin aktarımı bu çekirdeğin dışında kalan adaptörler üzerinden gerçekleştirilecektir.

## 2. Bug0 algoritmasının çalışma mantığı

Bug0, *go-to-goal* (hedefe git) ve *follow-boundary* (engel sınırını takip et) davranışları arasında geçiş yapan reaktif bir algoritmadır:

1. **Hedefe git:** Robot, mevcut pozundan hedefin doğrultusunu belirler ve güvenliyse o yönde ilerler.
2. **Engeli algıla:** Hedef doğrultusunda hareketi engelleyen bir nesne tespit edilirse robot güvenli biçimde yavaşlar veya durur.
3. **Engel sınırını takip et:** Robot, seçilmiş sağ veya sol tarafta engelle güvenli açıklığı koruyarak onun çevresinden dolaşır.
4. **Engelden ayrıl:** Hedefe doğru **doğrudan hareket etmenin güvenli olduğu doğrulandığında** sınır takibini bırakır ve yeniden hedefe ilerler.
5. **Hedefte dur:** Hedef toleransına ulaştığında hareketi sonlandırır.

> Bug0'ın temel ayrılma koşulu **hedef yönünün engelsiz olmasıdır**. Bug2'deki gibi başlangıç-hedef doğrusu (*M-line*) üzerine dönme zorunluluğu yoktur. Bug0 karmaşık/içbükey geometrilerde hedefe ulaşmayı garanti etmez; döngü ve sıkışma algılama eklenmelidir.

### Neden odometri gerekiyor?

HC-SR04 sensörleri **mesafe** verir; hedefin nerede bulunduğunu veya robotun hedefe göre yönünü tek başına belirlemez. Hedefe yönelmek ve hedefe ulaşıldığını anlamak için Kobuki'nin **tekerlek enkoder/odometri ve açısal yönelim** verisi ya da eşdeğer lokalizasyon bilgisi gereklidir. Odometri sürüklenmesi (drift) dikkate alınmalıdır.

## 3. Donanım mimarisi ve sorumluluklar

```text
    HC-SR04 Sol Ön       HC-SR04 Ön       HC-SR04 Sağ Ön
          |                   |                 |
          +-------------------+-----------------+
                              |
                       Arduino MCU
                  (tetikleme/echo ölçümü,
                  filtreleme, zaman damgası,
                    geçerlilik bayrakları)
                              |
                   Seri veri / USB / UART
                              |
                  Sensor Interface Adapter
                              |
Kobuki odometri ------> Bug0 Decision Engine
                              |
                      Motion Controller
                              |
                    Kobuki Driver Adapter
                              |
                    Kobuki Mobile Base
```

**İş bölümü:** Arduino yalnızca sensör ölçümünü güvenilir şekilde üretir; Bug0 durum makinesi ana kontrol uygulamasında çalışır. İstenirse fiziksel mimari değişebilir, ancak modüllerin sorumluluğu sabit kalmalıdır.

### Sensör konumlandırması

- **Ön sensör:** Robotun ilerleme yönündeki engeli denetler.
- **Sol ön sensör:** Sol ön çapraz yöndeki yakın engelleri gözler.
- **Sağ ön sensör:** Sağ ön çapraz yöndeki yakın engelleri gözler.
- Sensörlerin yönelim açıları ve robot gövdesine göre konumları kalibrasyon parametreleri olarak tanımlanır; örneğin ±45° yalnızca başlangıç önerisidir.
- **Kritik kısıt:** Üç HC-SR04, LiDAR benzeri sürekli çevre profili üretmez. Kör bölgeler, geniş ultrasonik hüzme, speküler yansımalar ve çapraz sensör paraziti nedeniyle **engelin sürekli sınırını ve hedefe giden hattın tamamen açık olduğunu kesin doğrulamak mümkün değildir**. Bu nedenle bu yapı bir **yaklaşık Bug0 uygulaması**dır. Özellikle köşe dönüşleri ve sensörlerin görmediği taraftaki engeller için hız sınırı, ek güvenlik tedbirleri veya ilave yan sensör gerekebilir.

### HC-SR04 ölçüm prensibi

Sensör, ultrasonik darbe gönderir ve dönüş süresini ölçer:

\[
d = \frac{c\,t}{2}
\]

Burada `c` havadaki ses hızı, `t` echo darbesinin gidiş-dönüş süresi ve `d` engelin yaklaşık mesafesidir. Sensörleri **sırayla** tetikleyin; echo için zaman aşımı tanımlayın, geçersiz ölçümleri işaretleyin ve ölçümler arası yeterli bekleme kullanın. Üç kanalı eşzamanlı tetiklemek çapraz yankı riskini artırır. HC-SR04'ün 5 V lojik seviyeleri ve kullanılan Arduino/Kobuki arayüzünün elektriksel uyumluluğu ayrıca doğrulanmalıdır.

## 4. Beklenen robot davranışı

### Durumlar

| Durum | Beklenen davranış | Sonraki durum |
|---|---|---|
| `INIT` | Sensör bağlantısı, Kobuki haberleşmesi, odometri ve hedefin geçerliliğini kontrol et; motor komutunu sıfırda tut. | `GO_TO_GOAL` veya `FAULT` |
| `GO_TO_GOAL` | Hedef açısına dön ve güvenli hızla ilerle. Ön/ilgili yön tehlikeli hale gelirse yavaşla veya dur. | `AVOID_OBSTACLE` |
| `AVOID_OBSTACLE` | Güvenli şekilde dur; uygun sağ/sol takip tarafını seç ve çarpışmasız dönüş yap. | `FOLLOW_BOUNDARY` |
| `FOLLOW_BOUNDARY` | Seçilen taraftaki engelle hedef açıklığı koruyarak düşük hızda çevresinden ilerle; sensör güvenilirliğini izle. | `GO_TO_GOAL`, `RECOVERY` veya `FAULT` |
| `RECOVERY` | Dönme/döngü, kayıp engel sınırı veya geçiş belirsizliğinde durup kontrollü yeniden yön belirleme uygula; güvenli çözüm yoksa dur. | `FOLLOW_BOUNDARY`, `GO_TO_GOAL` veya `FAULT` |
| `GOAL_REACHED` | Hedef pozisyon toleransında sıfır hız komutu gönder. | Terminal |
| `FAULT` | Geçersiz/eski sensör verisi, kayıp haberleşme, bumper/cliff olayı veya güvensiz durumlarda motor komutunu sıfırla. | Güvenli operatör müdahalesi/yeniden başlatma |

`AVOID_OBSTACLE`, `RECOVERY` ve `FAULT` **mühendislik amaçlı güvenlik/iyileştirme durumlarıdır**; klasik Bug0 çekirdeğinin iki temel hareket davranışı `GO_TO_GOAL` ve `FOLLOW_BOUNDARY` olarak kalır.

### Sensörlere bağlı karar örnekleri

| Sol ön | Ön | Sağ ön | Davranış (örnek) |
|---|---|---|---|
| Açık | Açık | Açık | Hedef yönü doğrulanabildiği ölçüde hedefe ilerle. |
| Açık | Engel | Engel | Güvenliyse sol kenardan dolaşmayı değerlendir. |
| Engel | Engel | Açık | Güvenliyse sağ kenardan dolaşmayı değerlendir. |
| Engel | Engel | Engel | Dur; açık dönüş alanı yoksa zorla ilerleme. |
| Ölçüm hatalı | Açık | Açık | Ölçüm güvenilirliği politikasına göre hızı azalt/dur; hatalı veriyi 'açık' sayma. |

Bu tablo yalnızca **başlangıç karar mantığıdır**. Engel takip tarafı belirlendikten sonra her döngüde keyfî olarak değiştirilmemelidir. Sınır takip geometrisi, robotun kendi yarıçapı, dönüş süpürme alanı ve sensör görüş açıları dikkate alınmalıdır.

### Kontrol ve güvenlik ilkeleri

- Hız kontrolü `v` (m/s) ve `ω` (rad/s) referansları üzerinden yapılmalı; Kobuki'nin izin verilen hız, ivme ve açısal hız sınırları ihlal edilmemelidir.
- `d_stop`, `d_slow`, `d_follow`, `d_clear` ve bunların histerezis değerleri, **ölçüm ve deneylerle** seçilmelidir; belgelenmemiş sabitler güvenlik garantisi gibi kullanılmamalıdır.
- **Durma mesafesi** robot hızı, algılama/işlem/iletişim gecikmesi ve frenleme kapasitesini kapsamalıdır.
- Hedefe dönme sırasında robotun **dönüş zarfının** boş olduğu doğrulanmalıdır; yalnızca ön sensörün açık olması yeterli değildir.
- Hedef doğrultusunun açık olduğu kararı en az birkaç ardışık geçerli ölçüm ve uygun güvenlik payıyla doğrulanmalıdır; tek bir echo ölçümüyle takipten ayrılınmamalıdır.
- `FOLLOW_BOUNDARY` durumunda engel takip edilecek tarafta artık görünmüyorsa robot körlemesine düz gitmemeli; hız azaltarak çevreyi yeniden değerlendirmelidir.
- Kobuki'nin bumper, cliff ve wheel-drop olayları varsa **Bug0 mantığından bağımsız bir güvenlik katmanında** izlenmelidir; gerekirse hareket kesilmelidir.
- Haberleşme kaybı, sensör timeout'u, verinin eskimesi ve kontrol yazılımının kilitlenmesi için watchdog ve güvenli duruş politikası uygulanmalıdır.
- HC-SR04'ün ince ayakları, eğimli/yumuşak yüzeyleri veya sensör görüşü dışındaki objeleri kaçırabileceği unutulmamalıdır. İlk testler düşük hızda, gözetim altında yapılmalıdır.

## 5. Önerilen yazılım modülleri

1. `UltrasonicAcquisition` — Arduino pin tetikleme, echo zamanlama, sıralı sensör döngüsü.
2. `SensorPacketProtocol` — çerçeveleme, kanal kimliği, zaman bilgisi, geçerlilik, paket bütünlüğü.
3. `SensorInterface` — seri port alıcısı, veri yaş kontrolü, ölçüm filtreleme.
4. `OdometryInterface` — Kobuki konumu, yönelimi, hızları ve veri geçerliliği.
5. `Bug0Planner` — durum makinesi, engel algılama, takip tarafı seçimi ve ayrılma kararı.
6. `BoundaryFollower` — sağ/sol sınır izleme ve hedef mesafeyi koruma davranışı.
7. `MotionController` — hız komutları, hız/ivme sınırlama ve güvenli yavaşlama.
8. `KobukiAdapter` — robotun gerçek sürüş protokolüne bağlanma.
9. `SafetySupervisor` — bumper, cliff, timeout, veri hataları, acil duruş.
10. `TelemetryAndTests` — hata ayıklama günlükleri, senaryo testleri ve performans ölçümü.

## 6. Doğrulama yaklaşımı

En az şu senaryolar ayrı ayrı test edilmelidir:

- Boş ortamda hedefe düz ilerleme ve hedefte durma.
- Önde tek engel: soldan ve sağdan çevresini dolaşma.
- Dar geçiş: robot genişliğine göre güvenli/güvensiz geçiş ayrımı.
- İçbükey/U biçimli engel: sıkışma/döngü tespiti ve güvenli duruş.
- Engel sınırının sensör görüşünden kaybolması.
- Ölçüm timeout'u, uç değer, ardışık ölçüm tutarsızlığı, seri port kopması.
- Hareket sırasında hedefin güncellenmesi ve yeni hedefe geçiş.
- Bumper veya cliff tetiklenmesi; kontrol ve sensör verilerinin durması.

Ölçümler: hedefe ulaşma oranı, çarpışma sayısı, minimum açıklık, rota uzunluğu, görev süresi, yeniden planlama/durum geçiş sayısı, algılama-karar süresi ve durma tepkisi.

---

# 7. Geliştirme için kullanılacak ana prompt

Aşağıdaki metni yazılım geliştirecek bir AI asistanına veya ekibe doğrudan görev tanımı olarak verin.

## PROMPT — Kobuki + Arduino + 3×HC-SR04 ile Bug0

**Rolün:** Otonom mobil robotlar, gömülü C/C++, sensör haberleşmesi ve diferansiyel sürüş kontrolü konusunda uzman bir robotik yazılım mühendisi olarak çalış.

**Görevin:** Kobuki mobil robot üzerinde, Arduino'ya bağlı **3 HC-SR04** ultrasonik sensör kullanarak **Bug0 engelden kaçınma** algoritmasını geliştir. Kod tabanını platform bağımsız, modüler ve test edilebilir tasarla. Öncelikle mimariyi, arayüz sözleşmelerini, durum makinesini ve test senaryolarını oluştur; ardından bileşenleri adım adım gerçek çalışan koda dönüştür.

### Donanım ve veri akışı

- Üç sensör **sol ön, ön, sağ ön** yönlerine bakacak. Gerçek montaj açılarını sabit varsayma; yapılandırılabilir parametre yap.
- Arduino, sensörleri sırayla tetikleyerek mesafe verisi üretsin. Echo zaman aşımı, veri geçerliliği, çapraz yankı önlemleri ve filtreleme uygula.
- Arduino'dan ana kontrol uygulamasına sürümü tanımlı, çerçevelenmiş, doğrulanabilir seri mesajlarla veri ilet; paket kaybı, eksik paket, yanlış veri ve güncellik kontrolü ekle.
- Kobuki sürücü arayüzü odometri/yönelim verisini versin ve `linear_velocity`, `angular_velocity` komutlarını kabul etsin. **Gerçek Kobuki protokolünü uydurma**; cihaz sürümü ve bağlantı türü belirlenince doğrulanmış dokümana göre adaptör yaz.
- Arduino ölçüm katmanı, Bug0 karar katmanı ve Kobuki sürüş arayüzünü birbirine bağımlı kılma.

### Bug0 davranış gereksinimleri

1. Robot hedefin konumunu ve kendi pozunu takip etsin; odometri veya eşdeğer lokalizasyon yoksa gerçek hedefe gitme davranışını çalışırmış gibi gösterme.
2. Başlangıçta sensörler, odometri ve haberleşme doğrulanana kadar motor komutları **sıfır** olsun.
3. `GO_TO_GOAL` durumunda hedef açısına yönelip güvenli hızda ilerlesin.
4. Hedef doğrultusunda tehlikeli yakınlıkta engel görünce yavaşlasın veya dursun; hareketi engelden kaçınmaya devretsin.
5. Engel takibi için uygun tarafı seçsin; seçimden sonra gereksiz sağ-sol geçişler yapmasın.
6. `FOLLOW_BOUNDARY` durumunda seçilen taraftaki engelle belirlenen açıklığı koruyarak düşük hızda hareket etsin; robot gövde boyutu ve dönüş zarfını hesaba katsın.
7. Hedef yönünün güvenli biçimde açık olduğu **birkaç geçerli ölçümle** teyit edildiğinde sınır takibini bıraksın. Bu davranış **Bug0** kuralına uygun olsun; Bug2'nin M-line şartını ekleme.
8. Üç HC-SR04'ün sınırlı görüş alanının tam engel sınırı izlemeyi ve boşluk doğrulamayı garanti etmediğini açıkça kabul et. Riskli durumda ilerleme yerine durma/yeniden değerlendirme davranışını seç.
9. Algılanan engel kaybolursa körlemesine ilerleme; hızı azalt, çevreyi değerlendir ve gerekiyorsa `RECOVERY` durumuna geç.
10. `GOAL_REACHED` durumunda robotu durdur ve görev tamamlanmasını bildir.
11. Döngü, çıkmaz, aşırı uzun sınır takibi, tekrarlanan durum geçişi, veri kaybı ve ölçüm hatası için sınırlı kurtarma stratejisi veya güvenli duruş uygula.
12. Bumper/cliff/wheel-drop gibi Kobuki güvenlik olayları Bug0 durumundan bağımsız olarak sürüş komutlarını iptal edebilsin.
13. Eşiklere histerezis ekle; sensör gürültüsü yüzünden hızlı durum değiştirme ve kararsız direksiyon davranışını engelle.
14. Komutların hız, ivme ve acil durma sınırlarını uygula. Güvenli durma mesafesi gecikme ve frenlemeyi içersin.

### Yazılım mimarisi

- Çekirdek algoritma **standart C++** ile yazılsın; ROS, belirli bir RTOS veya üretici SDK'sına bağımlı olmasın.
- Donanım bağımlı bölümler `ArduinoSensorAdapter`, `SerialTransport`, `KobukiAdapter`, `OdometryProvider` gibi arayüzlerin arkasında kalsın.
- `Bug0Planner` girişleri: zaman damgalı üç sensör ölçümü ve geçerlilikleri, robot pozu, hedef pozu, robot geometrisi, güvenlik/konfigürasyon parametreleri.
- `Bug0Planner` çıkışları: durum, seçili takip tarafı, güvenlik/arıza durumu ve hedef hareket referansı; son hız komutları ayrı `MotionController` tarafından sınırlandırılsın.
- Parametreler tek yapılandırma modülünde toplanmalı. Birimlerde SI sistemi kullanılmalı; `m`, `rad`, `m/s`, `rad/s`, `s` açıkça belirtilmeli.
- Hedef kontrol uygulamasının nerede çalıştığı (**Arduino / ayrı MCU / PC / SBC**), Kobuki'nin kesin sürümü, bağlantı arayüzü, sensör pinleri, odometri erişimi, hedef tanımı ve güvenlik gereksinimleri bilinmiyorsa **uydurma**; varsayımları listede belirt ve netleştirilmesi gereken soruları sor.

### Geliştirme sırası ve teslimatlar

1. Önce sistem sınırı, varsayımlar, veri akışı ve bileşen diyagramını yaz.
2. Ardından durum makinesi ve geçiş koşullarını tablo halinde tanımla.
3. Her sensör konumu için ölçüm ve karar sınırlarını (görüş alanı, kör bölge) belgele.
4. Arduino tarafında sıralı HC-SR04 okuma ve hata denetimli seri iletişim kodunu oluştur.
5. Ana uygulama için donanımdan bağımsız veri modellerini, `Bug0Planner` ve `SafetySupervisor` arayüzlerini tanımla.
6. Önce sahte sensör/odometri girdileriyle çalışan birim testleri ve simülasyon senaryolarını uygula.
7. Sonra gerçek Kobuki adaptörünü ve düşük hızlı donanım testlerini entegre et.
8. Durum değişimleri, ölçümler, hedef açısı, hız komutları, sensör zaman aşımı ve güvenlik olaylarını logla.
9. Test raporu oluştur: senaryo, beklenen sonuç, gerçekleşen sonuç, başarısızlık nedeni ve iyileştirme.

**Kabul kriterleri:** Robot engel yokken hedefe yönelmeli; öndeki engelde çarpışmadan durup uygun kenarı takip etmeyi denemeli; hedef hattı güvenilir biçimde serbest kalınca hedefe ilerlemeye dönmeli; veri kaybı ve belirsizliklerde güvenli şekilde durmalı; hedefe ulaştığında sıfır hız komutu üretmeli. **Üç ultrasonik sensörle her engel geometrisinde başarı veya çarpışmasızlık garantisi verme.**

**Çıktı biçimi:** Mimari açıklaması → gereksinimler → durum makinesi → arayüzler → modül/kaynak dosya yapısı → çalışan örnek kod → birim/integrasyon testleri → donanım test planı → bilinen kısıtlar. Adımları atlama; teknik kararları gerekçelendir.

---

## 8. Uygulamaya başlamadan önce netleştirilecek bilgiler

- Kobuki'nin **tam modeli/sürümü** ve hareket komutu arayüzü.
- Arduino modeli ve ana bilgisayar/denetleyici mimarisi.
- Sensörlerin gövdeye göre gerçek montaj konum ve yönleri.
- Hedef koordinatlarının nasıl verileceği ve Kobuki odometrisinin hangi arayüzden okunacağı.
- Robotun gerçek boyutları, azami hız ve frenleme karakteristikleri.
- Hedeflenen çalışma ortamı (iç mekân, düz zemin, cam/yumuşak/ince engeller, dinamik nesneler).

**Tasarım notu:** Üç ön yönlü HC-SR04 ile engel etrafında sürekli ve güvenilir sınır takibi özellikle zor olabilir. Testlerde bu eksiklik görülürse yana bakan ek mesafe sensörü ya da 2D LiDAR'a geçiş düşünülmelidir; algoritma çekirdeği donanımdan ayrıldığı için bu değişiklik yeniden yazım gerektirmemelidir.