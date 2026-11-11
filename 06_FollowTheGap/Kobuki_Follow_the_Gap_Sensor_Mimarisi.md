# Kobuki Robotlarında Mesafe Sensörleri ve Follow the Gap Uygulaması

## 1. Genel Bakış

Kobuki mobil robot platformu, çevreyi algılamak ve güvenli hareketi desteklemek amacıyla çeşitli yerleşik sensörlere sahiptir. Ancak **standart Kobuki donanımında, robotun önündeki engellerin açıya bağlı uzaklığını ölçen bir 2D LiDAR veya ileriye dönük ultrasonik mesafe sensörü bulunmaz.** Bu tür sensörler, kullanılan robot konfigürasyonuna bağlı olarak haricen eklenebilir.

**Follow the Gap (Boşluk Takibi)** algoritmasının uygulanmasında kritik gereksinim, robotun çevresindeki engellerin mesafelerini birden fazla yönde algılayabilmektir.

## 2. Kobuki Platformundaki Sensörler

| Sensör | Standart Kobuki'de | Temel işlevi | Follow the Gap açısından |
|---|---|---|---|
| Bumper (tampon) | Var | Engele fiziksel temas algılama | Güvenlik/durdurma; mesafe verisi sağlamaz |
| Cliff Sensor (IR) | Var | Zemindeki düşme tehlikesini algılama | İleriye dönük engel taramasının yerini tutmaz |
| Wheel Encoder | Var | Tekerlek dönüşleri üzerinden odometri | Hareket ve konum tahminini destekler |
| Gyroscope | Var | Açısal hareketi ölçme | Yönelim/hareket kontrolünü destekler |
| Ultrasonic Sensor | Harici/konfigürasyona bağlı | Engellere olan uzaklığı ölçme | Çok yönlü tarama sağlanırsa kullanılabilir |
| 2D LiDAR | Harici/konfigürasyona bağlı | Farklı açılarda engel mesafeleri ölçme | Follow the Gap için uygun veri sağlar |

> **Not:** Kobuki tabanı ile üzerine kurulmuş TurtleBot veya laboratuvar robot konfigürasyonları aynı sensör donanımına sahip olmak zorunda değildir. Gerçek cihazın üzerinde hangi sensörlerin bulunduğu ayrıca doğrulanmalıdır.

## 3. Mesafe Ölçümü Nasıl Yapılır?

### 3.1 Ultrasonik sensör

Ultrasonik sensör ses darbesi gönderir, engelden yansıyan sinyalin dönüş süresini ölçer ve mesafeyi hesaplar:

\[
 d = \frac{v_{\text{ses}}\,t}{2}
\]

- **d:** Engele olan uzaklık
- **v_ses:** Sesin ortamdaki hızı
- **t:** Ses darbesinin gidiş-dönüş süresi

Tek bir sabit ultrasonik sensör dar bir görüş alanını ölçer. Follow the Gap için farklı yönlerde mesafe bilgisi elde etmek amacıyla sensörün döndürülmesi veya birden fazla sensör kullanılması gerekebilir.

### 3.2 2D LiDAR

2D LiDAR, bir tarama düzleminde birden fazla açı için mesafe ölçümleri üretir. Veriler genel olarak **(açı, mesafe)** çiftleri halinde işlenebilir. Böylece engel sınırları ve aralarındaki geçilebilir açıklıklar belirlenebilir.

## 4. Follow the Gap Algoritmasına Entegrasyon

Önerilen, platform bağımsız veri akışı:

```text
Ultrasonic Sensor / 2D LiDAR
           |
           v
     Sensor Interface
  (okuma, filtreleme,
     normalizasyon)
           |
           v
   Follow the Gap
 (boşlukları algılama,
 uygun boşluğu seçme)
           |
           v
    Motion Controller
   (doğrusal hız v,
     açısal hız ω)
           |
           v
     Kobuki Mobile Base
   (diferansiyel sürüş)
```

### 4.1 Temel işlem adımları

1. Sensörden güncel açı–mesafe verilerini oku.
2. Hatalı/gürültülü ölçümleri filtrele.
3. Robot genişliği ve güvenlik payı dikkate alınarak geçilebilir boşlukları belirle.
4. Hedef doğrultusu ve güvenli açıklık gibi ölçütlere göre uygun boşluğu seç.
5. Seçilen yöne göre doğrusal ve açısal hız komutlarını üret.
6. Sensör verisi yenilendikçe karar sürecini tekrarla.
7. Geçerli ve güvenli bir geçiş bulunamazsa robotu güvenli şekilde durdur.

## 5. Yerleşik Sensörlerle Güvenlik İzleme

Kobuki'nin yerleşik **bumper**, **cliff** ve mevcutsa **wheel-drop** göstergeleri, Follow the Gap algoritmasından ayrı bir **Safety Monitoring (Güvenlik İzleme)** katmanında değerlendirilebilir. Bu katman, tehlikeli durumlarda hareket komutlarının engellenmesi veya robotun durdurulması için kullanılır.

Bu yerleşik sensörler, LiDAR'ın sağladığı açısal mesafe taramasının eşdeğeri değildir.

## 6. Proje İçin Sensör Seçimi

| Ölçüt | Ultrasonik sensör | 2D LiDAR |
|---|---|---|
| Tek ölçümde kapsanan açılar | Sınırlı | Geniş açısal tarama |
| Boşluk algılama | Çoklu sensör veya döner mekanizma gerekebilir | Doğrudan açısal mesafe profiliyle yapılabilir |
| Entegrasyon | Sensör ve denetleyiciye bağlı | Sensör sürücüsü/veri arayüzüne bağlı |
| Follow the Gap uygunluğu | Uygun bir tarama düzeniyle mümkün | Genellikle daha elverişli |

**Öneri:** Algoritmanın geliştirilmesi ve testinde **2D LiDAR** tercih edilmesi, boşlukların tespiti ve yön seçimini kolaylaştırır. Ultrasonik sensör kullanılacaksa açısal kapsama ve tarama hızı dikkatle tasarlanmalıdır.

## 7. Sonuç

Kobuki robotunda standart olarak engel teması, zemin güvenliği ve hareket kestirimi için sensörler bulunur. Bununla birlikte **Follow the Gap algoritmasının ihtiyaç duyduğu çok yönlü mesafe verilerini sağlamak amacıyla genellikle ek bir LiDAR veya uygun ultrasonik tarama düzeneği gerekir.** Uygulama, sensör arayüzü, boşluk tespiti, hareket kontrolü ve güvenlik izlemesini ayrı bileşenler olarak tasarlayarak platformdan bağımsız tutulabilir.

### Uygulamaya Başlamadan Önce Doğrulanacak Husus

Kullanılacak Kobuki robotunda standart sensörlere ek olarak **LiDAR, ultrasonik sensör veya derinlik kamerası** bulunup bulunmadığı kontrol edilmelidir. Bu seçim, sensör arayüzü ve algoritmanın uygulanma yöntemini belirler.
