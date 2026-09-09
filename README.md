# DRONE (CC) — Industrial Pro · Uçuş Nəzarət Sisteminin Texniki Sənədləri

Bu sənəd, Teensy 4.1 üzərində işləyən bərpa (recovery) dronunun uçuş nəzarət sisteminin **riyazi modelini və alqoritmlərini** tam təfərrüatı ilə izah edir. Sistem model raketin içərisinə yerləşdirilir; raket apogeyə çatdıqdan sonra burun açılır, dron xaricə çıxır və müəyyən bucaq altında yerə düşərkən mühərrikləri (əvvəlcə ESC1, 1 saniyə sonra ESC2) avtomatik işə salaraq enməni idarə edir.

---

## 1. Avadanlıq və Sensorlar

| Komponent | Model | İnterfeys | Məlumat |
|---|---|---|---|
| Mikrokontroller | Teensy 4.1 (IMXRT1062, 600 MHz) | — | — |
| IMU (9-DOF) | Adafruit BNO055 | I2C (0x28) | təcil (m/s²), giroskop (rad/s), maqnitometr (µT) |
| Barometr | Adafruit BME280 | I2C (0x76) | təzyiq (hPa), temperatur, rütubət |
| Temperatur/Rütubət | Adafruit AHT20 | I2C (0x38) | temperatur, nisbi rütubət |
| GPS | U-blox (NMEA) | Serial6 (9600) | mövqe, hündürlük, sürət, istiqamət, peyk |
| ESC | Standart PWM (50 Hz) | Pin 15, 23 | 1000–2000 µs impuls |
| RF Əlaqə | Serial2 (115200) | UART | binary telemetriya + komanda |

---

## 2. Sistem Memarlığı və Məlumat Axını

```
BNO055 (100 Hz) ──► AttitudeEKF ──► roll/pitch/yaw ──► meyil
                              │                          │
BME280 (25 Hz)  ──► AltVel (Kalman) ──► rel_alt, vel ──► FlightCtrl ──► ESC1/ESC2
                              ▲                          │
GPS (5 Hz)      ──────────────┘ (BME yoxdursa fallback)  └─► RF telemetriya + LED
```

Əsas dövrə (`loop`) kooperativ zamanlayıcıdır; hər alt sistem öz periodunda işləyir (bax Bölmə 10).

---

## 3. ESC və Slew-Rate Nəzarətçisi

### 3.1 PWM generasiyası
- Tezlik: **50 Hz** → period `T = 20 ms = 20000 µs`.
- Ayırdetmə: **16 bit** (`analogWriteResolution(16)`).

İmpuls eni (`us`) → iş dövrü (`duty`) çevirməsi:

```
duty(us) = round((us + 0.5) · 65536 / 20000)
```

`us` əvvəlcə `[ESC_US_MIN=1000, ESC_US_MAX=2000]` aralığına məhdudlaşdırılır.

### 3.2 Slew-rate (rampa) məhdudlaşdırıcı
Hədəf `target` dəyərinə saniyədə `ESC_SLEW_RATE_US_PER_S = 1500 µs` sürətlə yaxınlaşılır:

```
step = 1500 · dt
current = clamp(current ± step, hədəfə doğru)
```

Yəni 1000 → 1480 µs rampası təxminən `(1480−1000)/1500 ≈ 0.32 s` çəkir. Bu, ani qaz sıçrayışlarının və ESC desinxronizasiyasının qarşısını alır.

### 3.3 Mühərrik sırası
`FS_DESCENDING` vəziyyətində **ESC1 dərhal**, **ESC2 isə 1 saniyə sonra** `ESC_US_RUN=1480 µs` hədəfinə rampa edir.

---

## 4. Hündürlük / Şaquli Sürət Filtrasiyası (AltVel) — 2-Vəziyyətli Kalman

Vəziyyət vektoru:  **x = [rel_alt, vel]ᵀ**  (nisbi hündürlük, şaquli sürət)

### 4.1 Təzyiq kalibrasiyası (başlanğıc)
Boot-dan etibarən ilk **2.5 saniyə** (ən azı 15 nümunə) ərzində təzyiqin ortası alınır və referans `p₀` olur:

```
p₀ = (1/N) · Σ pressure_i
```

`rel_alt`, buraxılış rampasına görə nisbi hündürlükdür (rampada 0).

### 4.2 Aşağı tezlikli (hamarlaşdırma) filtrlər
```
g_smooth  += 0.15 · (g_raw − g_smooth),    g_raw = max(|a|/G, 0)
p_smooth  += 0.60 · (p − p_smooth)
dpdt_smooth += 0.50 · ((p_smooth − p_prev)/dt − dpdt_smooth)
```

### 4.3 Barometrik hündürlük düsturu
```
z = 44330 · ( 1 − (p_smooth / p₀)^0.1903 )        [metr]
```

### 4.4 Şaquli təcil (dünya oxunda)
```
a_vert = clamp(a_world_z − G, −50, +50)
a_smooth += 0.50 · (a_vert − a_smooth)
```
`a_world_z`, təcil vektorunun kvaternionla dünya oxuna döndərilmiş Z komponentidir.

### 4.5 Dinamik səs-küy modeli
Yüksək dinamikada (g 1-dən uzaqlaşdıqca) filtr barometrə daha az etibar edir:
```
dyn     = min(1 + 4·|g_raw − 1|, 10)
r_alt   = 0.3 · dyn      (ölçmə səs-küyü)
q_alt   = 0.05 · dyn     (hündürlük proses səs-küyü)
q_vel   = 0.6 · dyn      (sürət proses səs-küyü)
```

### 4.6 Proqnoz addımı (predict)
```
alt_p = rel_alt + vel·dt + 0.5·a_smooth·dt²
vel_p = vel + a_smooth·dt

P00_p = P00 + 2·dt·P01 + dt²·P11 + q_alt
P01_p = P01 + dt·P11
P11_p = P11 + q_vel
```

### 4.7 Yeniləmə addımı (update)
```
innov  = z − alt_p
S      = P00_p + r_alt
K0     = P00_p / S
K1     = P01_p / S

rel_alt = alt_p + K0 · innov
vel     = vel_p + K1 · innov

P00 = (1 − K0) · P00_p
P01 = (1 − K0) · P01_p
P11 = P11_p − K1 · P01_p
```

### 4.8 GPS hündürlük fallback (BME280 yoxdursa)
İlk etibarlı fix-də baza hündürlüyü `base` alınır; sonra:
```
raw_rel = alt_gps − base
rel_alt = rel_alt + 0.15 · (raw_rel − rel_alt)      // ağır aşağı-tezlikli filtr
vel     = 0.3 · vel + 0.7 · (Δrel_alt / dt)
```

---

## 5. Meyil (Attitude) Təxmini (AttitudeEKF) — 7-Vəziyyətli Kvaternion EKF

Vəziyyət vektoru:  **x = [q₀, q₁, q₂, q₃, bₓ, b_y, b_z]ᵀ**

`q = (w, x, y, z)` vahid kvaternion, `b` giroskop sapması (bias).

### 5.1 Kvaternion cəbri
- Vektor döndərmə: `v' = q ⊗ v ⊗ q*`
- Tərs kvaternion (vahid üçün qoşma): `q* = (w, −x, −y, −z)`
- Normallaşdırma: `q = q / ‖q‖`

### 5.2 Başlanğıc (init)
Cazibə vektorundan başlanğıc kvaternionu hesablanır (təcil [ax,ay,az] normallaşdırılır, ox-bucaq üsulu ilə). Başlanğıc kovariasiyası `P = 0.1 · I₇`.

### 5.3 Proqnoz addımı (predict) — giroskop inteqrasiyası
```
ω = g − b                       (bias düzəldilmiş giroskop)
q̇ = 0.5 · ω ⊗ q
q  = q + q̇ · dt ;  normalize
```

Vəziyyət keçid (Yakobi) matrisi `F` (7×7):
```
F[0][0..3] = [1, −0.5ωₓdt, −0.5ωydt, −0.5ωzdt]
F[1][0..3] = [ 0.5ωₓdt, 1,  0.5ωzdt, −0.5ωydt]
F[2][0..3] = [ 0.5ωydt, −0.5ωzdt, 1,  0.5ωₓdt]
F[3][0..3] = [ 0.5ωzdt,  0.5ωydt, −0.5ωₓdt, 1]
F[0..3][4..6] = 0.5 · q[1..3] · dt     (d q̇ / d b)
F[4..6][4..6] = I₃
```

Proses səs-küyü `Q`:
- Kvaternion hissəsi: `Q[i][j] = 0.25·(0.02)²·dt² · Σₖ Xi[i][k]·Xi[j][k]`  (Xi = kvaternion törəmə matrisi)
- Bias hissəsi: `Q[4..6][4..6] = (0.0005)² · dt`

Kovariasiya yayılması:  `P = F · P · Fᵀ + Q`

### 5.4 Akselerometr yeniləməsi (roll/pitch düzəlişi)
```
amag = ‖a‖ ; 3 < amag < 25 m/s² deyilsə rədd et
meas = a / amag
ref  = [0, 0, 1]                    (dünya oxunda cazibə)
dev  = |amag − G| / G
r    = 0.003 + 2·dev²               (uyğunlaşan ölçmə səs-küyü)
```

### 5.5 Maqnitometr yeniləməsi (yaw düzəlişi)
```
mmag = ‖m‖ ; 15 < mmag < 120 µT deyilsə rədd et
meas = m / mmag
m_world = q ⊗ meas ⊗ q*             (dünyaya döndər)
norm = √(m_world.x² + m_world.y²) ; norm < 0.2 isə rədd et
ref_raw = [m_world.x/norm, m_world.y/norm, 0]   (üfüqi şimal referansı)
_mag_ref += 0.05·(ref_raw − _mag_ref) ; normalize
r = 0.02 + 0.5·(|mmag − ref|/ref)²
```

### 5.6 Ümumi vektor ölçmə yeniləməsi (updateVectorMeasurement)
```
h    = q* ⊗ ref ⊗ q                (referansı gövdə oxuna döndər)
innov = meas − h
H    = ∂h/∂q  (3×7 Yakobi, ref-ə görə)
S    = H·P·Hᵀ + r·I₃
K    = P·Hᵀ·S⁻¹
q   += K[0..3]·innov ; normalize
b   += K[4..6]·innov
P    = (I − K·H)·P ; simmetrikləşdir
```

### 5.7 Eyler bucaqları (dərəcə)
```
roll  = atan2( 2(q₀q₁ + q₂q₃), 1 − 2(q₁² + q₂²) ) · 180/π
pitch = asin( clamp(2(q₀q₂ − q₃q₁), −1, 1) ) · 180/π
yaw   = atan2( 2(q₀q₃ + q₁q₂), 1 − 2(q₂² + q₃²) ) · 180/π
```

### 5.8 Kovariasiya düzəlişi (symmetrize)
- Diaqonal: `P[i][i]` → `[1e-12, 10]` aralığına məhdudlaşdır, NaN/Inf qorunması.
- Diaqonaldan kənar: `P[i][j] = P[j][i] = (P[i][j]+P[j][i])/2`.


---

## 6. Uçuş Vəziyyət Maşını (FlightCtrl)

Vəziyyətlər:
```
FS_STANDBY(0) → FS_LAUNCHED(1) → FS_DESCENDING(2) → FS_LANDED(3)
```

### 6.1 Keçid şərtləri (real uçuş, apogey ~4 km)

| Keçid | Şərt |
|---|---|
| STANDBY → LAUNCHED | `rel_alt > 5.0 m && vel > 2.0 m/s` (raket yüksəlir) |
| LAUNCHED → DESCENDING | `vel < −1.0 m/s && (max_alt − rel_alt > 2.0 m)` (apogeydən düşmə) |
| DESCENDING → LANDED | `rel_alt ≤ 1.0 m` və ya `\|vel\| < 0.3 m/s` 2 s ərzində |

`max_alt`, LAUNCHED boyu yenilənən zirvə (apogey) hündürlüyüdür.

### 6.2 Meyil (tilt) histerezisi
```
tilt = max(|roll|, |pitch|)
tilt > 60°  → mühərriklər bloklanır (tilt_hysteresis_ok = false)
tilt < 45°  → mühərriklər sərbəstdir (tilt_hysteresis_ok = true)
45°–60° arası → əvvəlki vəziyyət saxlanılır (histerezis)
```
Dron bucaq altında düşdüyü üçün 60°-yə qədər mühərrik işləyir; həddindən artıq fırlanmada təhlükəsizlik üçün gözləyir.

### 6.3 Mühərrik nəzarəti
```
FS_DESCENDING && tilt_ok:
    target_esc1 = 1480 µs  (dərhal)
    target_esc2 = 1480 µs  (ESC1-dən 1 s sonra)
digər vəziyyətlər:
    target_esc1 = target_esc2 = 1000 µs (söndürülü)
```

`FS_LANDED` terminaldır (avtomatik STANDBY-yə dönmür). Yeni uçuş üçün operator **DISARM → ARM** edir.

---

## 7. g-Qüvvəsi və Sərbəst Düşmə İzlənməsi

```
raw_g  = ‖a‖ / G
fast_g = fast_g + 0.45 · (raw_g − fast_g)   // aşağı-tezlikli filtr, α=0.45
fast_g_valid = true  (IMU məlumatı sonludursa)
```

- `fast_g ≈ 1.0` → sükunət (yerdə və ya sabit sürətdə).
- `fast_g < 1.0` → sərbəst düşmə / azalan təcil.
- `fast_g > 1.0` → raket itələmə fazası (yüksək g).

Bu siqnal, `AltVel`-in dinamik səs-küy əmsalını (`dyn_factor`) və maqnitometr düzəlişinin qapısını (`0.7 < fast_g < 1.8`) qidalandırır.

---

## 8. RF Əlaqə Protokolu

### 8.1 Paket strukturu (binary, little-endian)
```
[SYNC1 0xAA][SYNC2 0x55][VER 0x01][DEVID 0xCC][TYPE][SEQ u16][MILLIS u32][LEN u8][PAYLOAD][CRC16 u16]
```

### 8.2 CRC16-CCITT
```
poly = 0x1021,  init = 0xFFFF
crc ^= data[i] << 8;  8 dəfə:  crc = (crc & 0x8000) ? (crc<<1)^0x1021 : (crc<<1)
```

### 8.3 Komanda (ARM/DISARM) çərçivəsi
```
[0xAA][0x55][VER 0x01][DEVID 0xCC][CMD][CRC16(2 bayt)]
CMD: 0x01 = ARM, 0x00 = DISARM
CRC16, [VER, DEVID, CMD] 3 baytı üzərindən hesablanır.
```

### 8.4 Telemetriya payload-u (76 bayt, `RF_PKT_TELEM`)

| Bayt (0-bazlı) | Sahə | Tip | Miqyas |
|---|---|---|---|
| 0–1 | flags | u16 | bit bayraqları |
| 2–19 | ax,ay,az,gx,gy,gz,mx,my,mz | 9×i16 | ×100 / ×1000 / ×10 |
| 20–29 | bme_t, bme_p, bme_h, bme_a | i16,u16,u16,i32 | |
| 30–33 | aht_t, aht_h | i16,u16 | ×100 |
| 34–53 | lat_e7, lon_e7, alt_cm, speed, course, sats, fix_quality, hdop | 20 B | |
| 54–59 | roll, pitch, yaw | 3×i16 | ×100 |
| 60–69 | rel_alt, vel, g_force, dpdt | i32,i16,u16,i16 | Z×100(cm), Vz×100 |
| 70 | armed | u8 | 0/1 |
| 71 | state_code | u8 | 0..3 |
| 72–73 | ESC1 (µs) | u16 | 1000–2000 |
| 74–75 | ESC2 (µs) | u16 | 1000–2000 |

Flags bitləri: `FLAG_BNO_OK=0x0001, FLAG_BME_OK=0x0002, FLAG_AHT_OK=0x0004, FLAG_GPS_FIX=0x0008, FLAG_ARMED=0x0010, FLAG_DESCENDING=0x0020, FLAG_CAL_SAVED=0x0040`.

Vəziyyət dəyişəndə əlavə olaraq `RF_PKT_STATUS` paketi göndərilir: `[armed u8, state_code u8]`.

---

## 9. BNO055 Kalibrasiyası və EEPROM

EEPROM yerləşimi (ünvan `BNO_CAL_EEPROM_ADDR = 0`):
```
[magic u16 = 0xB0C0][version u8 = 1][checksum u8][offsets struct]
```

`checksum8` = bütün offset baytlarının mod-256 cəmi.

Kalibrasiya saxlama şərti (1 Hz yoxlama):
```
sys == 3 && gyro ≥ 2 && accel ≥ 2 && mag ≥ 2  →  3 s sabit qalsa saxla
```

---

## 10. Zamanlama / Planlaşdırma

| Alt sistem | Period | Tezlik |
|---|---|---|
| BNO055 oxuma + AttitudeEKF | 10 ms | 100 Hz |
| Uçuş nəzarəti (FlightCtrl) | 10 ms | 100 Hz |
| BME280 + AltVel | 40 ms | 25 Hz |
| Maqnitometr düzəlişi | 50 ms | 20 Hz |
| GPS parse | 200 ms | 5 Hz |
| USB debug yazdırma | 200 ms | 5 Hz |
| RF telemetriya | 66 ms | ~15 Hz |
| AHT20 | 1000 ms | 1 Hz |
| BNO kalibrasiya yoxlaması | 1000 ms | 1 Hz |

`dt` məhdudiyyətləri: IMU `[1 ms, 50 ms]`, AltVel `[1 ms, 100 ms]` (ani kilidlənmələrdə partlamanın qarşısını alır).

---

## 11. Əsas Sabitlər Xülasəsi

| Sabit | Dəyər | İzah |
|---|---|---|
| `GRAVITY` | 9.80665 m/s² | standart cazibə |
| `SEA_LEVEL_HPA` | 1013.25 hPa | dəniz səviyyəsi təzyiqi |
| `ESC_PWM_FREQ` | 50 Hz | ESC impuls tezliyi |
| `ESC_US_MIN/MAX` | 1000 / 2000 µs | impuls aralığı |
| `ESC_US_OFF/RUN` | 1000 / 1480 µs | mühərrik söndürülü/işlək |
| `ESC_SLEW_RATE_US_PER_S` | 1500 µs/s | rampa sürəti |
| `I2C_FREQ` | 400 kHz | I2C sürəti |
| `GPS_AGE_MAX_MS` | 3000 ms | GPS məlumat təzəlik həddi |
| `meyil histerezisi` | 60°/45° | mühərrik bucaq qapısı |
| Buraxılış həddi | 5 m & 2 m/s | |
| Düşmə həddi | −1 m/s & 2 m | |
| Enmə həddi | 1 m | |

---

*Bu sənəd `src/main.cpp` mənbə kodu ilə birebir uyğundur; dəyərlər kodda dəyişərsə, buradakı cədvəllər də yenilənməlidir.*
