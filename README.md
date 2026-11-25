# M3 — Control Firmware (Ultrasonik)

[![M3 tests](https://github.com/ruyakocak/Ping-Pong-Ball-Levitation-System/actions/workflows/ci.yml/badge.svg)](https://github.com/ruyakocak/Ping-Pong-Ball-Levitation-System/actions/workflows/ci.yml)

Kontrol kartı (ESP32) firmware'i + M3'ü diğer modüllerden bağımsız test eden simülatörler.
Dokümandaki karşılığı: Bölüm 6, gereksinimler R3.1–R3.13.

## Kurulum ve tek komut (Windows / Linux)

```
python -m pip install -r requirements.txt
python tools/run_all.py
```

Windows'ta native testler için `gcc` (MinGW-w64) PATH'te olmalı.
İlk çalıştırma ESP32 araç zincirini indirir (birkaç yüz MB); atlamak için `--no-firmware`.
Windows'ta proje yolu çok uzunsa (≈150+ karakter) Unity kurulumu "dosya adı çok uzun" ile düşer:
projeyi kısa bir klasöre koyun ya da `PLATFORMIO_WORKSPACE_DIR` ile `.pio`'yu kısa bir yola alın.
Rapor: `reports/report_<os>_<zaman>.md` — her gereksinim, testleri, PASS/FAIL, henüz kapsanmayanlar,
sanal kartla HIL sonuçları.

Tek komutun adımları (T3.13): firmware derlemesi (esp32, esp32_simball) → native C testleri
(kapalı çevrim simülasyonu dahil) → Python testleri (HIL simülatörü sanal karta karşı) → rapor.
CI: `.github/workflows/ci.yml`, aynı komutu `windows-latest` ve `ubuntu-latest`'te çalıştırır,
raporu job özetine ve artifact'e koyar.

HIL simülatörünü kartsız çalıştırmak (Windows / Linux):

```
gcc -O2 -std=gnu11 -Ilib/contract -Ilib/m3_core tools/vboard.c lib/contract/*.c lib/m3_core/*.c -lm -o vboard
python sim/hil_sim.py --virtual ./vboard --scenario hold
python sim/hil_sim.py --virtual ./vboard --scenario step --plot
```

Senaryolar: `hold`, `step` (30 → 50 cm, T3.8), `targets` (M4 dalgası, ağ gecikmesi, T3.7),
`commands` (canlı komutlar, T3.10), arızalar `camera_loss`, `bridged_only`, `m1_fault`, `estop`,
`fan_state_lost` (T3.5, T3.6, T3.11; arıza 10. s'de, nedeni 11. s'de kalkar, 12. s'de Reset/Start).
`--plot` canlı grafik açar (gerçek zamanlı); kaydedilmiş bir koşu: `python sim/live_plot.py reports/hil_<...>.csv`.

Gerçek kartla aynı komutlar `--virtual` olmadan çalışır: seri portlar ve kartın IP'si
`sim/hil_config.json`'da; simülatör Reset/Start'ı UDP ile gönderir, ack ve status'u 5010'da dinler.

## Demo — 4 Aralık (kartsız, yaklaşık 10 dk)

Önce bir kez: `python -m pip install -r requirements.txt` ve sanal kartı derle (yukarıdaki gcc satırı).
Hocaya baştan söyleyin: kart gelmediği için kontrol kodu, firmware'in aynı `lib/` kodunu çalıştıran
sanal kartta; HIL simülatörü kartla konuşacağı mesajların aynısını kullanıyor.

1. **Yapı (1 dk)** — bu README'nin Yapı tablosu: `lib/` donanımsız; aynı kod ESP32'ye ve sanal karta derleniyor;
   M1, M2, M4, M5 yerine kendi stand-in'lerimiz.
2. **CI (1 dk)** — GitHub Actions: Windows ve Linux yeşil, rapor job özetinde (R3.13).
3. **Tek komut (2 dk)** — `python tools/run_all.py`, sonra `reports/` altındaki raporu aç: gereksinim tablosu,
   HIL tablosu. R3.3 ve R3.12'yi kendiniz gösterin (kart / düzenek gerekiyor).
4. **Canlı grafik (4 dk)**, her biri ~15–30 s:
   ```
   python sim/hil_sim.py --virtual ./vboard --scenario step --plot
   python sim/hil_sim.py --virtual ./vboard --scenario targets --plot --duration 30 --settle 10
   python sim/hil_sim.py --virtual ./vboard --scenario m1_fault --plot --duration 16
   python sim/hil_sim.py --virtual ./vboard --scenario bridged_only --plot --duration 16
   ```
   Gösterilecek: basamakta aşım < 5 cm, 4 s içinde oturma; dalgada 150 ms gecikmeye rağmen kartın hedefi
   (mavi) gönderilenin üstünde; arızada kırmızı bant (Fault), fan sıfır, Reset gelene kadar Fault, sonra yeniden kalkış.
5. **Testler gerçekten yakalıyor mu (1 dk)** — bir güvenlik kuralını bozun (ör. `m3_cmd_check.c`'de
   `abs_max_rpm`'i yazılabilir yapın ya da `M3_HG_TIMEOUT_MS`'i 2000 yapın), `run_all` FAIL verir, geri alın.
6. **Eksikler (1 dk)** — kartta çalıştırılmadı; R3.3 ölçüm altyapısı hazır (`status.diag`); JSON alan adları ve
   payload düzenleri M5 sözleşmesini bekliyor; game board linki yok.

Canlı grafik açılmazsa: senaryoyu `--plot` olmadan çalıştırın, sonra `python sim/live_plot.py <csv>`.

## Kontrol kararları (25.11)

- **M2 stand-in'i filtreli yükseklik gönderir** (`tools/m2_standin.h`, `M2Filter`): M2 sözleşmede ham + filtreli
  yükseklik gönderiyor; eskiden stand-in filtreli alana ham değeri koyuyordu. Filtre bilerek M2'nin kendi geçme
  şartlarından zayıf (R2.5: 0,62 > 0,5; R2.4: 6,5 > 5 mm RMS; R2.10 gecikme 6 ms), `sim/tests/test_m2_standin.py`.
- **Kazançlar kp 1,8 · ki 0,8 · kd 0,5** (ekip kararı, orta seçim): R3.9 en kötü salınım 21 mm, T3.8 aşım 40 mm.
  kd 0,7 filtreliyle biraz daha iyiydi ama yalnız ham yükseklikte 120 mm salınım veriyordu.
- **Hedef rampası 250 mm/s** (`target_rate_mm_s`): bir basamak komutu rampaya dönüşür; fırlatmadan sonra rampa
  topun yüksekliğinden başlar. M4 dalgaları (≤ 190 mm/s) etkilenmez.
- **Bilinen sınırlar:** filtreli yükseklik yokken (yalnız ham, 5 mm gürültü) top tutulur ama salınım 60–90 mm'ye
  çıkar (R3.6 testi bunu raporlar; şart değil). Fırlatmadan sonra 300 mm hedefte top ~450 mm'ye çıkıp döner
  (500 mm'de aşım ~20 mm); fırlatma itmesi ±%30 modelde kalkış için gerekli.

## Ağ mesajları (UDP 5005, JSON)

Dokümanda yalnız içerik var, JSON alan adları yok: **adlar geçici**, M5 sözleşmesi donunca
değişecek. Hepsi tek yerde, `lib/m3_core/m3_net.h`'nin başında örnekleriyle.

- giriş: `command` (M5) → kontrol + `ack`; `target_points` (M4) → hedef tamponu
- çıkış: her komuta `ack` (gönderene), 30 Hz `status` (gateway 5010 ve M4 5006; varsayılan broadcast)
- aynı `request_id` tekrar gelirse uygulanmaz, aynı ack yeniden gönderilir
- JSON çözme ağ görevinde (core 0); kontrol görevine hazır yapı geçer (R3.3)
- `status.diag`: `lat_max_us` (height alındı → fan_cmd yazıldı, en kötü), `lat_over_5ms`,
  `heights_lost`, `net_bad` — R3.3 ölçümü bunlarla yapılacak

Wi-Fi adı/şifresi ve adresler: `include/m3_config.h`; şifreyi `include/wifi_secrets.h`'ye yazın
(git'e girmez):

```c
#define M3_WIFI_SSID "rig-hotspot"
#define M3_WIFI_PASS "..."
```

## Yapı

| Yol | Ne | Durum |
|---|---|---|
| `lib/contract/` | Seri çerçeve formatı, CRC16, payload'lar | Çerçeve ✔ · payload düzenleri **geçici** (M5 header'ı gelince değişecek) |
| `lib/m3_core/m3_pid.*` | Dış PID (§6.3 formülleri) | ✔ R3.1 |
| `lib/m3_core/m3_fsm.*` | Durum makinesi + reset sırası | ✔ R3.2 |
| `lib/m3_core/m3_height_gate.*` | filtered/raw seçimi, bridged ≤ 5, 200 ms güvenlik sayacı | ✔ R3.5, R3.6 |
| `lib/m3_core/m3_ctrl.*` | Hepsini birleştiren çekirdek (on_height / on_fan_state / on_tick) | ✔ R3.4, R3.8 (üç yükseklik + basamak); hedef rampası |
| `lib/m3_core/m3_stuck.*` | Sıkışmış top uyarısı (yalnız uyarı, fault değil). T3.11b'deki 3,5 s, tüm koşulların oluştuğu (hız ≥ %90 max) andan sayılır — ekip kararı 25.11 | ✔ R3.11b/c |
| `lib/m3_core/m3_plant.*` | Ders kitabı top + fan modeli (simball, native sim) | ✔ |
| `lib/m3_core/m3_target_buffer.*` | Zaman damgalı hedefler, saat ofseti (10 s'lik min), interpolasyon, yeni oturumda yumuşak geçiş | ✔ R3.7 (native + sanal kartla HIL) |
| `lib/m3_core/m3_cmd_check.*` | Parametre tablosu, aralıklar; güvenlik limitleri salt okunur | ✔ R3.10 |
| `lib/m3_core/m3_command.*` | command → ack (gerekçeli), mod değişimi, game_out yalnız game modunda | ✔ R3.10 |
| `lib/m3_core/m3_net.*` | UDP JSON: command/target_points çözme (sınırlı, bozuk girdiye dayanıklı), ack ve status yazma, tekrar eden istek | ✔ R3.7, R3.10 |
| `src/main.cpp` | ESP32: core 1 kontrol görevi, core 0 rx + net görevi (Wi-Fi, UDP), UART'lar, simball, R3.3 gecikme ölçümü | derleniyor (uyarısız, flash %57); karta henüz yüklenmedi · game board linki yok |
| `include/m3_config.h` | Wi-Fi, portlar, status hedefleri | ✔ |
| `tools/tune_gains.c` | Kazanç araması: R3.9 (27 model köşesi × 3 yükseklik, gürültü 5 mm, %5 düşen kare) ve T3.8 basamağı birlikte; en kötü durumu en iyi olan kazançlar | ✔ R3.9, T3.8 |
| `tools/m2_standin.h` | M2 stand-in filtresi (testler ve kazanç araması için; firmware değil) | ✔ |
| `sim/hil_sim.py` | HIL: M1 + M2'yi karta karşı oynar; seri port (gerçek kart) ya da `--virtual` (sanal kart). M5 (Reset/Start, canlı komutlar) ve M4 (target_points, 150 ms'ye kadar gecikme) yerine de geçer. Senaryolar: hold, step, targets, commands + 5 arıza (Reset/Start ile toparlanma) | ✔ sanal kartla · gerçek kartta denenmedi |
| `sim/live_plot.py` | Canlı grafik (`--plot`) ve kaydedilmiş CSV'nin çizimi; testler kullanmaz | ✔ demo |
| `tools/vboard.c` | Sanal kart: firmware'in aynı m3_core + contract + net kodu, main.cpp gibi bağlanmış; seri linkler, UDP ve saat stdin/stdout'tan, 1 ms adımlı | ✔ R3.13 |
| `sim/m3sim/` | Python contract aynası, plant modeli | ✔ |
| `contract/test_vectors.json` | C ve Python'un aynı byte'ları ürettiği vektörler | ✔ |

Kural: `lib/` altındaki hiçbir dosya donanım çağrısı yapmaz; zaman hep parametre olarak gelir.

## Sıradaki işler

**Firmware tarafı**
1. Karta yükle: önce `pio run -e esp32_simball -t upload`; Wi-Fi'ye bağlanınca status 5010'a gelir
2. Gerçek kartla HIL: `python sim/hil_sim.py --scenario hold` (port adları ve `board_ip` config'de)
3. R3.3: 10 dk HIL + Wi-Fi trafiği (status 30 Hz + target_points 30 Hz + ek yük), `status.diag.lat_max_us` ≤ 5000 ve `heights_lost` = 0
4. Game board linki (Serial1): game_in 30 Hz, game_out → `m3_command_game_out`
5. `fan_config` gönderimi (M1 hız döngüsü ayarları değişince)

**PC tarafı**
1. HIL'de eksik arızalar: frame tekrarı, M2 yeni session; 10 bridged'den yalnız 5'inin kullanılması (native'de var)
2. R3.9 köşelerini HIL senaryosu yap (native'de ve kazanç aramasında var)
3. M5 sözleşmesi gelince JSON alan adlarını ve binary payload düzenlerini ona uydur
4. Demo provası: yukarıdaki akışı baştan sona bir kez çalıştırın

## Donanım (HIL)

ESP32 DevKit + 2 adet 3.3 V USB-UART adaptörü:
- Kart USB'si → bilgisayar (height)
- Adaptör 1 ↔ GPIO16 (RX) / GPIO17 (TX) → M1 hattı
- Adaptör 2 ↔ GPIO26 (RX) / GPIO27 (TX) → game board hattı
- GND'ler ortak. Port adları `sim/hil_config.json`'da.
