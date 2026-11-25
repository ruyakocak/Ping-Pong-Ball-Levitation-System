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
python sim/hil_sim.py --virtual ./vboard --scenario camera_loss
python sim/hil_sim.py --virtual ./vboard --scenario targets
python sim/hil_sim.py --virtual ./vboard --scenario commands
```

Gerçek kartla aynı komutlar `--virtual` olmadan çalışır: seri portlar ve kartın IP'si
`sim/hil_config.json`'da; simülatör Reset/Start'ı UDP ile gönderir, ack ve status'u 5010'da dinler.

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
| `lib/m3_core/m3_ctrl.*` | Hepsini birleştiren çekirdek (on_height / on_fan_state / on_tick) | ✔ temel; R3.4, R3.8 (ideal model) |
| `lib/m3_core/m3_stuck.*` | Sıkışmış top uyarısı (yalnız uyarı, fault değil). T3.11b'deki 3,5 s, tüm koşulların oluştuğu (hız ≥ %90 max) andan sayılır — ekip kararı 25.11 | ✔ R3.11b/c |
| `lib/m3_core/m3_plant.*` | Ders kitabı top + fan modeli (simball, native sim) | ✔ |
| `lib/m3_core/m3_target_buffer.*` | Zaman damgalı hedefler, saat ofseti (10 s'lik min), interpolasyon, yeni oturumda yumuşak geçiş | ✔ R3.7 (native + sanal kartla HIL) |
| `lib/m3_core/m3_cmd_check.*` | Parametre tablosu, aralıklar; güvenlik limitleri salt okunur | ✔ R3.10 |
| `lib/m3_core/m3_command.*` | command → ack (gerekçeli), mod değişimi, game_out yalnız game modunda | ✔ R3.10 |
| `lib/m3_core/m3_net.*` | UDP JSON: command/target_points çözme (sınırlı, bozuk girdiye dayanıklı), ack ve status yazma, tekrar eden istek | ✔ R3.7, R3.10 |
| `src/main.cpp` | ESP32: core 1 kontrol görevi, core 0 rx + net görevi (Wi-Fi, UDP), UART'lar, simball, R3.3 gecikme ölçümü | derleniyor (uyarısız, flash %57); karta henüz yüklenmedi · game board linki yok |
| `include/m3_config.h` | Wi-Fi, portlar, status hedefleri | ✔ |
| `tools/tune_gains.c` | Kazanç araması: 27 model köşesi (k_air, c_drag, tau ×0,7/1/1,3) × 3 yükseklik, gürültü 5 mm, %5 düşen kare; en kötü durumu en iyi olan kazançlar | ✔ R3.9 |
| `sim/hil_sim.py` | HIL: M1 + M2'yi karta karşı oynar; seri port (gerçek kart) ya da `--virtual` (sanal kart). M5 (Reset/Start, canlı komutlar) ve M4 (target_points, 150 ms'ye kadar gecikme) yerine de geçer. Senaryolar: hold, camera_loss, targets, commands | ✔ sanal kartla · gerçek kartta denenmedi |
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
1. Arıza enjeksiyonu senaryoları: sadece bridged, frame tekrarı, M2 yeni session, M1 fault, E-stop, fan_state kes (native testlerde var, HIL'de yok)
2. R3.8 step 30→50 cm ve R3.9 köşelerini HIL senaryosu yap
3. Canlı grafik (gerçek / ölçülen / hedef yükseklik, fan hızı, durum) — demo için
4. M5 sözleşmesi gelince JSON alan adlarını ve binary payload düzenlerini ona uydur

## Donanım (HIL)

ESP32 DevKit + 2 adet 3.3 V USB-UART adaptörü:
- Kart USB'si → bilgisayar (height)
- Adaptör 1 ↔ GPIO16 (RX) / GPIO17 (TX) → M1 hattı
- Adaptör 2 ↔ GPIO26 (RX) / GPIO27 (TX) → game board hattı
- GND'ler ortak. Port adları `sim/hil_config.json`'da.
