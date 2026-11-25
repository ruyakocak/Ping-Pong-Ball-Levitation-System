# M3 — Control Firmware (Ultrasonik)

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
| `lib/m3_core/m3_target_buffer.*` | Zaman damgalı hedefler, saat ofseti (10 s'lik min), interpolasyon, yeni oturumda yumuşak geçiş | ✔ R3.7 (mantık; HIL hâli bekliyor) |
| `lib/m3_core/m3_cmd_check.*` | Parametre tablosu, aralıklar; güvenlik limitleri salt okunur | ✔ R3.10 |
| `lib/m3_core/m3_command.*` | command → ack (gerekçeli), mod değişimi, game_out yalnız game modunda | ✔ R3.10 |
| `src/main.cpp` | ESP32: core 1 kontrol görevi, UART'lar, simball | iskelet; esp32 + esp32_simball derleniyor (uyarısız), karta henüz yüklenmedi |
| `tools/tune_gains.c` | Kazanç araması: 27 model köşesi (k_air, c_drag, tau ×0,7/1/1,3) × 3 yükseklik, gürültü 5 mm, %5 düşen kare; en kötü durumu en iyi olan kazançlar | ✔ R3.9 |
| `sim/hil_sim.py` | HIL: M1 + M2'yi karta karşı oynar; seri port (gerçek kart) ya da `--virtual` (sanal kart). Senaryolar: hold, camera_loss | ✔ sanal kartla R3.13 · gerçek kartta Start/Reset UDP'si bekliyor |
| `tools/vboard.c` | Sanal kart: firmware'in aynı m3_core + contract kodu, main.cpp gibi bağlanmış; linkler ve saat stdin/stdout'tan, 1 ms adımlı | ✔ R3.13 |
| `sim/m3sim/` | Python contract aynası, plant modeli | ✔ |
| `contract/test_vectors.json` | C ve Python'un aynı byte'ları ürettiği vektörler | ✔ |

Kural: `lib/` altındaki hiçbir dosya donanım çağrısı yapmaz; zaman hep parametre olarak gelir.

## Sıradaki işler

**Firmware tarafı**
1. Karta yükle: önce `pio run -e esp32_simball -t upload`, seri monitörde simüle topu izle
2. Net görevi (core 0): Wi-Fi, UDP 5005 — command → `m3_ctrl_event` + ack, status 30 Hz
3. Net görevinde `target_points` JSON'unu çözüp `m3_ctrl_on_target_points`'e ver (`t` → ms)
4. Net görevinde `command` JSON → `m3_command_apply` → `ack` JSON; Serial1'de `game_out` → `m3_command_game_out`
5. `status` mesajına `ball_stuck` uyarısını koy; game board linki (Serial1)
6. Kart üstü zamanlama: height alındı → fan_cmd gönderildi süresi (R3.3), debug UDP'den

**PC tarafı**
1. HIL: Start/Reset'i UDP ile gönder, senaryo dosyaları (hold, step 30→50, sine, arızalar)
2. Arıza enjeksiyonu: heights dur, sadece bridged, frame tekrarı, M2 yeni session, M1 fault, E-stop, fan_state kes (R3.5, R3.11)
3. M4 yerine target_points göndericisi, 150 ms'ye kadar rastgele gecikme (R3.7)
4. R3.9'u HIL'de tekrarla (native sim'de geçiyor: kp 1,4 ki 0,8 kd 0,13)
5. Canlı grafik (gerçek / ölçülen / hedef yükseklik, fan hızı, durum) — demo için
6. Repoyu GitHub'a koy ve CI'ın iki sistemde de yeşil olduğunu gör (R3.13'ün son adımı)

## Donanım (HIL)

ESP32 DevKit + 2 adet 3.3 V USB-UART adaptörü:
- Kart USB'si → bilgisayar (height)
- Adaptör 1 ↔ GPIO16 (RX) / GPIO17 (TX) → M1 hattı
- Adaptör 2 ↔ GPIO26 (RX) / GPIO27 (TX) → game board hattı
- GND'ler ortak. Port adları `sim/hil_config.json`'da.
