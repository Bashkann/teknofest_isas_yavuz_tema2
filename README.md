# YAVUZ AUV — TEKNOFEST 2026 İleri Kategori Tema 2
> Otonom Navigasyon, İntikal ve Kontrollü Alan Geçişi

## Mimari Özeti

```
┌─────────────────────────────────────────────────────────────────┐
│                     YAVUZ AUV YAZILIM MİMARİSİ                  │
├─────────────┬──────────────┬─────────────┬──────────────────────┤
│ LOCALIZATION│   CONTROL    │ NAVIGATION  │     SIMULATION       │
│             │              │             │                      │
│ EKF Node    │ Depth PID    │ State Mach. │ Gazebo Harmonic      │
│ Dead Recon. │ Heading PID  │ Waypoint    │ Theme2 World         │
│ IMU Fusion  │ Surge PID    │ BuoyCircler │ Buoyancy Plugin      │
│ Depth→Z     │ Thruster Alloc│ Mission Exec│ Thruster Plugin     │
└─────────────┴──────────────┴─────────────┴──────────────────────┘
```

## Su Altı Konum Tespiti (Dead Reckoning + EKF)

GPS su altında çalışmaz. Çözümümüz:

| Sensör | Katkı | Doğruluk |
|--------|-------|-----------|
| BMX160 IMU (9-eksen) | X/Y/Z ivme → integral | ±0.5m/dk |
| Basınç sensörü | Z ekseni (derinlik) | ±5cm |
| Pixhawk EKF2 | Sensör füzyonu | - |
| Motor RPM modeli | Hız tahmini | ±10% |
| Manyetometre | Heading | ±2° |

**NOT:** 5 dakikalık görevde ~1-2m drift beklenir — yarışma havuzu boyutları için kabul edilebilir.

## Sistem Gereksinimleri

- Ubuntu 22.04 / 24.04
- ROS2 Jazzy Jalopy
- Gazebo Harmonic
- Python 3.10+

## Kurulum

```bash
chmod +x setup.sh && ./setup.sh
```

## Simülasyon Başlatma

```bash
source install/setup.bash

# Tam simülasyon (Gazebo + ROS2 + Otonom)
ros2 launch yavuz_bringup simulation.launch.py

# Sadece görev başlat (simülasyon açıkken)
ros2 launch yavuz_bringup mission.launch.py \
  rotation_lat:=37.1234 rotation_lon:=37.5678 \
  end_lat:=37.1240 end_lon:=37.5690
```

## Tema 2 Görev Adımları

```
[INIT] → [DIVE 2m] → [NAVIGATE→BUOY] → [CIRCLE 360°] → [NAVIGATE→END] → [SURFACE IN SQUARE] → [DONE]
   └─ Acil dur butonu her an aktif
```

## Puan Analizi

| Görev | Puan | Durum |
|-------|------|-------|
| Şamandıra etrafında dönüş | 40 | ✅ Otonom |
| Bitiş alanından çıkış | 60 | ✅ Otonom |
| **Toplam** | **100** | - |

## Paket Yapısı

```
src/
├── yavuz_description/    # URDF/SDF ROV modeli
├── yavuz_simulation/     # Gazebo dünyası + launch
├── yavuz_control/        # PID kontrolcüler + thruster allocator
├── yavuz_navigation/     # State machine + waypoint yöneticisi
├── yavuz_localization/   # EKF dead reckoning
└── yavuz_bringup/        # Ana launch dosyaları
```

## Akademik Danışman
Turgay KOÇ — turgaykoc@sdu.edu.tr  
Süleyman Demirel Üniversitesi — Robotik ve İnovasyon Topluluğu
