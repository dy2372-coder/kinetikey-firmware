# KinetiKey

Gesture-based combination lock on the STM32 B-L475E-IOT01A.
Hold the board in your fist and draw shapes in the air to set and enter a 3-gesture PIN.

---

## Gestures

| Gesture | ID |
|---------|----|
| Triangle — 3 strokes, pause at each corner | **1** |
| Square — 4 strokes, pause at each corner | **2** |
| Circle — one smooth continuous loop | **3** |
| Shake left-right | **← erase last digit** |

---

## Hardware

- **Board:** STM32 B-L475E-IOT01A (`disco_l475vg_iot01a`)
- **IMU:** LSM6DSL (onboard, I2C)
- **Power:** USB (ST-LINK port)

---

## Quick start

### 1. Install tools

- [VSCode](https://code.visualstudio.com) + [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- ST-Link driver via [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)

### 2. Flash

Connect via the left USB port (ST-LINK), then:

```bash
pio run --target upload
```

### 3. Serial monitor (optional)

```bash
pio device monitor --baud 115200
```

---

## Button controls

| Press duration | Action |
|----------------|--------|
| > 3 s | **CALIBRATE** — record one example of each shape |
| 1.5 – 3 s | **RECORD** — set a new 3-gesture PIN |
| Short (< 1.5 s) | **UNLOCK** — attempt to unlock |

**Calibrate before first use.** Classification won't work without templates.

---

## LED indicators

| LEDs | State |
|------|-------|
| LED1 slow blink | Idle |
| LED1 solid | Recording or calibrating |
| LED2 solid | Unlock mode |
| LED2 brief flash | Shape accepted (record) |
| LED1 brief flash | Digit accepted (unlock) |
| Both alternating fast | Success |
| LED1 × 3 slow blinks | Fail / timeout |

---

## Troubleshooting

**Board not detected**
- Use the left USB port (ST-LINK), not the right one
- Try a different cable — some are charge-only

**Shape not recognised**
- Calibrate first (hold 3 s from idle)
- Draw slowly and deliberately — each capture is ~3 s
- Triangle: 3 clear strokes with a small pause at each corner
- Square: 4 clear strokes with pauses
- Circle: one smooth loop, no stopping

**Shake erase not working**
- Shake quickly along one axis — the check runs in the first 0.75 s of motion

---

## Project structure

```
KinetiKey/
├── src/main.cpp      — firmware
├── platformio.ini    — build config
├── mbed_app.json     — float printf enable
└── README.md
```
