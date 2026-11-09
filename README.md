# MAX30102 + Arduino Uno pulse oximeter

Firmware, MATLAB scripts and the bench guide for our course project *Design of a Non-Invasive Pulse Oximeter Based on the MAX30102 Module and Arduino Uno Chip* (Istanbul Commerce University).

**Bench guide:** [English](https://sanselena.github.io/max30102-pulse-oximeter/) · [Türkçe](https://sanselena.github.io/max30102-pulse-oximeter/tr.html)
It covers the parts, the breadboard wiring, the first power-on steps, the firmware commands, recording, troubleshooting, and the finished test setup.

## Contents

| Path | What it is |
|---|---|
| `firmware/step1_i2c_scanner` | Lists the devices on the I²C bus. The sensor should answer at 0x57 (Table 10, step 3). |
| `firmware/step2_sensor_check` | Part ID (0x15), die temperature and INT wire test (Table 10, step 4). |
| `firmware/oximeter_main` | The full program (Section 2.7.3): FIFO reads every 40 ms, 0.5–4.6 Hz filters, beat detection, heart rate, SpO₂ = 104 − 17·R, quality flags, CSV output at 115200 baud. |
| `matlab/record_session.m` | Saves the Uno's output to a CSV file and prompts for the reference reading every 10 s. |
| `matlab/analyze_recording.m` | Lost samples, flag counts, SNR, and R per 10 s window for calibration. |
| `docs/` | The bench guide in English (`index.html`) and Turkish (`tr.html`). |

## Quick start

1. Install Arduino IDE 2. Select **Tools → Board → Arduino Uno** and your COM port.
2. Wire the circuit as in the guide's section 04, with USB unplugged.
3. Upload `step1_i2c_scanner`, then `step2_sensor_check`, then `oximeter_main`. Use 115200 baud for each.
4. In the Serial Monitor or Serial Plotter, send `p` for the pulse plot, `c` for CSV, or `?` for help.

All three sketches compile for the Uno with no warnings. `oximeter_main` uses 39% of flash and 27% of RAM.

## Data

Recordings (`*.csv`) are ignored by git on purpose. Volunteer data stays on our own computers, under codes, never names.

---

Fulya Hava Aykıt · Serenat Varol · Şamil Mirza Özavar
