# 🔥 ESP32 Dual-Rate Fire Alarm Sound Detection (FreeRTOS)

A real-time **fire alarm sound detection system** built on the **ESP32** using **FreeRTOS**, **I2S audio capture**, and **dual-window FFT analysis**.  
The system detects the characteristic fire-alarm frequency (~3100 Hz) with both **fast response** and **high frequency accuracy**, and publishes results via a **WiFi web dashboard**.

---

## 📌 Project Overview

Fire alarms emit a strong tonal component near **3.1 kHz**.  
This project exploits that property using **frequency-domain analysis** directly on the ESP32—no cloud processing required.

Key ideas:
- **Short FFT window (1024 samples)** → fast detection  
- **Long FFT window (4096 samples)** → precise frequency confirmation  
- **FreeRTOS dual-core multitasking** → real-time audio + networking  
- **Noise-robust detection** using interpolation and filtering

---

## 🧠 System Features

- 🎤 Real-time audio capture via **I2S digital microphone**
- ⚡ Dual-rate FFT processing (1024 & 4096 samples)
- 📈 Parabolic interpolation for sub-bin frequency estimation
- 🧹 Median filter + EMA smoothing to suppress noise
- 🗳️ History-based voting for robust detection
- 🌐 Built-in **WiFi web dashboard**
- 🔁 Fully real-time, runs entirely on ESP32

---

## 🧩 System Architecture

### Hardware
- **ESP32** (dual-core microcontroller)
- **I2S digital microphone**
- WiFi-enabled client (PC / phone)

### Software
- **Arduino framework**
- **FreeRTOS**
- **ArduinoFFT**
- **WebServer**
- **ArduinoJson**

---

## 🔀 Task Structure (FreeRTOS)

| Task | Core | Purpose |
|----|----|----|
| Audio Processing Task | Core 1 | FFT, interpolation, filtering |
| Web Server Task | Core 0 | Dashboard & WiFi communication |
| Gatekeeper Task | Any | Safe shared resource access (Serial) |
| Tick Hook ISR | ISR | Periodic system messaging |

---

## 🔐 Gatekeeper Design Pattern

This project uses the **FreeRTOS Gatekeeper pattern** to safely access shared resources like `Serial`.

- Multiple tasks **send messages via a queue**
- A single **gatekeeper task** prints/logs messages
- Prevents race conditions and corrupted output

---

## 📊 Signal Processing Pipeline

The audio signal captured from the I2S microphone is processed entirely on the ESP32 in real time.  
The following steps describe the complete digital signal processing (DSP) pipeline used for fire alarm detection:

1. **I2S Audio Capture**  
   - Digital audio samples are captured using the ESP32 I2S peripheral  
   - Sampling rate: **22,627 Hz**  
   - Sample format: **32-bit signed integers**

2. **Preprocessing**  
   - Raw integer samples are converted to floating-point values  
   - A **Hann window** is applied to reduce spectral leakage

3. **FFT Computation**  
   - FFT converts time-domain audio into the frequency domain  
   - Two FFT window sizes are used:
     - **1024 samples (short window)** → fast response
     - **4096 samples (long window)** → high frequency resolution

4. **Magnitude Spectrum Calculation**  
   - Complex FFT output is converted to magnitude values  
   - Magnitudes are analyzed in the **logarithmic (dB) domain**

5. **Peak Detection**  
   - The FFT bin with maximum magnitude is selected as the dominant frequency

6. **Parabolic Interpolation**  
   - A parabola is fitted around the peak bin using adjacent bins  
   - Improves frequency estimation beyond FFT bin resolution

7. **Median Filtering**  
   - A median filter removes spurious peaks and impulsive noise

8. **Exponential Moving Average (EMA) Smoothing**  
   - EMA smooths frequency estimates while preserving responsiveness

9. **History-Based Detection Logic**  
   - Detection results are stored in a rolling history buffer  
   - An alarm is confirmed only after consistent detections across frames

10. **Final Alarm Decision**  
    - Short window ensures rapid detection  
    - Long window confirms alarm with high precision  
    - Alarm triggers if either or both windows detect the target frequency

---

## 📐 FFT Configuration

| Window Size | Time Resolution | Frequency Resolution |
|------------|-----------------|----------------------|
| 1024 | ~45 ms | 22.09 Hz/bin |
| 4096 | ~181 ms | 5.52 Hz/bin |

---

## 🎯 Detection Strategy

- **Short FFT window** detects alarms quickly, even with frequency drift  
- **Long FFT window** confirms alarms with high frequency accuracy  

Detection logic:
- Both windows detect → **High confidence alarm**
- Only short window detects → **Fast alert with lower confidence**
- Neither detects → **No alarm**

---

## 📡 Web Dashboard

The ESP32 hosts a web-based dashboard accessible via its IP address.  
The dashboard displays:
- Peak frequency (short and long FFT windows)
- Individual detection flags
- Overall fire alarm status

This enables **real-time remote monitoring** using any web browser.

---

## 🛠️ Build & Run

### Requirements
- ESP32 development board
- Arduino IDE
- Required libraries:
  - ArduinoFFT
  - ArduinoJson
  - WiFi
  - WebServer
  - FreeRTOS (ESP32 core)

### Steps
1. Clone the repository  
2. Open the project in Arduino IDE  
3. Configure WiFi credentials  
4. Flash the firmware to ESP32  
5. Open Serial Monitor at **115200 baud**  
6. Access the web dashboard via browser

---

## 📷 Results Summary

| Input Frequency | Detection Result |
|----------------|------------------|
| No alarm sound | ❌ No detection |
| ~3094 Hz | ✅ Short window |
| ~3106 Hz | ✅ Short window |
| 3100 Hz | ✅✅ Both windows |

---

## 📘 Documentation

This repository includes a complete IEEE-format project report covering:
- System architecture
- FreeRTOS task design
- DSP pipeline
- Experimental evaluation

---

## 👨‍💻 Authors

- Muhammad Ali (498148)  
- Waqas Jahangir (494191)  
- Muhammad Umar (577600)  

**Supervisor:** Dr. Usman Zabit  

---

## 📄 License

This project is intended for academic and educational use.

---

## ⭐ Acknowledgements

- Espressif Systems  
- FreeRTOS  
- Arduino Community  


