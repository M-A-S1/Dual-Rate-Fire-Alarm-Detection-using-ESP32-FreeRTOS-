#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include <freertos/FreeRTOS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Web server instance
WebServer server(80);

// Shared data between cores
struct Metrics {
    float peakHzShort; // Peak frequency in Hz from short window (smoothed)
    float peakHzLong; // Peak frequency in Hz from long window (smoothed)
    bool fireAlarmDetected; // Overall
    bool fireShortDetected; // Short window
    bool fireLongDetected; // Long window
};
QueueHandle_t metricsQueue;

// Sizes
#define SAMPLES 1024
#define LONG_SAMPLES 4096 // ~0.18s window → 4× better resolution
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = SAMPLES;
const char* ssid = "Pixel_6762";
const char* password = "kecfr9dap3g5unp";
const float SAMPLE_RATE = 22627.0;

// Short window FFT
static float real[SAMPLES];
static float imag[SAMPLES];
static ArduinoFFT<float> fft(real, imag, SAMPLES, SAMPLE_RATE);

// Long window buffers + FFT
static int32_t long_buffer[LONG_SAMPLES];
static uint32_t long_idx = 0;
static float long_real[LONG_SAMPLES];
static float long_imag[LONG_SAMPLES];
static ArduinoFFT<float> long_fft(long_real, long_imag, LONG_SAMPLES, SAMPLE_RATE);

static unsigned int fireAlarm = 0; // Short window history
static unsigned int fireAlarmLong = 0; // Long window history
static unsigned long last = 0;

// ------------------------------ Filtering parameters (tune these) ------------------------------
const float MAG_THRESHOLD_SHORT = 1.0f; // minimum amplitude (linear) for short-window peaks (tune)
const float MAG_THRESHOLD_LONG  = 1.0f; // minimum amplitude (linear) for long-window peaks (tune)

// Short median filter length and EMA factor
const int SHORT_MEDIAN_N = 5; // must be odd
const float EMA_ALPHA_SHORT = 0.4f; // 0..1, higher -> less smoothing

// Long median filter length and EMA factor
const int LONG_MEDIAN_N = 3; // must be odd
const float EMA_ALPHA_LONG = 0.25f;

// -----------------------------------------------------------------------------------------------

// Precise major peak with parabolic interpolation on dB scale, returns freq and sets peak magnitude (linear)
double majorPeakParabolaDbWithMag(const float *vReal, const float *vImag, uint16_t samples, double samplingFrequency, double *outPeakMag) {
    double maxY = 0.0;
    uint16_t IndexOfMaxY = 0;
    const uint16_t half = (samples >> 1);

    // find max magnitude bin (skip DC at 0)
    for (uint16_t i = 1; i < (half - 1); i++) {
        double mag = sqrt((double)vReal[i] * vReal[i] + (double)vImag[i] * vImag[i]);
        if (mag > maxY) {
            maxY = mag;
            IndexOfMaxY = i;
        }
    }

    if (IndexOfMaxY < 1 || IndexOfMaxY >= half - 1) {
        if (outPeakMag) *outPeakMag = maxY;
        return (double)IndexOfMaxY * samplingFrequency / (double)samples;
    }

    // neighbor magnitudes
    double m1 = sqrt((double)vReal[IndexOfMaxY - 1] * vReal[IndexOfMaxY - 1] + (double)vImag[IndexOfMaxY - 1] * vImag[IndexOfMaxY - 1]);
    double m = maxY;
    double m2 = sqrt((double)vReal[IndexOfMaxY + 1] * vReal[IndexOfMaxY + 1] + (double)vImag[IndexOfMaxY + 1] * vImag[IndexOfMaxY + 1]);

    if (m1 <= 0 || m <= 0 || m2 <= 0) {
        if (outPeakMag) *outPeakMag = m;
        return (double)IndexOfMaxY * samplingFrequency / (double)samples;
    }

    // operate in log10 domain (as in your original routine)
    double db1 = log10(m1);
    double db = log10(m);
    double db2 = log10(m2);
    double denom = (2 * (db1 + db2 - 2 * db));
    double delta = 0.0;
    if (fabs(denom) > 1e-12) {
        delta = (db1 - db2) / denom;
    }
    double preciseIndex = (double)IndexOfMaxY + delta;
    // convert back: db is log10(m) so m = 10^db
    double precise_db = db + ( (db1 - db2) * delta / 2.0 ); // rough correction, magnitude estimate near peak
    double precise_m = pow(10.0, precise_db);
    if (outPeakMag) *outPeakMag = precise_m;
    return preciseIndex * samplingFrequency / (double)samples;
}

// ------------------------------ small utilities for median + EMA smoothing ------------------------------
static double short_peak_buf[SHORT_MEDIAN_N];
static int short_buf_count = 0;
static int short_buf_idx = 0;
static double short_ema = 0.0;

static double long_peak_buf[LONG_MEDIAN_N];
static int long_buf_count = 0;
static int long_buf_idx = 0;
static double long_ema = 0.0;

double median_of_copy(double *buffer, int n) {
    // copy to temp and sort (n small so O(n log n) fine)
    double tmp[21]; // ensure BIGGER than median sizes used (SHORT_MEDIAN_N <= 21)
    if (n > (int)sizeof(tmp)/sizeof(tmp[0])) n = sizeof(tmp)/sizeof(tmp[0]);
    for (int i = 0; i < n; ++i) tmp[i] = buffer[i];
    // simple sort (insertion)
    for (int i = 1; i < n; ++i) {
        double key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            --j;
        }
        tmp[j + 1] = key;
    }
    return tmp[n/2];
}

double update_short_peak_filter(double newVal, double mag) {
    // reject very small magnitude peaks (considered noise)
    if (mag < MAG_THRESHOLD_SHORT) {
        // don't insert the unreliable sample; but still decay EMA slightly to avoid sticking:
        short_ema = short_ema * (1.0 - EMA_ALPHA_SHORT) + short_ema * EMA_ALPHA_SHORT;
        return short_ema;
    }

    // circular buffer insert
    short_peak_buf[short_buf_idx] = newVal;
    short_buf_idx = (short_buf_idx + 1) % SHORT_MEDIAN_N;
    if (short_buf_count < SHORT_MEDIAN_N) short_buf_count++;

    double median = median_of_copy(short_peak_buf, short_buf_count);

    // EMA smoothing
    if (short_buf_count == 1 && short_ema == 0.0) short_ema = median;
    short_ema = EMA_ALPHA_SHORT * median + (1.0 - EMA_ALPHA_SHORT) * short_ema;
    return short_ema;
}

double update_long_peak_filter(double newVal, double mag) {
    if (mag < MAG_THRESHOLD_LONG) {
        long_ema = long_ema * (1.0 - EMA_ALPHA_LONG) + long_ema * EMA_ALPHA_LONG;
        return long_ema;
    }
    long_peak_buf[long_buf_idx] = newVal;
    long_buf_idx = (long_buf_idx + 1) % LONG_MEDIAN_N;
    if (long_buf_count < LONG_MEDIAN_N) long_buf_count++;

    double median = median_of_copy(long_peak_buf, long_buf_count);
    if (long_buf_count == 1 && long_ema == 0.0) long_ema = median;
    long_ema = EMA_ALPHA_LONG * median + (1.0 - EMA_ALPHA_LONG) * long_ema;
    return long_ema;
}

// Dashboard with enhanced overall indicator
void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Dual-Rate Fire Alarm Detector</title>
    <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@300;400;700&display=swap" rel="stylesheet">
    <style>
        :root { --bg: #121212; --card: #1e1e1e; --text: #e0e0e0; --accent: #bb86fc; --green: #03dac6; --red: #cf6679; }
        body { margin: 0; padding: 20px; font-family: 'Roboto', sans-serif; background: var(--bg); color: var(--text); min-height: 100vh; }
        h1 { text-align: center; color: var(--accent); margin-bottom: 10px; }
        .subtitle { text-align: center; font-weight: 300; margin-bottom: 40px; opacity: 0.8; }
        .overall-container { display: flex; justify-content: center; margin-bottom: 40px; }
        .overall-card { background: var(--card); border-radius: 16px; padding: 24px; box-shadow: 0 8px 24px rgba(0,0,0,0.4); text-align: center; max-width: 600px; width: 100%; transition: background 0.3s, transform 0.3s; }
        .overall-card h2 { margin: 0 0 16px; color: var(--accent); font-weight: 400; font-size: 1.3rem; }
        .overall-card .value { font-size: 3.5rem; font-weight: 700; margin: 20px 0; }
        .overall-card .info { font-size: 0.9rem; opacity: 0.7; margin-top: 10px; }
        .alarm-active { background: var(--red) !important; animation: pulse 2s infinite; }
        @keyframes pulse { 0%, 100% { transform: scale(1); } 50% { transform: scale(1.05); } }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 20px; max-width: 1200px; margin: 0 auto; }
        .card { background: var(--card); border-radius: 16px; padding: 24px; box-shadow: 0 8px 24px rgba(0,0,0,0.4); text-align: center; }
        .card h2 { margin: 0 0 16px; color: var(--accent); font-weight: 400; font-size: 1.3rem; }
        .card .value { font-size: 2.8rem; font-weight: 700; margin: 20px 0; }
        .detected { color: var(--green); }
        .not-detected { color: var(--red); }
        .unit { font-size: 1.2rem; opacity: 0.8; }
        .info { font-size: 0.9rem; opacity: 0.7; margin-top: 10px; }
        footer { text-align: center; margin-top: 50px; opacity: 0.7; font-size: 0.9rem; }
    </style>
    <script>
        function fetchData() {
            fetch('/metrics')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('peakHzShort').innerText = data.peakHzShort.toFixed(0);
                    document.getElementById('peakHzLong').innerText = data.peakHzLong.toFixed(0);
                    const shortEl = document.getElementById('fireShort');
                    shortEl.innerText = data.fireShortDetected ? "Detected" : "No";
                    shortEl.className = data.fireShortDetected ? "value detected" : "value not-detected";
                    const longEl = document.getElementById('fireLong');
                    longEl.innerText = data.fireLongDetected ? "Detected" : "No";
                    longEl.className = data.fireLongDetected ? "value detected" : "value not-detected";
                    const overallEl = document.getElementById('fireOverall');
                    const overallCard = document.querySelector('.overall-card');
                    if (data.fireAlarmDetected) {
                        overallEl.innerText = "ALARM ACTIVE!";
                        overallEl.className = "value detected";
                        overallCard.classList.add('alarm-active');
                    } else {
                        overallEl.innerText = "Safe";
                        overallEl.className = "value not-detected";
                        overallCard.classList.remove('alarm-active');
                    }
                })
                .catch(err => console.error('Error:', err));
        }
        setInterval(fetchData, 1000);
        window.onload = fetchData;
    </script>
</head>
<body>
    <h1>ESP32 Dual-Rate Fire Alarm Detection</h1>
    <p class="subtitle">Short window: fast response • Long window: 4× higher frequency resolution for improved accuracy</p>
    <div class="overall-container">
        <div class="overall-card">
            <h2>Overall Fire Alarm</h2>
            <div class="value" id="fireOverall">Loading...</div>
            <div class="info">Active if either window detects</div>
        </div>
    </div>
    <div class="grid">
        <div class="card"><h2>Peak Frequency (Short)</h2><div class="value" id="peakHzShort">--<span class="unit"> Hz</span></div></div>
        <div class="card"><h2>Peak Frequency (Long)</h2><div class="value" id="peakHzLong">--<span class="unit"> Hz</span></div></div>
        <div class="card"><h2>Fire Alarm - Fast</h2><div class="value" id="fireShort">Loading...</div><div class="info">Short window (~45ms, 22.09 Hz/bin)</div></div>
        <div class="card"><h2>Fire Alarm - Precise</h2><div class="value" id="fireLong">Loading...</div><div class="info">Long window (~180ms, 5.52 Hz/bin)</div></div>
    </div>
    <footer>Real-time dual-rate FFT processing • Peak frequencies from both windows • Updates every second</footer>
</body>
</html>
    )rawliteral";
    server.send(200, "text/html", html);
}

// JSON endpoint with peaks from both windows
void handleMetrics() {
    Metrics metrics;
    if (metricsQueue != NULL && xQueuePeek(metricsQueue, &metrics, 0)) {
        StaticJsonDocument<300> doc;
        doc["peakHzShort"] = metrics.peakHzShort;
        doc["peakHzLong"] = metrics.peakHzLong;
        doc["fireAlarmDetected"] = metrics.fireAlarmDetected;
        doc["fireShortDetected"] = metrics.fireShortDetected;
        doc["fireLongDetected"] = metrics.fireLongDetected;
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    } else {
        server.send(204, "application/json", "{}");
    }
}

// Helper functions
static void integerToFloat(int32_t *integer, float *vReal, float *vImag, uint16_t samples) {
    for (uint16_t i = 0; i < samples; i++) {
        vReal[i] = (integer[i] >> 16) / 10.0;
        vImag[i] = 0.0;
    }
}

unsigned int countSetBits(unsigned int n) {
    unsigned int count = 0;
    while (n) {
        count += n & 1;
        n >>= 1;
    }
    return count;
}

bool detectFrequency(unsigned int *mem, unsigned int minMatch, double peak_freq, double target_freq, double tol, unsigned int history_mask = 0xFFFFFFFF) {
    *mem <<= 1;
    *mem &= history_mask;
    if (fabs(peak_freq - target_freq) <= tol) {
        *mem |= 1;
    }
    return countSetBits(*mem) >= minMatch;
}

// Audio processing task (Core 1)
void loop2(void *pvParameters) {
    Metrics metrics = {0, 0, false, false, false};
    last = micros();
    while (1) {
        try {
            if (micros() - last < 45200) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }
            last = micros();
            static int32_t samples[BLOCK_SIZE];
            size_t num_bytes_read = 0;
            esp_err_t err = i2s_read(I2S_PORT, (char *)samples, BLOCK_SIZE * 4, &num_bytes_read, portMAX_DELAY);
            if (err != ESP_OK || num_bytes_read == 0) {
                continue;
            }
            // Accumulate for long window
            for (int i = 0; i < BLOCK_SIZE; i++) {
                if (long_idx < LONG_SAMPLES) {
                    long_buffer[long_idx++] = samples[i];
                }
            }

            // ---------------------- Short window processing ----------------------
            integerToFloat(samples, real, imag, SAMPLES);
            fft.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
            fft.compute(FFT_FORWARD);
            double peakMagShort = 0.0;
            double peak_freq_raw = majorPeakParabolaDbWithMag(real, imag, SAMPLES, SAMPLE_RATE, &peakMagShort);

            // Filter: median + EMA + magnitude threshold
            double peak_freq_short_smoothed = update_short_peak_filter(peak_freq_raw, peakMagShort);
            metrics.peakHzShort = (float)peak_freq_short_smoothed; // Set short window peak (smoothed)
            // Use smoothed frequency for detection
            bool fire_short_det = detectFrequency(&fireAlarm, 15, peak_freq_short_smoothed, 3100.0, 11.0);
            if (fire_short_det) {
                Serial.println("Detected fire alarm (short window)");
            }
            metrics.fireShortDetected = fire_short_det;

            // ---------------------- Long window high-resolution detection ----------------------
            if (long_idx >= LONG_SAMPLES) {
                integerToFloat(long_buffer, long_real, long_imag, LONG_SAMPLES);
                long_fft.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
                long_fft.compute(FFT_FORWARD);
                double peakMagLong = 0.0;
                double long_peak_freq_raw = majorPeakParabolaDbWithMag(long_real, long_imag, LONG_SAMPLES, SAMPLE_RATE, &peakMagLong);

                double long_peak_freq_smoothed = update_long_peak_filter(long_peak_freq_raw, peakMagLong);
                metrics.peakHzLong = (float)long_peak_freq_smoothed; // Set long window peak (smoothed)
                bool fire_long_det = detectFrequency(&fireAlarmLong, 8, long_peak_freq_smoothed, 3100.0, 3.0, 0xFFFF); // 16-bit history for faster off
                if (fire_long_det) {
                    Serial.println("Detected fire alarm (LONG high-resolution window)");
                }
                metrics.fireLongDetected = fire_long_det;
                long_idx = 0;
            }

            metrics.fireAlarmDetected = metrics.fireShortDetected || metrics.fireLongDetected;
            xQueueOverwrite(metricsQueue, &metrics);
        } catch (...) {
            Serial.println("Exception in loop2");
        }
    }
}

// Web server task (Core 0)
void loop1(void *pvParameters) {
    server.on("/", handleRoot);
    server.on("/metrics", handleMetrics);
    server.begin();
    Serial.println("Web server started");
    while (1) {
        server.handleClient();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup(void) {
    Serial.begin(115200);
    Serial.println("Configuring I2S...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
    metricsQueue = xQueueCreate(1, sizeof(Metrics));
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 22627,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = BLOCK_SIZE,
        .use_apll = true
    };
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,
        .ws_io_num = 15,
        .data_out_num = -1,
        .data_in_num = 32
    };
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    Serial.println("I2S driver installed.");
    xTaskCreatePinnedToCore(loop2, "AudioTask", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(loop1, "WebServer", 4096, NULL, 1, NULL, 0);
}

void loop(void) {
    // Empty
}
