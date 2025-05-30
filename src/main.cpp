#include <Arduino.h>
  #include <WiFi.h>
  #include <Arduino_MQTT_Client.h>
  #include <ThingsBoard.h>
  #include <DHT20.h>
  #include <Adafruit_NeoPixel.h>
  #include <Wire.h>
  #include <ArduinoOTA.h>

  // Thông tin WiFi
  constexpr char WIFI_SSID[] = "abcd";
  constexpr char WIFI_PASSWORD[] = "123456789";

  // Thông tin Core IoT (ThingsBoard)
  constexpr char TOKEN[] = "d29e04h4jwpi602bbc5z";
  constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
  constexpr uint16_t THINGSBOARD_PORT = 1883U;

  // Cấu hình NeoPixel
  #define LED_PIN 32 // GPIO 32
  #define LED_COUNT 4
  Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
  // Cấu hình DHT20
  #define SDA_PIN GPIO_NUM_21
  #define SCL_PIN GPIO_NUM_22
  DHT20 dht20; // Biến toàn cục duy nhất

  // Cấu hình ThingsBoard
  constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
  constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;
  constexpr int16_t telemetrySendInterval = 5000U; // Gửi telemetry mỗi 5 giây

  // Khởi tạo WiFi và ThingsBoard
  WiFiClient wifiClient;
  Arduino_MQTT_Client mqttClient(wifiClient);
  ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

  // Biến toàn cục cho trạng thái LED
  volatile uint32_t led_color = 0;
  volatile uint8_t led_brightness = 10;

  // Hàm thiết lập WiFi
  void setup_wifi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
  }

  // Hàm thiết lập OTA
  void setup_ota() {
    ArduinoOTA.setHostname("ESP32_DHT20_NeoPixel");
    ArduinoOTA.onStart([]() {
      Serial.println("OTA Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nOTA End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("OTA Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
  }

  // Hàm callback RPC để điều khiển LED
 RPC_Response setState(const RPC_Data &data) {
    Serial.println("Received RPC for LED state");
    int state = data; // Mặc định là 0 nếu không có trường "state"
    // Cập nhật LED
    switch (state) {
      case 5: // RED
        strip.fill(strip.Color(255, 0, 0), 0, LED_COUNT);
        break;
      case 4: // GREEN
        strip.fill(strip.Color(0, 255, 0), 0, LED_COUNT);
        break;   
      default: // OFF
        strip.fill(strip.Color(0, 0, 0), 0, LED_COUNT);
        break;
    }
    strip.show();
    Serial.printf("Setting LED state to %d\n", state);
    return RPC_Response("setState", "success");
}

const std::array<RPC_Callback, 1U> callbacks = {
    RPC_Callback{ "setState", setState }
};
  // Task đọc và gửi dữ liệu DHT20
  void dht20_task(void *pvParameters) {
    while (1) {
      if (WiFi.status() != WL_CONNECTED || !tb.connected()) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }
      if (!dht20.read()) {
        Serial.println("DHT20 read failed, retrying...");
        vTaskDelay(500 / portTICK_PERIOD_MS); // Tăng thời gian chờ lên 500ms
        continue;
      }

      float temperature = dht20.getTemperature();
      float humidity = dht20.getHumidity();
      if (!isnan(temperature) && !isnan(humidity)) {
        Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
        tb.sendTelemetryData("temperature", temperature);
        tb.sendTelemetryData("humidity", humidity);
      } else {
        Serial.println("Failed to read from DHT20 sensor!");
      }
      vTaskDelay(telemetrySendInterval / portTICK_PERIOD_MS);
    }
  }

  // Task xử lý kết nối ThingsBoard, OTA và RPC
  void thingsboard_task(void *pvParameters) {
    while (1) {
      if (!tb.connected() && WiFi.status() == WL_CONNECTED) {
        Serial.print("Connecting to Core IoT: ");
        Serial.println(THINGSBOARD_SERVER);
        if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
          Serial.println("Connected to Core IoT");
          if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
            Serial.println("Failed to subscribe for RPC");
          }
        } else {
          Serial.println("Failed to connect to Core IoT");
        }
      }

      tb.loop();
      ArduinoOTA.handle();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }

  void setup() {
    Serial.begin(SERIAL_DEBUG_BAUD);

    // Khởi tạo NeoPixel
    strip.begin();
    strip.setBrightness(50);
    strip.show();

    // Khởi tạo DHT20
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    if (!dht20.begin()) {
      Serial.println("DHT20 initialization failed!");
      while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Kết nối WiFi
    setup_wifi();

    // Thiết lập OTA
    setup_ota();

    // Tạo các task FreeRTOS
    xTaskCreate(dht20_task, "DHT20_Task", 8192, NULL, 1, NULL);
    xTaskCreate(thingsboard_task, "ThingsBoard_Task", 8192, NULL, 2, NULL);
  }

  void loop() {
    // Không cần loop vì các tác vụ được xử lý bởi FreeRTOS
  }