# 사용법

### 의존성 설치
```
chmod +x install_dependencies.sh
./install_dependencies.sh
```
---
## 센서 드라이버 사용법
### 초기화
``` cpp
#include "managers/sensor_manager.hpp"

SensorManager sm;
sm.initialize();
```

### 자료구조
```
struct SensorData {
    float temperature;  // 온도 (°C)
    float humidity;     // 습도 (%)
    float dust;        // 먼지 농도 (µg/m³)
    float ethanol;     // 에탄올 농도 (%)
    float heartrate;   // 심박수 (BPM)
    std::string error_message;  // 오류 
};
```

### 데이터 읽기
```
SensorData readings = sensorManager.readAllSensors();

if (readings.error_message.empty()) {
    std::cout << "Temperature: " << readings.temperature << "°C\n"
              << "Humidity: " << readings.humidity << "%\n"
              << "Dust: " << readings.dust << "µg/m³\n"
              << "Ethanol: " << readings.ethanol << "%\n"
              << "Heart Rate: " << readings.heartrate << "BPM\n";
} else {
    std::cerr << "Error: " << readings.error_message << std::endl;
}
```
---

## 데이터 센더 사용법
### 초기화
```
#include "DataSender.hpp"


DataSender sender(
    "family_id_123",               // Family ID
    "http://api.example.com/health",     // Health Data URL
    "http://api.example.com/environment", // Environment Data URL
    "http://api.example.com/login",      // Login URL
    "http://api.example.com/logout"      // Logout URL
);
```

### 로그인
```
if (!sender.Login("user@example.com", "password")) {
    std::cerr << "Login failed" << std::endl;
    return -1;
}
```

### 데이터 전송
```
// 건강 데이터 전송
float heartRate = 75.0f;
sender.SendHealthData(heartRate);

// 환경 데이터 전송
SensorData environmentData = {
    .temperature = 25.0f,
    .humidity = 60.0f,
    .dust = 35.0f,
    .ethanol = 0.5f
};
sender.SendEnvironmentData(environmentData);
```

### 로그아웃
```
sender.Logout();
```
