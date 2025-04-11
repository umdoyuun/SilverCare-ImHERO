// include/sensors/dust_sensor.hpp
#pragma once

#include <atomic>
#include <thread>
#include <string>
#include <memory>
#include <array>
#include <vector>
#include "drivers/spi.hpp"

class DustSensor {
public:
    explicit DustSensor(int ledPin = 21);
    ~DustSensor();

    // 복사 및 이동 연산 금지
    DustSensor(const DustSensor&) = delete;
    DustSensor& operator=(const DustSensor&) = delete;
    DustSensor(DustSensor&&) = delete;
    DustSensor& operator=(DustSensor&&) = delete;

    bool initialize();
    bool startReading();
    bool stopReading();
    bool isRunning() const { return running_; }
    float getDustDensity() const { return dustDensity_; }
    const std::string& getErrorMessage() const { return errorMessage_; }

private:
    static constexpr size_t MEDIAN_WINDOW_SIZE = 11;  // 중앙값 계산을 위한 윈도우 크기
    static constexpr int NUM_ADC_SAMPLES = 5;      // ADC 샘플링 수

    const int ledPin_;
    std::shared_ptr<SPI> spi_;
    std::atomic<bool> running_;
    std::atomic<float> dustDensity_;
    std::string errorMessage_;
    std::unique_ptr<std::thread> readThread_;

    // 중앙값 필터 관련 멤버
    std::vector<float> recentReadings_;
    size_t readingIndex_ = 0;
    bool bufferFilled_ = false;

    void readingLoop();
    void updateError(const std::string& error);
    void clearError();
    float calculateMedian(float newValue);
};
