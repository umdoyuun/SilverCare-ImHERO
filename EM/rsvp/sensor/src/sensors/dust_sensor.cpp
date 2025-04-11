// src/sensors/dust_sensor.cpp
#include "sensors/dust_sensor.hpp"
#include <cmath>
#include <wiringPi.h>
#include <chrono>
#include <thread>
#include <numeric>
#include <algorithm>
DustSensor::DustSensor(int ledPin)
    : ledPin_(ledPin)
    , running_(false)
    , dustDensity_(0.0f)
    , readingIndex_(0)
    , bufferFilled_(false) {
    recentReadings_.resize(MEDIAN_WINDOW_SIZE, 0.0f);
}

DustSensor::~DustSensor() {
    stopReading();
}

bool DustSensor::initialize() {
    try {
        // LED 핀 설정
        pinMode(ledPin_, OUTPUT);
        digitalWrite(ledPin_, HIGH);  // LED 초기 상태는 OFF

        // SPI 초기화 (SPI0, CS0)
        spi_ = std::make_unique<SPI>(SPI::CHANNEL_0, SPI::CS0);

        clearError();
        return true;
    }
    catch (const std::exception& e) {
        updateError(std::string("Initialization failed: ") + e.what());
        return false;
    }
}

bool DustSensor::startReading() {
    if (running_) {
        return true;
    }

    if (!spi_) {
        updateError("SPI not initialized");
        return false;
    }

    try {
        running_ = true;
        readThread_ = std::make_unique<std::thread>(&DustSensor::readingLoop, this);
        clearError();
        return true;
    }
    catch (const std::exception& e) {
        updateError(std::string("Failed to start reading thread: ") + e.what());
        running_ = false;
        return false;
    }
}

bool DustSensor::stopReading() {
    running_ = false;
    if (readThread_ && readThread_->joinable()) {
        readThread_->join();
    }
    return true;
}

float DustSensor::calculateMedian(float newValue) {
    recentReadings_[readingIndex_] = newValue;
    readingIndex_ = (readingIndex_ + 1) % MEDIAN_WINDOW_SIZE;

    if (readingIndex_ == 0) {
        bufferFilled_ = true;
    }

    // 정렬을 위해 현재 값들을 임시 벡터에 복사
    std::vector<float> sortedValues;
    if (bufferFilled_) {
        sortedValues = recentReadings_;
    } else {
        sortedValues.assign(recentReadings_.begin(), 
                          recentReadings_.begin() + readingIndex_);
    }

    // 값들을 정렬
    std::sort(sortedValues.begin(), sortedValues.end());

    // 중앙값 계산
    if (sortedValues.empty()) {
        return 0.0f;
    } else if (sortedValues.size() % 2 == 0) {
        // 짝수 개수일 경우 가운데 두 값의 평균
        size_t mid = sortedValues.size() / 2;
        return (sortedValues[mid - 1] + sortedValues[mid]) / 2.0f;
    } else {
        // 홀수 개수일 경우 가운데 값
        return sortedValues[sortedValues.size() / 2];
    }
}

void DustSensor::readingLoop() {
    const float SENSITIVITY = 0.5f;  // V/(100 µg/m³), typical value
    while (running_) {
        try {
            // LED ON (LOW is ON for this sensor)
            digitalWrite(ledPin_, LOW);
            std::this_thread::sleep_for(std::chrono::microseconds(280));

            // Read ADC with multiple samples
            float avgAdcValue = 0.0f;
            for (int i = 0; i < NUM_ADC_SAMPLES; i++) {
                avgAdcValue += static_cast<float>(spi_->readADC(7));
            }
            avgAdcValue /= NUM_ADC_SAMPLES;

            // LED OFF
            digitalWrite(ledPin_, HIGH);
            std::this_thread::sleep_for(std::chrono::microseconds(9620));

            // Calculate voltage (0-5V range)
            float voltage = avgAdcValue * (5.0f / 1024.0f);

            // Calculate dust density using the sensor's sensitivity
            float voltageOffset = voltage - 0.65f;  // No dust reference level
            float rawDustDensity = 0.0f;

            if (voltageOffset > 0.0f) {
                // Convert to ug/m3 using sensitivity
                rawDustDensity = (voltageOffset / SENSITIVITY) * 100.0f;
            }

            // Apply median filter and round to 2 decimal places
            dustDensity_ = std::round(calculateMedian(rawDustDensity) * 100.0f) / 100.0f;

            clearError();

            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        catch (const std::exception& e) {
            updateError(std::string("Reading error: ") + e.what());
        }
    }
}

void DustSensor::updateError(const std::string& error) {
    errorMessage_ = error;
}

void DustSensor::clearError() {
    errorMessage_.clear();
}
