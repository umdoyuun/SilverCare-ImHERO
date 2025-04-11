#!/bin/bash

# 에러 발생시 스크립트 중단
set -e

echo "필요한 라이브러리 설치를 시작합니다..."

# 필요한 기본 도구 설치
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    libcurl4-openssl-dev \
    libssl-dev

# wiringPi 설치
echo "wiringPi 설치 중..."
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
cd ..
rm -rf WiringPi

# nlohmann json 설치
echo "nlohmann json 설치 중..."
git clone https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..
rm -rf json

# OpenSSL이 이미 설치되어 있는지 확인
if ! dpkg -l | grep -q libssl-dev; then
    echo "OpenSSL 개발 라이브러리 설치 중..."
    sudo apt-get install -y libssl-dev
fi

echo "모든 라이브러리가 성공적으로 설치되었습니다!"

# 설치된 버전 정보 출력
echo "설치된 라이브러리 버전 정보:"
echo "wiringPi: $(gpio -v 2>&1 | head -n 1)"
echo "CURL: $(curl --version | head -n 1)"
echo "OpenSSL: $(openssl version)"

# ldconfig 실행하여 라이브러리 캐시 업데이트
sudo ldconfig

echo "설치가 완료되었습니다."
