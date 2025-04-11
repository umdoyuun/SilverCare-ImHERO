# Matter SDK 가이드

WSL 환경(ubuntu 22.04 LTS)에서 cross compile 진행 과정

개발 환경 구축
```
sudo apt-get install git gcc g++ python pkg-config libssl-dev libdbus-1-dev libglib2.0-dev ninja-build python3-venv python3-dev unzip

git clone --recurse-submodules https://github.com/project-chip/connectedhomeip.git
cd connetedhomeip

git submodule init
git submodule update
git submodule update --init --recursive


pip install -r requirements.txt

source ./gn_build.sh
source ./scripts/bootstrap.sh
source ./scripts/activate.sh
```







```
================================================================================

Environment looks good, you are ready to go!

To reactivate this environment in the future, run this in your
terminal:

  source ./activate.sh

To deactivate this environment, run this:

  deactivate

Installing pip requirements for all...

[notice] A new release of pip is available: 23.2.1 -> 25.0.1
[notice] To update, run: pip install --upgrade pip
.--------------------------------
-- Instructions
'--------------------------------

To activate existing build environment in your shell, run (do this first):
source ./scripts/activate.sh

To re-create the build environment from scratch, run:
source ./scripts/bootstrap.sh

To compile the generated debug build:
ninja -C ./out/debug

To test the generated debug build (idempotent):
ninja -C ./out/debug check

To compile the generated release build:
ninja -C ./out/release

To test the generated release build (idempotent):
ninja -C ./out/release check

To build a custom build (for help run "gn args --list out/debug")
gn args ./out/custom
ninja -C ./out/custom

Hint: Set $ANDROID_HOME and $ANDROID_NDK_HOME to enable building for Android
      The required android sdk platform version is 21. It can be obtained from
      https://dl.google.com/android/repository/platform-26_r02.zip

Hint: Pass enable_efr32_builds=true to enable building for EFR32

Hint: Pass enable_p6_builds=true to this script to enable building for PSoC6-43012

Hint: Set $NXP_K32W0_SDK_ROOT to enable building for K32W061

Hint: Pass enable_qpg_builds=true to this script to enable building for QPG

Hint: Set $TI_SYSCONFIG_ROOT to enable building for cc13x2_26x2


Hint: Set $TIZEN_SDK_ROOT and $TIZEN_SDK_SYSROOT to enable building for Tizen
      Required Tizen SDK can be obtained from
      https://developer.tizen.org/development/tizen-studio/download

.--------------------------------
-- Build: GN configure
'--------------------------------
Done. Made 5291 targets from 397 files in 1068ms
Done. Made 5291 targets from 397 files in 772ms
```

cross compiler 도구 체인 설치
```
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

arm64용 라이브러리 설치를 위해 레포지토리 추가
```
sudo bash -c 'cat > /etc/apt/sources.list.d/ports.list << EOL
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ jammy main restricted universe multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ jammy-security main restricted universe multiverse
EOL'
sudo dpkg --add-architecture arm64
sudo apt update
sudo apt install -y libc6:arm64 \
    libssl-dev:arm64 \
    libdbus-1-dev:arm64 \
    libglib2.0-dev:arm64 \
    libglib2.0-dev:arm64 \
    libgio-dev:arm64 \
    libdbus-1-dev:arm64 \
    pkg-config-aarch64-linux-gnu
```

### Matter Devicce Application 빌드
examples 중 원하는 app으로 이동하여 빌드 진행

```
cd examples/"원하는 app"/linux/
gn gen out/debug --args='target_cpu="arm64" is_clang=false sysroot="/usr/aarch64-linux-gnu"'
ninja -C out/debug
```
>[!NOTE]
>환경 변수 경로 설정과 링커 설정 문제가 일어날 수 있다. 에러 메시지 보고 하나 하나 처리해 주자자

실행 파일
out/debug/chip-원하는app
위 파일 라즈베리파이에 전달하여 실행

### Custom Application 빌드
examples 중 All-Cluster-App 활용
```
cd ~/connectedhomeip/examples
cp -r all-clusters-app ./custom-app

python3 connectedhomeip/scripts/tools/zap/zap_download.py
zap ./custom-app.zap
```
zap 인터페이스에서 필요한 클러스터 선택 후 빌드 진행
