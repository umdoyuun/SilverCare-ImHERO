#include "managers/sensor_manager.hpp"
#include "DataSender.hpp"
#include "managers/CredentialsManager.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <sys/types.h>
using namespace std;

volatile sig_atomic_t gSignalStatus = 0;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    switch(signum) {
        case SIGTERM: // systemd로부터의 종료 신호
            std::cout << "SIGTERM received: Termination requested\n";
            break;
        case SIGINT:  // Ctrl+C
            std::cout << "SIGINT received: Interactive attention signal\n";
            break;
        case SIGQUIT: // Ctrl+\
            std::cout << "SIGQUIT received: Quit program\n";
            break;
        case SIGPWR:  // 전원 실패
            std::cout << "SIGPWR received: Power failure\n";
            break;
        case SIGHUP:  // 터미널 연결 종료
            std::cout << "SIGHUP received: Terminal disconnected\n";
            break;
    }
    gSignalStatus = 1;
}


void load_env() {
    std::ifstream file(".env");
    if (!file.is_open()) {
        std::cerr << "Could not open .env file" << std::endl;
        return;
    }
    
    std::string line;    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);
            
            if (!key.empty() && !value.empty()) {
                setenv(key.c_str(), value.c_str(), 1);
            }
        }
    }
}

int main() {
	 struct sigaction sigIntHandler;
    
    sigIntHandler.sa_handler = signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
	bool login_flag = false;

    sigaction(SIGTERM, &sigIntHandler, NULL);
    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGQUIT, &sigIntHandler, NULL);
    sigaction(SIGPWR, &sigIntHandler, NULL);
    sigaction(SIGHUP, &sigIntHandler, NULL);

	load_env();
	SensorManager sm;
	CredentialsManager cm;
	string userid, password;
	const char* health_data_url = getenv("health_data_url");
	const char* environment_data_url = getenv("environment_data_url");
	const char* login_url = getenv("login_url");
	const char* logout_url = getenv("logout_url");
	const char* family_id = getenv("family_id");
	if(family_id){
		cout << "저장된 Family_ID가 있습니다.\n";
	}
	else{
		cout << "저장된 Family_ID가 없습니다. \nFamily_Id: ";
		string new_family_id;
		cin >> new_family_id;
		setenv("family_id", new_family_id.c_str(), 1);
	}
	
    DataSender sender(
		family_id,
		health_data_url,
		environment_data_url,
		login_url,
		logout_url
    );

	if(!cm.loadCredentials(userid, password)){
		cout << "최초 로그인 하십시오. \nUserId: ";
		cin >> userid;
		cout << "Password: ";
		cin >> password;
		if(sender.Login(userid, password)){
			login_flag = true;
			cm.saveCredentials(userid, password);
			cout << "로그인 정보가 저장되었습니다.\n";
		}
		else{
			while(!gSignalStatus){
				cout << "로그인 정보가 잘못되었습니다.\n";
				cout << "다시 로그인 하십시오\n.UserId: ";
				cin >> userid;
				cout << "Password: ";
				cin >> password;
				if(sender.Login(userid, password)){
					login_flag = true;
					cm.saveCredentials(userid, password);
					cout << "로그인 정보가 저장되었습니다.\n";
					break;
				}
			}
		}
	}
	else{
		sender.Login(userid, password);
		login_flag = true;
	}

	sm.initialize();
	auto lastTime = chrono::system_clock::now();
	while(!gSignalStatus){
		SensorData Data = sm.readAllSensors();
		auto currentTime = chrono::system_clock::now();
		auto elapsedTime = chrono::duration_cast<chrono::minutes>(currentTime - lastTime);
		if(elapsedTime >= chrono::minutes(5)){
			cout << "정기 환경 데이터 전송\n";
			sender.SendEnvironmentData(Data);
			lastTime = currentTime;
		}
		if(Data.ethanol > 2.0 || Data.dust > 25.0){
			cout << "긴급 환경 데이터 전송" << Data.ethanol << "% " << Data.dust << "㎍/㎥\n";
			sender.SendEnvironmentData(Data);
		}
		if(Data.heartrate > 10.0){
			cout << "심박 데이터 전송 " << Data.heartrate << "BPM\n";
			sender.SendHealthData(Data.heartrate);
		}
		this_thread::sleep_for(chrono::seconds(1));
	}
	if(login_flag){
		cout << "Logout\n";
		sender.Logout();
	}

    return 0;
}

