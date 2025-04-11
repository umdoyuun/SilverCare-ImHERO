//DataSender.cpp
#include "DataSender.hpp"
#include <sstream>
#include <iostream>

size_t DataSender::WriteCallback(void* contents, size_t size, size_t nmemb, void* userp){
	return size * nmemb;
}

DataSender::DataSender(const std::string& FamilyId, const std::string& HealthDataUrl, const std::string& EnvironmentDataUrl, const std::string& LoginUrl, const std::string& LogoutUrl)
	: FamilyId(FamilyId)
	, HealthDataUrl(HealthDataUrl)
	, EnvironmentDataUrl(EnvironmentDataUrl)
	, LoginUrl(LoginUrl)
	, LogoutUrl(LogoutUrl){
		curl_global_init(CURL_GLOBAL_ALL);
		curl = curl_easy_init();
		if(curl) {
			curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "");
		}
}

DataSender::~DataSender(){
	if(curl) {
		curl_easy_cleanup(curl);
	}
	curl_global_cleanup();
}

bool DataSender::SendRequest(const std::string& url, const json& jsonData){
	if(!curl){
		return false;
	}

	std::string jsonStr = jsonData.dump();

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonStr.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
	
	if(url == LoginUrl){
		curl_easy_setopt(curl, CURLOPT_COOKIELIST, "ALL");
		curl_easy_setopt(curl, CURLOPT_COOKIEJAR, "");
	}
	else{
		curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "");
	}

	curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "");

    struct curl_slist* headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    CURLcode res = curl_easy_perform(curl);
    curl_slist_free_all(headers);

    if(res != CURLE_OK) {
        std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }
	long response_code;
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
	std::cout << "response code: " << response_code << '\n';
    return (response_code >= 200 && response_code < 300);
}

bool DataSender::SendHealthData(float heartRate) {
    json jsonData = {
        {"family_id", FamilyId},
        {"heart_rate", heartRate}
    };

    return SendRequest(HealthDataUrl, jsonData);
}

bool DataSender::SendEnvironmentData(const SensorData& data) {
    std::stringstream othersStream;
    othersStream << "";

    json jsonData = {
        {"family_id", FamilyId},
        {"temperature", data.temperature},
        {"humidity", data.humidity},
        {"dust_level", data.dust},
        {"ethanol", data.ethanol},
        {"others", othersStream.str()}
    };

    return SendRequest(EnvironmentDataUrl, jsonData);
}

bool DataSender::Login(const std::string& id, const std::string& password){
    json jsonData = {
        {"email", id},
        {"password", password}
    };

    return SendRequest(LoginUrl, jsonData);
}

bool DataSender::Logout(){
    json jsonData = {
        {"family_id", FamilyId}
    };

    bool result = SendRequest(LogoutUrl, jsonData);
    
    if(result && curl) {
        // 로그아웃 성공 시 쿠키 삭제
        curl_easy_setopt(curl, CURLOPT_COOKIELIST, "ALL");
    }
    
    return result;
}
