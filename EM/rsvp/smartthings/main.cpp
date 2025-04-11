#include <iostream>
#include <string>
#include <fstream>
#include <curl/curl.h>

class SmartThingsAPI {
private:
    std::string apiToken;
    const std::string baseUrl = "https://api.smartthings.com/v1/devices";

    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
        size_t totalSize = size * nmemb;
        output->append(static_cast<char*>(contents), totalSize);
        return totalSize;
    }

    void loadEnvironmentVariables() {
        std::ifstream envFile(".env");
        if (!envFile.is_open()) {
            throw std::runtime_error("Could not open .env file");
        }

        std::string line;
        while (std::getline(envFile, line)) {
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
        
        const char* token = getenv("smartthings_token");
        if (!token) {
            throw std::runtime_error("smartthings_token not found in .env");
        }
        apiToken = token;
    }

public:
    SmartThingsAPI() {
        loadEnvironmentVariables();
    }

    void getDevices() {
        CURL* curl = curl_easy_init();
        if (!curl) {
            throw std::runtime_error("Failed to initialize CURL");
        }

        std::string response;
        struct curl_slist* headers = nullptr;
        
        try {
            headers = curl_slist_append(headers, ("Authorization: Bearer " + apiToken).c_str());
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, baseUrl.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

            CURLcode result = curl_easy_perform(curl);
            if (result != CURLE_OK) {
                throw std::runtime_error(curl_easy_strerror(result));
            }

            std::cout << "Response Data: " << response << std::endl;
        }
        catch (...) {
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
            throw;
        }

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }
};

int main() {
    try {
        SmartThingsAPI api;
        api.getDevices();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
