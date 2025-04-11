// CredentialsManager.hpp
#pragma once
#include <string>
#include <vector>

class CredentialsManager {
private:
    const std::string KEY_FILE;
    const std::string CRED_FILE;
    std::vector<unsigned char> key;

    void generateKey();
    bool loadKey();
    std::vector<unsigned char> encrypt(const std::string& plaintext);
    std::string decrypt(const std::vector<unsigned char>& ciphertext);

public:
    CredentialsManager(const std::string& keyFile = "encryption.key", 
                      const std::string& credFile = "credentials.enc");
    ~CredentialsManager();

    bool saveCredentials(const std::string& username, const std::string& password);
    bool loadCredentials(std::string& username, std::string& password);
};
