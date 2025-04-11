// CredentialsManager.cpp
#include "managers/CredentialsManager.hpp"
#include <fstream>
#include <openssl/evp.h>
#include <openssl/aes.h>
#include <openssl/rand.h>
#include <stdexcept>

CredentialsManager::CredentialsManager(const std::string& keyFile, const std::string& credFile)
    : KEY_FILE(keyFile), CRED_FILE(credFile) {
    if (!loadKey()) {
        generateKey();
    }
}

CredentialsManager::~CredentialsManager() {
    // 메모리에서 키 안전하게 제거
    std::fill(key.begin(), key.end(), 0);
}

void CredentialsManager::generateKey() {
    key.resize(32); // AES-256용 32바이트 키
    if (RAND_bytes(key.data(), key.size()) != 1) {
        throw std::runtime_error("Failed to generate secure key");
    }
    
    std::ofstream keyFile(KEY_FILE, std::ios::binary);
    if (!keyFile) {
        throw std::runtime_error("Failed to open key file for writing");
    }
    
    keyFile.write(reinterpret_cast<char*>(key.data()), key.size());
}

bool CredentialsManager::loadKey() {
    std::ifstream keyFile(KEY_FILE, std::ios::binary);
    if (!keyFile) return false;
    
    key.resize(32);
    keyFile.read(reinterpret_cast<char*>(key.data()), key.size());
    return keyFile.gcount() == 32;
}

std::vector<unsigned char> CredentialsManager::encrypt(const std::string& plaintext) {
    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("Failed to create cipher context");
    }

    unsigned char iv[12];  // GCM 모드용 12바이트 IV
    if (RAND_bytes(iv, sizeof(iv)) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to generate IV");
    }

    if (EVP_EncryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, key.data(), iv) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to initialize encryption");
    }

    std::vector<unsigned char> ciphertext(12 + plaintext.size() + EVP_MAX_BLOCK_LENGTH + 16);
    std::copy(iv, iv + 12, ciphertext.begin());  // IV를 ciphertext 시작 부분에 복사
    
    int len;
    if (EVP_EncryptUpdate(ctx, ciphertext.data() + 12, &len, 
        reinterpret_cast<const unsigned char*>(plaintext.data()), plaintext.size()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to encrypt data");
    }

    int finalLen;
    if (EVP_EncryptFinal_ex(ctx, ciphertext.data() + 12 + len, &finalLen) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to finalize encryption");
    }
    len += finalLen;
    
    unsigned char tag[16];
    if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_GET_TAG, 16, tag) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to get tag");
    }
    
    std::copy(tag, tag + 16, ciphertext.begin() + 12 + len);
    ciphertext.resize(12 + len + 16);  // IV(12) + 암호문 + TAG(16)
    
    EVP_CIPHER_CTX_free(ctx);
    return ciphertext;
}

std::string CredentialsManager::decrypt(const std::vector<unsigned char>& ciphertext) {
    if (ciphertext.size() < 28) {  // IV(12) + 최소 암호문 크기 + TAG(16)
        throw std::runtime_error("Invalid ciphertext size");
    }
    
    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("Failed to create cipher context");
    }

    // IV 추출 (처음 12바이트)
    unsigned char iv[12];
    std::copy(ciphertext.begin(), ciphertext.begin() + 12, iv);

    if (EVP_DecryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, key.data(), iv) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to initialize decryption");
    }

    std::vector<unsigned char> plaintext(ciphertext.size() - 28);  // IV와 TAG 크기 제외
    int len;
    if (EVP_DecryptUpdate(ctx, plaintext.data(), &len,
        ciphertext.data() + 12, ciphertext.size() - 28) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to decrypt data");
    }

    // TAG 설정 (마지막 16바이트)
    unsigned char tag[16];
    std::copy(ciphertext.end() - 16, ciphertext.end(), tag);
    if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_TAG, 16, tag) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to set tag");
    }

    int finalLen;
    int ret = EVP_DecryptFinal_ex(ctx, plaintext.data() + len, &finalLen);
    EVP_CIPHER_CTX_free(ctx);
    
    if (ret <= 0) {
        throw std::runtime_error("Failed to verify tag or finalize decryption");
    }

    plaintext.resize(len + finalLen);
    return std::string(plaintext.begin(), plaintext.end());
}

bool CredentialsManager::saveCredentials(const std::string& username, const std::string& password) {
    try {
        std::string credentials = username + "\n" + password;
        auto encrypted = encrypt(credentials);
        
        std::ofstream credFile(CRED_FILE, std::ios::binary);
        if (!credFile) return false;
        
        credFile.write(reinterpret_cast<char*>(encrypted.data()), encrypted.size());
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool CredentialsManager::loadCredentials(std::string& username, std::string& password) {
    try {
        std::ifstream credFile(CRED_FILE, std::ios::binary);
        if (!credFile) return false;
        
        std::vector<unsigned char> encrypted(
            (std::istreambuf_iterator<char>(credFile)),
            std::istreambuf_iterator<char>());
        
        std::string decrypted = decrypt(encrypted);
        size_t pos = decrypted.find('\n');
        if (pos == std::string::npos) return false;
        
        username = decrypted.substr(0, pos);
        password = decrypted.substr(pos + 1);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}
