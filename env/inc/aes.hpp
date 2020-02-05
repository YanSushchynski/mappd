#ifndef AES_HPP
#define AES_HPP

#include <openssl/aes.h>
#include <openssl/bio.h>
#include <openssl/bioerr.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/evperr.h>

#include <boost/format.hpp>
#include <stdexcept>
#include <string>

template <uint32_t bits> struct aes_ssl_t {
  static_assert(bits == 128 || bits == 192 || bits == 256, "Wrong AES key size");
  explicit aes_ssl_t(const char (&key)[(bits / 8u) + 1u]) : key_(key) {}
  virtual ~aes_ssl_t() = default;

  const std::string &get_key() const { return key_; }
  std::pair<std::shared_ptr<void>, int32_t> encrypt(const void *const plaintext, size_t plaintext_len) const {
    EVP_CIPHER_CTX *ctx;
    const EVP_CIPHER *cph;
    int32_t len, rc, ciphertext_len;

    if constexpr (bits == 128) {
      cph = EVP_aes_128_cbc();
    } else if constexpr (bits == 192) {
      cph = EVP_aes_192_cbc();
    } else if constexpr (bits == 256) {
      cph = EVP_aes_256_cbc();
    }

    uint8_t *ciphertext = reinterpret_cast<uint8_t *>(std::malloc(plaintext_len + EVP_CIPHER_block_size(cph)));
    if (!(ctx = EVP_CIPHER_CTX_new())) {
      raise_crypto_error_(ERR_get_error());
    }

    if (!(rc = EVP_EncryptInit_ex(ctx, cph, nullptr, reinterpret_cast<const uint8_t *>(key_), nullptr))) {
      raise_crypto_error_(ERR_get_error());
    }

    if (!(rc = EVP_EncryptUpdate(ctx, ciphertext, &len, reinterpret_cast<uint8_t *>(const_cast<void *>(plaintext)),
                                 plaintext_len))) {
      raise_crypto_error_(ERR_get_error());
    }

    ciphertext_len = len;
    if (!(rc = EVP_EncryptFinal_ex(ctx, ciphertext + len, &len))) {
      raise_crypto_error_(ERR_get_error());
    }

    ciphertext_len += len;
    EVP_CIPHER_CTX_free(ctx);
    return {std::shared_ptr<void>(ciphertext, [](const auto &data) -> void { std::free(data); }), ciphertext_len};
  }

  std::pair<std::shared_ptr<void>, int32_t> decrypt(const void *const ciphertext, size_t ciphertext_len) const {
    EVP_CIPHER_CTX *ctx;
    const EVP_CIPHER *cph;
    int32_t len, rc, plaintext_len;

    if constexpr (bits == 128) {
      cph = EVP_aes_128_cbc();
    } else if constexpr (bits == 192) {
      cph = EVP_aes_192_cbc();
    } else if constexpr (bits == 256) {
      cph = EVP_aes_256_cbc();
    }

    uint8_t *plaintext = reinterpret_cast<uint8_t *>(std::calloc(ciphertext_len, sizeof(uint8_t)));
    if (!(ctx = EVP_CIPHER_CTX_new())) {
      raise_crypto_error_(ERR_get_error());
    }

    if (!(rc = EVP_DecryptInit_ex(ctx, cph, nullptr, reinterpret_cast<const uint8_t *>(key_), nullptr))) {
      raise_crypto_error_(ERR_get_error());
    }

    if (!(rc = EVP_DecryptUpdate(ctx, plaintext, &len, reinterpret_cast<uint8_t *>(const_cast<void *>(ciphertext)),
                                 ciphertext_len))) {
      raise_crypto_error_(ERR_get_error());
    }

    plaintext_len = len;
    if (!(rc = EVP_DecryptFinal_ex(ctx, plaintext + len, &len))) {
      raise_crypto_error_(ERR_get_error());
    }

    plaintext_len += len;
    EVP_CIPHER_CTX_free(ctx);
    return {std::shared_ptr<void>(plaintext, [](const auto &data) -> void { std::free(data); }), plaintext_len};
  }

private:
  const char (&key_)[(bits / 8u) + 1u];
  std::string ossl_err_as_string_(uint64_t crypto_errno) const {
    char err_buf[512u];
    return ERR_error_string(crypto_errno, err_buf);
  }

  void raise_crypto_error_(uint64_t crypto_errno) const {
    throw std::runtime_error((boost::format("Crypto error : %1%:%2%, (%3:%4%)\r\n") % crypto_errno %
                              ossl_err_as_string_(crypto_errno).c_str() % __FILE__ % __LINE__)
                                 .str());
  }
};

#endif /* AES_HPP */
