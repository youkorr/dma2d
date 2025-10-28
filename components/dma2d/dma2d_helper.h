#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include <driver/dma2d.h>
#include <esp_cache.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

namespace esphome {
namespace dma2d_helper {

// ===== Constantes =====

static const size_t ALIGNMENT = 64;  // Alignement recommandé pour DMA
static const uint32_t DEFAULT_TIMEOUT_MS = 5000;

// ===== Types et Enums =====

enum class PixelFormat : uint8_t {
  GRAYSCALE = 0,
  RGB565 = 1,
  RGB888 = 2,
  ARGB8888 = 3,
  YUV422 = 4
};

enum class BlendMode : uint8_t {
  ALPHA = 0,
  ADDITIVE = 1,
  MULTIPLY = 2,
  OVERLAY = 3
};

enum class Rotation : uint8_t {
  NONE = 0,
  CW_90 = 1,
  CW_180 = 2,
  CW_270 = 3,
  FLIP_H = 4,
  FLIP_V = 5
};

// ===== Structures =====

struct Stats {
  uint32_t operations_count = 0;
  uint32_t bytes_transferred = 0;
  uint32_t errors_count = 0;
  uint32_t avg_speed_mbps = 0;
};

// ===== Callback =====

using DMA2DCallback = std::function<void(esp_err_t)>;

// ===== Classe principale DMA2DManager =====

class DMA2DManager {
 public:
  // Singleton
  static DMA2DManager& instance();
  
  // Pas de copie
  DMA2DManager(const DMA2DManager&) = delete;
  DMA2DManager& operator=(const DMA2DManager&) = delete;
  
  // Initialisation et destruction
  esp_err_t init();
  esp_err_t deinit();
  
  // Vérification de l'état
  bool is_initialized() const { return this->initialized_; }
  
  // ===== Opérations synchrones =====
  
  // Copie mémoire accélérée
  esp_err_t memcpy_sync(void *dst, const void *src, size_t size);
  
  // Copie 2D avec stride
  esp_err_t memcpy_2d_sync(void *dst, uint32_t dst_stride,
                           const void *src, uint32_t src_stride,
                           uint32_t width, uint32_t height,
                           uint8_t pixel_size);
  
  // Remplissage
  esp_err_t fill_sync(void *dst, uint32_t value, size_t size);
  
  // Remplissage rectangle
  esp_err_t fill_rect_sync(void *dst, uint32_t dst_stride,
                           uint32_t width, uint32_t height,
                           uint32_t color, PixelFormat pixel_format);
  
  // Conversion de format
  esp_err_t convert_format_sync(void *dst, const void *src,
                                uint32_t width, uint32_t height,
                                PixelFormat src_format,
                                PixelFormat dst_format);
  
  // Blending alpha
  esp_err_t blend_sync(void *dst,
                       const void *background,
                       const void *foreground,
                       uint32_t width, uint32_t height,
                       uint8_t alpha,
                       BlendMode mode,
                       PixelFormat pixel_format);
  
  // Rotation
  esp_err_t rotate_sync(void *dst, const void *src,
                        uint32_t width, uint32_t height,
                        Rotation rotation,
                        PixelFormat pixel_format);
  
  // ===== Opérations asynchrones =====
  
  esp_err_t memcpy_async(void *dst, const void *src, size_t size,
                         DMA2DCallback callback = nullptr);
  
  esp_err_t wait_done(uint32_t timeout_ms = 0);
  
  // ===== Statistiques =====
  
  const Stats& get_stats() const { return this->stats_; }
  void reset_stats() { this->stats_ = Stats{}; }
  uint32_t get_last_transfer_time_us() const { return this->last_transfer_time_us_; }
  
  // ===== Utilitaires statiques =====
  
  static bool is_aligned(const void *addr);
  static uint8_t get_pixel_size(PixelFormat format);
  
 protected:
  DMA2DManager();
  ~DMA2DManager();
  
  // Méthodes internes
  esp_err_t wait_transaction_done_(uint32_t timeout_ms);
  dma2d_pixel_format_t to_dma2d_format_(PixelFormat format);
  
  // Callback ISR
  static bool IRAM_ATTR dma2d_isr_callback(dma2d_channel_handle_t channel,
                                           const dma2d_event_data_t *event,
                                           void *user_data);
  
 private:
  bool initialized_ = false;
  bool transaction_done_ = true;
  
  dma2d_channel_handle_t tx_channel_ = nullptr;
  dma2d_channel_handle_t rx_channel_ = nullptr;
  
  Stats stats_;
  uint32_t last_transfer_time_us_ = 0;
  
  DMA2DCallback current_callback_;
};

} // namespace dma2d_helper
} // namespace esphome

#endif // USE_ESP32_VARIANT_ESP32P4
