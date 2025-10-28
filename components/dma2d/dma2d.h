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
namespace dma2d {

// ===== Forward declarations =====

class DMA2D;

// ===== Types et Enums =====

enum class PixelFormat : uint8_t {
  RGB565 = 0,
  RGB888 = 1,
  ARGB8888 = 2,
  GRAYSCALE = 3
};

enum class TransferMode : uint8_t {
  MEMCPY = 0,
  CONVERT = 1,
  FILL = 2,
  BLEND = 3
};

// ===== Callbacks =====

using dma2d_callback_t = std::function<void(void*)>;

// ===== Structures =====

struct DMA2DTransfer {
  TransferMode mode;
  
  // Buffers
  const void *src_buffer = nullptr;
  void *dst_buffer = nullptr;
  
  // Dimensions source
  uint32_t src_width = 0;
  uint32_t src_height = 0;
  PixelFormat src_format = PixelFormat::RGB565;
  
  // Dimensions destination
  uint32_t dst_width = 0;
  uint32_t dst_height = 0;
  PixelFormat dst_format = PixelFormat::RGB565;
  
  // Paramètres additionnels
  uint32_t fill_color = 0;
  uint8_t alpha = 255;
  
  // Callback et données utilisateur
  dma2d_callback_t callback;
  void *user_data = nullptr;
  
  // ID de transfert
  uint32_t transfer_id = 0;
};

struct DMA2DStats {
  uint32_t total_transfers = 0;
  uint32_t bytes_transferred = 0;
  uint32_t avg_transfer_time_us = 0;
  uint32_t min_transfer_time_us = UINT32_MAX;
  uint32_t max_transfer_time_us = 0;
  uint32_t queue_overflows = 0;
  uint32_t cache_sync_count = 0;
};

// ===== Classe principale DMA2D =====

class DMA2D : public Component {
 public:
  DMA2D() = default;
  ~DMA2D() override;
  
  // Component lifecycle
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  
  // Configuration
  void set_queue_size(uint8_t size) { this->queue_size_ = size; }
  void set_enable_cache_sync(bool enable) { this->enable_cache_sync_ = enable; }
  
  // État
  bool is_initialized() const { return this->initialized_; }
  
  // ===== API Publique =====
  
  // Copie mémoire asynchrone/synchrone
  bool memcpy_async(void *dst, const void *src, size_t size,
                    bool blocking = false,
                    dma2d_callback_t callback = nullptr,
                    void *user_data = nullptr);
  
  // Copie rapide de buffers RGB565
  bool copy_rgb565(void *dst, const void *src, 
                   uint32_t width, uint32_t height,
                   bool blocking = true);
  
  // Conversion de format
  bool convert_format(void *dst, PixelFormat dst_format,
                     const void *src, PixelFormat src_format,
                     uint32_t width, uint32_t height,
                     bool blocking = true);
  
  // Remplissage
  bool fill(void *dst, uint32_t color,
           uint32_t width, uint32_t height,
           PixelFormat format = PixelFormat::RGB565,
           bool blocking = true);
  
  // Blending alpha
  bool blend(void *dst, const void *src_bg, const void *src_fg,
            uint32_t width, uint32_t height, uint8_t alpha,
            PixelFormat format = PixelFormat::RGB565,
            bool blocking = true);
  
  // Synchronisation
  bool wait_all_transfers(uint32_t timeout_ms = 5000);
  
  // Statistiques
  const DMA2DStats& get_stats() const { return this->stats_; }
  void reset_stats();
  
  // Singleton (optionnel)
  static DMA2D* get_instance() { return instance_; }
  
 protected:
  // Initialisation interne
  bool init_dma2d_();
  
  // Gestion des transferts
  bool queue_transfer_(const DMA2DTransfer &transfer);
  bool execute_transfer_(const DMA2DTransfer &transfer);
  
  // Exécution par type
  bool execute_memcpy_(const DMA2DTransfer &transfer);
  bool execute_convert_(const DMA2DTransfer &transfer);
  bool execute_fill_(const DMA2DTransfer &transfer);
  bool execute_blend_(const DMA2DTransfer &transfer);
  
  // Gestion du cache
  void sync_cache_(const void *addr, size_t size, bool write_back);
  
  // Statistiques
  void update_stats_(const DMA2DTransfer &transfer, uint32_t duration_us);
  
  // Callback ISR
  static bool IRAM_ATTR dma2d_trans_done_callback_(
    dma2d_channel_handle_t channel,
    dma2d_event_data_t *event_data,
    void *user_data);
  
 private:
  bool initialized_ = false;
  uint8_t queue_size_ = 8;
  bool enable_cache_sync_ = true;
  
  // DMA2D handles
  dma2d_pool_handle_t pool_handle_ = nullptr;
  
  // FreeRTOS primitives
  QueueHandle_t transfer_queue_ = nullptr;
  SemaphoreHandle_t transfer_semaphore_ = nullptr;
  
  // Compteurs
  uint32_t next_transfer_id_ = 1;
  volatile uint32_t active_transfers_ = 0;
  
  // Statistiques
  DMA2DStats stats_;
  
  // Singleton
  static DMA2D* instance_;
};

} // namespace dma2d
} // namespace esphome

#endif // USE_ESP32_VARIANT_ESP32P4
