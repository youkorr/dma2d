#include "dma2d_helper.h"
#include "esphome/core/log.h"
#include "esp_timer.h"
#include <cstring>

#ifdef USE_ESP32_VARIANT_ESP32P4

namespace esphome {
namespace dma2d_helper {

static const char *TAG = "dma2d_helper";

// ===== Singleton =====

DMA2DManager& DMA2DManager::instance() {
  static DMA2DManager instance;
  return instance;
}

DMA2DManager::DMA2DManager() {
  // Constructor
}

DMA2DManager::~DMA2DManager() {
  if (this->initialized_) {
    this->deinit();
  }
}

// ===== Initialisation =====

esp_err_t DMA2DManager::init() {
  if (this->initialized_) {
    ESP_LOGW(TAG, "DMA2D already initialized");
    return ESP_OK;
  }
  
  ESP_LOGI(TAG, "🚀 Initializing DMA2D accelerator...");
  
  // Configuration du canal DMA2D pour transferts mémoire à mémoire
  dma2d_channel_alloc_config_t channel_config = {};
  channel_config.direction = DMA2D_CHANNEL_DIRECTION_TX;
  
  // Allouer le canal TX (pour les opérations M2M)
  esp_err_t ret = dma2d_new_channel(&channel_config, &this->tx_channel_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to allocate DMA2D TX channel: 0x%x", ret);
    return ret;
  }
  
  // Enregistrer le callback ISR
  dma2d_event_callbacks_t callbacks = {};
  callbacks.on_trans_done = DMA2DManager::dma2d_isr_callback;
  
  ret = dma2d_register_event_callbacks(this->tx_channel_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register DMA2D callbacks: 0x%x", ret);
    dma2d_del_channel(this->tx_channel_);
    this->tx_channel_ = nullptr;
    return ret;
  }
  
  this->initialized_ = true;
  this->transaction_done_ = true;
  this->stats_ = Stats{};
  
  ESP_LOGI(TAG, "✅ DMA2D initialized successfully");
  ESP_LOGI(TAG, "   Hardware acceleration: ENABLED");
  ESP_LOGI(TAG, "   Memory alignment: %zu bytes", ALIGNMENT);
  
  return ESP_OK;
}

esp_err_t DMA2DManager::deinit() {
  if (!this->initialized_) {
    return ESP_OK;
  }
  
  ESP_LOGI(TAG, "Deinitializing DMA2D...");
  
  // Attendre les transactions en cours
  this->wait_done(DEFAULT_TIMEOUT_MS);
  
  // Libérer les canaux
  if (this->tx_channel_) {
    dma2d_del_channel(this->tx_channel_);
    this->tx_channel_ = nullptr;
  }
  if (this->rx_channel_) {
    dma2d_del_channel(this->rx_channel_);
    this->rx_channel_ = nullptr;
  }
  
  this->initialized_ = false;
  
  ESP_LOGI(TAG, "DMA2D deinitialized (ops: %u, errors: %u)",
           this->stats_.operations_count, this->stats_.errors_count);
  
  return ESP_OK;
}

// ===== Callback ISR =====

bool IRAM_ATTR DMA2DManager::dma2d_isr_callback(dma2d_channel_handle_t channel,
                                                 const dma2d_event_data_t *event,
                                                 void *user_data) {
  DMA2DManager *mgr = (DMA2DManager*)user_data;
  
  mgr->transaction_done_ = true;
  
  // Appeler le callback utilisateur si présent
  if (mgr->current_callback_) {
    mgr->current_callback_(ESP_OK);
    mgr->current_callback_ = nullptr;
  }
  
  return false; // Ne pas yield de l'ISR
}

// ===== Opérations synchrones =====

esp_err_t DMA2DManager::memcpy_sync(void *dst, const void *src, size_t size) {
  if (!this->initialized_) {
    ESP_LOGE(TAG, "DMA2D not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (!dst || !src || size == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  
  // Vérifier l'alignement (recommandé mais pas toujours strict)
  if (!is_aligned(dst) || !is_aligned(src)) {
    ESP_LOGV(TAG, "Unaligned addresses detected, performance may be reduced");
  }
  
  // Attendre si une transaction est en cours
  esp_err_t ret = this->wait_transaction_done_(DEFAULT_TIMEOUT_MS);
  if (ret != ESP_OK) {
    return ret;
  }
  
  this->transaction_done_ = false;
  uint64_t start_time = esp_timer_get_time();
  
  // Préparer le descripteur de transaction
  dma2d_trans_config_t trans_config = {};
  trans_config.tx_desc_base = (void*)src;
  trans_config.tx_desc_cnt = (size + ALIGNMENT - 1) / ALIGNMENT;
  trans_config.rx_desc_base = dst;
  trans_config.rx_desc_cnt = trans_config.tx_desc_cnt;
  
  // Lancer le transfert
  ret = dma2d_enqueue(this->tx_channel_, &trans_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DMA2D enqueue failed: 0x%x", ret);
    this->transaction_done_ = true;
    this->stats_.errors_count++;
    return ret;
  }
  
  // Attendre la fin
  ret = this->wait_transaction_done_(DEFAULT_TIMEOUT_MS);
  if (ret != ESP_OK) {
    this->stats_.errors_count++;
    return ret;
  }
  
  // Mettre à jour les stats
  uint64_t elapsed_us = esp_timer_get_time() - start_time;
  this->last_transfer_time_us_ = (uint32_t)elapsed_us;
  this->stats_.operations_count++;
  this->stats_.bytes_transferred += size;
  
  if (elapsed_us > 0) {
    uint32_t speed_mbps = (size * 8) / elapsed_us; // Mbps
    // Moyenne glissante
    this->stats_.avg_speed_mbps = 
      (this->stats_.avg_speed_mbps * 9 + speed_mbps) / 10;
  }
  
  ESP_LOGV(TAG, "DMA2D memcpy: %zu bytes in %u us (%.1f MB/s)",
           size, this->last_transfer_time_us_,
           (float)this->stats_.avg_speed_mbps);
  
  return ESP_OK;
}

esp_err_t DMA2DManager::memcpy_2d_sync(void *dst, uint32_t dst_stride,
                                       const void *src, uint32_t src_stride,
                                       uint32_t width, uint32_t height,
                                       uint8_t pixel_size) {
  if (!this->initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  
  if (!dst || !src || width == 0 || height == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  
  // Si les strides sont identiques et contigus, utiliser memcpy simple
  uint32_t row_size = width * pixel_size;
  if (src_stride == row_size && dst_stride == row_size) {
    return this->memcpy_sync(dst, src, row_size * height);
  }
  
  // Sinon copier ligne par ligne
  const uint8_t *src_row = (const uint8_t*)src;
  uint8_t *dst_row = (uint8_t*)dst;
  
  for (uint32_t y = 0; y < height; y++) {
    esp_err_t ret = this->memcpy_sync(dst_row, src_row, row_size);
    if (ret != ESP_OK) {
      return ret;
    }
    src_row += src_stride;
    dst_row += dst_stride;
  }
  
  return ESP_OK;
}

esp_err_t DMA2DManager::fill_sync(void *dst, uint32_t value, size_t size) {
  if (!this->initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  
  if (!dst || size == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  
  // Pour ESP32-P4, le DMA2D peut faire du fill en mode constant
  // Si non supporté directement, fallback sur memset CPU
  // (à vérifier dans la doc ESP-IDF pour ESP32-P4)
  
  // Fallback temporaire sur memset
  memset(dst, value & 0xFF, size);
  
  this->stats_.operations_count++;
  this->stats_.bytes_transferred += size;
  
  return ESP_OK;
}

esp_err_t DMA2DManager::fill_rect_sync(void *dst, uint32_t dst_stride,
                                       uint32_t width, uint32_t height,
                                       uint32_t color, PixelFormat pixel_format) {
  if (!this->initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  
  uint8_t pixel_size = get_pixel_size(pixel_format);
  uint32_t row_size = width * pixel_size;
  
  // Créer un buffer temporaire d'une ligne remplie
  uint8_t *temp_row = (uint8_t*)heap_caps_malloc(row_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!temp_row) {
    return ESP_ERR_NO_MEM;
  }
  
  // Remplir la ligne temporaire avec la couleur
  if (pixel_size == 2) {
    uint16_t *row16 = (uint16_t*)temp_row;
    for (uint32_t x = 0; x < width; x++) {
      row16[x] = (uint16_t)color;
    }
  } else if (pixel_size == 4) {
    uint32_t *row32 = (uint32_t*)temp_row;
    for (uint32_t x = 0; x < width; x++) {
      row32[x] = color;
    }
  } else {
    memset(temp_row, color & 0xFF, row_size);
  }
  
  // Copier la ligne sur toute la hauteur
  uint8_t *dst_row = (uint8_t*)dst;
  for (uint32_t y = 0; y < height; y++) {
    esp_err_t ret = this->memcpy_sync(dst_row, temp_row, row_size);
    if (ret != ESP_OK) {
      heap_caps_free(temp_row);
      return ret;
    }
    dst_row += dst_stride;
  }
  
  heap_caps_free(temp_row);
  return ESP_OK;
}

esp_err_t DMA2DManager::convert_format_sync(void *dst, const void *src,
                                            uint32_t width, uint32_t height,
                                            PixelFormat src_format,
                                            PixelFormat dst_format) {
  if (!this->initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  
  // Conversion RGB565 → RGB888 (la plus courante)
  if (src_format == PixelFormat::RGB565 && dst_format == PixelFormat::RGB888) {
    const uint16_t *src16 = (const uint16_t*)src;
    uint8_t *dst24 = (uint8_t*)dst;
    
    for (uint32_t i = 0; i < width * height; i++) {
      uint16_t pixel = src16[i];
      uint8_t r = ((pixel >> 11) & 0x1F) << 3;
      uint8_t g = ((pixel >> 5) & 0x3F) << 2;
      uint8_t b = (pixel & 0x1F) << 3;
      
      dst24[i * 3 + 0] = r;
      dst24[i * 3 + 1] = g;
      dst24[i * 3 + 2] = b;
    }
    
    this->stats_.operations_count++;
    return ESP_OK;
  }
  
  // Conversion RGB888 → RGB565
  if (src_format == PixelFormat::RGB888 && dst_format == PixelFormat::RGB565) {
    const uint8_t *src24 = (const uint8_t*)src;
    uint16_t *dst16 = (uint16_t*)dst;
    
    for (uint32_t i = 0; i < width * height; i++) {
      uint8_t r = src24[i * 3 + 0];
      uint8_t g = src24[i * 3 + 1];
      uint8_t b = src24[i * 3 + 2];
      
      dst16[i] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    }
    
    this->stats_.operations_count++;
    return ESP_OK;
  }
  
  ESP_LOGE(TAG, "Format conversion not implemented: %d → %d",
           (int)src_format, (int)dst_format);
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t DMA2DManager::blend_sync(void *dst,
                                   const void *background,
                                   const void *foreground,
                                   uint32_t width, uint32_t height,
                                   uint8_t alpha,
                                   BlendMode mode,
                                   PixelFormat pixel_format) {
  if (!this->initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  
  // Implémentation basique du blending alpha pour RGB565
  if (pixel_format == PixelFormat::RGB565 && mode == BlendMode::ALPHA) {
    const uint16_t *bg = (const uint16_t*)background;
    const uint16_t *fg = (const uint16_t*)foreground;
    uint16_t *out = (uint16_t*)dst;
    
    for (uint32_t i = 0; i < width * height; i++) {
      uint16_t bg_pixel = bg[i];
      uint16_t fg_pixel = fg[i];
      
      // Extraire les composantes
      uint8_t bg_r = (bg_pixel >> 11) & 0x1F;
      uint8_t bg_g = (bg_pixel >> 5) & 0x3F;
      uint8_t bg_b = bg_pixel & 0x1F;
      
      uint8_t fg_r = (fg_pixel >> 11) & 0x1F;
      uint8_t fg_g = (fg_pixel >> 5) & 0x3F;
      uint8_t fg_b = fg_pixel & 0x1F;
      
      // Blending alpha
      uint8_t out_r = (fg_r * alpha + bg_r * (255 - alpha)) / 255;
      uint8_t out_g = (fg_g * alpha + bg_g * (255 - alpha)) / 255;
      uint8_t out_b = (fg_b * alpha + bg_b * (255 - alpha)) / 255;
      
      // Recomposer
      out[i] = ((out_r & 0x1F) << 11) | ((out_g & 0x3F) << 5) | (out_b & 0x1F);
    }
    
    this->stats_.operations_count++;
    return ESP_OK;
  }
  
  ESP_LOGE(TAG, "Blend mode not implemented");
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t DMA2DManager::rotate_sync(void *dst, const void *src,
                                    uint32_t width, uint32_t height,
                                    Rotation rotation,
                                    PixelFormat pixel_format) {
  if (!this->initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  
  if (rotation == Rotation::NONE) {
    size_t size = width * height * get_pixel_size(pixel_format);
    return this->memcpy_sync(dst, src, size);
  }
  
  // Rotation 90° CW pour RGB565
  if (rotation == Rotation::CW_90 && pixel_format == PixelFormat::RGB565) {
    const uint16_t *src16 = (const uint16_t*)src;
    uint16_t *dst16 = (uint16_t*)dst;
    
    for (uint32_t y = 0; y < height; y++) {
      for (uint32_t x = 0; x < width; x++) {
        uint32_t src_idx = y * width + x;
        uint32_t dst_idx = x * height + (height - 1 - y);
        dst16[dst_idx] = src16[src_idx];
      }
    }
    
    this->stats_.operations_count++;
    return ESP_OK;
  }
  
  ESP_LOGE(TAG, "Rotation not implemented for this format/angle");
  return ESP_ERR_NOT_SUPPORTED;
}

// ===== Opérations asynchrones =====

esp_err_t DMA2DManager::memcpy_async(void *dst, const void *src, size_t size,
                                     DMA2DCallback callback) {
  if (!this->initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  
  // Pour l'instant, fallback sur sync
  // TODO: Implémenter le vrai mode async quand ESP-IDF le supporte mieux
  this->current_callback_ = callback;
  esp_err_t ret = this->memcpy_sync(dst, src, size);
  
  if (callback) {
    callback(ret);
    this->current_callback_ = nullptr;
  }
  
  return ret;
}

esp_err_t DMA2DManager::wait_done(uint32_t timeout_ms) {
  return this->wait_transaction_done_(timeout_ms == 0 ? DEFAULT_TIMEOUT_MS : timeout_ms);
}

// ===== Helpers internes =====

esp_err_t DMA2DManager::wait_transaction_done_(uint32_t timeout_ms) {
  if (this->transaction_done_) {
    return ESP_OK;
  }
  
  uint64_t start = esp_timer_get_time();
  uint64_t timeout_us = timeout_ms * 1000ULL;
  
  while (!this->transaction_done_) {
    if (timeout_ms > 0 && (esp_timer_get_time() - start) > timeout_us) {
      ESP_LOGE(TAG, "DMA2D transaction timeout");
      this->stats_.errors_count++;
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(1); // Yield CPU
  }
  
  return ESP_OK;
}

dma2d_pixel_format_t DMA2DManager::to_dma2d_format_(PixelFormat format) {
  switch (format) {
    case PixelFormat::RGB565:
      return DMA2D_PIXEL_FORMAT_RGB565;
    case PixelFormat::RGB888:
      return DMA2D_PIXEL_FORMAT_RGB888;
    case PixelFormat::ARGB8888:
      return DMA2D_PIXEL_FORMAT_ARGB8888;
    default:
      return DMA2D_PIXEL_FORMAT_RGB565;
  }
}

// ===== Utilitaires statiques =====

bool DMA2DManager::is_aligned(const void *addr) {
  return ((uintptr_t)addr % ALIGNMENT) == 0;
}

uint8_t DMA2DManager::get_pixel_size(PixelFormat format) {
  switch (format) {
    case PixelFormat::GRAYSCALE:
      return 1;
    case PixelFormat::RGB565:
    case PixelFormat::YUV422:
      return 2;
    case PixelFormat::RGB888:
      return 3;
    case PixelFormat::ARGB8888:
      return 4;
    default:
      return 2;
  }
}

} // namespace dma2d_helper
} // namespace esphome

#endif // USE_ESP32_VARIANT_ESP32P4
