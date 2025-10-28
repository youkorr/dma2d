import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["esp32"]

dma2d_ns = cg.esphome_ns.namespace("dma2d")
DMA2D = dma2d_ns.class_("DMA2D", cg.Component)

CONF_QUEUE_SIZE = "queue_size"
CONF_ENABLE_CACHE_SYNC = "enable_cache_sync"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DMA2D),
    cv.Optional(CONF_QUEUE_SIZE, default=8): cv.int_range(min=1, max=32),
    cv.Optional(CONF_ENABLE_CACHE_SYNC, default=True): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_queue_size(config[CONF_QUEUE_SIZE]))
    cg.add(var.set_enable_cache_sync(config[CONF_ENABLE_CACHE_SYNC]))
    
    cg.add_define("USE_DMA2D")
    cg.add_define("USE_ESP32_VARIANT_ESP32P4")
    
    cg.add(cg.RawExpression(f'''
        ESP_LOGI("compile", "DMA2D configuration:");
        ESP_LOGI("compile", "  Queue size: {config[CONF_QUEUE_SIZE]}");
        ESP_LOGI("compile", "  Cache sync: {'enabled' if config[CONF_ENABLE_CACHE_SYNC] else 'disabled'}");
    '''))
