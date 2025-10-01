import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import SdMmc, CONF_SD_MMC_CARD_ID

DEPENDENCIES = ["sd_mmc_card"]

CONF_SD_CARD_TYPE = "sd_card_type"

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_SD_MMC_CARD_ID): cv.use_id(SdMmc),
}

async def to_code(config):
    sd_mmc_component = await cg.get_variable(config[CONF_SD_MMC_CARD_ID])
