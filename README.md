# esphome_sd_card

A collection of ESPHome components focus on operating a SD card.

To use any of these components in _your_ ESPHome device, check out the documentation for adding [external components](https://esphome.io/components/external_components#git).

## Components

### [sd_mmc_card](components/sd_mmc_card/README.md) 

The main component, allow reading and writing to the SD card. It also provide some sensors and other utilities to manipulate the card.

basic configuration:

SDMMC mode
```yaml
sd_mmc_card:
  id: sd_mmc_card
  mode_1bit: false
  clk_pin: GPIO14
  cmd_pin: GPIO15
  data0_pin: GPIO2
  data1_pin: GPIO4
  data2_pin: GPIO12
  data3_pin: GPIO13
```

SPI mode
```yaml
spi:
  - mosi_pin: GPIO18
    miso_pin: GPIO20
    clk_pin: GPIO19
sd_mmc_card:
  id: sd_spi_card
  type: sd_spi
  data3_pin: GPIO23
```

### [sd_file_server](components/sd_file_server/README.md)

A simple web page to navigate the card content and upload/download/delete files.

basic configuration:
```yaml
sd_file_server:
  id: file_server
  url_prefix: file
  root_path: "/"
  enable_deletion: true
  enable_download: true
  enable_upload: true
```

### Notes

SD MMC is only supported by ESP32 and ESP32-S3 board. For ESP32-C6 please use SPI mode. SPI mode supported in esp-idf framework. Feel free to contibute Arduino support.

#### Arduino Framework

[sd_mmc_card](components/sd_mmc_card/README.md) does not work entierly with arduino framework version prior to ```2.0.7```.
The issue as been fix by the pull request [espressif/arduino-esp32/#7646](https://github.com/espressif/arduino-esp32/pull/7646)

The recommended version by esphome is ```2.0.5```

```yaml
esp32:
  board: esp32dev
  framework:
    type: arduino
    version: latest
```

#### ESP-IDF Framework

By default long file name are not enabled, to change this behaviour ```CONFIG_FATFS_LFN_STACK``` or ```CONFIG_FATFS_LFN_HEAP``` should be set in the framework configuration. See the [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/kconfig.html#config-fatfs-long-filenames) for more detail.

```yaml
esp32:
  board: esp32dev
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_FATFS_LFN_STACK: "y"
```

## Contributors
[<img src="https://github.com/elproko.png" width="30px;" style="border-radius: 50%;" title="elproko"/>](https://github.com/elproko)
[<img src="https://github.com/youkorr.png" width="30px;" style="border-radius: 50%;" title="youkoor"/>](https://github.com/youkorr)
[<img src="https://github.com/asergunov.png" width="30px;" style="border-radius: 50%;" title="youkoor"/>](https://github.com/asergunov)
[<img src="https://github.com/Yax.png" width="30px;" style="border-radius: 50%;" title="Yax"/>](https://github.com/Yax)
[<img src="https://github.com/p1ngb4ck.png" width="30px;" style="border-radius: 50%;" title="p1ngb4ck"/>](https://github.com/p1ngb4ck)
