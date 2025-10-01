#include "sd_mmc_card.h"

#ifdef SDMMC_USE_SDMMC
#ifdef USE_ESP_IDF

#include <cerrno>
#include <algorithm>
#include "math.h"
#include "esphome/core/log.h"

extern "C" {
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
}

#ifndef VFS_FAT_MOUNT_DEFAULT_CONFIG
#define VFS_FAT_MOUNT_DEFAULT_CONFIG() \
    { \
        .format_if_mount_failed = false, \
        .max_files = 5, \
        .allocation_unit_size = 0, \
        .disk_status_check_enable = false, \
    }
#endif

int constexpr SD_OCR_SDHC_CAP = (1 << 30);

namespace esphome {
namespace sd_mmc_card {

static constexpr size_t FILE_PATH_MAX = ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN;
static const char *TAG = "sd_mmc_card";
static const std::string MOUNT_POINT("/sdcard");

std::string build_path(const char *path) { return MOUNT_POINT + path; }

#ifdef USE_SENSOR
FileSizeSensor::FileSizeSensor(sensor::Sensor *sensor, std::string const &path) : sensor(sensor), path(path) {}
#endif

void SdMmc::loop() {}

void SdMmc::dump_config() {
  ESP_LOGCONFIG(TAG, "SD MMC Component");
  ESP_LOGCONFIG(TAG, "  Mode 1 bit: %s", TRUEFALSE(this->mode_1bit_));
  ESP_LOGCONFIG(TAG, "  Slot: %d", this->slot_);
  ESP_LOGCONFIG(TAG, "  CLK Pin: %d", this->clk_pin_);
  ESP_LOGCONFIG(TAG, "  CMD Pin: %d", this->cmd_pin_);
  ESP_LOGCONFIG(TAG, "  DATA0 Pin: %d", this->data0_pin_);
  if (!this->mode_1bit_) {
    ESP_LOGCONFIG(TAG, "  DATA1 Pin: %d", this->data1_pin_);
    ESP_LOGCONFIG(TAG, "  DATA2 Pin: %d", this->data2_pin_);
    ESP_LOGCONFIG(TAG, "  DATA3 Pin: %d", this->data3_pin_);
  }
  if (this->power_ctrl_pin_ != nullptr) {
    LOG_PIN("  Power Ctrl Pin: ", this->power_ctrl_pin_);
  }

#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Used space", this->used_space_sensor_);
  LOG_SENSOR("  ", "Total space", this->total_space_sensor_);
  LOG_SENSOR("  ", "Free space", this->free_space_sensor_);
  for (auto &sensor : this->file_size_sensors_) {
    if (sensor.sensor != nullptr)
      LOG_SENSOR("  ", "File size", sensor.sensor);
  }
#endif

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Setup failed : %s", SdMmc::error_code_to_string(this->init_error_).c_str());
    return;
  }
}

std::string SdMmc::error_code_to_string(SdMmc::ErrorCode code) {
  switch (code) {
    case ErrorCode::ERR_PIN_SETUP:
      return "Failed to set pins";
    case ErrorCode::ERR_MOUNT:
      return "Failed to mount card";
    case ErrorCode::ERR_NO_CARD:
      return "No card found";
    default:
      return "Unknown error";
  }
}

#ifdef USE_SENSOR
void SdMmc::add_file_size_sensor(sensor::Sensor *sensor, std::string const &path) {
  this->file_size_sensors_.emplace_back(sensor, path);
}
#endif

void SdMmc::setup() {
  if (this->power_ctrl_pin_ != nullptr) {
    this->power_ctrl_pin_->setup();
    this->power_ctrl_pin_->digital_write(true);
  }

  esp_vfs_fat_sdmmc_mount_config_t mount_config = VFS_FAT_MOUNT_DEFAULT_CONFIG();
  mount_config.format_if_mount_failed = false;
  mount_config.max_files = 5;
  mount_config.allocation_unit_size = 16 * 1024;
  mount_config.disk_status_check_enable = false;

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.slot = this->slot_;

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.clk = static_cast<gpio_num_t>(this->clk_pin_);
  slot_config.cmd = static_cast<gpio_num_t>(this->cmd_pin_);
  slot_config.d0 = static_cast<gpio_num_t>(this->data0_pin_);

  if (this->mode_1bit_) {
    slot_config.width = 1;
  } else {
    slot_config.width = 4;
    slot_config.d1 = static_cast<gpio_num_t>(this->data1_pin_);
    slot_config.d2 = static_cast<gpio_num_t>(this->data2_pin_);
    slot_config.d3 = static_cast<gpio_num_t>(this->data3_pin_);
  }

  ESP_LOGV(TAG, "Mounting SD card");
  esp_err_t mount_error = esp_vfs_fat_sdmmc_mount(MOUNT_POINT.c_str(), &host, &slot_config, &mount_config, &this->card_);

  if (mount_error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(mount_error));
    switch (mount_error) {
      case ESP_FAIL:
      case ESP_ERR_INVALID_CRC:
        this->init_error_ = ErrorCode::ERR_MOUNT;
        break;
      case ESP_ERR_TIMEOUT:
      default:
        this->init_error_ = ErrorCode::ERR_NO_CARD;
    }
    mark_failed();
    return;
  }

  ESP_LOGI(TAG, "SD card mounted successfully");
  update_sensors();
}

File SdMmc::open(const char *path, const char *mode) {
  const std::string &absolute_path = build_path(path);
  ESP_LOGVV(TAG, "Opening file %s (absolute: %s) %s mode", path, absolute_path.c_str(), mode);
  auto file = fopen(absolute_path.c_str(), mode);
  size_t file_size = 0;
  if (file) {
    struct stat info;
    auto fd = fileno(file);
    ESP_LOGVV(TAG, "Calling fstat(%i)", fd);
    if (fstat(fd, &info) < 0) {
      ESP_LOGE(TAG, "Failed to fstat file %s (absolute: %s): %s", path, absolute_path.c_str(), strerror(errno));
    } else {
      file_size = info.st_size;
    }
  }
  return File(file, file_size);
}

void SdMmc::write_file(const char *path, const uint8_t *buffer, size_t len, const char *mode) {
  std::string absolute_path = build_path(path);
  FILE *file = NULL;
  file = fopen(absolute_path.c_str(), mode);
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open file for writing");
    return;
  }
  bool ok = fwrite(buffer, 1, len, file);
  if (!ok) {
    ESP_LOGE(TAG, "Failed to write to file");
  }
  fclose(file);
  this->update_sensors();
}

bool SdMmc::create_directory(const char *path) {
  ESP_LOGV(TAG, "Create directory: %s", path);
  std::string absolute_path = build_path(path);
  if (mkdir(absolute_path.c_str(), 0777) < 0) {
    ESP_LOGE(TAG, "Failed to create a new directory: %s", strerror(errno));
    return false;
  }
  this->update_sensors();
  return true;
}

bool SdMmc::remove_directory(const char *path) {
  ESP_LOGV(TAG, "Remove directory: %s", path);
  if (!this->is_directory(path)) {
    ESP_LOGE(TAG, "Not a directory");
    return false;
  }
  std::string absolute_path = build_path(path);
  if (remove(absolute_path.c_str()) != 0) {
    ESP_LOGE(TAG, "Failed to remove directory: %s", strerror(errno));
    return false;
  }
  this->update_sensors();
  return true;
}

bool SdMmc::delete_file(const char *path) {
  ESP_LOGV(TAG, "Delete File: %s", path);
  if (this->is_directory(path)) {
    ESP_LOGE(TAG, "Not a file");
    return false;
  }
  std::string absolute_path = build_path(path);
  if (remove(absolute_path.c_str()) != 0) {
    ESP_LOGE(TAG, "Failed to remove file: %s", strerror(errno));
    return false;
  }
  this->update_sensors();
  return true;
}

std::vector<uint8_t> SdMmc::read_file(char const *path) {
  ESP_LOGV(TAG, "Read File: %s", path);

  std::string absolute_path = build_path(path);
  FILE *file = nullptr;
  file = fopen(absolute_path.c_str(), "rb");
  if (file == nullptr) {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return std::vector<uint8_t>();
  }

  std::vector<uint8_t> res;
  size_t fileSize = this->file_size(path);
  res.resize(fileSize);
  size_t len = fread(res.data(), 1, fileSize, file);
  fclose(file);
  if (len == 0) {
    if (ferror(file)) {
      ESP_LOGE(TAG, "Failed to read file: %s", strerror(errno));
      return std::vector<uint8_t>();
    }
  }

  return res;
}

std::vector<FileInfo> &SdMmc::list_directory_file_info_rec(const char *path, uint8_t depth,
                                                           std::vector<FileInfo> &list) {
  ESP_LOGV(TAG, "Listing directory file info: %s\n", path);
  std::string absolute_path = build_path(path);
  DIR *dir = opendir(absolute_path.c_str());
  if (!dir) {
    ESP_LOGE(TAG, "Failed to open directory: %s", strerror(errno));
    return list;
  }
  char entry_absolute_path[FILE_PATH_MAX];
  char entry_path[FILE_PATH_MAX];
  const size_t dirpath_len = MOUNT_POINT.size();
  size_t entry_path_len = strlen(path);
  strlcpy(entry_path, path, sizeof(entry_path));
  strlcpy(entry_path + entry_path_len, "/", sizeof(entry_path) - entry_path_len);
  entry_path_len = strlen(entry_path);

  strlcpy(entry_absolute_path, MOUNT_POINT.c_str(), sizeof(entry_absolute_path));
  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    size_t file_size = 0;
    strlcpy(entry_path + entry_path_len, entry->d_name, sizeof(entry_path) - entry_path_len);
    strlcpy(entry_absolute_path + dirpath_len, entry_path, sizeof(entry_absolute_path) - dirpath_len);
    if (entry->d_type != DT_DIR) {
      struct stat info;
      if (stat(entry_absolute_path, &info) < 0) {
        ESP_LOGE(TAG, "Failed to stat file: %s '%s' %s", strerror(errno), entry->d_name, entry_absolute_path);
      } else {
        file_size = info.st_size;
      }
    }
    list.emplace_back(entry_path, file_size, entry->d_type == DT_DIR);
    if (entry->d_type == DT_DIR && depth)
      this->list_directory_file_info_rec(entry_absolute_path, depth - 1, list);
  }
  closedir(dir);
  return list;
}

bool SdMmc::is_directory(const char *path) {
  std::string absolute_path = build_path(path);
  DIR *dir = opendir(absolute_path.c_str());
  if (dir) {
    closedir(dir);
    return true;
  }
  return false;
}

size_t SdMmc::file_size(const char *path) {
  std::string absolute_path = build_path(path);
  struct stat info;
  if (stat(absolute_path.c_str(), &info) < 0) {
    ESP_LOGE(TAG, "Failed to stat file %s (absolute: %s): %s", path, absolute_path.c_str(), strerror(errno));
    errno = 0;
    return -1;
  }
  return info.st_size;
}

std::string SdMmc::sd_card_type() const {
  if (this->card_->is_sdio) {
    return "SDIO";
  } else if (this->card_->is_mmc) {
    return "MMC";
  } else {
    return (this->card_->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC";
  }
  return "UNKNOWN";
}

void SdMmc::update_sensors() {
#ifdef USE_SENSOR
  if (this->card_ == nullptr)
    return;

  FATFS *fs;
  DWORD fre_clust, fre_sect, tot_sect;
  uint64_t total_bytes = -1, free_bytes = -1, used_bytes = -1;
  auto res = f_getfree(MOUNT_POINT.c_str(), &fre_clust, &fs);
  if (!res) {
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;

    total_bytes = static_cast<uint64_t>(tot_sect) * FF_SS_SDCARD;
    free_bytes = static_cast<uint64_t>(fre_sect) * FF_SS_SDCARD;
    used_bytes = total_bytes - free_bytes;
  }

  if (this->used_space_sensor_ != nullptr)
    this->used_space_sensor_->publish_state(used_bytes);
  if (this->total_space_sensor_ != nullptr)
    this->total_space_sensor_->publish_state(total_bytes);
  if (this->free_space_sensor_ != nullptr)
    this->free_space_sensor_->publish_state(free_bytes);

  for (auto &sensor : this->file_size_sensors_) {
    if (sensor.sensor != nullptr)
      sensor.sensor->publish_state(this->file_size(sensor.path.c_str()));
  }
#endif
}

void SdMmc::set_clk_pin(uint8_t pin) { this->clk_pin_ = pin; }
void SdMmc::set_cmd_pin(uint8_t pin) { this->cmd_pin_ = pin; }
void SdMmc::set_data0_pin(uint8_t pin) { this->data0_pin_ = pin; }
void SdMmc::set_data1_pin(uint8_t pin) { this->data1_pin_ = pin; }
void SdMmc::set_data2_pin(uint8_t pin) { this->data2_pin_ = pin; }
void SdMmc::set_data3_pin(uint8_t pin) { this->data3_pin_ = pin; }
void SdMmc::set_mode_1bit(bool b) { this->mode_1bit_ = b; }
void SdMmc::set_power_ctrl_pin(GPIOPin *pin) { this->power_ctrl_pin_ = pin; }

}  // namespace sd_mmc_card
}  // namespace esphome

#endif  // USE_ESP_IDF
#endif  // SDMMC_USE_SDMMC
