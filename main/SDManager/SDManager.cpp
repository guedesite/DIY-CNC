#include "SDManager.h"
#include <cstring>

const char SDManager::extension[] = ".cnc";
const char* SDManager::TAG = "SDManager";
sdmmc_card_t* SDManager::card = nullptr;

esp_err_t SDManager::initSDCard() {
    esp_err_t ret;


    // Initialize the SD card
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)SD_CS;
    slot_config.host_id = VSPI_HOST;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // Mount the filesystem
    const char mount_point[] = "/sdcard";
    ESP_LOGI(TAG, "Mounting filesystem");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
		.disk_status_check_enable = false
    };

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem. (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Filesystem mounted");
    return ESP_OK;
}

esp_err_t SDManager::writeToFile_u(const char* filename, const uint8_t* data, size_t length) {
    char fullPath[128];
    snprintf(fullPath, sizeof(fullPath), "/sdcard/%s%s", filename, extension);
    FILE* file = fopen(fullPath, "wb");

    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", fullPath);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, length, file);
    fclose(file);

    if (written != length) {
        ESP_LOGE(TAG, "Failed to write all data to file: %s", fullPath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File written: %s", fullPath);
    return ESP_OK;
}

esp_err_t SDManager::readFromFile_u(const char* filename, uint8_t* data, size_t& length) {
    char fullPath[128];
    snprintf(fullPath, sizeof(fullPath), "/sdcard/%s%s", filename, extension);
    FILE* file = fopen(fullPath, "rb");

    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", fullPath);
        return ESP_FAIL;
    }

    fseek(file, 0, SEEK_END);
    length = ftell(file);
    fseek(file, 0, SEEK_SET);

    size_t read = fread(data, 1, length, file);
    fclose(file);

    if (read != length) {
        ESP_LOGE(TAG, "Failed to read all data from file: %s", fullPath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File read: %s", fullPath);
    return ESP_OK;
}


esp_err_t SDManager::writeToFile(const char* filename, const int8_t* data, size_t length) {
    char fullPath[128];
    snprintf(fullPath, sizeof(fullPath), "/sdcard/%s%s", filename, extension);
    FILE* file = fopen(fullPath, "wb");

    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", fullPath);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, length, file);
    fclose(file);

    if (written != length) {
        ESP_LOGE(TAG, "Failed to write all data to file: %s", fullPath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File written: %s", fullPath);
    return ESP_OK;
}

esp_err_t SDManager::readFromFile(const char* filename, int8_t* data, size_t& length) {
    char fullPath[128];
    snprintf(fullPath, sizeof(fullPath), "/sdcard/%s%s", filename, extension);
    FILE* file = fopen(fullPath, "rb");

    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", fullPath);
        return ESP_FAIL;
    }

    fseek(file, 0, SEEK_END);
    length = ftell(file);
    fseek(file, 0, SEEK_SET);

    if(data == nullptr) {
    	return ESP_OK;
    }

    size_t read = fread(data, 1, length, file);
    fclose(file);

    if (read != length) {
        ESP_LOGE(TAG, "Failed to read all data from file: %s", fullPath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File read: %s", fullPath);
    return ESP_OK;
}

esp_err_t SDManager::listFiles(char fileList[][64], size_t& count) {
    DIR* dir = opendir("/sdcard/");
    if (dir == nullptr) {
        ESP_LOGE(TAG, "Failed to open directory");
        return ESP_FAIL;
    }

    struct dirent* entry;
    count = 0;
    while ((entry = readdir(dir)) != nullptr) {
            const char* name = entry->d_name;
            size_t nameLen = strlen(name);
            size_t extLen = strlen(extension);

            if (nameLen >= extLen && strcmp(name + nameLen - extLen, extension) == 0) {
                strncpy(fileList[count], name, 63);
                fileList[count][63] = '\0'; // Ensure null-termination
                count++;
            }
        }
    closedir(dir);

    ESP_LOGI(TAG, "Listed files");
    return ESP_OK;
}

esp_err_t SDManager::deleteFile(const char* filename) {
    char fullPath[128];
    snprintf(fullPath, sizeof(fullPath), "/sdcard/%s%s", filename, extension);
    if (f_unlink(fullPath) != 0) {
        ESP_LOGE(TAG, "Failed to delete file: %s", fullPath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File deleted: %s", fullPath);
    return ESP_OK;
}
