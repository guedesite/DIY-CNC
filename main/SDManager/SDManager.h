/*
 * SDManager.h
 *
 *  Created on: 8 juin 2024
 *      Author: guedesite
 */

#ifndef MAIN_SDMANAGER_SDMANAGER_H_
#define MAIN_SDMANAGER_SDMANAGER_H_

#include <dirent.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_err.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#define SD_CS 13

class SDManager {
public:
    static esp_err_t initSDCard();
    static esp_err_t writeToFile_u(const char* filename, const uint8_t* data, size_t length);
    static esp_err_t readFromFile_u(const char* filename, uint8_t* data, size_t& length);

    static esp_err_t writeToFile(const char* filename, const int8_t* data, size_t length);
    static esp_err_t readFromFile(const char* filename, int8_t* data, size_t& length);

    static esp_err_t listFiles(char fileList[][64], size_t& count);
    static esp_err_t deleteFile(const char* filename);

private:
    static const char extension[];
    static const char* TAG;
    static sdmmc_card_t* card;
};

#endif /* MAIN_SDMANAGER_SDMANAGER_H_ */
