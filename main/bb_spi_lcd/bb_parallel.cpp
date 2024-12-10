//
// Parallel LCD support for bb_spi_lcd
//


#include "bb_spi_lcd.h"

static uint8_t u8BW, u8WR, u8RD, u8DC, u8CS, u8CMD;
//#define USE_ESP32_GPIO

#if __has_include (<esp_lcd_panel_io.h>)
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/gpio.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <driver/dedic_gpio.h>
#include <hal/gpio_ll.h>
#include <hal/lcd_hal.h>
//#include <soc/lcd_cam_reg.h>
//#include <soc/lcd_cam_struct.h>
#include <hal/lcd_types.h>
//extern DMA_ATTR uint8_t *ucTXBuf;
extern int bSetPosition;
extern volatile bool transfer_is_done;

uint32_t u32IOMask, u32IOMask2, u32IOLookup[256], u32IOLookup2[256]; // for old ESP32
uint8_t *_data_pins;

void spilcdParallelData(uint8_t *pData, int iLen);
static bool s3_notify_dma_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) // @suppress("Type cannot be resolved")
{
//    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
//    lv_disp_flush_ready(disp_driver);
    transfer_is_done = true;
    return false;
}
// from esp-idf/components/esp_lcd/src/esp_lcd_panel_io_i80.c
esp_lcd_i80_bus_handle_t i80_bus = NULL; // @suppress("Type cannot be resolved")
esp_lcd_panel_io_handle_t io_handle = NULL; // @suppress("Type cannot be resolved")
struct esp_lcd_i80_bus_t {
    int bus_id;            // Bus ID, index from 0
    portMUX_TYPE spinlock; // spinlock used to protect i80 bus members(hal, device_list, cur_trans)
    lcd_hal_context_t hal; // Hal object
    size_t bus_width;      // Number of data lines
    intr_handle_t intr;    // LCD peripheral interrupt handle
    void* pm_lock; // Power management lock
    size_t num_dma_nodes;  // Number of DMA descriptors
    uint8_t *format_buffer;  // The driver allocates an internal buffer for DMA to do data format transformer
    size_t resolution_hz;    // LCD_CLK resolution, determined by selected clock source
    gdma_channel_handle_t dma_chan; // DMA channel handle
};
#define MAX_TX_SIZE 4096

//static void _gpio_pin_init(int pin)
//{
//  if (pin >= 0)
//  {
//    gpio_pad_select_gpio(pin);
   // gpio_hi(pin);
//    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
//  }
//}

#endif // has lcd panel include

#ifdef USE_ESP32_GPIO
int bundleA_gpios[8];
gpio_config_t io_conf = {
    .mode = GPIO_MODE_OUTPUT,
};
dedic_gpio_bundle_handle_t bundleA = NULL;
dedic_gpio_bundle_config_t bundleA_config = {
    .gpio_array = bundleA_gpios,
    .array_size = 8, // debug
    .flags = {
        .out_en = 1,
    },
};
#endif // USE_ESP32_GPIO

volatile lcd_cam_dev_t* _dev;
esp_lcd_i80_bus_handle_t _i80_bus = nullptr; // @suppress("Type cannot be resolved")

#ifdef USE_ESP32_GPIO
static void esp32_gpio_clear(int8_t pin)
{
    if (pin < 32) {
        GPIO.out_w1tc = ((uint32_t)1 << pin);
    } else {
        GPIO.out1_w1tc.val = ((uint32_t)1 << (pin-32));
    }
}
static void esp32_gpio_set(int8_t pin)
{
    if (pin < 32) {
        GPIO.out_w1ts = ((uint32_t)1 << pin);
    } else {
        GPIO.out1_w1ts.val = ((uint32_t)1 << (pin-32));
    }
}
#endif // USE_ESP32_GPIO



void ParallelReset(void) {

} /* ParallelReset() */

void ParallelDataWrite(uint8_t *pData, int len, int iMode)
{


    uint32_t c;
    uint32_t u32Data, u32WR, u32 = REG_READ(GPIO_OUT_REG) & ~u32IOMask; // @suppress("Symbol is not resolved")
    uint32_t u32Data2, u32_2 = REG_READ(GPIO_OUT1_REG) & ~u32IOMask2; // @suppress("Symbol is not resolved")
//    if (iMode == MODE_COMMAND) {
//        Serial.printf("cmd, len=%d\n", len);
//    } else {
//        Serial.printf("data, len=%d\n", len);
//    }
//        digitalWrite(u8CS, LOW); // activate CS
//        digitalWrite(u8DC, iMode == MODE_DATA); // DC
        u32WR = 1 << u8WR;
        u32 &= ~u32WR; // Write low for first half of operation
        if (u8CS < 32) {
           u32 &= ~(1 << u8CS);
        }
        if (iMode == MODE_DATA)
           u32 |= (1 << u8DC);
        else
           u32 &= ~(1 << u8DC);
        for (int i=0; i<len; i++) {
            c = pData[i];
            gpio_set_level((gpio_num_t)u8WR, 0);
#ifdef BRUTE_FORCE
            for (int j=0; j<8; j++) {
                digitalWrite(_data_pins[j], (c & (1<<j));
            }
            digitalWrite(u8WR, 1);
#else
            u32Data = u32 | u32IOLookup[c];
            u32Data2 = u32_2 | u32IOLookup2[c];
            REG_WRITE(GPIO_OUT_REG, u32Data); // @suppress("Symbol is not resolved")
            REG_WRITE(GPIO_OUT1_REG, u32Data2); // @suppress("Symbol is not resolved")
            REG_WRITE(GPIO_OUT_REG, u32Data | u32WR); // toggle WR high to latch data // @suppress("Symbol is not resolved")
#endif // BRUTE_FORCE
        } // for i
        gpio_set_level((gpio_num_t)u8CS, 1);
        return;


#ifdef FUTURE
    uint8_t c, old = pData[0] -1;

        esp32_gpio_clear(u8CS); // activate CS (IO33)
        if (iMode == MODE_COMMAND)
            esp32_gpio_clear(u8DC); // clear DC
        else
            esp32_gpio_set(u8DC); // set DC for data mode
        for (int i=0; i<len; i++) {
            c = pData[i];
            esp32_gpio_clear(u8WR); // WR low
            if (c != old) {
//            GPIO.out_w1tc = u32BitMask; // clear our 8+1 bits
//            GPIO.out_w1ts = u32TransBits[c]; // set the current byte
                dedic_gpio_bundle_write(bundleA, 0xff, c);
                old = c;
             }
            esp32_gpio_set(u8WR); // toggle WR high to latch data
        } // for i
        esp32_gpio_set(u8CS); // (IO33) deactivate CS
#endif // FUTURE

} /* ParallelDataWrite() */
void spilcdParallelCMDParams(uint8_t ucCMD, uint8_t *pParams, int iLen)
{

#ifdef USE_ESP32_GPIO
    esp32_gpio_clear(u8DC); // clear DC
    spilcdParallelData(&ucCMD, 1);
    esp32_gpio_set(u8DC);
    if (iLen) {
        spilcdParallelData(pParams, iLen);
    }
#else
    while (!transfer_is_done) {
//        delayMicroseconds(1);
    }
    esp_lcd_panel_io_tx_param(io_handle, ucCMD, pParams, iLen); // @suppress("Invalid arguments")
    u8CMD = 0x2c; // memory restart
#endif // USE_ESP32_GPIO

} /* spilcdParallelCMDParams() */

void spilcdParallelData(uint8_t *pData, int iLen)
{

#ifdef USE_ESP32_GPIO
    uint8_t c, old = pData[0] -1;

        esp32_gpio_clear(u8CS); // activate CS (IO33)
//        if (iMode == MODE_COMMAND)
//            esp32_gpio_clear(u8DC); // clear DC
//        else
//            esp32_gpio_set(u8DC); // set DC for data mode
        for (int i=0; i<iLen; i++) {
            c = pData[i];
            esp32_gpio_clear(u8WR); // WR low
            if (c != old) {
//            GPIO.out_w1tc = u32BitMask; // clear our 8+1 bits
//            GPIO.out_w1ts = u32TransBits[c]; // set the current byte
                dedic_gpio_bundle_write(bundleA, 0xff, c);
                old = c;
             }
            esp32_gpio_set(u8WR); // toggle WR high to latch data
        } // for i
        esp32_gpio_set(u8CS); // (IO33) deactivate CS
#else
    int iSize;
    while (iLen) {
        while (!transfer_is_done) {
           // delayMicroseconds(1);
        }
        transfer_is_done = false; // since we're not using a ping-pong buffer scheme
        iSize = iLen;
        if (iSize > MAX_TX_SIZE) iSize = MAX_TX_SIZE;
        esp_lcd_panel_io_tx_color(io_handle, u8CMD, pData, iSize); // @suppress("Invalid arguments")
        u8CMD = 0x3c; // memory continue;
        iLen -= iSize;
        pData += iSize;
    }
#endif // USE_ESP32_GPIO
} /* spilcdParallelData() */
