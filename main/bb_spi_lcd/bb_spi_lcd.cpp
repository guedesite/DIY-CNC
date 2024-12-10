#include "bb_spi_lcd.h"

#define mySPI SPI

#define ESP32_SPI_HOST VSPI_HOST


#define HAS_DMA
#define SPI_DMA_CHAN SPI_DMA_CH_AUTO

volatile int iCurrentCS;

static spi_device_interface_config_t devcfg;
static spi_bus_config_t buscfg;


static spi_transaction_t trans[2];
static spi_device_handle_t spi;

//static TaskHandle_t xTaskToNotify = NULL;
// ESP32 has enough memory to spare 8K
DMA_ATTR uint8_t ucTXBuf[8192]="";
uint8_t *pDMA0 = ucTXBuf;
uint8_t *pDMA1 = &ucTXBuf[4096]; // 2 ping-pong buffers
uint8_t *pDMA;
static unsigned char ucRXBuf[4096];
//#ifndef ESP32_DMA

#define HIGH 1
#define LOW 0

#define LCD_DELAY 0xff

volatile bool transfer_is_done = true; // Done yet?
void spilcdWaitDMA(void);
void spilcdWriteDataDMA(SPILCD *pLCD, int iLen);

volatile int bSetPosition = 0;

void spilcdParallelData(uint8_t *pData, int iLen);
void spilcdWriteCommand(SPILCD *pLCD, unsigned char);
void spilcdWriteCmdParams(SPILCD *pLCD, uint8_t ucCMD, uint8_t *pParams, int iLen);
void spilcdWriteCommand16(SPILCD *pLCD, uint16_t us);
static void spilcdWriteData8(SPILCD *pLCD, unsigned char c);
static void spilcdWriteData16(SPILCD *pLCD, unsigned short us, int iFlags);
void spilcdSetPosition(SPILCD *pLCD, int x, int y, int w, int h, int iFlags);
int spilcdFill(SPILCD *pLCD, unsigned short usData, int iFlags);

uint8_t u8_22C_Pins[8] = {15,13,12,14,27,25,33,32};
uint8_t u8_WT32_Pins[8] = {9,46,3,8,18,17,16,15};
uint8_t u8D1R32DataPins[8] = {12,13,26,25,17,16,27,14};

const unsigned char ucE0_0[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
const unsigned char ucE1_0[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
const unsigned char ucE0_1[] = {0x1f, 0x1a, 0x18, 0x0a, 0x0f, 0x06, 0x45, 0x87, 0x32, 0x0a, 0x07, 0x02, 0x07, 0x05, 0x00};
const unsigned char ucE1_1[] = {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3a, 0x78, 0x4d, 0x05, 0x18, 0x0d, 0x38, 0x3a, 0x1f};

void delayMicroseconds(uint32_t us) {
    esp_rom_delay_us(us);
}

void spilcdSetTXBuffer(uint8_t *pBuf, int iSize)
{

}

void spilcdSetMode(SPILCD *pLCD, int iMode)
{
	gpio_set_level(pLCD->iDCPin, iMode == MODE_DATA);
	delayMicroseconds(1);
} /* spilcdSetMode() */

// ST7789
const unsigned char uc240x240InitList[] = {
    1, 0x13, // partial mode off
    1, 0x21, // display inversion off
    2, 0x36,0x08,    // memory access 0xc0 for 180 degree flipped
    2, 0x3a,0x55,    // pixel format; 5=RGB565
    3, 0x37,0x00,0x00, //
    6, 0xb2,0x0c,0x0c,0x00,0x33,0x33, // Porch control
    2, 0xb7,0x35,    // gate control
    2, 0xbb,0x1a,    // VCOM
    2, 0xc0,0x2c,    // LCM
    2, 0xc2,0x01,    // VDV & VRH command enable
    2, 0xc3,0x0b,    // VRH set
    2, 0xc4,0x20,    // VDV set
    2, 0xc6,0x0f,    // FR control 2
    3, 0xd0, 0xa4, 0xa1,     // Power control 1
    15, 0xe0, 0x00,0x19,0x1e,0x0a,0x09,0x15,0x3d,0x44,0x51,0x12,0x03,
        0x00,0x3f,0x3f,     // gamma 1
    15, 0xe1, 0x00,0x18,0x1e,0x0a,0x09,0x25,0x3f,0x43,0x52,0x33,0x03,
        0x00,0x3f,0x3f,        // gamma 2
    1, 0x29,    // display on
    0
};

// List of command/parameters to initialize the ili9341 display
const unsigned char uc240InitList[] = {
        4, 0xEF, 0x03, 0x80, 0x02,
        4, 0xCF, 0x00, 0XC1, 0X30,
        5, 0xED, 0x64, 0x03, 0X12, 0X81,
        4, 0xE8, 0x85, 0x00, 0x78,
        6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
        2, 0xF7, 0x20,
        3, 0xEA, 0x00, 0x00,
        2, 0xc0, 0x23, // Power control
        2, 0xc1, 0x10, // Power control
        3, 0xc5, 0x3e, 0x28, // VCM control
        2, 0xc7, 0x86, // VCM control2
        2, 0x36, 0x48, // Memory Access Control
        1, 0x20,        // non inverted
        2, 0x3a, 0x55,
        3, 0xb1, 0x00, 0x18,
        4, 0xb6, 0x08, 0x82, 0x27, // Display Function Control
        2, 0xF2, 0x00, // Gamma Function Disable
        2, 0x26, 0x01, // Gamma curve selected
        16, 0xe0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
                0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
        16, 0xe1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
                0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
        3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
        0
};


void memset16(uint16_t *pDest, uint16_t usPattern, int iCount)
{
    while (iCount--)
        *pDest++ = usPattern;
}

int spilcdIsDMABusy(void)
{
#ifdef HAS_DMA
    return !transfer_is_done;
#endif
    return 0;
}

//
// Send the data by bit-banging the GPIO ports
//
void SPI_BitBang(SPILCD *pLCD, uint8_t *pData, int iLen, int iMode)
{
    // Local variables for quick access
    gpio_num_t iMOSI = pLCD->iMOSIPin;
    gpio_num_t iCLK = pLCD->iCLKPin;
    gpio_num_t iCSPin = pLCD->iCSPin;
    gpio_num_t iDCPin = pLCD->iDCPin;
    uint8_t c, j;

    gpio_set_level(iCSPin, 0);
    if (iMode == MODE_COMMAND) {
        gpio_set_level(iDCPin, 0);
    }

    while (iLen) {
        c = *pData++;
        if (pLCD->iLCDFlags & FLAGS_CS_EACHBYTE) {
            gpio_set_level(iCSPin, 0);
        }
        if (c == 0 || c == 0xff) { // Quicker for all bits equal
            gpio_set_level(iMOSI, c);
            for (j = 0; j < 8; j++) {
                gpio_set_level(iCLK, 1);
                delayMicroseconds(0);
                gpio_set_level(iCLK, 0);
            }
        } else {
            for (j = 0; j < 8; j++) {
                gpio_set_level(iMOSI, (c & 0x80) != 0);
                gpio_set_level(iCLK, 1);
                c <<= 1;
                delayMicroseconds(0);
                gpio_set_level(iCLK, 0);
            }
        }
        if (pLCD->iLCDFlags & FLAGS_CS_EACHBYTE) {
            gpio_set_level(iCSPin, 1);
        }
        iLen--;
    }

    gpio_set_level(iCSPin, 1);
    if (iMode == MODE_COMMAND) {
        gpio_set_level(iDCPin, 1); // Restore to MODE_DATA before leaving
    }
} /* SPI_BitBang() */

static void myspiWrite(SPILCD *pLCD, unsigned char *pBuf, int iLen, int iMode, int iFlags)
{
    if (iLen == 0) return;
// swap DMA buffers
    if (pDMA == pDMA0) {
        pDMA = pDMA1;
    } else {
        pDMA = pDMA0;
    }
    if ((iFlags & DRAW_WITH_DMA) && !pLCD->bUseDMA) {
        iFlags &= ~DRAW_WITH_DMA; // DMA is not being used, so remove the flag
    }

    if (iMode == MODE_DATA && pLCD->pBackBuffer != NULL && !bSetPosition && (iFlags & DRAW_TO_RAM)) // write it to the back buffer
    {
        uint16_t *s, *d;
        int j, iOff, iStrip, iMaxX, iMaxY, i;
        iMaxX = pLCD->iWindowX + pLCD->iWindowCX;
        iMaxY = pLCD->iWindowY + pLCD->iWindowCY;
        iOff = 0;
        i = iLen/2;
        while (i > 0)
        {
            iStrip = iMaxX - pLCD->iCurrentX; // max pixels that can be written in one shot
            if (iStrip > i)
                iStrip = i;
            s = (uint16_t *)&pBuf[iOff];
            d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset];
            if (pLCD->iOffset > pLCD->iMaxOffset || (pLCD->iOffset+iStrip*2) > pLCD->iMaxOffset)
            { // going to write past the end of memory, don't even try
                i = iStrip = 0;
            }
            for (j=0; j<iStrip; j++) // memcpy could be much slower for small runs
            {
                *d++ = *s++;
            }

            pLCD->iOffset += iStrip*2; iOff += iStrip*2;
            i -= iStrip;
            pLCD->iCurrentX += iStrip;
            if (pLCD->iCurrentX >= iMaxX) // need to wrap around to the next line
            {
                pLCD->iCurrentX = pLCD->iWindowX;
                pLCD->iCurrentY++;
                if (pLCD->iCurrentY >= iMaxY)
                    pLCD->iCurrentY = pLCD->iWindowY;
                pLCD->iOffset = (pLCD->iCurrentWidth * 2 * pLCD->iCurrentY) + (pLCD->iCurrentX * 2);
            }
        }
    }
    if (!(iFlags & DRAW_TO_LCD)) {
        return; // don't write it to spi
    }

    //if (pLCD->pfnDataCallback) { // only ESP32-S2 and S3
    //    spilcdParallelData(pBuf, iLen);
    //    return;
    //}


    if (pLCD->iLCDFlags & FLAGS_BITBANG)
    {
        SPI_BitBang(pLCD, pBuf, iLen, iMode);
        return;
    }
    if (iMode == MODE_COMMAND)
    {
		#ifdef HAS_DMA
        spilcdWaitDMA(); // wait for any previous transaction to finish
		#endif
        spilcdSetMode(pLCD, MODE_COMMAND);
    }
    if ((iFlags & DRAW_WITH_DMA) == 0 || iMode == MODE_COMMAND)
    {
    	gpio_set_level(pLCD->iCSPin, 0);
    }
    // DMA was requested, so we initialized the SPI peripheral, but
    // The user is asking to not use it, so use polling SPI
    if (pLCD->bUseDMA && (iMode == MODE_COMMAND || !(iFlags & DRAW_WITH_DMA)))
    {
        esp_err_t ret;
        static spi_transaction_t t;
        iCurrentCS = -1;
        memset(&t, 0, sizeof(t));       //Zero out the transaction
        while (iLen) {
          int l = iLen;
          if (l > 4000) { // transmit maximum length (full duplex mode)
             l = 4000;
          }
          t.length=l*8;  // length in bits
          t.rxlength = 0;
          t.tx_buffer=pBuf;
          t.user=(void*)iMode;
    // Queue the transaction
//    ret = spi_device_queue_trans(spi, &t, portMAX_DELAY);
          ret=spi_device_polling_transmit(spi, &t);  //Transmit!
          assert(ret==ESP_OK);            //Should have had no issues.
          iLen -= l;
          pBuf += l;
        } // while (iLen)
        if (iMode == MODE_COMMAND) {// restore D/C pin to DATA
            spilcdSetMode(pLCD, MODE_DATA);
        }
        gpio_set_level(pLCD->iCSPin, 1);
        return;
    }
    // wait for it to complete
//    spi_transaction_t *rtrans;
//    spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);

#ifdef HAS_DMA
    if (iMode == MODE_DATA && (iFlags & DRAW_WITH_DMA)) // only pixels will get DMA treatment
    {
        while (iLen) {
            int l = iLen;
            if (l > 4000) {
                l = 4000;
            }
            spilcdWaitDMA(); // wait for any previous transaction to finish
            iCurrentCS = pLCD->iCSPin;
            gpio_set_level(pLCD->iCSPin, 0);
            if (pBuf != ucTXBuf) // for DMA, we must use the one output buffer
                memcpy(ucTXBuf, pBuf, l);
            pBuf += l;
            spilcdWriteDataDMA(pLCD, l);
            iLen -= l;
        }
        return;
    }
#endif // HAS_DMA


    esp_err_t ret;
        static spi_transaction_t t;
        memset(&t, 0, sizeof(t));       // Zero out the transaction
        while (iLen) {
            int l = iLen;
            if (l > 4000) { // Transmit maximum length (full duplex mode)
                l = 4000;
            }
            t.length = l * 8;  // length in bits
            t.rxlength = 0;
            t.tx_buffer = pBuf;

            // Transmit the data
            ret = spi_device_polling_transmit(spi, &t);
            assert(ret == ESP_OK);

            iLen -= l;
            pBuf += l;
        }


    if (iMode == MODE_COMMAND) { // restore D/C pin to DATA
        spilcdSetMode(pLCD, MODE_DATA);
    }
    if (pLCD->iCSPin != -1)
    {
       gpio_set_level(pLCD->iCSPin, 1);
    }
} /* myspiWrite() */

//
// Public wrapper function to write data to the display
//
void spilcdWriteDataBlock(SPILCD *pLCD, uint8_t *pData, int iLen, int iFlags)
{
  myspiWrite(pLCD, pData, iLen, MODE_DATA, iFlags);
} /* spilcdWriteDataBlock() */

//
// spilcdWritePixelsMasked
//
void spilcdWritePixelsMasked(SPILCD *pLCD, int x, int y, uint8_t *pData, uint8_t *pMask, int iCount, int iFlags)
{
    int i, pix_count, bit_count;
    uint8_t c, *s, *sPixels, *pEnd;
    s = pMask; sPixels = pData;
    pEnd = &pMask[(iCount+7)>>3];
    i = 0;
    bit_count = 8;
    c = *s++; // get first byte
    while (i<iCount && s < pEnd) {
        // Count the number of consecutive pixels to skip
        pix_count = 0;
        while (bit_count && (c & 0x80) == 0) { // count 0's
            if (c == 0) { // quick count remaining 0 bits
                pix_count += bit_count;
                bit_count = 0;
                if (s < pEnd) {
                    bit_count = 8;
                    c = *s++;
                }
                continue;
            }
            pix_count++;
            bit_count--;
            c <<= 1;
            if (bit_count == 0 && s < pEnd) {
                bit_count = 8;
                c = *s++;
            }
        }
        // we've hit the first 1 bit, skip the source pixels we've counted
        i += pix_count;
        sPixels += pix_count*2; // skip RGB565 pixels
        // Count the number of consecutive pixels to draw
        pix_count = 0;
        while (bit_count && (c & 0x80) == 0x80) { // count 1's
            if (c == 0xff) {
                pix_count += 8;
                bit_count = 0;
                if (s < pEnd) {
                    c = *s++;
                    bit_count = 8;
                }
                continue;
            }
            pix_count++;
            bit_count--;
            c <<= 1;
            if (bit_count == 0 && s < pEnd) {
                bit_count = 8;
                c = *s++;
            }
        }
        if (pix_count) {
            spilcdSetPosition(pLCD, x+i, y, pix_count, 1, iFlags);
            spilcdWriteDataBlock(pLCD, sPixels, pix_count*2, iFlags);
        }
        i += pix_count;
        sPixels += pix_count*2;
    } // while counting pixels
} /* spilcdWritePixelsMasked() */

//
// Choose the gamma curve between 2 choices (0/1)
// ILI9341 only
//
int spilcdSetGamma(SPILCD *pLCD, int iMode)
{
	int i;
	unsigned char *sE0, *sE1;

	if (iMode < 0 || iMode > 1 || pLCD->iLCDType != LCD_ILI9341)
		return 1;
	if (iMode == 0)
	{
		sE0 = (unsigned char *)ucE0_0;
		sE1 = (unsigned char *)ucE1_0;
	}
	else
	{
		sE0 = (unsigned char *)ucE0_1;
		sE1 = (unsigned char *)ucE1_1;
	}
	spilcdWriteCommand(pLCD, 0xe0);
	for(i=0; i<16; i++)
	{
		spilcdWriteData8(pLCD,*sE0++);
	}
	spilcdWriteCommand(pLCD, 0xe1);
	for(i=0; i<16; i++)
	{
		spilcdWriteData8(pLCD, *sE1++);
	}

	return 0;
} /* spilcdSetGamma() */

// *****************
//
// Configure a GPIO pin for input
// Returns 0 if successful, -1 if unavailable
// all input pins are assumed to use internal pullup resistors
// and are connected to ground when pressed
//
int spilcdConfigurePin(gpio_num_t iPin)
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupts
	io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
	io_conf.pin_bit_mask = (1ULL << iPin);  // Set the bit mask for the pin
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;       // Enable pull-up

	// Apply the configuration
	gpio_config(&io_conf);
	return 0;
} /* spilcdConfigurePin() */


// Read from a GPIO pin
int spilcdReadPin(gpio_num_t iPin)
{
    return (gpio_get_level(iPin) == 1);
} /* spilcdReadPin() */

int spilcdInit(SPILCD *pLCD, int iType, int iFlags, int32_t iSPIFreq, gpio_num_t iCS, gpio_num_t iDC, gpio_num_t iReset, gpio_num_t iMISOPin, gpio_num_t iMOSIPin, gpio_num_t iCLKPin, int bUseDMA)
{
	unsigned char *s, *d;
	int iCount;

    if (pLCD->pFont != NULL && iSPIFreq != 0) { // the structure is probably not initialized
        memset(pLCD, 0, sizeof(SPILCD));
    }
    pLCD->bUseDMA = bUseDMA;
    pLCD->iColStart = pLCD->iRowStart = pLCD->iMemoryX = pLCD->iMemoryY = 0;
    pLCD->iOrientation = 0;
    pLCD->iLCDType = iType;
    pLCD->iLCDFlags = iFlags;

    (void)iMISOPin;

    pLCD->iSPIMode = 0; // may be 3 for st7789

    pLCD->iSPISpeed = iSPIFreq;
	pLCD->iScrollOffset = 0; // current hardware scroll register value

    pLCD->iDCPin = iDC;
    pLCD->iCSPin = iCS;
    pLCD->iResetPin = iReset;

    if (iFlags & FLAGS_BITBANG)
    {
    	gpio_config_t io_conf;
		io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupts
		io_conf.mode = GPIO_MODE_OUTPUT;         // Set as input mode
		io_conf.pin_bit_mask = (1ULL << pLCD->iMOSIPin);  // Set the bit mask for the pin
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up
		// Apply the configuration
		gpio_config(&io_conf);

		io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupts
		io_conf.mode = GPIO_MODE_OUTPUT;         // Set as input mode
		io_conf.pin_bit_mask = (1ULL << pLCD->iCLKPin);  // Set the bit mask for the pin
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up
		// Apply the configuration
		gpio_config(&io_conf);

    }

		esp_err_t ret;

		memset(&buscfg, 0, sizeof(buscfg));
		buscfg.miso_io_num = iMISOPin;
		buscfg.mosi_io_num = iMOSIPin;
		buscfg.sclk_io_num = iCLKPin;
		buscfg.max_transfer_sz=240*9*2;
		buscfg.quadwp_io_num=-1;
		buscfg.quadhd_io_num=-1;
		//Initialize the SPI bus
		ret=spi_bus_initialize(ESP32_SPI_HOST, &buscfg, SPI_DMA_CHAN);
		assert(ret==ESP_OK);

		memset(&devcfg, 0, sizeof(devcfg));
		devcfg.clock_speed_hz = iSPIFreq;
		devcfg.mode = pLCD->iSPIMode;                         //SPI mode 0 or 3
		devcfg.spics_io_num = -1;               //CS pin, set to -1 to disable since we handle it outside of the master driver
		devcfg.queue_size = 2;                          //We want to be able to queue 2 transactions at a time
	// These callbacks currently don't do anything
	//	devcfg.pre_cb = spi_pre_transfer_callback;  //Specify pre-transfer callback to handle D/C line
	//	devcfg.post_cb = spi_post_transfer_callback;
		devcfg.flags = SPI_DEVICE_NO_DUMMY; // allow speeds > 26Mhz
	//    devcfg.flags = SPI_DEVICE_HALFDUPLEX; // this disables SD card access
		//Attach the LCD to the SPI bus
		ret=spi_bus_add_device(ESP32_SPI_HOST, &devcfg, &spi);
		assert(ret==ESP_OK);
		memset(&trans[0], 0, sizeof(spi_transaction_t));


	//
	// Start here if bit bang enabled
	//
    if (pLCD->iCSPin != -1)
    {
    	gpio_config_t io_conf;
		io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupts
		io_conf.mode = GPIO_MODE_OUTPUT;         // Set as input mode
		io_conf.pin_bit_mask = (1ULL << pLCD->iCSPin);  // Set the bit mask for the pin
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up
		// Apply the configuration
		gpio_config(&io_conf);

        gpio_set_level(pLCD->iCSPin, HIGH);
    }

    	gpio_config_t io_conf;
    		io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupts
    		io_conf.mode = GPIO_MODE_OUTPUT;         // Set as input mode
    		io_conf.pin_bit_mask = (1ULL << pLCD->iDCPin);  // Set the bit mask for the pin
    		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
    		io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up
    		// Apply the configuration
    		gpio_config(&io_conf);
	if (pLCD->iResetPin != -1)
	{
		gpio_config_t io_conf;
		    		io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupts
		    		io_conf.mode = GPIO_MODE_OUTPUT;         // Set as input mode
		    		io_conf.pin_bit_mask = (1ULL << pLCD->iResetPin);  // Set the bit mask for the pin
		    		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
		    		io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up
		    		// Apply the configuration
		    		gpio_config(&io_conf);

		gpio_set_level(pLCD->iResetPin, 1);
		delayMicroseconds(60000);
		gpio_set_level(pLCD->iResetPin, 0); // reset the controller
		delayMicroseconds(60000);
		gpio_set_level(pLCD->iResetPin, 1);
		delayMicroseconds(60000);
	}

	spilcdWriteCommand(pLCD, 0x01); // software reset
	delayMicroseconds(60000);

	spilcdWriteCommand(pLCD, 0x11);
	delayMicroseconds(60000);
	delayMicroseconds(60000);

    d = ucRXBuf;
	switch (pLCD->iLCDType) {

		case LCD_ST7789_240:
		{
			uint8_t iBGR = (pLCD->iLCDFlags & FLAGS_SWAP_RB) ? 8:0;
			s = (unsigned char *)&uc240x240InitList[0];
			memcpy(d, s, sizeof(uc240x240InitList));
			s = d;
			s[6] = 0x00 + iBGR;
			if (pLCD->iLCDFlags & FLAGS_INVERT)
			   s[3] = 0x20; // change inversion on (default) to off
			pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
			pLCD->iCurrentWidth = pLCD->iWidth = 240;
			pLCD->iCurrentHeight = pLCD->iHeight = 320;
			if (pLCD->iLCDType == LCD_ST7789_240 )
			{
					pLCD->iCurrentWidth = pLCD->iWidth = 240;
					pLCD->iCurrentHeight = pLCD->iHeight = 240;
			}
			break;

		} // ST7789

		case LCD_ILI9341:
		{  // copy to RAM to modify
			s = (unsigned char *)uc240InitList;
			memcpy(d, s, sizeof(uc240InitList));
			s = d;
			if (pLCD->iLCDFlags & FLAGS_INVERT)
				s[52] = 0x21; // invert pixels
			else
				s[52] = 0x20; // non-inverted
			pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
			pLCD->iCurrentWidth = pLCD->iWidth = 240;
			pLCD->iCurrentHeight = pLCD->iHeight = 320;
			break;
		}

	}
    iCount = 1;
    bSetPosition = 1; // don't let the data writes affect RAM
    s = d; // start of RAM copy of our data
	while (iCount)
	{
		iCount = *s++;
		if (iCount != 0)
		{
               unsigned char uc;
               if (iCount == LCD_DELAY)
               {
                 uc = *s++;
                 delayMicroseconds(uc*1000);
               }
               else
               {
				 spilcdWriteCmdParams(pLCD, s[0], &s[1], iCount-1);
				 s += iCount;
              }
          }
	  }
      bSetPosition = 0;

	spilcdWriteCommand(pLCD, 0x11); // sleep out
	delayMicroseconds(60000);
	spilcdWriteCommand(pLCD, 0x29); // Display ON
	delayMicroseconds(10000);

//	spilcdFill(0, 1); // erase memory
	spilcdScrollReset(pLCD);

	return 0;

} /* spilcdInit() */

//
// Reset the scroll position to 0
//
void spilcdScrollReset(SPILCD *pLCD)
{
	uint32_t u32Temp;
	int iLen;
    u32Temp = 0;
    iLen = 2;
	spilcdWriteCmdParams(pLCD, 0x37, (uint8_t *)&u32Temp, iLen); // scroll start address
} /* spilcdScrollReset() */


//
// Scroll the screen N lines vertically (positive or negative)
// The value given represents a delta which affects the current scroll offset
// If iFillColor != -1, the newly exposed lines will be filled with that color
//
void spilcdScroll(SPILCD *pLCD, int iLines, int iFillColor)
{

	pLCD->iScrollOffset = (pLCD->iScrollOffset + iLines) % pLCD->iHeight;

		spilcdWriteCommand(pLCD, 0x37); // Vertical scrolling start address
		spilcdWriteData16(pLCD, pLCD->iScrollOffset, DRAW_TO_LCD);


	if (iFillColor != -1) // fill the exposed lines
	{
	int i, iStart;
	uint16_t *usTemp = (uint16_t *)ucRXBuf;
	uint32_t *d;
	uint32_t u32Fill;
		// quickly prepare a full line's worth of the color
		u32Fill = (iFillColor >> 8) | ((iFillColor & -1) << 8);
		u32Fill |= (u32Fill << 16);
		d = (uint32_t *)&usTemp[0];
		for (i=0; i<pLCD->iWidth/2; i++)
			*d++ = u32Fill;
		if (iLines < 0)
		{
			iStart = 0;
			iLines = 0 - iLines;
		}
		else
			iStart = pLCD->iHeight - iLines;
        spilcdSetPosition(pLCD, 0, iStart, pLCD->iWidth, iLines, DRAW_TO_LCD);
		for (i=0; i<iLines; i++)
		{
			myspiWrite(pLCD, (unsigned char *)usTemp, pLCD->iWidth*2, MODE_DATA, DRAW_TO_LCD);
		}
	}

} /* spilcdScroll() */

void spilcdDrawPattern(SPILCD *pLCD, uint8_t *pPattern, int iSrcPitch, int iDestX, int iDestY, int iCX, int iCY, uint16_t usColor, int iTranslucency)
{
    int x, y;
    uint8_t *s, uc, ucMask;
    uint16_t us, *d;
    uint32_t ulMask = 0x07e0f81f; // this allows all 3 values to be multipled at once
    uint32_t ulSrcClr, ulDestClr, ulDestTrans;

    ulDestTrans = 32-iTranslucency; // inverted to combine src+dest
    ulSrcClr = (usColor & 0xf81f) | ((uint32_t)(usColor & 0x06e0) << 16); // shift green to upper 16-bits
    ulSrcClr *= iTranslucency; // prepare for color blending
    if (iDestX+iCX > pLCD->iCurrentWidth) // trim to fit on display
        iCX = (pLCD->iCurrentWidth - iDestX);
    if (iDestY+iCY > pLCD->iCurrentHeight)
        iCY = (pLCD->iCurrentHeight - iDestY);
    if (pPattern == NULL || iDestX < 0 || iDestY < 0 || iCX <=0 || iCY <= 0 || iTranslucency < 1 || iTranslucency > 32)
        return;
    if (pLCD->pBackBuffer == NULL) // no back buffer, draw opaque colors
    {
      uint16_t u16Clr;
      u16Clr = (usColor >> 8) | (usColor << 8); // swap low/high bytes
      spilcdSetPosition(pLCD, iDestX, iDestY, iCX, iCY, DRAW_TO_LCD);
      for (y=0; y<iCY; y++)
      {
        s = &pPattern[y * iSrcPitch];
        ucMask = uc = 0;
        d = (uint16_t *)&ucTXBuf[0];
        for (x=0; x<iCX; x++)
        {
            ucMask >>= 1;
            if (ucMask == 0)
            {
                ucMask = 0x80;
                uc = *s++;
            }
            if (uc & ucMask) // active pixel
               *d++ = u16Clr;
            else
               *d++ = 0;
        } // for x
        myspiWrite(pLCD, ucTXBuf, iCX*2, MODE_DATA, DRAW_TO_LCD);
      } // for y
      return;
    }
    for (y=0; y<iCY; y++)
    {
        int iDelta;
        iDelta = 1;
        d = (uint16_t *)&pLCD->pBackBuffer[((iDestY+y)*pLCD->iScreenPitch) + (iDestX*2)];
        s = &pPattern[y * iSrcPitch];
        ucMask = uc = 0;
        for (x=0; x<iCX; x++)
        {
            ucMask >>= 1;
            if (ucMask == 0)
            {
                ucMask = 0x80;
                uc = *s++;
            }
            if (uc & ucMask) // active pixel
            {
                us = d[0]; // read destination pixel
                us = (us >> 8) | (us << 8); // fix the byte order
                // The fast way to combine 2 RGB565 colors
                ulDestClr = (us & 0xf81f) | ((uint32_t)(us & 0x06e0) << 16);
                ulDestClr = (ulDestClr * ulDestTrans);
                ulDestClr += ulSrcClr; // combine old and new colors
                ulDestClr = (ulDestClr >> 5) & ulMask; // done!
                ulDestClr = (ulDestClr >> 16) | (ulDestClr); // move green back into place
                us = (uint16_t)ulDestClr;
                us = (us >> 8) | (us << 8); // swap bytes for LCD
                d[0] = us;
            }
            d += iDelta;
        } // for x
    } // for y

}

void spilcdRectangle(SPILCD *pLCD, int x, int y, int w, int h, unsigned short usColor1, unsigned short usColor2, int bFill, int iFlags)
{
unsigned short *usTemp = (unsigned short *)ucRXBuf;
int i, ty, th, iStart;
uint16_t usColor;

	// check bounds
	if (x < 0 || x >= pLCD->iCurrentWidth || x+w > pLCD->iCurrentWidth)
		return; // out of bounds
	if (y < 0 || y >= pLCD->iCurrentHeight || y+h > pLCD->iCurrentHeight)
		return;

	ty = y;
	th = h;
	if (bFill)
	{
        int32_t iDR, iDG, iDB; // current colors and deltas
        int32_t iRAcc, iGAcc, iBAcc;
        uint16_t usRInc, usGInc, usBInc;
        iRAcc = iGAcc = iBAcc = 0; // color fraction accumulators
        iDB = (int32_t)(usColor2 & 0x1f) - (int32_t)(usColor1 & 0x1f); // color deltas
        usBInc = (iDB < 0) ? 0xffff : 0x0001;
        iDB = abs(iDB);
        iDR = (int32_t)(usColor2 >> 11) - (int32_t)(usColor1 >> 11);
        usRInc = (iDR < 0) ? 0xf800 : 0x0800;
        iDR = abs(iDR);
        iDG = (int32_t)((usColor2 & 0x06e0) >> 5) - (int32_t)((usColor1 & 0x06e0) >> 5);
        usGInc = (iDG < 0) ? 0xffe0 : 0x0020;
        iDG = abs(iDG);
        iDB = (iDB << 16) / th;
        iDR = (iDR << 16) / th;
        iDG = (iDG << 16) / th;
        spilcdSetPosition(pLCD, x, y, w, h, iFlags);
	for (i=0; i<h; i++)
            {
                usColor = (usColor1 >> 8) | (usColor1 << 8); // swap byte order
                memset16((uint16_t*)usTemp, usColor, w);
                myspiWrite(pLCD, (unsigned char *)usTemp, w*2, MODE_DATA, iFlags);
                // Update the color components
                iRAcc += iDR;
                if (iRAcc >= 0x10000) // time to increment
                {
                    usColor1 += usRInc;
                    iRAcc -= 0x10000;
                }
                iGAcc += iDG;
                if (iGAcc >= 0x10000) // time to increment
                {
                    usColor1 += usGInc;
                    iGAcc -= 0x10000;
                }
                iBAcc += iDB;
                if (iBAcc >= 0x10000) // time to increment
                {
                    usColor1 += usBInc;
                    iBAcc -= 0x10000;
                }
            }
	}
	else // outline
	{
        usColor = (usColor1 >> 8) | (usColor1 << 8); // swap byte order
		// draw top/bottom
		spilcdSetPosition(pLCD, x, y, w, 1, iFlags);
        memset16((uint16_t*)usTemp, usColor, w);
        myspiWrite(pLCD, (unsigned char *)usTemp, w*2, MODE_DATA, iFlags);
		spilcdSetPosition(pLCD, x, y + h-1, w, 1, iFlags);
        memset16((uint16_t*)usTemp, usColor, w);
		myspiWrite(pLCD, (unsigned char *)usTemp, w*2, MODE_DATA, iFlags);
		// draw left/right
		if (((ty + pLCD->iScrollOffset) % pLCD->iCurrentHeight) > pLCD->iCurrentHeight-th)
		{
			iStart = (pLCD->iCurrentHeight - ((ty+pLCD->iScrollOffset) % pLCD->iCurrentHeight));
			spilcdSetPosition(pLCD, x, y, 1, iStart, iFlags);
            memset16((uint16_t*)usTemp, usColor, iStart);
			myspiWrite(pLCD, (unsigned char *)usTemp, iStart*2, MODE_DATA, iFlags);
			spilcdSetPosition(pLCD, x+w-1, y, 1, iStart, iFlags);
            memset16((uint16_t*)usTemp, usColor, iStart);
			myspiWrite(pLCD, (unsigned char *)usTemp, iStart*2, MODE_DATA, iFlags);
			// second half
			spilcdSetPosition(pLCD, x,y+iStart, 1, h-iStart, iFlags);
            memset16((uint16_t*)usTemp, usColor, h-iStart);
			myspiWrite(pLCD, (unsigned char *)usTemp, (h-iStart)*2, MODE_DATA, iFlags);
			spilcdSetPosition(pLCD, x+w-1, y+iStart, 1, h-iStart, iFlags);
            memset16((uint16_t*)usTemp, usColor, h-iStart);
			myspiWrite(pLCD, (unsigned char *)usTemp, (h-iStart)*2, MODE_DATA, iFlags);
		}
		else // can do it in 1 shot
		{
			spilcdSetPosition(pLCD, x, y, 1, h, iFlags);
            memset16((uint16_t*)usTemp, usColor, h);
			myspiWrite(pLCD, (unsigned char *)usTemp, h*2, MODE_DATA, iFlags);
			spilcdSetPosition(pLCD, x + w-1, y, 1, h, iFlags);
            memset16((uint16_t*)usTemp, usColor, h);
			myspiWrite(pLCD, (unsigned char *)usTemp, h*2, MODE_DATA, iFlags);
		}
	} // outline
} /* spilcdRectangle() */


// Show part or all of the back buffer on the display
// Used after delayed rendering of graphics
//
void spilcdShowBuffer(SPILCD *pLCD, int iStartX, int iStartY, int cx, int cy, int iFlags)
{
    int y;
    uint8_t *s;

    if (pLCD->pBackBuffer == NULL)
        return; // nothing to do
    if (iStartX + cx > pLCD->iCurrentWidth || iStartY + cy > pLCD->iCurrentHeight || iStartX < 0 || iStartY < 0)
        return; // invalid area
    spilcdSetPosition(pLCD, iStartX, iStartY, cx, cy, iFlags);
    bSetPosition = 1;

	for (y=iStartY; y<iStartY+cy; y++)
	{
		s = &pLCD->pBackBuffer[(y * pLCD->iCurrentWidth * 2) + iStartX*2];
		myspiWrite(pLCD, s, cx * 2, MODE_DATA, iFlags);
	}

    bSetPosition = 0;
} /* spilcdShowBuffer() */

// Sends a command to turn off the LCD display
// Turns off the backlight LED
// Closes the SPI file handle
//
void spilcdShutdown(SPILCD *pLCD)
{
	spilcdWriteCommand(pLCD, 0x28); // Display OFF
    spilcdFreeBackbuffer(pLCD);
} /* spilcdShutdown() */

//
// Write a command byte followed by parameters
//
void spilcdWriteCmdParams(SPILCD *pLCD, uint8_t ucCMD, uint8_t *pParams, int iLen)
{
    myspiWrite(pLCD, &ucCMD, 1, MODE_COMMAND, DRAW_TO_LCD);
    if (iLen) {
        myspiWrite(pLCD, pParams, iLen, MODE_DATA, DRAW_TO_LCD);
    }
} /* spilcdWriteCmdParams() */


//
// Send a command byte to the LCD controller
// In SPI 8-bit mode, the D/C line must be set
// high during the write
//
void spilcdWriteCommand(SPILCD *pLCD, unsigned char c)
{
	unsigned char buf[2];
	buf[0] = c;
	myspiWrite(pLCD, buf, 1, MODE_COMMAND, DRAW_TO_LCD);
} /* spilcdWriteCommand() */

void spilcdWriteCommand16(SPILCD *pLCD, uint16_t us)
{
	unsigned char buf[2];

    buf[0] = (uint8_t)(us >> 8);
    buf[1] = (uint8_t)us;
//    myspiWrite(pLCD, buf, 2, MODE_COMMAND, DRAW_TO_LCD);
    spilcdSetMode(pLCD, MODE_COMMAND);
    spilcdParallelData(buf, 2);
    spilcdSetMode(pLCD, MODE_DATA);
} /* spilcdWriteCommand() */

//
// Write a single byte of data
//
static void spilcdWriteData8(SPILCD *pLCD, unsigned char c)
{
	unsigned char buf[2];

	buf[0] = c;
    myspiWrite(pLCD, buf, 1, MODE_DATA, DRAW_TO_LCD);

} /* spilcdWriteData8() */

//
// Write 16-bits of data
// The ILI9341 receives data in big-endian order
// (MSB first)
//
static void spilcdWriteData16(SPILCD *pLCD, unsigned short us, int iFlags)
{
	unsigned char buf[2];

    buf[0] = (unsigned char)(us >> 8);
    buf[1] = (unsigned char)us;
    spilcdParallelData(buf, 2);
//    myspiWrite(pLCD, buf, 2, MODE_DATA, iFlags);

} /* spilcdWriteData16() */


//
// Set the text cursor position in pixels
//
void spilcdSetCursor(SPILCD *pLCD, int x, int y)
{
    pLCD->iCursorX = x;
    pLCD->iCursorY = y;
} /* spilcdSetCursor() */

//
// Position the "cursor" to the given
// row and column. The width and height of the memory
// 'window' must be specified as well. The controller
// allows more efficient writing of small blocks (e.g. tiles)
// by bounding the writes within a small area and automatically
// wrapping the address when reaching the end of the window
// on the curent row
//
void spilcdSetPosition(SPILCD *pLCD, int x, int y, int w, int h, int iFlags)
{
	unsigned char ucBuf[8];
	int iLen;
    pLCD->iWindowX = pLCD->iCurrentX = x; pLCD->iWindowY = pLCD->iCurrentY = y;
    pLCD->iWindowCX = w; pLCD->iWindowCY = h;
    pLCD->iOffset = (pLCD->iCurrentWidth * 2 * y) + (x * 2);

    if (!(iFlags & DRAW_TO_LCD)){ return; }// nothing to do

    bSetPosition = 1; // flag to let myspiWrite know to ignore data writes
    y = (y + pLCD->iScrollOffset) % pLCD->iHeight; // scroll offset affects writing position

    if (pLCD->iCMDType == CMD_TYPE_ILITEK_16BIT) // 16-bit commands and data
    { // The display flipping bits don't change the address information, just
        // the horizontal and vertical inc/dec, so we must place the starting
        // address correctly for the 4 different orientations
        int xs=0, xb=0, xe=0, ys=0, yb= 0,ye=0;
        switch (pLCD->iOrientation)
        {
            case LCD_ORIENTATION_0:
                xs = xb = x; xe = (x + w - 1);
                ys = yb = y; ye = (y + h - 1);
                break;
            case LCD_ORIENTATION_90:
                yb = ys = x;
                ye = x + w - 1;
                xb = pLCD->iCurrentHeight - y - h;
                xe = xs = pLCD->iCurrentHeight - 1 - y;
                break;
            case LCD_ORIENTATION_180:
                xb = (pLCD->iCurrentWidth - x - w);
                xe = xs = (pLCD->iCurrentWidth - 1 - x);
                yb = (pLCD->iCurrentHeight - y - h);
                ye = ys = (pLCD->iCurrentHeight - 1 - y);
                break;
            case LCD_ORIENTATION_270:
                ye = ys = pLCD->iCurrentWidth - 1 - x;
                yb = pLCD->iCurrentWidth - x - w;
                xb = xs = y;
                xe = y + h - 1;
                break;
        }
        spilcdWriteCommand16(pLCD, 0x36); // Horizontal Address end
        spilcdWriteData16(pLCD, xe, DRAW_TO_LCD);
        spilcdWriteCommand16(pLCD, 0x37); // Horizontal Address start
        spilcdWriteData16(pLCD, xb, DRAW_TO_LCD);
        spilcdWriteCommand16(pLCD, 0x38); // Vertical Address end
        spilcdWriteData16(pLCD, ye, DRAW_TO_LCD);
        spilcdWriteCommand16(pLCD, 0x39); // Vertical Address start
        spilcdWriteData16(pLCD, yb, DRAW_TO_LCD);

        spilcdWriteCommand16(pLCD, 0x20); // Horizontal RAM address set
        spilcdWriteData16(pLCD, xs, DRAW_TO_LCD);
        spilcdWriteCommand16(pLCD, 0x21); // Vertical RAM address set
        spilcdWriteData16(pLCD, ys, DRAW_TO_LCD);

        spilcdWriteCommand16(pLCD, 0x22); // write to RAM
        bSetPosition = 0;
        return;
    }
	if (pLCD->iCMDType == CMD_TYPE_SOLOMON_OLED1) // OLED has very different commands
	{
		spilcdWriteCommand(pLCD, 0x15); // set column
		ucBuf[0] = x;
		ucBuf[1] = x + w - 1;
		myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
		spilcdWriteCommand(pLCD, 0x75); // set row
		ucBuf[0] = y;
		ucBuf[1] = y + h - 1;
		myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
		spilcdWriteCommand(pLCD, 0x5c); // write RAM
		bSetPosition = 0;
		return;
	}
        else if (pLCD->iCMDType == CMD_TYPE_SOLOMON_LCD) // so does the SSD1283A
        {
            switch (pLCD->iOrientation) {
                case LCD_ORIENTATION_0:
                    spilcdWriteCommand(pLCD, 0x44); // set col
                    ucBuf[0] = x + w - 1;
                    ucBuf[1] = x;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x45); // set row
                    ucBuf[0] = y + h - 1;
                    ucBuf[1] = y;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x21); // set col+row
                    ucBuf[0] = y;
                    ucBuf[1] = x;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    break;
                case LCD_ORIENTATION_90:
                    spilcdWriteCommand(pLCD, 0x44); // set col
                    ucBuf[0] = y + h;
                    ucBuf[1] = y + 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x45); // set row
                    ucBuf[0] = x + w -1;
                    ucBuf[1] = x;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x21); // set col+row
                    ucBuf[0] = x+1;
                    ucBuf[1] = y+1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    break;
                case LCD_ORIENTATION_180:
                    spilcdWriteCommand(pLCD, 0x44);
                    ucBuf[0] = pLCD->iCurrentWidth - x - 1;
                    ucBuf[1] = pLCD->iCurrentWidth - (x+w);
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x45);
                    ucBuf[0] = pLCD->iCurrentHeight - y - 1;
                    ucBuf[1] = pLCD->iCurrentHeight - (y+h) - 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x21);
                    ucBuf[0] = pLCD->iCurrentHeight - y - 1;
                    ucBuf[1] = pLCD->iCurrentWidth - x - 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                  break;
                case LCD_ORIENTATION_270:
                    spilcdWriteCommand(pLCD, 0x44);
                    ucBuf[0] = pLCD->iCurrentHeight - (y + h) - 1;
                    ucBuf[1] = pLCD->iCurrentHeight - y - 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x45);
                    ucBuf[0] = x + w - 1;
                    ucBuf[1] = x;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x21);
                    ucBuf[0] = x + 2;
                    ucBuf[1] = y + 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                  break;
            } // switch on orientation
            spilcdWriteCommand(pLCD, 0x22); // write RAM
            bSetPosition = 0;
            return;
        }
    else if (pLCD->iCMDType == CMD_TYPE_SOLOMON_OLED2)
    {
        spilcdWriteCommand(pLCD, 0x15);
        ucBuf[0] = x;
        ucBuf[1] = x + w - 1;
        myspiWrite(pLCD, ucBuf, 2, MODE_COMMAND, iFlags);

        spilcdWriteCommand(pLCD, 0x75);
        ucBuf[0] = y;
        ucBuf[1] = y + h - 1;
        myspiWrite(pLCD, ucBuf, 2, MODE_COMMAND, iFlags);

        bSetPosition = 0;
        return;
    }
    if (x != pLCD->iOldX || w != pLCD->iOldCX)
    {
        pLCD->iOldX = x; pLCD->iOldCX = w;
	if (pLCD->iCMDType == CMD_TYPE_SITRONIX_8BIT)
	{
		x += pLCD->iMemoryX;
		ucBuf[0] = (unsigned char)(x >> 8);
		ucBuf[1] = (unsigned char)x;
		x = x + w - 1;
//		if ((x-iMemoryX) > iWidth-1) x = iMemoryX + iWidth-1;
		ucBuf[2] = (unsigned char)(x >> 8);
		ucBuf[3] = (unsigned char)x;
        iLen = 4;
	}
	else // must be Sitronix 16-bit
	{
// combine coordinates into 1 write to save time
		ucBuf[0] = 0;
 		ucBuf[1] = (unsigned char)(x >> 8); // MSB first
		ucBuf[2] = 0;
		ucBuf[3] = (unsigned char)x;
		x = x + w -1;
		if (x > pLCD->iWidth-1) x = pLCD->iWidth-1;
		ucBuf[4] = 0;
		ucBuf[5] = (unsigned char)(x >> 8);
		ucBuf[6] = 0;
		ucBuf[7] = (unsigned char)x;
        iLen = 8;
	}
        spilcdWriteCmdParams(pLCD, 0x2a, ucBuf, iLen); // set column address
    } // if X changed
    if (y != pLCD->iOldY || h != pLCD->iOldCY)
    {
        pLCD->iOldY = y; pLCD->iOldCY = h;
	if (pLCD->iCMDType == CMD_TYPE_SITRONIX_8BIT)
	{
                if (pLCD->iCurrentHeight == 135 && pLCD->iOrientation == LCD_ORIENTATION_90)
                   pLCD->iMemoryY+= 1; // ST7789 240x135 rotated 90 is off by 1
		y += pLCD->iMemoryY;
		ucBuf[0] = (unsigned char)(y >> 8);
		ucBuf[1] = (unsigned char)y;
		y = y + h;
		if ((y-pLCD->iMemoryY) > pLCD->iCurrentHeight-1) y = pLCD->iMemoryY + pLCD->iCurrentHeight;
		ucBuf[2] = (unsigned char)(y >> 8);
		ucBuf[3] = (unsigned char)y;
        iLen = 4;
                if (pLCD->iCurrentHeight == 135 && pLCD->iOrientation == LCD_ORIENTATION_90)
                   pLCD->iMemoryY -=1; // ST7789 240x135 rotated 90 is off by 1
	}
	else // must be Sitronix 16-bit
	{
// combine coordinates into 1 write to save time
		ucBuf[0] = 0;
		ucBuf[1] = (unsigned char)(y >> 8); // MSB first
		ucBuf[2] = 0;
		ucBuf[3] = (unsigned char)y;
		y = y + h - 1;
		if (y > pLCD->iHeight-1) y = pLCD->iHeight-1;
		ucBuf[4] = 0;
		ucBuf[5] = (unsigned char)(y >> 8);
		ucBuf[6] = 0;
		ucBuf[7] = (unsigned char)y;
        iLen = 8;
	}
        spilcdWriteCmdParams(pLCD, 0x2b, ucBuf, iLen); // set row address
    } // if Y changed
    if (!(pLCD->iLCDFlags & FLAGS_MEM_RESTART))
        spilcdWriteCommand(pLCD, 0x2c); // write memory begin
//	spilcdWriteCommand(0x3c); // write memory continue
    bSetPosition = 0;
} /* spilcdSetPosition() */

int spilcdSetPixel(SPILCD *pLCD, int x, int y, unsigned short usColor, int iFlags)
{
	spilcdSetPosition(pLCD, x, y, 1, 1, iFlags);
	spilcdWriteData16(pLCD, usColor, iFlags);
	return 0;
} /* spilcdSetPixel() */

uint8_t * spilcdGetDMABuffer(void)
{
  return (uint8_t *)ucTXBuf;
}


void spilcdWaitDMA(void)
{

    while (!transfer_is_done);
}


void spilcdWriteDataDMA(SPILCD *pLCD, int iLen)
{
	esp_err_t ret;

    trans[0].tx_buffer = ucTXBuf;
    trans[0].length = iLen * 8; // Length in bits
    trans[0].rxlength = 0; // defaults to the same length as tx length

    // Queue the transaction
//    ret = spi_device_polling_transmit(spi, &t);
    transfer_is_done = false;
    ret = spi_device_queue_trans(spi, &trans[0], portMAX_DELAY);
    assert (ret==ESP_OK);
//    iFirst = 0;
    return;
} /* spilcdWriteDataDMA() */

//
// Smooth expanded text
//
//  A    --\ 1 2
//C P B  --/ 3 4
//  D
// 1=P; 2=P; 3=P; 4=P;
// IF C==A AND C!=D AND A!=B => 1=A
// IF A==B AND A!=C AND B!=D => 2=B
// IF B==D AND B!=A AND D!=C => 4=D
// IF D==C AND D!=B AND C!=A => 3=C
void SmoothImg(uint16_t *pSrc, uint16_t *pDest, int iSrcPitch, int iDestPitch, int iWidth, int iHeight)
{
int x, y;
unsigned short *s,*s2;
uint32_t *d,*d2;
unsigned short A, B, C, D, P;
uint32_t ulPixel, ulPixel2;

// copy the edge pixels as-is
// top+bottom lines first
s = pSrc;
s2 = (unsigned short *)&pSrc[(iHeight-1)*iSrcPitch];
d = (uint32_t *)pDest;
d2 = (uint32_t *)&pDest[(iHeight-1)*2*iDestPitch];
for (x=0; x<iWidth; x++)
{
      ulPixel = *s++;
      ulPixel2 = *s2++;
      ulPixel |= (ulPixel << 16); // simply double it
      ulPixel2 |= (ulPixel2 << 16);
      d[0] = ulPixel;
      d[iDestPitch/2] = ulPixel;
      d2[0] = ulPixel2;
      d2[iDestPitch/2] = ulPixel2;
      d++; d2++;
}
for (y=1; y<iHeight-1; y++)
  {
  s = (unsigned short *)&pSrc[y * iSrcPitch];
  d = (uint32_t *)&pDest[(y * 2 * iDestPitch)];
// first pixel is just stretched
  ulPixel = *s++;
  ulPixel |= (ulPixel << 16);
  d[0] = ulPixel;
  d[iDestPitch/2] = ulPixel;
  d++;
  for (x=1; x<iWidth-1; x++)
     {
     A = s[-iSrcPitch];
     C = s[-1];
     P = s[0];
     B = s[1];
     D = s[iSrcPitch];
     if (C==A && C!=D && A!=B)
        ulPixel = A;
     else
        ulPixel = P;
     if (A==B && A!=C && B!=D)
        ulPixel |= (B << 16);
     else
        ulPixel |= (P << 16);
     d[0] = ulPixel;
     if (D==C && D!=B && C!=A)
        ulPixel = C;
     else
        ulPixel = P;
     if (B==D && B!=A && D!=C)
        ulPixel |= (D << 16);
     else
        ulPixel |= (P << 16);
     d[iDestPitch/2] = ulPixel;
     d++;
     s++;
     } // for x
// last pixel is just stretched
  ulPixel = s[0];
  ulPixel |= (ulPixel << 16);
  d[0] = ulPixel;
  d[iDestPitch/2] = ulPixel;
  } // for y
} /* SmoothImg() */
//
// Width is the doubled pixel width
// Convert 1-bpp into 2-bit grayscale
//
static void Scale2Gray(uint8_t *source, int width, int iPitch)
{
    int x;
    uint8_t ucPixels, c, d, *dest;

    dest = source; // write the new pixels over the old to save memory

    for (x=0; x<width/8; x+=2) /* Convert a pair of lines to gray */
    {
        c = source[x];  // first 4x2 block
        d = source[x+iPitch];
        /* two lines of 8 pixels are converted to one line of 4 pixels */
        ucPixels = (ucGray2BPP[(unsigned char)((c & 0xf0) | (d >> 4))] << 4);
        ucPixels |= (ucGray2BPP[(unsigned char)((c << 4) | (d & 0x0f))]);
        *dest++ = ucPixels;
        c = source[x+1];  // next 4x2 block
        d = source[x+iPitch+1];
        ucPixels = (ucGray2BPP[(unsigned char)((c & 0xf0) | (d >> 4))])<<4;
        ucPixels |= ucGray2BPP[(unsigned char)((c << 4) | (d & 0x0f))];
        *dest++ = ucPixels;
    }
    if (width & 4) // 2 more pixels to do
    {
        c = source[x];
        d = source[x + iPitch];
        ucPixels = (ucGray2BPP[(unsigned char) ((c & 0xf0) | (d >> 4))]) << 4;
        ucPixels |= (ucGray2BPP[(unsigned char) ((c << 4) | (d & 0x0f))]);
        dest[0] = ucPixels;
    }
} /* Scale2Gray() */


//
// Draw a string of characters in a custom font antialiased
// at 1/2 its original size
// A back buffer must be defined
//
int spilcdWriteStringAntialias(SPILCD *pLCD, GFXfont *pFont, int x, int y, char *szMsg, uint16_t usFGColor, uint16_t usBGColor, int iFlags)
{
	int i, end_y, cx, dx, dy, tx, ty, c, iBitOff;
	uint8_t *s, *d, bits, /*ucMask,*/ ucClr, uc;
	GFXfont font;
	GFXglyph glyph, *pGlyph;
	const uint32_t ulClrMask = 0x07E0F81F;
	uint32_t ulFG, ulBG;
	uint8_t ucTemp[64]; // enough space for a 256 pixel wide font
	uint16_t usTemp[128];

   if (pLCD == NULL || pFont == NULL)
      return -1;
    if (x == -1)
        x = pLCD->iCursorX;
    if (y == -1)
        y = pLCD->iCursorY;
    if (x < 0)
        return -1;
    // Prepare the foreground and background colors for alpha calculations
    ulFG = usFGColor | ((uint32_t)usFGColor << 16);
    ulBG = usBGColor | ((uint32_t)usBGColor << 16);
    ulFG &= ulClrMask; ulBG &= ulClrMask;
   // in case of running on Harvard CPU, get copy of data from FLASH
   memcpy(&font, pFont, sizeof(font));
   pGlyph = &glyph;

   i = 0;
   while (szMsg[i] && x < pLCD->iCurrentWidth)
   {
      c = szMsg[i++];
      if (c < font.first || c > font.last) // undefined character
         continue; // skip it
      c -= font.first; // first char of font defined
      memcpy(&glyph, &font.glyph[c], sizeof(glyph));
      dx = x + pGlyph->xOffset/2; // offset from character UL to start drawing
       cx = (pGlyph->width+1)/2;
       if (dx+cx > pLCD->iCurrentWidth)
           cx = pLCD->iCurrentWidth - dx;
      dy = y + (pGlyph->yOffset/2);
      s = font.bitmap + pGlyph->bitmapOffset; // start of bitmap data
      // Bitmap drawing loop. Image is MSB first and each pixel is packed next
      // to the next (continuing on to the next character line)
      iBitOff = 0; // bitmap offset (in bits)
      bits = uc = 0; // bits left in this font byte
      end_y = dy + (pGlyph->height+1)/2;
//      if (dy < 0) { // skip these lines
//          iBitOff += (pGlyph->width * (-dy));
//          dy = 0;
//      }
       spilcdSetPosition(pLCD, dx, dy, cx, end_y-dy, iFlags);
       memset(ucTemp, 0, sizeof(ucTemp));
       for (ty=0; ty<pGlyph->height; ty++) {
         d = &ucTemp[(ty & 1) * (sizeof(ucTemp)/2)]; // internal buffer dest
         for (tx=0; tx<pGlyph->width; tx++) {
            if (bits == 0) { // need to read more font data
               uc = s[iBitOff>>3]; // get more font bitmap data
               bits = 8;
               iBitOff += bits;
            } // if we ran out of bits
            if (uc & 0x80) { // set the pixel
                d[(tx>>3)] |= (0x80 >> (tx & 7));
            }
            bits--; // next bit
            uc <<= 1;
         } // for x
           if ((ty & 1) || ty == pGlyph->height-1) {
               uint8_t *pg; // pointer to gray source pixels
               uint16_t *pus = usTemp;
               uint32_t ulAlpha, ulPixel;
               //int j;
               const uint8_t ucClrConvert[4] = {0,5,11,16};
               // Convert this pair of lines to grayscale output
               Scale2Gray(ucTemp, pGlyph->width, sizeof(ucTemp)/2);
               // the Scale2Gray code writes the bits horizontally; crop and convert them for the internal memory format
               pg = ucTemp;
               ucClr = *pg++;
               for (tx=0; tx<cx; tx++) {
                   ulAlpha = ucClrConvert[((ucClr & 0xc0) >> 6)]; // 0-3 scaled from 0 to 100% in thirds
                   ulPixel = ((ulFG * ulAlpha) + (ulBG * (16-ulAlpha))) >> 4;
                   ulPixel &= ulClrMask; // separate the RGBs
                   ulPixel |= (ulPixel >> 16); // bring G back to RB
                   *pus++ = __builtin_bswap16(ulPixel); // final pixel
                   ucClr <<= 2;
                   if ((tx & 3) == 3)
                       ucClr = *pg++; // get 4 more pixels
               }
               myspiWrite(pLCD, (uint8_t *)usTemp, cx*sizeof(uint16_t), MODE_DATA, iFlags);
               memset(ucTemp, 0, sizeof(ucTemp));
           }
      } // for y
      x += pGlyph->xAdvance/2; // width of this character
   } // while drawing characters
    pLCD->iCursorX = x;
    pLCD->iCursorY = y;
   return 0;
} /* spilcdWriteStringAntialias() */

//
// Draw a string in a proportional font you supply
//
int spilcdWriteStringCustom(SPILCD *pLCD, GFXfont *pFont, int x, int y, char *szMsg, uint16_t usFGColor, uint16_t usBGColor, int bBlank, int iFlags)
{
	int i, /*j, iLen, */ k, dx, dy, cx, cy, c, iBitOff;
	int tx, ty;
	uint8_t *s, bits, uc;
	GFXfont font;
	GFXglyph glyph, *pGlyph;
	#define TEMP_BUF_SIZE 64
	#define TEMP_HIGHWATER (TEMP_BUF_SIZE-8)
	uint16_t *d, u16Temp[TEMP_BUF_SIZE];

   if (pFont == NULL)
      return -1;
    if (x == -1)
        x = pLCD->iCursorX;
    if (y == -1)
        y = pLCD->iCursorY;
    if (x < 0)
        return -1;
   // in case of running on AVR, get copy of data from FLASH
   memcpy(&font, pFont, sizeof(font));
   pGlyph = &glyph;
   usFGColor = (usFGColor >> 8) | (usFGColor << 8); // swap h/l bytes
   usBGColor = (usBGColor >> 8) | (usBGColor << 8);

   i = 0;
   while (szMsg[i] && x < pLCD->iCurrentWidth)
   {
      c = szMsg[i++];
      if (c < font.first || c > font.last) // undefined character
         continue; // skip it
      c -= font.first; // first char of font defined
      memcpy(&glyph, &font.glyph[c], sizeof(glyph));
      // set up the destination window (rectangle) on the display
      dx = x + pGlyph->xOffset; // offset from character UL to start drawing
      dy = y + pGlyph->yOffset;
      cx = pGlyph->width;
      cy = pGlyph->height;
      iBitOff = 0; // bitmap offset (in bits)
      if (dy + cy > pLCD->iCurrentHeight)
         cy = pLCD->iCurrentHeight - dy; // clip bottom edge
      else if (dy < 0) {
         cy += dy;
         iBitOff += (pGlyph->width * (-dy));
         dy = 0;
      }
      if (dx + cx > pLCD->iCurrentWidth)
         cx = pLCD->iCurrentWidth - dx; // clip right edge
      s = font.bitmap + pGlyph->bitmapOffset; // start of bitmap data
      // Bitmap drawing loop. Image is MSB first and each pixel is packed next
      // to the next (continuing on to the next character line)
      bits = uc = 0; // bits left in this font byte

      if (bBlank) { // erase the areas around the char to not leave old bits
         int miny, maxy;
         c = '0' - font.first;
         miny = y + pGlyph->yOffset;
         c = 'y' - font.first;
         maxy = miny + pGlyph->height;
         if (maxy > pLCD->iCurrentHeight)
            maxy = pLCD->iCurrentHeight;
         cx = pGlyph->xAdvance;
         if (cx + x > pLCD->iCurrentWidth) {
            cx = pLCD->iCurrentWidth - x;
         }
         spilcdSetPosition(pLCD, x, miny, cx, maxy-miny, iFlags);
            // blank out area above character
//            cy = font.yAdvance - pGlyph->height;
//            for (ty=miny; ty<miny+cy && ty < maxy; ty++) {
//               for (tx=0; tx<cx; tx++)
//                  u16Temp[tx] = usBGColor;
//               myspiWrite(pLCD, (uint8_t *)u16Temp, cx*sizeof(uint16_t), MODE_DATA, iFlags);
//            } // for ty
            // character area (with possible padding on L+R)
            for (ty=0; ty<pGlyph->height && ty+miny < maxy; ty++) {
               d = &u16Temp[0];
               for (tx=0; tx<pGlyph->xOffset && tx < cx; tx++) { // left padding
                  *d++ = usBGColor;
               }
            // character bitmap (center area)
               for (tx=0; tx<pGlyph->width; tx++) {
                  if (bits == 0) { // need more data
                     uc = s[iBitOff>>3];
                     bits = 8;
                     iBitOff += bits;
                  }
                  if (tx + pGlyph->xOffset < cx) {
                     *d++ = (uc & 0x80) ? usFGColor : usBGColor;
                  }
                  bits--;
                  uc <<= 1;
               } // for tx
               // right padding
               k = pGlyph->xAdvance - (int)(d - u16Temp); // remaining amount
               for (tx=0; tx<k && (tx+pGlyph->xOffset+pGlyph->width) < cx; tx++)
                  *d++ = usBGColor;
               myspiWrite(pLCD, (uint8_t *)u16Temp, cx*sizeof(uint16_t), MODE_DATA, iFlags);
            } // for ty
            // padding below the current character
            ty = y + pGlyph->yOffset + pGlyph->height;
            for (; ty < maxy; ty++) {
               for (tx=0; tx<cx; tx++)
                  u16Temp[tx] = usBGColor;
               myspiWrite(pLCD, (uint8_t *)u16Temp, cx*sizeof(uint16_t), MODE_DATA, iFlags);
            } // for ty
      } else if (usFGColor == usBGColor) { // transparent
          int iCount; // opaque pixel count
          d = u16Temp;
          for (iCount=0; iCount < cx; iCount++)
              d[iCount] = usFGColor; // set up a line of solid color
          iCount = 0; // number of sequential opaque pixels
             for (ty=0; ty<cy; ty++) {
             for (tx=0; tx<pGlyph->width; tx++) {
                if (bits == 0) { // need to read more font data
                   uc = s[iBitOff>>3]; // get more font bitmap data
                   bits = 8 - (iBitOff & 7); // we might not be on a byte boundary
                   iBitOff += bits; // because of a clipped line
                   uc <<= (8-bits);
                } // if we ran out of bits
                if (tx < cx) {
                    if (uc & 0x80) {
                        iCount++; // one more opaque pixel
                    } else { // any opaque pixels to write?
                        if (iCount) {
                            spilcdSetPosition(pLCD, dx+tx-iCount, dy+ty, iCount, 1, iFlags);
                       d = &u16Temp[0]; // point to start of output buffer
                            myspiWrite(pLCD, (uint8_t *)u16Temp, iCount*sizeof(uint16_t), MODE_DATA, iFlags);
                            iCount = 0;
                        } // if opaque pixels to write
                    } // if transparent pixel hit
                }
                bits--; // next bit
                uc <<= 1;
             } // for tx
             } // for ty
       // quicker drawing
      } else { // just draw the current character box fast
         spilcdSetPosition(pLCD, dx, dy, cx, cy, iFlags);
            d = &u16Temp[0]; // point to start of output buffer
            for (ty=0; ty<cy; ty++) {
            for (tx=0; tx<pGlyph->width; tx++) {
               if (bits == 0) { // need to read more font data
                  uc = s[iBitOff>>3]; // get more font bitmap data
                  bits = 8 - (iBitOff & 7); // we might not be on a byte boundary
                  iBitOff += bits; // because of a clipped line
                  uc <<= (8-bits);
                  k = (int)(d-u16Temp); // number of words in output buffer
                  if (k >= TEMP_HIGHWATER) { // time to write it
                     myspiWrite(pLCD, (uint8_t *)u16Temp, k*sizeof(uint16_t), MODE_DATA, iFlags);
                     d = &u16Temp[0];
                  }
               } // if we ran out of bits
               if (tx < cx) {
                  *d++ = (uc & 0x80) ? usFGColor : usBGColor;
               }
               bits--; // next bit
               uc <<= 1;
            } // for tx
            } // for ty
            k = (int)(d-u16Temp);
            if (k) // write any remaining data
               myspiWrite(pLCD, (uint8_t *)u16Temp, k*sizeof(uint16_t), MODE_DATA, iFlags);
      } // quicker drawing
      x += pGlyph->xAdvance; // width of this character
   } // while drawing characters
    pLCD->iCursorX = x;
    pLCD->iCursorY = y;
   return 0;
} /* spilcdWriteStringCustom() */

//
// Get the width of text in a custom font
//
void spilcdGetStringBox(GFXfont *pFont, char *szMsg, int *width, int *top, int *bottom)
{
int cx = 0;
int c, i = 0;
GFXfont font;
GFXglyph glyph, *pGlyph;
int miny, maxy;

   if (pFont == NULL)
      return;
   // in case of running on AVR, get copy of data from FLASH
   memcpy(&font, pFont, sizeof(font));
   pGlyph = &glyph;
   if (width == NULL || top == NULL || bottom == NULL || pFont == NULL || szMsg == NULL) return; // bad pointers
   miny = 1000; maxy = 0;
   while (szMsg[i]) {
      c = szMsg[i++];
      if (c < font.first || c > font.last) // undefined character
         continue; // skip it
      c -= font.first; // first char of font defined
      memcpy(&glyph, &font.glyph[c], sizeof(glyph));
      cx += pGlyph->xAdvance;
      if (pGlyph->yOffset < miny) miny = pGlyph->yOffset;
      if (pGlyph->height+pGlyph->yOffset > maxy) maxy = pGlyph->height+pGlyph->yOffset;
   }
   *width = cx;
   *top = miny;
   *bottom = maxy;
} /* spilcdGetStringBox() */

//
// Draw a string of text as quickly as possible
//
int spilcdWriteStringFast(SPILCD *pLCD, int x, int y, char *szMsg, unsigned short usFGColor, unsigned short usBGColor, int iFontSize, int iFlags)
{
	int i, j, k, iLen;
	int iStride;
	uint8_t *s;
	uint16_t usFG = (usFGColor >> 8) | ((usFGColor & -1)<< 8);
	uint16_t usBG = (usBGColor >> 8) | ((usBGColor & -1)<< 8);
	uint16_t *usD;
	int cx;
	uint8_t *pFont;

    if (iFontSize != FONT_6x8 && iFontSize != FONT_8x8 && iFontSize != FONT_12x16)
        return -1; // invalid size
    if (x == -1)
        x = pLCD->iCursorX;
    if (y == -1)
        y = pLCD->iCursorY;
    if (x < 0) return -1;

    if (iFontSize == FONT_12x16) {
        iLen = strlen(szMsg);
        if ((12*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/12; // can't display it all
        if (iLen < 0) return -1;
        iStride = iLen*12;
        spilcdSetPosition(pLCD, x, y, iStride, 16, iFlags);
        for (k = 0; k<8; k++) { // create a pair of scanlines from each original
           uint8_t ucMask = (1 << k);
           usD = (unsigned short *)&ucRXBuf[0];
           for (i=0; i<iStride*2; i++)
              usD[i] = usBG; // set to background color first
           for (i=0; i<iLen; i++)
           {
               uint8_t c0, c1;
               s = (uint8_t *)&ucSmallFont[((unsigned char)szMsg[i]-32) * 6];
               for (j=1; j<6; j++)
               {
                   uint8_t ucMask1 = ucMask << 1;
                   uint8_t ucMask2 = ucMask >> 1;
                   c0 = s[j];
                   if (c0 & ucMask)
                      usD[0] = usD[1] = usD[iStride] = usD[iStride+1] = usFG;
                   // test for smoothing diagonals
                   if (j < 5) {
                      c1 = s[j+1];
                      if ((c0 & ucMask) && (~c1 & ucMask) && (~c0 & ucMask1) && (c1 & ucMask1)) { // first diagonal condition
                          usD[iStride+2] = usFG;
                      } else if ((~c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask1) && (~c1 & ucMask1)) { // second condition
                          usD[iStride+1] = usFG;
                      }
                      if ((c0 & ucMask2) && (~c1 & ucMask2) && (~c0 & ucMask) && (c1 & ucMask)) { // repeat for previous line
                          usD[1] = usFG;
                      } else if ((~c0 & ucMask2) && (c1 & ucMask2) && (c0 & ucMask) && (~c1 & ucMask)) {
                          usD[2] = usFG;
                      }
                   }
                   usD+=2;
               } // for j
               usD += 2; // leave "6th" column blank
            } // for each character
            myspiWrite(pLCD, ucRXBuf, iStride*4, MODE_DATA, iFlags);
        } // for each scanline
        return 0;
    } // 12x16

    cx = (iFontSize == FONT_8x8) ? 8:6;
    pFont = (iFontSize == FONT_8x8) ? (uint8_t *)ucFont : (uint8_t *)ucSmallFont;
    iLen = strlen(szMsg);
	if (iLen <=0) return -1; // can't use this function

    if ((cx*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/cx; // can't display it all
    if (iLen > 32) iLen = 32;
    iStride = iLen * cx*2;
    for (i=0; i<iLen; i++)
    {
        s = &pFont[((unsigned char)szMsg[i]-32) * cx];
        uint8_t ucMask = 1;
        for (k=0; k<8; k++) // for each scanline
        {
            usD = (unsigned short *)&ucRXBuf[(k*iStride) + (i * cx*2)];
            for (j=0; j<cx; j++)
            {
                if (s[j] & ucMask)
                    *usD++ = usFG;
                else
                    *usD++ = usBG;
            } // for j
            ucMask <<= 1;
        } // for k
    } // for i
    // write the data in one shot
    spilcdSetPosition(pLCD, x, y, cx*iLen, 8, iFlags);
    myspiWrite(pLCD, ucRXBuf, iLen*cx*16, MODE_DATA, iFlags);
    pLCD->iCursorX = x + (cx*iLen);
    pLCD->iCursorY = y;
	return 0;
} /* spilcdWriteStringFast() */



//
// Draw a string of small (8x8) or large (16x32) characters
// At the given col+row
//
int spilcdWriteString(SPILCD *pLCD, int x, int y, char *szMsg, int usFGColor, int usBGColor, int iFontSize, int iFlags)
	{
	int i, j, k, iLen;
	#ifndef __AVR__
	int l;
	#endif
	unsigned char *s;
	unsigned short usFG = (usFGColor >> 8) | (usFGColor << 8);
	unsigned short usBG = (usBGColor >> 8) | (usBGColor << 8);
	uint16_t usPitch = pLCD->iScreenPitch/2;


    if (x == -1)
        x = pLCD->iCursorX;
    if (y == -1)
        y = pLCD->iCursorY;
    if (x < 0 || y < 0) return -1;
	iLen = strlen(szMsg);
    if (usBGColor == -1)
        iFlags = DRAW_TO_RAM; // transparent text doesn't get written to the display
    if (usFG == usBG) usBG = 0; // DEBUG!! no transparent option for now

	if (iFontSize == FONT_16x32) // draw 16x32 font
	{
		if (iLen*16 + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x) / 16;
		if (iLen < 0) return -1;
		for (i=0; i<iLen; i++)
		{
			uint16_t *usD, *usTemp = (uint16_t *)ucRXBuf;
			s = (uint8_t *)&ucBigFont[((unsigned char)szMsg[i]-32)*64];
			usD = &usTemp[0];
            if (usBGColor == -1) // transparent text is not rendered to the display
               iFlags = DRAW_TO_RAM;
            spilcdSetPosition(pLCD, x+(i*16), y,16,32, iFlags);
            for (l=0; l<4; l++) // 4 sets of 8 rows
            {
                uint8_t ucMask = 1;
                for (k=0; k<8; k++) // for each scanline
                { // left half
                    if (usBGColor == -1) // transparent text
                    {
                        uint16_t *d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + ((l*8+k)*pLCD->iScreenPitch)];
                        for (j=0; j<16; j++)
                        {
                            if (s[j] & ucMask)
                                *d = usFG;
                            d++;
                        } // for j
                    }
                    else
                    {
                        for (j=0; j<16; j++)
                        {
                            if (s[j] & ucMask)
                                *usD++ = usFG;
                            else
                                *usD++ = usBG;
                        } // for j
                    }
                    ucMask <<= 1;
                } // for each scanline
                s += 16;
            } // for each set of 8 scanlines
        if (usBGColor != -1) // don't write anything if we're doing transparent text
            myspiWrite(pLCD, (unsigned char *)usTemp, 1024, MODE_DATA, iFlags);
		} // for each character
        x += (i*16);
    }

    if (iFontSize == FONT_8x8 || iFontSize == FONT_6x8) // draw the 6x8 or 8x8 font
	{
		uint16_t *usD, *usTemp = (uint16_t *)ucRXBuf;
        int cx;
        uint8_t *pFont;

        cx = (iFontSize == FONT_8x8) ? 8:6;
        pFont = (iFontSize == FONT_8x8) ? (uint8_t *)ucFont : (uint8_t *)ucSmallFont;
		if ((cx*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/cx; // can't display it all
		if (iLen < 0)return -1;

		for (i=0; i<iLen; i++)
		{
			s = &pFont[((unsigned char)szMsg[i]-32) * cx];
			usD = &usTemp[0];
            spilcdSetPosition(pLCD, x+(i*cx), y, cx, 8, iFlags);
            uint8_t ucMask = 1;
            for (k=0; k<8; k++) // for each scanline
            {
                if (usBGColor == -1) // transparent text
                {
                    usD = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (k * pLCD->iScreenPitch)];
                    for (j=0; j<cx; j++)
                    {
                        if (s[j] & ucMask)
                            *usD = usFG;
                        usD++;
                    } // for j
                }
                else // regular text
                {
                    for (j=0; j<cx; j++)
                    {
                        if (s[j] & ucMask)
                            *usD++ = usFG;
                        else
                            *usD++ = usBG;
                    } // for j
                }
                ucMask <<= 1;
            } // for k
    // write the data in one shot
        if (usBGColor != -1) // don't write anything if we're doing transparent text
            myspiWrite(pLCD, (unsigned char *)usTemp, cx*16, MODE_DATA, iFlags);
		}
        x += (i*cx);
    } // 6x8 and 8x8
    if (iFontSize == FONT_12x16) // 6x8 stretched to 12x16 (with smoothing)
    {
        uint16_t *usD, *usTemp = (uint16_t *)ucRXBuf;

        if ((12*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/12; // can't display it all
        if (iLen < 0)return -1;

        for (i=0; i<iLen; i++)
        {
            s = (uint8_t *)&ucSmallFont[((unsigned char)szMsg[i]-32) * 6];
            usD = &usTemp[0];
            spilcdSetPosition(pLCD, x+(i*12), y, 12, 16, iFlags);
            uint8_t ucMask = 1;
            if (usBGColor != -1) // start with all BG color
            {
               for (k=0; k<12*16; k++)
                  usD[k] = usBG;
            }
            for (k=0; k<8; k++) // for each scanline
            {
                if (usBGColor == -1) // transparent text
                {
                    uint8_t c0, c1;
                    usD = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (k*2*pLCD->iScreenPitch)];
                    for (j=0; j<6; j++)
                    {
                        c0 = s[j];
                        if (c0 & ucMask)
                            usD[0] = usD[1] = usD[usPitch] = usD[usPitch+1] = usFG;
                        // test for smoothing diagonals
                        if (k < 7 && j < 5) {
                           uint8_t ucMask2 = ucMask << 1;
                           c1 =s[j+1];
                           if ((c0 & ucMask) && (~c1 & ucMask) && (~c0 & ucMask2) && (c1 & ucMask2)) // first diagonal condition
                               usD[usPitch+2] = usD[2*usPitch+1] = usFG;
                           else if ((~c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask2) && (~c1 & ucMask2))
                               usD[usPitch+1] = usD[2*usPitch+2] = usFG;
                        } // if not on last row and last col
                        usD += 2;
                    } // for j
                }
                else // regular text drawing
                {
                    uint8_t c0, c1;
                    for (j=0; j<6; j++)
                    {
                        c0 = s[j];
                        if (c0 & ucMask)
                           usD[0] = usD[1] = usD[12] = usD[13] = usFG;
                        // test for smoothing diagonals
                        if (k < 7 && j < 5) {
                           uint8_t ucMask2 = ucMask << 1;
                           c1 = s[j+1];
                           if ((c0 & ucMask) && (~c1 & ucMask) && (~c0 & ucMask2) && (c1 & ucMask2)) // first diagonal condition
                               usD[14] = usD[25] = usFG;
                           else if ((~c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask2) && (~c1 & ucMask2))
                               usD[13] = usD[26] = usFG;
                        } // if not on last row and last col
                        usD+=2;
                    } // for j
                }
                usD += 12; // skip the extra line
                ucMask <<= 1;
            } // for k
        // write the data in one shot
        if (usBGColor != -1) // don't write anything if we're doing transparent text
            myspiWrite(pLCD, (unsigned char *)&usTemp[0], 12*16*2, MODE_DATA, iFlags);
        }
        x += i*12;
    } // FONT_12x16
    if (iFontSize == FONT_16x16) // 8x8 stretched to 16x16
    {
        uint16_t *usD, *usTemp = (uint16_t *)ucRXBuf;

        if ((16*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/16; // can't display it all
        if (iLen < 0)return -1;

        for (i=0; i<iLen; i++)
        {
            s = (uint8_t *)&ucFont[((unsigned char)szMsg[i]-32) * 8];
            usD = &usTemp[0];
            spilcdSetPosition(pLCD, x+(i*16), y, 16, 16, iFlags);
            uint8_t ucMask = 1;
            if (usBGColor != -1) // start with all BG color
            {
               for (k=0; k<256; k++)
                  usD[k] = usBG;
            }
            for (k=0; k<8; k++) // for each scanline
            {
                if (usBGColor == -1) // transparent text
                {
                    usD = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (k*2*pLCD->iScreenPitch)];
                    for (j=0; j<8; j++)
                    {
                        if (s[j] & ucMask)
                            usD[0] = usD[1] = usD[usPitch] = usD[usPitch+1] = usFG;
                        usD += 2;
                    } // for j
                }
                else // regular text drawing
                {
                    uint8_t c0;
                    for (j=0; j<8; j++)
                    {
                        c0 = s[j];
                        if (c0 & ucMask)
                        usD[0] = usD[1] = usD[16] = usD[17] = usFG;
                        usD+=2;
                    } // for j
                }
                usD += 16; // skip the next line
                ucMask <<= 1;
            } // for k
        // write the data in one shot
        if (usBGColor != -1) // don't write anything if we're doing transparent text
            myspiWrite(pLCD, (unsigned char *)&usTemp[0], 512, MODE_DATA, iFlags);
        }
        x += (i*16);
    } // FONT_16x16
    pLCD->iCursorX = x;
    pLCD->iCursorY = y;
	return 0;
} /* spilcdWriteString() */

//
// For drawing ellipses, a circle is drawn and the x and y pixels are scaled by a 16-bit integer fraction
// This function draws a single pixel and scales its position based on the x/y fraction of the ellipse
//
void DrawScaledPixel(SPILCD *pLCD, int32_t iCX, int32_t iCY, int32_t x, int32_t y, int32_t iXFrac, int32_t iYFrac, unsigned short usColor, int iFlags)
{
    uint8_t ucBuf[2];
    if (iXFrac != 0x10000) x = (x * iXFrac) >> 16;
    if (iYFrac != 0x10000) y = (y * iYFrac) >> 16;
    x += iCX; y += iCY;
    if (x < 0 || x >= pLCD->iCurrentWidth || y < 0 || y >= pLCD->iCurrentHeight)
        return; // off the screen
    ucBuf[0] = (uint8_t)(usColor >> 8);
    ucBuf[1] = (uint8_t)usColor;
    spilcdSetPosition(pLCD, x, y, 1, 1, iFlags);
    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
} /* DrawScaledPixel() */

void DrawScaledLine(SPILCD *pLCD, int32_t iCX, int32_t iCY, int32_t x, int32_t y, int32_t iXFrac, int32_t iYFrac, uint16_t *pBuf, int iFlags)
{
    int32_t iLen, x2;
    if (iXFrac != 0x10000) x = (x * iXFrac) >> 16;
    if (iYFrac != 0x10000) y = (y * iYFrac) >> 16;
    iLen = x * 2;
    x = iCX - x; y += iCY;
    x2 = x + iLen;
    if (y < 0 || y >= pLCD->iCurrentHeight)
        return; // completely off the screen
    if (x < 0) x = 0;
    if (x2 >= pLCD->iCurrentWidth) x2 = pLCD->iCurrentWidth-1;
    iLen = x2 - x + 1; // new length
    spilcdSetPosition(pLCD, x, y, iLen, 1, iFlags);
//#ifdef ESP32_DMA
//    myspiWrite(pLCD, (uint8_t*)pBuf, iLen*2, MODE_DATA, iFlags);
//#else
    // need to refresh the output data each time
    {
    int i;
    unsigned short us = pBuf[0];
      for (i=1; i<iLen; i++)
        pBuf[i] = us;
    }
    myspiWrite(pLCD, (uint8_t*)&pBuf[1], iLen*2, MODE_DATA, iFlags);
//#endif
} /* DrawScaledLine() */

//
// Draw the 8 pixels around the Bresenham circle
// (scaled to make an ellipse)
//
void BresenhamCircle(SPILCD *pLCD, int32_t iCX, int32_t iCY, int32_t x, int32_t y, int32_t iXFrac, int32_t iYFrac, uint16_t iColor, uint16_t *pFill, uint8_t u8Parts, int iFlags)
{
    if (pFill != NULL) // draw a filled ellipse
    {
        static int prev_y = -1;
        // for a filled ellipse, draw 4 lines instead of 8 pixels
        DrawScaledLine(pLCD, iCX, iCY, y, x, iXFrac, iYFrac, pFill, iFlags);
        DrawScaledLine(pLCD, iCX, iCY, y, -x, iXFrac, iYFrac, pFill, iFlags);
        if (y != prev_y) {
            DrawScaledLine(pLCD, iCX, iCY, x, y, iXFrac, iYFrac, pFill, iFlags);
            DrawScaledLine(pLCD, iCX, iCY, x, -y, iXFrac, iYFrac, pFill, iFlags);
            prev_y = y;
        }
    }
    else // draw 8 pixels around the edges
    {
        if (u8Parts & 1) {
            DrawScaledPixel(pLCD, iCX, iCY, -x, -y, iXFrac, iYFrac, iColor, iFlags);
            DrawScaledPixel(pLCD, iCX, iCY, -y, -x, iXFrac, iYFrac, iColor, iFlags);
        }
        if (u8Parts & 2) {
            DrawScaledPixel(pLCD, iCX, iCY, x, -y, iXFrac, iYFrac, iColor, iFlags);
            DrawScaledPixel(pLCD, iCX, iCY, y, -x, iXFrac, iYFrac, iColor, iFlags);
        }
        if (u8Parts & 4) {
            DrawScaledPixel(pLCD, iCX, iCY, y, x, iXFrac, iYFrac, iColor, iFlags);
            DrawScaledPixel(pLCD, iCX, iCY, x, y, iXFrac, iYFrac, iColor, iFlags);
        }
        if (u8Parts & 8) {
            DrawScaledPixel(pLCD, iCX, iCY, -y, x, iXFrac, iYFrac, iColor, iFlags);
            DrawScaledPixel(pLCD, iCX, iCY, -x, y, iXFrac, iYFrac, iColor, iFlags);
        }
    }
} /* BresenhamCircle() */

void spilcdEllipse(SPILCD *pLCD, int32_t iCenterX, int32_t iCenterY, int32_t iRadiusX, int32_t iRadiusY, uint8_t u8Parts, unsigned short usColor, int bFilled, int iFlags)
{
    int32_t iRadius, iXFrac, iYFrac;
    int32_t iDelta, x, y;
    uint16_t us, *pus, *usTemp = (uint16_t *)ucRXBuf; // up to 320 pixels wide

    if (iCenterX < 0 || iCenterY < 0 || iRadiusX <= 0 || iRadiusY <= 0) return;

    if (iRadiusX > iRadiusY) // use X as the primary radius
    {
        iRadius = iRadiusX;
        iXFrac = 65536;
        iYFrac = (iRadiusY * 65536) / iRadiusX;
    }
    else
    {
        iRadius = iRadiusY;
        iXFrac = (iRadiusX * 65536) / iRadiusY;
        iYFrac = 65536;
    }
    // set up a buffer with the widest possible run of pixels to dump in 1 shot
    if (bFilled)
    {
        us = (usColor >> 8) | (usColor << 8); // swap byte order
        y = iRadius*2;
        if (y > 320) y = 320; // max size
//#ifdef ESP32_DMA
        for (x=0; x<y; x++)
        {
            usTemp[x] = us;
        }
//#else
//	usTemp[0] = us; // otherwise just set the first one to the color
//#endif
        pus = usTemp;
    }
    else
    {
        pus = NULL;
    }
    iDelta = 3 - (2 * iRadius);
    x = 0; y = iRadius;
    while (x < y)
    {
        BresenhamCircle(pLCD, iCenterX, iCenterY, x, y, iXFrac, iYFrac, usColor, pus, u8Parts, iFlags);
        x++;
        if (iDelta < 0)
        {
            iDelta += (4*x) + 6;
        }
        else
        {
            iDelta += 4 * (x-y) + 10;
            y--;
        }
    }

} /* spilcdEllipse() */

//
// Set the (software) orientation of the display
// The hardware is permanently oriented in 240x320 portrait mode
// The library can draw characters/tiles rotated 90
// degrees if set into landscape mode
//
int spilcdSetOrientation(SPILCD *pLCD, int iOrient)
{
int bX=0, bY=0, bV=0;

    pLCD->iOrientation = iOrient;
    // Make sure next setPos() resets both x and y
    pLCD->iOldX=-1; pLCD->iOldY=-1; pLCD->iOldCX=-1; pLCD->iOldCY = -1;
   switch(iOrient)
   {
     case LCD_ORIENTATION_0:
           bX = bY = bV = 0;
           pLCD->iMemoryX = pLCD->iColStart;
           pLCD->iMemoryY = pLCD->iRowStart;
           pLCD->iCurrentHeight = pLCD->iHeight;
           pLCD->iCurrentWidth = pLCD->iWidth;
        break;
     case LCD_ORIENTATION_90:
        bX = bV = 1;
        bY = 0;
           pLCD->iMemoryX = pLCD->iRowStart;
           pLCD->iMemoryY = pLCD->iColStart;
           pLCD->iCurrentHeight = pLCD->iWidth;
           pLCD->iCurrentWidth = pLCD->iHeight;
        break;
     case LCD_ORIENTATION_180:
        bX = bY = 1;
        bV = 0;
        if (pLCD->iColStart != 0)
            pLCD->iMemoryX = pLCD->iColStart;// - 1;
           pLCD->iMemoryY = pLCD->iRowStart;
           pLCD->iCurrentHeight = pLCD->iHeight;
           pLCD->iCurrentWidth = pLCD->iWidth;
        break;
     case LCD_ORIENTATION_270:
        bY = bV = 1;
        bX = 0;
           pLCD->iMemoryX = pLCD->iRowStart;
           pLCD->iMemoryY = pLCD->iColStart;
           pLCD->iCurrentHeight = pLCD->iWidth;
           pLCD->iCurrentWidth = pLCD->iHeight;
        break;
   }
   pLCD->iScreenPitch = pLCD->iCurrentWidth * 2;

    if (pLCD->iCMDType == CMD_TYPE_SITRONIX_8BIT && pLCD->iHeight == 240 && pLCD->iWidth == 240) {
        // special issue with memory offsets in certain orientations
        if (pLCD->iOrientation == LCD_ORIENTATION_180) {
            pLCD->iMemoryX = 0; pLCD->iMemoryY = 80;
        } else if (pLCD->iOrientation == LCD_ORIENTATION_270) {
            pLCD->iMemoryX = 80; pLCD->iMemoryY = 0;
        } else {
            pLCD->iMemoryX = pLCD->iMemoryY = 0;
        }
    }
   if (pLCD->iCMDType == CMD_TYPE_SITRONIX_8BIT)
   {
      uint8_t uc = 0;

      if (bY) uc |= 0x80;
      if (bX) uc |= 0x40;
      if (bV) uc |= 0x20;
       if (pLCD->iLCDType == LCD_ILI9341) {
          uc |= 8; // R/B inverted from other LCDs
           uc ^= 0x40; // x is inverted too
       }
       if (pLCD->iLCDFlags & FLAGS_FLIPX)
           uc ^= 0x40;
      if (pLCD->iLCDFlags & FLAGS_SWAP_RB)
          uc ^= 0x8;
      spilcdWriteCmdParams(pLCD, 0x36, &uc, 1); // MADCTL
   }


    return 0;
} /* spilcdSetOrientation() */

//
// Fill the frame buffer with a single color
//
int spilcdFill(SPILCD *pLCD, unsigned short usData, int iFlags)
{
	int x, y;
	uint16_t *u16Temp = (uint16_t *)ucRXBuf;

    // make sure we're in landscape mode to use the correct coordinates
    spilcdScrollReset(pLCD);
    usData = (usData >> 8) | (usData << 8); // swap hi/lo byte for LCD
    if (pLCD->iLCDFlags & FLAGS_MEM_RESTART) {
        // special case for parllel LCD using ESP32 LCD API
        for (x=0; x<pLCD->iCurrentWidth; x++)
            u16Temp[x] = usData;
        for (y=0; y<pLCD->iCurrentHeight; y++)
        {
            spilcdSetPosition(pLCD, 0,y,pLCD->iCurrentWidth,1, iFlags);
            myspiWrite(pLCD, (uint8_t *)u16Temp, pLCD->iCurrentWidth*2, MODE_DATA, iFlags);
        } // for y
        return 0;
    }
    spilcdSetPosition(pLCD, 0,0,pLCD->iCurrentWidth,pLCD->iCurrentHeight, iFlags);

    for (y=0; y<pLCD->iCurrentHeight; y+=2)
    {
        // MCUs with more RAM can do it faster
        for (x=0; x<pLCD->iCurrentWidth*2; x++) {
            u16Temp[x] = usData; // data will be overwritten
        }
        myspiWrite(pLCD, (uint8_t *)u16Temp, pLCD->iCurrentWidth*4, MODE_DATA, iFlags);
    } // for y

    return 0;
} /* spilcdFill() */

//
// Draw a line between 2 points using Bresenham's algorithm
// An optimized version of the algorithm where each continuous run of pixels is written in a
// single shot to reduce the total number of SPI transactions. Perfectly vertical or horizontal
// lines are the most extreme version of this condition and will write the data in a single
// operation.
//
void spilcdDrawLine(SPILCD *pLCD, int x1, int y1, int x2, int y2, unsigned short usColor, int iFlags)
{
    int temp;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int error;
    int xinc, yinc;
    int iLen, x, y;
//#ifndef ESP32_DMA
    int i;
//#endif
    uint16_t *usTemp = (uint16_t *)ucRXBuf, us;

    if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 || x1 >= pLCD->iCurrentWidth || x2 >= pLCD->iCurrentWidth || y1 >= pLCD->iCurrentHeight || y2 >= pLCD->iCurrentHeight)
        return;
    us = (usColor >> 8) | (usColor << 8); // byte swap for LCD byte order

    if(abs(dx) > abs(dy)) {
        // X major case
        if(x2 < x1) {
            dx = -dx;
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
//#ifdef ESP32_DMA
        for (x=0; x<dx+1; x++) // prepare color data for max length line
            usTemp[x] = us;
//#endif
//        spilcdSetPosition(x1, y1, dx+1, 1); // set the starting position in both X and Y
        y = y1;
        dy = (y2 - y1);
        error = dx >> 1;
        yinc = 1;
        if (dy < 0)
        {
            dy = -dy;
            yinc = -1;
        }
        for(x = x1; x1 <= x2; x1++) {
            error -= dy;
            if (error < 0) // y needs to change, write existing pixels
            {
                error += dx;
		iLen = (x1-x+1);
                spilcdSetPosition(pLCD, x, y, iLen, 1, iFlags);
//#ifndef ESP32_DMA
	        for (i=0; i<iLen; i++) // prepare color data for max length line
                   usTemp[i] = us;
//#endif
                myspiWrite(pLCD, (uint8_t*)usTemp, iLen*2, MODE_DATA, iFlags); // write the row we changed
                y += yinc;
//                spilcdSetPosY(y, 1); // update the y position only
                x = x1+1; // we've already written the pixel at x1
            }
        } // for x1
        if (x != x1) // some data needs to be written
        {
	    iLen = (x1-x+1);
//#ifndef ESP32_DMA
            for (temp=0; temp<iLen; temp++) // prepare color data for max length line
               usTemp[temp] = us;
//#endif
            spilcdSetPosition(pLCD, x, y, iLen, 1, iFlags);
            myspiWrite(pLCD, (uint8_t*)usTemp, iLen*2, MODE_DATA, iFlags); // write the row we changed
        }
    }
    else {
        // Y major case
        if(y1 > y2) {
            dy = -dy;
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
//#ifdef ESP32_DMA
        for (x=0; x<dy+1; x++) // prepare color data for max length line
            usTemp[x] = us;
//#endif
//        spilcdSetPosition(x1, y1, 1, dy+1); // set the starting position in both X and Y
        dx = (x2 - x1);
        error = dy >> 1;
        xinc = 1;
        if (dx < 0)
        {
            dx = -dx;
            xinc = -1;
        }
        x = x1;
        for(y = y1; y1 <= y2; y1++) {
            error -= dx;
            if (error < 0) { // x needs to change, write any pixels we traversed
                error += dy;
                iLen = y1-y+1;
//#ifndef ESP32_DMA
      		for (i=0; i<iLen; i++) // prepare color data for max length line
       		    usTemp[i] = us;
//#endif
                spilcdSetPosition(pLCD, x, y, 1, iLen, iFlags);
                myspiWrite(pLCD, (uint8_t*)usTemp, iLen*2, MODE_DATA, iFlags); // write the row we changed
                x += xinc;
//                spilcdSetPosX(x, 1); // update the x position only
                y = y1+1; // we've already written the pixel at y1
            }
        } // for y
        if (y != y1) // write the last byte we modified if it changed
        {
	    iLen = y1-y+1;
//#ifndef ESP32_DMA
            for (i=0; i<iLen; i++) // prepare color data for max length line
               usTemp[i] = us;
//#endif
            spilcdSetPosition(pLCD, x, y, 1, iLen, iFlags);
            myspiWrite(pLCD, (uint8_t*)usTemp, iLen*2, MODE_DATA, iFlags); // write the row we changed
        }
    } // y major case
} /* spilcdDrawLine() */


//
// Decompress one line of 8-bit RLE data
//
unsigned char * DecodeRLE8(unsigned char *s, int iWidth, uint16_t *d, uint16_t *usPalette)
{
	unsigned char c;
	static unsigned char ucColor, ucRepeat, ucCount;
	int iStartWidth = iWidth;
	long l;

   if (s == NULL) { // initialize
      ucRepeat = 0;
      ucCount = 0;
      return s;
   }

   while (iWidth > 0)
   {
      if (ucCount) // some non-repeating bytes to deal with
      {
         while (ucCount && iWidth > 0)
         {
            ucCount--;
            iWidth--;
            ucColor = *s++;
            *d++ = usPalette[ucColor];
         }
         l = (long)s;
         if (l & 1) s++; // compressed data pointer must always be even
      }
      if (ucRepeat == 0 && iWidth > 0) // get a new repeat code or command byte
      {
         ucRepeat = *s++;
         if (ucRepeat == 0) // command code
         {
            c = *s++;
            switch (c)
            {
               case 0: // end of line
                    if (iStartWidth != iWidth) {
                        return s; // true end of line
                    }
                break; // otherwise do nothing because it was from the last line
               case 1: // end of bitmap
                 return s;
               case 2: // move
                 c = *s++; // debug - delta X
                 d += c; iWidth -= c;
                 c = *s++; // debug - delta Y
                 break;
               default: // uncompressed data
                 ucCount = c;
                 break;
            } // switch on command byte
         }
         else
         {
            ucColor = *s++; // get the new colors
         }
      }
      while (ucRepeat && iWidth > 0)
      {
         ucRepeat--;
         *d++ = usPalette[ucColor];
         iWidth--;
      } // while decoding the current line
   } // while pixels on the current line to draw
   return s;
} /* DecodeRLE8() */

//
// Decompress one line of 4-bit RLE data
//
unsigned char * DecodeRLE4(uint8_t *s, int iWidth, uint16_t *d, uint16_t *usPalette)
{
	uint8_t c, ucOdd=0, ucColor;
	static uint8_t uc1, uc2, ucRepeat, ucCount;
	int iStartWidth = iWidth;

   if (s == NULL) { // initialize this bitmap
      ucRepeat = 0;
      ucCount = 0;
      return s;
   }

   while (iWidth > 0)
   {
      if (ucCount) // some non-repeating bytes to deal with
      {
         while (ucCount > 0 && iWidth > 0)
         {
            ucCount--;
            iWidth--;
            ucColor = *s++;
            uc1 = ucColor >> 4; uc2 = ucColor & 0xf;
            *d++ = usPalette[uc1];
            if (ucCount > 0 && iWidth > 0)
            {
               *d++ = usPalette[uc2];
               ucCount--;
               iWidth--;
            }
         }
         if ((int)(intptr_t)s & 1) {
            s++; // compressed data pointer must always be even
         }
      }
      if (ucRepeat == 0 && iWidth > 0) // get a new repeat code or command byte
      {
         ucRepeat = *s++;
         if (ucRepeat == 0) // command code
         {
            c = *s++;
            switch (c)
            {
               case 0: // end of line
                 if (iStartWidth - iWidth >= 2)
                    return s; // true end of line
                 break; // otherwise do nothing because it was from the last line
               case 1: // end of bitmap
                 return s;
               case 2: // move
                 c = *s++; // debug - delta X
                 d += c; iWidth -= c;
                 c = *s++; // debug - delta Y
                 break;
               default: // uncompressed data
                 ucCount = c;
                 break;
            } // switch on command byte
         }
         else
         {
            ucOdd = 0; // start on an even source pixel
            ucColor = *s++; // get the new colors
            uc1 = ucColor >> 4; uc2 = ucColor & 0xf;
         }
      }
      while (ucRepeat > 0 && iWidth > 0)
      {
         ucRepeat--;
         iWidth--;
         *d++ = (ucOdd) ? usPalette[uc2] : usPalette[uc1];
         ucOdd = !ucOdd;
      } // while decoding the current line
   } // while pixels on the current line to draw
   return s;
} /* DecodeRLE4() */

//
// Draw a 4, 8 or 16-bit Windows uncompressed bitmap onto the display
// Pass the pointer to the beginning of the BMP file
// Optionally stretch to 2x size
// Optimized for drawing to the backbuffer. The transparent color index is only used
// when drawinng to the back buffer. Set it to -1 to disable
// returns -1 for error, 0 for success
//
int spilcdDrawBMP(SPILCD *pLCD, uint8_t *pBMP, int iDestX, int iDestY, int bStretch, int iTransparent, int iFlags)
{
    int iOffBits, iPitch;
    uint16_t usPalette[256];
    uint8_t *pCompressed;
    uint8_t ucCompression;
    int16_t cx, cy, bpp, y; // offset to bitmap data
    int j, x;
    uint16_t *pus, us, *d, *usTemp = (uint16_t *)ucRXBuf; // process a line at a time
    uint8_t bFlipped = false;

    if (pBMP[0] != 'B' || pBMP[1] != 'M') // must start with 'BM'
        return -1; // not a BMP file
    cx = pBMP[18] | pBMP[19]<<8;
    cy = pBMP[22] | pBMP[23]<<8;
    ucCompression = pBMP[30]; // 0 = uncompressed, 1/2/4 = RLE compressed
    if (ucCompression > 4) // unsupported feature
        return -1;
    if (cy > 0) // BMP is flipped vertically (typical)
        bFlipped = true;
    else
        cy = -cy;
    bpp = pBMP[28] | pBMP[29]<<8;
    if (bpp != 16 && bpp != 4 && bpp != 8) // must be 4/8/16 bits per pixel
        return -1;
    if (iDestX + cx > pLCD->iCurrentWidth || iDestX < 0 || cx < 0)
        return -1; // invalid
    if (iDestY + cy > pLCD->iCurrentHeight || iDestY < 0 || cy < 0)
        return -1;
    if (iTransparent != -1) // transparent drawing can only happen on the back buffer
        iFlags = DRAW_TO_RAM;
    iOffBits = pBMP[10] | pBMP[11]<<8;
    iPitch = (cx * bpp) >> 3; // bytes per line
    iPitch = (iPitch + 3) & 0xfffc; // must be dword aligned
    // Get the palette as RGB565 values (if there is one)
    if (bpp == 4 || bpp == 8)
    {
        uint16_t r, g, b, us;
        int iOff, iColors;
        iColors = pBMP[46]; // colors used BMP field
        if (iColors == 0 || iColors > (1<<bpp))
            iColors = (1 << bpp); // full palette
        iOff = iOffBits - (4 * iColors); // start of color palette
        for (x=0; x<iColors; x++)
        {
            b = pBMP[iOff++];
            g = pBMP[iOff++];
            r = pBMP[iOff++];
            iOff++; // skip extra byte
            r >>= 3;
            us = (r  << 11);
            g >>= 2;
            us |= (g << 5);
            us |= (b >> 3);
            usPalette[x] = (us >> 8) | (us << 8); // swap byte order for writing to the display
        }
    }
    if (ucCompression) // need to do it differently for RLE compressed
    {
    uint16_t *d = (uint16_t *)ucRXBuf;
    int y, iStartY, iEndY, iDeltaY;

       pCompressed = &pBMP[iOffBits]; // start of compressed data
       if (bFlipped)
       {
          iStartY = iDestY + cy - 1;
          iEndY = iDestY - 1;
          iDeltaY = -1;
       }
       else
       {
          iStartY = iDestY;
          iEndY = iDestY + cy;
          iDeltaY = 1;
       }
       DecodeRLE4(NULL, 0,NULL,NULL); // initialize
       DecodeRLE8(NULL, 0,NULL,NULL);
       for (y=iStartY; y!= iEndY; y += iDeltaY)
       {
          spilcdSetPosition(pLCD, iDestX, y, cx, 1, iFlags);
          if (bpp == 4)
             pCompressed = DecodeRLE4(pCompressed, cx, d, usPalette);
          else
             pCompressed = DecodeRLE8(pCompressed, iPitch, d, usPalette);
           myspiWrite(pLCD, (uint8_t *)d, cx*2, MODE_DATA, iFlags);
       }
       return 0;
    } // RLE compressed

    if (bFlipped)
    {
        iOffBits += (cy-1) * iPitch; // start from bottom
        iPitch = -iPitch;
    }

        if (bStretch)
        {
            spilcdSetPosition(pLCD, iDestX, iDestY, cx*2, cy*2, iFlags);
            for (y=0; y<cy; y++)
            {
                pus = (uint16_t *)&pBMP[iOffBits + (y * iPitch)]; // source line
                for (j=0; j<2; j++) // for systems without half-duplex, we need to prepare the data for each write
                {
                    if (iFlags & DRAW_TO_LCD)
                        d = usTemp;
                    else
                        d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (y*pLCD->iScreenPitch)];
                    if (bpp == 16)
                    {
                        if (iTransparent == -1) // no transparency
                        {
                            for (x=0; x<cx; x++)
                            {
                                us = pus[x];
                                d[0] = d[1] = (us >> 8) | (us << 8); // swap byte order
                                d += 2;
                            } // for x
                        }
                        else
                        {
                            for (x=0; x<cx; x++)
                            {
                                us = pus[x];
                                if (us != (uint16_t)iTransparent)
                                    d[0] = d[1] = (us >> 8) | (us << 8); // swap byte order
                                d += 2;
                            } // for x
                        }
                    }
                    else if (bpp == 8)
                    {
                        uint8_t *s = (uint8_t *)pus;
                        if (iTransparent == -1) // no transparency
                        {
                            for (x=0; x<cx; x++)
                            {
                                d[0] = d[1] = usPalette[*s++];
                                d += 2;
                            }
                        }
                        else
                        {
                            for (x=0; x<cx; x++)
                            {
                                uint8_t uc = *s++;
                                if (uc != (uint8_t)iTransparent)
                                    d[0] = d[1] = usPalette[uc];
                                d += 2;
                            }
                        }
                    }
                    else // 4 bpp
                    {
                        uint8_t uc, *s = (uint8_t *)pus;
                        if (iTransparent == -1) // no transparency
                        {
                            for (x=0; x<cx; x+=2)
                            {
                                uc = *s++;
                                d[0] = d[1] = usPalette[uc >> 4];
                                d[2] = d[3] = usPalette[uc & 0xf];
                                d += 4;
                            }
                        }
                        else
                        {
                            for (x=0; x<cx; x+=2)
                            {
                                uc = *s++;
                                if ((uc >> 4) != (uint8_t)iTransparent)
                                    d[0] = d[1] = usPalette[uc >> 4];
                                if ((uc & 0xf) != (uint8_t)iTransparent)
                                    d[2] = d[3] = usPalette[uc & 0xf];
                                d += 4;
                            }
                        }
                    }
                    if (iFlags & DRAW_TO_LCD)
                        spilcdWriteDataBlock(pLCD, (uint8_t *)usTemp, cx*4, iFlags); // write the same line twice
                } // for j
            } // for y
        } // 2:1
        else // 1:1
        {
            spilcdSetPosition(pLCD, iDestX, iDestY, cx, cy, iFlags);
            for (y=0; y<cy; y++)
            {
                pus = (uint16_t *)&pBMP[iOffBits + (y * iPitch)]; // source line
                if (bpp == 16)
                {
                    if (iFlags & DRAW_TO_LCD)
                        d = usTemp;
                    else
                        d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (y * pLCD->iScreenPitch)];
                    if (iTransparent == -1) // no transparency
                    {
                        for (x=0; x<cx; x++)
                        {
                           us = *pus++;
                           *d++ = (us >> 8) | (us << 8); // swap byte order
                        }
                    }
                    else // skip transparent pixels
                    {
                        for (x=0; x<cx; x++)
                        {
                            us = *pus++;
                            if (us != (uint16_t)iTransparent)
                             d[0] = (us >> 8) | (us << 8); // swap byte order
                            d++;
                        }
                    }
                }
                else if (bpp == 8)
                {
                    uint8_t uc, *s = (uint8_t *)pus;
                    if (iFlags & DRAW_TO_LCD)
                        d = usTemp;
                    else
                        d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (y*pLCD->iScreenPitch)];
                    if (iTransparent == -1) // no transparency
                    {
                        for (x=0; x<cx; x++)
                        {
                            *d++ = usPalette[*s++];
                        }
                    }
                    else
                    {
                        for (x=0; x<cx; x++)
                        {
                            uc = *s++;
                            if (uc != iTransparent)
                                d[0] = usPalette[*s++];
                            d++;
                        }
                    }
                }
                else // 4 bpp
                {
                    uint8_t uc, *s = (uint8_t *)pus;
                    if (iFlags & DRAW_TO_LCD)
                        d = usTemp;
                    else // write to the correct spot directly to save time
                        d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (y*pLCD->iScreenPitch)];
                    if (iTransparent == -1) // no transparency
                    {
                        for (x=0; x<cx; x+=2)
                        {
                            uc = *s++;
                            *d++ = usPalette[uc >> 4];
                            *d++ = usPalette[uc & 0xf];
                        }
                    }
                    else // check transparent color
                    {
                        for (x=0; x<cx; x+=2)
                        {
                            uc = *s++;
                            if ((uc >> 4) != iTransparent)
                               d[0] = usPalette[uc >> 4];
                            if ((uc & 0xf) != iTransparent)
                               d[1] = usPalette[uc & 0xf];
                            d += 2;
                        }
                    }
                }
                if (iFlags & DRAW_TO_LCD)
                    spilcdWriteDataBlock(pLCD, (uint8_t *)usTemp, cx*2, iFlags);
            } // for y
        } // 1:1
    return 0;
} /* spilcdDrawBMP() */

//
// Returns the current backbuffer address
//
uint16_t * spilcdGetBuffer(SPILCD *pLCD)
{
    return (uint16_t *)pLCD->pBackBuffer;
}
//
// Set the back buffer
//
void spilcdSetBuffer(SPILCD *pLCD, void *pBuffer)
{
    pLCD->pBackBuffer = (uint8_t *)pBuffer;
    pLCD->iScreenPitch = pLCD->iCurrentWidth * 2;
    pLCD->iOffset = 0;
    pLCD->iMaxOffset = pLCD->iScreenPitch * pLCD->iCurrentHeight; // can't write past this point
    pLCD->iWindowX = pLCD->iWindowY = 0; // current window = whole display
    pLCD->iWindowCX = pLCD->iCurrentWidth;
    pLCD->iWindowCY = pLCD->iCurrentHeight;

} /* spilcdSetBuffer() */


//
// Allocate the back buffer for delayed rendering operations
// returns -1 for failure, 0 for success
//
int spilcdAllocBackbuffer(SPILCD *pLCD)
{
    if (pLCD->pBackBuffer != NULL) // already allocated
        return -1;
    pLCD->iScreenPitch = pLCD->iCurrentWidth * 2;
    pLCD->pBackBuffer = (uint8_t *)malloc(pLCD->iScreenPitch * pLCD->iCurrentHeight);
    if (pLCD->pBackBuffer == NULL) // no memory
        return -1;
    memset(pLCD->pBackBuffer, 0, pLCD->iScreenPitch * pLCD->iCurrentHeight);
    pLCD->iOffset = 0; // starting offset
    pLCD->iMaxOffset = pLCD->iScreenPitch * pLCD->iCurrentHeight; // can't write past this point
    pLCD->iWindowX = pLCD->iWindowY = 0; // current window = whole display
    pLCD->iWindowCX = pLCD->iCurrentWidth;
    pLCD->iWindowCY = pLCD->iCurrentHeight;
    return 0;
}
//
// Free the back buffer
//
void spilcdFreeBackbuffer(SPILCD *pLCD)
{
    if (pLCD->pBackBuffer)
    {
        free(pLCD->pBackBuffer);
        pLCD->pBackBuffer = NULL;
    }
}
void spilcdScroll1Line(SPILCD *pLCD, int iAmount)
{
	int i, iCount;
	int iPitch = pLCD->iCurrentWidth * sizeof(uint16_t);
	uint16_t *d, us;
    if (pLCD == NULL || pLCD->pBackBuffer == NULL)
        return;
    iCount = (pLCD->iCurrentHeight - iAmount) * iPitch;
    memmove(pLCD->pBackBuffer, &pLCD->pBackBuffer[iPitch*iAmount], iCount);
    d = (uint16_t *)&pLCD->pBackBuffer[iCount];
    us = (pLCD->iBG >> 8) | (pLCD->iBG << 8);
    for (i=0; i<iAmount * pLCD->iCurrentWidth; i++) {
        *d++ = us;
    }
} /* obdScroll1Line() */

//
// Full duplex SPI transfer for the touch controller
//
static void rtSPIXfer(SPILCD *pLCD, uint8_t ucCMD, uint8_t *pRXBuf, int iLen)
{

    uint8_t ucTemp[4];

    ucTemp[0] = ucCMD;
    ucTemp[1] = ucTemp[2] = 0;


        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        memcpy(pRXBuf, ucTemp, iLen); // ESP-IDF allows duplex overwrite

        t.length = iLen * 8; // length is in bits
        t.tx_buffer = ucTemp;
        t.rx_buffer = pRXBuf;

        gpio_set_level(pLCD->iRTCS, 0);
        esp_err_t ret = spi_device_transmit(spi, &t);
        assert(ret == ESP_OK);
        gpio_set_level(pLCD->iRTCS, 1);

} /* rtSPIXfer() */


int BB_SPI_LCD::rtInit(gpio_num_t u8CS)
{
    if (u8CS != 0xff) { _lcd.iRTCS = u8CS; }
    gpio_config_t io_conf;
    	io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupts
    	io_conf.mode = GPIO_MODE_OUTPUT;         // Set as input mode
    	io_conf.pin_bit_mask = (1ULL << u8CS);  // Set the bit mask for the pin
    	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
    	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up

    	// Apply the configuration
    	gpio_config(&io_conf);
    return 1;
}

// Return the average of the closest 2 of 3 values
static int rtAVG(int *pI)
{
  int da, db, dc;
  int avg = 0;
  if ( pI[0] > pI[1] ) da = pI[0] - pI[1]; else da = pI[1] - pI[0];
  if ( pI[0] > pI[2] ) db = pI[0] - pI[2]; else db = pI[2] - pI[0];
  if ( pI[2] > pI[1] ) dc = pI[2] - pI[1]; else dc = pI[1] - pI[2];

  if ( da <= db && da <= dc ) avg = (pI[0] + pI[1]) >> 1;
  else if ( db <= da && db <= dc ) avg = (pI[0] + pI[2]) >> 1;
  else avg = (pI[1] + pI[2]) >> 1;

  return avg;
} /* rtAVG() */

//
// returns 1 for a touch position available
// or 0 for no touch or invalid parameter
//
int BB_SPI_LCD::rtReadTouch(TOUCHINFO *ti)
{
// commands and SPI transaction filler to read 3 byte response for x/y
uint8_t ucTemp[4];
	int iOrient, x, y, xa[3], ya[3], x1 = 0, y1 = 0, z, z1, z2;
	const int iOrients[4] = {0,90,180,270};

    iOrient = iOrients[_lcd.iOrientation];
    iOrient += _lcd.iRTOrientation; // touch sensor relative to default angle
    iOrient %= 360;

    if (ti == NULL) {
    	ESP_LOGI("MAIN", "null");
        return 0;
    }
    if (_lcd.iRTCS >= 0 && _lcd.iRTCS < 99)  {
    	gpio_set_level(_lcd.iRTCS, 0);
    }
    // read the "pressure" value to see if there is a touch
    rtSPIXfer(&_lcd, 0xb1, ucTemp, 3);
    rtSPIXfer(&_lcd, 0xc1, ucTemp, 3);
    z1 = (int)((ucTemp[2] + (ucTemp[1]<<8)) >> 3);
    z = z1 + 4095;
    rtSPIXfer(&_lcd, 0x91, ucTemp, 3);
    z2 = (int)((ucTemp[2] + (ucTemp[1]<<8)) >> 3);
    z -= z2;
    if (z > _lcd.iRTThreshold) {
        ti->count = 0;
        if (_lcd.iRTCS >= 0 && _lcd.iRTCS < 99) {
        	gpio_set_level(_lcd.iRTCS, 1); // inactive CS
        }
        ESP_LOGI("MAIN", "threeshold %d", z);
        return 0; // not a valid pressure reading
    } else {
      //Serial.printf("pressure = %d\n", z);
    }
    ti->count = 1; // only 1 touch point possible
    z = (_lcd.iRTThreshold - z)/16;
    ti->pressure[0] = (uint8_t)z;

    // read the X and Y values 3 times because they jitter
    rtSPIXfer(&_lcd, 0x91, ucTemp, 3); // first Y is always noisy
    rtSPIXfer(&_lcd, 0xd1, ucTemp, 3);
    xa[0] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0x91, ucTemp, 3);
    ya[0] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0xd1, ucTemp, 3);
    xa[1] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0x91, ucTemp, 3);
    ya[1] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0xd0, ucTemp, 3); // last X, power down
    xa[2] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0x00, ucTemp, 3); // last Y
    ya[2] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    // take the average of the closest values
    x = rtAVG(xa);
    y = rtAVG(ya);
    // since we know the display size and orientation, scale the coordinates
    // to match. The 0 orientation corresponds to flipped X and correct Y
    switch (iOrient) {
        case 0:
        case 180:
            y1 = (1900 - y)*_lcd.iCurrentHeight;
            y1 /= 1780;
            if (y1 < 0) y1 = 0;
            else if (y1 >= _lcd.iCurrentHeight) y1 = _lcd.iCurrentHeight-1;
            x1 = (1950 - x) * _lcd.iCurrentWidth;
            x1 /= 1750;
            if (x1 < 0) x1 = 0;
            else if (x1 >= _lcd.iCurrentWidth) x1 = _lcd.iCurrentWidth-1;
            break;
        case 90:
        case 270:
            x1 = (1900 - y) * _lcd.iCurrentWidth;
            x1 /= 1780;
            if (x1 < 0) x1 = 0;
            else if (x1 >= _lcd.iCurrentWidth) x1 = _lcd.iCurrentWidth-1;
            y1 = (1950 - x) * _lcd.iCurrentHeight;
            y1 /= 1750;
            if (y1 < 0) y1 = 0;
            else if (y1 >= _lcd.iCurrentHeight) y1 = _lcd.iCurrentHeight-1;
            break;
    } // switch on orientation
    if (iOrient == 0 || iOrient == 90) {
        x1 = _lcd.iCurrentWidth - 1 - x1;
        y1 = _lcd.iCurrentHeight - 1 - y1;
    }
    ti->x[0] = x1;
    ti->y[0] = y1;
    gpio_set_level(_lcd.iRTCS, 1); // inactive CS
    //delay(10); // don't let the user try to read samples too quickly
    return 1;
} /* rtReadTouch() */


//
// Merge 2 class instances with transparence (must be the same size)
//
int BB_SPI_LCD::merge(uint16_t *s, uint16_t usTrans, int bSwap565)
{
    uint16_t *d = (uint16_t *)_lcd.pBackBuffer;
    int i;
    if (bSwap565) {
        for (i = 0; i<_lcd.iCurrentWidth * _lcd.iCurrentHeight; i++) {
            if (d[i] == usTrans) { // replace transparent pixels in current surface
                d[i] = __builtin_bswap16(s[i]);
            }
        }
    } else {
        for (i = 0; i<_lcd.iCurrentWidth * _lcd.iCurrentHeight; i++) {
            if (d[i] == usTrans) {
                d[i] = s[i];
            }
        }
    }
    return 1;
} /* merge() */


//
// Capture pixels being drawn into the current drawing surface
// onto a virtual surface at a specific position
// e.g. to capture the lower right corner of an image
// set dst_x = current_width/2, dst_y = current_height/2
//
int BB_SPI_LCD::captureArea(int dst_x, int dst_y, int src_x, int src_y, int src_w, int src_h, uint16_t *pPixels, int bSwap565)
{
uint16_t *s, *d;
int x, y, sx, sy, dx, dy, cx, cy;

    if (_lcd.pBackBuffer == 0) return 0; // no buffer
    // see if any overlap
    if (dst_x >= (src_x+src_w) || src_x >= (dst_x + _lcd.iCurrentWidth) || dst_y >= (src_y+src_h) || src_y >= (dst_y + _lcd.iCurrentHeight))
        return 0; // no intersection

    s = pPixels; d = (uint16_t *)_lcd.pBackBuffer;
    dx = dy = 0;
    cx = _lcd.iCurrentWidth; cy = _lcd.iCurrentHeight;
    sx = dst_x - src_x; // source starting point
    sy = dst_y - src_y;
    if (sx < 0) {
        dx -= sx;
        cx += sx;
        sx = 0;
    }
    if (sy < 0) {
        dy -= sy;
        cy += sy;
        sy = 0;
    }
    if (cx > src_w) cx = src_w;
    if (cy > src_h) cy = src_h;

    s += sx + (sy * src_w);
    d += dx + (dy * _lcd.iCurrentWidth);
    if (bSwap565) {
        for (y=0; y<cy; y++) {
            for (x=0; x<cx; x++) {
                d[x] = __builtin_bswap16(s[x]);
            } // for x
            s += src_w;
            d += _lcd.iCurrentWidth;
        } // for y
    } else { // no swap
        for (y=0; y<cy; y++) {
            memcpy(d, s, cx*2);
            s += src_w;
            d += _lcd.iCurrentWidth;
        }
    }
    return 1;
} /* captureArea() */

int BB_SPI_LCD::freeVirtual(void)
{
    if (_lcd.pBackBuffer == 0) return 0;
    free(_lcd.pBackBuffer);
    _lcd.pBackBuffer = 0;
    return 1;
} /* freeVirtual() */

uint16_t BB_SPI_LCD::color565(uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t u16;
   u16 = b >> 3;
   u16 |= ((g >> 2) << 5);
   u16 |= ((r >> 3) << 11);
   return u16;
} /* color565() */

int BB_SPI_LCD::begin(int iType, int iFlags, int iFreq, gpio_num_t iCSPin,gpio_num_t iDCPin, gpio_num_t iResetPin,gpio_num_t iMISOPin, gpio_num_t iMOSIPin, gpio_num_t iCLKPin)
{
  return spilcdInit(&_lcd, iType, iFlags, iFreq, iCSPin, iDCPin, iResetPin, iMISOPin, iMOSIPin, iCLKPin,1);
} /* begin() */

void BB_SPI_LCD::freeBuffer(void)
{
    spilcdFreeBackbuffer(&_lcd);
}

void BB_SPI_LCD::waitDMA(void)
{
    spilcdWaitDMA();
} /* waitDMA() */

uint8_t * BB_SPI_LCD::getDMABuffer(void)
{
    return (uint8_t *)ucTXBuf;
}

void * BB_SPI_LCD::getBuffer(void)
{
    return (void *)_lcd.pBackBuffer;
}
SPILCD * BB_SPI_LCD::getLCDStruct(void)
{
    return &_lcd;
}

int BB_SPI_LCD::fontHeight(void)
{
	int h;
    if (_lcd.pFont == NULL) { // built-in fonts
        if (_lcd.iFont == FONT_8x8 || _lcd.iFont == FONT_6x8) {
          h = 8;
        } else if (_lcd.iFont == FONT_12x16 || _lcd.iFont == FONT_16x16) {
          h = 16;
        } else {
          h = 32;
        }
    } else {
        int w, top, bottom;
        const char *string = "H";
        spilcdGetStringBox(_lcd.pFont, (char *)string, &w, &top, &bottom);
        h = bottom - top;
    }
    return h;
} /* fontHeight() */

void BB_SPI_LCD::getTextBounds(const char *string, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w1, uint16_t *h1)
{
    if (_lcd.pFont == NULL) { // use built-in fonts
        int iLen = strlen(string);
        int h, w;
        if (_lcd.iFont == FONT_8x8 || _lcd.iFont == FONT_6x8) {
          h = 8;
          w = (_lcd.iFont == FONT_8x8) ? 8 : 6;
        } else if (_lcd.iFont == FONT_12x16 || _lcd.iFont == FONT_16x16) {
          h = 16;
          w = (_lcd.iFont == FONT_12x16) ? 12 : 16;
        } else { w = 16; h = 32; }
        *x1 = x; *y1 = y; // starts drawing downward
        *w1 = w * iLen;
        *h1 = h;
    } else { // custom fonts
        int w, top, bottom;
        spilcdGetStringBox(_lcd.pFont, (char *)string, &w, &top, &bottom);
        *w1 = w;
        *h1 = bottom - top;
        *y1 = y + top;
        *x1 = x;
    }
} /* getTextBounds() */

bool BB_SPI_LCD::allocBuffer(void)
{
    int rc = spilcdAllocBackbuffer(&_lcd);
    return (rc == 0);
}

void BB_SPI_LCD::setScroll(bool bScroll)
{
    _lcd.bScroll = bScroll;
}


void BB_SPI_LCD::drawPattern(uint8_t *pPattern, int iSrcPitch, int iDestX, int iDestY, int iCX, int iCY, uint16_t usColor, int iTranslucency)
{
  spilcdDrawPattern(&_lcd, pPattern, iSrcPitch, iDestX, iDestY, iCX, iCY, usColor, iTranslucency);
} /* drawPattern() */

void BB_SPI_LCD::pushPixels(uint16_t *pixels, int count, int flags)
{
   spilcdWriteDataBlock(&_lcd, (uint8_t *)pixels, count * 2, flags);
} /* pushPixels() */

void BB_SPI_LCD::setAddrWindow(int x, int y, int w, int h)
{
   spilcdSetPosition(&_lcd, x, y, w, h, DRAW_TO_LCD);
} /* setAddrWindow() */

void BB_SPI_LCD::setRotation(int iAngle)
{
int i;
  switch (iAngle) {
    default: return;
    case 0:
      i = LCD_ORIENTATION_0;
      break;
    case 90:
    case 1: // allow Adafruit way or angle
      i = LCD_ORIENTATION_90;
      break;
    case 180:
    case 2:
      i = LCD_ORIENTATION_180;
      break;
    case 270:
    case 3:
      i = LCD_ORIENTATION_270;
      break;
  }
  spilcdSetOrientation(&_lcd, i);
} /* setRotation() */

void BB_SPI_LCD::fillScreen(int iColor)
{
  spilcdFill(&_lcd, iColor, DRAW_TO_LCD | DRAW_TO_RAM);
} /* fillScreen() */


void BB_SPI_LCD::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t fill)
{
  spilcdRectangle(&_lcd, x, y, w, h, color, color, fill, DRAW_TO_LCD | DRAW_TO_RAM);
} /* drawRect() */

void BB_SPI_LCD::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color, int iFlags)
{
  spilcdRectangle(&_lcd, x+r, y, w-(2*r), r, color, color, 1, iFlags);
  spilcdRectangle(&_lcd, x, y+r, w, h-(2*r), color, color, 1, iFlags);
  spilcdRectangle(&_lcd, x+r, y+h-r, w-(2*r), r, color, color, 1, iFlags);

  // draw four corners
  spilcdEllipse(&_lcd, x+w-r-1, y+r, r, r, 1, color, 1, iFlags);
  spilcdEllipse(&_lcd, x+r, y+r, r, r, 2, color, 1, iFlags);
  spilcdEllipse(&_lcd, x+w-r-1, y+h-r, r, r, 1, color, 1, iFlags);
  spilcdEllipse(&_lcd, x+r, y+h-r, r, r, 2, color, 1, iFlags);

} /* fillRoundRect() */

void BB_SPI_LCD::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                 int16_t r, uint16_t color, int iFlags)
{
    spilcdDrawLine(&_lcd, x+r, y, x+w-r, y, color, iFlags); // top
    spilcdDrawLine(&_lcd, x+r, y+h-1, x+w-r, y+h-1, color, iFlags); // bottom
    spilcdDrawLine(&_lcd, x, y+r, x, y+h-r, color, iFlags); // left
    spilcdDrawLine(&_lcd, x+w-1, y+r, x+w-1, y+h-r, color, iFlags); // right
    // four corners
    spilcdEllipse(&_lcd, x+r, y+r, r, r, 1, color, 0, iFlags);
    spilcdEllipse(&_lcd, x+w-r-1, y+r, r, r, 2, color, 0, iFlags);
    spilcdEllipse(&_lcd, x+w-r-1, y+h-r-1, r, r, 4, color, 0, iFlags);
    spilcdEllipse(&_lcd, x+r, y+h-r-1, r, r, 8, color, 0, iFlags);
} /* drawRoundRect() */


void BB_SPI_LCD::fillRect(int x, int y, int w, int h, int iColor)
{
  spilcdRectangle(&_lcd, x, y, w, h, iColor, iColor, 1, DRAW_TO_LCD | DRAW_TO_RAM);
} /* fillRect() */

void BB_SPI_LCD::setTextColor(int iFG, int iBG)
{
  _lcd.iFG = iFG;
  _lcd.iBG = (iBG == -1) ? iFG : iBG;
} /* setTextColor() */

void BB_SPI_LCD::setCursor(int x, int y)
{
  _lcd.iCursorX = x;
  _lcd.iCursorY = y;
} /* setCursor() */

void BB_SPI_LCD::setFont(int iFont)
{
  _lcd.iFont = iFont;
  _lcd.pFont = NULL;
} /* setFont() */


void BB_SPI_LCD::setFreeFont(const GFXfont *pFont)
{
  _lcd.pFont = (GFXfont *)pFont;
} /* setFreeFont() */

int BB_SPI_LCD::drawBMP(const uint8_t *pBMP, int iDestX, int iDestY, int bStretch, int iTransparent, int iFlags)
{
    return spilcdDrawBMP(&_lcd, (uint8_t *)pBMP, iDestX, iDestY, bStretch, iTransparent, iFlags);
} /* drawBMP() */

void BB_SPI_LCD::drawStringFast(const char *szText, int x, int y, int size)
{
    if (size == -1) size = _lcd.iFont;
    spilcdWriteStringFast(&_lcd, x, y, (char *)szText, _lcd.iFG, _lcd.iBG, size, DRAW_TO_LCD | DRAW_TO_RAM);
} /* drawStringFast() */

void BB_SPI_LCD::drawString(const char *pText, int x, int y, int size)
{
   if (size == 1) setFont(FONT_6x8);
   else if (size == 2) setFont(FONT_12x16);
   setCursor(x,y);
   for (int i=0; i<strlen(pText); i++) {
      write(pText[i]);
   }
} /* drawString() */

void BB_SPI_LCD::drawString(char *pText, int x, int y, int size)
{
   if (size == 1) setFont(FONT_6x8);
   else if (size == 2) setFont(FONT_12x16);
   setCursor(x,y);
   for (int i=0; i<strlen(pText); i++) {
      write(pText[i]);
   }
} /* drawString() */

void BB_SPI_LCD::drawLine(int x1, int y1, int x2, int y2, int iColor)
{
  spilcdDrawLine(&_lcd, x1, y1, x2, y2, iColor, DRAW_TO_LCD | DRAW_TO_RAM);
} /* drawLine() */

inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
  return gfxFont->glyph + c;
}

//
// write (Print friend class)
//
size_t BB_SPI_LCD::write(uint8_t c) {
char szTemp[2]; // used to draw 1 character at a time to the C methods
int w, h;

  szTemp[0] = c; szTemp[1] = 0;
  if (_lcd.pFont == NULL) { // use built-in fonts
      if (_lcd.iFont == FONT_8x8 || _lcd.iFont == FONT_6x8) {
        h = 8;
        w = (_lcd.iFont == FONT_8x8) ? 8 : 6;
      } else if (_lcd.iFont == FONT_12x16 || _lcd.iFont == FONT_16x16) {
        h = 16;
        w = (_lcd.iFont == FONT_12x16) ? 12 : 16;
      } else { w = 16; h = 32; }

    if (c == '\n') {              // Newline?
      _lcd.iCursorX = 0;          // Reset x to zero,
      _lcd.iCursorY += h; // advance y one line
        // should we scroll the screen up 1 line?
        if ((_lcd.iCursorY + (h-1)) >= _lcd.iCurrentHeight && _lcd.pBackBuffer && _lcd.bScroll) {
            spilcdScroll1Line(&_lcd, h);
            spilcdShowBuffer(&_lcd, 0, 0, _lcd.iCurrentWidth, _lcd.iCurrentHeight, DRAW_TO_LCD);
            _lcd.iCursorY -= h;
        }
    } else if (c != '\r') {       // Ignore carriage returns
      if (_lcd.iWrap && ((_lcd.iCursorX + w) > _lcd.iCurrentWidth)) { // Off right?
        _lcd.iCursorX = 0;               // Reset x to zero,
        _lcd.iCursorY += h; // advance y one line
          // should we scroll the screen up 1 line?
          if ((_lcd.iCursorY + (h-1)) >= _lcd.iCurrentHeight && _lcd.pBackBuffer && _lcd.bScroll) {
              spilcdScroll1Line(&_lcd, h);
              spilcdShowBuffer(&_lcd, 0, 0, _lcd.iCurrentWidth, _lcd.iCurrentHeight, DRAW_TO_LCD);
              _lcd.iCursorY -= h;
          }
      }
      spilcdWriteString(&_lcd, -1, -1, szTemp, _lcd.iFG, _lcd.iBG, _lcd.iFont, DRAW_TO_LCD | DRAW_TO_RAM);
    }
  } else { // Custom font
    if (c == '\n') {
      _lcd.iCursorX = 0;
      _lcd.iCursorY += (uint8_t)(_lcd.pFont->yAdvance);
    } else if (c != '\r') {
      uint8_t first = (_lcd.pFont->first);
      if ((c >= first) && (c <= (uint8_t)(_lcd.pFont->last))) {
        GFXglyph *glyph = pgm_read_glyph_ptr(_lcd.pFont, c - first);
        w = (glyph->width);
        h = (glyph->height);
        if ((w > 0) && (h > 0)) { // Is there an associated bitmap?
          int16_t xo = (int8_t)(glyph->xOffset);
          w += xo; // xadvance
          h = (uint8_t)(_lcd.pFont->yAdvance);
          if (_lcd.iAntialias) { w /= 2; h /= 2; }
          if (_lcd.iWrap && ((_lcd.iCursorX + w) > _lcd.iCurrentWidth)) {
            _lcd.iCursorX = 0;
            _lcd.iCursorY += h;
          }
          if (_lcd.iAntialias)
            spilcdWriteStringAntialias(&_lcd, _lcd.pFont, -1, -1, szTemp, _lcd.iFG, 0,  DRAW_TO_LCD | DRAW_TO_RAM);
          else
            spilcdWriteStringCustom(&_lcd, _lcd.pFont, -1, -1, szTemp, _lcd.iFG, _lcd.iBG, 0,  DRAW_TO_LCD | DRAW_TO_RAM);
        }
      }
    }
  }
  return 1;
} /* write() */

void BB_SPI_LCD::display(void)
{
    spilcdShowBuffer(&_lcd, 0, 0, _lcd.iCurrentWidth, _lcd.iCurrentHeight, DRAW_TO_LCD);
}

void BB_SPI_LCD::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  spilcdSetPosition(&_lcd, x, y, 1, 1, DRAW_TO_LCD);
  color = __builtin_bswap16(color);
  spilcdWriteDataBlock(&_lcd, (uint8_t *)&color, 2, DRAW_TO_LCD | DRAW_TO_RAM);
}

void BB_SPI_LCD::setAntialias(bool bAntialias)
{
  _lcd.iAntialias = (int)bAntialias;
}
int16_t BB_SPI_LCD::getCursorX(void)
{
  return _lcd.iCursorX;
}
int16_t BB_SPI_LCD::getCursorY(void)
{
  return _lcd.iCursorY;
}
uint8_t BB_SPI_LCD::getRotation(void)
{
  return _lcd.iOrientation;
}
int16_t BB_SPI_LCD::width(void)
{
   return _lcd.iCurrentWidth;
}
int16_t BB_SPI_LCD::height(void)
{
   return _lcd.iCurrentHeight;
}
void BB_SPI_LCD::drawCircle(int32_t x, int32_t y, int32_t r, uint32_t color)
{
  spilcdEllipse(&_lcd, x, y, r, r, 0xf, (uint16_t)color, 0, DRAW_TO_LCD | DRAW_TO_RAM);
}
void BB_SPI_LCD::fillCircle(int32_t x, int32_t y, int32_t r, uint32_t color)
{
  spilcdEllipse(&_lcd, x, y, r, r, 0xf, (uint16_t)color, 1, DRAW_TO_LCD | DRAW_TO_RAM);
}
void BB_SPI_LCD::drawEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color)
{
  spilcdEllipse(&_lcd, x, y, rx, ry, 0xf, (uint16_t)color, 0, DRAW_TO_LCD | DRAW_TO_RAM);
}
void BB_SPI_LCD::fillEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color)
{
  spilcdEllipse(&_lcd, x, y, rx, ry, 0xf, (uint16_t)color, 1, DRAW_TO_LCD | DRAW_TO_RAM);
}

void BB_SPI_LCD::pushImage(int x, int y, int w, int h, uint16_t *pixels, int iFlags)
{
  spilcdSetPosition(&_lcd, x, y, w, h, DRAW_TO_LCD);
  spilcdWriteDataBlock(&_lcd, (uint8_t *)pixels, w*h*2, iFlags);
}



