#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_task_wdt.h"
#include <cstring>
#include <vector>
#include <array>


#include "SDManager/SDManager.h"
#include "bb_spi_lcd/bb_spi_lcd.h"
#include "driver/sdspi_host.h"

#include "esp_rom_sys.h"

#define TFT_CS 5
#define TFT_DC 33
#define TFT_CLK 18
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_RESET 0

#define SD_CS 13

#define STEPPER_X_DIR 25
#define STEPPER_X_STEP 12
#define STEPPER_X_STEPMM 40

#define STEPPER_Y_DIR 27
#define STEPPER_Y_STEP 14
#define STEPPER_Y_STEPMM 400

#define STEPPER_Z_DIR 22
#define STEPPER_Z_STEP 21
#define STEPPER_Z_STEPMM 40

#define STEPPER_ENABLE 32


#define ANALOG_Z ADC1_CHANNEL_7 // 35
#define ANALOG_Y ADC1_CHANNEL_3 // 39
#define ANALOG_X ADC2_CHANNEL_9 //26

#define INT_X 4
#define INT_Z 2
#define INT_SEL1 36
#define INT_SEL2 34

#define SERVO_PIN	            16        // GPIO connects to the PWM signal line

#include "esp_system.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"

#include "driver/ledc.h"

#include "stepper/StepperMotor.h"

#include "driver/mcpwm_prelude.h"

#include "servo/servoControl.h"

#include <driver/ledc.h>
#define THREESHOLD 250

BB_SPI_LCD lcd;

volatile bool homeSet = false;

volatile uint32_t STEPPER_X_POS = 0;
volatile uint32_t STEPPER_Y_POS = 0;
volatile uint32_t STEPPER_Z_POS = 0;

volatile bool STEPPER_X_READY = true;
volatile bool STEPPER_Y_READY = true;
volatile bool STEPPER_Z_READY = true;

volatile uint16_t STEPPER_SPEED = 1000;

static const char *TAG = "MAIN";

StepperMotor STEPPER_X((gpio_num_t)STEPPER_X_STEP, (gpio_num_t)STEPPER_X_DIR, STEPPER_X_STEPMM, 1);
StepperMotor STEPPER_Y((gpio_num_t)STEPPER_Y_STEP, (gpio_num_t)STEPPER_Y_DIR, STEPPER_Y_STEPMM, 2);
StepperMotor STEPPER_Z((gpio_num_t)STEPPER_Z_STEP, (gpio_num_t)STEPPER_Z_DIR, STEPPER_Z_STEPMM, 3);

long INT_SEL1_last = 0;
long INT_SEL1_duration = 0;
bool INT_SEL1_flag = false;

long INT_SEL2_last = 0;
long INT_SEL2_duration = 0;
bool INT_SEL2_flag = false;

volatile uint8_t menuIndex = 0;
volatile uint8_t selectedMenuIndex = 255;
volatile bool menuSelectFlag = false;

const char* menus[] = {"Free move", "Register move", "Home", "toggleMotor", "play CNC"};
volatile uint8_t menuCount = 5;

bool flagMotor = false;

std::vector<std::array<int32_t, 3>> positions_array;

mcpwm_cmpr_handle_t comparator = NULL;

long lastRenderLcd = 0;


servoControl servo;

extern "C" {
	void app_main();

	void enable_stepper();
	void disable_stepper();
	void setMotorReady(int motorIndex);
	bool checkAndMoveMotor(StepperMotor& motor, uint16_t analog, uint8_t motorIndex);
	void write_lcd_stepper_position();
	void write_lcd_menu(void);
	void setupHome();
	void draw_lcd(void* pvParameters);
	char* floatToString(float number);
	char* concatStrings(const char* str1, const char* str2);
	uint16_t map_16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
	uint16_t read_analog_x();
	uint16_t read_analog_y();
	uint16_t read_analog_z();
	void write_lcd_analog(void);

	int8_t* vectorToInt8Array(size_t& outSize);
	void int8ArrayToVector(const int8_t* byteArray, size_t size);

	int map(int x, int in_min, int in_max, int out_min, int out_max);

	void enable_motor(void);
	void disable_motor(void);

	void positionStart(void);
	void positionAdd(int32_t a, int32_t b, int32_t c);
	void positionSave(const char* filename);
	void positionLoad(const char* filename);
	void positionPlay(void);

}

void app_main(void)
{

	ESP_LOGI(TAG, "start");

	// INIT LCD
	lcd.begin(LCD_ILI9341, FLAGS_NONE, 40000000, (gpio_num_t)TFT_CS, (gpio_num_t)TFT_DC, (gpio_num_t)TFT_RESET, (gpio_num_t)TFT_MISO, (gpio_num_t)TFT_MOSI, (gpio_num_t)TFT_CLK);
	lcd.setRotation(270);
	ESP_LOGI(TAG, "init");
	lcd.fillScreen(TFT_BLACK);
	lcd.setTextColor(TFT_WHITE, TFT_BLACK);

	esp_err_t ret = SDManager::initSDCard();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize SD card");

		lcd.drawString("failed to mount SD", 10,50, 2);

		return;
	}


	ESP_LOGI(TAG, "0");
	servo.attach((gpio_num_t)SERVO_PIN);
	servo.write(90);
	ESP_LOGI(TAG, "1");
	vTaskDelay(pdMS_TO_TICKS(1000));
	servo.write(95);
	ESP_LOGI(TAG, "2");
	vTaskDelay(pdMS_TO_TICKS(1000));
	servo.write(90);


	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE; // Disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT;       // Set as output mode
	io_conf.pin_bit_mask = (1ULL << STEPPER_ENABLE); // Bit mask of the pin
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // Disable pull-up mode
	gpio_config(&io_conf);


	io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupt
	io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
	io_conf.pin_bit_mask = (1ULL <<  INT_SEL1);  // Bit mask of the pin
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up mode (if needed)
	gpio_config(&io_conf);

	io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupt
	io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
	io_conf.pin_bit_mask = (1ULL <<  INT_SEL2);  // Bit mask of the pin
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up mode (if needed)
	gpio_config(&io_conf);


	io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupt
	io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
	io_conf.pin_bit_mask = (1ULL <<  INT_X);  // Bit mask of the pin
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up mode (if needed)
	gpio_config(&io_conf);

	io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupt
	io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
	io_conf.pin_bit_mask = (1ULL <<  INT_Z);  // Bit mask of the pin
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // Enable pull-up mode (if needed)
	gpio_config(&io_conf);


	//INIT ANALOG
	adc1_config_width(ADC_WIDTH_BIT_12);  // 12-bit resolution
	//adc2_config_channel_atten(ANALOG_X, ADC_ATTEN_DB_11);  // 0-3.3V range for GPIO36
	//adc1_config_channel_atten(ANALOG_Y, ADC_ATTEN_DB_11);  // 0-3.3V range for GPIO39 (formerly 35)
	//adc1_config_channel_atten(ANALOG_Z, ADC_ATTEN_DB_11);  // 0-3.3V range for GPIO26

	//write_lcd_stepper_position();
	//write_lcd_analog();
	// Initialize the task watchdog

	//write_lcd_menu();
	while (1) {
		if(selectedMenuIndex == 1) {
			if(gpio_get_level((gpio_num_t)INT_SEL1) == 1) {
				INT_SEL1_flag = true;
				selectedMenuIndex = 255;
				disable_stepper();
				positionSave("last");

			}
			if(gpio_get_level((gpio_num_t)INT_SEL2) == 1) {

				while(gpio_get_level((gpio_num_t)INT_SEL2) == 1) {
					vTaskDelay(pdMS_TO_TICKS(10));
				}

				positionAdd(STEPPER_X_POS, STEPPER_Y_POS, STEPPER_Z_POS);
				ESP_LOGI(TAG, "POS ADD %d %d %d", (int)STEPPER_X_POS, (int)STEPPER_Y_POS, (int)STEPPER_Z_POS);

			}
			uint16_t y = read_analog_y();
			uint16_t x = read_analog_x();
			uint16_t z = read_analog_z();

			if((y < 2048 - THREESHOLD || y > 2048 + THREESHOLD) ) {
				if(STEPPER_Y_READY) {
					if(checkAndMoveMotor(STEPPER_Y, y, 2)) {
						STEPPER_Y_READY = false;
					}
				}
			}
			else if(((z < 2048 - THREESHOLD) || (z > 2048 + THREESHOLD))) {
				if(STEPPER_Z_READY)  {
					if(checkAndMoveMotor(STEPPER_Z, z, 3)) {
						STEPPER_Z_READY = false;
					}
				}
			}
			else if((x < 2048 - THREESHOLD || x > 2048 + THREESHOLD)  && x < 4098) {
				if(STEPPER_X_READY) {
					if(checkAndMoveMotor(STEPPER_X, x, 1)) {
						STEPPER_X_READY = false;
					}
				}
			}
		}
		else if(selectedMenuIndex == 0) { // freemove
			//ESP_LOGI(TAG, " %d %d ", gpio_get_level((gpio_num_t)INT_X), gpio_get_level((gpio_num_t)INT_Z));
			if(!INT_SEL1_flag && gpio_get_level((gpio_num_t)INT_SEL1) == 1) {
				INT_SEL1_flag = true;
				INT_SEL1_last = esp_timer_get_time() / 1000;
			} else if(INT_SEL1_flag && gpio_get_level((gpio_num_t)INT_SEL1) == 0) {
				INT_SEL1_flag = false;
				INT_SEL1_duration = (esp_timer_get_time() / 1000) - INT_SEL1_last;

				disable_stepper();
				selectedMenuIndex = 255;
				//write_lcd_menu();
			}


			if(!INT_SEL2_flag && gpio_get_level((gpio_num_t)INT_SEL2) == 1) {
				INT_SEL2_flag = true;
				INT_SEL2_last = esp_timer_get_time() / 1000;
			} else if(INT_SEL2_flag && gpio_get_level((gpio_num_t)INT_SEL2) == 0) {
				INT_SEL2_flag = false;
				INT_SEL2_duration = (esp_timer_get_time() / 1000) - INT_SEL2_last;
				disable_stepper();
				selectedMenuIndex = 255;
				//write_lcd_menu();
			}

			uint16_t y = read_analog_y();
			uint16_t x = read_analog_x();
			uint16_t z = read_analog_z();

			if((y < 2048 - THREESHOLD || y > 2048 + THREESHOLD) ) {
				if(STEPPER_Y_READY) {
					if(checkAndMoveMotor(STEPPER_Y, y, 2)) {
						STEPPER_Y_READY = false;
					}
				}
			}
			else if(((z < 2048 - THREESHOLD) || (z > 2048 + THREESHOLD))) {
				if(STEPPER_Z_READY)  {
					if(checkAndMoveMotor(STEPPER_Z, z, 3)) {
						STEPPER_Z_READY = false;
					}
				}
			}
			else if((x < 2048 - THREESHOLD || x > 2048 + THREESHOLD) ) {
				if(STEPPER_X_READY) {
					if(checkAndMoveMotor(STEPPER_X, x, 1)) {
						STEPPER_X_READY = false;
					}
				}
			}


		} else {
			uint16_t y = read_analog_y();

			if(!INT_SEL2_flag && gpio_get_level((gpio_num_t)INT_SEL2) == 1) {
				INT_SEL2_flag = true;
				INT_SEL2_last = esp_timer_get_time() / 1000;
			} else if(INT_SEL2_flag && gpio_get_level((gpio_num_t)INT_SEL2) == 0) {
				INT_SEL2_flag = false;
				INT_SEL2_duration = (esp_timer_get_time() / 1000) - INT_SEL2_last;
				selectedMenuIndex = menuIndex;
				ESP_LOGI(TAG, "SELECT MENU %d", selectedMenuIndex);
				while( gpio_get_level((gpio_num_t)INT_SEL2) == 1) {
					vTaskDelay(pdMS_TO_TICKS(50));
				}
				if(selectedMenuIndex == 0) {
					enable_stepper();
				} else if(selectedMenuIndex == 1) {
					if(homeSet) {
						enable_stepper();
					} else {
						selectedMenuIndex = 255;
					}
				} else if(selectedMenuIndex == 2) {
					setupHome();
				} else if(selectedMenuIndex == 3) {
					selectedMenuIndex = 255;
					if(flagMotor) {
						disable_motor();
					} else {
						enable_motor();
					}
				} else if(selectedMenuIndex == 4) {
					if(homeSet) {
						enable_stepper();
						positionLoad("last");
						positionPlay();
					}
					selectedMenuIndex = 255;
				}
			} else if(!menuSelectFlag && y < 2048 - 1024) {
				menuSelectFlag = true;
				if(menuIndex == 0 ) {
					menuIndex = menuCount;
				} else {
					menuIndex--;
				}
				//write_lcd_menu();
			} else if(!menuSelectFlag && y > 2048 + 1024) {
				menuSelectFlag = true;
				if(menuIndex == menuCount - 1) {
					menuIndex = 0;
				} else {
					menuIndex++;
				}
				//write_lcd_menu();
			} else if(menuSelectFlag  && y < 2048 + 1024  && y > 2048 - 1024){
				menuSelectFlag = false;
			}
		}
		//vTaskDelay(pdMS_TO_TICKS(50));
		if(lastRenderLcd + 250 < esp_timer_get_time() / 1000) {
			lastRenderLcd = esp_timer_get_time() / 1000;
			xTaskCreate(draw_lcd, "draw_lcd", 2048, NULL, 5, NULL);
		}
	   }
}

void setupHome() {
	enable_stepper();
	STEPPER_Y.move(STEPPER_Y_STEPMM * 5, STEPPER_SPEED, nullptr);
	vTaskDelay(pdMS_TO_TICKS(2000));
	bool flagX = false;
	bool flagZ = false;
	while(!flagX || !flagZ) {
		if(!flagX && STEPPER_X_READY) {
			if(gpio_get_level((gpio_num_t)INT_X) == 1) {
				flagX = true;
			} else {
				STEPPER_X_READY = false;
				STEPPER_X.move(STEPPER_X_STEPMM * -2, STEPPER_SPEED, setMotorReady);
			}
		}
		if(!flagZ && STEPPER_Z_READY) {
			if(gpio_get_level((gpio_num_t)INT_Z) == 1) {
				flagZ = true;
			} else {
				STEPPER_Z_READY = false;
				STEPPER_Z.move(STEPPER_Z_STEPMM * -2, STEPPER_SPEED, setMotorReady);
			}
		}
	}

	STEPPER_X.move(STEPPER_X_STEPMM * 3, STEPPER_SPEED, nullptr);
	STEPPER_Z.move(STEPPER_Z_STEPMM * 3, STEPPER_SPEED, nullptr);

	STEPPER_X_POS = STEPPER_X_STEPMM * 125;
	STEPPER_Z_POS = STEPPER_Z_STEPMM * 125;
	STEPPER_X.move(STEPPER_X_STEPMM * 125, STEPPER_SPEED, nullptr);
	STEPPER_Z.move(STEPPER_Z_STEPMM * 125, STEPPER_SPEED, nullptr);

	bool flagY = false;
	while(!flagY) {
		if(gpio_get_level((gpio_num_t)INT_SEL1) == 1) {
			flagY = true;
		} else if(STEPPER_Y_READY) {
			uint16_t y = read_analog_y();
			if(checkAndMoveMotor(STEPPER_Y, y, 2)) {
				STEPPER_Y_READY = false;
			}
		}
	}
	STEPPER_Y_POS = STEPPER_Y_STEPMM * 5;
	STEPPER_Y.move(STEPPER_Y_STEPMM * 5, STEPPER_SPEED, nullptr);

	homeSet = true;
	selectedMenuIndex = 255;


}

bool checkAndMoveMotor(StepperMotor& motor, uint16_t analog, uint8_t motorIndex)  {


	if(analog < 2048 - THREESHOLD) {
		uint16_t vx = map(analog - THREESHOLD, 0, 2048 - THREESHOLD, STEPPER_SPEED, 100);
		int vd = map(analog - THREESHOLD, 0, 2048 - THREESHOLD, -40, -4);
		ESP_LOGI(TAG, "motor index %d, positive %d speed %d dist %d", motorIndex, analog, vx, vd);
		motor.move(vd, vx, setMotorReady);
		if(homeSet) {
			if(motorIndex == 1) {
				STEPPER_X_POS += vd;
			} else if(motorIndex == 2) {
				STEPPER_Y_POS += vd;
			} else if(motorIndex == 3) {
				STEPPER_Z_POS += vd;
			}
		}
		return true;
	} else if (analog > 2048 + THREESHOLD) {


		uint16_t vx = map((analog - 2048) - THREESHOLD, 0, 2048 - THREESHOLD, 100, STEPPER_SPEED);
		int vd = map((analog - 2048) - THREESHOLD, 0, 2048 - THREESHOLD, 4, 40);
		ESP_LOGI(TAG, "motor index %d, positive %d speed %d dist %d", motorIndex, analog, vx, vd);
		motor.move(vd, vx, setMotorReady);
		if(homeSet) {
			if(motorIndex == 1) {
				STEPPER_X_POS += vd;
			} else if(motorIndex == 2) {
				STEPPER_Y_POS += vd;
			} else if(motorIndex == 3) {
				STEPPER_Z_POS += vd;
			}
		}
		return true;
	}
	return false;
}

void setMotorReady(int motorIndex) {
	ESP_LOGI(TAG, "SET MOTOR READY %d", motorIndex);
	if(motorIndex == 1) {
		STEPPER_X_READY = true;
	} else if(motorIndex == 2) {
		STEPPER_Y_READY = true;
	}else if(motorIndex == 3) {
		STEPPER_Z_READY = true;
	}
}

void draw_lcd(void* pvParameters) {
    //while (true) {
    	write_lcd_analog();
    	write_lcd_stepper_position();
    	write_lcd_menu();
    	vTaskDelete(NULL);
      //  vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 250 ms
    //}
}

void enable_stepper() {
	ESP_LOGI(TAG, "ENABLE STEPPER");
	gpio_set_level((gpio_num_t)STEPPER_ENABLE, 0);
}
void disable_stepper() {
	ESP_LOGI(TAG, "DISABLE STEPPER");
	gpio_set_level((gpio_num_t)STEPPER_ENABLE, 1);
}

void enable_motor(void) {
	ESP_LOGI(TAG, "ENABLE ESC");
	servo.write(135);
	flagMotor = true;
}

void disable_motor(void) {
	ESP_LOGI(TAG, "DISABLE ESC");
	servo.write(90);
	flagMotor = false;
}

uint16_t read_analog_z() {
	return adc1_get_raw(ANALOG_Z);

}

uint16_t read_analog_y() {
	return adc1_get_raw(ANALOG_Y);
}

uint16_t read_analog_x() {
	int adc_reading;
	adc2_get_raw(ANALOG_X, ADC_WIDTH_BIT_12, &adc_reading);
	return adc_reading;
}

void write_lcd_menu(void) {
	lcd.drawRect(0, 35, 320, 140, TFT_BLACK,1);
	for(uint8_t i = 0; i < menuCount; i++) {
		if(selectedMenuIndex == i) {
			lcd.drawRect(5, 38 + (i * 20), 15, 15, TFT_WHITE, 1);
		}
		lcd.drawString(menus[i], 10 + (menuIndex == i ? 10 : 0), 40 + (i * 20) , 1);
	}
	if(selectedMenuIndex == 3) {
		char* temp4 = concatStrings("A: ",std::to_string(positions_array.size()).c_str() );
		lcd.drawString(temp4, 50, 280, 2);
		free(temp4);
	}

}

void write_lcd_analog(void) {
	uint16_t a_y = read_analog_y();
	//ESP_LOGI(TAG, "analog x: %d", a_y);
	uint16_t y = map(a_y, 0, 4095, 0, 30);
	lcd.drawRect(300, 190, 20, 50, TFT_BLACK, 1);
	lcd.drawRect(300, 190, 20, 50, TFT_WHITE, 0);
	lcd.drawRect(304, 198 + y -6, 12, 12, TFT_WHITE, 0);



	lcd.drawRect(246,196, 42, 42, TFT_BLACK, 1);
	lcd.drawRect(246,196, 42, 42, TFT_WHITE, 0);

	int a_x = read_analog_x();
	//ESP_LOGI(TAG, "analog x: %d", a_x);
	int x = map(a_x, 0, 4095, -18, 18);
	int a_z = read_analog_z();
	//ESP_LOGI(TAG, "analog z: %d", a_z);
	int z = map(a_z, 0, 4095, -18, 18);
	lcd.drawRect(266 + x,216 + z, 2,2, TFT_WHITE, 0);
}

void write_lcd_stepper_position(void) {
	lcd.drawRect(0, 0, 320, 30, TFT_BLACK,  1);
	char* xPos;
	char* yPos;
	char* zPos;

	if (homeSet) {
		xPos = floatToString(((float)STEPPER_X_POS) / STEPPER_X_STEPMM);
		yPos = floatToString(((float)STEPPER_Y_POS) / STEPPER_Y_STEPMM);
		zPos = floatToString(((float)STEPPER_Z_POS) / STEPPER_Z_STEPMM);
	} else {
		xPos = (char*)"?";
		yPos = (char*)"?";
		zPos = (char*)"?";
	}

	char* temp1 = concatStrings("X: ", xPos);
	char* temp2 = concatStrings(" Y: ", yPos);
	char* temp3 = concatStrings(" Z: ", zPos);

	char* result = concatStrings(temp1, concatStrings(temp2, temp3));

	lcd.drawString(result, 5, 5, 2);

	if (homeSet) {
		free(xPos);
		free(yPos);
		free(zPos);
	}
	free(temp1);
	free(temp2);
	free(temp3);
	free(result);
}




char* floatToString(float number) {
    // Calculate the required buffer size including the null terminator
    int bufferSize = snprintf(NULL, 0, "%.2f", number) + 1;
    char* buffer = (char*)malloc(bufferSize);

    if (buffer) {
        snprintf(buffer, bufferSize, "%.2f", number);
    }

    return buffer;
}

char* concatStrings(const char* str1, const char* str2) {
    // Calculate the total length for the concatenated string
    int totalLength = strlen(str1) + strlen(str2) + 1;
    char* result = (char*)malloc(totalLength);

    if (result) {
        strcpy(result, str1);
        strcat(result, str2);
    }

    return result;
}

uint16_t map_16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void positionStart(void) {
	positions_array.clear();
}
void positionAdd(int32_t a, int32_t b, int32_t c) {
	positions_array.emplace_back(std::array<int32_t, 3>{a, b, c});
}

void positionSave(const char* filename) {
	size_t byteArraySize;
	int8_t* byteArray = vectorToInt8Array(byteArraySize);
	esp_err_t ret = SDManager::writeToFile(filename, byteArray, byteArraySize);
	if(ret != ESP_OK ) {
		ESP_LOGI(TAG, "unable to save file");
		return;
	}


}

void positionLoad(const char* filename) {
	size_t size;
	esp_err_t ret = SDManager::readFromFile(filename, nullptr, size);

	int8_t* buffer = new int8_t[size];
	ret = SDManager::readFromFile(filename,buffer, size);
	if (ret != ESP_OK) {
		ESP_LOGI(TAG, "unable to load file");
		return;
	}

	int8ArrayToVector(buffer, size);
}

void positionPlay(void) {
	for (const auto& pos : positions_array) {
		int diff_x = pos[0] - STEPPER_X_POS;
		int diff_y = pos[1] - STEPPER_Y_POS;
		int diff_z = pos[2] - STEPPER_Z_POS;
		if(diff_x != 0) {
			STEPPER_X.move(diff_x, STEPPER_SPEED / 4, setMotorReady);
			STEPPER_X_POS += diff_x;
			STEPPER_X_READY = false;
		}
		if(diff_y != 0) {
			STEPPER_Y.move(diff_y, STEPPER_SPEED  , setMotorReady);
			STEPPER_Y_POS += diff_y;
			STEPPER_Y_READY = false;
		}
		if(diff_z != 0) {
			STEPPER_Z.move(diff_z, STEPPER_SPEED / 4 , setMotorReady);
			STEPPER_Z_POS += diff_z;
			STEPPER_Z_READY = false;
		}
		while(!STEPPER_X_READY || !STEPPER_Y_READY || !STEPPER_Z_READY) {
			vTaskDelay(pdMS_TO_TICKS(50));
		}
	}
	STEPPER_Y.move(5 * STEPPER_Y_STEPMM, STEPPER_SPEED, setMotorReady);
	STEPPER_Y_POS += 5 * STEPPER_Y_STEPMM;

	disable_stepper();
}



int8_t* vectorToInt8Array( size_t& outSize) {
    // Calculate the total size in bytes
    outSize = positions_array.size() * 3 * sizeof(int32_t);
    int8_t* byteArray = new int8_t[outSize];

    // Copy the data into the byte array
    size_t offset = 0;
    for (const auto& arr : positions_array) {
        std::memcpy(byteArray + offset, arr.data(), 3 * sizeof(int32_t));
        offset += 3 * sizeof(int32_t);
    }

    return byteArray;
}

// Function to convert int8_t* back to std::vector<std::array<int32_t, 3>>
void int8ArrayToVector(const int8_t* byteArray, size_t size) {
	positions_array.clear();
    size_t numElements = size / (3 * sizeof(int32_t));

    for (size_t i = 0; i < numElements; ++i) {
        std::array<int32_t, 3> arr;
        std::memcpy(arr.data(), byteArray + i * 3 * sizeof(int32_t), 3 * sizeof(int32_t));
        positions_array.push_back(arr);
    }

}
