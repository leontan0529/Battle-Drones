  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  * Student 1: Leon Tan Li Yang 
  * Student 2: Foo Xin Hui, Kelly 
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h" //Accelerometer
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h" //Gyroscope
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h" //Magnetometer
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h" //Temperature
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h" //Pressure (Barometer)
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h" //Humidity
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h" //LED
#include"wifi.h" //WiFi
#include "stdio.h"
#include "math.h" //pow() function to calculate Pressure
#include "string.h"
#include "stdlib.h"


void SystemClock_config(void);
static void UART1_Init(void);
UART_HandleTypeDef huart1;
static void MX_GPIO_Init(void);


/*Static variables required for program*/
//Threshold for telemetry monitoring
#define GYRO_THRES 350.0
#define MAGNETO_THRES -0.17
#define PRESSURE_THRES_HIGH 1500.0
#define PRESSURE_THRES_LOW 500.0
#define SEA_LEVEL_PRESSURE 1013.25
#define HEIGHT_THRES 100.0 //no value, since formula is calculated based on Earth's sea level pressure
#define HUMIDITY_THRES_HIGH 110.0 //no value, HSENSOR always 100% rH in SG's outdoor condition and quite sensitive.
#define HUMIDITY_THRES_LOW 40.0  //Optimally, survival humidity is 40-70% rH
#define TEMP_THRES_HIGH 60.0
#define TEMP_THRES_LOW -15.0
#define ACCEL_THRES 0.0
//Parameters for lasergun firing and charging
#define LASERGUN_CAP 10
#define LASERGUN_CHARGE 3
#define LASERGUN_FIRE 5
//Parameters for last message
#define MAX_MESSAGE_SIZE 256
#define UART_TIMEOUT_MS 10000 //timeout 10 secs for reading
//Parameters for WiFi
#define MAX_LENGTH 400
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000


/*Global variables that will be used for following functions to modify the state of drone*/
//Main signal to run program
volatile int on = 1; //1 (default) to run program, 0 to KILL program
//Track MODE of drone
volatile int mode = 0; //0 (default) for STANDBY_MODE, 1 for BATTLE_MODE
volatile int last = 0; //0 for without The Last of EE2028, 1 for The Last of EE2028
//Manage MODE printer
char mode_message [32]; //To store messages for different MODEs
//Track LED timer
volatile uint32_t led_last_toggle = 0; //to save the last time when LED was toggled
//Manage Telemetry Read & Monitor
char tele_message [200]; //To store required telemetry readings
volatile uint32_t trm_last_sent = 0; //to save the last time when telemetry was sent
volatile float gyro;
volatile float magneto;
volatile float pressure;
volatile float height;
volatile float humidity;
volatile float temp;
volatile float accel;
uint32_t rescue_time = 0; //to record what time drone is in RESCUE state
//Track button interrupts
volatile int pb_count = 0; //0 (RESET), 1 for single press, 2 for double press
uint32_t single_press_timer = 0;
//Manage lasergun
int lasergun_energy = 0; //No energy (default)
volatile int charge = 0; //0 for reset (default), 1 for signal to charge laser gun at appropriate state of drone
//Debugging purpose
char debug [200];
//For WiFi purpose
const char* WiFi_SSID = "2028";               // Replacce mySSID with WiFi SSID for your router / Hotspot
const char* WiFi_password = "chupapi29";   // Replace myPassword with WiFi password for your router / Hotspot
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK; // WiFi security your router / Hotspot. No need to change it unless you use something other than WPA2 PSK
const uint16_t SOURCE_PORT = 28;  // source port, which can be almost any 16 bit number
uint8_t ipaddr[4] = {172, 20, 10, 4};
const uint16_t DEST_PORT = 2028;        // 'server' port number - this is the port Packet Sender listens to (as you set in Packer Sender)
SPI_HandleTypeDef hspi3;
uint8_t req[MAX_LENGTH];  // request packet
uint8_t resp[MAX_LENGTH]; // response packet
uint16_t Datalen;
WIFI_Status_t WiFi_Stat; // WiFi status. Should remain WIFI_STATUS_OK if everything goes well


/*Routines and functions that will be run when drone is up*/
/*Read Gyroscope*/
float read_gyro()
{
	float gyro_data;
	float gyro_raw_data[3];
	BSP_GYRO_GetXYZ(gyro_raw_data);
	gyro_data = sqrt( (float)gyro_raw_data[0]/(1000.0f)*(float)gyro_raw_data[0]/(1000.0f) + (float)gyro_raw_data[1]/(1000.0f)*(float)gyro_raw_data[1]/(1000.0f) + (float)gyro_raw_data[2]/(1000.0f)*(float)gyro_raw_data[2]/(1000.0f) );
	//sprintf(debug, "gyro: %.2f\r\n", gyro_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
	return gyro_data;
}


/*Read Magnetometer*/
float read_magneto()
{
	float magneto_data;
	float magneto_raw_data[3];
	int16_t magneto_data_i16[3] = { 0 };
	BSP_MAGNETO_GetXYZ(magneto_data_i16);
	magneto_raw_data[1] = (float)magneto_data_i16[1]/10000.0f;
	magneto_data = magneto_raw_data[1];
	//sprintf(debug, "magneto: %.2f\r\n", magneto_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
	return magneto_data;
}


/*Read Pressure*/
float read_pressure()
{
	float pressure_data = BSP_PSENSOR_ReadPressure();
	//sprintf(debug, "pressure: %.2f\r\n", pressure_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
	return pressure_data;
}


/*Calculate Height*/
float calc_height(float pa)
{
	float height_data = 44330.0f*(1.0-pow((pa/SEA_LEVEL_PRESSURE),(1.0f/5.255f)));
	//sprintf(debug, "height: %.2f\r\n", height_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
	return height_data;
}


/*Read Humidity*/
float read_humidity()
{
	float humidity_data;
	humidity_data = BSP_HSENSOR_ReadHumidity();
	//sprintf(debug, "humidity: %.2f\r\n", humidity_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
	return humidity_data;
}


/*Read Temperature*/
float read_temp()
{
	float temp_data;
	temp_data = BSP_TSENSOR_ReadTemp();
	//sprintf(tele_message, "temp: %.2f\r\n", temp_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
	return temp_data;
}


/*Read Accelerometer*/
float read_accel()
{
	float accel_data;
	float accel_raw_data[3];
	int16_t accel_data_i16[3] = { 0 };
	BSP_ACCELERO_AccGetXYZ(accel_data_i16);
	accel_raw_data[2] = (float)accel_data_i16[2]*(9.8f/1000.0f);
	accel_data = accel_raw_data[2];
	//sprintf(debug, "accel: %.2f\r\n", accel_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
	return accel_data;
}


/*Telemetry Read and Monitor handler*/
void trm_event()
{
	switch(mode) {
	case 0: //STANDBY_MODE
		gyro = read_gyro();
		magneto = read_magneto();
		pressure = read_pressure();
		height = calc_height(pressure);
		humidity = read_humidity();
		if ((gyro < GYRO_THRES) && (magneto < MAGNETO_THRES) && (pressure > PRESSURE_THRES_LOW && pressure < PRESSURE_THRES_HIGH) && (height < HEIGHT_THRES) && (humidity > HUMIDITY_THRES_LOW && humidity < HUMIDITY_THRES_HIGH)) { //Thresholds not violated
			sprintf(tele_message, "G:%.2f dps, M(Y):%.2f T, P:%.2f hPa, Height:%.2f m, H:%.2f%% rH\r\n", gyro, magneto, pressure, height, humidity);
			HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
			sprintf((char*)req, "G:%.2f dps, M(Y):%.2f T, P:%.2f hPa, Height:%.2f m, H:%.2f%% rH\r", gyro, magneto, pressure, height, humidity);
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
		} else { //Only send violation logs
			if (gyro > GYRO_THRES) {
				sprintf(tele_message, "G:%.2f dps, exceeds threshold of %.2f dps.\r\n", gyro, GYRO_THRES);
				HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
				sprintf((char*)req, "G:%.2f dps, exceeds threshold of %.2f dps.\r", gyro, GYRO_THRES);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			}
			if (magneto > MAGNETO_THRES) {
				sprintf(tele_message, "M(Y):%.2f T, exceeds threshold of %.2f T.\r\n", magneto, MAGNETO_THRES);
				HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
				sprintf((char*)req, "M(Y):%.2f T, exceeds threshold of %.2f T.\r", magneto, MAGNETO_THRES);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			}
			if (pressure > PRESSURE_THRES_HIGH) {
				sprintf(tele_message, "P:%.2f hPa, exceeds threshold of %.2f hPa.\r\n", pressure, PRESSURE_THRES_HIGH);
				HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
				sprintf((char*)req, "P:%.2f hPa, exceeds threshold of %.2f hPa.\r", pressure, PRESSURE_THRES_HIGH);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			} else if (pressure < PRESSURE_THRES_LOW){
				sprintf(tele_message, "P:%.2f hPa, below threshold of %.2f hPa.\r\n", pressure, PRESSURE_THRES_LOW);
				HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
				sprintf((char*)req, "P:%.2f hPa, below threshold of %.2f hPa.\r", pressure, PRESSURE_THRES_LOW);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			}
			if (height > HEIGHT_THRES) {
				sprintf(tele_message, "Height:%.2f m, exceeds threshold of %.2f m.\r\n", height, HEIGHT_THRES);
				HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
				sprintf((char*)req, "Height:%.2f m, exceeds threshold of %.2f m.\r", height, HEIGHT_THRES);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			}
			if (humidity > HUMIDITY_THRES_HIGH) {
				sprintf(tele_message, "H:%.2f%% rH, exceeds threshold of %.2f%% rH.\r\n", humidity, HUMIDITY_THRES_HIGH);
				HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
				sprintf((char*)req, "H:%.2f%% rH, exceeds threshold of %.2f%% rH.\r", humidity, HUMIDITY_THRES_HIGH);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			} else if (humidity < HUMIDITY_THRES_LOW) {
				sprintf(tele_message, "H:%.2f%% rH, below threshold of %.2f%% rH.\r\n", humidity, HUMIDITY_THRES_LOW);
				HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
				sprintf((char*)req, "H:%.2f%% rH, below threshold of %.2f%% rH.\r", humidity, HUMIDITY_THRES_LOW);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			}
		}
		break;
	case 1: //BATTLE_MODE
		accel = read_accel();
		if (last == 0) { //Without the Last_of_EE2028
			temp = read_temp();
			pressure = read_pressure();
			height = calc_height(pressure);
			humidity = read_humidity();
			gyro = read_gyro();
			magneto = read_magneto();
			if ((temp > TEMP_THRES_LOW && temp < TEMP_THRES_HIGH) && (pressure > PRESSURE_THRES_LOW && pressure < PRESSURE_THRES_HIGH) && (height < HEIGHT_THRES) && (humidity > HUMIDITY_THRES_LOW && humidity < HUMIDITY_THRES_HIGH) && (accel > ACCEL_THRES) && (gyro < GYRO_THRES) && (magneto < MAGNETO_THRES)) { //Thresholds not violated
				sprintf(tele_message, "T:%.2f degC, P:%.2f hPa, Height:%.2f m, H:%.2f%% rH, A:%.2f mps^2, G:%.2f dps, M(Y):%.2f T\r\n", temp, pressure, height, humidity, accel, gyro, magneto);
				HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
				sprintf((char*)req, "T:%.2f degC, P:%.2f hPa, Height:%.2f m, H:%.2f%% rH, A:%.2f mps^2, G:%.2f dps, M(Y):%.2f T\r", temp, pressure, height, humidity, accel, gyro, magneto);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			} else {
				if (temp > TEMP_THRES_HIGH) {
					sprintf(tele_message, "T:%.2f degC, exceeds threshold of %.2f degC.\r\n", temp, TEMP_THRES_HIGH);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "T:%.2f degC, exceeds threshold of %.2f degC.\r", temp, TEMP_THRES_HIGH);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				} else if (temp < TEMP_THRES_LOW) {
					sprintf(tele_message, "T:%.2f degC, below threshold of %.2f degC.\r\n", temp, TEMP_THRES_LOW);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "T:%.2f degC, below threshold of %.2f degC.\r", temp, TEMP_THRES_LOW);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				}
				if (pressure > PRESSURE_THRES_HIGH) {
					sprintf(tele_message, "P:%.2f hPa, exceeds threshold of %.2f hPa.\r\n", pressure, PRESSURE_THRES_HIGH);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "P:%.2f hPa, exceeds threshold of %.2f hPa.\r", pressure, PRESSURE_THRES_HIGH);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				} else if (pressure < PRESSURE_THRES_LOW){
					sprintf(tele_message, "P:%.2f hPa, below threshold of %.2f hPa.\r\n", pressure, PRESSURE_THRES_LOW);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "P:%.2f hPa, below threshold of %.2f hPa.\r", pressure, PRESSURE_THRES_LOW);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				}
				if (height > HEIGHT_THRES) {
					sprintf(tele_message, "Height:%.2f m, exceeds threshold of %.2f m.\r\n", height, HEIGHT_THRES);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "Height:%.2f m, exceeds threshold of %.2f m.\r", height, HEIGHT_THRES);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				}
				if (humidity > HUMIDITY_THRES_HIGH) {
					sprintf(tele_message, "H:%.2f%% rH, exceeds threshold of %.2f%% rH.\r\n", humidity, HUMIDITY_THRES_HIGH);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "H:%.2f%% rH, exceeds threshold of %.2f%% rH.\r", humidity, HUMIDITY_THRES_HIGH);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				} else if (humidity < HUMIDITY_THRES_LOW) {
					sprintf(tele_message, "H:%.2f%% rH, below threshold of %.2f%% rH.\r\n", humidity, HUMIDITY_THRES_LOW);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "H:%.2f%% rH, below threshold of %.2f%% rH.\r", humidity, HUMIDITY_THRES_LOW);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				}
				if (gyro > GYRO_THRES) {
					sprintf(tele_message, "G:%.2f dps, exceeds threshold of %.2f dps.\r\n", gyro, GYRO_THRES);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "G:%.2f dps, exceeds threshold of %.2f dps.\r", gyro, GYRO_THRES);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				}
				if (magneto > MAGNETO_THRES) {
					sprintf(tele_message, "M(Y):%.2f T, exceeds threshold of %.2f T.\r\n", magneto, MAGNETO_THRES);
					HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
					sprintf((char*)req, "M(Y):%.2f T, exceeds threshold of %.2f T.\r", magneto, MAGNETO_THRES);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				}
			}
		}
		if (last == 1 && (HAL_GetTick() - rescue_time) < 10000) { //With the Last_of_EE2028 and waiting for rescue in 10 secs
			sprintf(tele_message, "Drone Was Attacked\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)tele_message, strlen(tele_message), 0xFFFF);
			sprintf((char*)req, "Drone Was Attacked\r");
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
		} else if (last == 1 && (HAL_GetTick() - rescue_time) > 10000) { //With the Last_of_EE2028 and rescue not given in 10 secs
			on = 0; //KILL program by exiting while loop
		}
		break;
	default:
		break;
	}
}


/*LED handler*/
void led_event()
{
	switch(mode) {
		case 0: //STANDBY_MODE: LED always on
			BSP_LED_On(LED2);
			break;
		case 1: //BATTLE_MODE
			if (last == 0) { //Without the Last of EE2028: Toggle LED every 1 sec
				if (HAL_GetTick() - led_last_toggle > 500) {
					BSP_LED_Toggle(LED2);
					led_last_toggle = HAL_GetTick();
				}
			} else if (last == 1) { //With the Last of EE2028: Toggle LED every 0.5 sec
				if (HAL_GetTick() - led_last_toggle > 250) {
					BSP_LED_Toggle(LED2);
					led_last_toggle = HAL_GetTick();
				}
			}
			break;
		default:
			break;
	}
}


/*Pushbutton handler*/
void pb_controller()
{
	switch(pb_count) {
		case 1: //Single press
			if ((HAL_GetTick() - single_press_timer) > 500){
				if (mode == 0) { //STANDBY_MODE
					pb_count = 0; //Single press ignored, reset to default
				} else if (mode == 1 && last == 0) { //BATTLE_MODE without the Last_of_EE2028
					charge = 1; //Initiate lasergun charging
					pb_count = 0;
				} else if (mode == 1 && last == 1) { //BATTLE_MODE with the Last_of_EE2028
					pb_count = 0; //Single press ignored, reset to default
				}
			}
			break;
		case 2: //Double press
			if ((HAL_GetTick() - single_press_timer) > 500){
				if (mode == 0) { //STANDBY_MODE
					mode = 1; //Change to BATTLE_MODE
					sprintf(mode_message, "Entering BATTLE MODE.\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)mode_message, strlen(mode_message), 0xFFFF);
					sprintf((char*)req, "Entering BATTLE MODE.\r");
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
					pb_count = 0;
				} else if (mode == 1 && last == 0) { //BATTLE_MODE without the Last_of_EE2028
					mode = 0; //Change to STANDBY_MODE
					sprintf(mode_message, "Entering STANDBY MODE.\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)mode_message, strlen(mode_message), 0xFFFF);
					sprintf((char*)req, "Entering STANDBY MODE.\r");
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
					pb_count = 0;
				} else if (mode == 1 && last == 1) { //BATTLE_MODE with the Last_of_EE2028
					last = 0; //Received rescue
					pb_count = 0;
				}
			}
			break;
		default:
			break;
	}
}


/*Rescue handler for Last of EE2028*/
void last_of_ee2028()
{
	if (mode == 1 && last == 0 && (accel < ACCEL_THRES)) {
		//sprintf(debug, "flipped");
		//HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		last = 1; //Change to with the Last_of_EE2028
		rescue_time = HAL_GetTick(); //Record timestamp of flipped drone
	}
}


/*Lasergun handler*/
/*
void lasergun_event()
{
	if (mode == 1 && last == 0) { //BATTLE_MODE without the Last_of_EE2028
		if (lasergun_energy >= 5) { //Automatically fire when lasergun has more than 5 energy
			lasergun_energy -= LASERGUN_FIRE;
			sprintf(debug, "Firing.\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
			sprintf((char*)req, "Firing.\r");
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
		}
		if (charge == 1) { //Initiated by single press
			sprintf(debug, "Charging...\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
			sprintf((char*)req, "Charging...\r");
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			lasergun_energy += LASERGUN_CHARGE; //Charge lasergun
			if (lasergun_energy > 10) { //Check if lasergun energy is more than capacity after charging
				lasergun_energy = LASERGUN_CAP;
			}
			sprintf(debug, "Charged! %d out of 10.\r\n", lasergun_energy);
			HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
			sprintf((char*)req, "Charged! %d out of 10.\r", lasergun_energy);
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			charge = 0; //Reset charge counter
		}
	}
} */

void lasergun_event()
{
	if (mode == 1 && last == 0) { //BATTLE_MODE without the Last_of_EE2028
		if (charge == 1) { //Initiated by single press
			sprintf(debug, "Charging...\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
			sprintf((char*)req, "Charging...\r");
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			lasergun_energy += LASERGUN_CHARGE; //Charge lasergun
			if (lasergun_energy > 10) { //Check if lasergun energy is more than capacity after charging
				lasergun_energy = LASERGUN_CAP;
			}
			sprintf(debug, "Charged! %d out of 10.\r\n", lasergun_energy);
			HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
			sprintf((char*)req, "Charged! %d out of 10.\r", lasergun_energy);
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			charge = 0; //Reset charge counter
			if (lasergun_energy >= 5) { //Automatically fire when lasergun has more than 5 energy
				lasergun_energy -= LASERGUN_FIRE;
				sprintf(debug, "Firing.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
				sprintf((char*)req, "Firing.\r");
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			}
		}
	}
}


/*EXTI handler*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	switch (GPIO_Pin) {
	case GPIO_PIN_1:
		SPI_WIFI_ISR();
		break;
	case BUTTON_EXTI13_Pin:
		//sprintf(debug, "pressed\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		static uint32_t last_press_time = 0; //Track last PB press time
		static int press_count = 0; //Count number of press in interrupt
		uint32_t current_time = HAL_GetTick(); //Store PB press time
		uint32_t diff_time = current_time - last_press_time; //Calculate difference in time between current and last presses
		if (diff_time > 500){ //Check if difference in time is within 0.5 sec window to proceed with double press
			press_count = 0; //Reset previous press since double press time window has expired
		}
		press_count++; //Increase press count: 1 if single press for more than 0.5 sec, 2 if double press within 0.5 sec
		last_press_time = current_time;
		single_press_timer = last_press_time;
		if (press_count == 1) {
			pb_count = 1; //Log that single press happened, allow pb_controller() to handle
		} else if (press_count == 2) {
			pb_count = 2; //Log that double press happened, allow pb_controller() to handle
		} else if (press_count > 2) {
			pb_count = 0;
		}
		sprintf(debug, "press_count: %d, pb_count: %d\r\n", press_count, pb_count);
		HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		sprintf((char*)req, "press_count: %d, pb_count: %d\r", press_count, pb_count);
		WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
		break;
	} */

	if (GPIO_Pin == BUTTON_EXTI13_Pin){
		//sprintf(debug, "pressed\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		static uint32_t last_press_time = 0; //Track last PB press time
		static int press_count = 0; //Count number of press in interrupt
		uint32_t current_time = HAL_GetTick(); //Store PB press time
		uint32_t diff_time = current_time - last_press_time; //Calculate difference in time between current and last presses
		if (diff_time > 500){ //Check if difference in time is within 0.5 sec window to proceed with double press
			press_count = 0; //Reset previous press since double press time window has expired
		}
		press_count++; //Increase press count: 1 if single press for more than 0.5 sec, 2 if double press within 0.5 sec
		last_press_time = current_time;
		single_press_timer = last_press_time;
		if (press_count == 1) {
			pb_count = 1; //Log that single press happened, allow pb_controller() to handle
		} else if (press_count == 2) {
			pb_count = 2; //Log that double press happened, allow pb_controller() to handle
		} else if (press_count > 2) {
			pb_count = 0; //More than 2 presses, reset pb_count to default
		}
		sprintf(debug, "press_count: %d, pb_count: %d\r\n", press_count, pb_count);
		HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		//sprintf((char*)req, "press_count: %d, pb_count: %d\r", press_count, pb_count);
		//WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
	}

	if (GPIO_Pin == GPIO_PIN_1) {
		SPI_WIFI_ISR();
	}

}


/*Last Message Handler*/
HAL_StatusTypeDef read_from_terminal(char* buffer)
{
    char received_char;
    uint16_t index = 0;
    uint32_t start_time = HAL_GetTick(); //Track the start time for reading

    while (1)
    {
        if (HAL_UART_Receive(&huart1, (uint8_t *)&received_char, 1, UART_TIMEOUT_MS) == HAL_OK)
        {
            if (received_char == '\n' || index >= (MAX_MESSAGE_SIZE - 1)) //Only newline is returned or message more than or equals to maximum size
            {
                buffer[index] = '\0'; // Null-terminate the string
                return HAL_OK; // Successfully received a message
            }
            else
            {
                buffer[index] = received_char; //Store input message
                index++;
            }
        }
        if ((HAL_GetTick() - start_time) >= UART_TIMEOUT_MS) //Check no message received within 10 secs
        {
            buffer[index] = '\0'; // Null-terminate the string
            return HAL_TIMEOUT; // Timeout occurred
        }
    }
}


int main(void)
{
	/*Reset of all peripherals, initializes sensors etc.*/
	HAL_Init();
	/*WiFi*/
	WiFi_Stat = WIFI_Init();
	WiFi_Stat &= WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security);
	if(WiFi_Stat!=WIFI_STATUS_OK) while(1); // halt computations if a WiFi connection could not be established.
	WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT); // Make a TCP connection.
	if(WiFi_Stat!=WIFI_STATUS_OK) while(1); // halt computations if a connection could not be established with the server
	UART1_Init(); //UART Init
	MX_GPIO_Init();
	BSP_LED_Init(LED2); //LED2 Init
	BSP_GYRO_Init(); //Gyroscope Init
	BSP_MAGNETO_Init(); //Magnetometer Init
	BSP_PSENSOR_Init(); //Pressure Init
	BSP_HSENSOR_Init(); //Humidity Init
	BSP_TSENSOR_Init(); //Temperature Init
	BSP_ACCELERO_Init(); //Accelerometer Init
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI); //Pushbutton Init

	/*Print default MODE Message*/
	sprintf(mode_message, "Entering STANDBY MODE.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)mode_message, strlen(mode_message), 0xFFFF);
	sprintf((char*)req, "Entering STANDBY MODE.\r");
	WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

	/*Power up drone*/
	while (on)
	{
		pb_controller();
		//sprintf(debug, "pb_controller\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		last_of_ee2028();
		//sprintf(debug, "last_of_ee2028\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		if ((HAL_GetTick() - trm_last_sent) > 1000) { //Send telemetry reading/violation logs every 1 sec
			trm_event();
			trm_last_sent = HAL_GetTick();
			//sprintf(debug,"trm_event\r\n");
			//HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		}
		led_event();
		//sprintf(debug, "led_event\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
		lasergun_event();
		//sprintf(debug, "lasergun_event\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
	}
	BSP_LED_Off(LED2);
	/*UART to read 1 message from terminal before terminating*/
	sprintf(debug, "Drone not rescued. Any last message for it? (Timeout: 10 seconds)\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
	sprintf((char*)req, "Drone not rescued. Any last message for it? (Timeout: 10 seconds)\r");
	WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
	char received_message[MAX_MESSAGE_SIZE];
	read_from_terminal(received_message);
	sprintf(debug, "Good luck E4-03-07, signing out.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 0xFFFF);
	sprintf((char*)req, "Good luck E4-03-07, signing out.\r");
	WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
	exit(0);
}


static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
	//HAL_NVIC_SetPriority()
}


/*UART configuration*/
static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1; //Configure PB6 and PB7 as Alternate Function 7 (AF7)
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200; //Set baudrate as 115200 bits/s
    huart1.Init.WordLength = UART_WORDLENGTH_8B; //8 bits data
    huart1.Init.StopBits = UART_STOPBITS_1; //1 stop bit
    huart1.Init.Parity = UART_PARITY_NONE; //No parity
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE; //No flow control
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi3);
}
