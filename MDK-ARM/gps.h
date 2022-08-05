 /******************************************************************************
 *
 * Module: NEO-M8P-2 High Precision GNSS Modules
 *
 * File Name: gps.h
 *
 * Description: Header file for NEO-M8P-2 GPS module
 *
 * Author: Kirollous Moheb
 *
 *******************************************************************************/ 
#ifndef _GPS_H_
#define _GPS_H_
/*******************************************************************************
 *                                Includes		                                  *
 *******************************************************************************/
#include "stdint.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
/*Uart handler to receive the data through*/
#define GPS_UART_HANDLER huart1 
/*Uart handler to print the data on serial*/
#define DEBUG_UART_HANDLER huart2
/*I2C handler to receive the data through*/
#define I2C_HANDLER hi2c1
/*Maximum size of the GPS Line*/
#define GPSBUFSIZE 100 
/*Slave adress of the GPS module 
*	It can be changed while configuring the module through U-Center app
*/
#define I2C_SLAVE_ADRESS (0x42<<1)	
/*Adress of the register containing the available data length*/
#define I2C_DATA_LENGTH_REG (0xFD)
/*Adress of the register containing the available data*/
#define I2C_DATA_REG (0xFF)
/*The rate at which the GPS module reads periodically in ms*/
#define GPS_PERIOD 		100
/*The Earth radius in meters*/
#define EARTH_RADIUS 6371000;
/*Maths PI*/
#define MATH_PI 3.14159265358979323846
/*Set DEBUG to 1 to monitor the data on the serial screen*/
#define DEBUG_DATA      0
#define DEBUG_LINE      0
#define USING_RTOS			1
/*******************************************************************************
 *                      			Unions				                                   *
 *******************************************************************************/
typedef union  {
	double send_longitude_data;
	uint8_t send_longitude_arr[4];
}send_longitude;

typedef union{
	double receive_longitude_data;
	uint8_t receive_longitude_arr[8];
}receive_longitude;

typedef union  {
	double send_lattitude_data;
	uint8_t send_lattitude_arr[8];
}send_lattitude;
typedef union  {
	double receive_lattitude_data;
	uint8_t receive_lattitude_arr[8];
}receive_lattitude;
typedef union  {
	double send_speed_data;
	uint8_t send_speed_arr[4];
}send_speed;
typedef union  {
	double receive_speed_data;
	uint8_t receive_speed_arr[4];
}receive_speed;

/*******************************************************************************
 *                      Preprocessor Macros                                    *
 *******************************************************************************/
#define MIN(x,y) ((x<y)?x:y)
/*******************************************************************************
 *                      			Externs				                                   *
 *******************************************************************************/
extern UART_HandleTypeDef GPS_UART_HANDLER;
extern UART_HandleTypeDef DEBUG_UART_HANDLER;
extern I2C_HandleTypeDef I2C_HANDLER;
extern	send_longitude long_send;
extern	receive_longitude long_receive;
extern	send_lattitude lat_send;
extern	receive_lattitude lat_receive;
extern	send_speed speed_send;
extern	receive_speed speed_receive;
/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/
 /***************************************************************************************************
 * [Function Name]: GPS_print         
 *
 * [Description]: The function prints the GPS read Line into the serial          
 *
 * [Arguments]:  The GPS Line array          
 *
 * [Returns]:    VOID           
 ***************************************************************************************************/
#if DEBUG
void GPS_print(uint8_t *data,uint8_t size);
#endif
 /***************************************************************************************************
 * [Function Name]: GPS_Parse         
 *
 * [Description]: The Function Parses The NMEA Sentences and place the values in GPS struct           
 *
 * [Arguments]:   The GPS Line array          
 *
 * [Returns]:    VOID           
 ***************************************************************************************************/
void GPS_Parse(unsigned char* GPS_Line);
 /***************************************************************************************************
 * [Function Name]: ublox_ReadLength         
 *
 * [Description]:  The Function returns the amount of data present in the data register of the GPS module          
 *
 * [Arguments]:   VOID         
 *
 * [Returns]:    VOID           
 ***************************************************************************************************/
uint16_t ublox_ReadLength(void);
 /***************************************************************************************************
 * [Function Name]:  GPS_I2C_receive        
 *
 * [Description]:  Receives data using I2C           
 *
 * [Arguments]:   VOID         
 *
 * [Returns]:     VOID          
 ***************************************************************************************************/
void GPS_I2C_receive(void);
 /***************************************************************************************************
 * [Function Name]: GPS_receive_Blocking         
 *
 * [Description]: The function reads the GPS Line using Blocking UART           
 *
 * [Arguments]:  VOID          
 *
 * [Returns]:   VOID            
 ***************************************************************************************************/
void GPS_receive_Blocking(void);
 /***************************************************************************************************
 * [Function Name]:   calculateDistance       
 *
 * [Description]:    The function calculates the measured distance between two points        
 *
 * [Arguments]:     The latitude and longitude of two points       
 *
 * [Returns]:      The distance between two point         
 ***************************************************************************************************/
double calculateDistance(double latitude1,double longitude1,double latitude2,double longitude2);
 /***************************************************************************************************
 * [Function Name]:  getInDegree        
 *
 * [Description]:   Converts latitude and longitude into degree         
 *
 * [Arguments]:    Latitude or Longitude        
 *
 * [Returns]:   An angle in degree            
 ***************************************************************************************************/
double getInDegree(double value);
 /***************************************************************************************************
 * [Function Name]: degToRad         
 *
 * [Description]:   Converts latitude and longitude into radian           
 *
 * [Arguments]:    The latitude or langitude in degree          
 *
 * [Returns]:  The latitude or langitude in radian             
 ***************************************************************************************************/
double degToRad(double degree);

#endif

