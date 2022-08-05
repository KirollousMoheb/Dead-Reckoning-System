 /******************************************************************************
 *
 * Module: NEO-M8P-2 High Precision GNSS Modules
 *
 * File Name: gps.c
 *
 * Description: Source file for NEO-M8P-2 GPS module
 *
 * Author: Kirollous Moheb
 *
 *******************************************************************************/ 
#include "gps.h"
/*******************************************************************************
 *                               Global Variables	                            *
 *******************************************************************************/
static uint8_t GPS_Line[GPSBUFSIZE];
/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
typedef struct
{
	// GGA - Global Positioning System Fixed Data
	double nmea_longitude;
	double nmea_latitude;
	double course_d;
	double utc_time;
	float speed_k;
	int date;
	char state;
	char ns, ew;
} GPS_Data;
GPS_Data GPS;
/*******************************************************************************
 *                      Functions Definitions                                  *
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

void GPS_print(uint8_t *data,uint8_t size)
{
	HAL_UART_Transmit( &DEBUG_UART_HANDLER, data,size ,HAL_MAX_DELAY );
}

 /***************************************************************************************************
 * [Function Name]: GPS_Parse         
 *
 * [Description]: The Function Parses The NMEA Sentences and place the values in GPS struct           
 *
 * [Arguments]:   The GPS Line array          
 *
 * [Returns]:    VOID           
 ***************************************************************************************************/
void GPS_Parse(unsigned char* GPS_Line)
{
	uint8_t GNRMC_matched = ~strncmp( (const char *)GPS_Line , "$GNRMC", 6 );
	uint8_t parsed=sscanf( (const char*)GPS_Line , "$GNRMC,%lf,%c,%lf,%c,%lf,%c,%f,%lf,%d", &GPS.utc_time, &GPS.state,\
			&GPS.nmea_latitude , &GPS.ns, &GPS.nmea_longitude , &GPS.ew , &GPS.speed_k , &GPS.course_d , &GPS.date ) ;

	if(GPS.ns == 'S'){
		GPS.nmea_latitude *= -1;
	}
	if(GPS.ew == 'W'){
		GPS.nmea_longitude *= -1;
	}
	GPS.speed_k /= 1.852; /*Convert the speed from knots to KM/h*/
	
	long_send.send_longitude_data = GPS.nmea_longitude;
	lat_send.send_lattitude_data = GPS.nmea_latitude;
	speed_send.send_speed_data = GPS.speed_k;


#if DEBUG_DATA
	uint8_t long_lat [12];
	uint8_t speed [9];
	uint8_t new_line [2] = "\r\n";		
	uint8_t space [2]= "  ";			
	
	if (GNRMC_matched){
		if(parsed >= 1 && GPS.state == 'A'){			
			GPS_print(new_line,2);	

			sprintf((char *)speed,"%f",GPS.speed_k);	
			GPS_print(speed,5);			
			GPS_print(space,2);								
			
			sprintf((char *)long_lat ,"%0.8lf" ,getInDegree(GPS.nmea_latitude));	
			GPS_print(long_lat,12);				
			GPS_print(space,2);								
			sprintf((char *)long_lat ,"%0.8lf" ,getInDegree(GPS.nmea_longitude));		
			GPS_print( long_lat , 12);				
			GPS_print( new_line , 2 );		

		}
	}
#endif
}
 /***************************************************************************************************
 * [Function Name]: ublox_ReadLength         
 *
 * [Description]:  The Function returns the amount of data present in the data register of the GPS module          
 *
 * [Arguments]:   VOID         
 *
 * [Returns]:    VOID           
 ***************************************************************************************************/
uint16_t ublox_ReadLength(void)
{
	uint8_t data_length [2];
	uint16_t length = 0;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&I2C_HANDLER,I2C_SLAVE_ADRESS, I2C_DATA_LENGTH_REG,1,\
		data_length, sizeof(data_length), HAL_MAX_DELAY);
	
	if(status == HAL_OK){
		length = ((uint16_t)data_length[0] << 8) + (uint16_t)data_length[1]; // Big Endian
	}else{
	/*Send CAN Error Message */
		
	}
	return length;
}
 /***************************************************************************************************
 * [Function Name]:  GPS_I2C_receive        
 *
 * [Description]:  Receives data using I2C           
 *
 * [Arguments]:   VOID         
 *
 * [Returns]:     VOID          
 ***************************************************************************************************/
void GPS_I2C_receive(void)
{
	uint16_t length = ublox_ReadLength();
	uint16_t min_length = MIN(length, GPSBUFSIZE);
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&I2C_HANDLER,I2C_SLAVE_ADRESS, I2C_DATA_REG, 1, GPS_Line, min_length, HAL_MAX_DELAY);
	#if !USING_RTOS
	HAL_Delay(GPS_PERIOD);
	#endif
	if(status == HAL_OK){
		GPS_Parse(GPS_Line);
		#if DEBUG_LINE
		HAL_UART_Transmit(&DEBUG_UART_HANDLER,GPS_Line,GPSBUFSIZE,HAL_MAX_DELAY);
		#endif
		memset(GPS_Line, 0, sizeof(GPS_Line));
	}else{
	/*Send CAN Error Message */
	
	}
}
 /****************************************************************************************************
 * [Function Name]: GPS_receive_Blocking         
 *
 * [Description]: The function reads the GPS Line using Blocking UART           
 *
 * [Arguments]:  VOID          
 *
 * [Returns]:   VOID            
 ***************************************************************************************************/
void GPS_receive_Blocking(void)
{
	int i = 0;
	uint8_t c;
	uint8_t GNRMC_matched;
	HAL_UART_Receive(&GPS_UART_HANDLER,GPS_Line,7,HAL_MAX_DELAY);
	
	GNRMC_matched=~strncmp((const char *)GPS_Line, "$GNRMC,", 7);

	if(GNRMC_matched){	
		i=8;
		HAL_UART_Receive(&GPS_UART_HANDLER,&c,1,HAL_MAX_DELAY);

		while(c!='*'){
				GPS_Line[i]=c;
				HAL_UART_Receive(&GPS_UART_HANDLER,&c,1,HAL_MAX_DELAY);
				i++;
		}
		GPS_Line [i++] = '*';
		HAL_UART_Receive(&GPS_UART_HANDLER,&c,1,HAL_MAX_DELAY);
		GPS_Line [i++] = c;
		HAL_UART_Receive(&GPS_UART_HANDLER,&c,1,HAL_MAX_DELAY);
		GPS_Line [i] = c;
		
		#if DEBUG
		GPS_print(GPS_Line,GPSBUFSIZE);
		#endif
		
	  memset(GPS_Line, 0, sizeof(GPS_Line));

	}
}
 /***************************************************************************************************
 * [Function Name]:   calculateDistance       
 *
 * [Description]:    The function calculates the measured distance between two points        
 *
 * [Arguments]:     The latitude and longitude of two points       
 *
 * [Returns]:      The distance between two point         
 ***************************************************************************************************/
double calculateDistance(double latitude1,double longitude1,double latitude2,double longitude2) 
{
	double R,lat1,long1,lat2,long2,latdiff,longdiff,a,c,distance;
	R = EARTH_RADIUS;       //earth's radius in meters
	lat1 = degToRad(getInDegree(latitude1));
	long1 = degToRad(getInDegree(longitude1));
	lat2 = degToRad(getInDegree(latitude2));
	long2 = degToRad(getInDegree(longitude2));
	latdiff = lat2 - lat1;
	longdiff = long2 - long1;
	a= pow(sin(latdiff/2),2)+cos(lat1)*cos(lat2)*pow(sin(longdiff/2),2); // Haversine formula: a = sin²(?f/2) + cos f1 · cos f2 · sin²(??/2)
	c = 2 * atan2(sqrt(a), sqrt(1 - a));
	distance=R*c;

	return distance;   //in meters
}
 /***************************************************************************************************
 * [Function Name]:  getInDegree        
 *
 * [Description]:   Converts latitude and longitude into degree         
 *
 * [Arguments]:    Latitude or Longitude        
 *
 * [Returns]:   An angle in degree            
 ***************************************************************************************************/
double getInDegree(double value) 
{    
	int degree = (int)value / 100;
	double minutes = value-(double)degree*100;
	return (degree+ (minutes/60));
}
 /***************************************************************************************************
 * [Function Name]: degToRad         
 *
 * [Description]:   Converts latitude and longitude into radian           
 *
 * [Arguments]:    The latitude or langitude in degree          
 *
 * [Returns]:  The latitude or langitude in radian             
 ***************************************************************************************************/
double degToRad(double degree) 
{    
	// function to convert from degree to radian
	double radian = degree * (MATH_PI / 180.00);
	return radian;
}
