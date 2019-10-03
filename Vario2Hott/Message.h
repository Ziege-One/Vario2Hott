/*
   Vario2HoTT
   Ziege-One
   v1.0
 
 Arduino pro Mini 5v/16mHz w/ Atmega 328
 
 */


#include "Arduino.h"

// Sensoren ein aus 
#define GAM                             // Ein/Aus GAM
#define EAM                             // Ein/Aus ESC
#define GPS                             // Ein/Aus ESC
#define VARIO                           // Ein/Aus ESC
#define ESC                             // Ein/Aus ESC

// Baudrate UART
#define SERIAL_COM_SPEED    19200

// Radio keys
#define HOTT_KEY_RIGHT  14
#define HOTT_KEY_DOWN   11
#define HOTT_KEY_UP     13
#define HOTT_KEY_SET    9
#define HOTT_KEY_LEFT   7

// Hott protocol v4 delay
#define HOTTV4_TX_DELAY 650

#define HOTT_BINARY_MODE_REQUEST_ID     0x80
#define HOTT_TEXT_MODE_REQUEST_ID       0x7f

//Graupner #General Air Module (GAM) 
#define HOTT_TELEMETRY_GAM_SENSOR_ID    0x8d
#define HOTT_TELEMETRY_GAM_SENSOR_TEXT  0xd0

//Graupner # Electric Air Module module (EAM)
#define HOTT_TELEMETRY_EAM_SENSOR_ID    0x8e
#define HOTT_TELEMETRY_EAM_SENSOR_TEXT  0xe0
   
//Graupner # GPS Module (GPS)
#define HOTT_TELEMETRY_GPS_SENSOR_ID    0x8a
#define HOTT_TELEMETRY_GPS_SENSOR_TEXT  0xa0
          
//Graupner # Vario Sensor Module (VARIO)
#define HOTT_TELEMETRY_VARIO_SENSOR_ID  0x89
#define HOTT_TELEMETRY_VARIO_SENSOR_TEXT 0x90

//Graupner #  Telemetry Speed Controller 70A ESC (ESC)
#define HOTT_TELEMETRY_ESC_SENSOR_ID    0x8C 
#define HOTT_TELEMETRY_ESC_SENSOR_TEXT  0xC0

// String for text mode
#define HOTT_TEXTMODE_MSG_TEXT_LEN 168


// TEXT MODE STRUCTURE
struct HOTT_TEXTMODE_MSG {
  byte start_byte;			    //#01 Starting constant value == 0x7b
  byte esc;				          //#02 Escape (higher-ranking menu in text mode or Text mode leave)
						                //0x00 to stay normal
						                //0x01 to exit
						                //I will send 2 times, so the ESCAPE works really well, so two data frames with 0x01 in byte 2
  byte warning_beeps;			  //#03 1=A 2=B ...
  byte text[8][21];			    //#04...#171 168 ASCII text to display to
						                // Bit 7 = 1 -> Inverse character display
						                // Display 21x8
  byte stop_byte;		        //#172 constant value 0x7d
  byte parity;				      //#173 Checksum / parity
};

//Graupner #General Air Module (GAM)
struct HOTT_GAM_MSG {
  byte start_byte;          //#01 start byte constant value 0x7c
  byte gam_sensor_id;       //#02 EAM sensort id. constat value 0x8d=GENRAL AIR MODULE
  byte warning_beeps;       //#03 1=A 2=B ... 0x1a=Z  0 = no alarm
					                  /* VOICE OR BIP WARNINGS
  					                Alarme sonore A.. Z, octet correspondant 1 à 26
          					        0x00  00  0  No alarm
          					        0x01  01  A  
          					        0x02  02  B  Negative Difference 2 B
          				          0x03  03  C  Negative Difference 1 C
          					        0x04  04  D  
          					        0x05  05  E  
          					        0x06  06  F  Min. Sensor 1 temp. F
          					        0x07  07  G  Min. Sensor 2 temp. G
          					        0x08  08  H  Max. Sensor 1 temp. H
              				      0x09  09  I  Max. Sensor 2 temp. I
              					    0xA   10  J  Max. Sens. 1 voltage J
              					    0xB   11  K  Max. Sens. 2 voltage K
        		                0xC   12  L  
        					          0xD   13  M  Positive Difference 2 M
        					          0xE   14  N  Positive Difference 1 N
        					          0xF   15  O  Min. Altitude O
        					          0x10  16  P  Min. Power Voltage P    // We use this one for Battery Warning
        					          0x11  17  Q  Min. Cell voltage Q
        					          0x12  18  R  Min. Sens. 1 voltage R
        					          0x13  19  S  Min. Sens. 2 voltage S
        					          0x14  20  T  Minimum RPM T
        					          0x15  21  U  
        					          0x16  22  V  Max. used capacity V
        					          0x17  23  W  Max. Current W
        					          0x18  24  X  Max. Power Voltage X
        					          0x19  25  Y  Maximum RPM Y
        					          0x1A  26  Z  Max. Altitude Z
        				            */
  byte sensor_id;           //#04 constant value 0xd0
  byte alarm_invers1;       //#05 alarm bitmask. Value is displayed inverted
				                    //Bit#  Alarm field
					                  // 0    all cell voltage
					                  // 1    Battery 1
					                  // 2    Battery 2
					                  // 3    Temperature 1
					                  // 4    Temperature 2
					                  // 5    Fuel
					                  // 6    mAh
					                  // 7    Altitude
  byte alarm_invers2;       //#06 alarm bitmask. Value is displayed inverted
    					              //Bit#  Alarm Field
    					              // 0    main power current
    					              // 1    main power voltage
    					              // 2    Altitude
    					              // 3    m/s                            
      					            // 4    m/3s
      					            // 5    unknown
    					              // 6    unknown
    					              // 7    "ON" sign/text msg active
  byte cell[6];				      //#7 Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
				                    //#8 Volt Cell 2 (in 2 mV increments, 210 == 4.20 V)
				                    //#9 Volt Cell 3 (in 2 mV increments, 210 == 4.20 V)
				                    //#10 Volt Cell 4 (in 2 mV increments, 210 == 4.20 V)
					                  //#11 Volt Cell 5 (in 2 mV increments, 210 == 4.20 V)
					                  //#12 Volt Cell 6 (in 2 mV increments, 210 == 4.20 V)
  uint16_t  Battery1;       //#13 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
				                    //#14 MSB 
  uint16_t  Battery2;       //#15 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
					                  //#16 MSB
  byte temperature1;        //#17 Temperature 1. Offset of 20. a value of 20 = 0°C
  byte temperature2;        //#18 Temperature 2. Offset of 20. a value of 20 = 0°C
  byte fuel_procent;        //#19 Fuel capacity in %. Values 0--100
					                  //graphical display ranges: 0-100% with new firmwares of the radios MX12/MX20/...
  uint16_t fuel_ml;         //#20 LSB Fuel in ml scale. Full = 65535!
					                  //#21 MSB
  uint16_t rpm;             //#22 RPM in 10 RPM steps. 300 = 3000rpm
				                    //#23 MSB
  uint16_t altitude;        //#24 altitude in meters. offset of 500, 500 = 0m
				                    //#25 MSB
  uint16_t climbrate_L;     //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
				                    //#27 MSB
  byte climbrate3s;         //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
  uint16_t current;         //#29 current in 0.1A steps 100 == 10,0A
				                    //#30 MSB current display only goes up to 99.9 A (continuous)
  uint16_t main_voltage;    //#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
					                  //#32 MSB (Appears in GAM display right as alternate display.)
  uint16_t batt_cap;        //#33 LSB used battery capacity in 10mAh steps
					                  //#34 MSB 
  uint16_t speed;           //#35 LSB (air?) speed in km/h(?) we are using ground speed here per default
					                  //#36 MSB speed
  byte min_cell_volt;       //#37 minimum cell voltage in 2mV steps. 124 = 2,48V
  byte min_cell_volt_num;   //#38 number of the cell with the lowest voltage
  uint16_t rpm2;            //#39 LSB 2nd RPM in 10 RPM steps. 100 == 1000rpm
					                  //#40 MSB
  byte general_error_number;//#41 General Error Number (Voice Error == 12) TODO: more documentation
  byte pressure;            //#42 High pressure up to 16bar. 0,1bar scale. 20 == 2.0bar
  byte version;             //#43 version number (Bytes 35 .43 new but not yet in the record in the display!)
  byte stop_byte;           //#44 stop byte 0x7D
  byte parity;              //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
};

//Graupner # Electric Air Module module (EAM)
struct HOTT_EAM_MSG {
  byte start_byte;          //#01 start byte constant value 0x7c
  byte eam_sensor_id;       //#02 EAM sensort id. constat value 0x8d=GENRAL AIR MODULE
  byte warning_beeps;       //#03 1=A 2=B ... 0x1a=Z  0 = no alarm
                            /* VOICE OR BIP WARNINGS
                            Alarme sonore A.. Z, octet correspondant 1 à 26
                            0x00  00  0  No alarm
                            0x01  01  A  
                            0x02  02  B  Negative Difference 2 B
                            0x03  03  C  Negative Difference 1 C
                            0x04  04  D  
                            0x05  05  E  
                            0x06  06  F  Min. Sensor 1 temp. F
                            0x07  07  G  Min. Sensor 2 temp. G
                            0x08  08  H  Max. Sensor 1 temp. H
                            0x09  09  I  Max. Sensor 2 temp. I
                            0xA   10  J  Max. Sens. 1 voltage J
                            0xB   11  K  Max. Sens. 2 voltage K
                            0xC   12  L  
                            0xD   13  M  Positive Difference 2 M
                            0xE   14  N  Positive Difference 1 N
                            0xF   15  O  Min. Altitude O
                            0x10  16  P  Min. Power Voltage P    // We use this one for Battery Warning
                            0x11  17  Q  Min. Cell voltage Q
                            0x12  18  R  Min. Sens. 1 voltage R
                            0x13  19  S  Min. Sens. 2 voltage S
                            0x14  20  T  Minimum RPM T
                            0x15  21  U  
                            0x16  22  V  Max. used capacity V
                            0x17  23  W  Max. Current W
                            0x18  24  X  Max. Power Voltage X
                            0x19  25  Y  Maximum RPM Y
                            0x1A  26  Z  Max. Altitude Z
                            */
  byte sensor_id;           //#04 constant value 0xe0
  byte alarm_invers1;       //#05 alarm bitmask. Value is displayed inverted
                            //Bit#  Alarm field
                            // 0    all cell voltage
                            // 1    Battery 1
                            // 2    Battery 2
                            // 3    Temperature 1
                            // 4    Temperature 2
                            // 5    Fuel
                            // 6    mAh
                            // 7    Altitude
  byte alarm_invers2;       //#06 alarm bitmask. Value is displayed inverted
                            //Bit#  Alarm Field
                            // 0    main power current
                            // 1    main power voltage
                            // 2    Altitude
                            // 3    m/s                            
                            // 4    m/3s
                            // 5    unknown
                            // 6    unknown
                            // 7    "ON" sign/text msg active
  byte cell_L[7];           //#7 Volt Cell_L 1 (in 2 mV increments, 210 == 4.20 V)
                            //#8 Volt Cell_L 2 (in 2 mV increments, 210 == 4.20 V)
                            //#9 Volt Cell_L 3 (in 2 mV increments, 210 == 4.20 V)
                            //#10 Volt Cell_L 4 (in 2 mV increments, 210 == 4.20 V)
                            //#11 Volt Cell_L 5 (in 2 mV increments, 210 == 4.20 V)
                            //#12 Volt Cell_L 6 (in 2 mV increments, 210 == 4.20 V)
                            //#13 Volt Cell_L 7 (in 2 mV increments, 210 == 4.20 V)
  byte cell_H[7];           //#14 Volt Cell_L 1 (in 2 mV increments, 210 == 4.20 V)
                            //#15 Volt Cell_H 2 (in 2 mV increments, 210 == 4.20 V)
                            //#16 Volt Cell_H 3 (in 2 mV increments, 210 == 4.20 V)
                            //#17 Volt Cell_H 4 (in 2 mV increments, 210 == 4.20 V)
                            //#18 Volt Cell_H 5 (in 2 mV increments, 210 == 4.20 V)
                            //#19 Volt Cell_H 6 (in 2 mV increments, 210 == 4.20 V)
                            //#20 Volt Cell_H 7 (in 2 mV increments, 210 == 4.20 V)   
  uint16_t  Battery1;       //#21 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                            //#22 MSB 
  uint16_t  Battery2;       //#23 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                            //#24 MSB
  byte temperature1;        //#25 Temperature 1. Offset of 20. a value of 20 = 0°C
  byte temperature2;        //#26 Temperature 2. Offset of 20. a value of 20 = 0°C
  uint16_t altitude;        //#27 altitude in meters. offset of 500, 500 = 0m
                            //#28 MSB  
  uint16_t current;         //#29 current in 0.1A steps 100 == 10,0A
                            //#30 MSB current display only goes up to 99.9 A (continuous)
  uint16_t main_voltage;    //#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
                            //#32 MSB (Appears in GAM display right as alternate display.)
  uint16_t batt_cap;        //#33 LSB used battery capacity in 10mAh steps
                            //#34 MSB 
  uint16_t climbrate_L;     //#35 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                            //#36 MSB
  byte climbrate3s;         //#37 climb rate in m/3sec. Value of 120 = 0m/3sec
  uint16_t rpm;             //#38 RPM in 10 RPM steps. 300 = 3000rpm
                            //#39 MSB
  byte Minutes;             //#40 Electric.Minutes (Time does start, when motor current is > 3 A)
  byte Seconds;             //#41 Electric.Seconds (1 byte)
  uint16_t speed;           //#42 LSB (air?) speed in km/h(?) we are using ground speed here per default
                            //#43 MSB speed  
  byte stop_byte;           //#44 stop byte 0x7D
  byte parity;              //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
};

//Graupner # GPS Module (GPS)
struct HOTT_GPS_MSG{
  uint8_t startByte;        //#01 0x7C = Start byte data 
  uint8_t sensorID;         //#02 0x8A = GPS Sensor
  uint8_t warning_beeps;    //#03 0…= warning beeps */
  uint8_t sensorTextID;     //#04 0xA0 Sensor ID Neu! */
  uint8_t alarmInverse1;    //#05 01 inverse status */
  uint8_t alarmInverse2;    //#06 00 inverse status status 1 = kein GPS Signal */
  uint8_t flightDirection;  //#07 119 = Flightdir./dir. 1 = 2°; 0°(North), 90°(East), 180°(South), 270°(West) */
  uint16_t GPSSpeed;        //#08 8 = /GPS speed low byte 8km/h */
                            //#09 MSB 
  uint8_t LatitudeNS;       //#10 48°39’988 #10 north = 0, south = 1*/
  uint16_t LatitudeMin;     //#11 231 0xE7 = 0x12E7 = 4839 */
                            //#12 MSB 
  uint16_t LatitudeSec;     //#13 171 220 = 0xDC = 0x03DC =0988 */
                            //#14 MSB 
  uint8_t longitudeEW;      //#15 000  = E= 9° 25’9360 east = 0, west = 1*/
  uint16_t longitudeMin;    //#16 150 157 = 0x9D = 0x039D = 0925 */
                            //#17 MSB 
  uint16_t longitudeSec;    //#18 056 144 = 0x90 0x2490 = 9360*/
                            //#19 MSB 
  uint16_t distance;        //#20 027 123 = /distance low byte 6 = 6 m */
                            //#21 MSB 
  uint16_t altitude;        //#22 243 244 = /Altitude low byte 500 = 0m */
                            //#23 MSB 
  uint16_t climbrate1s;     //#24 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                            //#25 MSB 
  int8_t climbrate3s;       //#26 climb rate in m/3sec. Value of 120 = 0m/3sec
  uint8_t GPSNumSat;        //#27 GPS.Satelites (number of satelites) (1 byte) */
  uint8_t GPSFixChar;       //#28 GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
  uint8_t HomeDirection;    //#29 HomeDirection (direction from starting point to Model position) (1 byte) */
  uint8_t angleXdirection;  //#30 angle x-direction (1 byte) */
  uint8_t angleYdirection;  //#31 angle y-direction (1 byte) */
  uint8_t angleZdirection;  //#32 angle z-direction (1 byte) */
  int8_t gps_time_h;        //#33 UTC time hours
  int8_t gps_time_m;        //#34 UTC time minutes
  int8_t gps_time_s;        //#35 UTC time seconds
  int8_t gps_time_sss;      //#36 UTC time milliseconds
  int16_t msl_altitude;     //#37 mean sea level altitude
                            //#38 MSB
  uint8_t vibration;        //#39 vibration (1 bytes) */
  uint8_t Ascii4;           //#40 00 ASCII Free Character [4] */
  uint8_t Ascii5;           //#41 00 ASCII Free Character [5] */
  uint8_t GPS_fix;          //#42 00 ASCII Free Character [6], we use it for GPS FIX */
  uint8_t version;          //#43 00 version number */
  uint8_t endByte;          //#44 0x7D Ende byte */
  uint8_t parity;           //#45 Parity Byte */
} ;

//Graupner # Vario Sensor Module (VARIO)
struct HOTT_VARIO_MSG{
  uint8_t start_byte;        //#1 0x7C = Start byte data
  uint8_t vario_sensor_id;         //#2 0x89 = Vario Sensor
  uint8_t warning_beeps;    //#3 0…= warning beeps
                            /*  VOICE OR BIP WARNINGS
                            0x00  00  No alarm
                            0x01  01  A 
                            0x02  02  B  Descent / B 3s -1 m / s
                            0x03  03  C  Descent / s C -10 m / s
                            0x04  04  D  
                            0x05  05  E  
                            0x06  06  F  
                            0x07  07  G  
                            0x08  08  H  
                            0x09  09  I  
                            0xA   10  J  
                            0xB   11  K  
                            0xC   12  L  
                            0xD   13  M  Rate of Climb / 3s M 1 m / s
                            0xE   14  N  Rate of Climb / s N 10 m / s
                            0xF   15  O  Min Alt O
                            0x10  16  P  
                            0x11  17  Q  
                            0x12  18  R  
                            0x13  19  S  
                            0x14  20  T  
                            0x15  21  U  
                            0x16  22  V  
                            0x17  23  W  
                            0x18  24  X  
                            0x19  25  Y  
                            0x1A  26  Z  Max Alt Z
                            */
  uint8_t sensor_id;        //#4 144 0x90 Sensor ID Neu!
  uint8_t alarmInverse;     //#5 0 Inverse status
  uint16_t altitude;        //#6 LSB 244 244 = Low Byte Actual Altitude -2m (244+256=0m)
                            //#7 MSB 1 1 = actual altitude (500 = 0m)
  uint16_t maxAltitude;     //#8 LSB 244 244 = Low Byte Maximum Altitude. -2m (244+256=0m)
                            //#9 MSB 1 = max. altitude (500 = 0m)
  uint16_t minAltitude;     //#10 LSB 244 244 = Low Byte min. altitude -2m (244+256=0m)
                            //#11 MSB 1 = High Byte min. altitude (500 = 0m)
  uint16_t m1s;             //#12 LSB 48 Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s)
                            //#13 MSB 117 117 = High Byte m/s resolution 0.01m 117
  uint16_t m3s;             //#14 LSB 48 Low Byte m/3s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/3s)
                            //#15 MSB 117 117 = High Byte m/3s resolution 0.01m 117
  uint16_t m10s;            //#16 LSB 48 Low Byte m/10s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/10s)
                            //#17 MSB 117 117 = High Byte m/10s resolution 0.01m 117
  uint8_t text[24];         //#18: 0 ASCII [1]
                            //#19: 0 ASCII [2]
                            //#20: 0 ASCII [3]
                            //#21: 0 ASCII [4]
                            //#22: 0 ASCII [5]
                            //#23: 0 ASCII [6]
                            //#24: 0 ASCII [7]
                            //#25: 0 ASCII [8]
                            //#26: 0 ASCII [9]
                            //#27: 0 ASCII [10]
                            //#28: 0 ASCII [11]
                            //#29: 0 ASCII [12]
                            //#30: 0 ASCII [13]
                            //#31: 0 ASCII [14]
                            //#32: 0 ASCII [15]
                            //#33: 0 ASCII [16]
                            //#34: 0 ASCII [17]
                            //#35: 0 ASCII [18]
                            //#36: 0 ASCII [19]
                            //#37: 0 ASCII [20]
                            //#38: 0 ASCII [21]
                            //#39: 0 ASCII Free Character [1]
                            //#40: 0 ASCII Free Character [2]
                            //#41: 0 ASCII Free Character [3]
  uint8_t empty;            //#42 0 free
  uint8_t version;          //#43 0 version number
  uint8_t stop_byte;        //#44 0x7D End byte
  uint8_t parity;           //#45 CHECKSUM Parity Byte
};

//Graupner #  Telemetry Speed Controller 70A ESC (ESC)
struct HOTT_ESC_MSG {
  byte start_byte;          //#01 start byte constant value 0x7c
  byte esc_sensor_id;       //#02 EAM sensort id. constat value 0x8c=GENRAL AIR MODULE
  byte warning_beeps;       //#03 1=A 2=B ... 0x1a=Z  0 = no alarm
                            /* VOICE OR BIP WARNINGS
                            Alarme sonore A.. Z, octet correspondant 1 à 26
                            0x00  00  0  No alarm
                            0x01  01  A  
                            0x02  02  B  Negative Difference 2 B
                            0x03  03  C  Negative Difference 1 C
                            0x04  04  D  
                            0x05  05  E  
                            0x06  06  F  Min. Sensor 1 temp. F
                            0x07  07  G  Min. Sensor 2 temp. G
                            0x08  08  H  Max. Sensor 1 temp. H
                            0x09  09  I  Max. Sensor 2 temp. I
                            0xA   10  J  Max. Sens. 1 voltage J
                            0xB   11  K  Max. Sens. 2 voltage K
                            0xC   12  L  
                            0xD   13  M  Positive Difference 2 M
                            0xE   14  N  Positive Difference 1 N
                            0xF   15  O  Min. Altitude O
                            0x10  16  P  Min. Power Voltage P    // We use this one for Battery Warning
                            0x11  17  Q  Min. Cell voltage Q
                            0x12  18  R  Min. Sens. 1 voltage R
                            0x13  19  S  Min. Sens. 2 voltage S
                            0x14  20  T  Minimum RPM T
                            0x15  21  U  
                            0x16  22  V  Max. used capacity V
                            0x17  23  W  Max. Current W
                            0x18  24  X  Max. Power Voltage X
                            0x19  25  Y  Maximum RPM Y
                            0x1A  26  Z  Max. Altitude Z
                            */
  byte sensor_id;           //#04 constant value 0xd0
  byte alarm_invers1;       //#05 alarm bitmask. Value is displayed inverted
                            //Bit#  Alarm field
                            // 0    all cell voltage
                            // 1    Battery 1
                            // 2    Battery 2
                            // 3    Temperature 1
                            // 4    Temperature 2
                            // 5    Fuel
                            // 6    mAh
                            // 7    Altitude
  byte alarm_invers2;       //#05 alarm bitmask. Value is displayed inverted
                            //Bit#  Alarm Field
                            // 0    main power current
                            // 1    main power voltage
                            // 2    Altitude
                            // 3    m/s                            
                            // 4    m/3s
                            // 5    unknown
                            // 6    unknown
                            // 7    "ON" sign/text msg active
  uint16_t  voltage;        //#06 voltage 0.1V steps. 55 = 5.5V
                            //#07 MSB               
  uint16_t  voltage_min;    //#08 voltage min 0.1V steps. 55 = 5.5V
                            //#09 MSB     
  uint16_t batt_cap;        //#10 used battery capacity in 10mAh steps
                            //#11 MSB
  byte temp1;               //#12 Temperature 1. Offset of 20. a value of 20 = 0°C
  byte temp1_max;           //#13 Temperature 1. max Offset of 20. a value of 20 = 0°C
  uint16_t current;         //#14 current in 0.1A steps 100 == 10,0A
                            //#15 MSB
  uint16_t current_max;     //#16 current max in 0.1A steps 100 == 10,0A
                            //#17 MSB
  uint16_t RPM;             //#18 RPM in 10 RPM steps. 300 = 3000rpm
                            //#19 MSB
  uint16_t RPM_max;         //#20 RPM max in 10 RPM steps. 300 = 3000rpm
                            //#21 MSB
  byte temp2;               //#22 Temperature 2. Offset of 20. a value of 20 = 0°C
  byte temp2_max;           //#23 Temperature 2. max Offset of 20. a value of 20 = 0°C
  byte dummy[19];           //#24-43 Dummy
  byte stop_byte;           //#44 stop byte 0x7D
  byte parity;              //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
};


class GMessage {

private:  
  void _hott_send_text_msg();
  char * _hott_invert_all_chars(char *str);
  char * _hott_invert_chars(char *str, int cnt);
  void init_gam_msg();
  void init_eam_msg();
  void init_gps_msg();
  void init_vario_msg();
  void init_esc_msg();
  void send(int lenght);
  void _hott_invert_ligne(int ligne) ;
  uint16_t read_eprom(int address);
  void write_eprom(int address,uint16_t val) ;

public:
  GMessage();
  void init();
  void main_loop();
  void debug();
  
  // Messbereiche im Eprom
  int getVoltOffset();
  int getVoltCOEF();
  int getCurrentOffset();
  int getCurrentCOEF();
  
};
