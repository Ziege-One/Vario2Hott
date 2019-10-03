/*
   Vario2HoTT
   Ziege-One
   v1.0
 
 Arduino pro Mini 5v/16mHz w/ Atmega 328
 
 */

#include "Message.h"
#include <EEPROM.h>
#include <SoftwareSerial.h>
 
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) //Code in here will only be compiled if an Arduino Uno (or older) is used.
  #define HOTTV4_RXTX 3           // Pin für HoTT Telemetrie Ausgang 2-7
  /**
    * Enables RX and disables TX
    */
  static inline void hottV4EnableReceiverMode() {
    DDRD  &= ~(1 << HOTTV4_RXTX);
    PORTD |= (1 << HOTTV4_RXTX);
  }

  /**
    * Enabels TX and disables RX
    */
  static inline void hottV4EnableTransmitterMode() {
    DDRD |= (1 << HOTTV4_RXTX);
  }
#endif
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) //Code in here will only be compiled if an Arduino Leonardo is used.
  #define HOTTV4_RXTX 9           // Pin für HoTT Telemetrie Ausgang 8-11
  /**
    * Enables RX and disables TX
    */
  static inline void hottV4EnableReceiverMode() {
    DDRB  &= ~(1 << (HOTTV4_RXTX - 4));
    PORTB |= (1 << (HOTTV4_RXTX - 4));
  }

  /**
    * Enabels TX and disables RX
    */
  static inline void hottV4EnableTransmitterMode() {
    DDRB |= (1 << (HOTTV4_RXTX - 4));
  }
#endif 

#define LEDPIN_OFF        digitalWrite(LED_BUILTIN, LOW);
#define LEDPIN_ON         digitalWrite(LED_BUILTIN, HIGH);

// Externe variable for Hott Telemetry
extern uint16_t altitude;
extern uint16_t m1s;
extern uint16_t m3s;
extern uint16_t m10s;
extern uint16_t maxAltitude;
extern uint16_t minAltitude;

extern float Altitude;
extern float M1s;
extern float M3s;
extern float M10s;
extern float MaxAltitude;
extern float MinAltitude;
 
SoftwareSerial SERIAL_HOTT(HOTTV4_RXTX , HOTTV4_RXTX); // RX, TX

static uint8_t _hott_serial_buffer[173];   //creating a buffer variable to store the struct

// pointer to the buffer structures "_hott_serial_buffer"
struct HOTT_TEXTMODE_MSG	*hott_txt_msg =	(struct HOTT_TEXTMODE_MSG *)&_hott_serial_buffer[0];

struct HOTT_VARIO_MSG     *hott_vario_msg = (struct HOTT_VARIO_MSG *)&_hott_serial_buffer[0];


// alarmists in the EPROM
static int min_height = 20;    // Minimale Flughöhe - 500 bis 3000 m 
static int max_height = 500;   // Maximale Flughöhe - 500 bis 3000 m 
static int neg_diff_1 = -10;   // Sinkrate / Sek. - 50.0 bis 0 m 
static int neg_diff_2 = -1;    // Sinkrate / 3 Sek. - 50.0 bis 0 m  
static int pos_diff_1 = 10;    // Steigrate / Sek. 0 bis 50.0 m 
static int pos_diff_2 = 1;     // Steigrate / 3 Sek. 0 bis 50.0 m 

// Timer for alarm
// for refresh time
int alarm_interval = 5000; // in ms
static unsigned long lastTime=0;  // in ms
unsigned long time=millis();      // in ms

// Messbereiche im Eprom
int Volt_Offset;
int Volt_SCALE;
int Current_Offset;
int Current_SCALE;

int GMessage::getVoltOffset() {return Volt_Offset; }
int GMessage::getVoltCOEF() {return Volt_SCALE; }
int GMessage::getCurrentOffset() {return Current_Offset; }
int GMessage::getCurrentCOEF() {return Current_SCALE; }

// For communication
static uint8_t octet1 = 0;  // reception
static uint8_t octet2 = 0;  // reception

// For saving settings in EEPROM
/*
 !WARNING!
 Writing takes 3.3ms.
 Maximum life of the EEPROM is 100000 writings/readings.
 Be careful not to use it too much, it is not replacable!
 */
#define adr_eprom_test 0          // For the test for 1st time init of the Arduino (First power on)
#define adr_eprom_min_height 2    // Minimale Flughöhe - 500 bis 3000 m 
#define adr_eprom_max_height 4    // Maximale Flughöhe - 500 bis 3000 m 
#define adr_eprom_neg_diff_1 6    // Sinkrate / Sek. - 50.0 bis 0 m 
#define adr_eprom_neg_diff_2 8    // Sinkrate / 3 Sek. - 50.0 bis 0 m  
#define adr_eprom_pos_diff_1 10   // Steigrate / Sek. 0 bis 50.0 m 
#define adr_eprom_pos_diff_2 12   // Steigrate / 3 Sek. 0 bis 50.0 m 

GMessage::GMessage(){

}
void GMessage::init(){
  
  SERIAL_HOTT.begin(SERIAL_COM_SPEED); // 19400 FOR GRAUPNER HOTT using SoftSerial lib.
  hottV4EnableReceiverMode(); 
  
  // Test for 1st time init of the Arduino (First power on)
  int test = read_eprom(adr_eprom_test);
  if (test != 123)
  {
    write_eprom(adr_eprom_test,123);
    write_eprom(adr_eprom_min_height,min_height);
    write_eprom(adr_eprom_max_height,max_height);
    write_eprom(adr_eprom_neg_diff_1,neg_diff_1);
    write_eprom(adr_eprom_neg_diff_2,neg_diff_2);
    write_eprom(adr_eprom_pos_diff_1,pos_diff_1);
    write_eprom(adr_eprom_pos_diff_2,pos_diff_2);
  }
  // Read saved values from EEPROM
    // alarm min on battery
    min_height = read_eprom(adr_eprom_min_height); 
    max_height = read_eprom(adr_eprom_max_height); 
    neg_diff_1 = read_eprom(adr_eprom_neg_diff_1); 
    neg_diff_2 = read_eprom(adr_eprom_neg_diff_2);
    pos_diff_1 = read_eprom(adr_eprom_pos_diff_1);
    pos_diff_2 = read_eprom(adr_eprom_pos_diff_2);
    
}

uint16_t GMessage::read_eprom(int address){
  return  (uint16_t) EEPROM.read(address) * 256 + EEPROM.read(address+1) ;
}

void GMessage::write_eprom(int address,uint16_t val){
  EEPROM.write(address, val  / 256);
  EEPROM.write(address+1,val % 256 );
}

void GMessage::init_vario_msg(){
  //puts to all Zero, then modifies the constants
  memset(hott_vario_msg, 0, sizeof(struct HOTT_VARIO_MSG));   
  hott_vario_msg->start_byte = 0x7c;
  hott_vario_msg->vario_sensor_id = HOTT_TELEMETRY_VARIO_SENSOR_ID;
  hott_vario_msg->sensor_id = HOTT_TELEMETRY_VARIO_SENSOR_TEXT;
  hott_vario_msg->stop_byte = 0x7d;
}


// Sending the frame
void GMessage::send(int lenght){ 
  uint8_t sum = 0;
  hottV4EnableTransmitterMode(); 
  delay(5);
  for(int i = 0; i < lenght-1; i++){
    sum = sum + _hott_serial_buffer[i];
    SERIAL_HOTT.write (_hott_serial_buffer[i]);
    delayMicroseconds(HOTTV4_TX_DELAY);
  }  
  //Emision checksum
  SERIAL_HOTT.write (sum);
  delayMicroseconds(HOTTV4_TX_DELAY);

  hottV4EnableReceiverMode();
}


void GMessage::main_loop(){ 
  
  // STARTING MAIN PROGRAM
  static byte page_settings = 1; // page number to display settings

  if(SERIAL_HOTT.available() >= 2) {
    uint8_t octet1 = SERIAL_HOTT.read();
    switch (octet1) {
    case HOTT_BINARY_MODE_REQUEST_ID:
      { 
        uint8_t  octet2 = SERIAL_HOTT.read();
        
        // Demande RX Module =	$80 $XX
        switch (octet2) {
 
         case HOTT_TELEMETRY_VARIO_SENSOR_ID: //0x89
          {    
               LEDPIN_ON
           
            // init structure
               init_vario_msg();
               hott_vario_msg->warning_beeps = 0;
               hott_vario_msg->altitude = altitude;          // (500 = 0m)
               hott_vario_msg->maxAltitude = maxAltitude;    // (500 = 0m)
               hott_vario_msg->minAltitude = minAltitude;    // (500 = 0m)
               hott_vario_msg->m1s = m1s;                    // 30000 = 0.00m/s (1=0.01m/s)
               hott_vario_msg->m3s = m3s;                    // 30000 = 0.00m/s (1=0.01m/s)
               hott_vario_msg->m10s = m10s;                  // 30000 = 0.00m/s (1=0.01m/s)
               snprintf((char *)&hott_vario_msg->text[0],21 ,"Vario2Hott V1.0");    

            // Alarmmanagement
               time=millis();
               if (time-lastTime > alarm_interval)  // if at least alarm_interval in ms have passed
               {
                     // Check for alarm beep
                     if (Altitude < min_height && min_height != 0) // wenn 0 kein Alarm
                     {
                     hott_txt_msg->warning_beeps =  0x0F    ; // alarm beep or voice for Min. Altitude O
                     }
                     if (Altitude > max_height && max_height != 0) // wenn 0 kein Alarm)
                     {
                     hott_txt_msg->warning_beeps =  0x1A    ; // alarm beep or voice for Max. Altitude Z
                     }
                     if (M1s < neg_diff_1 && neg_diff_2 != 0) // wenn 0 kein Alarm)
                     {
                     hott_txt_msg->warning_beeps =  0x03    ; // alarm beep or voice for Negative Difference 1 C
                     }
                     if (M3s < neg_diff_2 && neg_diff_2 != 0) // wenn 0 kein Alarm)
                     {
                     hott_txt_msg->warning_beeps =  0x02    ; // alarm beep or voice for Negative Difference 2 B
                     }
                     if (M1s > pos_diff_1 && pos_diff_1 != 0) // wenn 0 kein Alarm)
                     {
                     hott_txt_msg->warning_beeps =  0x0E    ; // alarm beep or voice for Positive Difference 1 N
                     }
                     if (M3s > pos_diff_2 && pos_diff_2 != 0) // wenn 0 kein Alarm)
                     {
                     hott_txt_msg->warning_beeps =  0x0D    ; // alarm beep or voice for Positive Difference 2 M
                     }                     
                 lastTime=time;  // reset timer
               }          
               
            // sending all data
            send(sizeof(struct HOTT_VARIO_MSG));
            break;
          } //end case Vario*/
     
        } //end case octet 2
        break;
      }

    case HOTT_TEXT_MODE_REQUEST_ID:
      {
        //LEDPIN_ON
        uint8_t  octet3 = SERIAL_HOTT.read();
        byte id_sensor = (octet3 >> 4);
        byte id_key = octet3 & 0x0f;
        static byte ligne_select = 4 ;
        static int8_t ligne_edit = -1 ;
        hott_txt_msg->start_byte = 0x7b;
        hott_txt_msg->esc = 0x00;
        hott_txt_msg->warning_beeps = 0x00;
        
        memset((char *)&hott_txt_msg->text, 0x20, HOTT_TEXTMODE_MSG_TEXT_LEN);
        hott_txt_msg->stop_byte = 0x7d;

        if (id_key == HOTT_KEY_LEFT && page_settings == 1)
        {   
          hott_txt_msg->esc = 0x01;
        }
        else
        {
          if (id_sensor == (HOTT_TELEMETRY_VARIO_SENSOR_ID & 0x0f)) 
          { 
            hott_txt_msg->esc = HOTT_TELEMETRY_VARIO_SENSOR_TEXT;
            switch (page_settings) { //SETTINGS
              
              case 1://PAGE 1 SETTINGS
              
                    {                    
                    if (id_key == HOTT_KEY_LEFT && ligne_edit == -1)
                        {
                        page_settings-=1;
                        if (page_settings <1)    // unter Seite 1 dann Seite 2
                          page_settings = 2;
                        }
                    else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        page_settings+=1;
                        if (page_settings >2)   // Über Seite 2 dann Seite 1
                          page_settings = 1;
                      }
                      
                    // Showing page 1
                    
                    char str_temp[6];
                    
                    //line 0:
                    snprintf((char *)&hott_txt_msg->text[0],21," Vario Sensor      >");
                    //line 1:
                    dtostrf(Altitude, 4, 2, str_temp);
                    snprintf((char *)&hott_txt_msg->text[1],21," Altitude  : %sM", str_temp);
                    //line 2:
                    dtostrf(MinAltitude, 4, 2, str_temp);
                    snprintf((char *)&hott_txt_msg->text[2],21," Min.      : %sM", str_temp);
                    //line 3:
                    dtostrf(MaxAltitude, 4, 2, str_temp);
                    snprintf((char *)&hott_txt_msg->text[3],21," Max.      : %sM", str_temp);
                    //line 4:
                    dtostrf(M1s, 4, 2, str_temp);
                    snprintf((char *)&hott_txt_msg->text[4],21," Diff./Sec : %sM", str_temp);
                    //line 5:
                    dtostrf(M3s, 4, 2, str_temp);
                    snprintf((char *)&hott_txt_msg->text[5],21," Diff./3S  : %sM", str_temp);    
                    //line 6:
                    dtostrf(M10s, 4, 2, str_temp);
                    snprintf((char *)&hott_txt_msg->text[6],21," Diff./10S : %sM", str_temp);
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Vario2HoTT  %d/2",page_settings); //Showing page number running down the screen to the right
                    
                    hott_txt_msg->text[ligne_select][0] = '>';
                    _hott_invert_ligne(ligne_edit);
                    break;                    
                    }//END PAGE 1
                    
               case 2: // PAGE 2
                    {
                      // config test for the screen display has
                    if (id_key == HOTT_KEY_LEFT && ligne_edit == -1)
                        {
                        page_settings-=1;
                        if (page_settings <1)    // unter Seite 1 dann Seite 2
                          page_settings = 2;
                        }
                    else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        page_settings+=1;
                        if (page_settings >2)   // Über Seite 2 dann Seite 1
                          page_settings = 1;
                      }
                                                      
                    else if (id_key == HOTT_KEY_UP && ligne_edit == -1)
                    ligne_select = min(6,ligne_select+1); // never gets above line 6 max
                    else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
                    ligne_select = max(1,ligne_select-1); // never gets above line 3 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    ligne_edit =  ligne_select ;

                    //LINE 1 SELECTED = text[1]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 1 )
                      min_height+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 1 )
                      min_height-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 1 )
                      min_height+=25;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 1 )
                      min_height-=25;                      
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 1)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_min_height,min_height);
                       }

                    else if (min_height>3000) // not over 3000
                        {
                          min_height=3000;
                        } 
                    else if (min_height<-500)  // not behind -500
                        {
                          min_height=-500;
                        }

                    //LINE 2 SELECTED = text[2]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 2 )
                      max_height+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 2 )
                      max_height-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 2 )
                      max_height+=25;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 2 )
                      max_height-=25;                      
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 2)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_max_height,max_height);
                       }

                    else if (max_height>3000) // not over 3000
                        {
                          max_height=3000;
                        } 
                    else if (max_height<0)  // not behind -500
                        {
                          max_height=-500;
                        }

                    //LINE 3 SELECTED = text[3]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 3 )
                      neg_diff_1+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 3 )
                      neg_diff_1-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 3 )
                      neg_diff_1+=5;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 3 )
                      neg_diff_1-=5;                      
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 3)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_neg_diff_1,neg_diff_1);
                       }

                    else if (neg_diff_1>0) // not over 0
                        {
                          neg_diff_1=0;
                        } 
                    else if (neg_diff_1<-50)  // not behind -50
                        {
                          neg_diff_1=-50;
                        }
                    
                    //LINE 4 SELECTED = text[4]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 4 )
                      neg_diff_2+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 4 )
                      neg_diff_2-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 4 )
                      neg_diff_2+=5;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 4 )
                      neg_diff_2-=5;                         
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 4)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_neg_diff_2,neg_diff_2);
                       }

                    else if (neg_diff_2>0) // not over 0
                        {
                          neg_diff_2=0;
                        } 
                    else if (neg_diff_2<-50)  // not behind -50
                        {
                          neg_diff_2=-50;
                        }
                    
                    //LINE 5 SELECTED = text[5]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 5 )
                      pos_diff_1+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 5 )
                      pos_diff_1-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 5 )
                      pos_diff_1+=5;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 5 )
                      pos_diff_1-=5;                         
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 5)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_pos_diff_1,pos_diff_1);
                       }

                    else if (pos_diff_1>50) // not over 50
                        {
                          pos_diff_1=50;
                        } 
                    else if (pos_diff_1<0)  // not behind 0
                        {
                          pos_diff_1=0;
                        }
                    
                    //LINE 6 SELECTED = text[6]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 6 )
                      pos_diff_2+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 6 )
                      pos_diff_2-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 6 )
                      pos_diff_2+=5;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 6 )
                      pos_diff_2-=5;                        
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 6)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_pos_diff_2,pos_diff_2);
                       }

                    else if (pos_diff_2>50) // not over 50
                        {
                          pos_diff_2=50;
                        } 
                    else if (pos_diff_2<0)  // not behind 0
                        {
                          pos_diff_2=0;
                        }                    
                    
                    // Showing page 2 settings
                    //line 0:                                  
                    snprintf((char *)&hott_txt_msg->text[0],21," Set Warning      <");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21," Min. Height : %iM",(min_height)); // Minimale Flughöhe - 500 bis 3000 m
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21," Max. Height : %iM",(max_height)); // Maximale Flughöhe - 500 bis 3000 m
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21," Neg.Diff.1  : %iM",(neg_diff_1)); // Sinkrate / Sek. - 50.0 bis 0 m
                    //line 4:
                    snprintf((char *)&hott_txt_msg->text[4],21," Neg.Diff.2  : %iM",(neg_diff_2)); // Sinkrate / 3 Sek. - 50.0 bis 0 m
                    //line 5:
                    snprintf((char *)&hott_txt_msg->text[5],21," Pos.Diff.1  : %iM",(pos_diff_1)); // Steigrate / Sek. 0 bis 50.0 m 
                    //line 6:
                    snprintf((char *)&hott_txt_msg->text[6],21," Pos.Diff.2  : %iM",(pos_diff_2)); // Steigrate / 3 Sek. 0 bis 50.0 m
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Vario2HoTT  %d/2",page_settings); //Showing page number running down the screen to the right
                    int Volt_Offset;

                    hott_txt_msg->text[ligne_select][0] = '>';
                    _hott_invert_ligne(ligne_edit);
                    break;
                    }//END PAGE 2
                    
                  default://PAGE 
                  {
                    break;
                  }
                  
                  
                  
            }//END SETTINGS

          } // END IF
          
             
          else {
            snprintf((char *)&hott_txt_msg->text[0],21,"Unknow sensor module <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }
        }
        _hott_send_text_msg();
        //LEDPIN_OFF
        break;

      }
      break;
    }	
  }
}




void GMessage::_hott_send_text_msg() {
  for(byte *_hott_msg_ptr = hott_txt_msg->text[0]; _hott_msg_ptr < &hott_txt_msg->stop_byte ; _hott_msg_ptr++){
    if (*_hott_msg_ptr == 0)
      *_hott_msg_ptr = 0x20;
  }  
  send(sizeof(struct HOTT_TEXTMODE_MSG));
}


char * GMessage::_hott_invert_all_chars(char *str) {
  return _hott_invert_chars(str, 0);
}


char * GMessage::_hott_invert_chars(char *str, int cnt) {
  if(str == 0) return str;
  int len = strlen(str);
  if((len < cnt)  && cnt > 0) len = cnt;
  for(int i=0; i< len; i++) {
    str[i] = (byte)(0x80 + (byte)str[i]);
  }
  return str;
}


void GMessage::_hott_invert_ligne(int ligne) {
  if (ligne>= 0 && ligne<= 7)
    for(int i=0; i< 21; i++) {
      if (hott_txt_msg->text[ligne][i] == 0)   //reversing the null character (end of string)
        hott_txt_msg->text[ligne][i] = (byte)(0x80 + 0x20);
      else
        hott_txt_msg->text[ligne][i] = (byte)(0x80 + (byte)hott_txt_msg->text[ligne][i]);
    }
}

void GMessage::debug(){
   // FOR DEBUG
    Serial.println("-----------Vario2HoTT-V1.0------------");
    Serial.print(" Altitude = ");
    Serial.print(Altitude);
    Serial.print(", M1s = ");
    Serial.print(M1s);
    Serial.print(", M3s = ");
    Serial.print(M3s);
    Serial.print(", M10s = ");
    Serial.print(M10s);
    Serial.print(", MaxAltitude = ");
    Serial.print(MaxAltitude);
    Serial.print(", MinAltitude = ");
    Serial.println(MinAltitude);     
}
