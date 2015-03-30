// ****************************************************************
// FrSky telemetry 
// Version: 0.4.0   - by QuadBow
// Changes: V0.4.0: - supports 2.4 with new features added
//                    device specific selection of data to be sent
//                    different ways for displaying             
//                    different ways to present data
// Version: 0.3.0
// Changes: V0.3.0: - new structure with cpp/h-files
// Date 20/09/2012
// Changes: V0.2.1: - make it work with 2.1 (shared dev)
// Date: 14/08/2012
// Changes: V0.2: - Byte stuffing added
//                - vBat will be send, if "#define FAS_100" is comment out
//                V0.1: - First release
// ****************************************************************

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Sensors.h"
#include "Serial.h"
#include "Telemetry.h"

#if defined(FRSKY_TELEMETRY)

void init_telemetry(void)
{
  SerialOpen(TELEMETRY_SERIAL,TELEMETRY_BAUD);
}

void write_FrSky8(uint8_t Data)
{
	SerialWrite(TELEMETRY_SERIAL, Data);
}

void check_FrSky_stuffing(uint8_t Data) //byte stuffing
{
	if (Data == 0x5E) {
	   write_FrSky8(0x5D);
	   write_FrSky8(0x3E);
	   }
	else if (Data == 0x5D) {
		write_FrSky8(0x5D);
		write_FrSky8(0x3D);
	        }
	else    {
		write_FrSky8(Data);
	        }
}

void write_FrSky16(uint16_t Data)
{
  uint8_t Data_send;
  Data_send = Data;      
  check_FrSky_stuffing(Data_send);
  Data_send = Data >> 8 & 0xff;
  check_FrSky_stuffing(Data_send);
}

static void sendDataHead(uint8_t Data_id)
{
  write_FrSky8(Protocol_Header);
  write_FrSky8(Data_id);
}

static void sendDataTail(void)
{
  write_FrSky8(Protocol_Tail);      
}

//*********************************************************************************
//-----------------   Telemetrie Data   ------------------------------------------   
//*********************************************************************************

// Temperature and NumSats
void inline send_Temperature(void)
{
  #if BARO
      int16_t Data_Temperature1;
      Data_Temperature1 = (baroTemperature + 50) / 100;
      sendDataHead(ID_Temperature1);
      write_FrSky16(Data_Temperature1);
  #endif
  #if GPS
      int16_t Data_NumSats;
      if (f.GPS_FIX && GPS_numSat >= 4) {           
         Data_NumSats = GPS_numSat;
         sendDataHead(ID_Temperature2);
         write_FrSky16(Data_NumSats);
         }
  #endif
}

// RPM is interpreted as Return distance to home Position in Meter
void inline send_RPM(void)
{
  #if GPS
      uint16_t Data_RPM;
      if (f.GPS_FIX && GPS_numSat >= 4) {           
         Data_RPM = GPS_distanceToHome; // Distance to home alias RPM
         sendDataHead(ID_RPM);
         write_FrSky16(Data_RPM);
         }     
  #endif
}

// Fuel level
void inline send_Fuel(void)
{
  #if defined(POWERMETER)
      uint16_t Data_Fuel;
      if ((pMeter[PMOTOR_SUM] < (pAlarm / 4)) || (pAlarm == 0))
         Data_Fuel = 100;
      else if (pMeter[PMOTOR_SUM] < (pAlarm / 2))
         Data_Fuel = 75;
      else if (pMeter[PMOTOR_SUM] < (3 * pAlarm / 4))
         Data_Fuel = 50;
      else if (pMeter[PMOTOR_SUM] < pAlarm)
         Data_Fuel = 25;
      else
         Data_Fuel = 0;
      sendDataHead(ID_Fuel_level);
      write_FrSky16(Data_Fuel);
  #endif
}

// Cell voltage 
void inline send_cell_volt(void) // Data compatibel to FrSky FLVS-01 voltage sensor
{
  #if defined(VBAT_CELLS)
      uint16_t Data_Volt;
      uint16_t temp;
      static uint8_t cell_counter = 0;
      // the resolution of analog.vbatcells results in only one decimal (eg 3.70V or 3.80V and nothing in between)
      // TODO: improve resolution of analog.vbatcells
      temp = 50 * analog.vbatcells[cell_counter];
      Data_Volt = (temp << 8) + (temp >> 8) + (cell_counter << 4);
      if (++cell_counter >= VBAT_CELLS_NUM)
         cell_counter = 0;
      sendDataHead(ID_Volt);
      write_FrSky16(Data_Volt);
  #endif
}

// Altitude
void inline send_Altitude(void)
{
  #if defined(TELEMETRY_ALT_BARO) and BARO
      int16_t Data_altitude;
      Data_altitude = (alt.EstAlt + 50) / 100;
      sendDataHead(ID_Altitude);
      write_FrSky16(Data_altitude);
  #endif
  #if defined(TELEMETRY_ALT_GPS) and GPS
      #if defined(FRSKY_FLD02) 
          #if not defined(TELEMETRY_ALT_BARO)
              int16_t Data_altitude;
              if (f.GPS_FIX && GPS_numSat >= 4) {           
                 Data_altitude = GPS_altitude;
                 sendDataHead(ID_Altitude);
                 write_FrSky16(Data_altitude);
                 }
          #endif
      #else
          int16_t Data_GPS_altitude_bp;
          uint16_t Data_GPS_altitude_ap;
          if (f.GPS_FIX && GPS_numSat >= 4) {
             Data_GPS_altitude_bp = GPS_altitude;
             Data_GPS_altitude_ap = 0;
             sendDataHead(ID_GPS_Altitude_bp);
             write_FrSky16(Data_GPS_altitude_bp);
             sendDataHead(ID_GPS_Altitude_ap);
             write_FrSky16(Data_GPS_altitude_ap);
             }
      #endif
  #endif
}

// Course
void inline send_Course(void)
{
  #if not defined(FRSKY_FLD02)
      #if defined TELEMETRY_COURSE_GPS and GPS
          uint16_t Data_Course_bp;
          uint16_t Data_Course_ap;
          if (f.GPS_FIX && GPS_numSat >= 4) {
              Data_Course_bp = GPS_ground_course / 10;
              Data_Course_ap = GPS_ground_course - Data_Course_bp * 10;
              sendDataHead(ID_Course_bp);
              write_FrSky16(Data_Course_bp);
              sendDataHead(ID_Course_ap);
              write_FrSky16(Data_Course_ap);
              }
      #elif defined TELEMETRY_COURSE_MAG and MAG
          uint16_t Data_Course_bp;
          uint16_t Data_Course_ap;
          Data_Course_bp = att.heading;
          Data_Course_ap = 0;
          sendDataHead(ID_Course_bp);
          write_FrSky16(Data_Course_bp);
          sendDataHead(ID_Course_ap);
          write_FrSky16(Data_Course_ap);
      #endif
  #endif
}

// GPS speed
void inline send_GPS_speed(void)
{
  #if GPS
      uint16_t Data_GPS_speed_bp;
      uint16_t Data_GPS_speed_ap;
      uint16_t temp;
      if (f.GPS_FIX && GPS_numSat >= 4) {           
         temp = (GPS_speed * 40 + 102) / 203;                       // FRSKY specific format in knots => factor ~50 (102 for rounding purpose)
         Data_GPS_speed_bp = temp / 10;
         Data_GPS_speed_ap = temp - Data_GPS_speed_bp * 10;
         sendDataHead(ID_GPS_speed_bp);
         write_FrSky16(Data_GPS_speed_bp);
         sendDataHead(ID_GPS_speed_ap);
         write_FrSky16(Data_GPS_speed_ap);
         }
  #endif
}

// GPS position
void inline send_GPS_longitude(void)
{
  #if GPS
      uint16_t Data_Longitude_bp;
      uint16_t Data_Longitude_ap;
      uint16_t Data_E_W;
      uint32_t temp, rest, decimal;
      if (f.GPS_FIX && GPS_numSat >= 4) {           
         temp = abs(GPS_coord[LON]);
         #if defined(COORDFORMAT_DECIMALMINUTES)
             decimal = temp / 10000000;
             temp -= decimal * 10000000;
             temp *= 6;
             rest = temp;
             temp /= 1000000;
             rest -= temp * 1000000;
             Data_Longitude_bp = decimal * 100 + temp;
             Data_Longitude_ap = rest / 100;
         #else
             decimal = temp / 100000;
             rest = temp - decimal * 100000;
             Data_Longitude_bp = decimal;
             Data_Longitude_ap = rest / 100;
         #endif
         Data_E_W = GPS_coord[LON] < 0 ? 'W' : 'E';
         sendDataHead(ID_Longitude_bp);
         write_FrSky16(Data_Longitude_bp);
         sendDataHead(ID_Longitude_ap);
         write_FrSky16(Data_Longitude_ap);
         sendDataHead(ID_E_W);
         write_FrSky16(Data_E_W);
         }
  #endif
}

void inline send_GPS_latitude(void)
{
  #if GPS
      uint16_t Data_Latitude_bp;
      uint16_t Data_Latitude_ap;
      uint16_t Data_N_S;
      uint32_t temp, rest, decimal;
      if (f.GPS_FIX && GPS_numSat >= 4) {           
         temp = abs(GPS_coord[LAT]);
         #if defined(COORDFORMAT_DECIMALMINUTES)
             decimal = temp / 10000000;
             temp -= decimal * 10000000;
             temp *= 6;
             rest = temp;
             temp /= 1000000;
             rest -= temp * 1000000;
             Data_Latitude_bp = decimal * 100 + temp;
             Data_Latitude_ap = rest / 100;
         #else
             decimal = temp / 100000;
             rest = temp - decimal * 100000;
             Data_Latitude_bp = decimal * 100 + temp;
             Data_Latitude_ap = rest / 100;
         #endif
         Data_N_S = GPS_coord[LAT] < 0 ? 'S' : 'N';
         sendDataHead(ID_Latitude_bp);
         write_FrSky16(Data_Latitude_bp);
         sendDataHead(ID_Latitude_ap);
         write_FrSky16(Data_Latitude_ap);
         sendDataHead(ID_N_S);
         write_FrSky16(Data_N_S);     
         }
  #endif
}

// Time of arming
void inline send_Time(void)
{
  uint16_t seconds_since_start;
  uint16_t Data_rest;
  uint16_t Data_hours;
  uint16_t Data_minutes;
  uint16_t Data_seconds;
  if (f.ARMED) {
    seconds_since_start = armedTime / 1000000;
    Data_hours   = seconds_since_start / 3600;
    Data_rest = seconds_since_start - Data_hours * 3600;
    Data_minutes = Data_rest / 60;
    Data_seconds = Data_rest -  Data_minutes * 60;
    sendDataHead(ID_Hour_Minute);
    write_FrSky16(Data_hours + Data_minutes * 256);
    sendDataHead(ID_Second);
    write_FrSky16(Data_seconds);
    }
}

// ACC
void inline send_Accel(void)
{
  #if ACC
      int16_t Data_Acc_X;
      int16_t Data_Acc_Y;
      int16_t Data_Acc_Z;
      Data_Acc_X = ((float)imu.accSmooth[0] / ACC_1G) * 1000;
      Data_Acc_Y = ((float)imu.accSmooth[1] / ACC_1G) * 1000;
      Data_Acc_Z = ((float)imu.accSmooth[2] / ACC_1G) * 1000;
      sendDataHead(ID_Acc_X);
      write_FrSky16(Data_Acc_X);
      sendDataHead(ID_Acc_Y);
      write_FrSky16(Data_Acc_Y);
      sendDataHead(ID_Acc_Z);
      write_FrSky16(Data_Acc_Z);     
  #endif
}

// Voltage (Ampere Sensor) 
void inline send_Voltage_ampere(void) // Data compatibel to FrSky FAS-100 voltage sensor, FLD-02 uses only analog data A1 and A2
{
  #if defined (VBAT) and not defined(FRSKY_FLD02)
      uint16_t voltage;
      uint16_t Data_Voltage_vBat_bp;
      uint16_t Data_Voltage_vBat_ap;   
      voltage = ((analog.vbat * 110) / 21);
      Data_Voltage_vBat_bp = voltage / 100;
      sendDataHead(ID_Voltage_Amp_bp);
      write_FrSky16(Data_Voltage_vBat_bp);
      Data_Voltage_vBat_ap = ((voltage % 100) + 5) / 10; 
      sendDataHead(ID_Voltage_Amp_ap);
      write_FrSky16(Data_Voltage_vBat_ap); 
  #endif  

  #if defined(POWERMETER)
      uint16_t Data_Voltage_I_Motor;
      Data_Voltage_I_Motor = analog.amperage;
      sendDataHead(ID_Current);
      write_FrSky16(Data_Voltage_I_Motor);   
  #endif
}

// Main function FrSky telemetry
void run_telemetry(void)
{
  static uint32_t lastTime;
  static uint8_t tele_loop;
  if ((millis() - lastTime) > 125) {
     // Data sent every 125ms due to scheduler
     lastTime = millis();
     tele_loop++;
     switch (tele_loop) {
			case 1:
                        send_Voltage_ampere();
                        send_Accel();         
                        break;
                        case 2:
			send_Fuel();         
			send_GPS_longitude();
			break;
			case 3:
			send_Temperature();  
                        send_Accel();        
                        break;
			case 4:
			send_Altitude();     
			send_GPS_speed();    
                        send_Course();       
			break;
			case 5:
                        send_Voltage_ampere();
                        send_Accel();         
                        break;
			case 6:
			send_RPM();       
			send_GPS_latitude();
			break;
			case 7:
                        send_GPS_speed();
                        send_Accel();    
			send_cell_volt();
                        break;
			default:
                        send_Altitude(); 
			send_Time();     
			tele_loop = 0;
			break;
		        }
	sendDataTail();
	}
}
#endif // FRSKY telemetry

#if defined(SPORT_TELEMETRY) // TO BE DONE

void init_telemetry(void)
{
}

void run_telemetry(void)
{
}
#endif // S.PORT telemetry

