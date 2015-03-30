#ifndef TELEMETRY_H_
#define TELEMETRY_H_ 
// ****************************************************************
// FrSky telemetry
// Version: 0.4.0
// Changes: V0.4.0: - supports 2.4
// Version: 0.3.0
// Cahnges: V0.3.0: - new structure with cpp/h-files
// Date 20/09/2012
// Changes: V0.2.1: - make it work with 2.1 (shared dev)
// Date: 14/08/2012
// Changes: V0.2: - Byte stuffing added
//                - vBat will be send, if "#define FAS_100" is comment out
//                V0.1: - First release
// ****************************************************************

#if defined(FRSKY_TELEMETRY)
    // Definitions
    #define TELEMETRY_BAUD     9600  

    // Frame protocol
    #define Protocol_Header    0x5E
    #define Protocol_Tail      0x5E

    // Data Ids  (bp = before point; af = after point)
    // Official data IDs
    #define ID_GPS_Altitude_bp    0x01
    #define ID_GPS_Altitude_ap    0x09
    #define ID_Temperature1       0x02
    #define ID_RPM                0x03
    #define ID_Fuel_level         0x04
    #define ID_Temperature2       0x05
    #define ID_Volt               0x06
    #define ID_Altitude           0x10
    #define ID_GPS_speed_bp       0x11
    #define ID_GPS_speed_ap       0x19
    #define ID_Longitude_bp       0x12
    #define ID_Longitude_ap       0x1A
    #define ID_E_W                0x22
    #define ID_Latitude_bp        0x13
    #define ID_Latitude_ap        0x1B
    #define ID_N_S                0x23
    #define ID_Course_bp          0x14
    #define ID_Course_ap          0x1C
    #define ID_Date_Month         0x15
    #define ID_Year               0x16
    #define ID_Hour_Minute        0x17
    #define ID_Second             0x18
    #define ID_Acc_X              0x24
    #define ID_Acc_Y              0x25
    #define ID_Acc_Z              0x26
    #define ID_Voltage_Amp_bp     0x3A
    #define ID_Voltage_Amp_ap     0x3B
    #define ID_Current            0x28
#endif // FRSKY_TELEMETRY

#if defined(SPORT_TELEMETRY) // TO BE DONE
    #define TELEMETRY_BAUD   57600  
#endif // S.PORT telemetry

// Variables
extern uint32_t armedTime;

// Functions
void init_telemetry(void);
void run_telemetry(void);
#endif /* TELEMETRY_H_ */

