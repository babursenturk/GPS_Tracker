/* ========================================
 *
 * Copyright BSENTURK, 2016-Q2
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF BSENTURK.
 *
 * ========================================
*/

#define MSEC_1_TICK		    24000
#define MPU9150_ADDR        0x68

#define WRITE 			    0
#define READ			    1
#define RXTIMEOUT           10

//#define CUSTOMER_LINK
#define DEBUG_LINK

#define MODEM_READY         0x3F

//Modem response strings
#define READY               0
#define OKAY                1
#define CPIN                2
#define CONNCLOSE           3
#define CALLREADY           5
#define NPDOWN              6
#define PROMPT              8
#define NO_LOCK             9
#define NOT_FIXED           10
#define FIXED_3D            11
#define FIXED_2D            12
#define INFORESP            13
#define LINEFEED            14
#define CLLREGS             15
#define GPRSREGS            16
#define CONNRESP            17
#define ALRDYCON            18

const float CONST_16G = 2048;
const float CONST_2000 = 16.4;
const float CONST_G = 9.81;
const float RADIANS_TO_DEGREES = 180 / 3.14159;
const float ALPHA = 0.96;
const float KMPH = 3.6;

enum {
    MODEMIDLE = 0,
    GPSSTATUS,
    GPSSTATUSRESP,
    GPSINFO,
    GPSINFORESP,
    CIPSENDREQ,
    PASSINFO,
    SMS_TASK1,
    SMS_TASK2,
};

enum {
    ATTACHREQ = 0,
    ATTACHRESP,
    FGCNTREQ,
    FGCNTRESP,
    APNSEND,
    APNRESP,
    QISTREQ,
    QISTRESP,
    MUXSELREQ,
    MUXSELRESP,
    BRINGUPSEND,
    BRINGUPRESP,
    CONREQ,
    CONRESP,
};


#define		 ZERO(o)		memset((char *)&(o), 0x0, sizeof( (o)))

//JSON objects
static const char jsonobj[11][15] = {
 "\x22mydevice\x22\x00",
 "\x22mydevid\x22\x00",    
 "\x22utctime\x22\x00",    
 "\x22latt\x22\x00", 
 "\x22lon\x22\x00",     
 "\x22satquality\x22\x00",
 "\x22satellites\x22\x00",
 "\x22myaltitude\x22\x00",
 "\x22speedkmh\x22\x00",
 "\x22myevent\x22\x00",
 "\x22mytemp\x22\x00",
};

static const char URCS[19][18] = {
"RDY\x00",
"OK\x00",				    
"READY\x00",
"CLOSED\x00",				// a connection was closed.
"PDP DEACT\x00",			// GPRS Context Deactivated.
"Ready\x00",			    // Modem Ready  Call Ready
"NORMAL POWER DOWN\x00",	// Modem Shutdown
"+CGATT:\x00",				// GPRS Connectivity.
">",                        // SMS/GPRS text cmd prompt
"Location Unknown",         // GPS is not run
"Location Not Fix",         // GPS is run but haven't fix
"Location 3D Fix",          // GPS 3D fixed. 
"Location 2D Fix",          // GPS 2D fixed.
"\x0d\x0a\x32\x2c",         // Info Header.
"\x0d\x0a",                 // LineFeed response.
"+CREG: 0,1",               // Registration response.
"+CGATT: 1",                // Attached response.    
"CONNECT OK",               // Server connection established. we are online now.
"ALREADY CONNECT",          // We are already connected. Its necessary if MCU reset indep.
};

static const char formhdr[6] = {"$GPGGA"};


static const char atCOMMANDS[29][22] = {	//the limit must 1 item greater than the longest question. so we will check \0 :)
/* 0*/"AT\x0d\x0a\x00",
/* 1*/"AT+IPR=115200\x3b\x26\x57\x0d\x0a\x00",
/* 2*/"ATE0\x0d\x0a\x00",
/* 3*/"AT+CPIN?\x0d\x0a\x00",
/* 4*/"AT+QIFGCNT=\x30\x0d\x0a\x00",
/* 5*/"AT+GSN\x0d\x0a\x00",
/* 6*/"AT+CSQ\x0d\x0a\x00",
/* 7*/"AT+CREG?\x0d\x0a\x00",
/* 8*/"AT+CMGF=\x31\x0d\x0a\x00",
/* 9*/"AT+CSCS=\x22\x47\x53\x4d\x22\x0d\x0a\x00",
/*10*/"AT+CSCA?\x0d\x0a\x00",
/*11*/"AT+QISTAT\x0d\x0a\x00",				 
/*12*/"AT+CIPRXGET=\x31\x0d\x0a\x00",
/*13*/"AT+CPOWD=\x31\x0d\x0a\x00",
/*14*/"AT+CGATT=\x31\x0d\x0a\x00",			
/*15*/"AT+QICSGP=\x31\x2c\x22\x00",                 //APN name
/*16*/"AT+QIDNSIP=\x30\x0d\x0a\x00",
/*17*/"AT+CIFSR\x0d\x0a\x00",
/*18*/"AT+CGATT?\x0d\x0a\x00",
/*19*/"AT+CMGS=\x22\x00",
/*20*/"AT+CIPRXGET=",
/*21*/"AT+QIMUX=\x30\x0d\x0a\x00",
/*22*/"AT+CGPSRST=\x31\x0d\x0a\x00",  
/*23*/"AT+CGPSINF=\x32\x0d\x0a\x00",  //GPGGA = 2 = $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,8,1.01,499.6,M,48.0,M,,0*5B
/*24*/"AT+CGPSSTATUS?\x0d\x0a\x00",
/*25*/"AT+CGPSRST=\x32\x0d\x0a\x00",    
/*26*/"AT+CFUN=\x31\x0d\x0a\x00",
/*27*/"AT+QISEND\x0d\x0a\x00",
/*28*/"AT+QIOPEN=\x22TCP\x22\x2c\x00",
};

// GGA-Global Positioning System Fixed Data, message 103,00
#define LOG_GGA 0
#define GGA_ON   "$PSRF103,00,00,01,01*25\r\n"
#define GGA_OFF  "$PSRF103,00,00,00,01*24\r\n"

// GLL-Geographic Position-Latitude/Longitude, message 103,01
#define LOG_GLL 0
#define GLL_ON   "$PSRF103,01,00,01,01*26\r\n"
#define GLL_OFF  "$PSRF103,01,00,00,01*27\r\n"

// GSA-GNSS DOP and Active Satellites, message 103,02
#define LOG_GSA 0
#define GSA_ON   "$PSRF103,02,00,01,01*27\r\n"
#define GSA_OFF  "$PSRF103,02,00,00,01*26\r\n"

// GSV-GNSS Satellites in View, message 103,03
#define LOG_GSV 0
#define GSV_ON   "$PSRF103,03,00,01,01*26\r\n"
#define GSV_OFF  "$PSRF103,03,00,00,01*27\r\n"

// RMC-Recommended Minimum Specific GNSS Data, message 103,04
#define LOG_RMC 1
#define RMC_ON   "$PSRF103,04,00,01,01*21\r\n"
#define RMC_OFF  "$PSRF103,04,00,00,01*20\r\n"

// VTG-Course Over Ground and Ground Speed, message 103,05
#define LOG_VTG 0
#define VTG_ON   "$PSRF103,05,00,01,01*20\r\n"
#define VTG_OFF  "$PSRF103,05,00,00,01*21\r\n"

// Switch Development Data Messages On/Off, message 105
#define LOG_DDM 1
#define DDM_ON   "$PSRF105,01*3E\r\n"
#define DDM_OFF  "$PSRF105,00*3F\r\n"

#define USE_WAAS   0     // useful in US, but slower fix
#define WAAS_ON    "$PSRF151,01*3F\r\n"       // the command for turning on WAAS
#define WAAS_OFF   "$PSRF151,00*3E\r\n"       // the command for turning off WAAS


extern void send_sms(/*char *smstxt*/);
extern void prepare_IMEI(char *buf);
extern void prepare_GPGGA(char *buf);
extern void sms_task(void);

/* [] END OF FILE */
