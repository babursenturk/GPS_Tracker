/* ========================================
 *
 * Copyright BSENTURK, 2016Q3
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF BSENTURK.
 *
 * ========================================
*/
#include <project.h>
#include <main.h>
#include <stdio.h>
#include <stdlib.h>	
#include <string.h>
#include <mpu6050.h>
#include <math.h>

static char         GSMRXBUF[200] = {0};
static char         GPSRXBUF[400] = {0};

static uint8_t      I2C1RXBUF[32] = {0};
static uint8_t      I2C1TXBUF[32] = {0};

uint8_t         rxcnt = 0;  //GSM modül gelen buffer length
uint16_t        rxscnt = 0; //GPS modül gelen buffer length
uint8_t         modemready = 0;
uint8_t         modeminited = 0;
uint8_t         gpsinited = 0;
uint8_t         f1000 = 0;
uint8_t         f300 = 0;
uint8_t         online = 0;
uint8           send_event_sms = 0;
char            smssent = 0;

unsigned long   systick = 0;

int16      ax, ay, az, temp;    //Accelerometer eksenleri ham datalar ve sıcaklık. 
int16      gx, gy, gz;          //Gyro eksenleri ham datalar
unsigned long last_read_time;
int16_t gyro_angle_x_l, gyro_angle_y_l;
int16_t angle_x_l, angle_y_l;
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int16_t temperature;


typedef struct {
    char IMEI[16];               //Modem imei nr
    char NMEA[256];              //NMEA data in GPGGA format
    char SPEED[10];              //Speed km/h
    char GX[10];
    char GY[10];
    char EVENT;                  //event code
    char TEMP[5];
} device;

char    jsonstring[300];         //Sunucuya gonderilecek JSON paket.


static device   mydevice;
CY_ISR_PROTO(USER_ISR);


/*****************************
* Systick ISR
* Used for rx buffer read organisation
* and make 1Hz tick
*****************************/
CY_ISR(USER_ISR)        //Manual SysTick ISR 1mSec
{
    uint32 ch,chs;
    static uint16   cnt = 0;
    static uint16   cnt3 = 0;
    char	*ptrb;
    
    systick++;
    
    if (++cnt == 1000)
    {
        cnt = 0;
        f1000 = 1;
    }
    
    if (++cnt3 == 300)
    {
        cnt3 = 0;
        f300 = 1;
    }
    
    ch  = UART_GSM_UartGetChar();
    chs = UART_GPS_UartGetChar();
    
    if (0u != ch)       //Linear Buffer
    {
        GSMRXBUF[rxcnt] = (char)ch;
        rxcnt++;
    }
    else
    {
        //if (rxcnt)
          //  while(1);
    }
    
    if (0u != chs)      //Circular Buffer
    {
        
        GPSRXBUF[rxscnt] = (char)chs;

        if (rxscnt < 399)   
        {
            rxscnt++;
            if (rxscnt == 399) 
                rxscnt  = 0;    //For security reason
        }
    } 
    else 
    {
        if (rxscnt)
        {
            rxscnt = 0;
            
            ptrb = strstr(GPSRXBUF, formhdr);	//GPGGA okey ise
            if (ptrb != NULL)
                prepare_GPGGA(&GPSRXBUF[7]);    //UTC'den itibaren oku kaydet
        }
        
    }
}

unsigned long millis(void)
{
    return systick;
}
/*****************************
* Led surer. Heartbeat yapildi.
* From: any
*****************************/
void indicator(void)
{
    static uint8_t toggle = 0;
    
    if (toggle)
    {
        toggle = 0;
        Led1_Write(0);    
    }
    else
    {
        toggle = 1;
        Led1_Write(1);
    }
    
}

/*****************************
* Wait for modem response from uart
* TODO: timeout geridönüs yazılabilir
* ama handle edilmeli.
* From: any
*****************************/
void waitresponse(void)
{
    uint8_t tout = 0;
    
    //600ms kadar bekler. yan 1snlik interrupttan cagrildiginda
    //buna dikkat edilmeli.
    while ((rxcnt == 0) && (tout++ < RXTIMEOUT))    CyDelay(40);
}

/*****************************
* check and parse modem response
* 
* From: any
*****************************/
uint8_t checkresponse(uint8_t fstr)
{
    uint8_t rsp = 0;
    char	*ptrb;
    
    ZERO(ptrb); 
    
    ptrb = strstr(GSMRXBUF, URCS[fstr]);	
    if (ptrb != NULL) rsp = fstr;

    return rsp;
}

/*****************************
* Send command
* 
* From: any
*****************************/
void sendcmd(const char *commandbase)
{
    rxcnt = 0; ZERO(GSMRXBUF);
    
    UART_GSM_UartPutString(commandbase);    
}

/*****************************
* Setup system interrupts.
* 
* From: main
*****************************/
void setup_sys(void)
{
    CyIntSetSysVector((SysTick_IRQn + 16), USER_ISR);    // Systick Related
    SysTick_Config(MSEC_1_TICK);
    
    CyGlobalIntEnable;
    
    ZERO(mydevice.IMEI); 
    mydevice.SPEED[0] = 0x30;
    mydevice.SPEED[1] = 0x30;
    mydevice.SPEED[2] = 0x30;
    mydevice.EVENT    = 0x30;
}

/*****************************
* Modem power cycle.
* Each calling inverts modem power
* From: check_modem
*****************************/
void pwrmodem(void)
{
    rxcnt = 0; ZERO(GSMRXBUF);
    
    PWRKEY_Write(1);
    CyDelay(1200);
    PWRKEY_Write(0);   
    CyDelay(3000);

}
/*****************************
* Check modem if present or not.
* Power it on if necessary
* From: main
* Takes: Max 5.6Sec
*****************************/
void check_modem(void)
{    

    rxcnt = 0; ZERO(GSMRXBUF);

    sof:
    
    sendcmd(atCOMMANDS[0]);
    waitresponse();
    
    if (checkresponse(OKAY) == OKAY)
    {
        rxcnt = 0; ZERO(GSMRXBUF);
        modemready = 1;
        return;
    }
    else
    {
        pwrmodem();
        sendcmd(atCOMMANDS[0]);
        waitresponse();
        
        if (checkresponse(OKAY) == OKAY)
        {
            rxcnt = 0; ZERO(GSMRXBUF);
            modemready = 1;
            return;
        }
        else goto sof;
    }
    
    
    rxcnt = 0; ZERO(GSMRXBUF);
}

/*****************************
* Modem init
* 
* From: any
*****************************/
void init_modem(void)
{
    
    sendcmd(atCOMMANDS[2]);     // echo off. Modeme gonderdigim sorular, cevapla birlikte gelmesin diye...
    CyDelay(300);
    waitresponse();
    if (checkresponse(OKAY) == OKAY) modeminited |= 0x01;   
    
    sendcmd(atCOMMANDS[3]);     // sim card query
    CyDelay(300);
    waitresponse();
    if (checkresponse(CPIN) == CPIN) modeminited |= 0x02; 

    
    sendcmd(atCOMMANDS[8]);     // te mode
    CyDelay(300);
    waitresponse();
    if (checkresponse(OKAY) == OKAY) modeminited |= 0x04; 
    
    sendcmd(atCOMMANDS[9]);     // sms mode
    CyDelay(300);
    waitresponse();
    if (checkresponse(OKAY) == OKAY) modeminited |= 0x08; 
    
    
    sendcmd(atCOMMANDS[26]);    // phone functionality
    CyDelay(300);
    waitresponse();
    if (checkresponse(OKAY) == OKAY) modeminited |= 0x10;  
    
    sendcmd(atCOMMANDS[7]);    // Network registration.
    CyDelay(350);
    waitresponse();
    if (checkresponse(CLLREGS) == CLLREGS) modeminited |= 0x20;      
   
    sendcmd(atCOMMANDS[5]);    // get imei
    CyDelay(300);
    waitresponse();
    if (checkresponse(LINEFEED) == LINEFEED) prepare_IMEI(&GSMRXBUF[0]);
    
    
    ZERO(GSMRXBUF);
}

/*****************************
* prepare_GPGGA
* From: anywhere    
*****************************/
void prepare_GPGGA(char *buf)
{
    char n = 0;
    uint8_t tout = 0;
    
    memcpy(&mydevice.NMEA[0], buf, sizeof(mydevice.NMEA));
    
    while ((buf[n] != 0x0d) && (tout++ < 254)) {n++;}
    mydevice.NMEA[n] = 0x00;
    
}

/*****************************
* prepare_IMEI
* From: anywhere    
*****************************/
void prepare_IMEI(char *buf)
{ 
    char n = 0;
    char tout = 0;
    
    while ((buf[n] <= 0x2f) && (tout++ < sizeof(mydevice.IMEI))) {n++;} //Sayısal bilgiyi ara.
    
    buf += n;
    memcpy(&mydevice.IMEI[0], buf, sizeof(mydevice.IMEI));
    mydevice.NMEA[15] = 0x00;
}

/*****************************
* getjsonobject
* From: prepareJSON
*****************************/
void getjsonobject(uint8_t objidx, char *buf, uint8_t *writelen)
{
    uint8_t n = 0;
    
    while (jsonobj[objidx][n] != 0x00)
    {
        *buf = jsonobj[objidx][n++];
        buf++;
    }
    *writelen += n;
}
    
/*****************************
* prepareJSON
* From: packet send    
*****************************/
void prepareJSON(void)
{   //123456;Dron 1;12;40.97441944;28.73333333;
    uint8   resplen = 0;
    uint8   den = 0;
    uint8   whtout = 0;
//    char    nmeapack[78] = {"151853,4058.471909,N,2843.367283,E,1,4,2.056997,41.890938,M,39.479111,M,,0000"};
    
    for (whtout = 0; whtout < 60; whtout++)
    {
        if ((mydevice.NMEA[whtout] == 0x2c) && (mydevice.NMEA[whtout+1] == 0x2c) && (mydevice.NMEA[whtout+2] == 0x2c))
        {
            ZERO(jsonstring);  //Not fixed yet.
            sprintf(&jsonstring[0], "Unknown Location or Not fixed yet");
            jsonstring[33] = 0x00;
            return;            
        }
    }
   
    //sprintf(&jsonstring[0], "123456;Dron 1;12;40.97441944;28.73333333;");
    sprintf(&jsonstring[0], "123456;Dron 1;12;");
    resplen +=17;

    //LATITUDE   
    den = 0;
    while ((mydevice.NMEA[den] != 0x2c) && (++whtout < 128)) den++;
    whtout = 0;
    den++;
    
    while ((mydevice.NMEA[den] != 0x4e) && (mydevice.NMEA[den] != 0x53) && (++whtout < 128))
    {
        jsonstring[resplen] = mydevice.NMEA[den];
        resplen++;
        den++;   
    }    
    
    jsonstring[resplen] = mydevice.NMEA[den];
    resplen++;    
    
    whtout = 0;    
    
    jsonstring[resplen++] = 0x3b;
    
    //LONGITUDE
    den++; den++;
    while ((mydevice.NMEA[den] != 0x45) && (mydevice.NMEA[den] != 0x57) && (++whtout < 128))
    {
        jsonstring[resplen] = mydevice.NMEA[den];
        resplen++;
        den++;   
    }

    jsonstring[resplen] = mydevice.NMEA[den];
    resplen++;  
    jsonstring[resplen] = 0x3b;
    resplen++;  
    jsonstring[resplen] = 0x00;    
    
//    getjsonobject(0, &jsonstring[resplen], &resplen);   //mydevice
//    
//    sprintf(&jsonstring[resplen], "\x3a\x20\x5b\x7b\x0d\x0a");
//    resplen +=6;
//    
//    getjsonobject(1, &jsonstring[resplen], &resplen);   //mydevid
//    
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3;
//    
//    sprintf(&jsonstring[resplen], "%s", mydevice.IMEI);
//    resplen +=15;
//    
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4;
//    
//    getjsonobject(2, &jsonstring[resplen], &resplen);   //utctime
//
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3;
//    
//    memcpy(&jsonstring[resplen], &mydevice.NMEA[0], 6);
//    resplen +=6;
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4;
//    
//    getjsonobject(3, &jsonstring[resplen], &resplen);   //latitude
//    
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3;
//    
//    
//    den = 0;
//    
//    while ((mydevice.NMEA[den] != 0x2c) && (++whtout < 128)) den++;
//    whtout = 0;
//    den++;
//    
//    while ((mydevice.NMEA[den] != 0x4e) && (mydevice.NMEA[den] != 0x53) && (++whtout < 128))
//    {
//        jsonstring[resplen] = mydevice.NMEA[den];
//        resplen++;
//        den++;   
//    }    
//    
//    jsonstring[resplen] = mydevice.NMEA[den];
//    resplen++;    
//    
//    whtout = 0;
//    
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4;
//    
//    getjsonobject(4, &jsonstring[resplen], &resplen);   //longitude
//    
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3;    
//
//    den++; den++;
//    while ((mydevice.NMEA[den] != 0x45) && (mydevice.NMEA[den] != 0x57) && (++whtout < 128))
//    {
//        jsonstring[resplen] = mydevice.NMEA[den];
//        resplen++;
//        den++;   
//    }
//
//    jsonstring[resplen] = mydevice.NMEA[den];
//    resplen++;  
//    
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4;
//    
//    getjsonobject(5, &jsonstring[resplen], &resplen);   // fix guality
//    
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3; 
//    
//    den++; den++;                                       
//    jsonstring[resplen] = mydevice.NMEA[den];           // 0 not fixed, 1 gps fixed, 2 dgps fixed
//    
//    if (mydevice.NMEA[den] == 0x30)
//    {
//        ZERO(jsonstring);  //Not fixed yet.
//        sprintf(&jsonstring[0], "Not fixed\x0d\x0a");
//        jsonstring[12] = 0x00;
//        return;
//    }    
//    resplen++;  
//    
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4;    
//    
//    getjsonobject(6, &jsonstring[resplen], &resplen);   // satellites are in view
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3;     
//    
//    den++; den++;
//    jsonstring[resplen] = mydevice.NMEA[den];
//    den++; resplen++;
//    jsonstring[resplen] = mydevice.NMEA[den];
//    
//    den+=6; resplen++;
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4;    
//    
//    
//    getjsonobject(7, &jsonstring[resplen], &resplen);   // altitude
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3; 
//    
//    whtout = 0;
//    while ((mydevice.NMEA[den] != 0x2c) && (++whtout < 32))
//    {
//        jsonstring[resplen] = mydevice.NMEA[den];
//        resplen++;
//        den++;         
//    }
//    
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4;  
//    
//    getjsonobject(8, &jsonstring[resplen], &resplen);   // speed km/h
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3;
//
//    jsonstring[resplen++] = mydevice.SPEED[0];
//    jsonstring[resplen++] = mydevice.SPEED[1];
//    jsonstring[resplen++] = mydevice.SPEED[2];
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4; 
//    
//    getjsonobject(9, &jsonstring[resplen], &resplen);   // event
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3;    
//    jsonstring[resplen++] = mydevice.EVENT;
//    sprintf(&jsonstring[resplen], "\x22\x2c\x0d\x0a");
//    resplen +=4;
//    
//    getjsonobject(10, &jsonstring[resplen], &resplen);   // temp
//    sprintf(&jsonstring[resplen], "\x3a\x20\x22");
//    resplen +=3; 
//    
//    jsonstring[resplen++] = mydevice.TEMP[0];
//    jsonstring[resplen++] = mydevice.TEMP[1];
//    jsonstring[resplen++] = mydevice.TEMP[2];
//    
//    
//    sprintf(&jsonstring[resplen], "\x22\x7d\x5d\x0d\x0a\x7d");
//    resplen +=6;    
    //jsonstring[42] = 0x00;
}

/*****************************
* GPS init
* Pre: modem init
* From: main
*****************************/
void init_gps(void)
{
    CyDelay(300);
    
    UART_GPS_UartPutString(GGA_ON);     //Bizim kullandigimiz
    UART_GPS_UartPutString(VTG_OFF);    //Bunu da acacagiz. GPS ile hiz tespiti icin.
    UART_GPS_UartPutString(GLL_OFF);
    UART_GPS_UartPutString(GSA_OFF);    
    UART_GPS_UartPutString(GSV_OFF);
    UART_GPS_UartPutString(RMC_OFF);  
    UART_GPS_UartPutString(DDM_OFF);
}

/*****************************
* Send an sms to specific nr.
* Pre: modem init
* From: anywhere
*****************************/
void send_sms(/*char *smstxt*/)
{
    char    clsr[4] = {"\x22\x0d\x0a\x00"};
#ifdef CUSTOMER_LINK
    char    number[14] = {"+905072802182\x00"};
#else
    char    number[14] = {"+905334537848\x00"};
#endif
    char    smstxte[19] = {"Accident\x00"};
    //sprintf(&jsonstring[resplen], "%s", mydevice.IMEI);
    sendcmd(atCOMMANDS[19]);     // send an sms
    sendcmd(number);
    sendcmd(clsr);
    
    CyDelay(300);
    waitresponse();
    
    sendcmd(smstxte);
    CyDelay(300);
    sendcmd("\x1a\x00");
    waitresponse(); //Bundan sonra 120Saniyeye kadar yanit gelmeyebilir.
    //Networke bagli.
}

/*****************************
* Manual I2C Read
* 
*****************************/
void i2c_read(uint32_t ramreg, uint32_t datalen)
{
    uint32_t i = 0;
    
    I2C_1_I2CMasterClearStatus(); I2C_1_I2CMasterClearReadBuf(); I2C_1_I2CMasterClearWriteBuf();
    
    
    I2C_1_Start();
    
    I2C1TXBUF[0] = (uint8_t)ramreg;
    I2C_1_I2CMasterSendStart(MPU9150_ADDR, WRITE);
    I2C_1_I2CMasterWriteByte(I2C1TXBUF[0]);
    
    I2C_1_I2CMasterSendStop();
    I2C_1_I2CMasterClearStatus(); I2C_1_I2CMasterClearReadBuf(); I2C_1_I2CMasterClearWriteBuf();
    
    
    I2C_1_Start();
 	I2C_1_I2CMasterSendStart(MPU9150_ADDR, READ);
    for (i=0; i<datalen; i++)
    {
        I2C1RXBUF[i] = I2C_1_I2CMasterReadByte(I2C_1_I2C_ACK_DATA);
    }
    I2C_1_I2CMasterSendStop();
}

/*****************************
* I2C Write
* 
*****************************/
void i2c_write(uint8_t ramreg, uint8_t regval, uint32 length)
{
    I2C_1_I2CMasterClearStatus(); I2C_1_I2CMasterClearReadBuf(); I2C_1_I2CMasterClearWriteBuf();
    I2C_1_Start();
    I2C1TXBUF[0] = ramreg;
    I2C1TXBUF[1] = regval; //2den fazlaysa direk I2C1TXBUF[2] den itibaren doldurmak yeterli   
    
    I2C_1_I2CMasterWriteBuf(MPU9150_ADDR, &I2C1TXBUF[0], length, I2C_1_I2C_MODE_COMPLETE_XFER);
    while((I2C_1_I2CMasterStatus() & I2C_1_I2C_MSTAT_WR_CMPLT) == 0);

    I2C_1_I2CMasterClearStatus();
    I2C_1_Stop();
}

/*****************************
* MPU6050 init
* 
* From: main
* Pre: init i2c
*****************************/
void init_mpu(void)
{
    uint8 dummy = 0; 
    dummy = 0b00000001;       //Exit from sleep and set clock for gyro x
    i2c_write(MPU6050_RA_PWR_MGMT_1, dummy, 2);
    CyDelay(300);
    
//    i2c_read(MPU6050_RA_WHO_AM_I, 1);   //0x68 gelmeli.
//    dummy = I2C1RXBUF[1];
    
    i2c_read(MPU6050_RA_GYRO_CONFIG, 1);
    dummy = I2C1RXBUF[0];
    i2c_write(MPU6050_RA_GYRO_CONFIG, dummy & 0b11100000, 2);

    i2c_read(MPU6050_RA_ACCEL_CONFIG, 1);
    dummy = I2C1RXBUF[0];
    i2c_write(MPU6050_RA_ACCEL_CONFIG, dummy & 0b11100000, 2);
     
    dummy = 0b10110000;
    i2c_write(MPU6050_RA_INT_PIN_CFG, dummy, 2);  
    
    dummy = 0x01;
    i2c_write(MPU6050_RA_INT_ENABLE, dummy, 2);    
    
}

/*****************************
* MPU6050 read 
* 
* From: any
* Pre: init mpu
*****************************/
void read_mpu(void)
{
    uint8 dummy = 0;
    static double xang, yang, zang = 0;
    
    //if (Pin_1_Read()) return;
    
    dummy = 0b00000001;       //Exit from sleep and set clock for gyro x
    i2c_write(MPU6050_RA_PWR_MGMT_1, dummy, 2);
    //CyDelay(10);
    
    i2c_read(MPU6050_RA_ACCEL_XOUT_H, 14);
    
    temp = (((I2C1RXBUF[6] << 8) | I2C1RXBUF[7]) - 1) ^ 0xFFFF;
    temp = temp / 340 + 36;
    
    sprintf(mydevice.TEMP, "%3d", (uint16_t)temp);
    
    ax = (((I2C1RXBUF[0] << 8) | I2C1RXBUF[1]) - 1) ^ 0xFFFF;
    ay = (((I2C1RXBUF[2] << 8) | I2C1RXBUF[3]) - 1) ^ 0xFFFF;
    az = (((I2C1RXBUF[4] << 8) | I2C1RXBUF[5]) - 1) ^ 0xFFFF;
    
    gx = (((I2C1RXBUF[8] << 8) | I2C1RXBUF[9]) - 1) ^ 0xFFFF;;
    gy = (((I2C1RXBUF[10] << 8) | I2C1RXBUF[11]) - 1) ^ 0xFFFF;;
    gz = (((I2C1RXBUF[12] << 8) | I2C1RXBUF[13]) - 1) ^ 0xFFFF;;       
  
}

/*****************************
* I2C module init
* 
* From: main
*****************************/
void init_i2c(void)
{
    I2C_1_I2CMasterClearStatus();
    I2C_1_EnableInt();
    I2C_1_Start();    
}

/*****************************
* UART gsm module init
* 
* From: main
*****************************/
void init_uart_gsm(void)
{
    UART_GSM_Start();
    rxcnt = 0;          //clear rx buffer pointer
}

/*****************************
* UART gsm module init
* 
* From: main
*****************************/
void init_uart_gps(void)
{
    UART_GPS_Start();
    rxscnt = 0;          //clear rx buffer pointer
    
}

/*****************************
* main_task
* 
* From: main
*****************************/
void main_task(void)
{
    static uint8 modemstate = CIPSENDREQ;
    
    if (online == 0) return;
    
    switch (modemstate)
    {  
        case CIPSENDREQ:
            sendcmd(atCOMMANDS[27]);            //CIPSEND
            modemstate++;
        break;               
        case PASSINFO:
            waitresponse();
            
            if (checkresponse(PROMPT) == PROMPT)
            {
                prepareJSON();
                sendcmd(jsonstring);
                sendcmd("\x1a\x00");
                
                //sms_task();         //SMS'in yeri burasi. Karisikliklari onlemek icin.
            }
            
            modemstate = CIPSENDREQ;     //Turn back
        break;        
    }
}

/*****************************
* gprs_task
* 
* From: main
*****************************/
static uint8 gprsstate = ATTACHREQ;

void gprs_task(void)
{
    static uint8_t contout = 0;
    
    char    apn[12] = {"internet\x22\x0d\x0a\x00"};      //turk telekom, vodafone ve turkcell icin gecerli.
    
    if (online)                 return;     //cevrimciysek cik
    if (modeminited != 0x3F)    return;     //modem yoksa cik
    
    switch (gprsstate)
    {
        case ATTACHREQ:
            sendcmd(atCOMMANDS[18]);         //attach gps network  14 attach 18 query
            gprsstate++;
        break;    
        case ATTACHRESP:
            if (checkresponse(GPRSREGS) == GPRSREGS) gprsstate++; else gprsstate--;
        break;
        case FGCNTREQ:
            sendcmd(atCOMMANDS[4]);         //AT+QIFGCNT
            gprsstate++;
        break;   
        case FGCNTRESP:
            if (checkresponse(OKAY) == OKAY) gprsstate++; else gprsstate--;
        break;           
        case APNSEND:
            sendcmd(atCOMMANDS[15]);            //send apn name  \x22internet\x22
            sendcmd(apn);                       //APN haricinde username ve password belirtilmeyecek.
            
            gprsstate++;
        break;            
        case APNRESP:
            if (checkresponse(OKAY) == OKAY) gprsstate++; else gprsstate--;
        break; 
        case QISTREQ:
            sendcmd(atCOMMANDS[1]);         //AT+QISTAT
            gprsstate++;
        break;   
        case QISTRESP:
            if (checkresponse(OKAY) == OKAY) gprsstate++; else gprsstate--;
        break;              
            
        case MUXSELREQ:
            sendcmd(atCOMMANDS[21]);    //error geliyor.
            gprsstate++;
        break;   
        case MUXSELRESP:
            if (checkresponse(OKAY) == OKAY) gprsstate++; else gprsstate++; //ERRORse de zaten yurur.
        break;            
            
        case BRINGUPSEND:
            sendcmd(atCOMMANDS[16]);         //bring up
            gprsstate++;
        break;            
        case BRINGUPRESP:
            if (checkresponse(OKAY) == OKAY) gprsstate++; else gprsstate--;
        break;  
            
        case CONREQ:
            
            sendcmd(atCOMMANDS[28]);         //start conn
            sendcmd("\x22\x00");
            //sendcmd("209.141.40.229\x00");  //ilerde degisecek. Belki SMS ile konfigürasyon eklenebilir.
            //sendcmd("94.138.220.66\x00");
            sendcmd("185.48.181.156\x00");
            sendcmd("\x22\x00");            //Mesela: "EKLE 5339876543,209.141.40.229,8000"
            sendcmd("\x2c\x00");
            sendcmd("\x22\x00");          
            sendcmd("9000\x22\x0d\x0a\x00");
            contout = 0;
            gprsstate++;
        break;            
        case CONRESP:
            if ((checkresponse(OKAY) == OKAY) || (checkresponse(ALRDYCON) == ALRDYCON))
                online = 1;
            else
            {/*burada server conn failure icin sms gonderme olabilir mesela*/
                if (++contout == 10) {contout = 0; gprsstate = CONREQ;}
            }
        break;          
            
    }
    
    
}

void check_network(void)
{
    static uint8_t smlstt = 0;
    
    if (smlstt == 0)
    {
        sendcmd(atCOMMANDS[7]);    // Network registration.
        smlstt++;
    }
    else if (smlstt == 1)
    {
        if (checkresponse(CLLREGS) == CLLREGS) modeminited |= 0x20;
        smlstt = 0;
    }
    
         
}

unsigned long get_last_time() {
  return last_read_time;
}

void set_last_time(unsigned long _time) {
  last_read_time = _time;
}

float get_delta_time(unsigned long t_now) {
  return (t_now - get_last_time()) / 1000.0;
}

int16_t get_last_gyro_angle_x() {
  return gyro_angle_x_l;
}

void set_last_gyro_angle_x(int16_t _gyro_angle_x) {
  gyro_angle_x_l = _gyro_angle_x;
}

int16_t get_last_gyro_angle_y() {
  return gyro_angle_y_l;
}

void set_last_gyro_angle_y(int16_t _gyro_angle_y) {
  gyro_angle_y_l = _gyro_angle_y;
}

int16_t get_last_angle_x() {
  return angle_x_l;
}

void set_last_angle_x(int16_t _ang_x) {
  angle_x_l = _ang_x;
}

int16_t get_last_angle_y() {
  return angle_y_l;
}

void set_last_angle_y(int16_t _ang_y) {
  angle_y_l = _ang_y;
}

float get_accel_xy(float ax_p, float ay_p) {
  return sqrt(pow(ax_p, 2) + pow(ay_p, 2));
}

void calibrate_sensors() {
    int     num_readings = 100;
    float   x_accel = 0;
    float   y_accel = 0;
    float   z_accel = 0;
    float   x_gyro = 0;
    float   y_gyro = 0;
    float   z_gyro = 0;
    int     i = 0;

  // Discard the first set of values read from the IMU
    read_mpu();

  // Read and average the raw values from the IMU
    for (i = 0; i < num_readings; i++) 
    {
        read_mpu();
  
        x_accel += ax;
        y_accel += ay;
        z_accel += az;
        x_gyro += gx;
        y_gyro += gy;
        z_gyro += gz;
        
        CyDelay(10);
    }
    
      x_accel /= num_readings;
      y_accel /= num_readings;
      z_accel /= num_readings;
      x_gyro /= num_readings;
      y_gyro /= num_readings;
      z_gyro /= num_readings;

      // Store the raw calibration values globally
      ax_offset = x_accel;
      ay_offset = y_accel;
      az_offset = z_accel;
      gx_offset = x_gyro;
      gy_offset = y_gyro;
      gz_offset = z_gyro;

}

void process_motion(void)
{
    unsigned long t_now = millis();
    float dt = get_delta_time(t_now);
  
    read_mpu();

    float ax_p = (ax - ax_offset) / CONST_16G;
    float ay_p = (ay - ay_offset) / CONST_16G;
    float az_p = (az / CONST_16G);

    float accel_angle_y = atan(-1 * ax_p / sqrt(pow(ay_p, 2) + pow(az_p, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_x = atan(ay_p / sqrt(pow(ax_p, 2) + pow(az_p, 2))) * RADIANS_TO_DEGREES;

    float gx_p = (gx - gx_offset) / CONST_2000;
    float gy_p = (gy - gy_offset) / CONST_2000;
    float gz_p = (gz - gz_offset) / CONST_2000;

    float gyro_angle_x = gx_p * dt + get_last_angle_x();
    float gyro_angle_y = gy_p * dt + get_last_angle_y();

    float angle_x = ALPHA * gyro_angle_x + (1.0 - ALPHA) * accel_angle_x;
    float angle_y = ALPHA * gyro_angle_y + (1.0 - ALPHA) * accel_angle_y;

    float vel_x = (ax_p * dt * CONST_G);
    float vel_y = (ay_p * dt * CONST_G);
    float vel = sqrt(pow(vel_x, 2) + pow(vel_y, 2)) * KMPH;

    mydevice.SPEED[0] = 0x30;
    mydevice.SPEED[1] = 0x30;
    mydevice.SPEED[2] = 0x30;
    mydevice.SPEED[3] = 0x0;
    
    if (vel > 5)
        sprintf(mydevice.SPEED, "%3d", (uint16_t)vel);
        
    //sprintf(mydevice.GX, "%3d", (uint16_t)angle_x);
    //sprintf(mydevice.GY, "%3d", (uint16_t)angle_y);
    
    if (angle_x > 260)          //Devrildiysek SMS atalim.
    {
        send_event_sms = 1; 
        mydevice.EVENT = 0x31;
    }
    else
        mydevice.EVENT = 0x30;
    
    set_last_time(t_now);

    set_last_gyro_angle_x(gyro_angle_x);
    set_last_gyro_angle_y(gyro_angle_y);

    set_last_angle_x(angle_x);
    set_last_angle_y(angle_y);    
}

void sms_task(void)
{
    if (send_event_sms)         //Olay smsi gonder.
    {
        send_event_sms = 0;
        
        if (smssent == 0)       //Tek bir kaza icin 1 sms gondermeli. Sistem resetlenmeden yeni sms yollamaz.
        {
            smssent = 1;
            send_sms();
        }
    }   
}
/*****************************
* MAIN
* 
* 
*****************************/
int main()
{
    
    CyDelay(500);               //wait for dc stabilisation

    setup_sys();                //setup interrupts and system specific

    //init_i2c();                 //init i2c for mems   
    
    init_uart_gsm();            //initialize uart for gsm module
    init_uart_gps();
    init_gps();                 //init gps
    
    check_modem();              //check modem and power it on if necessary

    init_modem();               //init modem if it present
    
    //init_mpu();                 //init mpu6050    //MPU6050 Kartı lehimlendiğinde acilmali
    //calibrate_sensors();          //MPU6050 Kartı lehimlendiğinde acilmali
    //sprintf(mydevice.TEMP, "%3d", 0);  //Gecici olarak 0 temp gonderiyoruz.
    
    set_last_time(millis());

    while (1)
    {
        
        if (checkresponse(CONNCLOSE) == CONNCLOSE)  //GPRS baglantisi karsi tarafca sonlandirildi. ya da baglanti koptu.
        {
            online = 0;   
            gprsstate = ATTACHREQ;
        }
        
        if (f300)   //300mSec Tasks
        {
            f300 = 0;
            if (online) indicator();
        }
        
        if (f1000) //1Hz Tasks
        {
            f1000 = 0;
            
            //process_motion();     //MPU6050 Kartı lehimlendiğinde acilmali
            
            if (modeminited == MODEM_READY)
            {
                if (online == 0) indicator();
                gprs_task();
                main_task();   
            }
            else
                check_network();
        }
    }
}

