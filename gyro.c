/************************I2C_LIBRARY***************************/

#include <avr/io.h>
#include <util/delay.h>
#include "i2c_lib.h"

#define dev_add 0x68 << 1 
//#define dev_add 0x71 << 1 // device address…last bit corresponds to R/W 
#define WHO_AM_I 0x00  /*internal register in which the device id is stored*/  
#define DEV_ID_VAL 0xE3  /*value of default device id which is stored in WHO_AM_I register*/ 
#define DATA01 0x33  //internal data register where data is stored
#define DATA02 0x34  //internal data register where data is stored 
#define DATA03 0x35  //internal data register where data is stored
#define DATA04 0x36  //internal data register where data is stored 
#define INIT_REG 0x2B  /* an internal initializing data register in the device into which INIT_VALUE needs to be send to initialize the device*/ 
#define INIT_VAL 0x30
#define A_XOUT_H     0x3B
#define A_XOUT_L     0x3C
#define A_YOUT_H     0x3D
#define A_YOUT_L     0x3E
#define A_ZOUT_H     0x3F
#define A_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define G_XOUT_H      0x43
#define G_XOUT_L      0x44
#define G_YOUT_H      0x45
#define G_YOUT_L      0x46
#define G_ZOUT_H      0x47
#define G_ZOUT_L      0x48
void check_status();
void check_status(STAT status) 
{ 
       if(status != OK)  
         {  
             while(1);  //error in transmission using i2c
          }
 } 

/********************************
function name	:	i2c_init
functionality	:	initialise i2c or enabling i2cen bit
arguments		:	none
return value	:	void
*********************************/
void i2c_init()
{
	TWSR = 0x00;
	TWCR = 0x00;
	TWCR = i2cen | done | eack;		//enables i2c , ack bit and clears TWINT bit
	TWBR = 0X0A;					//400khz speed for TWI in 14745600 hz clock frequency
}

/********************************
function name	:	i2c_start
functionality	:	starts tranmission or setting start bit high
arguments		:	none
return value	:	void
********************************/
void i2c_start()
{
	TWCR |= start | eack;			//starting transmission ie TWSTA bit is made high
}

/********************************
function name	:	clear_twint
functionality	:	clears the interrupt bit
arguments		:	none	
return value	:	void
********************************/
void clear_twint()
{
	TWCR |= done;					//clearing TWINT bit
}

/********************************
function name	:	wait
functionality	:	waiting till the data is transmitted or till the interrupt bit gets high
arguments		:	none
return value	:	void
*********************************/
void wait()
{
	while(!(TWCR & done));			//waiting for TWINT bit to be high ie to complete transmission
}

/********************************
function name	:	i2c_stop
functionality	:	stops the transmission or sets the stop bit high
arguments		:	none
return value	:	void
********************************/
void i2c_stop()
{
	TWCR |= stop;					//TWSTO bit is set high
}

/********************************
function name	:	i2c_getstatus	
functionality	:	to find the status of the i2c
arguments		:	none
return value	:	UINT8 (returns the status)
********************************/
UINT8 i2c_getstatus()
{
	UINT8 status;
	status = TWSR & 0xF8;			//5 bits of TWSR register gives the status
	return status;
}

/********************************
function name	:	i2c_write
functionality	:	writes a byte of data to TWDR register
arguments		:	UINT8 data(the byte which is to be written)
return value	:	void
*********************************/
void i2c_write(UINT8 data)
{
	TWDR = data;					//writing data or add to TWDR register
}

/********************************
function name	:	i2c_get
functionality	:	reads a byte of data from TWDR register
arguments		:	INT8 *data (points to the byte to which the data needs to be copied)
return value	:	void
*********************************/
void i2c_get(INT8 *data)
{
	*data = TWDR;					//copying data from TWDR register
}

/********************************
function name	:	i2c_sendbyte
functionality	:	sends a byte of data to the slave
arguments		:	UINT8 dev_add (the device address of the slave),
					UINT8 int_add (address of the internal register of the slave),
					UINT8 data(byte of data which needs to be sent)
return value	:	enum (value which tells if the transmission was succesfull or not)
********************************/
STAT i2c_sendbyte(UINT8 dev_add, UINT8 int_add,UINT8 data)
{
	i2c_start();					//starting transmission
	wait();							//then wait for succesfull transmission
	if(i2c_getstatus() != 0x08)		//check status
		return START_ERR;

	i2c_write(dev_add | write);		//writing slave + write to TWDR
	TWCR &= ~start;
	clear_twint();					//clear TWINT bit starts transmission
	wait();
	if(i2c_getstatus()!=0x18)
		return SLAVEW_ERR;

	i2c_write(int_add);				//writing int_add to TWDR
	clear_twint();
	wait();
	if(i2c_getstatus()!=0x28)
	{	return WRITE_ERR;	}

	i2c_write(data);				//writing one byte of data...to write multi byte | with 0x80
	clear_twint();
	wait();
	if(i2c_getstatus()!=0x28)
		return WRITE_ERR;

	i2c_stop();

	return OK;
}

/********************************
function name	:	i2c_getbyte
functionality	:	reads a byte of data from the slave
arguments		:	UINT8 dev_add (the device address of the slave),
					UINT8 int_add (address of the internal register of the slave),
					INT8 *data(points to the byte to which the data needs to be copied)					
return value	:	enum (value which tells if the transmission was succesfull or not)
********************************/
STAT i2c_getbyte(UINT8 dev_add,UINT8 int_add,INT8 *data)
{
	i2c_start();					//starting transmission
	wait();							//then wait for succesfull transmission
	if(i2c_getstatus() != 0x08)		//checking status
		return START_ERR;

	i2c_write(dev_add | write);		//writing slave + write to TWDR
	TWCR &= ~start;
	clear_twint();					//clear TWINT bit and starts transmission
	wait();
	if(i2c_getstatus()!=0x18)
		return SLAVEW_ERR;	

	i2c_write(int_add);				//writing int_add to TWDR
	clear_twint();
	wait();
	if(i2c_getstatus()!=0x28)
		return WRITE_ERR;

	i2c_start();					//repeated start
	wait();
	if(i2c_getstatus() != 0x10)
		return REPSTART_ERR;

	i2c_write(dev_add | read);		//writing slave + read to TWDR register
	TWCR &= ~start;
	clear_twint();
	wait();
	if(i2c_getstatus()!=0x40)
		return SLAVER_ERR;

	TWCR &= ~eack;
	clear_twint();
	wait();
	if(i2c_getstatus()!=0x58)
		return READ_ERR;

	i2c_get(data);					//getting data to the char pointer

	i2c_stop();						//stopping the transmission

	return OK;
}

/********************************
function name	:	i2c_read_multi_byte
functionality	:	reads multiple no of bytes from the slave
arguments		:	UINT8 dev_add (the device address of the slave),
					UINT8 int_add (address of the internal register of the slave),
					UINT16 n(no of bytes which needs to be read)
					INT8 *data(points to the byte to which the data needs to be copied)	
return value	:	enum (value which tells if the transmission was succesfull or not)
*********************************/
STAT i2c_read_multi_byte(UINT8 dev_add,UINT8 int_add,UINT16 n,INT8 *data)
{
	UINT16 i;
	i2c_start();					//starting transmission
	wait();							//then wait for succesfull transmission
	if(i2c_getstatus() != 0x08)		//check status
		return START_ERR;

	i2c_write(dev_add | write);		//writing slave + write to TWDR
	TWCR &=~start;					//to avoid transmitting start bit again
	clear_twint();					//clear TWINT bit and starts transmission
	wait();
	if(i2c_getstatus()!=0x18)
		return SLAVEW_ERR;

	i2c_write(int_add | 0x80);		//to read multi-byte logical or(|) the register with 0x80
	clear_twint();
	wait();
	if(i2c_getstatus()!=0x28)
		return WRITE_ERR;

	i2c_start();					//repeated start
	wait();
	if(i2c_getstatus() != 0x10)
		return REPSTART_ERR;

	i2c_write(dev_add | read);		//writing slave + read to TWDR register
	TWCR &= ~start;
	clear_twint();
	wait();
	if(i2c_getstatus()!=0x40)
		return SLAVER_ERR;

	for(i=0;i<n;i++)
	{
		if(i==(n-1))				//if last byte send nack
		{
			TWCR &= ~eack;			//sending NACK since it is the last bit
			clear_twint();
			wait();
			if(i2c_getstatus()!=0x58)
				return NACK_ERR;	
			i2c_get(&data[i]);		
		}
		else
		{
			TWCR |= eack;			//sending ACK
			clear_twint();
			wait();
			if(i2c_getstatus()!=0x50)
				return ACK_ERR;	

			i2c_get(&data[i]);		//getting required data
		}
	}

	i2c_stop();						//stopping the transmission

	return OK;	
}





int main()
   {  int Ax,Ay,Az,Gx,Gy,Gz;
       UINT8 devid;  
       INT8 data[4];  
       i2c_init(); 
 
                 /*getting the device id of the device and checking if it is the default value */ 
                check_status(i2c_getbyte(dev_add, WHO_AM_I,&devid));
                 if(devid != DEV_ID_VAL) 
                       {   while(1);        //Invalid Device id  
                         } 
 
                /* initializing the device */ 
               check_status(i2c_sendbyte(dev_add, INIT_REG, INIT_VAL));
                  while(1)                            //reading data continuously
            {  
              /* reading 4 data bytes and storing it into data[4] array */  
               check_status(i2c_read_multi_byte(dev_add, DATA01,4,data));    
			  //we can then process the data accordingly 
         
		    }

       Ax=i2c_read_multi_byte(dev_add, A_XOUT_H,4,A_XOUT_L);
	   Ay=i2c_read_multi_byte(dev_add, A_YOUT_H,4,A_YOUT_L);
	   Az=i2c_read_multi_byte(dev_add, A_ZOUT_H,4,A_ZOUT_L);
	   
	   Gx=i2c_read_multi_byte(dev_add, G_XOUT_H,4,G_XOUT_L);
	   Gy=i2c_read_multi_byte(dev_add, G_YOUT_H,4,G_YOUT_L);   
	   Gz=i2c_read_multi_byte(dev_add, G_ZOUT_H,4,G_ZOUT_L); 
	   
	   
	   
	        
	   
	   




   
     }



