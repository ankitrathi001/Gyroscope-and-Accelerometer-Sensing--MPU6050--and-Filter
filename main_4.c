/***********************************************************************
 *
 * File Name: main_4.c
 *
 * Author: Ankit Rathi (ASU ID: 1207543476)
 * 			(Ankit.Rathi@asu.edu)
 *
 * Date: 24-NOV-2014
 *
 * Description: A User application program to control 3D aerial vehicle
 * 	using raw data obatined from MPU6050
 **********************************************************************/
 
/**
*	Include Library Headers 
*/
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include "MPU6050.h"
#include "declarations.h"
#include <math.h>
#include <linux/input.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <poll.h>
#include <unistd.h>
#include "Kalman.h"

/**
 * Define constants using the macro
 */
#define MICE_DEVICE 					"/dev/input/event2"
#define I2C_DEVICE 						"/dev/i2c-0"
#define GPIO29_MUX_PIN      			29
#define GPIO30_MUX_PIN      			30
#define GPIO15        					15
#define GPIO_DIRECTION_IN 				1
#define GPIO_DIRECTION_OUT 				0
#define GPIO_VALUE_LOW 					0
#define GPIO_VALUE_HIGH 				1
#define MAX_BUF 						64
#define I2C_MPU6050_SLAVE_ADDRESS		0x68
#define ACCEL_MOVEMENT_END_CHECK 		2
#define AVERAGE_NO_SAMPLES_INTEGRATE	5
#define AVERAGE_NO_SAMPLES_KALMAN		5
#define gyro_xsensitivity 				66.5 //66.5 Dead on at last check
#define gyro_ysensitivity 				66.5 //72.7 Dead on at last check
#define gyro_zsensitivity 				65.5
#define WINDOW_LENGTH					100
#define LEFT_BUTTON_PRESSED				0
#define LEFT_BUTTON_RELEASED			1
#define RIGHT_BUTTON_PRESSED			2
#define RIGHT_BUTTON_RELEASED			3
#define UNITY_IP_ADDRESS				"169.254.147.71"
#define UNITY_PORT_NUMBER				9999
#define POLL_TIMEOUT					-1
#define IS_DATA_PROCESSED_YES			1
#define IS_DATA_PROCESSED_NO			0
#define IS_RESET_REQUIRED_YES			1
#define IS_RESET_REQUIRED_NO			0
#define GALILEO_CPU_FREQUENCY			400000000.00
#define PI								3.14159265
#define ANGLE_DIVIDER					500.0
#define POSITION_DIVIDER				20000.0


unsigned int RESET_REQUIRED = 0;
unsigned int IS_DATA_READY = 0;
unsigned int IS_DATA_PROCESSED = 0;
pthread_mutex_t mutex_mice, mutex_sensor, mutex_interrupt;// mutex for protecting
int fd_mice = 0, fd_i2c = 0, fd_server_sock = 0;
unsigned int GLOBAL_MICE_EVENTS = 0;//  0 = Left Button Pressed, 1 = Left Button Released, 2 = Right Button Pressed , 3 = Right Button Released
struct sockaddr_in server_address;
unsigned long timeStart = 0, timeEnd = 0;
float deltaT = 0.0;
unsigned char connectedToUnity = 0;

int gpio_export(unsigned int gpio);
int gpio_unexport(unsigned int gpio);
int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
int gpio_set_value(unsigned int gpio, unsigned int value);
void MPU6050_Test_I2C();
unsigned int LDByteReadI2C(unsigned char ControlByte, unsigned char Address, unsigned char *data, unsigned char Length);
unsigned int LDByteWriteI2C(unsigned char ControlByte, unsigned char LowAdd, unsigned char data);
void Setup_MPU6050();
void* thread_sensor(void* data);
void* thread_mice_events(void* data);
void* thread_interrupt(void* data);
void Reset_Positions(void);
void movement_end_check(void);
void Send_Data_To_UNITY(void);
void Calibrate_Accel_Gyro(void);
void Read_Accel_Gyro_Values(void);
static inline u64 rdtsc(void);
void FilterTheData();

/***********************************************************************
* main - This is the main function. It creates sepearte threads
* 	for different operations.
*
* Returns -
* 
* Description: This is the main function. It creates sepearte threads
* 	for different operations.
***********************************************************************/
int main(int argc, char **argv, char **envp)
{
	int retValue;
	pthread_t thread_id_interrupt, thread_id_sensor, thread_id_mice;
	
    if((fd_mice = open(MICE_DEVICE, O_RDONLY)) == -1)
    {
        printf("Mice Device opened Failed\n");
        exit(EXIT_FAILURE);
    }
    else
    {
		printf("Mice Device opened Successfully\n");
    }
    
    //Unexport all GPIO Pins
	gpio_unexport(GPIO29_MUX_PIN);
	gpio_unexport(GPIO30_MUX_PIN);
	gpio_unexport(GPIO15);

	//Export all GPIO Pins
	gpio_export(GPIO29_MUX_PIN);
	gpio_export(GPIO30_MUX_PIN);
	gpio_export(GPIO15);

	//Set Directions for all GPIO Pins
	gpio_set_dir(GPIO29_MUX_PIN, GPIO_DIRECTION_OUT);
	gpio_set_dir(GPIO30_MUX_PIN, GPIO_DIRECTION_OUT);
	gpio_set_dir(GPIO15, GPIO_DIRECTION_OUT);

	//Set Values for all GPIO Pins
	gpio_set_value(GPIO29_MUX_PIN, GPIO_VALUE_LOW);
	gpio_set_value(GPIO30_MUX_PIN, GPIO_VALUE_LOW);
	gpio_set_value(GPIO15, GPIO_VALUE_LOW);

	//Set Directions for IO3 to IN direction
	gpio_set_dir(GPIO15, GPIO_DIRECTION_IN);
	
	fd_i2c = open(I2C_DEVICE, O_RDWR);
	
	fd_server_sock = socket( AF_INET, SOCK_STREAM, 0);
    if(fd_server_sock == -1)
    {
        printf("Could not create socket");
        exit(EXIT_FAILURE);
    }
    else
    {
		printf("Socket Created\n");
	}
	
	server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = inet_addr(UNITY_IP_ADDRESS);
    server_address.sin_port = htons(UNITY_PORT_NUMBER);
	
	//Connect to remote UNITY server
    if(connect(fd_server_sock , (struct sockaddr *)&server_address , sizeof(server_address)) < 0)
    {
        perror("Connect Failed. Error\n");
        connectedToUnity = 0;
        //exit(EXIT_FAILURE);
        //return 1;
    }
    else
    {
		printf("Connected to UNITY\n");
		connectedToUnity = 1;
	}
	
	ioctl(fd_i2c, I2C_SLAVE, I2C_MPU6050_SLAVE_ADDRESS);
	
	/* Mutex Initialization*/
	pthread_mutex_init(&mutex_mice, NULL);
	pthread_mutex_init(&mutex_sensor, NULL);
	pthread_mutex_init(&mutex_interrupt, NULL);
	
	
	retValue = pthread_create(&thread_id_mice, NULL, &thread_mice_events, NULL);
	if(retValue)
	{
		printf("ERROR; return code from pthread_create() is %d\n", retValue);
		exit(-1);
	}

	retValue = pthread_create(&thread_id_sensor, NULL, &thread_sensor, NULL);
	if(retValue)
	{
		printf("ERROR; return code from pthread_create() is %d\n", retValue);
		exit(-1);
	}
	
	retValue = pthread_create(&thread_id_interrupt, NULL, &thread_interrupt, NULL);
	if(retValue)
	{
		printf("ERROR; return code from pthread_create() is %d\n", retValue);
		exit(-1);
	}
	
	pthread_join(thread_id_mice, NULL);
	pthread_join(thread_id_interrupt, NULL);
	pthread_join(thread_id_sensor, NULL);
	
	//Unexport all GPIO Pins
	gpio_unexport(GPIO29_MUX_PIN);
	gpio_unexport(GPIO30_MUX_PIN);
	gpio_unexport(GPIO15);

	/*Close the file descriptors*/
	close(fd_mice);
	close(fd_i2c);
	close(fd_server_sock);
	
	return 0;
}

/***********************************************************************
* thread_interrupt - Thread Function to check if the interrupt has
* 	arrived, and set the flag to check if data is ready for reading 
* 	and processing.
* 
* Returns -
* 
* Description: Thread Function to check if the interrupt has
* 	arrived, and set the flag to check if data is ready for reading 
* 	and processing.
***********************************************************************/
void* thread_interrupt(void *data)
{
	struct pollfd poll_io3;
	int rc, retValue;
	char *buf[MAX_BUF];
	//printf("thread_interrupt\n");
	poll_io3.fd = gpio_fd_open(GPIO15);
	poll_io3.events = POLLPRI|POLLERR;
	
	lseek(poll_io3.fd, 0, SEEK_SET);
	retValue = read(poll_io3.fd, buf, 1);
	
	gpio_set_edge(GPIO15, "rising");
	IS_DATA_READY = IS_DATA_PROCESSED_YES;
	while(1)
	{
		//printf("Polling\n");
		lseek(poll_io3.fd, 0, SEEK_SET);
		rc = poll(&poll_io3, 1, POLL_TIMEOUT);
		
		if(rc < 0)
		{
			printf("poll() failed!!! Error Ocurred\n");
			//return -1;
		}
		if(rc == 0)
		{
			//printf("poll() timed out!!! No File Descriptors were ready\n");
		}
		if(rc > 0)
		{
			if(poll_io3.revents & POLLPRI)
			{
				if(IS_DATA_READY == IS_DATA_PROCESSED_YES)
				{
					retValue = read(poll_io3.fd, buf, 1);
					if(retValue > 0)
					{
						pthread_mutex_lock(&mutex_interrupt);
						IS_DATA_READY = IS_DATA_PROCESSED_NO;
						pthread_mutex_unlock(&mutex_interrupt);
						//printf("Rising Edge Detected\n");
					}
				}
				else
				{
					//printf("Data Processing in Progress\n");
				}
			}
			else
			{
				printf("Error in detecting Rising Edge\n");
			}
		}
	}
	pthread_exit(0);
}

/***********************************************************************
* thread_mice_events - Thread Function to detect mouse click events.
* 	Corresponding flags are set with left and right click.
* 
* Returns -
* 
* Description:  Thread Function to detect mouse click events.
* 	Corresponding flags are set with left and right click.
***********************************************************************/
void *thread_mice_events(void *data)
{
    struct input_event ie;
	//printf("thread_mice_events\n");
    while(read(fd_mice, &ie, sizeof(struct input_event)))
    {
        if(ie.code == 272)
        {
			if(ie.value == 1)
			{
				pthread_mutex_lock(&mutex_mice);
				GLOBAL_MICE_EVENTS = LEFT_BUTTON_PRESSED;
				pthread_mutex_unlock (&mutex_mice);
				printf("Left Button Pressed\n");
			}
			else if(ie.value == 0)
			{
				pthread_mutex_lock(&mutex_mice);
				GLOBAL_MICE_EVENTS = LEFT_BUTTON_RELEASED;
				RESET_REQUIRED = IS_RESET_REQUIRED_YES;
				pthread_mutex_unlock (&mutex_mice);
				printf("Left Button Released\n");
			}
		}
		else if(ie.code == 273)
		{
			if(ie.value == 1)
			{
				pthread_mutex_lock(&mutex_mice);
				GLOBAL_MICE_EVENTS = RIGHT_BUTTON_PRESSED;
				pthread_mutex_unlock (&mutex_mice);
				printf("Right Button Pressed\n");
			}
			else if(ie.value == 0)
			{
				pthread_mutex_lock(&mutex_mice);
				GLOBAL_MICE_EVENTS = RIGHT_BUTTON_RELEASED;
				RESET_REQUIRED = IS_RESET_REQUIRED_YES;
				pthread_mutex_unlock (&mutex_mice);
				printf("Right Button Released\n");
			}
		}
		usleep(5000);
    }
	pthread_exit(0);
}

/***********************************************************************
* thread_sensor - Thread Function to calibrate, calculate and pass
* 	values to the UNITY application.
* 
* Returns -
* 
* Description:  Thread Function to calibrate, calculate and pass
* 	values to the UNITY application.
***********************************************************************/
void *thread_sensor(void *data)
{
	int error = 1;
	//printf("thread_sensor\n");
	do
	{
		Setup_MPU6050();
		MPU6050_Test_I2C();
		error = MPU6050_Check_Registers();
	}
	while(error==1);
	
	Calibrate_Accel_Gyro();
	timeStart = rdtsc();
	
	while(1)
	{
		pthread_mutex_lock(&mutex_interrupt);
		IS_DATA_READY = IS_DATA_PROCESSED_YES;
		pthread_mutex_unlock(&mutex_interrupt);
		
		timeEnd = rdtsc();
		Read_Accel_Gyro_Values();
		Send_Data_To_UNITY();
		deltaT = (timeEnd - timeStart) / GALILEO_CPU_FREQUENCY;
		timeStart = rdtsc();
		//printf("deltaT ::: %f\n",deltaT);

		if(RESET_REQUIRED == IS_RESET_REQUIRED_YES)
		{
			Reset_Positions();
			Send_Data_To_UNITY();
			RESET_REQUIRED = IS_RESET_REQUIRED_NO;
		}
		usleep(100000);
		//sleep(1);
	}
	pthread_exit(0);
}

/***********************************************************************
* MPU6050_Test_I2C - Function to test if we are able to communicate with
* 	device.
* 
* Returns -
* 
* Description:  Function to test if we are able to communicate with
* 	device.
* 
* Source : https://github.com/big5824/Quadrocopter
***********************************************************************/
void MPU6050_Test_I2C()
{
    unsigned char Data = 0;
    LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &Data, 1);
	
    if(Data == 0x68)
    {
        printf("I2C Read Test Passed, MPU6050 Address: 0x%x\n", Data);
    }
    else
    {
        printf("ERROR: I2C Read Test Failed, Stopping\n");
    }
}

/***********************************************************************
* Setup_MPU6050 - Function to set all the values of the register to
* 	proper values for optimal usage and interrupt enabling.
* 
* Returns -
* 
* Description:  Function to set all the values of the register to
* 	proper values for optimal usage and interrupt enabling.
* 
* Source : https://github.com/big5824/Quadrocopter
***********************************************************************/
void Setup_MPU6050()
{
	//Sets sample rate to 1000/1+1 = 500Hz
	//LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x01);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x30);
	//Disable FSync, 48Hz DLPF
	//LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x03);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x00);
	//Disable gyro self tests, scale of 500 degrees/s
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0b00001000);
	//Disable accel self tests, scale of +-4g, no DHPF
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0b00001000);
	//Freefall threshold of <|0mg|
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, 0x00);
	//Freefall duration limit of 0
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 0x00);
	//Motion threshold of >0mg
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 0x00);
	//Motion duration of >0s
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 0x00);
	//Zero motion threshold
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 0x00);
	//Zero motion duration threshold
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 0x00);
	//Disable sensor output to FIFO buffer
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);
	//AUX I2C setup
	//Sets AUX I2C to single master control, plus other config
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);
	//Setup AUX I2C slaves
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, 0x00);

	//Setup INT pin and AUX I2C pass through
	//LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x30);
	//Enable data ready interrupt
	//LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x01);
	//Slave out, dont care
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00);
	//More slave config
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
	//Reset sensor signal paths
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
	//Motion detection control
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
	//Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
	//Sets clock source to gyro reference w/ PLL
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0b00000010);
	//Controls frequency of wakeups in accel low power mode plus the sensor standby modes
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
	//Data transfer to and from the FIFO buffer
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 0x00);
	//MPU6050_RA_WHO_AM_I //Read-only, I2C address
	printf("MPU6050 Setup Complete\n");
}

/***********************************************************************
* MPU6050_Check_Registers - Function to check if all registers have been
* 	set to proper values as done in Setup_MPU6050 function.
* 
* Returns -
* 
* Description:  Function to check if all registers have been
* 	set to proper values as done in Setup_MPU6050 function.
* 
* Source : https://github.com/big5824/Quadrocopter
***********************************************************************/
int MPU6050_Check_Registers()
{
	unsigned char Data = 0x00;
	unsigned char Failed = 0;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, &Data, 1);
	//if(Data != 0x01)
	if(Data != 0x30)
	{
		printf("\nRegister check 1 failed, value should be 0x01, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 2 failed, value should be 0x03, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, &Data, 1);
	if(Data != 0b00001000)
	{
		printf("\nRegister check 3 failed, value should be 0b00001000, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &Data, 1);
	if(Data != 0b00001000)
	{
		printf("\nRegister check 4 failed, value should be 0b00001000, was 0x%x", Data); 
		Failed = 1; 
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, &Data, 1);
	if(Data != 0x00) 
	{ 
		printf("\nRegister check 5 failed, value should be 0x00, was 0x%x", Data); 
		Failed = 1; 
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, &Data, 1);
	if(Data != 0x00) 
	{ 
		printf("\nRegister check 6 failed, value should be 0x00, was 0x%x", Data); 
		Failed = 1; 
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 7 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 8 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 9 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 10 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 11 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 12 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 13 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 14 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 15 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 16 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 17 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 18 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 19 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 20 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 21 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 22 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 23 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 24 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 25 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 26 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 27 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 28 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 29 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, &Data, 1);
	if(Data != 0x30)
	{
		printf("\nRegister check 30 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, &Data, 1);
	if(Data != 0x01)
	{
		printf("\nRegister check 31 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 32 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 33 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 34 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 35 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1; 
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 36 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 37 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 38 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister 39 check failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, &Data, 1);
	if(Data != 0x02)
	{
		printf("\nRegister check 40 failed, value should be 0x02, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 41 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, &Data, 1);
	if(Data != 0x00)
	{
		printf("\nRegister check 42 failed, value should be 0x00, was 0x%x", Data);
		Failed = 1;
	}
	
	if (Failed == 0)
	{
		printf("Register value check passed\n");
	}
	else
	{
		printf("Register value check failed\n");
	}
	return(Failed);
}

/***********************************************************************
* Read_Accel_Gyro_Values - Gets raw accelerometer data, raw Gyrometer 
* 	data, performs processing and makes data ready for transmission over
* 	I2C to UNITY.
* 
* Returns -
* 
* Description:  Gets raw accelerometer data, raw Gyrometer 
* 	data, performs processing and makes data ready for transmission over
* 	I2C to UNITY.
***********************************************************************/
void Read_Accel_Gyro_Values()
{
	unsigned int count1;
	unsigned char whichButton = LEFT_BUTTON_RELEASED;
	signed int avg_sample_count = 5;
	unsigned char dataReceived[14];
	count1 = 0;
	
	
	
	//printf("\n Read_Accel_Gyro_Values Start\n");
	
	accel_X[1] = 0;
	accel_Y[1] = 0;
	accel_Z[1] = 0;
	
	gyro_Rate_X[1] = 0;
	gyro_Rate_Y[1] = 0;
	gyro_Rate_Z[1] = 0;
	
	pthread_mutex_lock(&mutex_mice);
	whichButton = GLOBAL_MICE_EVENTS;
	pthread_mutex_unlock(&mutex_mice);
	
	if(whichButton == LEFT_BUTTON_RELEASED)
	{
		avg_sample_count = AVERAGE_NO_SAMPLES_INTEGRATE;
	}
	else if(whichButton == RIGHT_BUTTON_RELEASED)
	{
		avg_sample_count = AVERAGE_NO_SAMPLES_KALMAN;
	}
	else
	{
		avg_sample_count = AVERAGE_NO_SAMPLES_INTEGRATE;
	}
	
	do
	{
		if(IS_DATA_READY == IS_DATA_PROCESSED_YES)
		{
			pthread_mutex_lock(&mutex_interrupt);
			IS_DATA_READY = IS_DATA_PROCESSED_NO;
			pthread_mutex_unlock(&mutex_interrupt);
			
			LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &dataReceived[0], 14);
			
			//int i=0;
			//printf("\nData Receieved :::");
			//for(i=0;i<14;i++)
			//{
				//printf(" ox%x ",dataReceived[i]);
			//}
			
			ACCEL_XOUT = (dataReceived[0] << 8 | dataReceived[1]);
			ACCEL_YOUT = (dataReceived[2] << 8 | dataReceived[3]);
			ACCEL_ZOUT = (dataReceived[4] << 8 | dataReceived[5]);

			GYRO_XOUT = (dataReceived[8] << 8 |dataReceived[9]);
			GYRO_YOUT = (dataReceived[10] << 8|dataReceived[11]);
			GYRO_ZOUT = (dataReceived[12] << 8|dataReceived[13]);

			//filtering routine for noise attenuation 64 samples are averaged. The resulting average represents the acceleration of an instant
			accel_X[1] = accel_X[1] + ACCEL_XOUT;
			accel_Y[1] = accel_Y[1] + ACCEL_YOUT;
			accel_Z[1] = accel_Z[1] + ACCEL_ZOUT;
			
			//filtering routine for noise attenuation 64 samples are averaged. The resulting average represents the gyro rate of an instant
			gyro_Rate_X[1] = gyro_Rate_X[1] + (float)GYRO_XOUT;
			gyro_Rate_Y[1] = gyro_Rate_Y[1] + (float)GYRO_YOUT;
			gyro_Rate_Z[1] = gyro_Rate_Z[1] + (float)GYRO_ZOUT;
			
			count1++;

			pthread_mutex_lock(&mutex_interrupt);
			IS_DATA_READY = IS_DATA_PROCESSED_YES;
			pthread_mutex_unlock(&mutex_interrupt);
		}
	}while(count1 != avg_sample_count);
	
	
	//division by avg_sample_count
	accel_X[1]= accel_X[1] / avg_sample_count;
	accel_Y[1]= accel_Y[1] / avg_sample_count;
	accel_Z[1]= accel_Z[1] / avg_sample_count;
	
	//eliminating zero reference offset of the acceleration data to obtain positive and negative acceleration
	accel_X[1] = accel_X[1] - (int)ACCEL_XOUT_OFFSET;
	accel_Y[1] = accel_Y[1] - (int)ACCEL_YOUT_OFFSET;
	accel_Z[1] = accel_Z[1] - (int)ACCEL_ZOUT_OFFSET;
	
	//Discrimination window applied to the X axis acceleration variable
	if((accel_X[1] <= WINDOW_LENGTH)&&(accel_X[1] >= -WINDOW_LENGTH))
	{
		accel_X[1] = 0;
	}
	if((accel_Y[1] <= WINDOW_LENGTH)&&(accel_Y[1] >= -WINDOW_LENGTH))
	{
		accel_Y[1] = 0;
	}
	if((accel_Z[1] <= WINDOW_LENGTH)&&(accel_Z[1] >= -WINDOW_LENGTH))
	{
		accel_Z[1] = 0;
	}
	
	//first X,Y,Z integration of Acceleration:
	velocity_X[1] = velocity_X[0] + accel_X[0] + (((accel_X[1] - accel_X[0])/(2)) * deltaT);
	velocity_Y[1] = velocity_Y[0] + accel_Y[0] + (((accel_Y[1] - accel_Y[0])/(2)) * deltaT);
	velocity_Z[1] = velocity_Z[0] + accel_Z[0] + (((accel_Z[1] - accel_Z[0])/(2)) * deltaT);
	
	//second X,Y,Z integration of Acceleration:
	position_X[1] = position_X[0] + velocity_X[0] + (((velocity_X[1] - velocity_X[0])/(2)) * 1);
	position_Y[1] = position_Y[0] + velocity_Y[0] + (((velocity_Y[1] - velocity_Y[0])/(2)) * 1);
	position_Z[1] = position_Z[0] + velocity_Z[0] + (((velocity_Z[1] - velocity_Z[0])/(2)) * 1);
	
	//The current acceleration value must be sent to the previous acceleration variable in order to introduce the new acceleration value.
	accel_X[0] = accel_X[1];
	accel_Y[0] = accel_Y[1];
	accel_Z[0] = accel_Z[1];
	
	//The current velocity value must be sent to the previous velocity variable in order to introduce the new velocity value.
	velocity_X[0] = velocity_X[1];
	velocity_Y[0] = velocity_Y[1];
	velocity_Z[0] = velocity_Z[1];
	
	movement_end_check();
	
	//The current position value must be sent to the previous position variable in order to introduce the new position value.
	position_X[0] = position_X[1];
	position_Y[0] = position_Y[1];
	position_Z[0] = position_Z[1];
	
	//division by avg_sample_count
	gyro_Rate_X[1] = gyro_Rate_X[1] / (float)avg_sample_count;
	gyro_Rate_Y[1] = gyro_Rate_Y[1] / (float)avg_sample_count;
	gyro_Rate_Z[1] = gyro_Rate_Z[1] / (float)avg_sample_count;
	
	//eliminating zero reference offset of the gyro data to obtain positive and negative gyro
	gyro_Rate_X[1] = gyro_Rate_X[1] - (float)GYRO_XOUT_OFFSET;
	gyro_Rate_Y[1] = gyro_Rate_Y[1] - (float)GYRO_YOUT_OFFSET;
	gyro_Rate_Z[1] = gyro_Rate_Z[1] - (float)GYRO_ZOUT_OFFSET;
	
	gyro_Rate_X[1] = (float)gyro_Rate_X[1]/gyro_xsensitivity;
	gyro_Rate_Y[1] = (float)gyro_Rate_Y[1]/gyro_ysensitivity;
	gyro_Rate_Z[1] = (float)gyro_Rate_Z[1]/gyro_zsensitivity;

	//first X,Y,Z integration of Gyro Rates
	gyro_Angle_X[1] = gyro_Angle_X[0] + gyro_Rate_X[0] + (((gyro_Rate_X[1] - gyro_Rate_X[0]) / (2)) * deltaT);
	gyro_Angle_Y[1] = gyro_Angle_Y[0] + gyro_Rate_Y[0] + (((gyro_Rate_Y[1] - gyro_Rate_Y[0]) / (2)) * deltaT);
	gyro_Angle_Z[1] = gyro_Angle_Z[0] + gyro_Rate_Z[0] + (((gyro_Rate_Z[1] - gyro_Rate_Z[0]) / (2)) * deltaT);
	
	GYRO_XRATE = gyro_Rate_X[1];
	GYRO_YRATE = gyro_Rate_Y[1];
	GYRO_ZRATE = gyro_Rate_Z[1];
	
	GYRO_XANGLE = gyro_Angle_X[1];
	GYRO_YANGLE = gyro_Angle_Y[1];
	GYRO_ZANGLE = gyro_Angle_Z[1];
	
	/* Implement Kalman filter here */
	KALMAN_ANGLE_X = kalman_updateX(gyro_Angle_X[1], (float)accel_Y[1], (float)accel_Z[1], deltaT);
	KALMAN_ANGLE_Y = kalman_updateY(gyro_Angle_Y[1], (float)accel_X[1], (float)accel_Z[1], deltaT);
	KALMAN_ANGLE_Z = gyro_Angle_Z[1];

	gyro_angle_temp_X = pre_gyro_Angle_X;
	gyro_angle_temp_Y = pre_gyro_Angle_Y;
	gyro_angle_temp_Z = pre_gyro_Angle_Z;

	if(whichButton == LEFT_BUTTON_RELEASED)
	{
		if(isnanf(gyro_Angle_X[1]))
		{
			gyro_angle_temp_X = gyro_Angle_X[1];
		}
		if(isnanf(gyro_Angle_Y[1]))
		{
			gyro_angle_temp_Y = gyro_Angle_Y[1];
		}
		if(isnanf(gyro_Angle_Z[1]))
		{
			gyro_angle_temp_Z = gyro_Angle_Z[1];
		}
	}
	else if(whichButton == RIGHT_BUTTON_RELEASED)
	{
		if(!isnanf(KALMAN_ANGLE_X))
		{
			gyro_angle_temp_X = KALMAN_ANGLE_X;
		}
		if(!isnanf(KALMAN_ANGLE_Y))
		{
			gyro_angle_temp_Y = KALMAN_ANGLE_Y;
		}
		if(!isnanf(KALMAN_ANGLE_Z))
		{
			gyro_angle_temp_Z = KALMAN_ANGLE_Z;
		}
	}

	//The current angle value must be sent to the previous angle variable in order to introduce the new angle value.
	gyro_Angle_X[0] = gyro_Angle_X[1];
	gyro_Angle_Y[0] = gyro_Angle_Y[1];
	gyro_Angle_Z[0] = gyro_Angle_Z[1];
	
	//The current rate value must be sent to the previous rate variable in order to introduce the new rate value.
	gyro_Rate_X[0] = gyro_Rate_X[1];
	gyro_Rate_Y[0] = gyro_Rate_Y[1];
	gyro_Rate_Z[0] = gyro_Rate_Z[1];

	//printf("\n Read_Accel_Gyro_Values End\n");
}

/***********************************************************************
* Send_Data_To_UNITY - Function to send data to Unity
* 
* Returns -
* 
* Description:  Function to send data to Unity
***********************************************************************/
void Send_Data_To_UNITY(void)
{
	char message[1000];

    sprintf(message,"%f %f %f %f %f %f\n",	(float)((position_X[0] - pre_position_X)/POSITION_DIVIDER),
											(float)((position_Y[0] - pre_position_Y)/POSITION_DIVIDER),
											(float)((position_Z[0] - pre_position_Z)/POSITION_DIVIDER),
											(float)((gyro_angle_temp_X - pre_gyro_Angle_X)/ANGLE_DIVIDER),
											(float)((gyro_angle_temp_Y - pre_gyro_Angle_Y)/ANGLE_DIVIDER),
											(float)((gyro_angle_temp_Z - pre_gyro_Angle_Z)/ANGLE_DIVIDER)
											);

      
	printf("\nmessage to UNITY ::: %s\n",message);
	
	if(connectedToUnity == 1)
	{
		if(sendto(fd_server_sock,message,strlen(message),0,(struct sockaddr *)&server_address,sizeof(server_address)) < 0)
		{
			printf("TCP : error \n");
		}
		else
		{
			pre_position_X = position_X[0];
			pre_position_Y = position_Y[0];
			pre_position_Z = position_Z[0];
	
			PRE_GYRO_XRATE = GYRO_XRATE;
			PRE_GYRO_YRATE = GYRO_YRATE;
			PRE_GYRO_ZRATE = GYRO_ZRATE;
			
			pre_gyro_Angle_X = gyro_angle_temp_X;
			pre_gyro_Angle_Y = gyro_angle_temp_Y;
			pre_gyro_Angle_Z = gyro_angle_temp_Z;
		}
	}
	else
	{
		pre_position_X = position_X[0];
		pre_position_Y = position_Y[0];
		pre_position_Z = position_Z[0];

		PRE_GYRO_XRATE = GYRO_XRATE;
		PRE_GYRO_YRATE = GYRO_YRATE;
		PRE_GYRO_ZRATE = GYRO_ZRATE;

		pre_gyro_Angle_X = gyro_angle_temp_X;
		pre_gyro_Angle_Y = gyro_angle_temp_Y;
		pre_gyro_Angle_Z = gyro_angle_temp_Z;
	}
}

/***********************************************************************
* Reset_Positions - Function to calibrate Accelerometer and Gyrometer
* 	readings.
* 
* Returns -.
* 
* Description:  Function to calibrate Accelerometer and Gyrometer
* 	readings.
***********************************************************************/
void Calibrate_Accel_Gyro(void)
{
	unsigned int count1;
	unsigned char dataReceived[14];
	printf("\nCalibrating Accelerometer Beginning\n");
	
	count1 = 0;

	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &dataReceived[0], 14);
	do
	{
		if(IS_DATA_READY == IS_DATA_PROCESSED_NO)
		{
			//IS_DATA_PROCESSED = IS_DATA_PROCESSED_NO;
			IS_DATA_READY = IS_DATA_PROCESSED_NO;
			
			LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &dataReceived[0], 14);
			
			/*
			int i=0;
			printf("\nData Receieved :::");
			for(i=0;i<14;i++)
			{
				printf(" %d ",dataReceived[i]);
			}
			*/
			
			ACCEL_XOUT = (dataReceived[0] << 8|dataReceived[1]);
			ACCEL_YOUT = (dataReceived[2] << 8|dataReceived[3]);
			ACCEL_ZOUT = (dataReceived[4] << 8|dataReceived[5]);
			GYRO_XOUT = (dataReceived[8] << 8 |dataReceived[9]);
			GYRO_YOUT = (dataReceived[10] << 8|dataReceived[11]);
			GYRO_ZOUT = (dataReceived[12] << 8|dataReceived[13]);
			
			// Accumulate Samples
			ACCEL_XOUT_OFFSET_1000SUM += ACCEL_XOUT;
			ACCEL_YOUT_OFFSET_1000SUM += ACCEL_YOUT;
			ACCEL_ZOUT_OFFSET_1000SUM += ACCEL_ZOUT;
			
			GYRO_XOUT_OFFSET_1000SUM += GYRO_XOUT;
			GYRO_YOUT_OFFSET_1000SUM += GYRO_YOUT;
			GYRO_ZOUT_OFFSET_1000SUM += GYRO_ZOUT;
			count1++;
			//printf("count1 = %d\n",count1);
			IS_DATA_READY = IS_DATA_PROCESSED_YES;
		}
	}while(count1!=1024); // 1024 times
	
	// division between 1024
	ACCEL_XOUT_OFFSET = (short)(ACCEL_XOUT_OFFSET_1000SUM / 1024);
	ACCEL_YOUT_OFFSET = (short)(ACCEL_YOUT_OFFSET_1000SUM / 1024);
	ACCEL_ZOUT_OFFSET = (short)(ACCEL_ZOUT_OFFSET_1000SUM / 1024);
	
	GYRO_XOUT_OFFSET = (short)(GYRO_XOUT_OFFSET_1000SUM / 1024);
	GYRO_YOUT_OFFSET = (short)(GYRO_YOUT_OFFSET_1000SUM / 1024);
	GYRO_ZOUT_OFFSET = (short)(GYRO_ZOUT_OFFSET_1000SUM / 1024);
	
	printf("\nCalibrating Accelerometer and Gyrometer Completed Successfully\n");
	
	printf("\nCalibrated Acceleration offset X = %i \n",ACCEL_XOUT_OFFSET);
	printf("Calibrated Acceleration offset Y = %i \n",ACCEL_YOUT_OFFSET);
	printf("Calibrated Acceleration offset Z = %i \n",ACCEL_ZOUT_OFFSET);
	
	printf("\nCalibrated Gyro offset X = %i \n",GYRO_XOUT_OFFSET);
	printf("Calibrated Gyro offset Y = %i \n",GYRO_YOUT_OFFSET);
	printf("Calibrated Gyro offset Z = %i \n",GYRO_ZOUT_OFFSET);
}

/***********************************************************************
* Reset_Positions - Function to reset the position back to (0,0,0).
* 
* Returns -.
* 
* Description:  Function to reset the position back to (0,0,0).
***********************************************************************/
void Reset_Positions(void)
{
	printf("\nResetting Positions\n");
	pthread_mutex_lock(&mutex_sensor);
	
	position_X[0] = 0;
	position_Y[0] = 0;
	position_Z[0] = 0;
	position_X[1] = 0;
	position_Y[1] = 0;
	position_Z[1] = 0;
	
	velocity_X[0] = 0;
	velocity_Y[0] = 0;
	velocity_Z[0] = 0;
	velocity_X[1] = 0;
	velocity_Y[1] = 0;
	velocity_Z[1] = 0;
	
	accel_X[0] = 0;
	accel_Y[0] = 0;
	accel_Z[0] = 0;
	accel_X[1] = 0;
	accel_Y[1] = 0;
	accel_Z[1] = 0;
	
	gyro_Rate_X[0] = 0;
	gyro_Rate_Y[0] = 0;
	gyro_Rate_Z[0] = 0;
	gyro_Rate_X[1] = 0;
	gyro_Rate_Y[1] = 0;
	gyro_Rate_Z[1] = 0;

	gyro_Angle_X[0] = 0;
	gyro_Angle_Y[0] = 0;
	gyro_Angle_Z[0] = 0;
	gyro_Angle_X[1] = 0;
	gyro_Angle_Y[1] = 0;
	gyro_Angle_Z[1] = 0;
	
	pre_gyro_Angle_X = 0;
	pre_gyro_Angle_Y = 0;
	pre_gyro_Angle_Z = 0;
	
	gyro_angle_temp_X = 0;
	gyro_angle_temp_Y = 0;
	gyro_angle_temp_Z = 0;
	
	pthread_mutex_unlock (&mutex_sensor);
	printf("\nResetting Positions Complete\n");
}

/***********************************************************************
* movement_end_check - Function to check if movement has ended.
* 
* Returns -.
* 
* Description: Function to check if movement has ended.
***********************************************************************/
void movement_end_check(void)
{
	//we count the number of acceleration samples that equals zero
	if(accel_X[1] == 0) 
	{
		countx++;
	}
	else
	{
		countx = 0;
	}
	 //if this number exceeds ACCEL_MOVEMENT_END_CHECK, we can assume that velocity is zero
	if(countx >= ACCEL_MOVEMENT_END_CHECK)
	{
		velocity_X[1] = 0;
		velocity_X[0] = 0;
	}
	
	if(accel_Y[1] == 0) //we do the same for the Y axis
	{
		county++;
	}
	else
	{
		county =0;
	}
	if(county >= ACCEL_MOVEMENT_END_CHECK)
	{
		velocity_Y[1] = 0;
		velocity_Y[0] = 0;
	}
	
	if(accel_Z[1] == 0) //we do the same for the Z axis
	{
		countz++;
	}
	else
	{
		countz = 0;
	}
	if(countz >= ACCEL_MOVEMENT_END_CHECK)
	{
		velocity_Z[1] = 0;
		velocity_Z[0] = 0;
	}
}

/***********************************************************************
* LDByteReadI2C - Function to read from I2C.
* @ControlByte: Control Byte
* @Address : Address
* @data : Data
* @Length : Length
* 
* Returns -.
* 
* Description: Function to read from I2C.
***********************************************************************/
unsigned int LDByteReadI2C(unsigned char ControlByte, unsigned char Address, unsigned char *data, unsigned char Length)
{
	write(fd_i2c, &Address, 1);
	read(fd_i2c, data, Length);
	return;
}

/***********************************************************************
* LDByteWriteI2C - Function to write to I2C.
* @ControlByte: Control Byte
* @Address : Address
* @data : Data
* 
* Returns -.
* 
* Description: Function to write to I2C.
***********************************************************************/
unsigned int LDByteWriteI2C(unsigned char ControlByte, unsigned char Address, unsigned char data)
{
	unsigned char writeByte[2];
	writeByte[0] = Address;
	writeByte[1] = data;
    
	write(fd_i2c, &writeByte, sizeof(writeByte));
	return;
}

/***********************************************************************
* gpio_export - Function to export gpio pins.
* @gpio: GPIO PIN Number
*
* Returns 0 on success.
* 
* Description: Function to export gpio pins.
***********************************************************************/
int gpio_export(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];
 
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
 
	return 0;
}

/***********************************************************************
* gpio_export - Function to unexport gpio pins.
* @gpio: GPIO PIN Number
*
* Returns 0 on success.
* 
* Description: Function to unexport gpio pins.
***********************************************************************/
int gpio_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];
 
	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/unexport");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

/***********************************************************************
* gpio_set_dir - Function to set directions for gpio pins.
* @gpio: GPIO PIN Number
* @out_flag: Directions of GPIO PIN
*
* Returns 0 on success.
* 
* Description: Function to set directions for gpio pins.
***********************************************************************/
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
	int fd, len;
	char buf[MAX_BUF];
 
	len = snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/direction");
		return fd;
	}
 
	if (out_flag == GPIO_DIRECTION_OUT)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);
 
	close(fd);
	return 0;
}

/***********************************************************************
* gpio_set_value - Function to set value for gpio pins.
* @gpio: GPIO PIN Number
* @value: value of GPIO PIN
*
* Returns 0 on success.
* 
* Description: Function to set value for gpio pins.
***********************************************************************/
int gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/set-value");
		return fd;
	}

	if(value == GPIO_VALUE_HIGH)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);

	close(fd);
	return 0;
}

/***********************************************************************
* gpio_fd_open - Function to create file descriptor for the gpio.
* @gpio: GPIO PIN Number
*
* Returns fd on success.
* 
* Description: Function to create file descriptor for the gpio.
***********************************************************************/
int gpio_fd_open(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		perror("gpio/fd_open");
	}
	return fd;
}

/***********************************************************************
 * rdtsc() function is used to calulcate the number of clock ticks
 * and measure the time. TSC(time stamp counter) is incremented 
 * every cpu tick (1/CPU_HZ).
 **********************************************************************/
static inline u64 rdtsc(void)
{
	if (sizeof(long) == sizeof(u64))
	{
		u32 lo, hi;
		asm volatile("rdtsc" : "=a" (lo), "=d" (hi));
		return ((u64)(hi) << 32) | lo;
	}
	else
	{
		u64 tsc;
		asm volatile("rdtsc" : "=A" (tsc));
		return tsc;
	}
}

/***********************************************************************
* gpio_set_edge - Function to set edge for gpio pins.
* @gpio: GPIO PIN Number
* @edge: edge of GPIO PIN
*
* Returns 0 on success.
* 
* Description: Function to set edge for gpio pins.
***********************************************************************/
int gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/edge", gpio);
 
	fd = open(buf, O_WRONLY);
	if(fd < 0)
	{
		perror("gpio/set-edge");
		return fd;
	}
 
	write(fd, edge, strlen(edge)+1);
	close(fd);
	return 0;
}
