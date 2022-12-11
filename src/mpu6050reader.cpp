#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>


#define MPU6050_I2C_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define SMPRT_DIV   0x19
#define INT_ENABLE   0x38
#define CONFIG 0x1A

#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

#define REG_FIFO_EN 0x23
#define REG_USER_CTRL 0x6A
#define REG_FIFO_COUNT_L 0x72
#define REG_FIFO_COUNT_H 0x73
#define REG_FIFO 0x74
#define REG_WHO_AM_I 0x75

int fd; 

void MPU6050_Init()
{
	
	wiringPiI2CWriteReg8 (fd, SMPRT_DIV, 0x07);	/* Write to sample rate divider register */
	wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
	wiringPiI2CWriteReg8 (fd, CONFIG, 0);		/* Write to Configuration register */
	wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
	wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register */

} 

short read_raw_data(int addr_h, int addr_l)
{
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(fd, addr_h);
	low_byte = wiringPiI2CReadReg8(fd, addr_l);
	value = (high_byte << 8) | low_byte;
	return value;
}

void ms_delay(int val){
	int i,j;
	for(i=0;i<=val;i++)
		for(j=0;j<1200;j++);
}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "odometry_node");
    ros::NodeHandle node_obj; 
	ros::Rate loop_rate(10);



	float Acc_x,Acc_y,Acc_z;
	float Gyro_x,Gyro_y,Gyro_z;
	float Ax=0, Ay=0, Az=0;
	float Gx=0, Gy=0, Gz=0;
	fd = wiringPiI2CSetup(MPU6050_I2C_ADDR);   /*Initializes I2C with device Address*/
	MPU6050_Init();		                 /* Initializes MPU6050 */
	
	while(ros::ok)
	{
		/*Read raw value of Accelerometer and gyroscope from MPU6050*/
		Acc_x = read_raw_data(ACCEL_XOUT_H, ACCEL_XOUT_L);
		Acc_y = read_raw_data(ACCEL_YOUT_H, ACCEL_YOUT_L);
		Acc_z = read_raw_data(ACCEL_ZOUT_H, ACCEL_ZOUT_L);
		
		Gyro_x = read_raw_data(GYRO_XOUT_H, GYRO_XOUT_L);
		Gyro_y = read_raw_data(GYRO_YOUT_H, GYRO_YOUT_L);
		Gyro_z = read_raw_data(GYRO_ZOUT_H, GYRO_ZOUT_L);
		
		/* Divide raw value by sensitivity scale factor */
		Ax = Acc_x/16384.0;
		Ay = Acc_y/16384.0;
		Az = Acc_z/16384.0;
		
		Gx = Gyro_x/131;
		Gy = Gyro_y/131;
		Gz = Gyro_z/131;
		
		ROS_INFO("\n Gx=%.3f °/s\tGy=%.3f °/s\tGz=%.3f °/s\tAx=%.3f g\tAy=%.3f g\tAz=%.3f g\n",Gx,Gy,Gz,Ax,Ay,Az);
		ros::spinOnce();
		loop_rate.sleep(); 
	}
	return 0;
}