/*  Programa que realiza la inicialización de un IMU BNO055
 *  y realiza lecturas de cuaterniones. 
 *
 *  Dr. Arturo Espinosa Romero, Ing. Alex Antonio Turriza Suárez, 2019
 */

#include <cstdio>
#include <cstdlib>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>

/*DIRECCIÓN DEL IMU*/
#define BNO055_ADDRESS 0x28

/*REGISTRO DE PÁGINA DE MAPA DE REGS*/
#define BNO055_PAGE_ID_ADDR 0x07
#define PAGE0 0x00
#define PAGE1 0x01

/*REGISTROS DE SELECCIÓN DE UNIDADES*/
#define BNO055_UNIT_SEL_ADDR 0x3B
#define BNO055_DATA_SELECT_ADDR 0x3C

/*UNIDADES A UTILIZAR*/
#define UNIDADES_DEFAULT 0x82 //1 <- Android orientation
			      //0 <- Reservado
			      //0 <- Reservado
			      //0 <- Grados celsius
			      //0 <- Reservado
			      //0 <- Grados
			      //1 <- Rps
			      //0 <- m/s² 

/*REGISTROS DE DATOS*/
#define BNO055_QUATDATA_ADD 0x20 // WLSB, WMSB, X, Y, Z [0x20 -> 0x27]

/*REGISTRO DE CALIBRACIÓN*/
#define BNO055_CALIB_STAT_ADD 0x35

/*REGISTROS DE OPERACIÓN*/
#define BNO055_OPR_MODE_ADD 0x3D
#define BNO055_PWR_MODE_ADD 0x3E
#define BNO055_SYS_TRIGGER_ADD 0x3D

/*OPERACIONES*/
#define RESET 0x20

/*MODOS DE OPERACIÓN*/
#define OPERATION_MODE_CONFIG 0x00
#define OPERATION_MODE_IMU 0x08
#define OPERATION_MODE_NDOF_FMC_OFF 0x0B
#define OPERATION_MODE_NDOF 0x0C

/*MODO DE ENERGÍA*/
#define POWER_MODE_NORMAL 0x00

using namespace std;

struct IMU
{
	int file; //Descriptor
	unsigned char data[16];
	char _buffer[8];
	unsigned char imuAddress;
	int16_t x, y, z, w;
	int8_t calGyro, calMag, calAcc, calSys;
	const double scale = (1.0 / (1 << 14));

	void setAddress(unsigned char address)
	{
		if(ioctl(file, I2C_SLAVE, address) < 0)
		{
			cout << "Failed to acquire bus access and/or talk to slave." << endl;
			cout.flush();
			exit(-1);
		}
	}

	void writeData(int n)
	{
		int wval;
		wval = write(file, data, n);
		if(wval != n)
		{
			cout << "Failed to write to i2c bus: " << wval << endl;
			strerror_r(errno, _buffer, 63);
			cout << _buffer << endl;
		}
		usleep(5000);
	}

	void start(unsigned char quatadd = BNO055_ADDRESS)
	{
		x = y = z = w = 0;
		calGyro = calMag = calAcc = calSys = 0;
		imuAddress = quatadd;
		setAddress(imuAddress);

		/*Se envían parámetros de inicialización*/
		//Modo de configuración
		data[0] = BNO055_OPR_MODE_ADD;
		data[1] = OPERATION_MODE_CONFIG;
		writeData(2);

		//Explícitamente iniciamos en modo normal de uso
		data[0] = BNO055_PWR_MODE_ADD;
		data[1] = POWER_MODE_NORMAL;
		writeData(2);

		//Colocamos la página 0 del mapa de registros
		data[0] = BNO055_PAGE_ID_ADDR;
		data[1] = PAGE0;
		writeData(2);

		//Reiniciamos
		data[0] = BNO055_SYS_TRIGGER_ADD;
		data[1] = RESET;
		writeData(2);
		usleep(500000);

		//Fijamos las unidades de los sensores
		data[0] = BNO055_UNIT_SEL_ADDR;
		data[1] = UNIDADES_DEFAULT;
		writeData(2);

		//Reiniciamos el sensor
		data[0] = BNO055_SYS_TRIGGER_ADD;
		data[1] = 0x00;
		writeData(2);

		//Modo de operación a utilizar
		data[0] = BNO055_OPR_MODE_ADD;
		data[1] = OPERATION_MODE_NDOF;
		writeData(2);

		usleep(20000);
	}

	IMU(char *filename)
	{
		if((file = open(filename, O_RDWR)) < 0)
		{
			perror("IMU: failed to open i2c bus");
			exit(-2);
		}
		start();
		memset(data, 0, 16*sizeof(unsigned char));
	}

	~IMU()
	{
		close(file);
	}

	void readCalibVals()
	{
		int rval;
		if(ioctl(file, I2C_SLAVE, BNO055_ADDRESS) < 0)
		{
			printf("Calib: Failed to acquire bus access and/or talk to slave");
			exit(-1);
		}
		data[0] = BNO055_CALIB_STAT_ADD;
		writeData(1);
		rval = read(file, data, 8);
		if(rval < 0)
		{
			printf("Calib: Failed to acquire bus access and/or talk to slave");
			strerror_r(errno, _buffer, 63);
			printf("%s\n\n", _buffer);
		}
		cout << "Raw data: " << data[0] << endl;
		calSys = int8_t ((data[0] >> 6) & 0x03);
		calGyro = int8_t ((data[0] >> 4) & 0x03);
		calAcc = int8_t ((data[0] >> 2) & 0x03);
		calMag = int8_t ((data[0]) & 0x03);
	}

	void readQuatVals()
	{
		int cont, rval;
		cont = 0;
		if(ioctl(file, I2C_SLAVE, BNO055_ADDRESS) < 0)
		{
			printf("Quat: Failed to acquire bus access and/or talk to slave.\n");
			exit(-1);
		}
		data[0] = BNO055_QUATDATA_ADD;
		writeData(1);
		do
        	{
            		rval = read(file,data+cont,8);
            		if (rval < 0)
            		{
                		/* ERROR HANDLING: i2c transaction failed */
                		printf("Quat: Failed to read to the i2c bus.\n");
                		strerror_r(errno, _buffer, 63);
                		printf("%s\n\n", _buffer);
            		}
            		else
                		cont += rval;
        	} 
		while (cont < 8);
		w = (int16_t) ((data[1] << 8) | data[0]); //W
		x = (int16_t) ((data[3] << 8) | data[2]); //X
		y = (int16_t) ((data[5] << 8) | data[4]); //Y
		z = (int16_t) ((data[7] << 8) | data[6]); //Z
	}
};

int main()
{
	int cont = 0;
	char filename[] = "/dev/i2c-1";
	IMU sensors(filename);
	do
	{
		/*sensors.readQuatVals();
		cout << "W: " << (double) sensors.w * sensors.scale << endl;
		cout << "X: " << (double) sensors.x * sensors.scale << endl;
		cout << "Y: " << (double) sensors.y * sensors.scale << endl;
		cout << "Z: " << (double) sensors.z * sensors.scale << endl;
		usleep (50000);
		cout << endl;*/
		sensors.readCalibVals();
		cout << "Sys: " << sensors.calSys << endl;
		cout << "Mag: " << sensors.calMag << endl;
		cout << "Gyro: " << sensors.calGyro << endl;
		cout << "Accel: " << sensors.calAcc << endl;
		usleep(50000);
		cout << endl;
	} 
	while (cont++ < 20000);
}
