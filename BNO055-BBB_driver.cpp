/*  Programa que realiza la inicialización de un IMU BNO055
 *  y realiza lecturas de cuaterniones. 
 *
 *  Dr. Arturo Espinosa Romero, Ing. Alex Antonio Turriza Suárez, 2019
 */

#include <cstdlib>
#include <iostream>
#include <BNO055-BBB_driver.h>
#include <bitset>

using namespace std;

	BNO055::BNO055()
	{
	   initMembers();
		memset(data, 0, 16*sizeof(unsigned char));
      file = -1;
	}


	BNO055::BNO055(char *filename)
	{
	   openDevice(filename);
	}

	BNO055::~BNO055()
	{
		close(file);
	}

	void BNO055::openDevice(char *filename)
   {
		if((file = open(filename, O_RDWR)) < 0)
		{
			perror("BNO055: failed to open i2c bus");
			exit(-2);
		}
		start();
		memset(data, 0, 16*sizeof(unsigned char));
   }
	void BNO055::initMembers ()
	{
	   memset(qOrientation.vi, 0, 4 * sizeof(int16_t));
	   memset(eOrientation.vi, 0, 3 * sizeof(int16_t));
	   memset(gyroVect.vi, 0, 3 * sizeof(int16_t));
	   memset(accelVect.vi, 0, 3 * sizeof(int16_t));
	   memset(gravVect.vi, 0, 3 * sizeof(int16_t));
		calGyro = calMag = calAcc = calSys = 0;
	}

	void BNO055::start(unsigned char quatadd, operationMode opMode)
	{
	   initMembers();
		imuAddress = quatadd;
		setAddress(imuAddress);

		/*Se envían parámetros de inicialización*/
		//Modo de configuración
		data[0] = BNO055_OPR_MODE_ADD;
		data[1] = OPERATION_MODE_CONFIG;
		writeData(2);
#ifdef __VERBOSE__
      cout << "Se envían parámetros de inicialización" << endl;
      cout.flush();
#endif

		//Explícitamente iniciamos en modo normal de uso
		data[0] = BNO055_PWR_MODE_ADD;
		data[1] = POWER_MODE_NORMAL;
		writeData(2);
#ifdef __VERBOSE__
      cout << "Iniciamos en el modo normal de uso, en forma explicita." << endl;
      cout.flush();
#endif

		//Colocamos la página 0 del mapa de registros
		data[0] = BNO055_PAGE_ID_ADDR;
		data[1] = PAGE0;
		writeData(2);
#ifdef __VERBOSE__
      cout << "Se coloca la pagina 0 del amap de registros." << endl;
      cout.flush();
#endif

		//Reiniciamos
		data[0] = BNO055_SYS_TRIGGER_ADD;
		data[1] = RESET;
		writeData(2);
#ifdef __VERBOSE__
      cout << "Se reinicia el sensor." << endl;
      cout.flush();
#endif
		usleep(500000);

		//Fijamos las unidades de los sensores
		data[0] = BNO055_UNIT_SEL_ADDR;
		data[1] = UNIDADES_DEFAULT;
		writeData(2);
#ifdef __VERBOSE__
      cout << "Se definen las unidades de los sensores." << endl;
      cout.flush();
#endif

		//Reiniciamos el sensor
		data[0] = BNO055_SYS_TRIGGER_ADD;
		data[1] = 0x00;
		writeData(2);
#ifdef __VERBOSE__
      cout << "Se reinicia el sensor." << endl;
      cout.flush();
#endif

		//Modo de operación a utilizar
		data[0] = BNO055_OPR_MODE_ADD;
		data[1] = opMode;
		writeData(2);
#ifdef __VERBOSE__
      cout << "Se define el modo de operacion a utilizar." << endl;
      cout.flush();
#endif

		usleep(20000);
#ifdef __VERBOSE__
      cout << "Termino la inicialización." << endl << endl;
      cout.flush();
#endif
	}

	void BNO055::setAddress(unsigned char address)
	{
		if(ioctl(file, I2C_SLAVE, address) < 0)
		{
			cout << "Failed to acquire bus access and/or talk to slave." << endl;
			cout.flush();
			exit(-1);
		}
	}

	void BNO055::writeData(int n)
	{
		int wval;
		wval = write(file, data, n);
		if(wval != n)
			perror("Error al escribir al bus i2c :");
		usleep(5000);
	}


	void BNO055::readCalibVals()
	{
		int rval;
		if(ioctl(file, I2C_SLAVE, BNO055_ADDRESS) < 0)
		{
			cerr << "Calib: Failed to acquire bus access and/or talk to slave" << endl;
			exit(-1);
		}
		data[0] = BNO055_CALIB_STAT_ADD;
		writeData(1);
		rval = read(file, data, 8);
		if(rval < 0)
		   perror("readCalibVals: Failed to acquire bus access and/or talk to slave:");
		calGyro = int8_t ((data[0] >> 4) & 0x03);
		calSys = int8_t ((data[0] >> 6) & 0x03);
		calAcc = int8_t ((data[0] >> 2) & 0x03);
		calMag = int8_t ((data[0]) & 0x03);
	}

   void BNO055::readVector(int address, int n, u_int8_t *v)
	{
		int cont, rval;
		cont = 0;
		if(ioctl(file, I2C_SLAVE, BNO055_ADDRESS) < 0)
		{
			cerr << "readVector: Failed to acquire bus access and/or talk to slave." << endl;
			exit(-1);
		}
		data[0] = address;
		writeData(1);
		do
        	{
            		rval = read(file, v + cont, n);
            		if (rval < 0)
                  /* ERROR HANDLING: i2c transaction failed */
		               perror("readVector - Failed to read to the i2c bus: ");
            		else
                		cont += rval;
        	} 
		while (cont < n);
	}

	void BNO055::readAll()
	{
	   u_int8_t info[32];

		readVector(BNO055_GYRO_ADD, 32, info);
		memcpy(gyroVect.vc, info, 6 * sizeof( u_int8_t));
		memcpy(eOrientation.vc, info+6, 6 * sizeof( u_int8_t));
		memcpy(qOrientation.vc, info+12, 8 * sizeof( u_int8_t));
		memcpy(accelVect.vc, info+20, 6 * sizeof( u_int8_t));
		memcpy(gravVect.vc, info+26, 6 * sizeof( u_int8_t));
	}

	void BNO055::readOrientation_E()
	{
		readVector(BNO055_EULERDATA_ADD, 6, eOrientation.vc);
	}

	void BNO055::readOrientation_Q()
	{
		readVector(BNO055_QUATDATA_ADD, 8, qOrientation.vc);
	}

	void BNO055::readGyroVector()
	{
		readVector(BNO055_GYRO_ADD, 6, gyroVect.vc);
	}

	void BNO055::readLinearAccVector()
	{
		readVector (BNO055_LINACC_ADD, 6, accelVect.vc);
	}
	
	void BNO055::readGravityVector()
	{
		readVector (BNO055_GRAVITY_ADD, 6, gravVect.vc);
	}
