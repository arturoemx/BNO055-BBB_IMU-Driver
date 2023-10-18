#ifndef __BNO055_DRIVER__
#define  __BNO055_DRIVER__

/*  Programa que realiza la inicialización de un IMU BNO055
 *  y realiza lecturas de cuaterniones. 
 *
 *  Dr. Arturo Espinosa Romero, Ing. Alex Antonio Turriza Suárez, 2019
 */

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

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
#define BNO055_EULERDATA_ADD 0x1A // Heading[LSB,MSB], Roll[LSB,MSB], Pitch[LSB,MSB], [0x1A -> 0x1F]
#define BNO055_QUATDATA_ADD 0x20 // W[LSB,MSB], X[LSB,MSB], Y[LSB,MSB], Z[LSB,MSB] [0x20 -> 0x27]
#define BNO055_LINACC_ADD 0x28 // X[LSB,MSB], Y[LSB,MSB], Z[LSB,MSB], [0x28 -> 0x2D]
#define BNO055_GRAVITY_ADD 0x2E // gX[LSB,MSB], gY[LSB,MSB], gZ[LSB,MSB], [0x2E -> 0x33]
#define BNO055_GYRO_ADD 0x14 // gyX[LSB,MSB], gyY[LSB,MSB], gyZ[LSB,MSB], [0x14 -> 0x19]

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

/**
\enum operationMode
\brief This type defines an enumeration that describes all the possible configuration modes of the Bosch BNO055 IMU Sensor.
**/
enum operationMode: int8_t
{
   CONFIG=0,
   ACCONLY=1,
   MAGONLY=2,
   GIRONLY=3,
   ACCMAG=4,
   ACCGYRO=5,
   MAGGYRO=6,
   AMG=7,
   IMU=8,
   COMPASS=9,
   M4G=10,
   NDOF_FMC_OFF=11,
   NDOF=12
};

/**
/union Vec3
\brief This union object allows us to access a three 16-bit integer vector as a six 8 bit unsigned integer vector.  
**/
union Vec3
{
   u_int8_t vc[6];
   int16_t vi[3];
};

/**
/union Vec4
\brief This union object allows us to access a four 16-bit integer vector as a eight 8 bit unsigned integer vector.  
**/
union Vec4
{
   u_int8_t vc[8];
   int16_t vi[4];
};

/**
\struct BNO055
\brief This structure provides an interface to control and use the BNO055 IMU sensor. It contains attributes that store the measurements of the orientation, rotational velocity, translational acceleration and magnetometer measurements.
**/
struct BNO055
{
	int file; //!< Descriptor used to access the i2c serial interface.
	unsigned char data[16]; //!< array used to store information sent and received from the BNO055.
	char _buffer[8]; //!< string buffer needed to store error feedback info.
	unsigned char imuAddress; //!< I2C IMU Address.
	Vec4 qOrientation; //!<  Array used to store the IMU orientation as a quaternion.
	Vec3 eOrientation; //!< Array used to store the three eulerian angles that define de 3D IMU orientation.
	Vec3 gyroVect; //!< Array used to store the 3D rotational velocity gyro-measurements.
	Vec3 accelVect; //!< Array used to store de 3D translational acceleration measurements.
	Vec3 gravVect; //!< Array used to store the 3D Gravity vector.
	int8_t calGyro;//!< Gyro Calibration status value.
	int8_t calMag;//!< Magnetometer Calibration status value. 
	int8_t calAcc;//!< Accelerometer Calibration status value. 
	int8_t calSys;//!< Calibration status value. 
	const double Scale = (1.0 / (1 << 14)); //!< Scale Factor.

   /**
   \func BNO055()
   \brief Class constructor.
   **/
	BNO055();

   /**
   \func BNO055(char *filename)
   \brief Class constructor.
   \param char *filename The name of the I2C device used to access the BNO055.
   **/
	BNO055(char *filename);

   /**
   \func ~BNO055()
   \brief Object constructor.
   **/
	~BNO055();

   /**
   \func void openDevice(char *filename)
   \brief Open the I2C device used to access the BNO055, and 'starts' the device(see method start).
   **/
	void openDevice(char *filename);

   /**
   \func void initMembers ()
   \brief Clean up (sets to 0) the object attributes that store the BNO055 measurements, and calibration status. 
   **/
   void initMembers ();

   /**
   \func void setAddress(unsigned char address)
   \brief Aquire buss access to talk to the BNO055.
   \param unsigned char address The I2C device address.
   **/
	void setAddress(unsigned char address);

   /**
   \func void writeData(int n)
   \brief Writes the first n bytes stored in the object's data attribute into the BNO055 device
   \param int n The number of bytes to be written.
   **/
	void writeData(int n);

   /**
   \func void start(unsigned char quatadd = BNO055_ADDRESS, operationMode opMode=NDOF)
   \brief Starts the BNO055. That means following the procedure to set the operation mode, sensors units, mainly.
   \param unsigned char quatadd  The I2C BNOO55 address. It defaults to the value BNO055 defined in this file.
   \param operationMode opMode The BNO055 operation mode set. It defaults to the NDOF.
   **/
	void start(unsigned char quatadd = BNO055_ADDRESS, operationMode opMode=NDOF);

   /**
   \func void readVector(int address, int n, u_int8_t *v)
   \brief Reads n bytes into de the array pointed by 'v' from the device whose address is 'address'. 
   \param int address The I2C address from where the bytes will be read.
   \param int n The number of bytes that will be read.
   \param u_int8_t *v A pointer to the array where the read bytes will be stored.
   **/
   void readVector(int address, int n, u_int8_t *v);

   /**
   \func void readCalibVals();
   \brief Reads the calibration status values from the BNO055 into the corresponding object's attributes.
   **/
	void readCalibVals();

   void BNO055::readAll();

   /**
   \func void readOrientation_Q();
   \brief Reads the quaternion vector from the BNO055 into the corresponding object's attribute.
   **/
	void readOrientation_Q();

   /**
   \func void readOrientation_E()
   \brief Reads the Euler angles from the BNO055 into the corresponding object's attributes.
   **/
	void readOrientation_E();

   /**
   \func void readGyroVector()
   \brief Reads the rotational velocity measurements from the BNO055 into the corresponding object's attributes.
   **/
	void readGyroVector();

   /**
   \func void readLinearAccVector()
   \brief Reads the BNO055 translational acceleration measurements with the gravity vector substracted into the corresponding object's attributes.
   **/
	void readLinearAccVector();

   /**
   \func void readGravityVector()
   \brief Reads the BNO055 gravity vector estimation into the corresponding object's attribute.
   **/
	void readGravityVector();
};
#endif

