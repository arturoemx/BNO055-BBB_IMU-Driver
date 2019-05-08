#include <BNO055-BBB_driver.h>
#include <iostream>

using namespace std;

int main()
{
	int cont = 0;
	char filename[] = "/dev/i2c-1";
	BNO055 sensors(filename);
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
		cout << "Sys: " << (int)sensors.calSys << endl;
		cout << "Mag: " << (int)sensors.calMag << endl;
		cout << "Gyro: " << (int)sensors.calGyro << endl;
		cout << "Accel: " << (int)sensors.calAcc << endl;
		usleep(50000);
		cout << endl;
	} 
   while (cont++ < 2000);
}
