#include <cmath>
#include <BNO055-BBB_driver.h>
#include <ncurses.h>

using namespace std;


int main()
{
   WINDOW *win;
   char buff[80];
	int cont = 0, tipo, prnt;
	bool flag;
	double mG;
	char filename[] = "/dev/i2c-1";
	BNO055 sensors;

   sensors.openDevice(filename);
	win = initscr();
	cbreak();
   clearok (win, TRUE);
	keypad (win, TRUE);
	noecho ();
	halfdelay (1);
	nodelay (win, 1);
	do
	{
		sensors.readCalibVals();
		snprintf(buff, 79, "Sys:%d, CalMag:%d, CalGyro:%d, CalAcc:%d", (int)sensors.calSys, (int)sensors.calMag, (int)sensors.calGyro, (int)sensors.calAcc);
      mvwaddstr(win, LINES-1, 0, buff);
      wrefresh (win);
		usleep(50000);
		flag = sensors.calSys == 3 && sensors.calMag == 3 && sensors.calGyro == 3 && sensors.calAcc == 3; 
	} 
   while (cont++ < 2000 && !flag );

   cont = 0;
   do
   {

      sensors.readOrientation_Q();
      prnt = snprintf(buff, 79, "Q={%07.3lf, [%07.5lf, %07.5lf, %07.3lf]}",  sensors.qOrientation.vi[0] * sensors.Scale, sensors.qOrientation.vi[1] * sensors.Scale, sensors.qOrientation.vi[2] * sensors.Scale,sensors.qOrientation.vi[3] * sensors.Scale);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 1, 0, buff);

      sensors.readOrientation_E();
      prnt = snprintf(buff, 79, "EULER=[%07.5lf, %07.5lf, %07.3lf]",  sensors.eOrientation.vi[0] * sensors.Scale, sensors.eOrientation.vi[1] * sensors.Scale, sensors.eOrientation.vi[2] * sensors.Scale);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 3, 0, buff);

      sensors.readLinearAccVector();
      prnt = snprintf(buff, 79, "lin_Acc=[%07.3lf, %07.3lf, %07.3lf]",  sensors.accelVect.vi[0] * sensors.Scale, sensors.accelVect.vi[1] * sensors.Scale, sensors.accelVect.vi[2] * sensors.Scale);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 5, 0, buff);

      sensors.readGravityVector();
      mG = sqrt( (double)(sensors.gravVect.vi[0]) * double(sensors.gravVect.vi[0]) +  (double)(sensors.gravVect.vi[1]) * (double)(sensors.gravVect.vi[1]) +  double(sensors.gravVect.vi[2]) * (double)(sensors.gravVect.vi[2]));
      prnt = snprintf(buff, 79, "G=[%07.3lf, %07.3lf, %07.3lf] , |G| = %07.3lf",  (double)sensors.gravVect.vi[0] * 0.01, (double)sensors.gravVect.vi[1] * 0.01, (double)sensors.gravVect.vi[2] * 0.01, mG*0.01);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 7, 0, buff);

      sensors.readGyroVector();
      prnt = snprintf(buff, 79, "Gyro=[%07.3lf, %07.3lf, %07.3lf]",  (double)sensors.gyroVect.vi[0] * 0.01, (double)sensors.gyroVect.vi[1] * 0.01, (double)sensors.gyroVect.vi[2] * 0.01 );
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 9, 0, buff);

      wrefresh (win);

      tipo = wgetch (win);
		if (tipo != ERR)
		{
		   if (tipo == 27)
		      break;
      }
      
      cont++;
   } while (true);
   endwin();
}
