#include <cmath>
#include <BNO055-BBB_driver.h>
#include <ncurses.h>
#include <sys/timeb.h>

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
	timeb timeL, timeS, timeE;
   long TimeQ, TimeE, TimeA, TimeG, TimeGy, TimeL=0;

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
      ftime(&timeS);
      sensors.readOrientation_Q();
      ftime(&timeE);
      TimeQ = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;
      timeL=timeS;

      ftime(&timeS);
      sensors.readOrientation_E();
      ftime(&timeE);
      TimeE = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;
      
      ftime(&timeS);
      sensors.readLinearAccVector();
      ftime(&timeE);
      TimeA = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;
      
      ftime(&timeS);
      sensors.readGravityVector();
      ftime(&timeE);
      TimeG = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;
      
      ftime(&timeS);
      sensors.readGyroVector();
      ftime(&timeE);
      TimeGy = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;

      prnt = snprintf(buff, 79, "Q={%07.3lf, [%07.5lf, %07.5lf, %07.3lf]} : %ld ms",  sensors.qOrientation.vi[0] * sensors.Scale, sensors.qOrientation.vi[1] * sensors.Scale, sensors.qOrientation.vi[2] * sensors.Scale,sensors.qOrientation.vi[3] * sensors.Scale, TimeQ);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 1, 0, buff);

      prnt = snprintf(buff, 79, "EULER=[%07.5lf, %07.5lf, %07.3lf] : %ld ms",  sensors.eOrientation.vi[0] * sensors.Scale, sensors.eOrientation.vi[1] * sensors.Scale, sensors.eOrientation.vi[2] * sensors.Scale, TimeE);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 3, 0, buff);


      prnt = snprintf(buff, 79, "lin_Acc=[%07.3lf, %07.3lf, %07.3lf] : %ld ms",  sensors.accelVect.vi[0] * sensors.Scale, sensors.accelVect.vi[1] * sensors.Scale, sensors.accelVect.vi[2] * sensors.Scale, TimeA);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 5, 0, buff);

      mG = sqrt( (double)(sensors.gravVect.vi[0]) * double(sensors.gravVect.vi[0]) +  (double)(sensors.gravVect.vi[1]) * (double)(sensors.gravVect.vi[1]) +  double(sensors.gravVect.vi[2]) * (double)(sensors.gravVect.vi[2]));
      prnt = snprintf(buff, 79, "G=[%07.3lf, %07.3lf, %07.3lf] , |G| = %07.3lf : %ld ms",  (double)sensors.gravVect.vi[0] * 0.01, (double)sensors.gravVect.vi[1] * 0.01, (double)sensors.gravVect.vi[2] * 0.01, mG*0.01, TimeG);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 7, 0, buff);

      prnt = snprintf(buff, 79, "Gyro=[%07.3lf, %07.3lf, %07.3lf] : %ld ms",  (double)sensors.gyroVect.vi[0] * 0.01, (double)sensors.gyroVect.vi[1] * 0.01, (double)sensors.gyroVect.vi[2] * 0.01, TimeGy);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 9, 0, buff);


      prnt = snprintf(buff, 79, "TimeSensing, TimeLoop = %ld ms, %ld ms", TimeQ + TimeE + TimeA + TimeG + TimeGy, TimeL);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 11, 0, buff);
      wrefresh (win);

      tipo = wgetch (win);
		if (tipo != ERR)
		{
		   if (tipo == 27)
		      break;
      }
      
      cont++;
      ftime(&timeE);
      TimeL = timeE.time * 1000 + timeE.millitm - timeL.time * 1000 - timeL.millitm;

   } while (true);
   endwin();
}
