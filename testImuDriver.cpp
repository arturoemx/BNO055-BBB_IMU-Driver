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
	BNO055 sensors(filename);

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
      mvwaddstr(win, COLS-1, 0, buff);
      wrefresh (win);
		usleep(50000);
		flag = sensors.calSys == 3 && sensors.calMag == 3 && sensors.calGyro == 3 && sensors.calAcc == 3; 
	} 
   while (cont++ < 2000 && !flag );

   cont = 0;
   do
   {

      sensors.readOrientation_Q();
      prnt = snprintf(buff, 79, "Q={%07.3lf, [%07.5lf, %07.5lf, %07.3lf]}",  sensors.w * sensors.Scale, sensors.x * sensors.Scale, sensors.y * sensors.Scale,sensors.z * sensors.Scale);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 1, 0, buff);
      wrefresh (win);

      sensors.readLinearAcc();
      prnt = snprintf(buff, 79, "lin_Acc=[%07.3lf, %07.3lf, %07.3lf]",  sensors.laX * sensors.Scale, sensors.laY * sensors.Scale, sensors.laZ * sensors.Scale);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 3, 0, buff);
      wrefresh (win);

      sensors.readGravityVector();
      mG = sqrt( sensors.gX * sensors.gX +  sensors.gY * sensors.gY +  sensors.gZ * sensors.gZ);
      prnt = snprintf(buff, 79, "G=[%07.3lf, %07.3lf, %07.3lf] , |G| = %07.3lf",  (double)sensors.gX * 0.01, (double)sensors.gY * 0.01, (double)sensors.gZ * 0.01, mG*0.01);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 5, 0, buff);
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
