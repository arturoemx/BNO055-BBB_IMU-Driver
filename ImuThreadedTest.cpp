#include <cmath>
#include <BNO055-BBB_driver.h>
#include <ncurses.h>
#include <pthread.h> 
#include <RingBuffer.h>
#include <sys/timeb.h>

using namespace std;

struct Vec3T
{
   Vec3 v;
   unsigned long time;
};

struct Vec4T
{
   Vec4 v;
   unsigned long time;
};

struct sensInfo4
{
   unsigned long time, meanTime, k;
   BNO055 *imu;
   RingBuffer <Vec4T> RB = RingBuffer<Vec4T>(40);
   bool Run;
   sensInfo4(BNO055 *ptr)
   {
      Run = true;
      time = meanTime = k = 0;
      imu = ptr;
   }
};

struct sensInfo3
{
   unsigned long time, meanTime, k;
   BNO055 *imu;
   RingBuffer <Vec3T> RB = RingBuffer<Vec3T>(40);
   bool Run;
   sensInfo3(BNO055 *ptr)
   {
      Run = true;
      time = meanTime = k = 0;
      imu = ptr;
   }
};

//**************
//Función que atiende a cada cliente
//**************
void *quaternionThread(void *cd)
{
   timeb timeS, timeE;
   unsigned long T, time0;
   sensInfo4 *si = (sensInfo4 *)cd;
   Vec4T q;

   ftime(&timeS);
   time0 = timeS.time * 1000 + timeS.millitm;
   while (si->Run)
   {
      ftime(&timeS);
      si->imu->readVector(BNO055_QUATDATA_ADD, 8, q.v.vc);
      ftime(&timeE);
      T = timeE.time * 1000 + timeE.millitm;
      q.time = T - time0;
      si->RB.Queue(q);

      T -= timeS.time * 1000 + timeS.millitm;
      si->meanTime = (si->k * si->meanTime + T);
      si->k++;
      si->meanTime /= si->k;
   }
   return 0;
}

void *eulerThread(void *cd)
{
   timeb timeS, timeE;
   unsigned long T, time0;
   sensInfo3 *si = (sensInfo3 *)cd;
   Vec3T e;

   ftime(&timeS);
   time0 = timeS.time * 1000 + timeS.millitm;
   while (si->Run)
   {
      ftime(&timeS);
      si->imu->readVector(BNO055_EULERDATA_ADD, 6, e.v.vc);
      ftime(&timeE);
      T = timeE.time * 1000 + timeE.millitm;
      e.time = T - time0;
      si->RB.Queue(e);

      T -= timeS.time * 1000 + timeS.millitm;
      si->meanTime = (si->k * si->meanTime + T);
      si->k++;
      si->meanTime /= si->k;
   }
   return 0;
}

void *linAccThread(void *cd)
{
   timeb timeS, timeE;
   unsigned long T, time0;
   sensInfo3 *si = (sensInfo3 *)cd;
   Vec3T a;

   ftime(&timeS);
   time0 = timeS.time * 1000 + timeS.millitm;
   while (si->Run)
   {
      ftime(&timeS);
      si->imu->readVector(BNO055_LINACC_ADD, 6, a.v.vc);
      ftime(&timeE);
      T = timeE.time * 1000 + timeE.millitm;
      a.time = T - time0;
      si->RB.Queue(a);

      T -= timeS.time * 1000 + timeS.millitm;
      si->meanTime = (si->k * si->meanTime + T);
      si->k++;
      si->meanTime /= si->k;
   }
   return 0;
}


void *gravityThread(void *cd)
{
   timeb timeS, timeE;
   unsigned long T, time0;
   sensInfo3 *si = (sensInfo3 *)cd;
   Vec3T g;

   ftime(&timeS);
   time0 = timeS.time * 1000 + timeS.millitm;
   while (si->Run)
   {
      ftime(&timeS);
      si->imu->readVector(BNO055_GRAVITY_ADD, 6, g.v.vc);
      ftime(&timeE);
      T = timeE.time * 1000 + timeE.millitm;
      g.time = T - time0;
      si->RB.Queue(g);

      T -= timeS.time * 1000 + timeS.millitm;
      si->meanTime = (si->k * si->meanTime + T);
      si->k++;
      si->meanTime /= si->k;
   }
   return 0;
}


void *gyroThread(void *cd)
{
   timeb timeS, timeE;
   unsigned long T, time0;
   sensInfo3 *si = (sensInfo3 *)cd;
   Vec3T gy;

   ftime(&timeS);
   time0 = timeS.time * 1000 + timeS.millitm;
   while (si->Run)
   {
      ftime(&timeS);
      si->imu->readVector(BNO055_GYRO_ADD, 6, gy.v.vc);
      ftime(&timeE);
      T = timeE.time * 1000 + timeE.millitm;
      gy.time = T - time0;
      si->RB.Queue(gy);

      T -= timeS.time * 1000 + timeS.millitm;
      si->meanTime = (si->k * si->meanTime + T);
      si->k++;
      si->meanTime /= si->k;
   }
   return 0;
}

int main()
{
   WINDOW *win;
   char buff[80];
	int cont = 0, tipo, prnt;
	bool flag;
	double mG;
	char filename[] = "/dev/i2c-1";
	BNO055 imu;
	timeb timeL, timeS, timeE;
   long TimeQ, TimeE, TimeA, TimeG, TimeGy, TimeL=0;

   imu.openDevice(filename);
	win = initscr();
	cbreak();
   clearok (win, TRUE);
	keypad (win, TRUE);
	noecho ();
	halfdelay (1);
	nodelay (win, 1);
	do
	{
		imu.readCalibVals();
		snprintf(buff, 79, "Sys:%d, CalMag:%d, CalGyro:%d, CalAcc:%d", (int)imu.calSys, (int)imu.calMag, (int)imu.calGyro, (int)imu.calAcc);
      mvwaddstr(win, LINES-1, 0, buff);
      wrefresh (win);
		usleep(50000);
		flag = imu.calSys == 3 && imu.calMag == 3 && imu.calGyro == 3 && imu.calAcc == 3; 
	} 
   while (cont++ < 2000 && !flag );

   cont = 0;
   do
   {
      ftime(&timeS);
      imu.readOrientation_Q();
      ftime(&timeE);
      TimeQ = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;
      timeL=timeS;

      ftime(&timeS);
      imu.readOrientation_E();
      ftime(&timeE);
      TimeE = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;
      
      ftime(&timeS);
      imu.readLinearAccVector();
      ftime(&timeE);
      TimeA = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;
      
      ftime(&timeS);
      imu.readGravityVector();
      ftime(&timeE);
      TimeG = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;
      
      ftime(&timeS);
      imu.readGyroVector();
      ftime(&timeE);
      TimeGy = timeE.time * 1000 + timeE.millitm - timeS.time * 1000 - timeS.millitm;

      prnt = snprintf(buff, 79, "Q={%07.3lf, [%07.5lf, %07.5lf, %07.3lf]} : %ld ms",  imu.qOrientation.vi[0] * imu.Scale, imu.qOrientation.vi[1] * imu.Scale, imu.qOrientation.vi[2] * imu.Scale,imu.qOrientation.vi[3] * imu.Scale, TimeQ);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 1, 0, buff);

      prnt = snprintf(buff, 79, "EULER=[%07.5lf, %07.5lf, %07.3lf] : %ld ms",  imu.eOrientation.vi[0] * imu.Scale, imu.eOrientation.vi[1] * imu.Scale, imu.eOrientation.vi[2] * imu.Scale, TimeE);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 3, 0, buff);


      prnt = snprintf(buff, 79, "lin_Acc=[%07.3lf, %07.3lf, %07.3lf] : %ld ms",  imu.accelVect.vi[0] * imu.Scale, imu.accelVect.vi[1] * imu.Scale, imu.accelVect.vi[2] * imu.Scale, TimeA);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 5, 0, buff);

      mG = sqrt( (double)(imu.gravVect.vi[0]) * double(imu.gravVect.vi[0]) +  (double)(imu.gravVect.vi[1]) * (double)(imu.gravVect.vi[1]) +  double(imu.gravVect.vi[2]) * (double)(imu.gravVect.vi[2]));
      prnt = snprintf(buff, 79, "G=[%07.3lf, %07.3lf, %07.3lf] , |G| = %07.3lf : %ld ms",  (double)imu.gravVect.vi[0] * 0.01, (double)imu.gravVect.vi[1] * 0.01, (double)imu.gravVect.vi[2] * 0.01, mG*0.01, TimeG);
      if (prnt < 79)
         memset ( buff+prnt, ' ', 79-prnt);
      buff[79] = 0;
      mvwaddstr(win, 7, 0, buff);

      prnt = snprintf(buff, 79, "Gyro=[%07.3lf, %07.3lf, %07.3lf] : %ld ms",  (double)imu.gyroVect.vi[0] * 0.01, (double)imu.gyroVect.vi[1] * 0.01, (double)imu.gyroVect.vi[2] * 0.01, TimeGy);
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
