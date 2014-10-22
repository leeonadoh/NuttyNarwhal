/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 5 m/s at touchdown
	  * Maximum landing angle should be less than 10 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- Ignore all code related to the graphics, you don't
	  need to look at it for your work, but you
	  can study it afterwards if you are curious.

	- These are the 'sensors' you have available to control
          the lander. You do not have to look at the functions
          that implement them, and you ARE NOT ALLOWED to change
          the way these functions work.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by Paconetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!

	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thurster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  Credits: Lander image and rocky texture provided by NASA
*/

// Simulation parameters - YOU MUST NOT CHANGE ANY OF THESE
#define G_ACCEL 8.87
#define MT_ACCEL 35.0
#define RT_ACCEL 25.0
#define LT_ACCEL 25.0
#define MAX_ROT_RATE .075
#define SONAR_RANGE 9.0
#define NP1 .05
#define NP2 .05
#define T_STEP .005
#define S_SCALE 5.0
#define PI 3.14159265359
#define DISPLAY_LATENCY 10
#define HIST 180

/*
  Standard C libraries
*/
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<string.h>
#include<time.h>

/*
   Headers for OpenGL libraries. If you want to run this
   on your computer, make sure you have installed OpenGL,
   GLUT, and GLUI
*/
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

// Global variables accessible to your flight computer
int MT_OK;
int RT_OK;
int LT_OK;
double PLAT_X;
double PLAT_Y;
double SONAR_DIST[36];

// OpenGL global data - YOU MUST NOT CHANGE THIS!
int FAIL_MODE;
int MKmode,kbuf[6];
double s_sec;
double s_sec2;
int windowID;               // Glut window ID (for display)
int Win[2];                 // window (x,y) size
int F_LIST[10];
int F_comp[10];
double Xhist[HIST];
double Yhist[HIST];
double dXhist[HIST];
double dYhist[HIST];
double Thist[HIST];
double *rst, *pst;
int *fst;
unsigned char *map, *map_b, *varis; // image data
unsigned char *lander, *tmp;
char line[1024];

// Function prototypes for code you need to look at
void Lander_Control(void);
void Safety_Override(void);

// Function prototypes for code you don't need to look at
// (including OpenGL stuff and utilities)
int main(int argc, char *argv[]);
void load_map(const char *name);
void load_lander(void);
int render_frame(double *st, double *parm, int *flg, double *s_dir, double *s_dst);
void plotHist(int x, int y, int R, int G, int B, unsigned char *map);
double state_update(double *st, double *parm, int *flg, double *s_dir, double *s_dst);
unsigned char *readPPMimage(const char *filename);
void imageOutput(unsigned char *im, const char *filename);
void kbHandler(unsigned char key, int x, int y);
void kbUpHandler(unsigned char key, int x, int y);
void initGlut(char* winName);
void GL_Settings_Init();
void WindowReshape(int w, int h);
void WindowDisplay(void);
void DoNothin(clock_t delay);
void Main_Thruster(double power);
void Left_Thruster(double power);
void Right_Thruster(double power);
void Rotate(double angle);
double Velocity_X(void);
double Velocity_Y(void);
double Position_X(void);
double Position_Y(void);
double Angle(void);
double RangeDist(void);

inline double normalizeAngle(double angle);
inline void turnToAngle(double angle);
int angleWithinRange(double angle, double range);

/***************************************************
 LANDER CONTROL CODE BEGINS HERE
***************************************************/

void Lander_Control(void){
  /*
    This is the main control function for the lander. It attempts
    to bring the ship to the location of the landing platform
    keeping landing parameters within the acceptable limits.

    How it works:

    - First, if the lander is rotated away from zero-degree angle,
      rotate lander back onto zero degrees.
    - Determine the horizontal distance between the lander and
      the platform, fire horizontal thrusters appropriately
      to change the horizontal velocity so as to decrease this
     distance
    - Determine the vertical distance to landing platform, and
      allow the lander to descend while keeping the vertical
      speed within acceptable bounds. Make sure that the lander
      will not hit the ground before it is over the platform!

    As noted above, this function assumes everything is working
    fine.
  */

  /*************************************************
    TO DO: Modify this function so that the ship safely
           reaches the platform even if components and
           sensors fail!

           Note that sensors are noisy, even when
           working properly.

           Finally, YOU SHOULD provide your own
           functions to provide sensor readings,
           these functions should work even when the
           sensors are faulty.

           For example: Write a function Velocity_X_robust()
           which returns the module's horizontal velocity.
           It should determine whether the velocity
           sensor readings are accurate, and if not,
           use some alternate method to determine the
           horizontal velocity of the lander.

           NOTE: Your robust sensor functions can only
           use the available sensor functions and control
           functions!
   	DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
           ACCESS THE SIMULATION STATE. That's cheating,
           I'll give you zero.
  **************************************************/

  double VXlim;
  double VYlim;

  // Set velocity limits depending on distance to platform.
  // If the module is far from the platform allow it to
  // move faster, decrease speed limits as the module
  // approaches landing. You may need to be more conservative
  // with velocity limits when things fail.
  if (fabs(Position_X()-PLAT_X)>200) VXlim=25;
  else if (fabs(Position_X()-PLAT_X)>100) VXlim=15;
  else VXlim=5;

  if (PLAT_Y-Position_Y()>200) VYlim=-20;
  else if (PLAT_Y-Position_Y()>100) VYlim=-10;  // These are negative because they
  else VYlim=-4;				       // limit descent velocity

  // Ensure we will be OVER the platform when we land
  if (fabs(PLAT_X-Position_X())/fabs(Velocity_X())>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y())) 
    VYlim=0;

  // IMPORTANT NOTE: The code below assumes all components working
  // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
  // fail. More likely, you will need a set of case-based code
  // chunks, each of which works under particular failure conditions.

  // Check for rotation away from zero degrees - Rotate first,
  // use thrusters only when not rotating to avoid adding
  // velocity components along the rotation directions
  // Note that only the latest Rotate() command has any
  // effect, i.e. the rotation angle does not accumulate
  // for successive calls.

  if (Angle()>1&&Angle()<359){
    if (Angle()>=180) 
      Rotate(360-Angle());
    else 
      Rotate(-Angle());
    return;
  }

  // Module is oriented properly, check for horizontal position
  // and set thrusters appropriately.
  if (Position_X()>PLAT_X){
    // Lander is to the LEFT of the landing platform, use Right thrusters to move
    // lander to the left.
    Left_Thruster(0);	// Make sure we're not fighting ourselves here!
    if (Velocity_X()>(-VXlim)) 
      Right_Thruster((VXlim+fmin(0,Velocity_X()))/VXlim);
    else{
      // Exceeded velocity limit, brake
      Right_Thruster(0);
      Left_Thruster(fabs(VXlim-Velocity_X()));
    }
  }
  else{
    // Lander is to the RIGHT of the landing platform, opposite from above
    Right_Thruster(0);
    if (Velocity_X()<VXlim) 
      Left_Thruster((VXlim-fmax(0,Velocity_X()))/VXlim);
    else{
      Left_Thruster(0);
      Right_Thruster(fabs(VXlim-Velocity_X()));
    }
  }

  // Vertical adjustments. Basically, keep the module below the limit for
  // vertical velocity and allow for continuous descent. We trust
  // Safety_Override() to save us from crashing with the ground.
  if (Velocity_Y()<VYlim) Main_Thruster(1.0);
  else Main_Thruster(0);
}

void Safety_Override(void)
{
  /*
    This function is intended to keep the lander from
    crashing. It checks the sonar distance array,
    if the distance to nearby solid surfaces and
    uses thrusters to maintain a safe distance from
    the ground unless the ground happens to be the
    landing platform.

    Additionally, it enforces a maximum speed limit
    which when breached triggers an emergency brake
    operation.
  */

  /**************************************************
   TO DO: Modify this function so that it can do its
          work even if components or sensors
          fail
  **************************************************/

  /**************************************************
    How this works:
    Check the sonar readings, for each sonar
    reading that is below a minimum safety threshold
    AND in the general direction of motion AND
    not corresponding to the landing platform,
    carry out speed corrections using the thrusters
  **************************************************/

  double DistLimit;
  double Vmag;
  double dmin;

  // Establish distance threshold based on lander
  // speed (we need more time to rectify direction
  // at high speed)
  Vmag=Velocity_X()*Velocity_X();
  Vmag+=Velocity_Y()*Velocity_Y();

  DistLimit=fmax(75,Vmag);

  // If we're close to the landing platform, disable
  // safety override (close to the landing platform
  // the Control_Policy() should be trusted to
  // safely land the craft)
  if (fabs(PLAT_X-Position_X())<150&&fabs(PLAT_Y-Position_Y())<150) return;

  // Determine the closest surfaces in the direction
  // of motion. This is done by checking the sonar
  // array in the quadrant corresponding to the
  // ship's motion direction to find the entry
  // with the smallest registered distance

  // Horizontal direction.
  dmin=1000000;
  if (Velocity_X()>0){
    for (int i=5;i<14;i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  }
  else{
    for (int i=22;i<32;i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  }
  // Determine whether we're too close for comfort. There is a reason
  // to have this distance limit modulated by horizontal speed...
  // what is it?
  if (dmin<DistLimit*fmax(.25,fmin(fabs(Velocity_X())/5.0,1)))
  { // Too close to a surface in the horizontal direction
    if (Angle()>1&&Angle()<359){
      if (Angle()>=180) Rotate(360-Angle());
      else Rotate(-Angle());
      return;
    }
    if (Velocity_X()>0){
      Right_Thruster(1.0);
      Left_Thruster(0.0);
    }
    else{
      Left_Thruster(1.0);
      Right_Thruster(0.0);
    }
  }

  // Vertical direction
  dmin=1000000;
  if (Velocity_Y()>5){      // Mind this! there is a reason for it...
    for (int i=0; i<5; i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
    for (int i=32; i<36; i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  }
  else{
    for (int i=14; i<22; i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  }
  if (dmin<DistLimit){  // Too close to a surface in the horizontal direction
    if (Angle()>1||Angle()>359){
      if (Angle()>=180) Rotate(360-Angle());
      else Rotate(-Angle());
      return;
    }
    if (Velocity_Y()>2.0){
      Main_Thruster(0.0);
    }
    else{
      Main_Thruster(1.0);
    }
  }
}

inline double normalizeAngle(double angle){
  if (angle > 360)
    return angle - 360;
  else if (angle < 0)
    return angle + 360;
  return angle;
}

inline void turnToAngle(double angle){
  double dAngle = angle - Angle();
  if (dAngle > 180) dAngle -= 360;
  else if (dAngle < -180) dAngle += 360;
  Rotate(dAngle);
}

/**
* Return true if angle sensor returns angle within specified range.
* False otherwise. 
* Require 0 <= angle < 360
*         0 <= range < 180
*/
int angleWithinRange(double angle, double range){
  double start1, end1, start2, end2;
  if (angle - range < 0){
    start2 = angle - range + 360;
    end2 = 360;
    start1 = 0;
    end1 = angle + range;
  } else if (angle + range > 360) {
    start2 = 0;
    end2 = angle + range - 360;
    start1 = angle - range;
    end1 = 360;
  } else {
    start1 = angle - range;
    end1 = angle + range;
  }
  double readAngle = Angle();
  if ((readAngle >= start1 && readAngle <= end1) ||
      (readAngle >= start2 && readAngle <= end2)) {
    return 1;
  }
  return 0;
}


/**************************************************
 LANDER CONTROL CODE ENDS HERE.
 YOU CAN IGNORE ALL CODE BELOW THIS POINT.

 DO NOT SPEND TIME ON THE CODE BELOW HERE UNLESS
 YOU:

 a) Have completed your task and have implemented
    control software that works in the face of
    failures.
 b) Have too much time on your hand and want to
    see how I built this thing...

 Evidently, you should only spend time on the code
 below this point if (a AND b)
**************************************************/

/**************************************************
 Setup, image management, and OpenGL stuff
 DO NOT alter any of the code below!
 You do not need to change the code below to
 solve this problem. However, if you're curious
 you may learn a few tricks...
***************************************************/
int main(int argc, char *argv[])
{
 /*
   Main function. Read command line parameters and
   set up simulation parameters accordingly
 */
 char name[1024];
 int tpx;

 srand48(time(NULL));
 memset(&Xhist[0],0,HIST*sizeof(double));
 memset(&Yhist[0],0,HIST*sizeof(double));
 memset(&dXhist[0],0,HIST*sizeof(double));
 memset(&dYhist[0],0,HIST*sizeof(double));
 memset(&Thist[0],0,HIST*sizeof(double));

 if (argc<3)
 {
  fprintf(stderr,"Usage: Lander_Control MapName FailMode [component1] [component2] ... [component n]\n");
  fprintf(stderr,"See header of Lander_Control.cpp for details\n");
  exit(1);
 }

 for (int i=0;i<10;i++){F_LIST[i]=1;F_comp[i]=1;}
 s_sec=-1;
 s_sec2=-1;
 FAIL_MODE=atoi(argv[2]);
 if (FAIL_MODE==3)
 {
  for (int i=3; i<argc; i++)
   if (atoi(argv[i])>0&&atoi(argv[i])<10) F_comp[atoi(argv[i])]=0;
  s_sec=.5;
 }
 else if (FAIL_MODE==1||FAIL_MODE==2)
 {
  s_sec=4*drand48();
  s_sec2=8*drand48();
 }
 else FAIL_MODE=0;

 map=readPPMimage(argv[1]);
 if (map==NULL)
 {
  fprintf(stderr,"Unable to open map image %s, please check name and path\n",argv[1]);
  exit(1);
 }
 map_b=(unsigned char *)calloc(1024*1024*3,sizeof(unsigned char));
 lander=(unsigned char *)calloc(64*64*3*36,sizeof(unsigned char));
 if (!map_b||!lander)
 {
  fprintf(stderr,"Unable to allocate image data\n");
  free(map);
 }
 memcpy(map_b,map,1024*1024*3*sizeof(unsigned char));

 for (int i=0; i<36; i++)
 {
  sprintf(&name[0],"lander_%03d.ppm",i*10);
  tmp=readPPMimage(name);
  if (tmp==NULL)
  {
   fprintf(stderr,"Unable to load lander images. Ensure they are in the same directory\n");
   free(map);
   free(map_b);
   free(lander);
   exit(1);
  }
  memcpy(lander+(64*64*3*i),tmp,64*64*3*sizeof(unsigned char));
  free(tmp);
 }

 varis=readPPMimage("varis.ppm");
 if (varis==NULL)
 {
  fprintf(stderr,"Unable to load variable labels.\n");
  free(map);
  free(map_b);
  free(lander);
  exit(0);
 }

 tpx=0;
 PLAT_X=0;
 PLAT_Y=0;
 for (int i=0; i<1024; i++)
  for (int j=0; j<1024; j++)
  {
   if (*(map+((i+(j*1024))*3)+0)>250&&*(map+((i+(j*1024))*3)+1)<10&&*(map+((i+(j*1024))*3)+2)<10)
   {
    PLAT_X+=i;
    PLAT_Y+=j;
    tpx++;
   }
  }
 PLAT_X/=tpx;
 PLAT_Y/=tpx;

 // Initialize glut, glui, and opengl
 Win[0]=700;
 Win[1]=700;
 glutInit(&argc, argv);
 initGlut(argv[0]);

 MKmode=0;
 memset(&kbuf[0],0,6*sizeof(int));
 glutMainLoop();
 exit(0);
}

double state_update(double *st, double *parm, int *flg, double *s_dir, double *s_dst)
{
 /*
   Lander state update. Accounts for all forces acting on the lander,
   commands from the control software, and so on.
 */

 static double SimTime=0;
 static double PingTime=0;
 double dice;
 double hang;

 if (SimTime==0)
 {
  // Init
  *(st+0)=50+(drand48()*925);
  *(st+1)=50+(drand48()*50);
  *(st+2)=(drand48()*25)-12.5;
  *(st+3)=-(drand48()*15);
  *(st+4)=(drand48()*2.0*PI);
  *(st+5)=0;
  *(st+6)=0;
  memset(parm,0,10*sizeof(double));
  for (int i=0; i<10; i++) *(flg+i)=F_LIST[i];
  for (int i=0; i<36; i++)
  {
   *(s_dir+i)=1;
   *(s_dst+i)=15;
   SONAR_DIST[i]=-1;
  }
  MT_OK=*(flg+1);
  LT_OK=*(flg+2);
  RT_OK=*(flg+3);
 }

 if (*(parm+9)!=0)
 {
  if (*(parm+9)>0)
  {
   *(st+4)+=fmin(*(parm+9),MAX_ROT_RATE);
   *(parm+9)-=fmin(*(parm+9),MAX_ROT_RATE);
  }
  else
  {
   *(st+4)-=fmin(fabs(*(parm+9)),MAX_ROT_RATE);
   *(parm+9)+=fmin(fabs(*(parm+9)),MAX_ROT_RATE);
  }
 }
 if (*(st+4)<0) *(st+4)=(2.0*PI)+(*(st+4));
 *(st+4)=fmod(*(st+4),2.0*PI);

 *(st+6)=-G_ACCEL;
 *(st+5)=0;
 if (*(parm+1)>0&&*(flg+1)==1)
 {
  *(st+6)+=cos(*(st+4))*MT_ACCEL*(*(parm+1));
  *(st+5)+=sin(*(st+4))*MT_ACCEL*(*(parm+1));
 }
 if (*(parm+2)>0&&*(flg+2)==1)
 {
  hang=-PI+(*(st+4));   		// -PI to PI
  *(st+6)+=sin(hang)*LT_ACCEL*(*(parm+2));
  *(st+5)-=cos(hang)*LT_ACCEL*(*(parm+2));
 }
 if (*(parm+3)>0&&*(flg+3)==1)
 {
  hang=2.0*PI-(*(st+4));               // 0 to 2PI
  *(st+6)-=sin(hang)*RT_ACCEL*(*(parm+3));
  *(st+5)-=cos(hang)*RT_ACCEL*(*(parm+3));
 }

 *(st+2)+=T_STEP*(*(st+5));
 *(st+3)+=T_STEP*(*(st+6));
 *(st+0)+=S_SCALE*T_STEP*(*(st+2));
 *(st+1)-=S_SCALE*T_STEP*(*(st+3));

 for (int i=0; i<36; i++){*(s_dst+i)+=(*(s_dir+i))*SONAR_RANGE; if (*(s_dst+i)<0) *(s_dst+i)=0;}

 SimTime+=T_STEP;
 PingTime+=T_STEP;

 if (PingTime>.25)
 {
  PingTime=0;
  for (int i=0; i<36; i++)
  {
   if (*(s_dir+i)==1) SONAR_DIST[i]=-1;
   *(s_dir+i)=1;
   *(s_dst+i)=15;
  }
 }

 // Check for failures
 if (FAIL_MODE==1||FAIL_MODE==2||FAIL_MODE==3)
 {
  dice=drand48();
  if ((s_sec>0&&s_sec<SimTime)||(s_sec2>0&&s_sec2<SimTime))
  {
   if (FAIL_MODE==1)
   {
    if (dice<.5) {*(flg+1)=0; MT_OK=0; fprintf(stderr,"Main Thruster malfunction!\n");}
    else if (dice<.75) {*(flg+2)=0; LT_OK=0; fprintf(stderr,"Left Thruster malfunction!\n");}
    else {*(flg+3)=0; RT_OK=0; fprintf(stderr,"Right Thruster malfunction!\n");}
   }
   else if (FAIL_MODE==3)
   {
    for (int i=0; i<10; i++)
    {
     *(flg+i)=F_comp[i];
     if (F_comp[i]==0) fprintf(stderr,"Failing component %d\n",i);
    }
    if (F_comp[1]==0) MT_OK=0;
    if (F_comp[2]==0) LT_OK=0;
    if (F_comp[3]==0) RT_OK=0;
   }
   else
   {
    if (dice==1) dice=.999;
    *(flg+1+(int)(dice*8))=0;
    switch ((int)((dice*8)+1))
    {
     case 1:
      fprintf(stderr,"Main Thruster malfunction\n");
      MT_OK=0;
      break;
     case 2:
      fprintf(stderr,"Left Thruster malfunction\n");
      LT_OK=0;
      break;
     case 3:
      fprintf(stderr,"Right Thruster malfunction\n");
      RT_OK=0;
      break;
     case 4:
      fprintf(stderr,"Horizontal Velocity sensor malfunction\n");
      break;
     case 5:
      fprintf(stderr,"Vertical Velocity sensor malfunction\n");
      break;
     case 6:
      fprintf(stderr,"Horizontal Position sensor malfunction\n");
      break;
     case 7:
      fprintf(stderr,"Vertical Position sensor malfunction\n");
      break;
     case 8:
      fprintf(stderr,"Angle sensor malfunction\n");
      break;
     case 9:
      fprintf(stderr,"Sonar malfunction\n");
      break;
     default:
      fprintf(stderr,"Something just went wrong!\n");
      break;
    }
   }
   if (s_sec>0) s_sec=-1; else s_sec2=-1;
  }
 }

 return(SimTime*100);
}

void plotHist(int x, int y, int R, int G, int B, unsigned char *map, double *hist, double ref)
{
 /*
   Make a simple plot of the historical data for some variable in the simulation
   starting at (x,y) with the specified colour.
 */

 int i,j,xc,yc;
 double col,cR,cG,cB;
 double ydisp;
 int widthScale=1;
 int heightScale=30;

 for (i=0;i<HIST*widthScale;i++)
 {
  *(map+((x+i+((y-heightScale)*1024))*3)+0)=127;
  *(map+((x+i+((y-heightScale)*1024))*3)+1)=127;
  *(map+((x+i+((y-heightScale)*1024))*3)+2)=127;
  *(map+((x+i+((y+heightScale)*1024))*3)+0)=127;
  *(map+((x+i+((y+heightScale)*1024))*3)+1)=127;
  *(map+((x+i+((y+heightScale)*1024))*3)+2)=127;
  *(map+((x+i+(y*1024))*3)+0)=127;
  *(map+((x+i+(y*1024))*3)+1)=0;
  *(map+((x+i+(y*1024))*3)+2)=0;
 }
 for (i=-heightScale;i<=heightScale;i++)
 {
  *(map+((x+(HIST*widthScale)+((y+i)*1024))*3)+0)=127;
  *(map+((x+(HIST*widthScale)+((y+i)*1024))*3)+1)=127;
  *(map+((x+(HIST*widthScale)+((y+i)*1024))*3)+2)=127;
  *(map+((x+((y+i)*1024))*3)+0)=127;
  *(map+((x+((y+i)*1024))*3)+1)=127;
  *(map+((x+((y+i)*1024))*3)+2)=127;
 }

 for (i=0; i<HIST; i++)
 {
  ydisp=((*(hist+i))-ref);
  if (ydisp<-.99) ydisp=-.99;
  else if (ydisp>.99) ydisp=.99;

  yc=y+(int)(-ydisp*heightScale);
  for (j=0; j<widthScale; j++)
  {
   xc=x+(i*widthScale)+j;
   if (yc>=0&&yc<1024&&xc>=0&&xc<1024)
   {
    *(map+((xc+(yc*1024))*3)+0)=(unsigned char)R;
    *(map+((xc+(yc*1024))*3)+1)=(unsigned char)G;
    *(map+((xc+(yc*1024))*3)+2)=(unsigned char)B;
   }
  }
 }

}

int render_frame(double *st, double *parm, int *flg, double *s_dir, double *s_dst)
{
 /*
   Update the image for display
 */
 unsigned char *lp;
 int lnd;
 int xp,yp,xp2,yp2,len;
 double vx,vy;
 int xc,yc;
 int col,bnc,Elvis;
 int nCC;

 memcpy(map_b,map,1024*1024*3*sizeof(unsigned char));
 lnd=round((*(st+4))*36/(2*PI));
 if (lnd>35||lnd<0) lnd=0;
 lp=(lander+(64*64*3*lnd));

 // Update history data
 for (int i=0; i<HIST-1; i++)
 {
  Xhist[i]=Xhist[i+1];
  Yhist[i]=Yhist[i+1];
  dXhist[i]=dXhist[i+1];
  dYhist[i]=dYhist[i+1];
  Thist[i]=Thist[i+1];
 }
 Xhist[HIST-1]=Position_X()/512.0;
 Yhist[HIST-1]=Position_Y()/512.0;
 dXhist[HIST-1]=Velocity_X()/25.0;
 dYhist[HIST-1]=Velocity_Y()/25.0;
 Thist[HIST-1]=(Angle()/360.0)-.5;

 // Collision check
 Elvis=1;
 col=0;
 nCC=0;
 for (int i=0; i<64; i++)
  for (int j=0; j<64; j++)
  {
   xp=((int)*(st+0))+i-32;
   yp=((int)*(st+1))+j-32;
   if (xp>=0&&yp>=0&&xp<1024&&yp<1024)
   {
    Elvis=0;
    if (*(lp+((i+(64*j))*3)+0)>0)
    {
     if (*(map+((xp+(yp*1024))*3)+0)==255&&*(map+((xp+(yp*1024))*3)+1)==0&&*(map+((xp+(yp*1024))*3)+2)==0)
     {
      if (((fabs(*(st+4))<15.0*(PI/180))||(*(st+4)>345*(PI/180)))&&fabs(*(st+3))<10.0) col=2; else nCC++;
     }
     else if (*(map+((xp+(yp*1024))*3)+0)>0) nCC++;
    }
   }
  }
 if (nCC>10) col=1;
 if (Elvis) col=3;

 // Render effects below the lander
 // Sonar ping / echo

 // Render range finder line
 vy=cos(*(rst+4));
 vx=-sin(*(rst+4));
 for (int i=19; i<1024; i++)
 {
  xp=round((*(rst+0))+(vx*i));
  yp=round((*(rst+1))+(vy*i));
  if (xp>=0&&xp<1024&&yp>=0&&yp<1024)
  {
   if (*(map+((xp+(yp*1024))*3)+0)>5) break;
   *(map_b+((xp+(yp*1024))*3)+0)=(unsigned char)255;
  }
 }

 if (*(flg+9))
 {
  for (int i=0;i<36; i+=1)
  {
   xp=((int)*(st+0));
   yp=((int)*(st+1));
   vy=-cos(i*20.0*PI/360.0);
   vx=sin(i*20.0*PI/360.0);
   xp=round(xp+((*(s_dst+i))*vx));
   yp=round(yp+((*(s_dst+i))*vy));
   vx=-vy;
   vy=sin(i*20.0*PI/360.0);
   bnc=0;
   for (int j=1; j<*(s_dst+i)/10.0; j++)
   {
    xc=round(xp+(j*vx));
    yc=round(yp+(j*vy));
    if (xc>=0&&xc<1024&&yc>=0&&yc<1024)
    {
     if (*(map+((xc+(yc*1024))*3)+0)!=0||*(map+((xc+(yc*1024))*3)+1)!=0||*(map+((xc+(yc*1024))*3)+2)!=0) bnc=1;
     *(map_b+((xc+(yc*1024))*3)+0)=(unsigned char)(0.0);
     *(map_b+((xc+(yc*1024))*3)+1)=(unsigned char)(255.0-fmin(255.0,(*(s_dst+i))));
     *(map_b+((xc+(yc*1024))*3)+2)=(unsigned char)(255.0-fmin(255.0,(*(s_dst+i))));
    }
    xc=round(xp-(j*vx));
    yc=round(yp-(j*vy));
    if (xc>=0&&xc<1024&&yc>=0&&yc<1024)
    {
     if (*(map+((xc+(yc*1024))*3)+0)!=0||*(map+((xc+(yc*1024))*3)+1)!=0||*(map+((xc+(yc*1024))*3)+2)!=0) bnc=1;
     *(map_b+((xc+(yc*1024))*3)+0)=(unsigned char)(0.0);
     *(map_b+((xc+(yc*1024))*3)+1)=(unsigned char)(255.0-fmin(255.0,(*(s_dst+i))));
     *(map_b+((xc+(yc*1024))*3)+2)=(unsigned char)(255.0-fmin(255.0,(*(s_dst+i))));
    }
   }
   if (bnc==1)
   {
    if (*(s_dir+i)!=-1)
    {
     SONAR_DIST[i]=*(s_dst+i)+((drand48()*(*(s_dst+i)))-(*(s_dst+i)/2));
     (*(s_dir+i)=-1);
    }
   }
  }

 } // end if (*(flg+9))

 // Overlay variable names
 for (int i=0; i<770; i++)
  for (int j=0; j<23; j++)
  {
   xp=i+17;
   yp=j+7;
   *(map_b+((xp+(yp*1024))*3)+0)=*(varis+((i+(j*770))*3)+0);
   *(map_b+((xp+(yp*1024))*3)+1)=*(varis+((i+(j*770))*3)+1);
   *(map_b+((xp+(yp*1024))*3)+2)=*(varis+((i+(j*770))*3)+2);
  }

 // The fires of creation
 if (*(parm+1)>0&&(*(flg+1)))
 {
  for (double i=*(st+4)+PI-(PI/16.0);i<*(st+4)+PI+(PI/16.0);i+=.001)
  {
   xp=((int)*(st+0));
   yp=((int)*(st+1));
   vy=-cos(i);
   vx=sin(i);
   for (int j=15;j<15+(int)(drand48()*75*(*(parm+1)));j++)
   {
    xc=round(xp+(j*vx));
    yc=round(yp+(j*vy));
    if (xc>=0&&xc<1024&&yc>=0&&yc<1024)
    {
     *(map_b+((xc+(yc*1024))*3)+0)=(unsigned char)200+(unsigned char)(drand48()*50);
     *(map_b+((xc+(yc*1024))*3)+1)=(unsigned char)(drand48()*250);
     *(map_b+((xc+(yc*1024))*3)+2)=0;
    }
   }
  }
 }
 if (*(parm+2)>0&&(*(flg+2)))
 {
  for (double i=*(st+4)+(1.5*PI)-(PI/32.0);i<*(st+4)+(1.5*PI)+(PI/32.0);i+=.001)
  {
   xp=((int)*(st+0));
   yp=((int)*(st+1));
   vy=-cos(i);
   vx=sin(i);
   for (int j=20;j<20+(int)(drand48()*55*(*(parm+2)));j++)
   {
    xc=round(xp+(j*vx));
    yc=round(yp+(j*vy));
    if (xc>=0&&xc<1024&&yc>=0&&yc<1024)
    {
     *(map_b+((xc+(yc*1024))*3)+0)=(unsigned char)200+(unsigned char)(drand48()*50);
     *(map_b+((xc+(yc*1024))*3)+1)=(unsigned char)(drand48()*250);
     *(map_b+((xc+(yc*1024))*3)+2)=0;
    }
   }
  }
 }
 if (*(parm+3)>0&&(*(flg+3)))
 {
  for (double i=*(st+4)+(.5*PI)-(PI/32.0);i<*(st+4)+(.5*PI)+(PI/32.0);i+=.001)
  {
   xp=((int)*(st+0));
   yp=((int)*(st+1));
   vy=-cos(i);
   vx=sin(i);
   for (int j=20;j<20+(int)(drand48()*55*(*(parm+3)));j++)
   {
    xc=round(xp+(j*vx));
    yc=round(yp+(j*vy));
    if (xc>=0&&xc<1024&&yc>=0&&yc<1024)
    {
     *(map_b+((xc+(yc*1024))*3)+0)=(unsigned char)200+(unsigned char)(drand48()*50);
     *(map_b+((xc+(yc*1024))*3)+1)=(unsigned char)(drand48()*250);
     *(map_b+((xc+(yc*1024))*3)+2)=0;
    }
   }
  }
 }

 // Render the lander
 for (int i=0; i<64; i++)
  for (int j=0; j<64; j++)
  {
   xp=((int)*(st+0))+i-32;
   yp=((int)*(st+1))+j-32;
   if (xp>=0&&yp>=0&&xp<1024&&yp<1024)
   {
    if (*(lp+((i+(64*j))*3)+0)>5)
    {
     *(map_b+((xp+(yp*1024))*3)+0)=*(lp+((i+(j*64))*3)+0);
     *(map_b+((xp+(yp*1024))*3)+1)=*(lp+((i+(j*64))*3)+1);
     *(map_b+((xp+(yp*1024))*3)+2)=*(lp+((i+(j*64))*3)+2);
    }
   }
  }

  // Plot readouts from vehicle parameters
  plotHist(15,35,0,255,0,map_b,&Xhist[0],1.0);
  plotHist(200,35,0,255,255,map_b,&Yhist[0],1.0);
  plotHist(385,35,255,0,255,map_b,&dXhist[0],0.0);
  plotHist(570,35,255,255,0,map_b,&dYhist[0],0.0);
  plotHist(755,35,128,128,255,map_b,&Thist[0],0.0);

  return(col);
}

void Main_Thruster(double power)
{
 if (power<0) power=0;
 if (power>1) power=1;
 *(pst+1)=(.95*power)+(.05*drand48());
}

void Left_Thruster(double power)
{
 if (power<0) power=0;
 if (power>1) power=1;
 *(pst+2)=(.95*power)+(.05*drand48());
}

void Right_Thruster(double power)
{
 if (power<0) power=0;
 if (power>1) power=1;
 *(pst+3)=(.95*power)+(.05*drand48());
}

void Rotate(double angle)
{
 *(pst+9)=(((.95*angle)+(.05*drand48()))*(PI/180.0));
}

double Velocity_X(void)
{
 if (*(fst+4)) return(*(rst+2)+(NP1*(drand48()-.5)*(*(rst+2))));
 else return((50*drand48())-25);
}

double Velocity_Y(void)
{
 if (*(fst+5)) return(*(rst+3)+(NP1*(drand48()-.5)*(*(rst+3))));
 else return((50*drand48())-25);
}

double Position_X(void)
{
 if (*(fst+6)) return(*(rst+0)+((drand48()-.5)*NP1*(*(rst+0))));
 else return(1024*drand48());
}

double Position_Y(void)
{
 if (*(fst+7)) return(*(rst+1)+((drand48()-.5)*NP1*(*(rst+0))));
 else return(1024*drand48());
}

double Angle(void)
{
 if (*(fst+8)) return((*(rst+4)+((NP2*drand48())-(NP2/2.0)))*(180.0/PI));
 else return((*(rst+4)+((50*NP2*drand48())-(50*NP2/2.0)))*(180.0/PI));
}

double RangeDist(void)
{
 double vx,vy;
 int xp,yp;
 vy=cos(*(rst+4));
 vx=-sin(*(rst+4));

 for (int i=0; i<1024; i++)
 {
  xp=round((*(rst+0))+(vx*i));
  yp=round((*(rst+1))+(vy*i));
  if (xp>=0&&xp<1024&&yp>=0&&yp<1024)
  {
   if (*(map+((xp+(yp*1024))*3)+0)>5) return(i-19);
  }
 }
 return(-1);

}

unsigned char *readPPMimage(const char *filename)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # Optionally, one or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //
 // readPPMdata converts the image colour information to floating point. This is so that
 // the texture mapping function doesn't have to do the conversion every time
 // it is asked to return the colour at a specific location.
 //
 // On error, the function returns NULL
 //

 FILE *f;
 unsigned char *im;
 int sizx,sizy;

 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }

 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",&sizx,&sizy);           // Read file size

 fgets(&line[0],9,f);  	                // Read the remaining header line
 im=(unsigned char *)calloc(sizx*sizy*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }

 fread(im,sizx*sizy*3*sizeof(unsigned char),1,f);
 fclose(f);
 return(im);
}

// Initialize glut and create a window with the specified caption
void initGlut(char* winName)
{
    // Set video mode: double-buffered, color, depth-buffered
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    // Create window
    glutInitWindowPosition (5, 5);
    glutInitWindowSize(Win[0],Win[1]);
    windowID = glutCreateWindow(winName);

    // Setup callback functions to handle window-related events.
    // In particular, OpenGL has to be informed of which functions
    // to call when the image needs to be refreshed, and when the
    // image window is being resized.
    glutReshapeFunc(WindowReshape);   // Call WindowReshape whenever window resized
    glutDisplayFunc(WindowDisplay);   // Call WindowDisplay whenever new frame needed
    glutKeyboardFunc(kbHandler);
    glutKeyboardUpFunc(kbUpHandler);
}

void WindowReshape(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();			// Initialize with identity matrix
    gluOrtho2D(0, 800, 800, 0);
    glViewport(0,0,w,h);
    Win[0] = w;
    Win[1] = h;
}

void kbHandler(unsigned char key, int x, int y)
{
 MKmode=1;
 if (key=='a') {kbuf[0]=1; Left_Thruster(1.0);}
 if (key=='s') {kbuf[1]=1; Right_Thruster(1.0);}
 if (key=='k') {kbuf[2]=1; Rotate(-5.0);}
 if (key=='l') {kbuf[3]=1; Rotate(5.0);}
 if (key==' ') {kbuf[4]=1; Main_Thruster(1.0);}
 if (key=='z') kbuf[5]=1;
}

void kbUpHandler(unsigned char key, int x, int y)
{
 if (key=='a') {kbuf[0]=0; Left_Thruster(0.0);}
 if (key=='s') {kbuf[1]=0; Right_Thruster(0.0);}
 if (key=='k') {kbuf[2]=0; Rotate(0.0);}
 if (key=='l') {kbuf[3]=0; Rotate(0.0);}
 if (key==' ') {kbuf[4]=0; Main_Thruster(0.0);}
 if (key=='z') kbuf[5]=0;
 if (kbuf[0]==0&&kbuf[1]==0&&kbuf[2]==0&&kbuf[3]==0&&kbuf[4]==0&&kbuf[5]==0) MKmode=0;
}

void WindowDisplay(void)
{
  static int flg[10];
  static double parm[10];
  static double st[10];
  static double s_dir[36];
  static double s_dst[36];
  static int LStat=0;
  static int frameno=1;
  static int dcount=0;
  double stime;
  int xp,yp;
  unsigned char *tmp;
  GLuint texture;
  static int initF=1;

  rst=&st[0];
  fst=&flg[0];
  pst=&parm[0];

  // Main simulation loop while no collision, landing, or lander leaving the screen
   if (LStat==0)
   {
    stime=state_update(st,parm,flg,s_dir,s_dst);
    if (!MKmode) Lander_Control();
    Safety_Override();
    LStat=render_frame(st,parm,flg,s_dir,s_dst);
   }
   else if (LStat==1)		// Lander crashed
   {
    if (frameno>=50)
    {
     fprintf(stderr,"The Lander Has Crashed!\n");
     free(map);
     free(map_b);
     free(lander);
     free(varis);
     exit(0);
    }
    else
    {
     memcpy(map_b,map,1024*1024*3*sizeof(unsigned char));
     sprintf(&line[0],"toasted_%04d.ppm",frameno);
     tmp=readPPMimage(line);
     if (tmp!=NULL)
     {
      for (int i=0; i<64; i++)
       for (int j=0; j<64; j++)
       {
        xp=((int)*(st+0))+i-32;
        yp=((int)*(st+1))+j-32;
        if (xp>=0&&yp>=0&&xp<1024&&yp<1024)
        {
         if (*(tmp+((i+(64*j))*3)+0)>5&&*(map+((xp+(yp*1024))*3)+0)<5)
         {
          *(map_b+((xp+(yp*1024))*3)+0)=*(tmp+((i+(j*64))*3)+0);
          *(map_b+((xp+(yp*1024))*3)+1)=*(tmp+((i+(j*64))*3)+1);
          *(map_b+((xp+(yp*1024))*3)+2)=*(tmp+((i+(j*64))*3)+2);
         }
        }
       }
      free(tmp);
     }
     frameno++;
    }
   }
   else if (LStat==2)		// Landing occurred
   {
    if (frameno>=50)
    {
     fprintf(stderr,"We have landing!\n");
     free(map);
     free(map_b);
     free(lander);
     free(varis);
     exit(0);
    }
    else
    {
     memcpy(map_b,map,1024*1024*3*sizeof(unsigned char));
     for (int i=0; i<64; i++)
      for (int j=0; j<64; j++)
      {
       xp=((int)*(st+0))+i-32;
       yp=((int)*(st+1))+j-32;
       if (xp>=0&&yp>=0&&xp<1024&&yp<1024)
       {
        if (*(lander+((i+(64*j))*3)+0)>5&&*(map+((xp+(yp*1024))*3)+0)<5)
        {
         *(map_b+((xp+(yp*1024))*3)+0)=*(lander+((i+(j*64))*3)+0);
         *(map_b+((xp+(yp*1024))*3)+1)=*(lander+((i+(j*64))*3)+1);
         *(map_b+((xp+(yp*1024))*3)+2)=*(lander+((i+(j*64))*3)+2);
        }
       }
      }
     for (int i=0; i<1024; i++)
      for (int j=0; j<1024; j++)
      {
       xp=i;
       yp=j;
       if (*(map+((xp+(yp*1024))*3)+0)==255&&*(map+((xp+(yp*1024))*3)+1)==0&&*(map+((xp+(yp*1024))*3)+2)==0)
       {
        if (frameno%2)
        {
         *(map_b+((xp+(yp*1024))*3)+0)=0;
         *(map_b+((xp+(yp*1024))*3)+1)=55;
         *(map_b+((xp+(yp*1024))*3)+2)=255;
        }
        else
        {
         *(map_b+((xp+(yp*1024))*3)+0)=255;
         *(map_b+((xp+(yp*1024))*3)+1)=255;
         *(map_b+((xp+(yp*1024))*3)+2)=255;
        }
       }
      }
     frameno++;
    }
   }
   else			// Lander left the screen
   {
    fprintf(stderr,"Elvis has left the building!\n");
    free(map);
    free(map_b);
    free(lander);
    free(varis);
    exit(0);
   }

  // Clear the screen and depth buffers
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  if (initF)
  {
   glGenTextures( 1, &texture);
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
   glBindTexture( GL_TEXTURE_2D, texture);

   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

   glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

   glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, map_b); 
   initF=0;
  }
  else glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1024, 1024, GL_RGB, GL_UNSIGNED_BYTE, map_b);

  // Draw box bounding the viewing area
  glBegin (GL_QUADS);
  glTexCoord2f (0.0, 0.0);
  glVertex3f (0.0, 0.0, 0.0);
  glTexCoord2f (1.0, 0.0);
  glVertex3f (800.0, 0.0, 0.0);
  glTexCoord2f (1.0, 1.0);
  glVertex3f (800.0, 800.0, 0.0);
  glTexCoord2f (0.0, 1.0);
  glVertex3f (0.0, 800.0, 0.0);
  glEnd ();

  // Make sure all OpenGL commands are executed
  glFlush();
  // Swap buffers to enable smooth animation
  glutSwapBuffers();

  // Tell glut window to update ls itself
  glutSetWindow(windowID);
  glutPostRedisplay();
}

void DoNothin(clock_t delay)
{
 clock_t deadline;
 deadline=delay+clock();
 while(deadline>clock());
}
