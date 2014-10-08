#define WHI 6
#define RED 5
#define YEL 4
#define GRN 3
#define BLU 2
#define BLK 1

#define scanMoveFull(speed) RotateMotor(OUT_BC, speed, 270);
#define scanMoveHalf(speed) RotateMotor(OUT_BC, speed, 140);
#define adjustR(angle) RotateMotorEx(OUT_BC, 20, angle, -100, true, false);
#define adjustL(angle) RotateMotorEx(OUT_BC, 20, angle, 100, true, false);

/**
* Use while robot is still moving. Samples an area of color 128
* times, returning the color that occured the most.
*/
inline int getColor(){
  int i;
  int w = 0, r = 0, y = 0, g = 0, b = 0, k = 0;
  for (i = 0; i < 72; i++){
    int color = Sensor(IN_1);
    if (color == WHI) w++;
    else if (color == RED) r++;
    else if (color == YEL) y++;
    else if (color == GRN) g++;
    else if (color == BLU) b++;
    else if (color == BLK) k++;
    Wait(2);
  }
  int max = 0;
  int color = 0;
  if (k > max){
    max = k;
    color = BLK;
  }
  if (b > max){
    max = b;
    color = BLU;
  }
  if (g > max){
    max = g;
    color = GRN;
  }
  if (y > max){
    max = y;
    color = YEL;
  }
  if (r > max){
    max = r;
    color = RED;
  }
  if (w > max) color = WHI;
  return color;
}

/**
* Moves the robot at specified speed untill a color has changed.
* return: the changed color's value.
*/
int moveUntilColorChange(int speed){
  int startColor = getColor(); // Starting color
  int color;
  OnFwdSync(OUT_BC, speed, 0);
  while (true){
    color = getColor();
    if (startColor != color){
      // Stop motor, break loop, and return color.
      Off(OUT_BC);
      return color;
    }
  }
}

/**
* Assumes the bot is already on a building. Will only register once a color
* change is detected.
* When crossRoad is true, black or yellow must be crossed in order for a building
* to be registered. 
*/
int moveUntilBuilding(int speed, int currentColor, bool crossRoad){
  OnFwdSync(OUT_BC, speed, 0);
  int color;
  bool colorChanged = false; // Stays false untill a color change is detected.
  while (true){
    color = getColor();
    // Condition given to start recognizing building colors. 
    if (color != currentColor && (!crossRoad || (color == YEL || color == BLK))) colorChanged = true;
    if (colorChanged && ((color == GRN) || (color == BLU) || (color == WHI))){
      // If color changed from currentColor, and a new building color has been registered,
      // wait a while for the bot to overshoot into the color block and return the reading.
      Wait(300);
      Off(OUT_BC);
      return getColor();
    }
  }
}

/**
* Moves forward untill a yellow is hit.
*/
void moveUntilColor(int speed, int c){
  OnFwdSync(OUT_BC, speed, 0);
  int color;
  while (true){
    color = getColor();
    if (color == c){
      Off(OUT_BC);
      return;
    }
  }
}

/**
* Goes in reverse for the specified number of ticks.
*/
inline void revFor(int speed, long ticks){
  long t0 = CurrentTick();
  OnFwdSync(OUT_BC, -speed, 0);
  until( (CurrentTick() - t0) > ticks);
  Off(OUT_BC);
}

inline void right90(){
  RotateMotorEx(OUT_BC, 25, 270, -100, true, false);
  RotateMotor(OUT_BC, 20, 60);
}

inline void right180(){
  RotateMotorEx(OUT_BC, 25, 530, -100, true, false);
  RotateMotor(OUT_BC, 20, 120);
}

inline void left90(){
  RotateMotorEx(OUT_BC, 25, 270, 100, true, false);
  RotateMotor(OUT_BC, 20, 60);
}

inline void adjustR45(){
  RotateMotorEx(OUT_BC, 25, 140, -100, true, false);
}

inline void adjustL45(){
  RotateMotorEx(OUT_BC, 25, 140, 100, true, false);
}

void scanIntersection(){
  int pinpointF = 92; // Some angle movement values used to pinpoint the bot's
  int pinpointB = 94; // orientation on the yellow square.

  // Move 45 degreese first to attempt to allign to NE building.
  adjustR45();
  RotateMotor(OUT_BC, 20, pinpointF);
  int currentColor = getColor();
  bool adjusted = false;

  // Attempt to fix error if a black was read.
  while (currentColor == BLK){
    adjusted = true;
    TextOut(8, 8, NumToStr(currentColor));
    RotateMotor(OUT_BC, -20, pinpointB);
    adjustR(30);
    RotateMotor(OUT_BC, 20, pinpointF);;
    currentColor = getColor();
  }
  if (adjusted){
    RotateMotor(OUT_BC, -20, pinpointB);
    adjustR(30);
    currentColor = moveUntilBuilding(25, YEL, false);
  } else {
    currentColor = moveUntilBuilding(25, YEL, false);
  }
  TextOut(8, 8, NumToStr(currentColor));
  Wait(1000);
  currentColor = moveUntilBuilding(-25, currentColor, true);
  TextOut(8, 8, NumToStr(currentColor));
  Wait(1000);
  moveUntilColor(25, YEL);
  adjustL45();
  adjustL45();
  currentColor = moveUntilBuilding(25, YEL, false);
  TextOut(8, 8, NumToStr(currentColor));
  Wait(1000);
  currentColor = moveUntilBuilding(-25, currentColor, true);
  TextOut(8, 8, NumToStr(currentColor));
  Wait(1000);
  moveUntilColor(25, YEL);
  adjustR45();
}

void moveToNextIntersection(){
  long t0, t1;
  bool d0, d1;
  long sTime;

  t0 = 0;
  d0 = false;

  // Assume we start on black.
  while(true){
    sTime = CurrentTick();
    int curColor = moveUntilColorChange(30);
    t1 = CurrentTick() - sTime;
    if (curColor == YEL){
      return;
    }
    else if (curColor == RED){
      right180();
    }
    else if (curColor != BLK){
      if (t1 > 1200) moveUntilColor(-30, BLK);
      else revFor(30, t1 * 0.75);
      if (t1 >= t0) d1 = d0; // Previous turn was a good decision. Fix using same turn.
      else d1 = !d0; // Previous turn was not a good decision. Fix by turning oposite way.
      if (d1) {adjustR(20);}
      else {adjustL(20);}
    }
    // Shift t1, d1 to t0, d0

    t0 = t1;
    d0 = d1;
  }
}

task main(){
  SetSensorColorFull(IN_1);
  while(true){
    while(true){
      moveToNextIntersection();
      if (getColor() == YEL) break; // Double check for yellow.
    }
    scanIntersection();
    right90();
  }
}