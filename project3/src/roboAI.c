/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  Version: 0.2 - Updated Oct 2, 2014 - F. Estrada
***************************************************************************/

#include "imagecapture/imageCapture.h"
#include "roboAI.h"			// <--- Look at this header file!
#include <nxtlibc/nxtlibc.h>
#include <stdio.h>
#include <stdlib.h>

// Field size: 6 x 5 Leo's feet.
#define CAM_HEIGHT 768
#define CAM_WIDTH 1024

#define SD 140 // Unit of distance, where each distance of SD implies a change in movement speed.
#define OP_RADIUS 120
#define Q_DIST 120 //130
#define R_DIST 150
#define CLOSE_DIST 70 //100
#define CLOSE_DIST_MORE 25 //50
#define BALL_SPEED_THRES 200
#define ANG_THRES 5
#define KICK_VERIF_COUNT 3

void chaseBallSM(struct RoboAI *ai);
void penaltySM(struct RoboAI *ai);

void clear_motion_flags(struct RoboAI *ai)
{
 // Reset all motion flags. See roboAI.h for what each flag represents
 // *You may or may not want to use these*
 ai->st.mv_fwd=0; 
 ai->st.mv_back=0;
 ai->st.mv_bl=0;
 ai->st.mv_br=0;
 ai->st.mv_fl=0;
 ai->st.mv_fr=0;
}

struct blob *id_coloured_blob(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This function looks for and identifies a blob with the specified colour.
 // It uses colour contrast betwen the R, G, and B channels to pick the 
 // blob that is closest in colour to the specified target. If multiple
 // blobs with similar colour exist, then it picks the most saturated one.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> R
 //                   1 -> G
 //                   2 -> B
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double BCRT=1.05;			// Ball colour ratio threshold
 double c1,c2,c3,m,mi,ma;
 double oc1,oc2,oc3;
 int i;

 oc1=1000;
 oc2=1;
 oc3=1;

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  if (col==0) {c1=p->R; c2=p->G; c3=p->B;} 	// detect red
  else if (col==1) {c1=p->G; c2=p->R; c3=p->B;} // detect green
  else if (col==2){c1=p->B; c2=p->G; c3=p->R;}  // detect blue

  // Normalization and range extension
  mi=p->R;
  if (p->G<mi) mi=p->G;
  if (p->B<mi) mi=p->B;
  ma=p->R;
  if (p->G>ma) ma=p->G;
  if (p->B>ma) ma=p->B;

  c1=(c1-mi)/(ma-mi);
  c2=(c2-mi)/(ma-mi);
  c3=(c3-mi)/(ma-mi);
  c1+=.001;
  c2+=.001;
  c3+=.001;
  
  if (c1/c2>BCRT&&c1/c3>BCRT)			// Blob has sufficient colour contrast
  {
   m=(c1/c2)+(c1/c3);				// Total color contrast ch1 vs ch2 and ch3
   if (fnd==NULL||m>(oc1/oc2)+(oc1/oc3)) 	// Found the first blob with this color, or a more colorful one
   {
    fnd=p;
    oc1=c1;
    oc2=c2;
    oc3=c3;
   }
  }
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Heading (a unit vector in the direction of motion). Not valid
 //   while rotating - possibly valid while turning
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // Note that the blob data
 // structure itself contains another useful vector with the blob
 // orientation (obtained directly from the blob shape, valid at all
 // times even under rotation, but can be pointing backward!)
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 /////////////////////////////////////////////////////////////////////////
 static double prevSVX; // CUSTOM VARIABLE.
 static int prevMvFwd; // CUSTOM VARIABLE.
 static int waiting = 0; // CUSTOM VARIABLE.
 static int numInIf = 0;
 static int numWaited = 0;
 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 double NOISE_VAR=5;

 // Reset ID flags
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;

 // Find the ball
 p=id_coloured_blob(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute heading direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot
 if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,1);
 else p=id_coloured_blob(ai,blobs,0);
 if (p)
 {
  ai->st.self=p;			// Update pointer to self-blob

  // Adjust Y position if we have calibration data
  if (fabs(p->adj_Y[0][0])>.1)
  {
   dmax=384.0-p->adj_Y[0][0];
   dmin=767.0-p->adj_Y[1][0];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][0]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;

  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }

  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
 }
 else ai->st.self=NULL;

 // ID our opponent
 if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,0);
 else p=id_coloured_blob(ai,blobs,1);
 if (p)
 {
  ai->st.opp=p;	

  if (fabs(p->adj_Y[0][1])>.1)
  {
   dmax=384.0-p->adj_Y[0][1];
   dmin=767.0-p->adj_Y[1][1];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][1]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

  ////////////////////////////////////
  //////// CUSTOM CODE HERE ////////
  ////////////////////////////////////
  // Find the most "correct" direction vector
  // If magnitude of velocity is greater than 10, and our motors are fired, 
  // use heading vector. This is to prevent bad heading vectors when we get nudged by opponent.
  // This should also fix bad direction_Toggle values once the robot starts to move forward. 
  if ((ai->st.mv_fwd || ai->st.mv_back) && ai->st.svx*ai->st.svx + ai->st.svy*ai->st.svy > 25){
    double vMag = sqrt(ai->st.svy*ai->st.svy + ai->st.svx*ai->st.svx);
    // Adjust self blob direction_Toggle according to valid heading. 

    // Problem: When move flag changes from forward to backwards (or vice versa), velocity takes
    // a few frames to change to its negative counterpart. 
    // Fix by keeping toggle the same until velocity reaches its negative counterpart. 

    if (numInIf == 0) prevMvFwd = ai->st.mv_fwd;

    if (waiting && prevMvFwd != ai->st.mv_fwd){
      // case: we are already waiting, but the movement flag switches to what it was before.
      waiting = 0;
    } else if (prevMvFwd != ai->st.mv_fwd){
      // case: not waiting, but movement flag switches.
      waiting = 1;
    }
    if (waiting && prevSVX * ai->st.svx < 0){
      // case: waiting, and velocity has flipped over to its correct sign.
      waiting = 0;
    }
    if (!waiting || numWaited > 4) {
      // case: not waiting, simply set direction toggle to direction of header, with regards to
      // whether or not we're moving forward. 
      ai->st.direction_Toggle = ((ai->st.mv_fwd ? 1 : -1) * ai->st.svx/vMag >= 0 ? 1 : -1);
    } else {
      numWaited ++;
    }
    numInIf ++;
  } else { // Else use blob's direction vector. 
    waiting = 0;
    numInIf = 0;
    numWaited = 0;
    // Adjust self blob direction_Toggle if it jumps sporadically
    // This occurs when the robot rotates over the -90 or 90 degrees mark, where
    // 0 degrees is the positive x direction.
    // This is unreliable, so hopefully the above will the direction vector back to
    // reality once triggered. 
    if (acos(ai->st.self->dy*ai->st.old_sdy + ai->st.self->dx*ai->st.old_sdx) > 2){
      ai->st.direction_Toggle = ai->st.direction_Toggle * -1;
    }
    // Flip blob's direction vector by direction toggle.
  }
  ai->st.sdx = ai->st.direction_Toggle * ai->st.self->dx;
  ai->st.sdy = ai->st.direction_Toggle * ai->st.self->dy;
  ai->st.old_sdx = ai->st.self->dx;
  ai->st.old_sdy = ai->st.self->dy;

  // Update static values.
  prevSVX = ai->st.svx;
  prevMvFwd = ai->st.mv_fwd;
  //printf("dx: %f dy: %f vx: %f vy: %f t: %d mF: %d mB %d W %d\n", ai->st.sdx, ai->st.sdy, ai->st.svx, ai->st.svy, ai->st.direction_Toggle, ai->st.mv_fwd, ai->st.mv_back, waiting);
}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 double frame_inc=1.0/5.0;

  // TODO: MODIFIED
  drive_speed(-30);			// Start forward motion to establish heading
  // Reset motion flags.
  clear_motion_flags(ai);
  ai->st.mv_fwd = 1;
  // Will move for a few frames.

 track_agents(ai,blobs);		// Call the tracking function to find each agent

 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  all_stop();
  // Reset motion flags.
  clear_motion_flags(ai);
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}

int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
	fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 all_stop();			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 clear_motion_flags(ai);
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 track_agents(ai,blobs);
}

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is the main AI loop.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
                  state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
                  knows where the robot is, as well as where the opponent and
                  ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
  data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your NXT to get damaged!

  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/
 if (ai->st.state % 100 != 0) track_agents(ai,blobs);
 //printf ("Self bot found: %d\n", ai->st.selfID);
 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)   // Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)  // The id_bot() routine will change the AI state to initial state + 1
  {       // if robot identification is successful.
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;
   all_stop();
   clear_motion_flags(ai);
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.self->mx,ai->st.self->my,ai->st.state);
  }
  // This shouldn't be needed anymore since this is calculated at bottom of loop.
  // if (ai->st.side == 1){
  //   ai->st.direction_Toggle = -1;
  // } else {
  //   ai->st.direction_Toggle = 1;
  // }
 }
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
  else if (ai->st.state/100 == 0){
    soccerSM(ai);
  }
  else if (ai->st.state/100 == 1){
    // Penalty kick state.
    penaltySM(ai);
  } 
  else if (ai->st.state/100 == 2){
    // Chase ball state.
    chaseBallSM(ai);
  }
  //printf("%f\n", ai->st.self->dy*ai->st.self->dy + ai->st.self->dx*ai->st.self->dx);

}



/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/

// Chase ball state machine. If statements control transitions, 
// while the rest of each case statement executes the state. 
void chaseBallSM(struct RoboAI *ai){
  double dx, dy;
  switch (ai->st.state) {
    case 200: // Initial state for chase ball. 
      // Make sure ball is stationary.
      if (ai->st.ballID && (fabs(ai->st.bvx) < 5 && fabs(ai->st.bvy) < 5)){
        ai->st.state = 201;
      }
    break;
    case 201: // Move to ball.
      stop_kicker();
      moveInDirection(ai, ai->st.old_bcx, ai->st.old_bcy, 0, 30);

      dx = ai->st.old_scx - ai->st.old_bcx;
      dy = ai->st.old_scy - ai->st.old_bcy;
      if (!ai->st.ballID){
        ai->st.state = 203;
      }
      // Within a certain distance from ball. Go to state 202
      else if (dx*dx + dy*dy < CLOSE_DIST*CLOSE_DIST){
        ai->st.state = 202;
      }
    break;
    case 202: // Kick the ball.
      dx = ai->st.old_scx - ai->st.old_bcx;
      dy = ai->st.old_scy - ai->st.old_bcy;

      moveAndKick(ai, 40);
      if (!ai->st.ballID){
        ai->st.state = 203;
      } 
      else if (ai->st.bvx*ai->st.bvx + ai->st.bvy*ai->st.bvy > BALL_SPEED_THRES || 
               dx*dx + dy*dy < CLOSE_DIST*CLOSE_DIST*16){
        ai->st.state = 201;
      }
    break;
    case 203: // attempt to reveal ball by going reverse.
      // If we hit border, ball is out of bounds.
      // If ball reveals, continue playing. 
      stop_kicker();
      reverse_speed(-30);
      // Reset motion flags.
      clear_motion_flags(ai);
      ai->st.mv_back = 1;

      if (ai->st.ballID) {
        ai->st.state = 201;
      }
      else if ((ai->st.old_scx < CLOSE_DIST || (CAM_WIDTH - ai->st.old_scx) < CLOSE_DIST) || 
               (ai->st.old_scy < CLOSE_DIST || (CAM_HEIGHT - ai->st.old_scy) < CLOSE_DIST)){
        ai->st.state = 204;
      }
    break;
    case 204: //finish
      all_stop();
      stop_kicker();
      clear_motion_flags(ai);
      printf("Chase ball state machine done.\n"); //FIXED: Added newline to chase done message
    break;
  }
}

// Penalty kick state machine. 
void penaltySM(struct RoboAI *ai){
  double vx, vy, qx, qy, vmag;
  switch (ai->st.state) {
    case 100: // Initial state
      // Make sure ball is stable, and found.
      if (ai->st.ballID && (fabs(ai->st.bvx) < 5 && fabs(ai->st.bvy) < 5)){
        ai->st.state = 101;
      }
    break;
    case 101: // Move to Q
      // Within a certain distance from point q,
      // go to state 102
      // Calculate q, the point just behind the ball (relative to the direction where
      // front = towards net)
      vx = ai->st.old_bcx - ((ai->st.side) ? 0 : CAM_WIDTH);
      vy = ai->st.old_bcy - CAM_HEIGHT / 2;
      vmag = sqrt(vx*vx + vy*vy);
      qx = ai->st.old_bcx + Q_DIST*vx/vmag;
      qy = ai->st.old_bcy + Q_DIST*vy/vmag;

      stop_kicker();
      moveInDirection(ai, qx, qy, 0, 30);
      if (fabs(ai->st.old_scy-qy) < CLOSE_DIST_MORE){
        all_stop();
        clear_motion_flags(ai);
        ai->st.state = 102;
      }
      // printf("101 Q: %f, %f | B: %f, %f | S: %f, %f\n", qx, qy, ai->st.old_bcx, ai->st.old_bcy, ai->st.old_scx, ai->st.old_scy);
    break;
    case 102: // Reach ball
      // Within a certain distance from ball. Go to state 103
      stop_kicker();
      moveInDirection(ai, ai->st.old_bcx, ai->st.old_bcy, 1, 20);

      vx = ai->st.old_scx - ai->st.old_bcx;
      vy = ai->st.old_scy - ai->st.old_bcy;
      if (vx*vx + vy*vy < CLOSE_DIST*CLOSE_DIST){
        ai->st.state = 103;
      }
      // printf("102 B: %f, %f | S: %f, %f\n", ai->st.old_bcx, ai->st.old_bcy, ai->st.old_scx, ai->st.old_scy);
    break;
    case 103: // kick
        moveAndKick(ai, 40);
      // Ball is in motion. Go to state 104
      // Make sure this was not a bad sensor reading by checking it 3 times.
      if (ai->st.bvx*ai->st.bvx + ai->st.bvy*ai->st.bvy > BALL_SPEED_THRES){
        ai->st.satisfactionCount ++;
      } else {
        ai->st.satisfactionCount = 0;
      }
      if (ai->st.satisfactionCount == KICK_VERIF_COUNT){
        ai->st.satisfactionCount = 0; // FIXED: Prevented early satisfactions from aborted kick attempts
        ai->st.state = 104;
      }
      printf("Kick satisfied for: %d\n", ai->st.satisfactionCount);
      // Otherwise try kick again. Go to state 102
      // else {
      //   ai->st.state = 102;
      // }    
    break;
    case 104: // Finished
      stop_kicker();
      all_stop();
      clear_motion_flags(ai);
      fprintf(stderr, "Penalty state machine done.\n");
    break;
  }
}

void soccerSM(struct RoboAI *ai){
  double resultX, resultY, attackBackoffX, attackBackoffY, defendBackoffX, defendBackoffY;
  double *rx = &resultX;
  double *ry = &resultY;
  double *qx = &attackBackoffX;
  double *qy = &attackBackoffY;
  double *Rx = &defendBackoffX;
  double *Ry = &defendBackoffY;

  //FIXED: Patched transition status of many states due to uncalculated Qs and Rs
  switch(ai->st.state){
    case 0: // Choose mode.
      stop_kicker();
    break;
    case 1: // Attack, avoid opponent.
      stop_kicker();
      findQ(ai, qx, qy, Q_DIST);
      findDefendPoint(ai, Rx, Ry, OP_RADIUS, R_DIST);
      obstAvoid(rx, ry, *qx, *qy,// FIXED: Used to be ai->st.old_bcx, ai->st.old_bcy, 
        ai->st.old_ocx, ai->st.old_ocy, 
        ai->st.old_scx, ai->st.old_scy, OP_RADIUS);
      moveInDirection(ai, *rx, *ry, 0, 35);
      printf("r = (%f, %f) ", *rx, *ry);
    break;
    case 2: // Attack, chase the ball.
      stop_kicker();
      findQ(ai, qx, qy, Q_DIST);
      findDefendPoint(ai, Rx, Ry, OP_RADIUS, R_DIST);
      moveInDirection(ai, ai->st.old_bcx, ai->st.old_bcy, 0, 30);
    break;
    case 3: // Attack, prepare to shoot.
      stop_kicker();
      findQ(ai, qx, qy, Q_DIST);
      findDefendPoint(ai, Rx, Ry, OP_RADIUS, R_DIST);
      moveInDirection(ai, *qx, *qy, 0, 30);
      printf("Q = (%f, %f) ", *qx, *qy);
    break;
    case 4: // Attack, chase ball kick
      moveAndKick(ai, 40);
    break;
    case 5: // Attack, shoot align
      stop_kicker();
      moveInDirection(ai, ai->st.old_bcx, ai->st.old_bcy, 1, 20);
    break;
    case 6: // Attacking shoot kick
      moveAndKick(ai, 40);
    break;
    case 7: // Defend, avoid enemy.
      stop_kicker();
      findQ(ai, qx, qy, Q_DIST);
      findDefendPoint(ai, Rx, Ry, OP_RADIUS, R_DIST);
      printf("R = (%f, %f) ", *Rx, *Ry);
      obstAvoid(rx, ry, *Rx, *Ry,
        ai->st.old_ocx, ai->st.old_ocy,
        ai->st.old_scx, ai->st.old_scy, OP_RADIUS);
      moveInDirection(ai, *rx, *ry, 0, 35);
      printf("r = (%f, %f) ", *rx, *ry);
    break;
    case 8: // Defend, move to R.
      stop_kicker();
      findQ(ai, qx, qy, Q_DIST);
      findDefendPoint(ai, Rx, Ry, OP_RADIUS, R_DIST);
      moveInDirection(ai, *Rx, *Ry, 0, 30);
      printf("R = (%f, %f) ", *Rx, *Ry);
    break;
    case 9: // Defend, move to Q.
      stop_kicker();
      findQ(ai, qx, qy, Q_DIST);
      findDefendPoint(ai, Rx, Ry, OP_RADIUS, R_DIST);
      moveInDirection(ai, *qx, *qy, 0, 30);
      printf("Q = (%f, %f) ", *qx, *qy);
    break;
    case 10: // Defend, block ball kick
      moveAndKick(ai, 40);
    break;
    case 11: // Defend, counter-attack align.
      stop_kicker();
      moveInDirection(ai, ai->st.old_bcx, ai->st.old_bcy, 1, 20);
    break;
    case 12: // Defend, counter-attack kick.
      moveAndKick(ai, 40);
    break;
    case 98: // Ball missing
      stop_kicker();
      reverse_speed(-30);
      // Reset motion flags.
      clear_motion_flags(ai);
      ai->st.mv_back = 1;
    break;
    case 99: 
      stop_kicker();
      all_stop();
      clear_motion_flags(ai);
      printf("Soccer finished");
    break;
  }
  printf("State: %d => ", ai->st.state);
  soccerSMTrans(ai, Rx, Ry, qx, qy);
  printf("%d | Direction Toggle: %d\n", ai->st.state, ai->st.direction_Toggle);
}

inline void soccerSMTrans(struct RoboAI *ai, double *Rx, double *Ry, double *qx, double*qy){
  switch(ai->st.state){
    case 0: // Choose mode.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransD(ai)) ai->st.state = 1;
      else if (ssmTransE(ai, qx, qy)) ai->st.state = 2;
      else if (ssmTransF(ai, qx, qy)) ai->st.state = 3;
      else if (ssmTransG(ai, Rx, Ry)) ai->st.state = 7;
      else if (ssmTransI(ai, qx, qy)) ai->st.state = 9;
      else if (ssmTransH(ai, Rx, Ry)) ai->st.state = 8;
    break;
    case 1: // Attack, avoid opponent.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransD(ai)) ai->st.state = 1;
      else if (ssmTransE(ai, qx, qy)) ai->st.state = 2;
      else if (ssmTransF(ai, qx, qy)) ai->st.state = 3;
      else if (ssmTransG(ai, Rx, Ry)) ai->st.state = 7;
      else if (ssmTransI(ai, qx, qy)) ai->st.state = 9;
      else if (ssmTransH(ai, Rx, Ry)) ai->st.state = 8;
    break;
    case 2: // Attack, chase the ball.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransJ(ai)) ai->st.state = 4;
      else if (ssmTransD(ai)) ai->st.state = 1;
      else if (ssmTransE(ai, qx, qy)) ai->st.state = 2;
      else if (ssmTransF(ai, qx, qy)) ai->st.state = 3;
      else if (ssmTransG(ai, Rx, Ry)) ai->st.state = 7;
      else if (ssmTransI(ai, qx, qy)) ai->st.state = 9;
      else if (ssmTransH(ai, Rx, Ry)) ai->st.state = 8;
    break;
    case 3: // Attack, prepare to shoot.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransL(ai, qx, qy)) ai->st.state = 5;
      else if (ssmTransD(ai)) ai->st.state = 1;
      else if (ssmTransE(ai, qx, qy)) ai->st.state = 2;
      else if (ssmTransF(ai, qx, qy)) ai->st.state = 3;
      else if (ssmTransG(ai, Rx, Ry)) ai->st.state = 7;
      else if (ssmTransI(ai, qx, qy)) ai->st.state = 9;
      else if (ssmTransH(ai, Rx, Ry)) ai->st.state = 8;
    break;
    case 4: // Attack, chase ball kick
      if (!ai->st.selfID) ai->st.state = 99;
      else if (ssmTransN(ai)) ai->st.state = 2; //FIXED: Kick failed
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransK(ai)) ai->st.state = 2;
    break;
    case 5: // Attack, shoot align
      if (!ai->st.selfID) ai->st.state = 99;
      else if (ssmTransN(ai)) ai->st.state = 2; //FIXED: Kick failed
      //else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransJ(ai)) ai->st.state = 6;
    break;
    case 6: // Attacking shoot kick
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransK(ai)) ai->st.state = 3;
    break;
    case 7: // Defend, avoid enemy.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransD(ai)) ai->st.state = 1;
      else if (ssmTransE(ai, qx, qy)) ai->st.state = 2;
      else if (ssmTransF(ai, qx, qy)) ai->st.state = 3;
      else if (ssmTransG(ai, Rx, Ry)) ai->st.state = 7;
      else if (ssmTransI(ai, qx, qy)) ai->st.state = 9;
      else if (ssmTransH(ai, Rx, Ry)) ai->st.state = 8;
    break;
    case 8: // Defend, move to R.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransM(ai, Rx, Ry)) ai->st.state = 11;
      else if (ssmTransJ(ai)) ai->st.state = 11;
      else if (ssmTransD(ai)) ai->st.state = 1;
      else if (ssmTransE(ai, qx, qy)) ai->st.state = 2;
      else if (ssmTransF(ai, qx, qy)) ai->st.state = 3;
      else if (ssmTransG(ai, Rx, Ry)) ai->st.state = 7;
      else if (ssmTransI(ai, qx, qy)) ai->st.state = 9;
    break;
    case 9: // Defend, move to Q.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransL(ai, qx, qy)) ai->st.state = 11; // FIXED: Should use Q instead of R
      else if (ssmTransD(ai)) ai->st.state = 1;
      else if (ssmTransE(ai, qx, qy)) ai->st.state = 2;
      else if (ssmTransF(ai, qx, qy)) ai->st.state = 3;
      else if (ssmTransG(ai, Rx, Ry)) ai->st.state = 7;
      else if (ssmTransH(ai, Rx, Ry)) ai->st.state = 8;
      else if (ssmTransI(ai, qx, qy)) ai->st.state = 9;
    break;
    case 10: // Defend, block ball kick
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransN(ai)) ai->st.state = 2; //FIXED: Kick failed
      else if (ssmTransK(ai)) ai->st.state = 9;
    break;
    case 11: // Defend, counter-attack align.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransN(ai)) ai->st.state = 2; //FIXED: Kick failed
      else if (ssmTransJ(ai)) ai->st.state = 12;
    break;
    case 12: // Defend, counter-attack kick.
      if (!ai->st.selfID) ai->st.state = 99;
      else if (!ai->st.ballID || !ai->st.oppID) ai->st.state = 98;
      else if (ssmTransN(ai)) ai->st.state = 2; //FIXED: Kick failed
      else if (ssmTransK(ai)) ai->st.state = 9;
    break;
    case 98: // Ball missing
      if (!ai->st.selfID) ai->st.state = 99;
      else if (ai->st.ballID && ai->st.oppID) ai->st.state = 1;
      else if (ssmTransC(ai)) ai->st.state = 99;
    break;
  }
}

// If bot is close to border.
inline int ssmTransC(struct RoboAI *ai){
  return (ai->st.old_scx < CLOSE_DIST || (CAM_WIDTH - ai->st.old_scx) < CLOSE_DIST) || 
         (ai->st.old_scy < CLOSE_DIST || (CAM_HEIGHT - ai->st.old_scy) < CLOSE_DIST);
}

// If attacking and the bot's path to ball is obstructed by opponent.
inline int ssmTransD(struct RoboAI *ai){
  return attackMode(ai) && 
  !hasClearPath(OP_RADIUS, ai->st.old_bcx, ai->st.old_bcy, 
    ai->st.old_ocx, ai->st.old_ocy,
    ai->st.old_scx, ai->st.old_scy);
}

// If attacking and Q is obstructed
inline int ssmTransE(struct RoboAI *ai, double *qx, double *qy){
  return attackMode(ai) && pointObstructed(ai, OP_RADIUS, *qx, *qy);
}

// If attacking and Q is unobstructed
inline int ssmTransF(struct RoboAI *ai, double *qx, double *qy){
  return attackMode(ai) && !pointObstructed(ai, OP_RADIUS, *qx, *qy);
}

// If defending and the bot's path to the ball is obstructed
// by the enemy, or R is obstructed.
inline int ssmTransG(struct RoboAI *ai, double *rx, double *ry){
  return !attackMode(ai) &&
    (!hasClearPath(OP_RADIUS, ai->st.old_bcx, ai->st.old_bcy, 
      ai->st.old_ocx, ai->st.old_ocy,
      ai->st.old_scx, ai->st.old_scy) || pointObstructed(ai, OP_RADIUS, *rx, *ry));
}

// If defending and R is unobstructed and the ball's path to the enemy's
// goal is obstructed.
// FIXED: Prevented unwanted transitions into state 8
inline int ssmTransH(struct RoboAI *ai, double *rx, double *ry){
  return !attackMode(ai) && !pointObstructed(ai, OP_RADIUS, *rx, *ry) &&
    !hasClearPath(OP_RADIUS, 
      (ai->st.side ? 0 : CAM_WIDTH), CAM_HEIGHT / 2, 
      ai->st.old_ocx, ai->st.old_ocy,
      ai->st.old_bcx, ai->st.old_bcy);
}

// if defending and the ball's path to the enemy goal is unobstructed,
// and the point to kick it from is unobstructed.
inline int ssmTransI(struct RoboAI *ai, double *qx, double *qy){
  return !attackMode(ai) && 
    hasClearPath(OP_RADIUS, 
      (ai->st.side ? 0 : CAM_WIDTH), CAM_HEIGHT / 2, 
      ai->st.old_ocx, ai->st.old_ocy,
      ai->st.old_bcx, ai->st.old_bcy) && !pointObstructed(ai, CLOSE_DIST_MORE, *qx, *qy);
}

// If the ball is within a set radius of the bot
inline int ssmTransJ(struct RoboAI *ai){
  double dx = ai->st.old_scx - ai->st.old_bcx;
  double dy = ai->st.old_scy - ai->st.old_bcy;
  return dx*dx + dy*dy < CLOSE_DIST*CLOSE_DIST;
}

// If the ball achieves a set speed for K steps 
inline int ssmTransK(struct RoboAI *ai){
  if (ai->st.bvx*ai->st.bvx + ai->st.bvy*ai->st.bvy > BALL_SPEED_THRES){
    ai->st.satisfactionCount ++;
  } else {
    ai->st.satisfactionCount = 0;
  }
  int result = ai->st.satisfactionCount;
  if (result >= KICK_VERIF_COUNT) {
    ai->st.satisfactionCount = 0;
  }
  return result;
}

// If the bot is within a set radius of q
inline int ssmTransL(struct RoboAI *ai, double *qx, double *qy){
  double dx = ai->st.old_scx - *qx;
  double dy = ai->st.old_scy - *qy;
  // Within a certain distance from ball. Go to state 202
  return dx*dx + dy*dy < CLOSE_DIST*CLOSE_DIST;
}

// If the bot is within a set radius of R
inline int ssmTransM(struct RoboAI *ai, double *rx, double *ry){
  double dx = ai->st.old_scx - *rx;
  double dy = ai->st.old_scy - *ry;
  // Within a certain distance from ball. Go to state 202
  return dx*dx + dy*dy < CLOSE_DIST*CLOSE_DIST;
}

// If the ball is not within a set radius of the bot
inline int ssmTransN(struct RoboAI *ai){
  double dx = ai->st.old_scx - ai->st.old_bcx;
  double dy = ai->st.old_scy - ai->st.old_bcy;
  return dx*dx + dy*dy > 4*CLOSE_DIST*CLOSE_DIST;
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////////////////////////////////////////////////////////

// Rotates the robot (if needed) and move to the specified point. 
// Will continue moving until state machine transitions out. 
void moveInDirection(struct RoboAI *ai, double x, double y, int pivot, int minSpeed){
  // Check orientation.
  // Find vector from bot to point.
  double dx = x - ai->st.old_scx;
  double dy = y - ai->st.old_scy;
  double dMag = dx*dx + dy*dy;
  int actualSpeed = minSpeed;

  // Reset motion flags.
  clear_motion_flags(ai);

  actualSpeed = fmin(minSpeed + (10 * (dMag / (SD*SD))), 100);

  // The angles of both vectors.
  double _dtheta = atan2(dy, dx);
  // double _stheta = atan2(ai->st.direction_Toggle*ai->st.self->dy, ai->st.direction_Toggle*ai->st.self->dx);
  double _stheta = atan2(ai->st.sdy, ai->st.sdx); // Use custom "corrected" heading.
  // Normalize angles to 0, 2PI
  _dtheta = (_dtheta < 0) ? _dtheta + 2*M_PI : _dtheta;
  _stheta = (_stheta < 0) ? _stheta + 2*M_PI : _stheta;
  int dtheta = (int)(_dtheta * 180/M_PI);
  int stheta = (int)(_stheta * 180/M_PI);
  double angle = ((((dtheta - stheta) % 360) + 540) % 360) - 180;
  // double angle = dtheta - stheta;
  // angle = (angle < 0) ? angle + 2*M_PI : angle;
  // angle = angle * 180/M_PI; // Convert to deg for now.
  // angle = angle > 180 
  // If oriented, more forward. 
  int rotSpeed = (angle > 0) ? (int)(10 + angle/4) : (int)(-10 + angle/4);
  // printf("Angle: %f, rSpeed: %d\n", angle, rotSpeed);
  if (fabs(angle) <= 10){
    drive_speed(-actualSpeed);
    ai->st.mv_fwd = actualSpeed > 0;
    ai->st.mv_back = actualSpeed < 0;
  }
  else if (pivot){
      pivot_right_speed(-rotSpeed);
  } else {
    int lSpeed, rSpeed;
    if (fabs(angle) <= 35){
      lSpeed = actualSpeed + rotSpeed;
      rSpeed = actualSpeed - rotSpeed;
    } else {
      lSpeed = rotSpeed;
      rSpeed = -rotSpeed;
    }

    // Only set motion flags if both motors are moving in same direction,
    // and are not zero in speed. 
    if (fabs(lSpeed - rSpeed) < 10){ // If difference in left wheel and right wheel is less than 7
      if (lSpeed < 0 && rSpeed < 0) ai->st.mv_back = 1;
      else if (lSpeed > 0 && rSpeed > 0) ai->st.mv_fwd = 1;
    } else {
      if (lSpeed > 0 && rSpeed > 0){
        ai->st.mv_fl = lSpeed < rSpeed; // r wheel faster than l wheel -> l turn.
        ai->st.mv_fr = rSpeed < lSpeed; // l wheel faster than r wheel -> r turn.
      } else if (lSpeed < 0 && rSpeed < 0){
        // Note negative speed -> faster in this case.
        ai->st.mv_bl = rSpeed < lSpeed; // r wheel faster than l wheel -> butt swings left.
        ai->st.mv_br = lSpeed < rSpeed; // l wheel faster than r wheel -> butt swings right.
      }
    }
    drive_custom(-lSpeed, -rSpeed);// TODO: Flip sign when controls are fixed.
  }
  //printf("Move completed\n");
}

// Move forward and kick at the given speed. 
void moveAndKick(struct RoboAI *ai, int speed){
  // IDEA: Make kicker toggle between on and off on each iteration. 
  retract();
  drive_speed(-speed);
  // Reset motion flags.
  clear_motion_flags(ai);
  ai->st.mv_fwd = 1;
}

/**
* Return whether the path from source to target is obstructed by o with specified size. 
* size: Radius of the obstacle.
* gx, gy: The position of the target.
* ox, oy: The position of the obstacle.
* bx, by: The position of the source.
* Return true if obstructed.  
*/
int hasClearPath(int size, double gx, double gy, double ox, double oy, double bx, double by){
  // Reuse variables to avoid unnecessary memory use. 
  // Let o now be vector from b to o.
  ox = ox - bx;
  oy = oy - by;
  // Let g now be vector from b to g
  gx = gx - bx;
  gy = gy - by;
  // If b is currently within the size of o
  if (ox*ox + oy*oy < size*size){
    return 0;
  }
  // If obstacle is behind us, then we have clear path.
  else if (ox*gx + oy*gy < 0){
    return 1;
  }
  // If the obstacle is farther away from us than the target and
  // the target is not in range of the obstacle, then we have
  // clear path.
  // FIXED: Prevented false negatives when the obstacle stands far behind
  // the goal
  else if ((ox*gx + oy*gy)/(gx*gx + gy*gy) > 1 && (gx-ox)*(gx-ox)+(gy-oy)*(gy-oy) > size*size) {
    return 1;
  }
  // See if mag of rejection of o onto g is greater than size. 
  // If true, then we have clear path.
  // projected vector is = (o dot g / |g|^2 * g)
  double temp = (ox*gx + oy*gy)/(gx*gx + gy*gy);
  ox = ox - temp*gx;
  oy = oy - temp*gy; 
  return ox*ox + oy*oy > size*size;
}

/**
* Return whether we should be attacking. True -> attack mode.
*/
int attackMode(struct RoboAI *ai){
  // If side is 0, meaning we are on left side, goal is on right side (x=1024)
  double vToGoalx = (ai->st.side ? 0 : CAM_WIDTH) - ai->st.old_ocx;
  double vToGoaly = CAM_HEIGHT/2 - ai->st.old_ocy;
  double vToBallx = ai->st.old_bcx - ai->st.old_ocx;
  double vToBally = ai->st.old_bcy - ai->st.old_ocy;
  return vToGoalx*vToBallx + vToGoaly*vToBallx > 0;
}

/**
* Return whether point p is covered by opponent.
*/
int pointObstructed(struct RoboAI *ai, int size, double px, double py){
  // return true if opponent is within size distance of ball.
  if ((px <= 0 || px >= CAM_WIDTH) || (py <= 0 || py >= CAM_HEIGHT)){
    printf("obstructed: 1");
    return 1;
  }
  px = ai->st.old_ocx - px;
  py = ai->st.old_ocy - py;
  printf ("obstructed: %d\n", px*px + py*py < size*size);
  return px*px + py*py < size*size;
  // TODO identify when ball is out of field?
}

/**
 * Locate the optimal point q to be at to kick the ball into goal. 
 * ai: robot ai struct
 * qx, qy: Computed q value is stored here.
 * backoffDist: the distance that should be between q and ball. 
*/
void findQ(struct RoboAI *ai, double *qx, double *qy, int backoffDist){
  double vx = ai->st.old_bcx - (ai->st.side ? 0 : CAM_WIDTH);
  double vy = ai->st.old_bcy - CAM_HEIGHT / 2;
  double vmag = sqrt(vx*vx + vy*vy);
  *qx = ai->st.old_bcx + backoffDist*vx/vmag;
  *qy = ai->st.old_bcy + backoffDist*vy/vmag;
}

/**
* Find the point for the source to travel to while avoiding the obstacle.
* rx, ry: Position for the bot to move to. 
* gx, gy: Position of target.
* ox, oy: Position of obstacle.
* bx, by: Position of source. 
* size: radius of the obstacle.
* return: true if there's a clear path from b to g. False if an avoidance path is computed. 
*/
int obstAvoid(double *rx, double *ry, double gx, double gy, double ox, double oy, double bx, double by, int size){
  // If a clear path already exists, simply return g as unit vector.
  if (hasClearPath(size, gx, gy, ox, oy, bx, by)){
    // gx = gx - bx;
    // gy = gy - by;
    // double mag = sqrt(gx*gx + gy*gy);
    // *rx = gx/mag;
    // *ry = gy/mag;
    *rx = gx;
    *ry = gy;
    return 1;
  } else {
    // Else, the obstacle is in our path, or covering us. Move in a direction to avoid it. 
    // Establish new direction by finding rejection vector.
    // let e = vector from b to o.
    // let p = vector from b to g.
    // let g = negative rejection vector found by projecting e onto p, unified. 
    double ex = ox - bx;
    double ey = oy - by;
    double px = gx - bx;
    double py = gy - by;

    double temp = (ex*px + ey*py) / (px*px + py*py);
    // Reuse gx and gy
    gx = ex - temp * px;
    gy = ey - temp * py;

    // let r be the point to go in to avoid the obstacle.
    // r = o + size*(g/|g|) = a point just on the edge of the circle created with center at o, of radius size. 
    // Since we're using  
    temp = sqrt(gx*gx + gy*gy);

    // gx = ox + gx/temp*size - bx;
    // gy = oy + gy/temp*size - by;
    // temp = sqrt(gx*gx + gy*gy);
    // *rx = gx / temp;
    // *ry = gy / temp;
    *rx = ox - (size+10)*gx/temp;
    *ry = oy - (size+10)*gy/temp;
    return 0;
  }
}

/*
* rx, ry: Returned point to be at when defending.
* gx, gy: 
*/
void findDefendPoint(struct RoboAI *ai, double *rx, double *ry, int size, int backoffDist){
  obstAvoid(rx, ry, 
    ai->st.side ? 0 : CAM_WIDTH, CAM_HEIGHT / 2, 
    ai->st.old_ocx, ai->st.old_ocy, 
    ai->st.old_bcx, ai->st.old_bcy, 
    size);
  double rMag = sqrt((*rx)*(*rx) + (*ry)*(*ry));
  *rx = ai->st.old_bcx + backoffDist/rMag*(*rx);
  *ry = ai->st.old_bcy + backoffDist/rMag*(*ry);
  //printf("rx: %f, ry: %f\n", *rx, *ry);
}
