/***************************************************
 CSC C85 - UTSC RoboSoccer AI core
 
 This file contains the definition of the AI data
 structure which holds the state of your bot's AI.

 You must become familiar with this structure and
 its contents. 

 You will need to modify this file to add headers
 for any functions you added to implemet the 
 soccer playing functionality of your bot.

 Be sure to document everything you do thoroughly.

 AI scaffold: Parker-Lee-Estrada, Summer 2013

***************************************************/

#ifndef _ROBO_AI_H
#define _ROBO_AI_H

#include "imagecapture/imageCapture.h"

#define AI_SOCCER 0     // Play soccer!
#define AI_PENALTY 1    // Go score some goals!
#define AI_CHASE 2  // Kick the ball around and chase it!

struct AI_data{
    // This data structure is used to hold all data relevant to the state of the AI.
    // This includes, of course, the current state, as well as the status of
    // our own bot, the opponent (if present), and the ball (if present).
    // For each agent in the game we keep a pointer to the blob that corresponds
    // to the agent (see the blob data structure in imageCapture.h), and data
    // about its old position, as well as current velocity and heading vectors.
    //
    // MIND THE NOISE.

    // Robot's playfield side id (w.r.t. the viepoint of the camera).
    int side;       // side=0 implies the robot's own side is the left side
                // side=1 implies the robot's own side is the right side
                // This is set based on the robot's initial position
                // on the field
    int botCol;     // Own bot's colour. 0 - green, 1 - red

    int state;      // Current AI state

    // Motion flags - ** These should be set by your own code, if you want to  use them **
    int mv_fwd, old_mv_fwd;     // moving forward
    int mv_back, old_mv_back;   // moving backward
    int mv_bl;      // moving back while turning left
    int mv_br;      // moving back while turning right  
    int mv_fl;      // moving forward while turning left
    int mv_fr;      // moving forward while turning right

    // Object ID status for self, opponent, and ball. Just boolean 
    // values indicating whether blobs have been found for each of these
    // entities.
    int selfID;
    int oppID;
    int ballID;

    // Blob track data. Ball likely needs to be detected at each frame
    // separately. So we keep old location to estimate v
    struct blob *ball;      // Current ball blob
    double old_bcx, old_bcy;// Previous ball cx,cy
    double bcx, bcy; // A predicted position of the ball.
    double bvx,bvy;         // Ball velocity vector
    double bmx,bmy;         // Ball heading
    int satisfactionCount;  // Used to determine whether the ball is truly in motion.

    // Self track data. Done separately each frame
    struct blob *self;      // Current self blob
    double old_scx, old_scy;// Previous self (cx,cy)
    double svx,svy;         // Current self [vx vy]
    double smx,smy;         // Self heading
    double sdx,sdy;         // Self direction vector.
    // The below two added variables help with detecting the actual direction vector,
    // since the one provided in the blobs always point right.
    double old_sdx, old_sdy; // Stores previous direction vector.
    int direction_Toggle; // Used to detect when the direction vector flips over.

    // Opponent track data. Done separately each frame
    struct blob *opp;       // Current opponent blob
    double old_ocx, old_ocy;    // Previous opponent (cx,cy)
    double ovx,ovy;         // Current opponent [vx vy]
    double omx,omy;         // Opponent heading
};

struct RoboAI {
    // Main AI data container. It allows us to specify which function
    // will handle the AI, and sets up a data structure to store the
    // AI's data (see above).
    void (* runAI)(struct RoboAI *ai, struct blob *, void *state);
    void (* calibrate)(struct RoboAI *ai, struct blob *);
    struct AI_data st;
};

/**
 * \brief Set up an AI structure for playing roboSoccer
 *
 * Set up an AI structure for playing roboSoccer. Must be
 * called before using the AI structure during gameplay.
 * \param[in] mode The operational mode for the AI
 * \param[out] ai A structure containing data necessary for
 *      AI algorithms
 * \pre ai is uninitialized
 * \post ai is set up for use, and must be cleaned up using
 *      cleanupAI
 */
int setupAI(int mode, int own_col, struct RoboAI *ai);

/**
 * \brief Top-level AI loop.
 * 
 * Decides based on current state and blob configuration what
 * the bot should do next, and calls the appropriate behaviour
 * function.
 *
 * \param[in] ai, pointer to the data structure for the running AI
 * \param[in] blobs, pointer to the current list of tracked blobs
 * \param[out] void, but the state description in the AI structure may have changed
 * \pre ai is not NULL, blobs is not NULL
 * \post ai is not NULL, blobs is not NULL
 */
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state);

// Calibration stub
void AI_calibrate(struct RoboAI *ai, struct blob *blobs);

/* PaCode - just the function headers - see the functions for descriptions */
void id_bot(struct RoboAI *ai, struct blob *blobs);
struct blob *id_coloured_blob(struct RoboAI *ai, struct blob *blobs, int col);
void track_agents(struct RoboAI *ai, struct blob *blobs);
void clear_motion_flags(struct RoboAI *ai);

/****************************************************************************
 TO DO:
   Add headers for your own functions implementing the bot's soccer
   playing functionality below.
*****************************************************************************/

void chaseBallSM(struct RoboAI *ai);
void penaltySM(struct RoboAI *ai);
void soccerSM(struct RoboAI *ai);

inline void soccerSMTrans(struct RoboAI *ai, double *Rx, double *Ry, double *qx, double*qy);
inline int ssmTransC(double px, double py);
inline int ssmTransD(struct RoboAI *ai);
inline int ssmTransE(struct RoboAI *ai, double *qx, double *qy);
inline int ssmTransF(struct RoboAI *ai, double *qx, double *qy);
inline int ssmTransG(struct RoboAI *ai, double *rx, double *ry);
inline int ssmTransH(struct RoboAI *ai, double *rx, double *ry);
inline int ssmTransI(struct RoboAI *ai, double *qx, double *qy);
inline int ssmTransJ(struct RoboAI *ai);
inline int ssmTransK(struct RoboAI *ai);
inline int ssmTransL(struct RoboAI *ai, double *qx, double *qy);
inline int ssmTransM(struct RoboAI *ai, double *rx, double *ry);
inline int ssmTransN(struct RoboAI *ai, double multiple);
inline int ssmTransO(struct RoboAI *ai);

void moveInDirection(struct RoboAI *ai, double x, double y, int pivot, int minSpeed);
void moveAndKick(struct RoboAI *ai, int speed);
int hasClearPath(int size, double gx, double gy, double ox, double oy, double bx, double by);
int attackMode(struct RoboAI *ai);
int pointObstructed(struct RoboAI *ai, int size, double px, double py);
void findQ(struct RoboAI *ai, double *qx, double *qy, int backoffDist);
int obstAvoid(double *rx, double *ry, double gx, double gy, double ox, double oy, double bx, double by, int size);
void findDefendPoint(struct RoboAI *ai, double *rx, double *ry, int size, int backoffDist);

#endif
