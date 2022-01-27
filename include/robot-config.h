using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor mLUpper;
extern motor mLLower;
extern motor mRUpper;
extern motor mRLower;
extern gps GPS;
extern motor mConveyor;
extern motor mLTray;
extern motor mArm;
extern digital_out clawPiston;
extern inertial GYRO;
extern motor mRTray;
extern limit limLift;
extern pot potTilter;
extern signature Vision__SIG_1;
extern signature Vision__SIG_2;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );