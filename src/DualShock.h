#ifndef __DUALSHOCK__H__
#define __DUALSHOCK__H__

#define CIRCLE		0x800000
#define CROSS		0x400000
#define TRIANGLE	0x200000
#define SQUARE		0x100000
#define R1		0x080000
#define R3		0x040000
#define START		0x020000

#define LEFT		0x008000
#define RIGHT		0x004000
#define UP		0x002000
#define DOWN		0x001000
#define L1		0x000800
#define L3		0x000400
#define SELECT		0x000200

#define L2		0x0000E0
#define R2		0x00001C
#define PS		0x000002

extern int circle_button;
extern int cross_button;
extern int triangle_button;
extern int square_button;
extern int left_button;
extern int right_button;
extern int up_button;
extern int down_button;
extern int l1_button;
extern int l2_button;
extern int l3_button;
extern int r1_button;
extern int r2_button;
extern int r3_button;
extern int ps_button;
extern int start_button;
extern int select_button;

extern int joystick_rx;
extern int joystick_ry;
extern int joystick_lx;
extern int joystick_ly;

#define DEAD_BAND	10

int updataState(int *rcv);
void updataButtonState(int button_state1, int button_state2, int button_state3);
void updataJoyStickState(int _jsrx, int _jsry, int _jslx, int _jsly);
int JoyStick_DeadBand(int js, int a);
//int checkSum(int *rcv);

#endif