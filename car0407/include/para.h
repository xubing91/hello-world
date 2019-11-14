#ifndef PARA_H__
#define PARA_H__


#define SENSOR_GPS_UBLOX
/* *********************************************** */
/* specify the quadrotor used: QUAD_GREEN/QUAD_RED */

//#define SWARM_FC_LEADER
#define SWARM_FC_FOLLOWER

//control way
#define Consensus_based
//#define PID
//#define Adaptive
//#define UWB_GUIDANCE
#define v_robot 0.38  //速度
//第一个大圆的圆心和半径
#define cx0 5.25
#define cy0 4.1
#define r0 2.0
//小圆环半径
#define r00 1.0
//分成两个圆时的圆心和半径
#define cx1 7.8
#define cy1 6.0
#define cx2 3.1
#define cy2 2.0
#define r1 1.0

//#define CAR_0
#define CAR_1
//#define CAR_2
//#define CAR_3
//#define CAR_4
//#define CAR_5
//#define CAR_6
//#define CAR_7
//#define CAR_8
//#define CAR_9
//#define CAR_10
//#define CAR_11
//#define CAR_12
//#define CAR_13
//#define CAR_14
//#define CAR_15
#define XBEE
/* specify the control scheme ******************** */
//#define CONTROL_NORMAL	// LF, trajectory, normal
#define CONTROL_DECENT1	// decentrialized formation [XW, D.]

#define DECENT_SCHEME_1	// traditional distributed formation control scheme
#define	DECENT_SCHEME_2 // new distributed formation control scheme
//#define	DECENT_SCHEME_FC	// distributed formation-containment control scheme
/* *********************************************** */

/*
	ADDRESS
*/
#define	N_SWARM		1
#define	N_NEIGHBORS	3
#define GRAVITY 9.8

#if defined(XBEE)

#define ADDRESS_1_H		0x0013A200
#define ADDRESS_1_L		0x41680F4E


#define ADDRESS_2_H		0x0013A200
#define ADDRESS_2_L		0x415C25F0

#define ADDRESS_3_H		0x0013A200
#define ADDRESS_3_L		0x41680F50

#define ADDRESS_4_H		0x0013A200
//#define ADDRESS_4_L		0x415C25F0
#define ADDRESS_4_L		0x415C25DC

#define ADDRESS_5_H		0x0013A200
#define ADDRESS_5_L		0x412745B6

#define ADDRESS_6_H		0x0013A200
#define ADDRESS_6_L		0x415C25DC

#define ADDRESS_7_H		0x0013A200
#define ADDRESS_7_L		0x415C25E2

#define ADDRESS_8_H		0x0013A200
#define ADDRESS_8_L		0x415C25F0

#define ADDRESS_9_H		0x0013A200
//#define ADDRESS_9_L		0x41574574
#define ADDRESS_9_L     0x415C25F0

#define ADDRESS_10_H	0x0013A200
#define ADDRESS_10_L	0x41680F55

#define ADDRESS_SERVER_H	0x0013A200
#define ADDRESS_SERVER_L	0x415C25E2

#define ADDRESS_INVALID_H   0x00000000
#define ADDRESS_INVALID_L   0x00000000

#define	ADDRESS_1	{ADDRESS_1_H,ADDRESS_1_L}
#define	ADDRESS_2	{ADDRESS_2_H,ADDRESS_2_L}
#define ADDRESS_3	{ADDRESS_3_H,ADDRESS_3_L}
#define ADDRESS_4	{ADDRESS_4_H,ADDRESS_4_L}
#define ADDRESS_5  	{ADDRESS_5_H,ADDRESS_5_L}
#define ADDRESS_6	{ADDRESS_6_H,ADDRESS_6_L}
#define ADDRESS_7 	{ADDRESS_7_H,ADDRESS_7_L}
#define	ADDRESS_8	{ADDRESS_8_H,ADDRESS_8_L}
#define	ADDRESS_9	{ADDRESS_9_H,ADDRESS_9_L}
#define	ADDRESS_10	{ADDRESS_10_H,ADDRESS_10_L}
#define ADDRESS_SERVER  {ADDRESS_SERVER_H,ADDRESS_SERVER_L}
#define ADDRESS_INVALID {ADDRESS_INVALID_H,ADDRESS_INVALID_L}

#if defined(CAR_0)
#define FORMATION_INDEX	0
#define FORMATION_INDEX_0
#define BIASX_CMPS	   92.7443
#define BIASY_CMPS	   119.7788
#define BIASZ_CMPS     -24.4591
#define ZIGBEE_ADDRESS ADDRESS_1
#define XBEE_ADDRESS_H ADDRESS_1_H
#define XBEE_ADDRESS_L ADDRESS_1_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_1)
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
//#define BIASX_CMPS	  75.1848
//#define BIASY_CMPS	  75.8457
//#define BIASZ_CMPS    -16.9714
#define BIASX_CMPS	   47.2925
#define BIASY_CMPS	   120.7856
#define BIASZ_CMPS     33.3488
#define ZIGBEE_ADDRESS ADDRESS_2
#define XBEE_ADDRESS_H ADDRESS_2_H
#define XBEE_ADDRESS_L ADDRESS_2_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_2)
#define FORMATION_INDEX	2
#define FORMATION_INDEX_2
//#define BIASX_CMPS	  75.1848
//#define BIASY_CMPS	  75.8457
//#define BIASZ_CMPS    -16.9714
#define BIASX_CMPS	   -73.0847
#define BIASY_CMPS	   148.2541
#define BIASZ_CMPS     -22.1133
#define ZIGBEE_ADDRESS ADDRESS_2
#define XBEE_ADDRESS_H ADDRESS_2_H
#define XBEE_ADDRESS_L ADDRESS_2_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_3)
#define FORMATION_INDEX	3
#define FORMATION_INDEX_3
#define BIASX_CMPS	   -168.2560
#define BIASY_CMPS	   92.7735
#define BIASZ_CMPS     -37.7193
#define ZIGBEE_ADDRESS ADDRESS_3
#define XBEE_ADDRESS_H ADDRESS_3_H
#define XBEE_ADDRESS_L ADDRESS_3_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_4)
#define FORMATION_INDEX	4
#define FORMATION_INDEX_4
//#define BIASX_CMPS	  57.6114
//#define BIASY_CMPS	  97.0959
//#define BIASZ_CMPS    10.7363
#define BIASX_CMPS	  -31.7181
#define BIASY_CMPS	  61.2920
#define BIASZ_CMPS    -71.2941
#define ZIGBEE_ADDRESS ADDRESS_4
#define XBEE_ADDRESS_H ADDRESS_4_H
#define XBEE_ADDRESS_L ADDRESS_4_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_5)
#define FORMATION_INDEX	5
#define FORMATION_INDEX_5
#define BIASX_CMPS	   55.6319
#define BIASY_CMPS	   186.0616
#define BIASZ_CMPS     52.6712
#define ZIGBEE_ADDRESS ADDRESS_5
#define XBEE_ADDRESS_H ADDRESS_5_H
#define XBEE_ADDRESS_L ADDRESS_5_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_6)
#define FORMATION_INDEX	6
#define FORMATION_INDEX_6
#define BIASX_CMPS	   180.3728
#define BIASY_CMPS	   -51.1199
#define BIASZ_CMPS     67.8569
#define ZIGBEE_ADDRESS ADDRESS_6
#define XBEE_ADDRESS_H ADDRESS_6_H
#define XBEE_ADDRESS_L ADDRESS_6_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_7)
#define FORMATION_INDEX	7
#define FORMATION_INDEX_7
#define BIASX_CMPS	  -76.7746
#define BIASY_CMPS	  65.4624
#define BIASZ_CMPS    -21.9831
#define ZIGBEE_ADDRESS ADDRESS_7
#define XBEE_ADDRESS_H ADDRESS_7_H
#define XBEE_ADDRESS_L ADDRESS_7_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_8)
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
//#define BIASX_CMPS	  129.9145
//#define BIASY_CMPS	  72.5138
//#define BIASZ_CMPS    -108.7370
#define BIASX_CMPS	  -31.7181
#define BIASY_CMPS	  61.2920
#define BIASZ_CMPS    -71.2941
#define ZIGBEE_ADDRESS ADDRESS_8
#define XBEE_ADDRESS_H ADDRESS_8_H
#define XBEE_ADDRESS_L ADDRESS_8_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_9)
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
//#define BIASX_CMPS	   27.7749
//#define BIASY_CMPS	   76.0455
//#define BIASZ_CMPS     33.6174
//#define BIASX_CMPS	   -50.4511
//#define BIASY_CMPS	   188.7317
//#define BIASZ_CMPS     -1.0772
//#define BIASX_CMPS	   171.9949
//#define BIASY_CMPS	   -243.6516
//#define BIASZ_CMPS     10.1831
#define BIASX_CMPS	  -31.7181
#define BIASY_CMPS	  61.2920
#define BIASZ_CMPS    -71.2941
#define ZIGBEE_ADDRESS ADDRESS_9
#define XBEE_ADDRESS_H ADDRESS_9_H
#define XBEE_ADDRESS_L ADDRESS_9_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_10)
#define FORMATION_INDEX	7
#define FORMATION_INDEX_7
#define BIASX_CMPS	   171.9949
#define BIASY_CMPS	   -243.6516
#define BIASZ_CMPS     10.1831
#define ZIGBEE_ADDRESS ADDRESS_10
#define XBEE_ADDRESS_H ADDRESS_10_H
#define XBEE_ADDRESS_L ADDRESS_10_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_11)
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
//#define BIASX_CMPS	  75.1848
//#define BIASY_CMPS	  75.8457
//#define BIASZ_CMPS    -16.9714
#define BIASX_CMPS	   -73.3474
#define BIASY_CMPS	   181.7931
#define BIASZ_CMPS     5.1477
#define ZIGBEE_ADDRESS ADDRESS_2
#define XBEE_ADDRESS_H ADDRESS_2_H
#define XBEE_ADDRESS_L ADDRESS_2_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_12)
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
//#define BIASX_CMPS	  75.1848
//#define BIASY_CMPS	  75.8457
//#define BIASZ_CMPS    -16.9714
#define BIASX_CMPS	   5.2876
#define BIASY_CMPS	   167.4825
#define BIASZ_CMPS     71.8823
#define ZIGBEE_ADDRESS ADDRESS_2
#define XBEE_ADDRESS_H ADDRESS_2_H
#define XBEE_ADDRESS_L ADDRESS_2_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_13)
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
//#define BIASX_CMPS	  75.1848
//#define BIASY_CMPS	  75.8457
//#define BIASZ_CMPS    -16.9714
#define BIASX_CMPS	   11.4740
#define BIASY_CMPS	   63.7911
#define BIASZ_CMPS     -45.9056
#define ZIGBEE_ADDRESS ADDRESS_2
#define XBEE_ADDRESS_H ADDRESS_2_H
#define XBEE_ADDRESS_L ADDRESS_2_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_14)
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
//#define BIASX_CMPS	  75.1848
//#define BIASY_CMPS	  75.8457
//#define BIASZ_CMPS    -16.9714
#define BIASX_CMPS	   17.0448
#define BIASY_CMPS	   129.4591
#define BIASZ_CMPS     5.5831
#define ZIGBEE_ADDRESS ADDRESS_2
#define XBEE_ADDRESS_H ADDRESS_2_H
#define XBEE_ADDRESS_L ADDRESS_2_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_15)
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
//#define BIASX_CMPS	   -43.5140
//#define BIASY_CMPS	   140.8983
//#define BIASZ_CMPS     75.6466
//#define BIASX_CMPS	  99.7287
//#define BIASY_CMPS	  127.4815
//#define BIASZ_CMPS    -104.1158
#define BIASX_CMPS	  -73.0847
#define BIASY_CMPS	  148.2541
#define BIASZ_CMPS    22.1133
#define ZIGBEE_ADDRESS ADDRESS_2
#define XBEE_ADDRESS_H ADDRESS_2_H
#define XBEE_ADDRESS_L ADDRESS_2_L
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#endif
#else
#define	ADDRESS_1		0x0001
#define	ADDRESS_2		0x3CB8
#define ADDRESS_3		0x287B
#define ADDRESS_4		0x143E
#define ADDRESS_5  		0x6532
#define ADDRESS_6		0x143F
#define ADDRESS_7 		0x179C
#define	ADDRESS_8		0xFFFE
#define	ADDRESS_9		0xFFFE
#define	ADDRESS_10		0xFFFE
#define	ADDRESS_INVALID	0xFFFE


#define	ADDRESS_SERVER		0x0000
#define	ADDRESS_BROADCAST	0xFFFF
#define	ADDRESS_FOLLOWER	ADDRESS_1
#define	ADDRESS_LEADER		ADDRESS_2

#if defined(CAR_1)
#define FORMATION_INDEX	0
#define FORMATION_INDEX_0
#define BIASX_CMPS	   -6.9185
#define BIASY_CMPS	   150.3188
#define BIASZ_CMPS     -21.0080
#define ZIGBEE_ADDRESS ADDRESS_1
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_2)//9#
#define FORMATION_INDEX	1
#define FORMATION_INDEX_1
#define BIASX_CMPS	   38.9335
#define BIASY_CMPS	   78.0094
#define BIASZ_CMPS     -58.1269
#define ZIGBEE_ADDRESS ADDRESS_2
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID, ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_3)
#define FORMATION_INDEX	2
#define FORMATION_INDEX_2
#define BIASX_CMPS	   66.1242
#define BIASY_CMPS	   56.5556
#define BIASZ_CMPS    -81.2929
#define ZIGBEE_ADDRESS ADDRESS_3
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_4)
#define FORMATION_INDEX	3
#define FORMATION_INDEX_3
#define BIASX_CMPS	  -98.3370
#define BIASY_CMPS	  250.9283
#define BIASZ_CMPS    -90.6326
#define ZIGBEE_ADDRESS ADDRESS_4
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_YELLOW,ADDRESS_BLACK}
#elif defined(CAR_5)
#define FORMATION_INDEX	4
#define FORMATION_INDEX_4
#define BIASX_CMPS	   35.4978
#define BIASY_CMPS	   94.9516
#define BIASZ_CMPS     -54.3652
#define ZIGBEE_ADDRESS ADDRESS_5
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_6)
#define FORMATION_INDEX	5
#define FORMATION_INDEX_5
#define BIASX_CMPS	   55.2866
#define BIASY_CMPS	   51.0658
#define BIASZ_CMPS     -42.7639
#define ZIGBEE_ADDRESS ADDRESS_6
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_7)
#define FORMATION_INDEX	6
#define FORMATION_INDEX_6
#define BIASX_CMPS	   57.9790
#define BIASY_CMPS	   -12.8008
#define BIASZ_CMPS     -62.5258
#define ZIGBEE_ADDRESS ADDRESS_7
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_8)
#define FORMATION_INDEX	7
#define FORMATION_INDEX_7
#define BIASX_CMPS	   121.1288
#define BIASY_CMPS	   24.7048
#define BIASZ_CMPS     -114.2433
#define ZIGBEE_ADDRESS ADDRESS_8
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_9)
#define FORMATION_INDEX	8
#define FORMATION_INDEX_8
#define BIASX_CMPS	   23.9758
#define BIASY_CMPS	   70.2723
#define BIASZ_CMPS     -55.9816
#define ZIGBEE_ADDRESS ADDRESS_9
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#elif defined(CAR_10)
#define FORMATION_INDEX	9
#define FORMATION_INDEX_9
#define BIASX_CMPS	   57.2085
#define BIASY_CMPS	   87.4940
#define BIASZ_CMPS     -117.2102
#define ZIGBEE_ADDRESS ADDRESS_10
#define ADDRESS_NEIGHBORS	{ADDRESS_INVALID,ADDRESS_INVALID,ADDRESS_INVALID}
#endif
#endif


#define MAG_SCALING_ON	// scaling on
#define SCALEX_CMPS	1
#define	SCALEY_CMPS 1
#define SCALEZ_CMPS	1
#define BIAS_ACC
#define BIASX_ACC	0
#define BIASY_ACC	0
#define BIASZ_ACC	0

/* 	
	Message Code	
*/
#define	CMD_MESSAGE_TYPE	2///////////234567891011
#define	CMD_MESSAGE_FORMATIONFLIGHT	0x20
#define	CMD_SETPARA_TYPE	3
#define	CMD_SETPARA_PID			0x30
#define	CMD_SETPARA_CONSENSUS 	0x31
#define	CMD_SETPARA_ADAPTIVE	0x32
#define	CMD_SETDEST_TYPE	4
#define	CMD_SETDEST_LINEAB	0x40// line, absolute position
#define	CMD_SETDEST_LINERE	0x41// line, relative position
#define	CMD_SETDEST_ARCAB	0x42// arc, absolute position
#define CMD_SETDEST_ARC_RING 0x43
#define CMD_SETDEST_ARC_DOUBLE 0x44
#define CMD_SETDEST_GUIDANCE 0x45
#define	CMD_SETDEST_EIGHTAB	0x46// 8, absolute position
#define	CMD_SETDEST_SWARMAB	0x48// swarm on a circle, line, absolute position
#define CMD_SETDEST_BUTTERFLY 0x49
#define CMD_SETDEST_SWARM_RING 0x4A
#define CMD_SETDEST_SWARM_DOUBLE 0x4B
#define CMD_SETDEST_SWARM_ONE 0x4C
#define CMD_SETDEST_SWARM_TWO 0x4D
#define CMD_SETDEST_SWARM_THREE 0x4E
//#define	CMD_SETDEST_TYPE	4
//#define	CMD_SETDEST_LINEAB	0x40// line, absolute position
//#define	CMD_SETDEST_LINERE	0x41// line, relative position
//#define	CMD_SETDEST_ARCAB	0x42// arc, absolute position
//#define	CMD_SETDEST_ARCRE	0x43// arc, relative position
//#define	CMD_SETDEST_TRIANGLE_INV	0x45// butterfly(8), relative position(usu. [0 0])
//#define	CMD_SETDEST_EIGHTAB	0x46// 8, absolute position
//#define	CMD_SETDEST_TRIANGLE_VAR	0x47// 8, relative position
//#define	CMD_SETDEST_SWARMAB	0x48// swarm on a circle, line, absolute position
//#define CMD_SETDEST_BUTTERFLY 0x49
//#define CMD_SETDEST_TRIANGLE_ROUND 0x4A
////#define CMD_SETDEST_SQURAE_SQURAE 0x4B
//#define CMD_SETDEST_SWARM_LINE 0x4B
//#define CMD_SETDEST_SWARM_SQURAE 0x4C
//#define CMD_SETDEST_SQURAEAB 0x4D
#define CMD_SETREF_TYPE		0xD
#define	CMD_SETREF_LINEAB	0xD0// line, absolute position
#define	CMD_SETREF_LINERE	0xD1// line, relative position
#define	CMD_SETREF_ARCAB	0xD2// arc, absolute position
#define	CMD_SETREF_ARCRE	0xD3// arc, relative position
#define	CMD_SETREF_BUTTERFLY	0xD5// butterfly(8), relative position(usu. [0 0])
#define	CMD_SETREF_EIGHTAB	0xD6// 8, absolute position
#define	CMD_SETREF_EIGHTRE	0xD7// 8, absolute position
#define CMD_SETSTATE_TYPE	5
#define CMD_SETSTATE_LAND	0x50	// land
#define	CMD_SETSTATE_HALT	0x51	// halt trajectory, hover where I am
#define	CMD_SETSTATE_TAKEOFF	0x52	// take off
#define	CMD_FORMATION_TYPE	6
#define	FORMLF_LEADER_FORM	0x60	// leader sends its state
#define	CMD_FORMD_TYPE		7
#define FORM_DECENTRALIZED_LINEAR1	0x70	// decentralized formation, linear, type 1
#define CMD_SETDFPARA_TYPE	8		// set decentralized formation type
#define	CMD_SETDFPARA_K		0x80
#define	CMD_SETVAR_TYPE		9		// set variables directly
#define CMD_SETVAR_ROTOR	0x90
#define	CMD_SETVAR_COMRATE	0x91	// communication rate
#define CMD_SETCOMTOPO_SCHEME	0x92	// set scheme of communication topology
#define	CMD_SETVAR_CENTERPOSITION	0x93
#define	CMD_CHIRP_TYPE		10		// for test
#define	CMD_CHIRP_SETPARA	0xA0	// set parameters for chirp & start chirp
#define	CMD_SETSCHEDULE_TYPE	11	// set flight schedule
#define	CMD_SETSCHEDULE_ON		0xB0// turn on the schedule

#define CMD_DESCENDDEBUG_TYPE	0xE
#define	CMD_GPSDEBUG_ON		0xE1
#define	CMD_GPSDEBUG_OFF	0xE2




///*
//	constants in constants.c
//*/
extern const double fATTSET;
extern const double PITCH_SAT;
extern const double DTIME_INNERLOOP;
//extern const double v_robot = 0.38;  //速度
////第一个大圆的圆心和半径
//extern const double cx0 = 5.25;
//extern const double cy0 = 4.1;
//extern const double r0 = 2.0;
////小圆环半径
//extern const double r00 = 1.0;
////分成两个圆时的圆心和半径
//extern const double cx1 = 7.8;
//extern const double cy1 = 6.0;
//extern const double cx2 = 3.1;
//extern const double cy2 = 2.0;
//extern const double r1 = 1.0;
//分成四个圆时的半径
// extern const double r2;
#endif

