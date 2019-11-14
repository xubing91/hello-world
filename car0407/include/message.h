/*
	Define the data structure of messages flowing on the network
*/
#include "para.h"
#ifndef MESSAGE_H__
#define MESSAGE_H__
#include "DSP2833x_Device.h"

// Msg: message sent through wireless network
#if defined(XBEE)
typedef struct {	


	Uint32 destH;	//Ŀ���ַ4����λ�ֽ�
	Uint32 destL;	//Ŀ���ַ4����λ�ֽ�
	Uint32 srcH;	//
	Uint32 srcL;
	Uint16 len;		// �غɳ���
	unsigned char frame[90];	// payload

} Msg;
#else
typedef struct {
	unsigned char len;		// �غɳ���
	unsigned char frame[60];	// payload
	Uint16	dest;			// Ŀ���ַ
	Uint16	src;
} Msg;
#endif

// Cmd: command. can be the payload of a message
#if defined(XBEE)
typedef struct{
    unsigned char code;     // ָ����
    int16   para[9];        // ����
    Uint32  destH;          // Ŀ���ַ��λ4Bytes
    Uint32  destL;          // Ŀ���ַ��λ4Bytes
    Uint32  srcH;           // Դ��ַ��λ4Bytes
    Uint32  srcL;           // Դ��ַ��λ4Bytes
    Uint32  len;            // ��������
} Cmd;
#else
typedef	struct{	
	unsigned char code;		// ָ����
	int16	para[9];		// ����
	Uint16	dest;			// Ŀ���ַ
	Uint16	src;			// Դ��ַ
	unsigned char len;		// ��������
} Cmd;
#endif

/*
	Set the DESTINATION/SOURCE of a message;
	Reset a message;
*/
#ifdef XBEE
void	MsgSetDest(Msg* msg, const Uint32* addr_server);
#else
void	MsgSetDest(Msg* msg, const Uint16* addr_server);
#endif
void	MsgSetSrc(Msg* msg);
void	MsgReset(Msg* msg);

/*
	Transformation between MSG / CMD
*/
void	GetCmd(const Msg* msg, Cmd* cmd);
void	Cmd2Msg(const Cmd* cmd, Msg* msg);

/*	
	Calculate the XORSUM of a given string
*/
unsigned char XorSum(const unsigned char* str, unsigned char len);//������
unsigned char XORCheck(const unsigned char* str, unsigned char len);//�����ӽ�����

/*
	Get TYPE of a command
*/
unsigned char GetCmdType(Cmd* cmd);
void MsgAppendAddFrame(Msg *msg);
/*
	Find if message is empty
*/
unsigned char IsMsgEmpty(Msg* msg);
void MsgAppendFrame(Msg *msg,const Uint32* addr_server);
#endif
