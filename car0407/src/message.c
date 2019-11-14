#include "message.h"
#include "para.h"

/*
	Set the DESTINATION/SOURCE of a message;
	Reset a message;
*/
#ifdef XBEE
void	MsgSetDest(Msg* msg, const Uint32* addr_server)
{
	msg->destH = *addr_server;
	msg->destL = *(addr_server+1);
}
#else
void	MsgSetDest(Msg* msg, const Uint16* addr_server)
{
	msg->dest = *addr_server;
}
#endif

void	MsgSetSrc(Msg* msg)
{
#if defined(XBEE)
	msg->srcH = XBEE_ADDRESS_H;
	msg->srcL = XBEE_ADDRESS_L;
#else
	msg->src = ZIGBEE_ADDRESS;
#endif
}

void	MsgReset(Msg* msg)
{
#if defined(XBEE)
	msg->len = 0;
#else
	msg->len = 0;
	msg->src = 0xFFFF;	// not a possible src addr, to indicate empty message
#endif
}


#if defined(XBEE)
void MsgAppendFrame(Msg *msg,const Uint32* addr_server)
{
	msg->frame[0] = 0x11;
	msg->frame[1] = 0x00;
	msg->frame[2] = (addr_server[0] >> 24) &0xFF;
	msg->frame[3] = (addr_server[0] >> 16) &0xFF;
	msg->frame[4] = (addr_server[0]  >> 8) & 0xFF;
	msg->frame[5] = (addr_server[0] ) & 0xFF;
	msg->frame[6] = (addr_server[1]  >> 24) & 0xFF;
	msg->frame[7] = (addr_server[1] >> 16) & 0xFF;
	msg->frame[8] = (addr_server[1] >> 8) & 0xFF;
	msg->frame[9] = (addr_server[1]) & 0xFF;

	msg->frame[10] = 0x00;
	msg->frame[11] = 0x00;
	msg->frame[12] = 0xE8;
	msg->frame[13] = 0xE8;
	msg->frame[14] = 0x00;
	msg->frame[15] = 0x11;
	msg->frame[16] = 0xC1;
	msg->frame[17] = 0x05;
	msg->frame[18] = 0x00;
	msg->frame[19] = 0x00;

	msg->len = 20;

}

void MsgAppendAddFrame(Msg *msg)
{

    msg->frame[(msg->len) ] = XorSum(msg->frame, (msg->len));
    (msg->len) =(msg->len) +1;

}


#endif
/*
	Transformation between MSG / CMD
*/
void	GetCmd(const Msg* msg, Cmd* cmd)
{	// frame:[CODE][PARA1]...[PARAn][CHECKSUM]
#if defined(XBEE)
	unsigned char cnt_GC = 0;
	if (!XORCheck(msg->frame, msg->len))	// XOR check not passed
	{
		cmd->code = 0;
		return;
	}

	cmd->code = msg->frame[12];
	cmd->srcH = msg->srcH;
	cmd->srcL = msg->srcL;
	cmd->len = (msg->len - 14) >> 1;
	for (cnt_GC = 0; cnt_GC < cmd->len; cnt_GC++)
	{
		cmd->para[cnt_GC] = (msg->frame[13 + (cnt_GC << 1)]) & 0xFF;
		cmd->para[cnt_GC] <<= 8;
		cmd->para[cnt_GC] &= 0xFF00;
		cmd->para[cnt_GC] |= (msg->frame[14 + (cnt_GC << 1)]) & 0xFF;
	}
#else
	unsigned char cnt_GC = 0;
	if (!XORCheck(msg->frame, msg->len))	// XOR check not passed
	{
		cmd->code = 0;
		return;
	}
	
	cmd->code = msg->frame[0];
	cmd->dest = msg->dest;
	cmd->src = msg->src;
	cmd->len = ((msg->len)>>1) - 1;
	for(cnt_GC = 0; cnt_GC < cmd->len; cnt_GC ++)
	{
		cmd->para[cnt_GC] = (msg->frame[1 + (cnt_GC<<1)]) & 0xFF;
		cmd->para[cnt_GC] <<= 8;
		cmd->para[cnt_GC] &= 0xFF00;
		cmd->para[cnt_GC] |= (msg->frame[2 + (cnt_GC<<1)]) & 0xFF;
	}
#endif
}


void	Cmd2Msg(const Cmd* cmd, Msg* msg)
{
#if defined(XBEE)
	unsigned char cnt_CM = 0;
	msg->len = (cmd->len << 1) + 22;
	msg->frame[0] = 0x11;
	msg->frame[1] = 0x00;
	msg->frame[2] = (cmd->destH >> 24) &0xFF;
	msg->frame[3] = (cmd->destH >> 16) &0xFF;
	msg->frame[4] = (cmd->destH >> 8) & 0xFF;
	msg->frame[5] = (cmd->destH) & 0xFF;
	msg->frame[6] = (cmd->destL >> 24) & 0xFF;
	msg->frame[7] = (cmd->destL >> 16) & 0xFF;
	msg->frame[8] = (cmd->destL >> 8) & 0xFF;
	msg->frame[9] = (cmd->destL) & 0xFF;
	msg->frame[10] = 0x00;
	msg->frame[11] = 0x00;
	msg->frame[12] = 0xE8;
	msg->frame[13] = 0xE8;
	msg->frame[14] = 0x00;
	msg->frame[15] = 0x11;
	msg->frame[16] = 0xC1;
	msg->frame[17] = 0x05;
	msg->frame[18] = 0x00;
	msg->frame[19] = 0x00;
	msg->frame[20] = cmd->code;

	for (cnt_CM = 0; cnt_CM < cmd->len; cnt_CM++)
	{
		msg->frame[21 + (cnt_CM << 1)] = ((cmd->para[cnt_CM]) >> 8) & 0xFF;
		msg->frame[22 + (cnt_CM << 1)] = (cmd->para[cnt_CM]) & 0xFF;
	}
	msg->frame[(msg->len) - 1] = XorSum(msg->frame, (msg->len) - 1);
#else
	unsigned char cnt_CM = 0;
	msg->dest = cmd->dest;
	msg->src = cmd->src;
	msg->len = (cmd->len << 1) + 2;
	msg->frame[0] = cmd->code;
	for(cnt_CM = 0; cnt_CM < msg->len; cnt_CM ++)
	{
		msg->frame[1+(cnt_CM<<1)] = ((cmd->para[cnt_CM])>>8) & 0xFF;
		msg->frame[2+(cnt_CM<<1)] = (cmd->para[cnt_CM]) & 0xFF;
	}
	msg->frame[msg->len - 1] = XorSum(msg->frame, msg->len - 1);
#endif
}

/*	
	Calculate the XORSUM of a given string
*/
unsigned char cnt_XS;
unsigned char XorSum(const unsigned char* str, unsigned char len)
{
#if defined(XBEE)
	unsigned int ret_XS = 0;
	for (cnt_XS = 0; cnt_XS < len; cnt_XS++)
		ret_XS +=str[cnt_XS];
	ret_XS=ret_XS%256;
	return (0xFF-ret_XS);
#else
	unsigned char cnt_XS;
	unsigned char ret_XS = 0;
	for(cnt_XS = 0; cnt_XS < len; cnt_XS ++)
		ret_XS ^= str[cnt_XS];
	return ret_XS;
#endif
}

unsigned char XORCheck(const unsigned char* str, unsigned char len)
{
	if(XorSum(str, len) == 0)	// XorSum == 0 --> Pass XOR Check
		return 1;
	else
		return 0;
}

/*
	Get TYPE of a command
*/
unsigned char GetCmdType(Cmd* cmd)
{
	return (cmd->code >> 4) & 0xFF;
}

/*
	Find if message is empty
*/
unsigned char IsMsgEmpty(Msg* msg)
{
#if defined(XBEE)
	if (msg->len == 0)
		return 1;
	else
		return 0;
#else
	if( (msg->len == 0) && (msg->src == 0xFFFF) )
		return 1;
	else 
		return 0;
#endif
}
