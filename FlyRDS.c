// FLY7541Radio.cpp : Defines the entry point for the DLL application.
#include <fcntl.h>  
#include <errno.h>  
//C For FlyRadio RDS
//Author : FlyChan
//Date   : 2012/11/21
#include <termios.h>
#include <stdio.h>
#include <pthread.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/select.h>
#include <sys/types.h> 
#include <cutils/atomic.h>
#include <semaphore.h>
#include <cutils/log.h> 
#include <asm/termbits.h>
#include <hardware/hardware.h>  

#include "../../include/types_def.h"
#include "../../include/ShareMemoryStruct.h"

#include "FlyRDS.h"
#include "FlyRadio.h"
#include "FlyRadio_Hardware.h"
#include "TDA7541.h"
#include "TEF6624.h"
#include "SAF7741_Radio.h"
#include "../../include/commonFunc.h"
#include "../../include/allInOneOthers.h"

extern struct flyradio_struct_info *pFlyRadioInfo;
extern t_bRadio_SignalGood p_bRadio_SignalGood;
extern t_Radio_Mute p_Radio_Mute;
extern t_Radio_Set_Freq p_Radio_Set_Freq;

static int add_af_to_list(struct flyradio_struct_info *pFlyRadioInfo,BYTE temp_freq);

#if RADIO_RDS

char *rds_di_table[8] =	
{
	"0---->Mono",
	"1---->Stereo",
	"2---->Not Used",
	"3---->Stereo && Artificial head",
	"4---->Mono && Compressed",
	"5---->Stereo && Compressed",
	"6---->Not Used",
	"7---->Stereo && Artificial head && Compressed"
};

char *rds_pty_table[32] =	
{
	"0---->None",
	"1---->News",
	"2---->Current Affairs",
	"3---->Information",
	"4---->Sport",
	"5---->Education",
	"6---->Drama",
	"7---->Cultures",
	"8---->Science",
	"9---->Varied Speech",
	"10---->Pop Music",
	"11---->Rock Music",
	"12---->Easy Listening",
	"13---->Light Classics M",
	"14---->Serious Classics",
	"15---->Other Music",
	"16---->Weather & Metr",
	"17---->Finance",
	"18---->Children’s Progs",
	"19---->Social Affairs",
	"20---->Religion",
	"21---->Phone In",
	"22---->Travel & Touring",
	"23---->Leisure & Hobby ",
	"24---->Jazz Music",
	"25---->Country Music",
	"26---->National Music",
	"27---->Oldies Music",
	"28---->Folk Music",
	"29---->Documentary",
	"30---->Alarm Test",
	"31---->Alarm - Alarm !"
};

static BYTE odd_check(BYTE data)
{		  
	BYTE i;
	BYTE p = 0;
	for(i=0;i<8;i++)
	{
		if(data&(1<<(i)))
			p++;
	}
	return p%2;
}

static UINT16 RDS_check_crc(UINT16 rds_raw_data)
{
	BYTE bCFlag = 1;
	BYTE TempH, TempL, ACC;

	UINT16 crc = 0;

	TempH = rds_raw_data / 0x100;
	TempL = rds_raw_data % 256;

	//C9
	ACC = TempH & 0x7c;
	bCFlag = odd_check(ACC);
	ACC = TempL & 0x3e;
	bCFlag ^= odd_check(ACC);
	if(bCFlag) crc = 0x200;

	//C8
	ACC = TempH & 0x3e;
	bCFlag = odd_check(ACC);				
	ACC = TempL & 0x1f;
	bCFlag ^= odd_check(ACC);
	if(bCFlag) crc |= 0x100;

	//C7
	ACC = TempH & 0x63;
	bCFlag = odd_check(ACC);
	ACC = TempL & 0x31;
	bCFlag ^= odd_check(ACC);		
	if(bCFlag) crc |= 0x80;

	//C6
	ACC = TempH & 0xcd;
	bCFlag = odd_check(ACC);
	ACC = TempL & 0xa6;
	bCFlag ^= odd_check(ACC);
	if(bCFlag)crc |= 0x40;

	//C5
	ACC = TempH & 0xe6;
	bCFlag = odd_check(ACC);
	ACC = TempL & 0xd3;
	bCFlag ^= odd_check(ACC);
	if(bCFlag)crc |= 0x20;

	//C4
	ACC = TempH & 0x8f;
	bCFlag = odd_check(ACC);
	ACC = TempL & 0x57;
	bCFlag ^= odd_check(ACC);
	if(bCFlag) crc |= 0x10;

	//C3
	ACC = TempH & 0x3b;
	bCFlag = odd_check(ACC);			 
	ACC = TempL & 0x95;
	bCFlag ^= odd_check(ACC);
	if(bCFlag)crc |= 0x08;

	//C2
	ACC = TempH & 0xe1;
	bCFlag = odd_check(ACC);
	ACC = TempL & 0xf4;
	bCFlag ^= odd_check(ACC);
	if(bCFlag)crc |= 0x04;

	//C1
	ACC = TempH & 0xf0;
	bCFlag = odd_check(ACC);
	ACC = TempL & 0xfa;
	bCFlag ^= odd_check(ACC);
	if(bCFlag) crc |= 0x02;

	//C0
	ACC = TempH & 0xf8;
	bCFlag = odd_check(ACC);
	ACC = TempL & 0x7d;
	bCFlag ^= odd_check(ACC);
	if(bCFlag) crc |= 0x01;

	return crc;
}

void RDS_Decode(DWORD data)
{
	BYTE i = 0;

	UINT16 temp_crc_from_raw_data_info = 0;

	for (i=32;i>0;i--)
	{
		pFlyRadioInfo->RDSInfo.rds_raw_data_info <<= 1;
		if (pFlyRadioInfo->RDSInfo.rds_raw_data_crc & 0x0200)
		{
			pFlyRadioInfo->RDSInfo.rds_raw_data_info |= 0x0001;
		}

		pFlyRadioInfo->RDSInfo.rds_raw_data_crc <<= 1;
		if (data & (1<<(i-1)))
		{
			pFlyRadioInfo->RDSInfo.rds_raw_data_crc |= 0x0001;
		}
		pFlyRadioInfo->RDSInfo.rds_raw_data_crc &= 0x03FF;

		pFlyRadioInfo->RDSInfo.rds_raw_date_bit_cnt++;

		if (FALSE == pFlyRadioInfo->RDSInfo.rds_raw_data_Synch)
		{
			temp_crc_from_raw_data_info = RDS_check_crc(pFlyRadioInfo->RDSInfo.rds_raw_data_info);
			temp_crc_from_raw_data_info ^= pFlyRadioInfo->RDSInfo.rds_raw_data_crc;

			//DBG0(debugPrintf("\nrds_crc-->%d",temp_crc_from_raw_data_info);)

			if (temp_crc_from_raw_data_info == OFFSET_A)
			{
				DBG1(debugPrintf("\nrds_raw_data_info-->%d",pFlyRadioInfo->RDSInfo.rds_raw_data_info);)
				DBG1(debugPrintf("  rds_CRC-->%d",pFlyRadioInfo->RDSInfo.rds_raw_data_crc);)
				pFlyRadioInfo->RDSInfo.rds_raw_data_seq = 1;
				pFlyRadioInfo->RDSInfo.rds_raw_data_Synch = TRUE;
				pFlyRadioInfo->RDSInfo.rds_raw_date_bit_cnt = 0;

				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][0] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info>>8);
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][1] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info);
			}
		}
		else if (26 == pFlyRadioInfo->RDSInfo.rds_raw_date_bit_cnt)
		{
			pFlyRadioInfo->RDSInfo.rds_raw_date_bit_cnt = 0;

			temp_crc_from_raw_data_info = RDS_check_crc(pFlyRadioInfo->RDSInfo.rds_raw_data_info);
			temp_crc_from_raw_data_info ^= pFlyRadioInfo->RDSInfo.rds_raw_data_crc;

			if ((temp_crc_from_raw_data_info == OFFSET_A) && (pFlyRadioInfo->RDSInfo.rds_raw_data_seq == 0))
			{
				pFlyRadioInfo->RDSInfo.rds_raw_data_seq = 1;
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][0] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info>>8);
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][1] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info);
			}
			else if ((temp_crc_from_raw_data_info == OFFSET_B) && (pFlyRadioInfo->RDSInfo.rds_raw_data_seq == 1))
			{
				pFlyRadioInfo->RDSInfo.rds_raw_data_seq = 2;
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][2] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info>>8);
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][3] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info);
			}
			else if ((temp_crc_from_raw_data_info == OFFSET_C) && (pFlyRadioInfo->RDSInfo.rds_raw_data_seq == 2))
			{
				pFlyRadioInfo->RDSInfo.rds_raw_data_seq = 3;
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][4] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info>>8);
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][5] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info);
			}
			else if ((temp_crc_from_raw_data_info == OFFSET_C2) && (pFlyRadioInfo->RDSInfo.rds_raw_data_seq == 2))
			{
				pFlyRadioInfo->RDSInfo.rds_raw_data_seq = 3;
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][4] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info>>8);
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][5] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info);
			}
			else if ((temp_crc_from_raw_data_info == OFFSET_D) && (pFlyRadioInfo->RDSInfo.rds_raw_data_seq == 3))
			{
				pFlyRadioInfo->RDSInfo.rds_raw_data_seq = 0;
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][6] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info>>8);
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx++][7] = (BYTE)(pFlyRadioInfo->RDSInfo.rds_raw_data_info);

				if (pFlyRadioInfo->rds_data_hx >= (RDS_DATA_BUF_MAX - 1))
				{
					pFlyRadioInfo->rds_data_hx = 0;
				}
			}
			else
			{
				pFlyRadioInfo->RDSInfo.rds_raw_data_Synch = FALSE;
			}
		}
	}
}

void returnRDS_PS(BYTE *p)//20100427	 频道名称
{
	unsigned short int sWideChar[16];
	UINT iWideCharLength;

	BYTE buff[1+16+2];
	buff[0] = RADIORDS_ID_PS;

	iWideCharLength = gb2312_to_unicode(sWideChar,p,8);
	memcpy(&buff[1],sWideChar,iWideCharLength);
	buff[1+iWideCharLength] = 0;
	buff[1+iWideCharLength+1] = 0;
	flyRadioReturnToUserPutToBuff(buff,1+iWideCharLength+2);
}

void returnRDS_RT(BYTE *p,BYTE len) //RadioText
{
	unsigned short int sWideChar[128];
	UINT iWideCharLength;

	BYTE buff[1+128+2];
	buff[0] = RADIORDS_ID_RT;
	
	iWideCharLength = gb2312_to_unicode(sWideChar,p,len);

	memcpy(&buff[1],sWideChar,iWideCharLength);
	buff[1+iWideCharLength] = 0;
	buff[1+iWideCharLength+1] = 0;
	flyRadioReturnToUserPutToBuff(buff,1+iWideCharLength+2);
}

void returnRDS_TA(BYTE TA)
{
	BYTE buff[2] = {RADIORDS_ID_TA,0x00};

	buff[1] = TA;

	flyRadioReturnToUserPutToBuff(buff,2);
}

void returnRDS_TP(BYTE TP)
{
	BYTE buff[2] = {RADIORDS_ID_TP,0x00};

	buff[1] = TP;

	flyRadioReturnToUserPutToBuff(buff,2);
}

void returnRDS_Date(void)
{

}
void returnRDS_PTYN(BYTE *p)
{

}

int rds_get_check_updata_PI(struct flyradio_struct_info *pFlyRadioInfo)
{
	UINT16 temp;
	temp  = (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][0] << 8) | pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][1];        

	if(pFlyRadioInfo->RDSInfo.New_PI_Code != temp)
	{
		pFlyRadioInfo->RDSInfo.New_PI_Code = temp;

#if RADIO_RDS_DEBUG_MSG
		debugPrintf("\nrds PI---->%d",temp);
#endif
	}

	if(0 == pFlyRadioInfo->RDSInfo.Base_PI_Code)//确定基准PI
	{
		pFlyRadioInfo->RDSInfo.Base_PI_Code = temp;
		add_af_to_list(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.ibasefreq);

		//PostSignal(&pFlyRadioInfo->RDSAFThreadMutex,&pFlyRadioInfo->RDSAFThreadCond,&pFlyRadioInfo->bRDSAFThreadRunAgain);
	}

	if ((pFlyRadioInfo->RDSInfo.Base_PI_Code != pFlyRadioInfo->RDSInfo.New_PI_Code) || (0 == pFlyRadioInfo->RDSInfo.Base_PI_Code))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void get_updata_PTY(struct flyradio_struct_info *pFlyRadioInfo)
{
	BYTE buff[] = {RADIORDS_ID_PTY,0x00};
	BYTE temp;
	temp  = ((pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][2] & 0x03) << 3) | (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][3] >> 5); 

	if(pFlyRadioInfo->RDSInfo.Program_Type != temp)
	{
		pFlyRadioInfo->RDSInfo.Program_Type = temp;
		buff[1] = temp;
		flyRadioReturnToUserPutToBuff(buff,2);

#if RADIO_RDS_DEBUG_MSG
		debugPrintf("\nrds pty-->");
		debugPrintf(rds_pty_table[temp]);
#endif
	}
}

void get_updata_TP(struct flyradio_struct_info *pFlyRadioInfo)
{	
	BOOL bswitch;

	bswitch  = (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][2] >> 2) & 0x01;          

	if(pFlyRadioInfo->RDSInfo.Traffic_Program_id_Tuned != bswitch)
	{
		pFlyRadioInfo->RDSInfo.Traffic_Program_id_Tuned = bswitch;
		returnRDS_TP(bswitch);
#if RADIO_RDS_DEBUG_MSG
		debugPrintf("\nrds TP_TUNED FLAG---->%d",pFlyRadioInfo->RDSInfo.Traffic_Program_id_Tuned);
#endif
	}

	if (14 == pFlyRadioInfo->RDSInfo.rds_group_type)
	{
		bswitch  = (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][3] >> 4) & 0x01;

		if(pFlyRadioInfo->RDSInfo.Traffic_Program_id_Other != bswitch)
		{
			pFlyRadioInfo->RDSInfo.Traffic_Program_id_Other = bswitch;
#if RADIO_RDS_DEBUG_MSG
			debugPrintf("\nrds TP_OTHER FLAG---->%d",pFlyRadioInfo->RDSInfo.Traffic_Program_id_Other);
#endif
		}
	}
} 

void get_updata_MS(struct flyradio_struct_info *pFlyRadioInfo)
{
	BOOL bswitch;
	bswitch = (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][3] >> 3) & 0x01;

	if(pFlyRadioInfo->RDSInfo.Music_Speech_Switch != bswitch)
	{
		pFlyRadioInfo->RDSInfo.Music_Speech_Switch = bswitch;
#if RADIO_RDS_DEBUG_MSG
		if (bswitch)
		{
			debugPrintf("\nrds MS---->Music");
		}
		else
		{
			debugPrintf("\nrds MS---->Speech");
		}
#endif
	}
}

void get_updata_DI(struct flyradio_struct_info *pFlyRadioInfo)
{
	BOOL benable;
	benable = (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][3] >> 2) & 0x01;

	if (0 == pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af)
	{
		pFlyRadioInfo->RDSInfo.rds_di_segment = 0;
	}

	if(pFlyRadioInfo->RDSInfo.rds_di_segment == pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af)
	{
		if(pFlyRadioInfo->RDSInfo.tempDI[pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af] != benable)
		{

			pFlyRadioInfo->RDSInfo.tempDI[pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af] = benable;
		}

		if (3 == pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af)
		{
			pFlyRadioInfo->RDSInfo.Decoder_id = ((pFlyRadioInfo->RDSInfo.tempDI[0] << 3) | (pFlyRadioInfo->RDSInfo.tempDI[1] << 2) | (pFlyRadioInfo->RDSInfo.tempDI[2] << 1) | (pFlyRadioInfo->RDSInfo.tempDI[3] << 0));
#if RADIO_RDS_DEBUG_MSG
			debugPrintf("\nrds DI-->");
			debugPrintf(rds_di_table[pFlyRadioInfo->RDSInfo.Decoder_id]);
#endif	
		}

		pFlyRadioInfo->RDSInfo.rds_di_segment++;
	}
}

void get_updata_TA(struct flyradio_struct_info *pFlyRadioInfo)	 
{
	BOOL flag;

	if (0 == pFlyRadioInfo->RDSInfo.rds_group_type)
	{
		flag = ((pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][3] >> 4) & 0x01);

		if(pFlyRadioInfo->RDSInfo.Traffic_Announcement_id_Tuned != flag)
		{
			pFlyRadioInfo->RDSInfo.Traffic_Announcement_id_Tuned = flag;
			returnRDS_TA(flag);
#if RADIO_RDS_DEBUG_MSG
			debugPrintf("\nrds TA_TUNED FLAG---->%d",pFlyRadioInfo->RDSInfo.Traffic_Announcement_id_Tuned);
#endif
			if (pFlyRadioInfo->RDSInfo.Traffic_Program_id_Tuned && pFlyRadioInfo->RDSInfo.Traffic_Announcement_id_Tuned)
			{
				pFlyAllInOneInfo->pMemory_Share_Common->bRadioRDS_Traffic_Broadcast = TRUE;
			}
			else
			{
				pFlyAllInOneInfo->pMemory_Share_Common->bRadioRDS_Traffic_Broadcast = FALSE;
			}

			ipcStartEvent(EVENT_RADIO_RDS_TRAFFIC_BOARDCAST_ID);
		}
	}
	else if((14 == pFlyRadioInfo->RDSInfo.rds_group_type) && (GROUP_B == pFlyRadioInfo->RDSInfo.rds_group))
	{
		flag = ((pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][3] >> 3) & 0x01);

		if(pFlyRadioInfo->RDSInfo.Traffic_Announcement_id_Other != flag)
		{
			pFlyRadioInfo->RDSInfo.Traffic_Announcement_id_Other = flag;
#if RADIO_RDS_DEBUG_MSG
			debugPrintf("\nrds TA_OTHER FLAG---->%d",pFlyRadioInfo->RDSInfo.Traffic_Announcement_id_Other);
#endif
		}
	}
}

void get_updata_PS(struct flyradio_struct_info *pFlyRadioInfo,BYTE tempGroup)
{
	//BYTE tempGroup;
	//tempGroup = pFlyRadioInfo->RDSInfo.rds_group;

	if (0 == pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af)
	{
		pFlyRadioInfo->RDSInfo.rds_ps_segment[tempGroup] = 0;
	}
	
	DBG0(debugPrintf("\nrds pFlyRadioInfo->RDSInfo.rds_ps_segment[tempGroup]-->%d",pFlyRadioInfo->RDSInfo.rds_ps_segment[tempGroup]);)

	if(pFlyRadioInfo->RDSInfo.rds_ps_segment[tempGroup] == pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af)
	{
		pFlyRadioInfo->RDSInfo.preProgram_Service_name[tempGroup][pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af * 2 + 0] = pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][6];
		pFlyRadioInfo->RDSInfo.preProgram_Service_name[tempGroup][pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af * 2 + 1] = pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][7];
	
		if (3 == pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af)
		{
			//if (0 != memcmp(pFlyRadioInfo->RDSInfo.preProgram_Service_name[tempGroup], pFlyRadioInfo->RDSInfo.curProgram_Service_name[tempGroup], 8))
			{
				memcpy(pFlyRadioInfo->RDSInfo.curProgram_Service_name[tempGroup], pFlyRadioInfo->RDSInfo.preProgram_Service_name[tempGroup], 8);

#if RADIO_RDS_DEBUG_MSG
				DBG0(debugPrintf("\nFlyRadio RDS PS Group-->%d",tempGroup);)
				DBG0(debugPrintf(pFlyRadioInfo->RDSInfo.curProgram_Service_name[tempGroup]);)
#endif
			}

			returnRDS_PS(pFlyRadioInfo->RDSInfo.curProgram_Service_name[tempGroup]);	
		}
		pFlyRadioInfo->RDSInfo.rds_ps_segment[tempGroup]++;
	}
}

void get_updata_RT(struct flyradio_struct_info *pFlyRadioInfo)
{
	BYTE tempGroup;
	BYTE tempRT_G;
	BYTE temp_RT_SEG;
	BYTE temp;
	BYTE i;
	tempGroup = pFlyRadioInfo->RDSInfo.rds_group;
	tempRT_G = GET_RT_GROUP(pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx]);
	temp_RT_SEG = GET_SEG_RT(pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx]);

	DBG0(debugPrintf("\nrds rt group-->%d",tempRT_G);)
	DBG0(debugPrintf(" seg-->%d",temp_RT_SEG);)

	if (GROUP_A == tempGroup)
	{
		if (0 == temp_RT_SEG)
		{
			pFlyRadioInfo->RDSInfo.rds_rt_segment_a = 0;
		}

		if (tempRT_G != pFlyRadioInfo->RDSInfo.rds_rt_A_G)
		{
			pFlyRadioInfo->RDSInfo.rds_rt_A_G = tempRT_G;
			pFlyRadioInfo->RDSInfo.rds_rt_segment_a = 0;
		}
	}
	else if (GROUP_B == tempGroup)
	{
		if (0 == temp_RT_SEG)
		{
			pFlyRadioInfo->RDSInfo.rds_rt_segment_b = 0;
		}
	}

	if (GROUP_A == tempGroup)
	{
		if (pFlyRadioInfo->RDSInfo.rds_rt_segment_a == temp_RT_SEG)
		{
			temp = temp_RT_SEG * 4;
			for (i = 0; i<4; i++)
			{
				pFlyRadioInfo->RDSInfo.preRadio_Text_A[temp + i] = pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][4 + i];
				if ((0x0D == pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][4 + i]) || (temp + i == 63))
				{
					if (0 != memcmp(pFlyRadioInfo->RDSInfo.preRadio_Text_A, pFlyRadioInfo->RDSInfo.curRadio_Text_A, temp + i + 1))
					{
						memcpy(pFlyRadioInfo->RDSInfo.curRadio_Text_A, pFlyRadioInfo->RDSInfo.preRadio_Text_A, temp + i + 1);
					}
#if RADIO_RDS_DEBUG_MSG
					debugPrintf("\nrds RT Group-->%d",tempRT_G);
					debugPrintf(" count-->%d",temp + i + 1);
					pFlyRadioInfo->RDSInfo.curRadio_Text_A[temp + i + 1] = '\0';
					debugPrintf(pFlyRadioInfo->RDSInfo.curRadio_Text_A);
#endif
					returnRDS_RT(pFlyRadioInfo->RDSInfo.curRadio_Text_A,temp + i + 1);
				}
			}

			pFlyRadioInfo->RDSInfo.rds_rt_segment_a++;
		}
	}
	else if (GROUP_B == tempGroup)
	{
		if (pFlyRadioInfo->RDSInfo.rds_rt_segment_b == temp_RT_SEG)
		{
			temp = temp_RT_SEG * 2;
			for (i = 0; i<2; i++)
			{
				pFlyRadioInfo->RDSInfo.preRadio_Text_B[temp + i] = pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][6 + i];
				if ((0x0D == pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][6 + i]) || (temp + i == 31))
				{
					if (0 != memcmp(pFlyRadioInfo->RDSInfo.preRadio_Text_B, pFlyRadioInfo->RDSInfo.curRadio_Text_B, temp + i + 1))
					{
						memcpy(pFlyRadioInfo->RDSInfo.curRadio_Text_B, pFlyRadioInfo->RDSInfo.preRadio_Text_B, temp + i + 1);
					}
#if RADIO_RDS_DEBUG_MSG
					debugPrintf("\nrds RT Group B-->");
					debugPrintf(" count-->%d",temp + i + 1);
					pFlyRadioInfo->RDSInfo.curRadio_Text_B[temp + i + 1] = '\0';
					debugPrintf(pFlyRadioInfo->RDSInfo.curRadio_Text_B);
#endif
					returnRDS_RT(pFlyRadioInfo->RDSInfo.curRadio_Text_B,temp + i + 1);
				}
			}

			pFlyRadioInfo->RDSInfo.rds_rt_segment_b++;
		}
	}
}

void get_updata_rds_date(struct flyradio_struct_info *pFlyRadioInfo)
{
	int tempdate,temphour,tempmin,tempofs;
	int yd,md,d,y,m;

	BYTE temp_rdsdate[100];
	BYTE temp_rdstime[100];

	if(pFlyRadioInfo->RDSInfo.rds_group == GROUP_A)
	{
		tempdate = ((pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][3]&3)<<15) | (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][4]<<7) | (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][5]>>1);
		temphour = ((pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][5]&1)<<4)  | ((pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][6]>>4));
		tempmin  = ((pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][6]&0xf)<<2)| ((pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][7]>>6));
		tempofs  = (pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][7]&0x3f);

		yd = (int)(((double)tempdate-15078.2)/365.25);
		md = (int)(((((double)tempdate-14956.1)-((double)yd*365.25)))/30.6001);
		d = (int)(tempdate-14956-((double)yd*365.25)-((double)md*30.6001));

		if(md==14 || md==15) {
			y = yd + 1;
			m = md -13;
		} else {
			y = yd;
			m = md -1;
		}
		
		pFlyRadioInfo->RDSInfo.dataInfo.rds_year = 1900 + y;
		pFlyRadioInfo->RDSInfo.dataInfo.rds_month= m;
		pFlyRadioInfo->RDSInfo.dataInfo.rds_day  = d + 1;
		pFlyRadioInfo->RDSInfo.dataInfo.rds_weekday = (BYTE)(((long)tempdate +2)%7 + 1); 

		pFlyRadioInfo->RDSInfo.dataInfo.rds_hour = temphour;
		pFlyRadioInfo->RDSInfo.dataInfo.rds_min  = tempmin;
		pFlyRadioInfo->RDSInfo.dataInfo.rds_ofs  = tempofs;

#if RADIO_RDS_DEBUG_MSG
		snprintf(temp_rdsdate,sizeof(temp_rdsdate),"\nrds data-->%d--%d--%d",pFlyRadioInfo->RDSInfo.dataInfo.rds_year,pFlyRadioInfo->RDSInfo.dataInfo.rds_month,pFlyRadioInfo->RDSInfo.dataInfo.rds_day);
		snprintf(temp_rdstime,sizeof(temp_rdstime),"\nrds time-->%d:%d",pFlyRadioInfo->RDSInfo.dataInfo.rds_hour,pFlyRadioInfo->RDSInfo.dataInfo.rds_min);
		debugPrintf(temp_rdsdate);
		debugPrintf(temp_rdstime);
		debugPrintf("\nrds weekday-->%d",pFlyRadioInfo->RDSInfo.dataInfo.rds_weekday);
#endif

		//if(date != 0)
		//{
		//	printfRDS_Date();
		//}
	}
}

void RDSParaInit(void)
{
	debugPrintf("\nFlyRadio RDS para init!");

	pFlyRadioInfo->breceiveRDSdata = FALSE;
	control_rds_buffer(FALSE);

	memset(&pFlyRadioInfo->RDSInfo,0,sizeof(struct _FLY_RDS_INFO));
	memset(&pFlyRadioInfo->rdsdec_buf,0,sizeof(pFlyRadioInfo->rdsdec_buf));
	pFlyRadioInfo->rds_data_lx = 0;
	pFlyRadioInfo->rds_data_hx = 0;

	pFlyRadioInfo->RDSInfo.ibasefreq = ((*pFlyRadioInfo->radioInfo.pCurRadioFreq) / 10 - 875);

	pFlyRadioInfo->breceiveRDSdata = TRUE;
	control_rds_buffer(TRUE);

	debugPrintf("\nbase freq-->%d",*pFlyRadioInfo->radioInfo.pCurRadioFreq);
	debugPrintf("  base af freq-->%d",pFlyRadioInfo->RDSInfo.ibasefreq);
}

static UINT16 get_af_freq(struct flyradio_struct_info *pFlyRadioInfo)
{
	if (0 == pFlyRadioInfo->RDSInfo.afInfo.aflist_count)
	{
		return 0;
	}

	if (pFlyRadioInfo->RDSInfo.afInfo.af_checklist_point < pFlyRadioInfo->RDSInfo.afInfo.aflist_count)
	{
		return pFlyRadioInfo->RDSInfo.afInfo.aflist[pFlyRadioInfo->RDSInfo.afInfo.af_checklist_point++];
	}
	else
	{
		return 1;
	}
}

static int add_af_to_list(struct flyradio_struct_info *pFlyRadioInfo,BYTE temp_freq)
{
	BYTE i;
	UINT16 freq;

	freq = temp_freq + 875;

	if ((freq > 1080) || (freq < 875))
	{
		//debugPrintf("\nfreq error----->%d",freq);
		return 0;
	}

	if (pFlyRadioInfo->RDSInfo.afInfo.aflist_count >= 25)
	{
		debugPrintf("\nRDS AF LIST IS FULL");
		return 0;
	}
	if (0 == pFlyRadioInfo->RDSInfo.afInfo.aflist_count)
	{
		debugPrintf("\naf list add-->%d",freq);
		pFlyRadioInfo->RDSInfo.afInfo.aflist[pFlyRadioInfo->RDSInfo.afInfo.aflist_count++] = freq;
	}
	else
	{
		for (i = 0;i < pFlyRadioInfo->RDSInfo.afInfo.aflist_count; i++)
		{
			if (pFlyRadioInfo->RDSInfo.afInfo.aflist[i] == freq)
			{
				return 0;
			}
		}
		debugPrintf("\naf list add-->%d",freq);
		pFlyRadioInfo->RDSInfo.afInfo.aflist[pFlyRadioInfo->RDSInfo.afInfo.aflist_count++] = freq;
	}

	//if (pFlyRadioInfo->RDSInfo.afInfo.aflist_count > 0)
	//{
	//	for (i = 0;i<pFlyRadioInfo->RDSInfo.afInfo.aflist_count;i++)
	//	{
	//		debugPrintf("\nrds af--> %d %d",i,pFlyRadioInfo->RDSInfo.afInfo.aflist[i]);
	//	}
	//}

	return 1;
}

//////////////////////////RDS数据接收处理
//=============================================================================
// RDS Decode Type 0x
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================

void rdsdec_prc_0x(struct flyradio_struct_info *pFlyRadioInfo,BYTE Group)
{
	pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af = GET_SEG_PS_DI_AF(pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx]);

	debugPrintf("\nFlyRadio rds process rds_segment_ps_di_af--->%d",pFlyRadioInfo->RDSInfo.rds_segment_ps_di_af);

	get_updata_MS(pFlyRadioInfo);

	get_updata_DI(pFlyRadioInfo);

	get_updata_TA(pFlyRadioInfo);

	if(Group == GROUP_A)
	{
		add_af_to_list(pFlyRadioInfo,pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][4]);
		add_af_to_list(pFlyRadioInfo,pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx][5]);
	}
	
	get_updata_PS(pFlyRadioInfo,Group);
}


//=============================================================================
// RDS Decode Type 1A and 1B
//  Program Item Number and slow labeling codes
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_1x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
//	UINT16 blk[4];
//	BYTE  vc;
//	//	BYTE r_page,la;
//	//	BYTE  day,hour,min;
//	UINT16 label;
//
//	blk[0] = (pFlyRadioInfo->rdsdec_buf[0]<<8) | pFlyRadioInfo->rdsdec_buf[1];
//	blk[1] = (pFlyRadioInfo->rdsdec_buf[2]<<8) | pFlyRadioInfo->rdsdec_buf[3];
//	blk[2] = (pFlyRadioInfo->rdsdec_buf[4]<<8) | pFlyRadioInfo->rdsdec_buf[5];
//	blk[3] = (pFlyRadioInfo->rdsdec_buf[6]<<8) | pFlyRadioInfo->rdsdec_buf[7];
//
//	//	r_page = blk[1]&0x1F;
//	//	la     = blk[2]&0x8000;    // Linkage Actuator
//	vc     =(blk[2]>>12)&0x07; // Variant Code
//
//	//	day = (blk[3]>>11) & 0x1F;
//	//	hour= (blk[3]>>6) & 0x1f;
//	//	min = 0x3f & blk[3];
//
//	if(fab==GROUP_A){
//		label = blk[2]&0x0FFF;
//
//		switch(vc){
//case 0: // Extended Country Code
//	label = blk[2]&0x00FF;
//	pFlyRadioInfo->RDSInfo.rds_ecc = label;
//	break;
//
//case 1: // TMC identification
//	break;
//
//case 2: // Paging identification
//	break;
//
//case 3: // Language codes
//	pFlyRadioInfo->RDSInfo.rds_language_code =	label;
//	break;
//
//case 4: // not assigned
//case 5: // not assigned
//	break;
//
//case 6: // For use by broadcasters
//	break;
//
//case 7: // Identification of EWS channnel
//	break;
//		}
//	}
}

//=============================================================================
// RDS Decode 2A and 2B
//  Radio Text (2A has 64 charactors,2B has 32 charactors)
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_2x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
	get_updata_RT(pFlyRadioInfo);
}



//=============================================================================
// RDS Decode type 3A or 3B
//  3A : Application identification for Open data
//  3B : Open Data Application
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_3x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
//	//	blk[0] = (rdsdec_buf[0]<<8) | rdsdec_buf[1];
//	//	blk[1] = (rdsdec_buf[2]<<8) | rdsdec_buf[3];
//	//	blk[2] = (rdsdec_buf[4]<<8) | rdsdec_buf[5];
//	//	blk[3] = (rdsdec_buf[6]<<8) | rdsdec_buf[7];
//
//
//	if(fab==GROUP_A){
//
//	}
//
}



//=============================================================================
// RDS Decode type 4A and 4B
//  4A : "Clock-time and date"
//  4B : Open data application
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_4x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
	get_updata_rds_date(pFlyRadioInfo);
}
//=============================================================================
// RDS Decode 5A and 5B 
//  5A and 5B : Transparent data channels or ODA
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_5x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{

}


//=============================================================================
// RDS Decode 6A and 6B
// In-house applications of ODA
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_6x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
}


//=============================================================================
// RDS Decode 7A and 7B
// 7A : Radio Paging or ODA
// 7B : Open data application
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_7x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
}



//=============================================================================
// RDS Decode 8A and 8B
// Traffic Message Channes or ODA
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_8x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)	//TMC
{
}



//=============================================================================
// RDS Decode 9A and 9B
// Emergency warining systems or ODA
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_9x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)	//EWS似乎有不同标准，ODA
{
}

//=============================================================================
// RDS Decode 10A and 10B
// 10A : Program Type Name
// 10B : Open data
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_10x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
	//BYTE ofs;
	//BYTE b;

	//if(fab==GROUP_A)
	//{
	//	ofs = (pFlyRadioInfo->rdsdec_buf[3]&0x01)*4;
	//	b = (pFlyRadioInfo->rdsdec_buf[3]>>4)&0x01;

	//	if(ofs == 0){sRDSTemp.rdsdec_cnt10x = 0;}
	//	else if(sRDSTemp.rdsdec_cnt10x == 0){return;	}
	//	sRDSTemp.rdsdec_cnt10x++;

	//	sRDSTemp.rds_ptyn[ofs+0] = pFlyRadioInfo->rdsdec_buf[4];
	//	sRDSTemp.rds_ptyn[ofs+1] = pFlyRadioInfo->rdsdec_buf[5];
	//	sRDSTemp.rds_ptyn[ofs+2] = pFlyRadioInfo->rdsdec_buf[6];
	//	sRDSTemp.rds_ptyn[ofs+3] = pFlyRadioInfo->rdsdec_buf[7];


	//	if(ofs==4 && sRDSTemp.rdsdec_cnt10x==2)
	//	{
	//		if(memcmp(pFlyRadioInfo->RDSInfo.rdsdec_ptyn[b],sRDSTemp.rds_ptyn,8))
	//		{
	//			memcpy(pFlyRadioInfo->RDSInfo.rdsdec_ptyn[b],sRDSTemp.rds_ptyn,8);
	//			printfRDS_PTYN(pFlyRadioInfo->RDSInfo.rdsdec_ptyn[b]);
	//		}
	//	}
	//}
	//else
	//{
	//}
}

#ifdef DEF_SUPPORT_11	//For Open Data Application
//=============================================================================
// RDS Decode 11A and 11B
// Open Data Application
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_11x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
}
#endif

#ifdef DEF_SUPPORT_12	//For Open Data Application
//=============================================================================
// RDS Decode 12A and 12B
// Open Data Application
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_12x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
}
#endif


#ifdef DEF_SUPPORT_13	//For Open Data Application
//=============================================================================
// RDS Decode 13A and 13B
// Enhanced Radio Paging or ODA
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_13x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
//	if(fab==GROUP_A){
//		BYTE sty = pFlyRadioInfo->rdsdec_buf[3]&0x07;
//		BYTE inf = pFlyRadioInfo->rdsdec_buf[3]&0x08;
//
//		switch(sty){
//case 0: // address notification bits 24...0,when only 25bits are used 
//	break;
//
//case 1: // address notification bits 49..25,when 50bits are used
//	break;
//
//case 2: // adress notification bits 24..0,when 50bits are used
//	break;
//
//case 3: // Reserved for Value Added Services system information
//	break;
//
//case 4: // Reserved for future use
//case 5:
//case 6:
//case 7:
//	break;
//		}
//
//		host_rds_update(PTY_13A);
//	}
}
#endif

#ifdef DEF_SUPPORT_14	//等待处理

UINT16 rds_14a[16];
BYTE  rds_14a_flag;
//=============================================================================
// RDS Decode 14A and 14B
// 14A : Enhanced Other Networks informations
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_14x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
	//UINT16 pi_tn;
	//UINT16 pi_on;
	//BYTE tp_tn;
	//BYTE tp_on;

	//pi_tn = (pFlyRadioInfo->rdsdec_buf[0]<<8) | pFlyRadioInfo->rdsdec_buf[1];
	//pi_on = (pFlyRadioInfo->rdsdec_buf[6]<<8) | pFlyRadioInfo->rdsdec_buf[7];
	//tp_tn = (pFlyRadioInfo->rdsdec_buf[2]&0x08)?1:0;
	//tp_on = (pFlyRadioInfo->rdsdec_buf[2]&0x04)?1:0;

	//if(fab==GROUP_A) {
	//	BYTE vc = pFlyRadioInfo->rdsdec_buf[3]&0x0f;

	//	if(vc==0) {
	//		rds_14a_flag=0;
	//	}

	//	rds_14a[vc]=(pFlyRadioInfo->rdsdec_buf[4]<<8) | pFlyRadioInfo->rdsdec_buf[5];
	//	rds_14a_flag++;

	//	if(rds_14a_flag==16 && vc==15){
	//		host_rds_update(PTY_14A);
	//	}
	//}
	//else {
	//	host_rds_update(PTY_14B);
	//}

}
#endif

void rdsdec_prc_14x_spec(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
	//if(fab == GROUP_B)
	//{
	//	printfRDS_TA_Spec();
	//}
}

#ifdef DEF_SUPPORT_15	//已在其他地方处理完成
BYTE rds_15a[8];
BYTE rds_15a_flag;
//=============================================================================
// RDS Decode 15A and 15B
// Fast basic tuning and switching information
//-----------------------------------------------------------------------------
// Parameter(s)
//  fab : Type A or B flag
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_prc_15x(struct flyradio_struct_info *pFlyRadioInfo,BYTE fab)
{
	//if(fab==GROUP_A){
	//	BYTE ps = (pFlyRadioInfo->rdsdec_buf[3]&1)*4;

	//	if(ps==0){
	//		rds_15a_flag=0;
	//	}

	//	rds_15a[ps+0]=pFlyRadioInfo->rdsdec_buf[4];
	//	rds_15a[ps+1]=pFlyRadioInfo->rdsdec_buf[5];
	//	rds_15a[ps+2]=pFlyRadioInfo->rdsdec_buf[6];
	//	rds_15a[ps+3]=pFlyRadioInfo->rdsdec_buf[7];
	//	rds_15a_flag++;

	//	if(rds_15a_flag==2 && ps){
	//		host_rds_update(PTY_15A);
	//	}
	//}
}
#endif

static void read_data(struct flyradio_struct_info *pFlyRadioInfo, BYTE *p, UINT16 len)
{
	I2C_Read_TEF6688(RDS_DATA_START, p, len);
}

//=============================================================================
// RDS Decode Processing
//-----------------------------------------------------------------------------
// Parameter(s)
//  none
//-----------------------------------------------------------------------------
// Return Value
//  none
//=============================================================================
void rdsdec_process(struct flyradio_struct_info *pFlyRadioInfo)//处理接收到的完整的一组数据
{
	pFlyRadioInfo->RDSInfo.rds_group = GET_G_AB(pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx]);
	pFlyRadioInfo->RDSInfo.rds_group_type = GET_GROUP_TYPE(pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx]);

	DBG0(debugPrintf("\nFlyRadio rds_group-->%d",pFlyRadioInfo->RDSInfo.rds_group);)
	DBG0(debugPrintf("  group_type-->%d",pFlyRadioInfo->RDSInfo.rds_group_type);)

	if (TRUE == rds_get_check_updata_PI(pFlyRadioInfo))
	{
		get_updata_PTY(pFlyRadioInfo);

		get_updata_TP(pFlyRadioInfo);

		switch(pFlyRadioInfo->RDSInfo.rds_group_type)
		{
			case 0: rdsdec_prc_0x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 1: rdsdec_prc_1x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 2: rdsdec_prc_2x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 3: rdsdec_prc_3x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 4: rdsdec_prc_4x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 5: rdsdec_prc_5x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 6: rdsdec_prc_6x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 7: rdsdec_prc_7x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 8: rdsdec_prc_8x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 9: rdsdec_prc_9x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

			case 10: rdsdec_prc_10x(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;

				/*case 11: rdsdec_prc_11x(pTDA7541RadioInfo,fab); break;

				case 12: rdsdec_prc_12x(pTDA7541RadioInfo,fab); break;

				case 13: rdsdec_prc_13x(pTDA7541RadioInfo,fab); break;

				case 14: rdsdec_prc_14x(pTDA7541RadioInfo,fab); break;

				case 15: rdsdec_prc_15x(pTDA7541RadioInfo,fab); break; */

			case 14: rdsdec_prc_14x_spec(pFlyRadioInfo, pFlyRadioInfo->RDSInfo.rds_group); break;//特别版，解TA信号

			default :
				break;
		}
	}
}

void *radio_af_thread(void *arg)
{
	struct flyradio_struct_info *pFlyRadioInfo = (struct flyradio_struct_info *)arg;
	BOOL bInnerDelayOn = FALSE;
	ULONG delaytime = 0;
	UINT16 itempfreq;
	BYTE delay_count = 0;

	while (!pFlyRadioInfo->bKillRadioRDSAFThread)
	{
		if (bInnerDelayOn)
		{
			WaitSignedTimeOut(&pFlyRadioInfo->RDSAFThreadMutex,&pFlyRadioInfo->RDSAFThreadCond,&pFlyRadioInfo->bRDSAFThreadRunAgain,delaytime);
		}
		else
		{
			WaitSignedTimeOut(&pFlyRadioInfo->RDSAFThreadMutex,&pFlyRadioInfo->RDSAFThreadCond,&pFlyRadioInfo->bRDSAFThreadRunAgain,2000);
		}

		if (/*(pFlyAllInOneInfo->pMemory_Share_Common->eCurAudioInput != RADIO) ||*/
			(AM == pFlyRadioInfo->radioInfo.eCurRadioMode) || 
			(TRUE == pFlyRadioInfo->radioInfo.bPreScaning) ||
			(TRUE == pFlyRadioInfo->radioInfo.bCurScaning) ||
			(*pFlyRadioInfo->radioInfo.pPreRadioFreq != *pFlyRadioInfo->radioInfo.pCurRadioFreq) ||
			(pFlyRadioInfo->radioInfo.eCurRadioMode != pFlyRadioInfo->radioInfo.ePreRadioMode) ||
			(TRUE == pFlyRadioInfo->radioInfo.bPreStepButtomDown) ||
			//(FALSE == pFlyRadioInfo->RDSInfo.RadioRDSAFControlOn) ||
			((pFlyRadioInfo->RDSInfo.afInfo.aflist_count == 1)&&(pFlyRadioInfo->RDSInfo.afInfo.aflist[0]*10 == *pFlyRadioInfo->radioInfo.pCurRadioFreq)) ||
			(0 == pFlyRadioInfo->RDSInfo.Base_PI_Code) ||
			pFlyAllInOneInfo->pMemory_Share_Common->iHostPowerOff
			)
		{
			DBG1(debugPrintf("\nradio_af_thread");)
			pFlyRadioInfo->RDSInfo.badbasesignalcount = 0;
			pFlyRadioInfo->RDSInfo.badlistsignalcount = 0;
			pFlyRadioInfo->bafthread_stop = TRUE;
			pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_CHECK_CUR_LEVEL;
			bInnerDelayOn = FALSE;
			continue;
		}

		pFlyRadioInfo->bafthread_stop = FALSE;

		if (TRUE == pFlyRadioInfo->RDSInfo.bswitch_fail)
		{
			delay_count++;
			DBG1(debugPrintf("\ndelay_count--->%d",delay_count);)
			bInnerDelayOn = FALSE;
			if (delay_count <= 30)
			{
				continue;
			}
			else
			{
				pFlyRadioInfo->RDSInfo.bswitch_fail = FALSE;
				pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_GET_SWITCH_FREQ;
				delay_count = 0;
			}
		}

		DBG1(debugPrintf("\neafswitch_mode--->%d",pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode);)
		DBG1(debugPrintf("  badbasesignalcount-->%d",pFlyRadioInfo->RDSInfo.badbasesignalcount);)
		DBG1(debugPrintf("  badlistsignalcount-->%d",pFlyRadioInfo->RDSInfo.badlistsignalcount);)
		switch (pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode)
		{
		case AF_CHECK_CUR_LEVEL:
				   if ((*p_bRadio_SignalGood)(pFlyRadioInfo->radioInfo.eCurRadioMode,CHECK_SIGNAL_AF))
				   {
					   pFlyRadioInfo->RDSInfo.badbasesignalcount = 0;
					   bInnerDelayOn = FALSE;
				   }
				   else
				   {
					   pFlyRadioInfo->RDSInfo.badbasesignalcount++;
					   if (pFlyRadioInfo->RDSInfo.badbasesignalcount >= 60)
					   {
						   pFlyRadioInfo->RDSInfo.badbasesignalcount = 0;
						   pFlyRadioInfo->RDSInfo.bswitch_success = FALSE;
						   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_GET_SWITCH_FREQ;
						   bInnerDelayOn = TRUE;
						   delaytime = 50;
					   }
					   else
					   {		
							bInnerDelayOn = FALSE;
					   }
				   }
				   break;
		case AF_GET_SWITCH_FREQ:
				   itempfreq = get_af_freq(pFlyRadioInfo);
				   if (0 == itempfreq)
				   {
					   DBG1(debugPrintf("\nget af freq--->count is 0!");)
					   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_CHECK_CUR_LEVEL;
					   bInnerDelayOn = FALSE;
					   pFlyRadioInfo->bCurMute = !pFlyRadioInfo->bPreMute;
				   }
				   else if(1 == itempfreq)
				   {
						DBG1(debugPrintf("\nget af freq--->around end!");)
						pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_UPDATA_END;
						bInnerDelayOn = TRUE;
						delaytime = 50;
				   }
				   else if (*pFlyRadioInfo->radioInfo.pCurRadioFreq == itempfreq * 10)
				   {
					    bInnerDelayOn = TRUE;
					    delaytime = 50;
				   }
				   else
				   {
					   DBG1(debugPrintf("\nget af freq-->%d",itempfreq);)

					   *pFlyRadioInfo->radioInfo.pCurRadioFreq = itempfreq * 10;
					   *pFlyRadioInfo->radioInfo.pPreRadioFreq = *pFlyRadioInfo->radioInfo.pCurRadioFreq;
					   (*p_Radio_Set_Freq)(pFlyRadioInfo->radioInfo.eCurRadioMode,*pFlyRadioInfo->radioInfo.pCurRadioFreq);

					   pFlyRadioInfo->breceiveRDSdata = FALSE;
					   control_rds_buffer(FALSE);
					   memset(&pFlyRadioInfo->rdsdec_buf,0,sizeof(pFlyRadioInfo->rdsdec_buf));
					   pFlyRadioInfo->rds_data_lx = 0;
					   pFlyRadioInfo->rds_data_hx = 0;
					   pFlyRadioInfo->breceiveRDSdata = TRUE;
					   control_rds_buffer(TRUE);

					   pFlyRadioInfo->RDSInfo.New_PI_Code = 0;

					   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_CHECK_LIST_LEVEL;

					   PostSignal(&pFlyRadioInfo->RDSRecThreadMutex,&pFlyRadioInfo->RDSRecThreadCond,&pFlyRadioInfo->bRDSThreadRunAgain);

					   bInnerDelayOn = TRUE;
					   delaytime = 1000;
				   }
				   break;
		case AF_CHECK_LIST_LEVEL:
				   if ((*p_bRadio_SignalGood)(pFlyRadioInfo->radioInfo.eCurRadioMode,CHECK_SIGNAL_AF))
				   {
					   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_CHECK_LIST_RDSPI;
					   bInnerDelayOn = TRUE;
					   delaytime = 2000;
				   }
				   else
				   {
					   pFlyRadioInfo->RDSInfo.badlistsignalcount++;
					   if (pFlyRadioInfo->RDSInfo.badlistsignalcount >= 5)
					   {
						   pFlyRadioInfo->RDSInfo.badlistsignalcount = 0;
						   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_GET_SWITCH_FREQ;
						   bInnerDelayOn = TRUE;
						   delaytime = 50;
					   }
					   else
					   {
						   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_CHECK_LIST_LEVEL;
						   bInnerDelayOn = TRUE;
						   delaytime = 10;
					   }
				   }
				   break;
		case AF_CHECK_LIST_RDSPI:
				   if ((pFlyRadioInfo->RDSInfo.New_PI_Code == 0) ||
					   (pFlyRadioInfo->RDSInfo.New_PI_Code != pFlyRadioInfo->RDSInfo.Base_PI_Code))
				   {
					   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_GET_SWITCH_FREQ;
					   bInnerDelayOn = TRUE;
					   delaytime = 50;
				   }	
				   else
				   {
					   pFlyRadioInfo->RDSInfo.bswitch_success = TRUE;
					   returnRadioFreq(*pFlyRadioInfo->radioInfo.pCurRadioFreq);
					   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_UPDATA_END;
					   bInnerDelayOn = TRUE;
					   delaytime = 50;
				   }
				   break;
		case AF_UPDATA_END:
				   if (FALSE == pFlyRadioInfo->RDSInfo.bswitch_success)
				   {
					   *pFlyRadioInfo->radioInfo.pCurRadioFreq = (pFlyRadioInfo->RDSInfo.ibasefreq + 875) * 10;
					   *pFlyRadioInfo->radioInfo.pPreRadioFreq = *pFlyRadioInfo->radioInfo.pCurRadioFreq;
					   (*p_Radio_Set_Freq)(pFlyRadioInfo->radioInfo.eCurRadioMode,*pFlyRadioInfo->radioInfo.pCurRadioFreq);
					   returnRadioFreq(*pFlyRadioInfo->radioInfo.pCurRadioFreq);
					   
					   pFlyRadioInfo->RDSInfo.bswitch_fail = TRUE;
					   delay_count = 0;

					   pFlyRadioInfo->RDSInfo.afInfo.af_checklist_point = 0;
					   pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode = AF_CHECK_CUR_LEVEL;

					   pFlyRadioInfo->RDSInfo.badbasesignalcount = 0;
					   pFlyRadioInfo->RDSInfo.badlistsignalcount = 0;
				   }
				   else
				   {
					   RDSParaInit();
				   }

				   bInnerDelayOn = FALSE;
				   break;
		}	
	}

	return 0;
}

void *radio_rdsrec_thread(void *arg)
{
	struct flyradio_struct_info *pFlyRadioInfo = (struct flyradio_struct_info *)arg;
	INT ret = 0;
	struct timeval timenow;
	struct timespec timeout;
	BOOL bInnerDelayOn = FALSE;
	ULONG delaytime = 300;

	BYTE rds_signal_quality = 0;
	BYTE rds_signal_check_count = 0;
	BYTE i;

	BYTE rds_buf[11];
	BYTE rds_status_buf[2];

	BYTE  a_block_error_code;
	BYTE  b_block_error_code;
	BYTE  c_block_error_code;
	BYTE  d_block_error_code;

	static BYTE rds_data_available_flag;
	static BYTE rds_data_loss_flag;
	static BYTE	rds_data_available_type;
	static BYTE	rds_group_type; 
	static BYTE rds_sync_status; 

	while (!pFlyRadioInfo->bKillRadioRDSRecThread)
	{
		//if (RADIO_TDA7541 == pFlyAllInOneInfo->pMemory_Share_Common->iRadioChip)
		//{
		//	if (TRUE == bInnerDelayOn)
		//	{
		//		WaitSignedTimeOut(&pFlyRadioInfo->RDSRecThreadMutex,&pFlyRadioInfo->RDSRecThreadCond,&pFlyRadioInfo->bRDSThreadRunAgain,delaytime);
		//	}
		//	else
		//	{
		//		WaitSignedTimeOut(&pFlyRadioInfo->RDSRecThreadMutex,&pFlyRadioInfo->RDSRecThreadCond,&pFlyRadioInfo->bRDSThreadRunAgain,INFINITE);
		//	}
		//}
		//else if (RADIO_TEF6624 == pFlyAllInOneInfo->pMemory_Share_Common->iRadioChip)
		//{
		//	if (TRUE == bInnerDelayOn)
		//	{
		//		WaitSignedTimeOut(&pFlyRadioInfo->RDSRecThreadMutex,&pFlyRadioInfo->RDSRecThreadCond,&pFlyRadioInfo->bRDSThreadRunAgain,20);
		//	}
		//	else
		//	{
		//		WaitSignedTimeOut(&pFlyRadioInfo->RDSRecThreadMutex,&pFlyRadioInfo->RDSRecThreadCond,&pFlyRadioInfo->bRDSThreadRunAgain,INFINITE);
		//	}
		//}

		if (TRUE == bInnerDelayOn)
		{
			WaitSignedTimeOut(&pFlyRadioInfo->RDSRecThreadMutex,&pFlyRadioInfo->RDSRecThreadCond,&pFlyRadioInfo->bRDSThreadRunAgain,delaytime);
		}
		else
		{
			WaitSignedTimeOut(&pFlyRadioInfo->RDSRecThreadMutex,&pFlyRadioInfo->RDSRecThreadCond,&pFlyRadioInfo->bRDSThreadRunAgain,INFINITE);
		}
		DBG1(debugPrintf("\n Read RDS data Start ");)
		
		if (!pFlyRadioInfo->bPowerUp)
		{
			pFlyRadioInfo->brecthread_stop = TRUE;
			bInnerDelayOn = FALSE;
			continue;
		}

		if (/*(pFlyAllInOneInfo->pMemory_Share_Common->eCurAudioInput != RADIO) ||*/
			(AM == pFlyRadioInfo->radioInfo.eCurRadioMode) || 
			(TRUE == pFlyRadioInfo->radioInfo.bPreScaning) ||
			(TRUE == pFlyRadioInfo->radioInfo.bCurScaning) ||
			(*pFlyRadioInfo->radioInfo.pPreRadioFreq != *pFlyRadioInfo->radioInfo.pCurRadioFreq) ||
			(pFlyRadioInfo->radioInfo.eCurRadioMode != pFlyRadioInfo->radioInfo.ePreRadioMode) ||
			(TRUE == pFlyRadioInfo->radioInfo.bPreStepButtomDown) ||
			pFlyAllInOneInfo->pMemory_Share_Common->iHostPowerOff
			)
		{
			pFlyRadioInfo->brecthread_stop = TRUE;
			bInnerDelayOn = FALSE;
			DBG1(debugPrintf("\n Read RDS Conditions no good! ");)
			continue;
		}

		pFlyRadioInfo->brecthread_stop = FALSE;

		if (RADIO_TEF6624 == pFlyAllInOneInfo->pMemory_Share_Common->iRadioChip)
		{
			bInnerDelayOn = TRUE;

			rds_signal_quality = TEF6624_Get_RDS_Raw_Date();
			if (rds_signal_quality > TEF6624_RDS_DATA_ERROR_LEVEL)
			{
				if (rds_signal_check_count < 20)
				{
					rds_signal_check_count++;
				}		
			}
			else
			{
				rds_signal_check_count = 0;
			}

			if (pFlyRadioInfo->RDSInfo.afInfo.eafswitch_mode == AF_CHECK_CUR_LEVEL || rds_signal_check_count < 20)
			{
				delaytime = 20;
			}
			else
			{
				delaytime = 5000;
			}
		}

		if (RADIO_TEF6688 == pFlyAllInOneInfo->pMemory_Share_Common->iRadioChip)
		{
			bInnerDelayOn = TRUE;
			
			I2C_Read_TEF6688(RDS_READ_STATUS, rds_status_buf, 2);
			
			rds_data_available_flag = (rds_status_buf[0]>>7) & 0x01;
			rds_data_loss_flag		= (rds_status_buf[0]>>6) & 0x01;
			rds_data_available_type = (rds_status_buf[0]>>5) & 0x01;
			rds_group_type =  (rds_status_buf[0]>>4) & 0x01;
			rds_sync_status = (rds_status_buf[0]>>1) & 0x01;

			if (!rds_data_available_flag)
			{
				DBG1(debugPrintf("\n Read RDS data No data Available");)
				continue;	
			}

			Sleep(5);
			read_data(pFlyRadioInfo,rds_buf, 11);

			DBG1(debugPrintf("\n read rds buf: ");)
			for (i=0; i< READ_DATA_END; i++)
			{
				DBG1(debugPrintf("\n%2x",rds_buf[i]);)
			}

			a_block_error_code = (rds_buf[10]>>6) & 0x03;
			b_block_error_code = (rds_buf[10]>>4) & 0x03;
			c_block_error_code = (rds_buf[10]>>2) & 0x03;
			d_block_error_code = rds_buf[10]& 0x03;

			if (a_block_error_code >= 0x02 || b_block_error_code >= 0x02 || c_block_error_code >= 0x02
			|| d_block_error_code >= 0x02)
			{
				DBG1(debugPrintf("\n Read RDS data no data correction possible ");)
				continue;				
			}

			pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][0] = rds_buf[2];
			pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][1] = rds_buf[3];		//BLOCK1

			pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][2] = rds_buf[4];
			pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][3] = rds_buf[5];		//BLOCK2

			if (!rds_group_type)
			{
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][4] = rds_buf[6];
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][5] = rds_buf[7];		//BLOCK3
			}
			else if (rds_group_type)
			{
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][4] = rds_buf[6];
				pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][5] = rds_buf[7];		//BLOCK3
			}

			pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx][6] = rds_buf[8];
			pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_hx++][7] = rds_buf[9];		//BLOCK4

			if (pFlyRadioInfo->rds_data_hx >= (RDS_DATA_BUF_MAX - 1))
			{
				pFlyRadioInfo->rds_data_hx = 0;
			}
				
		}

		if (pFlyRadioInfo->rds_data_lx != pFlyRadioInfo->rds_data_hx)
		{
			DBG0(debugBuf("\nrds_data_lx-->",pFlyRadioInfo->rdsdec_buf[pFlyRadioInfo->rds_data_lx],8);)
			rdsdec_process(pFlyRadioInfo);//处理接收到的完整的一组数据
			pFlyRadioInfo->rds_data_lx++;
			if (pFlyRadioInfo->rds_data_lx >= (RDS_DATA_BUF_MAX - 1))
			{
				pFlyRadioInfo->rds_data_lx = 0;
			}
		}

		if (RADIO_TDA7541 == pFlyAllInOneInfo->pMemory_Share_Common->iRadioChip)
		{
			if (pFlyRadioInfo->rds_data_lx != pFlyRadioInfo->rds_data_hx)
			{
				bInnerDelayOn = TRUE;
				delaytime = 100;
			}
			else
			{
				bInnerDelayOn = FALSE;
			}
		}
		
	}
	debugPrintf("\nFlyRadio RDSRecThread exit!");
	return 0;
}

#endif

