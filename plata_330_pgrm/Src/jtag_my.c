
#include "main.h"


void JTAG_TCK (u32 a)
{
	
}

void TLR (void)   // откуда угодно
{
	int i=0;
	TMS(1);
	for (i=0;i<5;i++)
	{
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
	}
}

void RUN_TEST_IDLE (void)
{
	TMS(0);
	TDI(0);
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
}

void Shift_IR (void)
{
	TMS(1);	
	TCK(1);
	Delay(1);
	TCK(0);
	Delay(1);
	
	TMS(1);	
	TCK(1);
	Delay(1);
	TCK(0);
	Delay(1);
	
	TMS(0);	
	TCK(1);
	Delay(1);
	TCK(0);
	Delay(1);
	
	TMS(0);	
	TCK(1);
	Delay(1);
	TCK(0);
	Delay(1);
}

void Exit1_IR (void)  //
{
	TMS(1);	
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);	
}

void Update_IR (void)
{
	TMS(1);
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
}

void Shift_DR (void)
{
	TMS(1);
    TDI(0);
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
	TMS(0);
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
	TMS(0);
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);	
}

void Exit1_DR (void)
{
TMS(1);
	  TCK(1);
	  Delay(1);
	  TCK(0);
	  Delay(1);	  	  
}

void Update_DR (void)
{
TMS(1);
	  TCK(1);
	  Delay(1);
	  TCK(0);
	  Delay(1);	  	  
}


u8 SCAN_N (void)
{
u16 i=0;
u16 k=0;//счётчик узлов у чейне
u8  dat=0;

for (i=0;i<256;i++) //отправляем 256 "1" чтобы подготовится к ожиданию нуля
	{
		TDI(1);
		
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
	}

	TDI(0);
for (i=0;i<256;i++)
	{
		dat=TDO();		
		if (dat==0) break;
		k++;
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);		
	}
	
return k;
}

u32 GET_DATA (u8 n)
{
	u16 i=0;
	u32 dat=0;
	u32 bit=0;

	for (i=0;i<n;i++)
	{
		bit=TDO();
		dat=(dat>>1)+(bit<<31);
		
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
	}
	return dat;
}


void JTAG_SCAN (void)
{
u16 i=0;
u8  n=0;
	
	TLR();
	RUN_TEST_IDLE();
	Shift_IR();
//---------------------
	for (i=0;i<256;i++)  //отправляем 32 раза по 8 бит = 256 "1"
	{
		TDI(1);

		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
	}	
//--------------------
	Exit1_IR  ();
	Update_IR ();
	RUN_TEST_IDLE();
	Shift_DR ();
n=	SCAN_N ();
	u_out("JTAG CHAIN:",n);	
}

void ID_recive (u8 cmd)
{
u16 i=0;
u32 n=0;
	
	TLR();
	RUN_TEST_IDLE();
	Shift_IR();
//---------------------
	n=cmd&1;
	for (i=0;i<4;i++)  //отправляем 4 бита команды в IR последнего TAP
	{
		TDI(n);
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
		
		n=(cmd>>1)&1;
	}	
//--------------------	
	n=1;
	for (i=0;i<4;i++)  //отправляем 5 бит , сейчас 4 бита и потом 1 бит
	{
		TDI(n);
		TCK(1);
		Delay(1);
		TCK(0);
		Delay(1);
	}
//--------------------
	 TDI(0);
     Exit1_IR  ();
	 Update_IR ();
	 RUN_TEST_IDLE();
	 Shift_DR ();
   n=GET_DATA (32);
   x_out("JTAG_ID:",n);
	 
}

