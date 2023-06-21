/*************************************************************************
 *
 *  ti_list.c - Library of routines for readout and buffering of
 *                     events using a JLAB Trigger Interface V3 (TI) with
 *                     a Linux VME controller in CODA 3.0.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*64      /* Size in Bytes */

/* TI_MASTER / TI_SLAVE defined in Makefile */

#ifdef TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
#else
#ifdef TI_SLAVE5
#define TI_SLAVE
#define SLAVE_FLAG TI_INIT_SLAVE_FIBER_5
#endif
/* TS trigger source (e.g. fiber), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
#endif

/* TI VME address, or 0 for Auto Initialize (search for TI by slot) */
#define TI_ADDR  0

/* Skip the firmware check while we're testing 3v11.1 */
#ifdef SLAVE_FLAG
#define TI_FLAG TI_INIT_SKIP_FIRMWARE_CHECK|SLAVE_FLAG
#else
#define TI_FLAG TI_INIT_SKIP_FIRMWARE_CHECK
#endif

/* Measured longest fiber length in system */
//#define FIBER_LATENCY_OFFSET 0x4A
#define FIBER_LATENCY_OFFSET 0x5A

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 5

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

#include "usrstrutils.c"
#ifdef TI_MASTER
/* TI-Master Fiber-Slave configuration */
typedef struct
{
  int enable;
  int port;
  char rocname[64];
} TI_SLAVE_MAP;

enum tiSlaves
  {
   slave1 = 0,
   slave2,
   slave3,
   slave4,
   slave5,
   nSlaves
  };


TI_SLAVE_MAP tiSlaveConfig[nSlaves] =
  {
    { 0,  1,  "dontuse"}, // reserved for HMS TI connection
    { 0,  2,  "npsvme2"},
    { 0,  3,  "npsvme3"},
    { 0,  4,  "npsvme4"},
    { 0,  5,  "npsvme5"}
  };
#endif

/* TI VTP configuration */
int enable_vtp = 0;

/*
  Read the user flags/configuration file.
  10sept21 - BM
    - Support for
      all,
      HCAL, LHRS, BIGBITE
      ROCs listed in tdSlaveConfig
*/
void
readUserFlags()
{
  int flag = 0, flagval = 0;

  printf("%s: Reading user flags file.\n", __func__);
  init_strings();

#ifdef TI_MASTER
  /* enable 'npsvme1', 'npsvme1=1'
     disable 'npsvme1=0'
     similar for the rest of the crates
  */
  int islave=0;
  for(islave = 0; islave < nSlaves; islave++)
    {
      flagval = 0;
      flag = getflag(tiSlaveConfig[islave].rocname);

      if(flag)
	{
	  flagval = 1;

	  if(flag > 1)
	    flagval = getint(tiSlaveConfig[islave].rocname);

	  tiSlaveConfig[islave].enable = flagval;
	}

    }


  /* Print config */
  printf("%s\n TI Slave Config from usrstringutils\n",
	 __func__);
  printf("  Enable      Port       Roc\n");
  printf("|----------------------------------------|\n");

  for(islave = 0; islave < nSlaves; islave++)
    {
      printf("       %d  ",
	     tiSlaveConfig[islave].enable);

      printf("       %d  ",
	     tiSlaveConfig[islave].port);

      printf(" %-20s",
	     tiSlaveConfig[islave].rocname);

      printf("\n");
    }

  printf("\n");

#endif

  /* VTP Flag */
  flagval = 0;
  flag = getflag("vtp");
  if(flag)
    {
      flagval = 1;

      if(flag > 1)
	flagval = getint("vtp");

      enable_vtp = flagval;
    }

  printf("%s\n TI VTP config from usrstrutils\n",
	 __func__);

  if(enable_vtp)
    printf("\tENABLED\n");
  else
    printf("\tDISABLED\n");

}

/*
  Global to configure the trigger source
      0 : tsinputs
      1 : TI random pulser
      2 : TI fixed pulser

  Set with rocSetTriggerSource(int source);
*/
int rocTriggerSource = 1;
void rocSetTriggerSource(int source); // routine prototype

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int stat;

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  /* Define BLock Level */
  blockLevel = BLOCKLEVEL;


  /*****************
   *   TI SETUP
   *****************/
#ifdef TI_MASTER
  /*
   * Set Trigger source
   *    For the TI-Master, valid sources:
   *      TI_TRIGGER_FPTRG     2  Front Panel "TRG" Input
   *      TI_TRIGGER_TSINPUTS  3  Front Panel "TS" Inputs
   *      TI_TRIGGER_TSREV2    4  Ribbon cable from Legacy TS module
   *      TI_TRIGGER_PULSER    5  TI Internal Pulser (Fixed rate and/or random)
   */
  if(rocTriggerSource == 0)
    {
      tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */
    }
  else
    {
      tiSetTriggerSource(TI_TRIGGER_PULSER); /* Internal Pulser */
    }

  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 );

  /* Load the trigger table that associates
   *  pins 21/22 | 23/24 | 25/26 : trigger1
   *  pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);
#endif

  tiSetTriggerPulse(1,0,25,0);

  /* Set prompt output width (127 + 2) * 4 = 516 ns */
  tiSetPromptTriggerWidth(127);

  tiStatus(0);

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

  /* Read User Flags with usrstringutils
     What's set
     - TI Slave Ports
     - VTP
   */
  readUserFlags();

  DALMAGO;
  tiStatus(0);
  DALMASTOP;

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{

  /* Print out the Run Number and Run Type (config id) */
  printf("rocGo: Activating Run Number %d, Config id = %d\n",
	 rol->runNumber,rol->runType);

  int bufferLevel = 0;
  /* Get the current buffering settings (blockLevel, bufferLevel) */
  blockLevel = tiGetCurrentBlockLevel();
  bufferLevel = tiGetBroadcastBlockBufferLevel();
  printf("%s: Block Level = %d,  Buffer Level (broadcasted) = %d (%d)\n",
	 __func__,
	 blockLevel,
	 tiGetBlockBufferLevel(),
	 bufferLevel);

#ifdef TI_MASTER
  tiResetSlaveConfig();

  /* Enable TI ports that have been flagged from usrstringutils */
  int ii;
  for(ii = 0; ii < nSlaves; ii++)
    {
      if(tiSlaveConfig[ii].enable)
	{
	  tiAddSlave(tiSlaveConfig[ii].port);
	}
    }
#endif

  /* Add VTP as a slave, if flagged from usrstringutils */
  if(enable_vtp)
    tiRocEnable(2);

/* #ifdef TI_SLAVE */
/*   /\* In case of slave, set TI busy to be enabled for full buffer level *\/ */

/*   /\* Check first for valid blockLevel and bufferLevel *\/ */
/*   if((bufferLevel > 10) || (blockLevel > 1)) */
/*     { */
/*       daLogMsg("ERROR","Invalid blockLevel / bufferLevel received: %d / %d", */
/* 	       blockLevel, bufferLevel); */
/*       tiUseBroadcastBufferLevel(0); */
/*       tiSetBlockBufferLevel(1); */

/*       /\* Cannot help the TI blockLevel with the current library. */
/* 	 modules can be spared, though */
/*       *\/ */
/*       blockLevel = 1; */
/*     } */
/*   else */
/*     { */
/*       tiUseBroadcastBufferLevel(1); */
/*     } */
/* #endif */

#ifdef TI_MASTER
  /* Enable/Set Block Level on modules, if needed, here */
  if(rocTriggerSource != 0)
    {
      printf("************************************************************\n");
      daLogMsg("INFO","TI Configured for Internal Pulser Triggers");
      printf("************************************************************\n");

      if(rocTriggerSource == 1)
	{
	  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
	  tiSetRandomTrigger(1,0x4);
	  //tiSetRandomTrigger(1,0x4);
	}

      if(rocTriggerSource == 2)
	{
	  /*    Enable fixed rate with period (ns)
		120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
		- arg2 = 0xffff - Continuous
		- arg2 < 0xffff = arg2 times
	  */
	  tiSoftTrig(1,0xffff,100,0);
	}
    }
#endif

  DALMAGO;
  tiStatus(0);
  DALMASTOP;
}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

#ifdef TI_MASTER
  if(rocTriggerSource == 1)
    {
      /* Disable random trigger */
      tiDisableRandomTrigger();
    }

  if(rocTriggerSource == 2)
    {
      /* Disable Fixed Rate trigger */
      tiSoftTrig(1,0,100,0);
    }
#endif

  DALMAGO;
  tiStatus(0);
  DALMASTOP;

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int dCnt;

  /* Set TI output 1 high for diagnostics */
  /*BQ  tiSetOutputPort(1,0,0,0);*/

  /* Readout the trigger block from the TI
     Trigger Block MUST be readout first */
  dCnt = tiReadTriggerBlock(dma_dabufp);

  if(dCnt<=0)
    {
      printf("No TI Trigger data or error.  dCnt = %d\n",dCnt);
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */
      dma_dabufp += dCnt;
    }

  /* EXAMPLE: How to open a bank (name=5, type=ui4) and add data words by hand */
  BANKOPEN(5,BT_UI4,blockLevel);
  *dma_dabufp++ = tiGetIntCount();
  *dma_dabufp++ = 0xdead;
  *dma_dabufp++ = 0xcebaf111;
  *dma_dabufp++ = 0xcebaf222;
  BANKCLOSE;

  /* Set TI output 0 low */
  /*BQ    tiSetOutputPort(0,0,0,0);*/

}

void
rocLoad()
{
  dalmaInit(1);
}

void
rocCleanup()
{
  printf("%s: Reset all Modules\n",__FUNCTION__);
#ifdef TI_MASTER
  tiResetSlaveConfig();
#endif
  dalmaClose();
}


void
rocSetTriggerSource(int source)
{
#ifdef TI_MASTER
  if(TIPRIMARYflag == 1)
    {
      printf("%s: ERROR: Trigger Source already enabled.  Ignoring change to %d.\n",
	     __func__, source);
    }
  else
    {
      rocTriggerSource = source;

      if(rocTriggerSource == 0)
	{
	  tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */
	}
      else
	{
	  tiSetTriggerSource(TI_TRIGGER_PULSER); /* Internal Pulser */
	}

      daLogMsg("INFO","Setting trigger source (%d)", rocTriggerSource);
    }
#else
  printf("%s: ERROR: TI is not Master  Ignoring change to %d.\n",
	 __func__, source);
#endif
}


/*
  Local Variables:
  compile-command: "make -k ti_master_list.so ti_slave_list.so ti_slave5_list.so"
  End:
 */
