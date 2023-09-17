/*************************************************************************
 *
 *  nps_vme_list.c - Library of routines for config and readout of
 *                    fadc250s using a JLab pipeline TI as a source
 *                    for trigger and syncreset.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10

/* Size in Bytes Original size -  Use setrf.tcl if greater than 60kB  */
//#define MAX_EVENT_LENGTH   10*1024*240
#define MAX_EVENT_LENGTH   100*(16*120*2+10*4)      /* Size in Bytes - modified Alexandre Apr 30th 2023 for 16 FADC x 120 samples x 10 events*/

#ifdef TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
#else
#ifdef TI_SLAVE5
#define TI_SLAVE
#define TI_FLAG TI_INIT_SLAVE_FIBER_5
#endif
/* TS trigger source (e.g. fiber), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
#endif
#define TI_ADDR    (21<<19)          /* GEO slot 21 */

// Fiber Latency Offset depends on fiber used

// TI-MASTER and TI_SLAVE5 used in NPS stand-alone
#if defined(TI_MASTER) || defined(TI_SLAVE5)
#define FIBER_LATENCY_OFFSET 0x10  /* longest from NPS-VME{2,3,4,5} to NPS-VME1 is 0xF */
// #warning Using 0x10 for Fiber Latency Offset
#else
#define FIBER_LATENCY_OFFSET 0xD0  /* longest from NPS-VME{1,2,3,4,5} HMS ROC1 is 0xBD */
// #warning Using 0xD0 for Fiber Latency Offset
#endif

#include <unistd.h>
#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "sdLib.h"
#include "fadcLib.h"        /* library of FADC250 routines */
#include "fadc250Config.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

/* Routines to add string buffers to banks */
#include "rocUtils.c"

#define BLOCKLEVEL  1
#define BUFFERLEVEL 5

void writeConfigToFile();

/* FADC Library Variables */
extern int fadcA32Base, nfadc;
#define NFADC     18
/* Address of first fADC250 */
#define FADC_ADDR (3<<19)
/* Increment address to find next fADC250 */
#define FADC_INCR (1<<19)
#define FADC_BANK 0x3

/* Filename (with path) for fa250 config*/
#include <libgen.h>
char fa250_config_file[256];

#define FADC_READ_CONF_FILE {				\
    fadc250Config("");					\
    if(strlen(fa250_config_file) > 0)			\
      fadc250Config(fa250_config_file);			\
  }

/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;

/* SD variables */
static unsigned int sdScanMask = 0;

#define FADC_SCALERS

#ifdef FADC_SCALERS
/* FADC Scalers */
// Define this to include scaler data in coda events
#define FADC_SCALER_BANKS
int scaler_period=2;
struct timespec last_time;
#include "../scaler_server/scale32LibNew.c"
#include "../scaler_server/linuxScalerLib.c"
#endif

#define VLD_READOUT
#ifdef VLD_READOUT
// VLD headers for VLD readout through shared memory
#include "vldLib.h"
#include "vldShm.h"
#define VLD_BANK 0x1ed
#endif

#define INTERNAL_FLAGS "ffile=/home/hccoda/nps-vme/cfg/coda.flags"
#include "usrstrutils.c"
#ifdef TI_MASTER
/* TI-Master Fiber-Slave configuration */
typedef struct
{
  int enable;
  int port;
  char rocname[64];
} TI_SLAVE_MAP;

enum sbsSlaves
  {
   eslave1 = 0,
   eslave2,
   eslave3,
   eslave4,
   eslave5,
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

  /* Configfile type
    - modify the path of the config file path, based on the configtype string

    - e.g.
    Define in COOL (jcedit):
      rol->usrConfig = "/home/coda/cfg/fa250.cfg";
    Define in COOL (jcedit):
      rol->usrString = "configtype=pedestal_suppression";

    -> Modified config file
      fa250_config_file = "/home/coda/cfg/pedestal_suppression/fa250.cfg";

    - fa250_config_file = rol->usrConfig, if configtype is not defined.

  */

  char configfile_copy[256];
  strncpy(configfile_copy, rol->usrConfig, 256);

  char *configfile_base = basename(configfile_copy);
  char *configfile_path = dirname(configfile_copy);

  /* Bail if either of those failed */
  if((configfile_base != NULL) && (configfile_path != NULL))
    {
      flagval = 0;
      flag = getflag("configtype");
      if(flag)
	{
	  char *configfile_type = getstr("configtype");
	  if(configfile_type != NULL)
	    {
	      printf("%s: configtype = %s\n",
		     __func__, configfile_type);
	      strncpy(fa250_config_file, configfile_path, 256);
	      strcat(fa250_config_file, "/");
	      strcat(fa250_config_file, configfile_type);
	      strcat(fa250_config_file, "/");
	      strcat(fa250_config_file, configfile_base);

	      daLogMsg("WARN",
		       "configtype = %s, updated config file path",
		       configfile_type);

	      free(configfile_type);
	    }
	  else
	    {
	      strncpy(fa250_config_file, rol->usrConfig, 256);
	    }
	}
      else
	{
	  strncpy(fa250_config_file, rol->usrConfig, 256);
	}
    }

  printf("%s: fa250_config_file = %s\n",
	 __func__, fa250_config_file);
}

/*
  Global to configure the trigger source
      0 : tsinputs
      1 : TI random pulser
      2 : TI fixed pulser

  Set with rocSetTriggerSource(int source);
*/
int rocTriggerSource = 0;
void rocSetTriggerSource(int source); // routine prototype


/* function prototype */
void rocTrigger(int arg);

void
rocDownload()
{
  unsigned short iflag;
  int ifa, stat;

 /* define Block level */
 blockLevel = BLOCKLEVEL;


#ifdef TI_MASTER
  /*****************
   *   TI SETUP
   *****************/

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

  /* Enable specific TS input bits (1-6) */
  tiEnableTSInput(
		  TI_TSINPUT_1 |
		  TI_TSINPUT_2 |
		  TI_TSINPUT_3 |
		  TI_TSINPUT_4 |
		  TI_TSINPUT_5 |
		  TI_TSINPUT_6
		);

  /* Load the trigger table that associates
   *    - TS#1,2,3,4,5,6 : Physics trigger,
   */
  tiLoadTriggerTable(3);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set the SyncReset width to 4 microSeconds */
  tiSetSyncResetType(1);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  /* Sync event every 1000 blocks */
  tiSetSyncEventInterval(1000);

  /* Set L1A prescale ... rate/(x+1) */
  tiSetPrescale(0);

  /* Set TS input #1 prescale rate/(2^(x-1) + 1)*/
  tiSetInputPrescale(1, 0);

  /* Add trigger latch pattern to datastream */
  tiSetFPInputReadout(1);
#endif
  int offset_fudge = 3;
  int fiber_latency_offset[16] =
    { 0,0,0,0, 0,0,0,0, 0,0,// ROC ID = 0 - 9
#if defined(TI_MASTER) || defined(TI_SLAVE5)
      /* Fiber offsets for NPS only configs */
      0x10 + offset_fudge, // 10: nps-vme1
      0x10 + offset_fudge, // 11: nps-vme2
      0x10 + offset_fudge, // 12: nps-vme3
      0x0E + offset_fudge, // 13: nps-vme4
      0x0E + offset_fudge, // 14: nps-vme5
#else
      /* Fiber offsets for NPS+HMS configs */
      0xD0, // 10: nps-vme1
      0xD1, // 11: nps-vme2
      0xD0, // 12: nps-vme3
      0xCE, // 13: nps-vme4
      0xCD, // 14: nps-vme5
#endif
      0, };  // 15
  int measured_fiber_latency = tiGetFiberLatencyMeasurement();

  /* Re-set the fiber delay */
  tiSetFiberDelay(measured_fiber_latency, fiber_latency_offset[ROCID]);

  /* Init the SD library so we can get status info */
  sdScanMask = 0;
  stat = sdInit(0);
  if (stat != OK)
    {
      printf("%s: WARNING: sdInit() returned %d\n",__func__, stat);
      daLogMsg("ERROR","SD not found");
    }

  /* FADC library init */
  faInit(FADC_ADDR, FADC_INCR, NFADC, FA_INIT_SKIP);

  faGStatus(0);
#ifdef FADC_SCALERS
  if(fadcscaler_init_crl()) {
    printf("Scalers initialized\n");
  } else {
    printf("Failed to initialize scalers\n");
  }
  set_runstatus(0);
#endif

#ifdef VLD_READOUT
  vldInit(0, 0, 0, 0); // Initialize library using auto-locate (all args = 0)
#endif

  tiStatus(0);
  sdStatus(0);
  faGStatus(0);
#ifdef VLD_READOUT
  if(vldGetNVLD() > 0)
    vldGStatus(1);
#endif

  printf("block level = %d  \n", blockLevel);
  printf("Fiber latency 0x%x\n",FIBER_LATENCY_OFFSET);

  printf("rocDownload: (a) User Download Executed\n");

}

void
rocPrestart()
{
  int ifa, if1;

#ifdef FADC_SCALERS
  /* Suspend scaler task */
  set_runstatus(1);
  clock_gettime(CLOCK_REALTIME, &last_time);
#endif

  rocTriggerSource = 0;

  /* Read User Flags with usrstringutils
     What's set
     - TI Slave Ports
     - VTP
     - configtype (config file path)
   */
  readUserFlags();

  /* Program/Init VME Modules Here */


  /*****************
   *   FADC SETUP
   *****************/

  /* FADC Initialization flags */
  int iflag = 0; /* NO SDC */
  iflag |= (1<<0);  /* VXS sync-reset */
  iflag |= FA_INIT_VXS_TRIG;  /* VXS trigger source */
  iflag |= FA_INIT_VXS_CLKSRC;  /* VXS 250MHz Clock source */

  fadcA32Base = 0x09000000;

  faInit(FADC_ADDR, FADC_INCR, NFADC, iflag);

  /* Just one FADC250 */
  if(nfadc == 1)
    faDisableMultiBlock();
  else
    faEnableMultiBlock(1);

  /* configure all modules based on config file */
  FADC_READ_CONF_FILE;

  for(ifa = 0; ifa < nfadc; ifa++)
    {
      /* Bus errors to terminate block transfers (preferred) */
      faEnableBusError(faSlot(ifa));

      /*trigger-related*/
      faResetMGT(faSlot(ifa),1);
      faSetTrigOut(faSlot(ifa), 7);

      /* Enable busy output when too many events are being processed */
      faSetTriggerBusyCondition(faSlot(ifa), 3);
    }

  sdSetActiveVmeSlots(faScanMask()); /* Tell the sd where to find the fadcs */


  for(ifa=0; ifa < nfadc; ifa++)
    {
      faSoftReset(faSlot(ifa),0);
      faResetToken(faSlot(ifa));
      faResetTriggerCount(faSlot(ifa));
      faEnableSyncReset(faSlot(ifa));
    }

#ifdef VLD_READOUT
  if(vldGetNVLD() > 0)
    vldShmResetCounts(1, 1);
#endif

#ifdef TI_MASTER
  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);
#endif
  tiSetTriggerPulse(1,25,3,0); // delay is second argument in units of 16ns

  DALMAGO;
  sdStatus(0);
  tiStatus(0);
  faGStatus(0);
#ifdef VLD_READOUT
  if(vldGetNVLD() > 0)
    vldGStatus(1);
#endif
  DALMASTOP;

  /* Add configuration files to user event type 137 */
  int maxsize = MAX_EVENT_LENGTH-128, inum = 0, nwords = 0;

  if(strlen(fa250_config_file) > 0)
    {
      UEOPEN(137, BT_BANK, 0);
      nwords = rocFile2Bank(fa250_config_file,
			    (uint8_t *)rol->dabufp,
			    ROCID, inum++, maxsize);
      if(nwords > 0)
	rol->dabufp += nwords;

      UECLOSE;
    }

  /* Write Hardware Config to file for log entry */
  if(ROCID == 10) // only write out for nps-vme1 (ROCID = 10)
    writeConfigToFile();

  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{
  int fadc_mode = 0;
  unsigned int pl=0, ptw=0, nsb=0, nsa=0, np=0;

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

  faGSetBlockLevel(blockLevel);

  /* Get the FADC mode and window size to determine max data size */
  faGetProcMode(faSlot(0), &fadc_mode, &pl, &ptw,
		&nsb, &nsa, &np);

  /* Set Max words from fadc (proc mode == 1 produces the most)
     nfadc * ( Block Header + Trailer + 2  # 2 possible filler words
               blockLevel * ( Event Header + Header2 + Timestamp1 + Timestamp2 +
	                      nchan * (Channel Header + (WindowSize / 2) )
             ) +
     scaler readout # 16 channels + header/trailer
   */
  MAXFADCWORDS = nfadc * (4 + blockLevel * (4 + 16 * (1 + (ptw / 2))) + 18);

  /*  Enable FADC */
  faGEnable(0, 0);


#ifdef TI_MASTER
  if(rocTriggerSource != 0)
    {
      printf("************************************************************\n");
      daLogMsg("INFO","TI Configured for Internal Pulser Triggers");
      printf("************************************************************\n");

      if(rocTriggerSource == 1)
	{
          tiLoadTriggerTable(0);
	  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
	  /* Enable Random at rate 500kHz/(2^10) = ~488Hz */
	  tiSetRandomTrigger(1,10);
	}

      if(rocTriggerSource == 2)
	{
          tiLoadTriggerTable(0);
	  /*    Enable fixed rate with period (ns)
		120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
		- arg2 = 0xffff - Continuous
		- arg2 < 0xffff = arg2 times
	  */
	  tiSoftTrig(1,0xffff,100,0);
	}
    }
#endif
  /* Interrupts/Polling enabled after conclusion of rocGo() */

#ifdef FADC_SCALERS
  /* Clear and enable FADC scalers */
  set_runstatus(1);
  printf("fadc scalers cleared\n");
  enable_scalers();
#endif

}

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

  /* FADC Disable */
  faGDisable(0);

  DALMAGO;
  sdStatus(0);
  tiStatus(0);
  faGStatus(0);
#ifdef VLD_READOUT
  if(vldGetNVLD() > 0)
    vldGStatus(1);
#endif
  DALMASTOP;

#ifdef FADC_SCALERS
  /* Resume stand alone scaler server */
  disable_scalers();
  set_runstatus(0);		/* Tell Stand alone scaler task to resume  */
#endif

  printf("rocEnd: Ended after %d events\n",tiGetIntCount());

}

void
rocTrigger(int arg)
{
  int ifa = 0, stat, nwords, dCnt;
  unsigned int datascan, scanmask;
  int roType = 2, roCount = 0, blockError = 0;
  int ii, islot;

  roCount = tiGetIntCount();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  dCnt = tiReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      daLogMsg("ERROR","No TI Trigger data or error.  dCnt = %d\n",dCnt);
    }
  else
    {
      dma_dabufp += dCnt;
    }

  /* fADC250 Readout */
  BANKOPEN(FADC_BANK, BT_UI4, blockLevel);

  /* Mask of initialized modules */
  scanmask = faScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faGBlockReady(scanmask, 100);
  stat = (datascan == scanmask);

  if(stat)
    {
      if(nfadc == 1)
	roType = 1;   /* otherwise roType = 2   multiboard reaodut with token passing */
      nwords = faReadBlock(0, dma_dabufp, MAXFADCWORDS, roType);

      /* Check for ERROR in block read */
      blockError = faGetBlockError(1);

      if(blockError)
	{
	  daLogMsg("ERROR","fadc Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		 faSlot(ifa), roCount, nwords);

	  for(ifa = 0; ifa < nfadc; ifa++)
	    faResetToken(faSlot(ifa));

	  if(nwords > 0)
	    dma_dabufp += nwords;
	}
      else
	{
	  dma_dabufp += nwords;
	  faResetToken(faSlot(0));
	}
    }
  else
    {
      daLogMsg("ERROR","Event %d: fadc Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }
  BANKCLOSE;

#ifdef VLD_READOUT
  if(vldGetNVLD() > 0)
    {
      BANKOPEN(VLD_BANK, BT_UI4, 0);

      nwords = vldShmReadBlock(dma_dabufp, 256);
      if(nwords > 0)
	{
	  dma_dabufp += nwords;
	}
      else
	{
	  daLogMsg("ERROR","Event %d: Error in VLD readout\n", roCount);
	}

      BANKCLOSE;
    }
#endif

  /* Check for SYNC Event */
  if(tiGetBlockSyncFlag() == 1)
    {
      int iflush = 0, maxflush = 10;
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  daLogMsg("ERROR","TI Data available (%d) after readout in SYNC event (%d)\n",
		   davail, tiGetIntCount());

	  iflush = 0;
	  while(tiBReady() && (++iflush < maxflush))
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  davail = faBready(faSlot(ifa));
	  if(davail > 0)
	    {
	      daLogMsg("ERROR", "fADC250 Data available (%d) after readout in SYNC event (%d)\n",
		     davail, tiGetIntCount());

	      iflush = 0;
	      while(faBready(faSlot(ifa)) && (++iflush < maxflush))
		{
		  vmeDmaFlush(faGetA32(faSlot(ifa)));
		}
	    }
	}


    }
#ifdef FADC_SCALERS
  if (scaler_period > 0) {
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    if((scaler_period>0 &&
	((now.tv_sec - last_time.tv_sec
	  + ((double)now.tv_nsec - (double)last_time.tv_nsec)/1000000000L) >= scaler_period))) {
#define FADC_SCALER_BANKS
#ifdef FADC_SCALER_BANKS
      BANKOPEN(9250,BT_UI4,0);
      read_fadc_scalers(&dma_dabufp,0);
      BANKCLOSE;
      BANKOPEN(9001,BT_UI4,syncFlag);
      read_ti_scalers(&dma_dabufp,0);
      BANKCLOSE;
#else
      read_fadc_scalers(0,0);
      read_ti_scalers(0,0);
#endif
      last_time = now;
      read_clock_channels();
    }
  }
#endif


}

void
rocLoad()
{
  dalmaInit(1);
}

void
rocCleanup()
{

  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faGReset(1);

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

void
writeConfigToFile()
{
  char str[16001];
  fadc250UploadAll(str, 16000);

  char host[256];
  int jj;

  gethostname(host, 256);	/* obtain our hostname - and drop any domain extension */
  for(jj = 0; jj < strlen(host); jj++)
    {
      if(host[jj] == '.')
	{
	  host[jj] = '\0';
	  break;
	}
    }

  char out_config_filename[256] =
    "/net/cdaqfs1/cdaqfs-coda-home/coda/coda/scripts/EPICS_logging/Sessions/NPS/";

  strcat(out_config_filename, host);
  strcat(out_config_filename, ".dat");

  FILE *out_fd = fopen(out_config_filename, "w+");

  if(out_fd != NULL)
    {
      /* This is who we are */
      fprintf(out_fd, "# host: %s\n", host);
      fprintf(out_fd, "# ROCID: %d\n", ROCID);
      fprintf(out_fd, "# Runnumber: %d\n", rol->runNumber);
      fprintf(out_fd, "# RunType: %d\n", rol->runType);
      fprintf(out_fd, "# usrString: %s\n", rol->usrString);
      fprintf(out_fd, "# usrConfig: %s\n", rol->usrConfig);
      fprintf(out_fd, "# FA250 Config: %s\n", fa250_config_file);
      fprintf(out_fd, "%s\n", str);

      fclose(out_fd);
    }
  else
    {
      perror("fopen");
    }

}

/*
  Local Variables:
  compile-command: "make -B nps_vme_master_list.so nps_vme_slave_list.so nps_vme_slave5_list.so "
  End:
 */
