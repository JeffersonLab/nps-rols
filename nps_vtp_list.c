/*************************************************************************
 *
 *  nps_vtp_list.c -  Library of routines for readout of events using a
 *                    JLAB Trigger Interface V3 (TI) with a VTP in
 *                    CODA 3.0.
 *
 *                    This is for a VTP with serial connection to a TI
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_LENGTH 40960
#define MAX_EVENT_POOL   100

#include <VTP_source.h>
#include <byteswap.h>
/* Routines to add string buffers to banks */
#include "rocUtils.c"


#define USE_DMA
#define READOUT_TI
#define READOUT_VTP

#define VTP_BANK 0x56

#define MAXBUFSIZE 100000
unsigned int gDmaBufPhys_TI;
unsigned int gDmaBufPhys_VTP;


int blklevel = 1;
int maxdummywords = 200;

/* trigBankType:
   Type 0xff10 is RAW trigger No timestamps
   Type 0xff11 is RAW trigger with timestamps (64 bits)
*/
int trigBankType = 0xff10;
int firstEvent;

/**
                        DOWNLOAD
**/
void
rocDownload()
{

  if(vtpInit(VTP_INIT_CLK_VXS_250))
    {
      printf("vtpInit() **FAILED**. User should not continue.\n");
      daLogMsg("ERROR", "Failed to initialize VTP");

      return;
    }

  if(vtpDmaMemOpen(2, MAXBUFSIZE * 4) == OK)
    {
      printf("%s: VTP Memory allocation successful.\n", __func__);
    }
  else
    {
      daLogMsg("ERROR","VTP Memory allocation failed");
      return;
    }

  firstEvent = 1;
}

/**
                        PRESTART
**/
void
rocPrestart()
{
  VTPflag = 0;

  printf("calling VTP_READ_CONF_FILE ..\n");fflush(stdout);

  printf("%s: rol->usrConfig = %s\n",
	 __func__, rol->usrConfig);

  /* Read Config file and Intialize VTP */
  vtpInitGlobals();
  if(rol->usrConfig)
    vtpConfig(rol->usrConfig);


  printf("%s: Initialize DMA\n",
	 __func__);
  if(vtpDmaInit(VTP_DMA_TI) == OK)
    {
      printf("%s: TI DMA Initialized\n", __func__);
    }
  else
    {
      daLogMsg("ERROR","TI DMA Init Failed");
      return;
    }
  if(vtpDmaInit(VTP_DMA_VTP) == OK)
    {
      printf("%s: VTP DMA Initialized\n", __func__);
    }
  else
    {
      daLogMsg("ERROR","VTP DMA Init Failed");
      return;
    }

  DALMAGO;
  vtpDmaStatus(0);
  vtpSDPrintScalers();
  vtpTiLinkStatus();
  vtpSerdesStatusAll();
  DALMASTOP;

  /* Add configuration files to user event type 137 */
  int maxsize = MAX_EVENT_LENGTH-128, inum = 0, nwords = 0;

  if(rol->usrConfig)
    {
      UEOPEN(137, BT_BANK, 0);
      nwords = rocFile2Bank(rol->usrConfig,
			    (uint8_t *)rol->dabufp,
			    ROCID, inum++, maxsize);
      if(nwords > 0)
	rol->dabufp += nwords;

      UECLOSE;
    }

  printf(" Done with User Prestart\n");

}

/**
                        PAUSE
**/
void
rocPause()
{
  VTPflag = 0;
  CDODISABLE(VTP, 1, 0);
}

/**
                        GO
**/
void
rocGo()
{

  /* Clear TI Link recieve FIFO */
  vtpTiLinkResetFifo(1);

  if(vtpSerdesCheckLinks() == ERROR)
    {
      daLogMsg("ERROR","VTP Serdes links not up");
      return;
    }

  vtpSerdesStatusAll();

  /* If there's an error in the status, re-initialize */
  if(vtpTiLinkStatus() == ERROR)
    {
      printf("%s: WARN: Error from TI Link status.  Resetting.\n",
	     __func__);
      vtpTiLinkInit();
    }

  blklevel = vtpTiLinkGetBlockLevel(0);
  printf("Block level read from TI: %d\n", blklevel);
  printf("Setting VTP block level to: %d\n", blklevel);
  vtpSetBlockLevel(blklevel);

  vtpV7SetResetSoft(1);
  vtpV7SetResetSoft(0);

  vtpEbResetFifo();

/* Do DMA readout before Go enabled to clear out any buffered data
   - hack fix until problem with extra TI block header from past run is found */
#ifdef READOUT_TI
    vtpDmaStart(VTP_DMA_TI, vtpDmaMemGetPhysAddress(0), MAXBUFSIZE*4);
    vtpDmaWaitDone(VTP_DMA_TI);

    //    vtpEbTiReadEvent(gpDmaBuf, MAXBUFSIZE);
#endif

#ifdef READOUT_VTP
  vtpDmaStart(VTP_DMA_VTP, vtpDmaMemGetPhysAddress(1), MAXBUFSIZE*4);
  vtpDmaWaitDone(VTP_DMA_VTP);
#endif

  DALMAGO;
  vtpDmaStatus(0);
  vtpSDPrintScalers();
  vtpTiLinkStatus();
  vtpSerdesStatusAll();
  DALMASTOP;

  /* Enable to recieve Triggers */
  VTPflag = 1;
  CDOENABLE(VTP, 1, 0);
}

/**
                        END
**/
void
rocEnd()
{
  VTPflag = 0;
  CDODISABLE(VTP, 1, 0);

  DALMAGO;
  vtpDmaStatus(0);
  vtpSDPrintScalers();
  vtpTiLinkStatus();
  vtpSerdesStatusAll();
  DALMASTOP;

}

/**
                        READOUT
**/
void
rocTrigger(int EVTYPE)
{
  int ii;
  int len;
  volatile unsigned int *pBuf;


#ifdef READOUT_TI
  vtpDmaStart(VTP_DMA_TI, vtpDmaMemGetPhysAddress(0), MAXBUFSIZE * 4);
#endif

#ifdef READOUT_VTP
  vtpDmaStart(VTP_DMA_VTP, vtpDmaMemGetPhysAddress(1), MAXBUFSIZE * 4);
#endif

#ifdef READOUT_TI
  len = vtpDmaWaitDone(VTP_DMA_TI) >> 2;
  if(len) len--;
  pBuf = (volatile unsigned int *) vtpDmaMemGetLocalAddress(0);

  if(len > 1000)
    {
      printf("LEN1=%d\n", len);
      for(ii = 0; ii < len; ii++)
	printf("vtpti[%2d] = 0x%08x\n", (int)ii, pBuf[ii]);
    }

  len = vtpTIData2TriggerBank(pBuf, len);

  if(len > 0)
    {
      syncFlag = vtpTIGetBlockSyncFlag();

      /* Open an event, containing Banks */
      CEOPEN(ROCID, BT_BANK, blklevel);

      for(ii = 0; ii < len; ii++)
	{
	  *rol->dabufp++ = pBuf[ii];
	}
    }

#endif

#ifdef READOUT_VTP
  len = vtpDmaWaitDone(VTP_DMA_VTP) >> 2;
  if(len)
    len--;
  pBuf = (volatile unsigned int *) vtpDmaMemGetLocalAddress(1);

  if(len > (MAXBUFSIZE / 4))	/* if we are using more then 25% of the buffer, print message */
    {
      printf("LEN2=%d\n", len);
      for(ii = 0; ii < len; ii++)
	printf("vtp[%2d] = 0x%08x\n", (int)ii, pBuf[ii]);
    }

  CBOPEN(VTP_BANK, BT_UI4, blklevel);
  for(ii = 0; ii < len; ii++)
    {
      *rol->dabufp++ = bswap_32(pBuf[ii]);
    }
  CBCLOSE;
#endif


  /* Close event */
  CECLOSE;

}

/**
                        READOUT ACKNOWLEDGE
**/
void
rocTrigger_done()
{
  CDOACK(VTP, 0, 0);
}

/**
                        RESET
**/
void
rocReset()
{
#ifdef USE_DMA
  vtpDmaMemClose();
#endif
  vtpClose(VTP_FPGA_OPEN|VTP_I2C_OPEN|VTP_SPI_OPEN);
}

void
rocLoad()
{
  int stat;

  /* Open VTP library */
  stat = vtpOpen(VTP_FPGA_OPEN | VTP_I2C_OPEN | VTP_SPI_OPEN);
  if(stat < 0)
    {
      printf(" Unable to Open VTP driver library.\n");
    }

  dalmaInit(1);

}

void
rocCleanup()
{
  dalmaClose();

  /* Close the VTP Library */
  vtpClose(VTP_FPGA_OPEN|VTP_I2C_OPEN|VTP_SPI_OPEN);
}

/*
  Local Variables:
  compile-command: "make -k nps_vtp_list.so"
  End:
 */
