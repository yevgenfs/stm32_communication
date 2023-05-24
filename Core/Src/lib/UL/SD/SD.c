/*
 * SD.c
 *
 *  Created on: 16 трав. 2023 р.
 *      Author: yevhen.surkov
 */

#include "SD.h"
#include "stm32f7xx_hal.h"
#include  "../../drivers/SPI/SPI.h"

#define TRUE  1
#define FALSE 0
#define bool BYTE

extern SPI_HandleTypeDef 	hspi1;
extern spi_t 	                obj_spi;
#define HSPI_SDCARD		&obj_spi
#define	SD_CS_PORT		GPIOB
#define SD_CS_PIN		GPIO_PIN_1

extern volatile uint16_t u16_timer1, u16_timer2;					/* 1ms Timer Counter */

static volatile DSTATUS objS_stat = STA_NOINIT;	/* Disk Status */
static uint8_t u8S_card_type;                    /* Type 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t u8LS_power_flag = 0;				/* Power flag */


/***************************************
 * SD functions
 **************************************/

/* wait SD ready */
static uint8_t SD_ready_wait(void)
{
  uint8_t u8L_res;

  /* timeout 500ms */
  u16_timer2 = 500;

  /* if SD goes ready, receives 0xFF */
  do
  {
    u8L_res = SPI_rx_byte(HSPI_SDCARD);
  }
  while ((u8L_res != 0xFF) && u16_timer2);

  return u8L_res;
}

/* power on */
static void SD_power_on(void)
{
  uint8_t u8L_args[6];
  uint32_t u32L_cnt = 0x1FFF;

  /* transmit bytes to wake up */
  SPI_deselect(SD_CS_PORT, SD_CS_PIN);
  for (int i = 0; i < 10; i++)
  {
    SPI_tx_byte(HSPI_SDCARD, 0xFF);
  }

  /* slave select */
  SPI_select(SD_CS_PORT, SD_CS_PIN);

  /* make idle state */
  u8L_args[0] = CMD0; /* CMD0:GO_IDLE_STATE */
  u8L_args[1] = 0;
  u8L_args[2] = 0;
  u8L_args[3] = 0;
  u8L_args[4] = 0;
  u8L_args[5] = 0x95; /* CRC */

  SPI_tx_buffer(HSPI_SDCARD, u8L_args, sizeof(u8L_args));

  /* wait response */
  while ((SPI_rx_byte(HSPI_SDCARD) != 0x01) && u32L_cnt)
  {
    u32L_cnt--;
  }

  SPI_deselect(SD_CS_PORT, SD_CS_PIN);
  SPI_tx_byte(HSPI_SDCARD, 0XFF);

  u8LS_power_flag = 1;
}

/* power off */
static void SD_power_off(void)
{
  u8LS_power_flag = 0;
}

/* check power flag */
static uint8_t SD_check_power(void)
{
  return u8LS_power_flag;
}

/* receive data block */
static bool SD_rx_data_block(BYTE* u8PL_buff, UINT u16L_len)
{
  uint8_t u8L_token;

  /* timeout 200ms */
  u16_timer1 = 200;

  /* loop until receive a response or timeout */
  do
  {
    u8L_token = SPI_rx_byte(HSPI_SDCARD);
  }
  while ((u8L_token == 0xFF) && u16_timer1);

  /* invalid response */
  if (u8L_token != 0xFE)
  {
      return FALSE;
  }

  /* receive data */
  do
  {
    SPI_rx_byte_ptr(HSPI_SDCARD, u8PL_buff++);
  }
  while (u16L_len--);

  /* discard CRC */
  SPI_rx_byte(HSPI_SDCARD);
  SPI_rx_byte(HSPI_SDCARD);

  return TRUE;
}

/* transmit data block */
#if _USE_WRITE == 1
static bool SD_tx_data_block(const uint8_t* u8PL_buff, BYTE u8L_token)
{
  uint8_t resp;
  uint8_t i = 0;

  /* wait SD ready */
  if (SD_ready_wait () != 0xFF)
    return FALSE;

  /* transmit u8L_token */
  SPI_tx_byte(HSPI_SDCARD, u8L_token);

  /* if it's not STOP u8L_token, transmit data */
  if (u8L_token != 0xFD)
  {
    SPI_tx_buffer(HSPI_SDCARD, (uint8_t*) u8PL_buff, 512);

    /* discard CRC */
    SPI_rx_byte(HSPI_SDCARD);
    SPI_rx_byte(HSPI_SDCARD);

    /* receive response */
    while (i <= 64)
    {
      resp = SPI_rx_byte(HSPI_SDCARD);

      /* transmit 0x05 accepted */
      if ((resp & 0x1F) == 0x05)
        break;

      i++;
    }

    /* recv buffer clear */
    while (SPI_rx_byte(HSPI_SDCARD) == 0);
  }

  /* transmit 0x05 accepted */
  if ((resp & 0x1F) == 0x05)
    return TRUE;

  return FALSE;
}
#endif /* _USE_WRITE */

/* transmit command */
static BYTE SD_SendCmd(BYTE u8L_cmd, uint32_t u32L_arg)
{
  uint8_t u8L_crc, u8L_res;

  /* wait SD ready */
  if (SD_ready_wait () != 0xFF)
    return 0xFF;

  /* transmit command */
  SPI_tx_byte(HSPI_SDCARD, u8L_cmd); /* Command */
  SPI_tx_byte(HSPI_SDCARD, (uint8_t) (u32L_arg >> 24)); /* Argument[31..24] */
  SPI_tx_byte(HSPI_SDCARD, (uint8_t) (u32L_arg >> 16)); /* Argument[23..16] */
  SPI_tx_byte(HSPI_SDCARD, (uint8_t) (u32L_arg >> 8)); /* Argument[15..8] */
  SPI_tx_byte(HSPI_SDCARD, (uint8_t) u32L_arg); /* Argument[7..0] */

  /* prepare CRC */
  if (u8L_cmd == CMD0)
    u8L_crc = 0x95; /* CRC for CMD0(0) */
  else if (u8L_cmd == CMD8)
    u8L_crc = 0x87; /* CRC for CMD8(0x1AA) */
  else
    u8L_crc = 1;

  /* transmit CRC */
  SPI_tx_byte(HSPI_SDCARD, u8L_crc);

  /* Skip a stuff byte when STOP_TRANSMISSION */
  if (u8L_cmd == CMD12)
    SPI_rx_byte(HSPI_SDCARD);

  /* receive response */
  uint8_t n = 10;
  do
  {
    u8L_res = SPI_rx_byte(HSPI_SDCARD);
  }
  while ((u8L_res & 0x80) && --n);

  return u8L_res;
}

/***************************************
 * user_diskio.c functions
 **************************************/

/* initialize SD */
DSTATUS SD_disk_initialize(BYTE u8L_drv)
{
  uint8_t u8L_n, u8L_type, u8L_ocr[4];

  /* single drive, drv should be 0 */
  if (u8L_drv)
    return STA_NOINIT;

  /* no disk */
  if (objS_stat & STA_NODISK)
    return objS_stat;

  /* power on */
  SD_power_on();

  /* slave select */
  SPI_select(SD_CS_PORT, SD_CS_PIN);

  /* check disk type */
  u8L_type = 0;

  /* send GO_IDLE_STATE command */
  if (SD_SendCmd(CMD0, 0) == 1)
  {
    /* timeout 1 sec */
    u16_timer1 = 1000;

    /* SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html */
    if (SD_SendCmd(CMD8, 0x1AA) == 1)
    {
      /* operation condition register */
      for (u8L_n = 0; u8L_n < 4; u8L_n++)
      {
        u8L_ocr[u8L_n] = SPI_rx_byte(HSPI_SDCARD);
      }

      /* voltage range 2.7-3.6V */
      if (u8L_ocr[2] == 0x01 && u8L_ocr[3] == 0xAA)
      {
        /* ACMD41 with HCS bit */
	do
	{
	  if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0)
	    break;
	}
	while (u16_timer1);

	/* READ_OCR */
	if (u16_timer1 && SD_SendCmd(CMD58, 0) == 0)
	{
	  /* Check CCS bit */
	  for (u8L_n = 0; u8L_n < 4; u8L_n++)
	  {
	    u8L_ocr[u8L_n] = SPI_rx_byte(HSPI_SDCARD);
	  }

	  /* SDv2 (HC or SC) */
	   u8L_type = (u8L_ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
	}
      }
    }
    else
    {
      /* SDC V1 or MMC */
      u8L_type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? CT_SD1 : CT_MMC;

      do
      {
        if (u8L_type == CT_SD1)
	{
	  if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0)
	    break; /* ACMD41 */
	}
	else
	{
	  if (SD_SendCmd(CMD1, 0) == 0)
	    break; /* CMD1 */
	}

      }
      while (u16_timer1);

      /* SET_BLOCKLEN */
      if (!u16_timer1 || SD_SendCmd(CMD16, 512) != 0)
        u8L_type = 0;
    }
  }

  u8S_card_type = u8L_type;

  /* Idle */
  SPI_deselect(SD_CS_PORT, SD_CS_PIN);
  SPI_rx_byte(HSPI_SDCARD);

  /* Clear STA_NOINIT */
  if (u8L_type)
  {
    objS_stat &= ~STA_NOINIT;
  }
  else
  {
    /* Initialization failed */
    SD_power_off();
  }

  return objS_stat;
}

/* return disk status */
DSTATUS SD_disk_status(BYTE u8L_pdrv)
{
  if(u8L_pdrv)
  {
    return STA_NOINIT;
  }

  return objS_stat;
}

/* read sector */
DRESULT SD_disk_read(BYTE u8L_pdrv, BYTE* u8PL_buff, DWORD objL_sector, UINT u16L_count)
{
  /* pdrv should be 0 */
  if (u8L_pdrv || !u16L_count)
    return RES_PARERR;

  /* no disk */
  if (objS_stat & STA_NOINIT)
    return RES_NOTRDY;

  /* convert to byte address */
  if (!(u8S_card_type & CT_SD2))
    objL_sector *= 512;

  SPI_select(SD_CS_PORT, SD_CS_PIN);

  if (u16L_count == 1)
  {
    /* READ_SINGLE_BLOCK */
    if ((SD_SendCmd(CMD17, objL_sector) == 0) && SD_rx_data_block(u8PL_buff, 512))
      u16L_count = 0;
  }
  else
  {
    /* READ_MULTIPLE_BLOCK */
    if (SD_SendCmd(CMD18, objL_sector) == 0)
    {
      do
      {
	if (!SD_rx_data_block(u8PL_buff, 512))
	  break;
	    u8PL_buff += 512;
      }
      while (--u16L_count);

      /* STOP_TRANSMISSION */
      SD_SendCmd (CMD12, 0);
    }
  }

  /* Idle */
  SPI_deselect(SD_CS_PORT, SD_CS_PIN);
  SPI_rx_byte(HSPI_SDCARD);

  return u16L_count ? RES_ERROR : RES_OK;
}

/* write sector */
#if _USE_WRITE == 1
DRESULT SD_disk_write(BYTE u8L_pdrv, const BYTE* u8PL_buff, DWORD objL_sector, UINT u16L_count)
{
  /* pdrv should be 0 */
  if (u8L_pdrv || !u16L_count)
    return RES_PARERR;

  /* no disk */
  if (objS_stat & STA_NOINIT)
    return RES_NOTRDY;

  /* write protection */
  if (objS_stat & STA_PROTECT)
    return RES_WRPRT;

  /* convert to byte address */
  if (!(u8S_card_type & CT_SD2))
    objL_sector *= 512;

  SPI_select(SD_CS_PORT, SD_CS_PIN);

  if (u16L_count == 1)
  {
    /* WRITE_BLOCK */
    if ((SD_SendCmd(CMD24, objL_sector) == 0) && SD_tx_data_block(u8PL_buff, 0xFE))
      u16L_count = 0;
  }
  else
  {
    /* WRITE_MULTIPLE_BLOCK */
    if (u8S_card_type & CT_SD1)
    {
        SD_SendCmd(CMD55, 0);
        SD_SendCmd(CMD23, u16L_count); /* ACMD23 */
    }

    if (SD_SendCmd(CMD25, objL_sector) == 0)
    {
      do
      {
        if (!SD_tx_data_block(u8PL_buff, 0xFC))
          break;

	u8PL_buff += 512;
      }
      while (--u16L_count);

      /* STOP_TRAN token */
      if (!SD_tx_data_block(0, 0xFD))
      {
	u16L_count = 1;
      }
    }
  }

  /* Idle */
  SPI_deselect(SD_CS_PORT, SD_CS_PIN);
  SPI_rx_byte(HSPI_SDCARD);

  return u16L_count ? RES_ERROR : RES_OK;
}
#endif /* _USE_WRITE */

/* ioctl */
DRESULT SD_disk_ioctl(BYTE u8L_pdrv, BYTE u8L_ctrl, void* u8PL_buff)
{
  DRESULT objL_res;
  uint8_t u8L_n, u8L_csd[16], *u8PL_ptr = u8PL_buff;
  WORD objL_csize;

  /* pdrv should be 0 */
  if (u8L_pdrv)
    return RES_PARERR;

  objL_res = RES_ERROR;

  if (u8L_ctrl == CTRL_POWER)
  {
    switch (*u8PL_ptr)
    {
      case 0:
        SD_power_off (); /* Power Off */
        objL_res = RES_OK;
        break;
      case 1:
        SD_power_on (); /* Power On */
        objL_res = RES_OK;
        break;
      case 2:
        *(u8PL_ptr + 1) = SD_check_power ();
        objL_res = RES_OK; /* Power Check */
        break;
      default:
        objL_res = RES_PARERR;
    }
  }
  else
  {
    /* no disk */
    if (objS_stat & STA_NOINIT)
      return RES_NOTRDY;

    SPI_select(SD_CS_PORT, SD_CS_PIN);

    switch (u8L_ctrl)
    {
      case GET_SECTOR_COUNT:
       /* SEND_CSD */
       if ((SD_SendCmd(CMD9, 0) == 0) && SD_rx_data_block(u8L_csd, 16))
       {
         if ((u8L_csd[0] >> 6) == 1)
	 {
	   /* SDC V2 */
	   objL_csize = u8L_csd[9] + ((WORD) u8L_csd[8] << 8) + 1;
	   *(DWORD*) u8PL_buff = (DWORD) objL_csize << 10;
	 }
	 else
	 {
	   /* MMC or SDC V1 */
	   u8L_n = (u8L_csd[5] & 15) + ((u8L_csd[10] & 128) >> 7) + ((u8L_csd[9] & 3) << 1) + 2;
	   objL_csize = (u8L_csd[8] >> 6) + ((WORD) u8L_csd[7] << 2) + ((WORD) (u8L_csd[6] & 3) << 10) + 1;
	   *(DWORD*) u8PL_buff = (DWORD) objL_csize << (u8L_n - 9);
	 }

	 objL_res = RES_OK;
       }
       break;
     case GET_SECTOR_SIZE:
       *(WORD*) u8PL_buff = 512;
       objL_res = RES_OK;
       break;
     case CTRL_SYNC:
       if (SD_ready_wait() == 0xFF)
	 objL_res = RES_OK;
       break;
     case MMC_GET_CSD:
       /* SEND_CSD */
       if (SD_SendCmd(CMD9, 0) == 0 && SD_rx_data_block (u8PL_ptr, 16))
         objL_res = RES_OK;
       break;
     case MMC_GET_CID:
       /* SEND_CID */
       if (SD_SendCmd(CMD10, 0) == 0 && SD_rx_data_block (u8PL_ptr, 16))
         objL_res = RES_OK;
       break;
     case MMC_GET_OCR:
       /* READ_OCR */
       if (SD_SendCmd(CMD58, 0) == 0)
       {
         for (u8L_n = 0; u8L_n < 4; u8L_n++)
         {
           *u8PL_ptr++ = SPI_rx_byte(HSPI_SDCARD);
         }

         objL_res = RES_OK;
       }
     default:
	  objL_res = RES_PARERR;
     }

     SPI_deselect(SD_CS_PORT, SD_CS_PIN);
     SPI_rx_byte(HSPI_SDCARD);
  }

  return objL_res;
}
