/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/
#include <string.h>
#include "diskio.h"
#include "stm32f4xx.h"
#include "sdio_sdcard.h"   // 替换为正点原子的标准 SDIO 驱动头文件

/* 为每个设备定义一个物理编号 */
#define ATA			           0     // SD卡
#define SPI_FLASH		       1     // 预留外部SPI Flash使用

//固定只支持blocksize大小为512的卡，兼容大于512的卡时，该卡容量会变小
#define SD_BLOCKSIZE     512//SDCardInfo.CardBlockSize 


/*-----------------------------------------------------------------------*/
/* 获取设备状态                                                          */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (
	BYTE pdrv		/* 物理编号 */
)
{
	DSTATUS status = STA_NOINIT;
	
	switch (pdrv) {
		case ATA:	/* SD CARD */
			status &= ~STA_NOINIT;
			break;
    
		case SPI_FLASH:        /* SPI Flash */   
			break;

		default:
			status = STA_NOINIT;
	}
	return status;
}

/*-----------------------------------------------------------------------*/
/* 设备初始化                                                            */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
	BYTE pdrv				/* 物理编号 */
)
{
	DSTATUS status = STA_NOINIT;	
	switch (pdrv) {
		case ATA:	         /* SD CARD */
			if(SD_Init() == 0) // 0 表示成功
			{				
				status &= ~STA_NOINIT;
			}
			else 
			{
				status = STA_NOINIT;
			}
		
			break;
    
		case SPI_FLASH:    /* SPI Flash */ 
			break;
      
		default:
			status = STA_NOINIT;
	}
	return status;
}


/*-----------------------------------------------------------------------*/
/* 读扇区：读取扇区内容到指定存储区                                              */
/*-----------------------------------------------------------------------*/
DRESULT disk_read (
	BYTE pdrv,		/* 设备物理编号(0..) */
	BYTE *buff,		/* 数据缓存区 */
	DWORD sector,	/* 扇区首地址 */
	UINT count		/* 扇区个数(1..128) */
)
{
	DRESULT status = RES_PARERR;
	uint8_t SD_state = 0; // 改用标准的 u8 接受状态
	
	switch (pdrv) {
		case ATA:	/* SD CARD */						
			
			// 适配正点原子的 SDIO 驱动函数 (直接按扇区读)
			SD_state = SD_ReadDisk(buff, sector, count);

			if(SD_state != 0) // 0 表示成功
				status = RES_PARERR;
		  else
			  status = RES_OK;	
			break;  
			
		case SPI_FLASH:
		break;
    
		default:
			status = RES_PARERR;
	}
	return status;
}



/*-----------------------------------------------------------------------*/
/* 写扇区：见数据写入指定扇区空间上                                      */
/*-----------------------------------------------------------------------*/
#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			  /* 设备物理编号(0..) */
	const BYTE *buff,	/* 欲写入数据的缓存区 */
	DWORD sector,		  /* 扇区首地址 */
	UINT count			  /* 扇区个数(1..128) */
)
{
	DRESULT status = RES_PARERR;
	uint8_t SD_state = 0; // 改用标准 u8 接受状态
	
	if (!count) {
		return RES_PARERR;		/* Check parameter */
	}

	switch (pdrv) {
		case ATA:	/* SD CARD */  
		
			// 适配正点原子的 SDIO 驱动函数 (直接按扇区写)
			SD_state = SD_WriteDisk((uint8_t *)buff, sector, count);

			if(SD_state != 0) // 0 表示成功
				status = RES_PARERR;
		  else
			  status = RES_OK;	
		break;

		case SPI_FLASH:
		break;
    
		default:
			status = RES_PARERR;
	}
	return status;
}
#endif


/*-----------------------------------------------------------------------*/
/* 其他控制                                                              */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* 物理编号 */
	BYTE cmd,		  /* 控制指令 */
	void *buff		/* 写入或者读取数据地址指针 */
)
{
	DRESULT status = RES_PARERR;
	switch (pdrv) {
		case ATA:	/* SD CARD */
			switch (cmd) 
			{
				// Get R/W sector size (WORD) 
				case GET_SECTOR_SIZE :    
					*(WORD * )buff = SD_BLOCKSIZE;
				break;
				// Get erase block size in unit of sector (DWORD)
				case GET_BLOCK_SIZE :      
					*(DWORD * )buff = 1;
				break;

				case GET_SECTOR_COUNT:
					*(DWORD * )buff = SDCardInfo.CardCapacity/SDCardInfo.CardBlockSize;
					break;
				case CTRL_SYNC :
				break;
			}
			status = RES_OK;
			break;
    
		case SPI_FLASH:		      
		break;
    
		default:
			status = RES_PARERR;
	}
	return status;
}
#endif

							 
__weak DWORD get_fattime(void) {
	/* 返回当前时间戳 */
//	return	  ((DWORD)(2023) << 25)	/* Year 2015 */
//			| ((DWORD)4 << 21)				/* Month 1 */
//			| ((DWORD)19 << 16)				/* Mday 1 */
//			| ((DWORD)14 << 11)				/* Hour 0 */
//			| ((DWORD)45 << 5)				  /* Min 0 */
//			| ((DWORD)0 >> 1);				/* Sec 0 */
	return 0;
}

