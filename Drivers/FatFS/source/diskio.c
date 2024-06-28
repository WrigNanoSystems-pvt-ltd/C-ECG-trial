/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"     /* FatFs lower layer API */

#include "mscmem.h"

/*
Visit for file system cluster size
https://support.microsoft.com/en-gb/help/140365/default-cluster-size-for-ntfs-fat-and-exfat
*/


#define MX66_EXP_ID 			0xC2253C

/* Prototypes for module-only functions */
static DRESULT ctrl_sync(void *buff);

/* Globals */
unsigned int init_done = 0;

/* Locals */
static uint8_t rtc_en;


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/



DSTATUS disk_status (
    BYTE pdrv       /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS status = 0;


    if (MX66_ID() == MX66_EXP_ID) {
    	status = 0;
    }
    else{
    	init_done = 0;
    	status = STA_NOINIT | STA_NODISK;
    }

    return status;
}

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv               /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS status;
    
    rtc_en = 0;
#if (FF_FS_NORTC == 0)
    //Initialize RTC
    if (MXC_RTC->cn & MXC_F_RTC_CN_WE) {
	rtc_en = 1;
    } else {
	start_time_sec = (FF_NORTC_YEAR-1980)*SEC_IN_YEAR_AVG;
	start_time_sec += FF_NORTC_MON*SEC_IN_MONTH_AVG; 
	start_time_sec += FF_NORTC_MDAY*SEC_IN_DAY;
	if(RTC_init(MXC_RTC, start_time_sec, 0) == E_NO_ERROR) {
	    rtc_en = 1; 
	}
    }
#endif

#if FF_MAX_SS == 4096
    if ((MX66_Init() == E_NO_ERROR) && (MX66_Quad(1) == E_NO_ERROR)) {
    	init_done = 1;
    	status = 0;
    }
    else{
    	status = STA_NOINIT;
    }
#endif

#if FF_MAX_SS == 512
    mscmem_init();
    init_done = 1;
    status = 0;
#endif
	    
    return status;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,      /* Physical drive nmuber to identify the drive */
    BYTE *buff,     /* Data buffer to store read data */
    DWORD sector,   /* Start sector in LBA */
    UINT count      /* Number of sectors to read */
)
{
    DRESULT status;

    int index = 0;

    status = RES_OK;

#if FF_MAX_SS == 4096

    for(index = 0; index < count; index++){
    	if(MX66_Read_Sector(sector, buff) == E_NO_ERROR){
    		sector++;
    		buff += MX66_SECTOR_SIZE;
    	}
    	else{
    		status = RES_ERROR;
    		break;
    	}
    }
#endif

#if FF_MAX_SS == 512

    for(index = 0; index < count; index++){
    	mscmem_read(sector, buff);
    	sector++;
    	buff+=512;
    }


#endif

    return status;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
    BYTE pdrv,          /* Physical drive nmuber to identify the drive */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Start sector in LBA */
    UINT count          /* Number of sectors to write */
)
{
    DRESULT status;


    int index = 0;
    status = RES_OK;

#if FF_MAX_SS == 4096


    for(index = 0; index < count; index++){
    	if(MX66_Erase_Sector(sector) == E_NO_ERROR){
    		if(MX66_Write_Sector(sector, (uint8_t *)buff) == E_NO_ERROR){
    			sector++;
    			buff += MX66_SECTOR_SIZE;
    		}
    		else{
    			status = RES_ERROR;
    			break;
    		}
    	}
    	else{
    		status = RES_ERROR;
    		break;
    	}
    }

#endif

#if FF_MAX_SS == 512

    for(index = 0; index < count; index++){
    	/*mscmem_write function does not manipulate buff variable*/
        mscmem_write(sector, (uint8_t *)buff);
    	sector++;
    	buff+=512;
    }

#endif




    return status;   
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE pdrv,      /* Physical drive nmuber (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    DRESULT status;


    status = RES_OK;
#if FF_MAX_SS == 4096
    switch(cmd) {
        case CTRL_SYNC:
			/* Mandatory */
			status = ctrl_sync(buff);
	    break;
        case GET_SECTOR_COUNT:
        	/* Mandatory */
        	*(DWORD*)buff = MX66_NUM_SECTORS;
	    break;
        case GET_BLOCK_SIZE:
        	/* Mandatory */
        	*(DWORD*)buff = MX66_BLOCK_SIZE;
	    break;
		case GET_SECTOR_SIZE:
			*(DWORD*)buff = MX66_SECTOR_SIZE;
		break;
	default:
	    status = RES_PARERR;
	    break;
    }
#endif


#if FF_MAX_SS == 512

    switch(cmd) {
        case CTRL_SYNC:
			/* Mandatory */
			status = ctrl_sync(buff);
	    break;
        case GET_SECTOR_COUNT:
        	/* Mandatory */
        	*(DWORD*)buff = MX66_NUM_SECTORS * (MX66_SECTOR_SIZE / 512);
	    break;
        case GET_BLOCK_SIZE:
        	/* Mandatory */
        	*(DWORD*)buff = 1;
	    break;
		case GET_SECTOR_SIZE:
			*(DWORD*)buff = MX66_SECTOR_SIZE;
		break;
	default:
	    status = RES_PARERR;
	    break;
    }

#endif


    return status;
}

DWORD get_fattime(void) {
    if(rtc_en) {
        DWORD result;
        uint32_t seconds;
        uint8_t year, month, day, hour, minute, half_seconds;
        
        //Convert RTC Seconds to time
        seconds = MXC_RTC->sec + (FF_RTC_EPOCH_DELTA);
        year = seconds/SEC_IN_YEAR_AVG;    //year from epoch
        seconds = seconds%SEC_IN_YEAR_AVG; //seconds from Jan 1, $year
        month = seconds/SEC_IN_MONTH_AVG;
        seconds = seconds%SEC_IN_MONTH_AVG;
        day = seconds/SEC_IN_DAY;        //hours from 12:00am
        seconds = seconds%SEC_IN_DAY;    
        hour = seconds/SEC_IN_HOUR;
        seconds = seconds%SEC_IN_HOUR;
        minute = seconds/SEC_IN_MINUTE;
        seconds = seconds%SEC_IN_MINUTE;
        half_seconds = seconds*2;

        /* Mask bits for inclusion in result */
        year &= 0x7F;
        month &= 0x0F;
        day &= 0x1F;
        hour &= 0x1F;
        minute &= 0x3F;
        half_seconds &= 0x1F;

        /* Add fields into 32bit result */
        result = year<<25;
        result |= month<<21;
        result |= day<<16;
        result |= hour<<11;
        result |= minute<<5;
        result |= half_seconds;
        return result;
    }
    else {
        return RES_NOTRDY;    
    }
}

static DRESULT ctrl_sync(void *buff)
{
	mscmem_write_dirty_sector();
    return RES_OK;
}

