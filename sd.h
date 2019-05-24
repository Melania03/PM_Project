#ifndef SD_H
#define SD_H

#include "integer.h"


/*******************************************************\
 * Definitii pentru parametrizarea bibliotecii	       *
\*******************************************************/

/* Definitii pentru portul SS */
#define SD_PORT		PORTA
#define SD_DDR		DDRA
#define SD_SS		PA2

/* Controlul portului */
#define SD_SELECT()	SD_PORT &= ~(1<<SD_SS)	        /* SD SS = 0 */
#define SD_DESELECT()	SD_PORT |=  (1<<SD_SS)	        /* SD SS = 1 */
#define	SD_SELECTED()	!(SD_PORT &  (1<<SD_SS))	/* SD SS status (1 = selectat; 0 = deselectat) */

#define SD_DELAY()	__builtin_avr_delay_cycles(2500 / (1000000000/F_CPU))

/*******************************************************\
 * Definitii pentru utilizarea cardului SD	       *
\*******************************************************/

/* Comenzi pentru MMC/SDC */
#define CMD0			(0x40+0)	/* GO_IDLE_STATE */
#define CMD1			(0x40+1)	/* SEND_OP_COND (MMC) */
#define	ACMD41			(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8			(0x40+8)	/* SEND_IF_COND */
#define CMD16			(0x40+16)	/* SET_BLOCKLEN */
#define CMD17			(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD24			(0x40+24)	/* WRITE_BLOCK */
#define CMD55			(0x40+55)	/* APP_CMD */
#define CMD58			(0x40+58)	/* READ_OCR */

#define STA_NOINIT		0x01		/* Drive not initialized */
#define STA_NODISK		0x02		/* No medium in the drive */

/* Card type flags (CardType) */
#define CT_MMC			0x01		/* MMC ver 3 */
#define CT_SD1			0x02		/* SD ver 1 */
#define CT_SD2			0x04		/* SD ver 2 */
#define CT_SDC			(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK		0x08		/* Block addressing */

/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Function succeeded */
	RES_ERROR,		/* 1: Disk error */
	RES_NOTRDY,		/* 2: Not ready */
	RES_PARERR		/* 3: Invalid parameter */
} DRESULT;

/*******************************************************\
 * Functii pentru utilizarea cardului SD	       *
\*******************************************************/

DSTATUS disk_initialize();
DRESULT disk_readp(BYTE *buf, DWORD lba, WORD offset, WORD cnt);
DRESULT disk_writep(const BYTE *buf, DWORD sa);


#endif // SD_H
