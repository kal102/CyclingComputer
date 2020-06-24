#ifndef SD_STM32_H
#define SD_STM32_H

DSTATUS card_initialize (BYTE drv);
DSTATUS card_status (BYTE drv);
DRESULT card_read (BYTE drv, BYTE *buff, DWORD sector, UINT count);
DRESULT card_write (BYTE drv, const BYTE *buff, DWORD sector, UINT count);
DRESULT card_ioctl (BYTE drv, BYTE cmd, void *buff);
void card_timerproc (void);

#endif
