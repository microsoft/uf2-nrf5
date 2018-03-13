#define UF2_VERSION "1.00"
#define PRODUCT_NAME "NRF52 Board"
#define BOARD_ID "NRF52-generic-v0"
#define INDEX_URL "https://pxt.io"
#define UF2_NUM_BLOCKS 8000
#define VOLUME_LABEL "NRFBOOT"
#define FLASH_SIZE (1024*1024)
// TODO
// where the UF2 files are allowed to write data - 0-0x3000 is MBR, and after 0xf1000 the bootloader starts
#define USER_FLASH_START 0x3000
#define USER_FLASH_END 0xf1000

#define FLASH_PAGE_SIZE 4096
