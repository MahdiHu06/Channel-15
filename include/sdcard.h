#include "../include/ff.h" // FatFs library header

#define SD_MISO 12
#define SD_CS   18
#define SD_SCK  14
#define SD_MOSI 15

#define SD_SPI_INST spi1


void cd(int argc, char *argv[]);
void input(int argc, char *argv[]);
void ls(int argc, char *argv[]);
void mkdir(int argc, char *argv[]);
void mount(int argc, char *argv[]);
void pwd(int argc, char *argv[]);
void rm(int argc, char *argv[]);
void cat(int argc, char *argv[]);
void append(int argc, char *argv[]);
void date(int argc, char *argv[]);
void restart(int argc, char *argv[]);


FRESULT log_data_csv(const char *filename, int loggedData[10][3]);
int fetch_data_csv(const char *filename, int data[][3], int maxRows);
void init_spi_sdcard();
void disable_sdcard();
void enable_sdcard();
void sdcard_io_high_speed();
void init_sdcard_io();
FRESULT save_measurement(const char *filename, int temp, int pressure, int humidity);