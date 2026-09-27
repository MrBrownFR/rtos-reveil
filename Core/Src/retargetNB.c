// All credit to Carmine Noviello for this code
// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f030R8/system/src/retarget/retarget.c

#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>
#include "retarget.h"
#include <stdint.h>
#include <stdio.h>

#include "cmsis_os.h" //NB

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

UART_HandleTypeDef *gHuart;

SemaphoreHandle_t semaph_printf ; //NB p


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);//NB

void RetargetInit(UART_HandleTypeDef *huart) {
  gHuart = huart;

  semaph_printf = xSemaphoreCreateBinary(); //NB
//  configASSERT(semaph_printf != NULL);
  xSemaphoreGive(semaph_printf);

  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
   setvbuf(stdout, NULL, _IONBF, 0);
}

int _isatty(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 1;

  errno = EBADF;
  return 0;
}

int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    xSemaphoreTake(semaph_printf, portMAX_DELAY);   // attend que l'UART soit libre

    hstatus = HAL_UART_Transmit_IT(gHuart, (uint8_t *) ptr, len);
    if (hstatus != HAL_OK) {
      xSemaphoreGive(semaph_printf);
      return EIO;
    }

    // attend que CETTE transmission se termine (donné par le callback ISR)
    xSemaphoreTake(semaph_printf, portMAX_DELAY);
    xSemaphoreGive(semaph_printf);   // libère pour le prochain appel

    return len;
  }
  errno = EBADF;
  return -1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart ->Instance == gHuart ->Instance)
	xSemaphoreGiveFromISR(semaph_printf, NULL);
}



int _close(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}

int _lseek(int fd, int ptr, int dir) {
  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

int _read(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == STDIN_FILENO) {
    hstatus = HAL_UART_Receive(gHuart, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return 1;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat* st) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)
