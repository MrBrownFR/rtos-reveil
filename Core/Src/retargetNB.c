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

static SemaphoreHandle_t uart_mutex;   // xSemaphoreCreateMutex()
static SemaphoreHandle_t uart_done;    // xSemaphoreCreateBinary()

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);//NB

void RetargetInit(UART_HandleTypeDef *huart) {
  gHuart = huart;

  uart_mutex = xSemaphoreCreateMutex();
  uart_done = xSemaphoreCreateBinary();

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

int _write(int fd, char *ptr, int len) {
  if (fd != STDOUT_FILENO && fd != STDERR_FILENO) {
    errno = EBADF;
    return -1;
  }

  xSemaphoreTake(uart_mutex, portMAX_DELAY);

  if (HAL_UART_Transmit_IT(gHuart, (uint8_t *)ptr, len) != HAL_OK) {
    xSemaphoreGive(uart_mutex);
    errno = EIO;
    return -1;
  }

  xSemaphoreTake(uart_done, portMAX_DELAY);   // attend le callback ISR
  xSemaphoreGive(uart_mutex);
  return len;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == gHuart->Instance) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(uart_done, &woken);
    portYIELD_FROM_ISR(woken);
  }
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
