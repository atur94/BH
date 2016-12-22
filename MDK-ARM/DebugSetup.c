/**
   @file fputc_debug.c
   @brief Trying to redirect printf() to debug port
   @date 2012/06/25
*/
 
#include <stdio.h>
#include <stm32f4xx.h>
#include "DebugSetup.h"


struct __FILE { int handle; /* Add whatever needed */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
  if (DEMCR & TRCENA) {
    while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch);
}
