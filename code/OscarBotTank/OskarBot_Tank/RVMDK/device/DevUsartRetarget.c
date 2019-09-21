#include <stdio.h>
#include "DevUsart.h"


#pragma import(__use_no_semihosting_swi)


struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;


int fputc(int ch, FILE *f) {
    return devUsartSendData(USART1, ch);
}

int fgetc(FILE *f) {
    return devUsartRecvData(USART1);
}


int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int ch) {
    devUsartSendData(USART1, ch);
}


void _sys_exit(int return_code) {
    while (1);    /* endless loop */
}
