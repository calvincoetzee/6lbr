#include <stdio.h>
#include "contiki.h"
#include "utils/syscalls.h"
#include "inc/lm3s6965.h"
#include <sys/types.h>
typedef char * caddr_t;

extern int  __HEAP_START;
extern int  __HEAP_END;

caddr_t _sbrk ( int incr )
{
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *)&__HEAP_START;
	}
	prev_heap = heap;
	/* check removed to show basic approach */

	if((heap + incr) >= (unsigned char *)&__HEAP_END) return((void *)-1);

	heap += incr;

	return (caddr_t) prev_heap;
}
caddr_t _sbrk_r ( int incr )
{
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *)&__HEAP_START;
	}
	prev_heap = heap;
	/* check removed to show basic approach */

	if((heap + incr) >= (unsigned char *)&__HEAP_END) return((void *)-1);

	heap += incr;

	return (caddr_t) prev_heap;
}
