#include <stdio.h>
#include <sys/time.h>
#include "./ftd2xx.h"

int main(int argc, char *argv[])
{
	FT_STATUS	ftStatus;
	FT_HANDLE	ftHandle0;
	int iport = 0;
	
	if(argc > 1) {
		sscanf(argv[1], "%d", &iport);
	}
	else {
		iport = 0;
	}
	printf("opening port %d\n", iport);

	ftStatus = FT_Open(iport, &ftHandle0);
	
	if(ftStatus != FT_OK) {
		printf("FT_Open(%d) failed\n", iport);
		return 1;
	}

    ftStatus = FT_CyclePort(ftHandle0);
    if(ftStatus != FT_OK) {
        printf("FT_CyclePort failed\n");
        return 1;
    }

	printf("ftHandle0 = %p\n", ftHandle0);
	FT_Close(ftHandle0);
    return 0;
}