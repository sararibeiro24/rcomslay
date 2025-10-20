// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole Lrole = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    LinkLayer parameters = {serialPort, Lrole, baudrate, nTries, timeout};
    
    if (llopen(parameters)<0){
        printf("Failed to open\n");
        exit(-1);
    }
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (llclose()) {
        exit (-1);
    }
}
