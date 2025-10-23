// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "serial_port.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define CTRL_START 0x02
#define CTRL_END   0x03
#define CTRL_DATA  0x01

#define T_FILE_SIZE 0x00
#define T_FILE_NAME 0x01

#define MAX_PACKET_SIZE 1024

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
  LinkLayer connectionParameters = {
        .serialPort = "",
        .role = strcmp(role, "tx") == 0 ? LlTx : LlRx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout};
    
    FILE *file = fopen(filename, "rb");
    fseek(filename, 0, SEEK_END);
    long fileSize = ftell(filename);
    fseek(filename, 0, SEEK_SET);

    if (llopen(parameters)<0){
        printf("Failed to open\n");
        exit(-1);
    }



    if (llclose()) {
        exit (-1);
    }
}
int buildStartPacket(unsigned char *packet, long fileSize, const char *filename) {
    int i = 0;

    // Control field
    packet[i++] = CTRL_START;

    // --- File size TLV ---
    packet[i++] = T_FILE_SIZE;     
    packet[i++] = sizeof(long);    
    for (int u = sizeof(long) - 1; u >= 0; u--) { 
        packet[i++] = (fileSize >> (8 * u)) & 0xFF;
    }

    int nameLen = strlen(filename);
    packet[i++] = T_FILE_NAME;     
    packet[i++] = nameLen;         
    memcpy(&packet[i], filename, nameLen);
    i += nameLen;

    return i; // total size of packet
}
