// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


int sendFrame(unsigned char A, unsigned char C){
    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = A^C;
    frame[4] = FLAG;
    int bytesW = writeBytesSerialPort(frame,5);
    if (bytesW != 5){
        printf("Error writing into the serial port\n");
    }
}
int readFrame(unsigned char expectedA, unsigned char expectedC){
    //fazer maquina de estados louco(switch case)

}

int llopen(LinkLayer connectionParameters)
{
    openSerialPort(connectionParameters.serialPort,connectionParameters.baudRate);
    if (connectionParameters.role == LlTx){
        int ntries = 0;
        int sucess = 0;
        while (ntries < connectionParameters.nRetransmissions){
            sendframe(A_TX,C_SET);
            //falta acabar (cena do timeout)
            if (sucess) break;
            ntries +=1; 
    }
}
    else{
        if (readFrame(A_TX, C_SET) < 0) {
            printf("SET not received.\n");
            return -1;
        }
        sendFrame(A_RX, C_UA);

    }
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}

