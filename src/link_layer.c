// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


int sendFrame(unsigned char A, unsigned char C,unsigned char *data,int dataSize){// afinal meter para que a data tambem consiga se for control meter null ou 0 values na parte da data 
    unsigned char frame[4096]; 
    int i = 0;

    frame[i++] = FLAG;
    frame[i++] = A;
    frame[i++] = C;
    frame[i++] = A ^ C; 

    if (data != NULL && dataSize > 0) {
        
       
    frame[i++] = FLAG;
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
    // TODO: Implement this function(vai ser para o bytestuffing e o ntries)

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function(recebe os frames e manda ack)

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

