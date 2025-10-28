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
    // 1. Inicializa a struct e preenche o serialPort
    LinkLayer connectionParameters = {
        .serialPort="",
        .role = strcmp(role, "tx") == 0 ? LlTx : LlRx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout};
    strcpy(connectionParameters.serialPort, serialPort);

    FILE *file = fopen(filename, "rb");

    // Verifica se o ficheiro abriu corretamente
    if (file == NULL) {
        perror("Erro ao abrir ficheiro");
        exit(-1);
    }
    
    // CORRIGIDO: Usar 'file' (FILE *) em vez de 'filename' (const char *)
    fseek(file, 0, SEEK_END); // Antigo: fseek(filename, 0, SEEK_END);
    long fileSize = ftell(file);    // Antigo: long fileSize = ftell(filename);
    fseek(file, 0, SEEK_SET);  // Antigo: fseek(filename, 0, SEEK_SET);

    // Variável para armazenar o File Descriptor (fd) retornado por llopen
    int fd = llopen(connectionParameters);
    
    // 2. Chama llopen com a struct connectionParameters
    if (fd < 0){ // Antigo: if (llopen(parameters)<0){
        printf("Failed to open link layer connection.\n");
        // Fecha o ficheiro antes de sair
        fclose(file); 
        exit(-1);
    }

    // --- CÓDIGO DO TRANSMISSOR: ENVIAR START PACKET E DADOS ---
    if (connectionParameters.role == LlTx) {
        printf("TX AL: Ligação estabelecida. Tamanho do ficheiro: %ld bytes.\n", fileSize);

        // TODO: Enviar Start Control Packet (usando buildStartPacket e llwrite)
        // unsigned char startPacket[MAX_PACKET_SIZE];
        // int packetSize = buildStartPacket(startPacket, fileSize, filename);
        // llwrite(fd, startPacket, packetSize);

        // TODO: Enviar Data Packets (lendo do 'file' e usando llwrite)
        // ...

        // TODO: Enviar End Control Packet (usando llwrite)
        // ...
    }
    // --- FIM DO CÓDIGO DO TRANSMISSOR ---

    // 3. Fecha a ligação da camada de ligação, passando o fd e o role
    if (llclose(fd, connectionParameters.role) < 0) { // Antigo: if (llclose())
        printf("Failed to close link layer connection.\n");
        // Fecha o ficheiro antes de sair
        fclose(file); 
        exit (-1);
    }

    // 4. Fecha o ficheiro da camada de aplicação
    fclose(file);
    printf("Transferência concluída e ficheiro fechado.\n");
}

int buildStartPacket(unsigned char *packet, long fileSize, const char *filename) {
    int i = 0;

    // Control field
    packet[i++] = CTRL_START;

    // --- File size TLV ---
    packet[i++] = T_FILE_SIZE;      
    
    // O tamanho do campo LENGTH é o tamanho do valor (sizeof(long))
    packet[i++] = sizeof(long);     
    
    // Coloca o tamanho do ficheiro (8 bytes no meu sistema, big-endian)
    for (int u = sizeof(long) - 1; u >= 0; u--) { 
        packet[i++] = (fileSize >> (8 * u)) & 0xFF;
    }

    // --- File name TLV ---
    int nameLen = strlen(filename);
    packet[i++] = T_FILE_NAME;      
    packet[i++] = nameLen;          
    memcpy(&packet[i], filename, nameLen);
    i += nameLen;

    return i; // total size of packet
}
