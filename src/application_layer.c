// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "serial_port.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define CTRL_START 0x02
#define CTRL_END   0x03
#define CTRL_DATA  0x01

#define T_FILE_SIZE 0x00
#define T_FILE_NAME 0x01

// Definimos o tamanho máximo para o payload de DADOS (buffer que vai para llwrite)
#define MAX_PAYLOAD_SIZE 1000 
#define MAX_PACKET_SIZE 1024 // Buffer geral

// Variável para controlar o número de sequência dos pacotes de dados
static unsigned char N_DATA = 0; 


int buildDataPacket(unsigned char *packet, const unsigned char *data, int dataSize);
int buildEndPacket(unsigned char *packet, long fileSize, const char *filename);
int buildStartPacket(unsigned char *packet, long fileSize, const char *filename);

int buildDataPacket(unsigned char *packet, const unsigned char *data, int dataSize) {
    int i = 0;

    // 1. Control field (C)
    packet[i++] = CTRL_DATA;

    // 2. Sequence Number (N)
    // Usamos N_DATA e depois incrementamos. O N_DATA só usa o bit 0-255.
    packet[i++] = N_DATA; 
    N_DATA = (N_DATA + 1) % 256;

    // 3. Length fields (L2, L1) - L = L2*256 + L1
    // L é o tamanho dos dados (dataSize)
    unsigned short length = (unsigned short)dataSize;
    packet[i++] = (unsigned char)((length >> 8) & 0xFF); // L2 (Most Significant Byte)
    packet[i++] = (unsigned char)(length & 0xFF);        // L1 (Least Significant Byte)

    // 4. Data payload (P1...Pk)
    memcpy(&packet[i], data, dataSize);
    i += dataSize;

    return i; // Total size of the data packet
}


int buildEndPacket(unsigned char *packet, long fileSize, const char *filename) {
    int i = 0;

    // Control field
    packet[i++] = CTRL_END;

    // --- File size TLV ---
    packet[i++] = T_FILE_SIZE;      
    packet[i++] = sizeof(long);     
    for (int u = sizeof(long) - 1; u >= 0; u--) { 
        packet[i++] = (fileSize >> (8 * u)) & 0xFF;
    }

    // --- File name TLV ---
    int nameLen = strlen(filename);
    packet[i++] = T_FILE_NAME;      
    packet[i++] = nameLen;          
    memcpy(&packet[i], filename, nameLen);
    i += nameLen;

    return i;
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

    // --- File name TLV ---
    int nameLen = strlen(filename);
    packet[i++] = T_FILE_NAME;      
    packet[i++] = nameLen;          
    memcpy(&packet[i], filename, nameLen);
    i += nameLen;

    return i;
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    int fd;
    
    // 1. Inicializa a struct e preenche o serialPort
    LinkLayer connectionParameters = {
        .serialPort="",
        .role = strcmp(role, "tx") == 0 ? LlTx : LlRx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout};
    
    strcpy(connectionParameters.serialPort, serialPort); 
    
    
    // 2. Chama llopen com a struct connectionParameters
    fd = llopen(connectionParameters);
    
    if (fd < 0){ 
        printf("Application Layer: Falha ao abrir a ligação.\n");
        exit(-1);
    }
    
    printf("Application Layer: Ligação estabelecida (FD: %d).\n", fd);

    // --- CÓDIGO DO TRANSMISSOR: ENVIAR START PACKET E DADOS ---
    if (connectionParameters.role == LlTx) {
        FILE *file = fopen(filename, "rb");
        long fileSize;

        if (file == NULL) {
            perror("Application Layer: Erro ao abrir ficheiro para transmissão");
            llclose(); // Fecha a ligação se o ficheiro falhar
            exit(-1);
        }
        
        // Calcula o tamanho do ficheiro
        fseek(file, 0, SEEK_END);
        fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);

        printf("TX AL: Tamanho do ficheiro: %ld bytes. Iniciando transferência...\n", fileSize);

        // ----------------------------------------------------
        // 3. ENVIO DO START CONTROL PACKET
        // ----------------------------------------------------
        unsigned char startPacket[MAX_PACKET_SIZE];
        int startPacketSize = buildStartPacket(startPacket, fileSize, filename);
        
        printf("TX AL: Enviando Start Packet (%d bytes)...\n", startPacketSize);
        if (llwrite(startPacket, startPacketSize) < 0) {
            printf("TX AL: Falha ao enviar Start Packet.\n");
            fclose(file);
            llclose();
            exit(-1);
        }

        // ----------------------------------------------------
        // 4. ENVIO DOS DATA PACKETS (LOOP)
        // ----------------------------------------------------
        unsigned char dataBuffer[MAX_PAYLOAD_SIZE];
        unsigned char dataPacket[MAX_PACKET_SIZE];
        int bytesRead;
        long bytesSent = 0;
        
        while ((bytesRead = fread(dataBuffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            
            int dataPacketSize = buildDataPacket(dataPacket, dataBuffer, bytesRead);
            
            if (llwrite(dataPacket, dataPacketSize) < 0) {
                printf("TX AL: Falha ao enviar Data Packet na sequência %d.\n", N_DATA);
                fclose(file);
                llclose();
                exit(-1);
            }
            bytesSent += bytesRead;
            printf("TX AL: Enviado Data Packet (N=%d, Bytes=%d). Total: %ld/%ld\r", 
                   (N_DATA == 0 ? 255 : N_DATA - 1), bytesRead, bytesSent, fileSize);
            fflush(stdout);
        }
        printf("\nTX AL: Todos os dados (%ld bytes) enviados.\n", bytesSent);

        // 5. Fecha o ficheiro de aplicação
        fclose(file);
        printf("TX AL: Ficheiro de entrada fechado.\n");
        
        // ----------------------------------------------------
        // 6. ENVIO DO END CONTROL PACKET
        // ----------------------------------------------------
        unsigned char endPacket[MAX_PACKET_SIZE];
        int endPacketSize = buildEndPacket(endPacket, fileSize, filename);
        
        printf("TX AL: Enviando End Packet (%d bytes)...\n", endPacketSize);
        if (llwrite(endPacket, endPacketSize) < 0) {
            printf("TX AL: Falha ao enviar End Packet.\n");
            llclose();
            exit(-1);
        }
    }
    // --- FIM DO CÓDIGO DO TRANSMISSOR ---

    // --- CÓDIGO DO RECETOR: RECEBER START PACKET E DADOS ---
    else { // LlRx
        printf("RX AL: Ligação estabelecida. Aguardando Start Packet...\n");
        // TODO: Chamar llread para receber o Start Packet
        // TODO: Criar e abrir o ficheiro de saída (com o nome recebido no Start Packet)
        // TODO: Chamar llread em loop para receber Data Packets
        // TODO: Fechar o ficheiro de saída
    }
    
    // 7. Fecha a ligação da camada de ligação
    if (llclose() < 0) { 
        printf("Application Layer: Falha ao fechar a ligação.\n");
        exit (-1);
    }

    printf("Application Layer: Transferência concluída e ligação llclose feita.\n");
}