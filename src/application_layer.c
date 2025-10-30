// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "serial_port.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h> 

#define CTRL_START 0x02
#define CTRL_END   0x03
#define CTRL_DATA  0x01

#define T_FILE_SIZE 0x00
#define T_FILE_NAME 0x01

// Definimos o tamanho máximo para o payload de DADOS (buffer que vai para llwrite)
#define MAX_PAYLOAD_SIZE 1000 
#define MAX_PACKET_SIZE 1024 // Buffer geral

// Variável para controlar o número de sequência dos pacotes de dados no TX
static unsigned char N_DATA = 0; 


int buildDataPacket(unsigned char *packet, const unsigned char *data, int dataSize);
int buildEndPacket(unsigned char *packet, long fileSize, const char *filename);
int buildStartPacket(unsigned char *packet, long fileSize, const char *filename);

/**
 * Constrói um pacote de dados (C=0x01) para a camada de aplicação.
 * Formato: C | N | L2 | L1 | Data...
 */
int buildDataPacket(unsigned char *packet, const unsigned char *data, int dataSize) {
    int i = 0;

    packet[i++] = CTRL_DATA;

    // Usamos N_DATA e depois incrementamos.
    packet[i++] = N_DATA; 
    N_DATA = (N_DATA + 1) % 256;

    unsigned short length = (unsigned short)dataSize;
    packet[i++] = (unsigned char)((length >> 8) & 0xFF); // L2 (Most Significant Byte)
    packet[i++] = (unsigned char)(length & 0xFF);        // L1 (Least Significant Byte)

    memcpy(&packet[i], data, dataSize);
    i += dataSize;

    return i; // Total size of the data packet
}

/**
 * Constrói um pacote de fim (C=0x03) para a camada de aplicação.
 * Inclui os campos TLV (File Size e File Name).
 */
int buildEndPacket(unsigned char *packet, long fileSize, const char *filename) {
    int i = 0;

    // Control field
    packet[i++] = CTRL_END;

    // --- File size TLV (T=0x00, L=sizeof(long), V=fileSize) ---
    packet[i++] = T_FILE_SIZE;       
    packet[i++] = sizeof(long);      
    for (int u = sizeof(long) - 1; u >= 0; u--) { 
        packet[i++] = (fileSize >> (8 * u)) & 0xFF;
    }

    // --- File name TLV (T=0x01, L=nameLen, V=filename) ---
    int nameLen = strlen(filename);
    packet[i++] = T_FILE_NAME;       
    packet[i++] = nameLen;           
    memcpy(&packet[i], filename, nameLen);
    i += nameLen;

    return i;
}

/**
 * Constrói um pacote de início (C=0x02) para a camada de aplicação.
 * Inclui os campos TLV (File Size e File Name).
 */
int buildStartPacket(unsigned char *packet, long fileSize, const char *filename) {
    int i = 0;

    // Control field
    packet[i++] = CTRL_START;

    packet[i++] = T_FILE_SIZE;       
    packet[i++] = sizeof(long);      
    for (int u = sizeof(long) - 1; u >= 0; u--) { 
        packet[i++] = (fileSize >> (8 * u)) & 0xFF;
    }

    // --- File name TLV (T=0x01, L=nameLen, V=filename) ---
    int nameLen = strlen(filename);
    packet[i++] = T_FILE_NAME;       
    packet[i++] = nameLen;           
    memcpy(&packet[i], filename, nameLen);
    i += nameLen;

    return i;
}

/**
 * Implementação da camada de aplicação (transmissor ou recetor).
 */
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    int fd;
    
    LinkLayer connectionParameters = {
        .serialPort="",
        .role = strcmp(role, "tx") == 0 ? LlTx : LlRx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout};
    
    strcpy(connectionParameters.serialPort, serialPort); 
    
    
    fd = llopen(connectionParameters);
    
    if (fd < 0){ 
        printf("Application Layer: Falha ao abrir a ligação.\n");
        exit(-1);
    }
    
    printf("Application Layer: Ligação estabelecida (FD: %d).\n", fd);

    if (connectionParameters.role == LlTx) {
        FILE *file = fopen(filename, "rb");
        long fileSize;

        if (file == NULL) {
            perror("TX AL: Erro ao abrir ficheiro para transmissão");
            llclose();
            exit(-1);
        }
        
        fseek(file, 0, SEEK_END);
        fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);

        printf("TX AL: Tamanho do ficheiro: %ld bytes. Iniciando transferência...\n", fileSize);

       
        unsigned char startPacket[MAX_PACKET_SIZE];
        int startPacketSize = buildStartPacket(startPacket, fileSize, filename);
        
        printf("TX AL: Enviando Start Packet (%d bytes)...\n", startPacketSize);
        if (llwrite(startPacket, startPacketSize) < 0) {
            printf("TX AL: Falha ao enviar Start Packet.\n");
            fclose(file);
            llclose();
            exit(-1);
        }

       
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

        fclose(file);
        printf("TX AL: Ficheiro de entrada fechado.\n");
        
     
        unsigned char endPacket[MAX_PACKET_SIZE];
        int endPacketSize = buildEndPacket(endPacket, fileSize, filename);
        
        printf("TX AL: Enviando End Packet (%d bytes)...\n", endPacketSize);
        if (llwrite(endPacket, endPacketSize) < 0) {
            printf("TX AL: Falha ao enviar End Packet.\n");
            llclose();
            exit(-1);
        }
    }

    else { // LlRx
        
        unsigned char *packet = (unsigned char*)malloc(MAX_PACKET_SIZE);
        if (packet == NULL) {
            perror("RX AL: Falha na alocação de memória");
            llclose();
            exit(-1);
        }

        long expectedFileSize = 0;
        char receivedFilename[256] = {0}; 

        printf("RX AL: Aguardando Start Packet...\n");
        int packetSize = llread(packet);

        if (packetSize < 0 || packet[0] != CTRL_START) {
            printf("RX AL: Erro ou pacote inválido na recepção do Start Packet. Código de Controlo: %x\n", packet[0]);
            free(packet);
            llclose();
            exit(-1);
        }
        
        int currentPos = 1;
        char tempFilename[256] = {0}; 
        
        while (currentPos < packetSize) {
            unsigned char T = packet[currentPos++];
            unsigned char L = packet[currentPos++];
            
            if (T == T_FILE_SIZE && L == sizeof(long)) {
                // T=0x00 (File Size)
                expectedFileSize = 0;
                for (int u = 0; u < L; u++) {
                    expectedFileSize = (expectedFileSize << 8) | packet[currentPos++];
                }
                printf("RX AL: Tamanho esperado do ficheiro: %ld bytes.\n", expectedFileSize);
            } 
            else if (T == T_FILE_NAME && L < 256) {
                memcpy(tempFilename, &packet[currentPos], L);
                tempFilename[L] = '\0';
                currentPos += L;
                printf("RX AL: Nome do ficheiro recebido no pacote: %s\n", tempFilename);
            } else {
                printf("RX AL: AVISO - TLV desconhecido ou inválido (T=%x, L=%d). Ignorando.\n", T, L);
                currentPos += L;
            }
        }
        
        if (expectedFileSize == 0 || strlen(tempFilename) == 0) {
            printf("RX AL: Falha ao extrair File Size ou File Name do Start Packet.\n");
            free(packet);
            llclose();
            exit(-1);
        }

    
        const char *finalOutputFilename = "penguin-received.gif";
        strcpy(receivedFilename, finalOutputFilename); 

        printf("RX AL: Nome do ficheiro de saída (forçado): %s\n", finalOutputFilename);
        
        FILE *outFile = fopen(finalOutputFilename, "wb");
        if (outFile == NULL) {
            perror("RX AL: Erro ao abrir ficheiro de saída");
            free(packet);
            llclose();
            exit(-1);
        }

      
        unsigned char N_EXPECTED = 0;
        long bytesReceived = 0;
        int finished = 0;

        printf("RX AL: Iniciando recepção dos Data Packets...\n");

        while (!finished) {
            int dataPacketSize = llread(packet); 
            
            if (dataPacketSize < 0) {
                printf("RX AL: Erro na recepção do pacote (llread falhou). Abortando.\n");
                finished = 1;
                break;
            }

            unsigned char C = packet[0];

            if (C == CTRL_END) {
                // Pacote de FIM (0x03) recebido.
                printf("RX AL: End Packet recebido. A terminar recepção de dados.\n");
                finished = 1; 
                break;
            } 
            else if (C == CTRL_DATA) {
                // Pacote de DADOS (0x01) recebido
                unsigned char N_RECEIVED = packet[1];
                
                if (N_RECEIVED != N_EXPECTED) {
                    printf("RX AL: Data Packet (N=%d) recebido, mas esperado N=%d. Ignorando pacote duplicado.\n", 
                           N_RECEIVED, N_EXPECTED);
                    continue; // Vai para a próxima iteração do loop (próximo llread)
                }

                unsigned short dataLength = (packet[2] << 8) | packet[3];
                
                unsigned char *dataPayload = &packet[4];
                
                size_t written = fwrite(dataPayload, 1, dataLength, outFile);
                
                if (written != dataLength) {
                    printf("RX AL: ERRO na escrita de %d bytes no ficheiro!\n", dataLength);
                }

                bytesReceived += dataLength;
                
                printf("RX AL: Recebido Data Packet (N=%d, Bytes=%d). Total: %ld/%ld\r", 
                       N_RECEIVED, dataLength, bytesReceived, expectedFileSize);
                fflush(stdout);

                N_EXPECTED = (N_EXPECTED + 1) % 256; 
            }
            else {
                printf("RX AL: Pacote de controlo desconhecido (%x) recebido. Ignorando.\n", C);
            }
        }
        
     

        fclose(outFile);
        free(packet);
        
        if (finished && bytesReceived == expectedFileSize) {
            printf("\nRX AL: Sucesso! Ficheiro de saída ('%s') guardado com %ld bytes.\n", receivedFilename, bytesReceived);
        } else {
             printf("\nRX AL: FALHA! Recebido %ld bytes, mas esperado %ld. (Fim: %s).", 
                   bytesReceived, expectedFileSize, finished ? "Sim" : "Não");
             printf(" O ficheiro foi tentado guardar como: '%s'\n", receivedFilename);
        }
        
        printf("RX AL: Ficheiro de saída fechado e memória libertada.\n");
    }
    
    if (llclose() < 0) { 
        printf("Application Layer: Falha ao fechar a ligação.\n");
        exit (-1);
    }

    printf("Application Layer: Transferência concluída e ligação llclose feita.\n");
}