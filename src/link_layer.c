// ----------------------------------------------------
// Link Layer Protocol Implementation
// ----------------------------------------------------

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

// ----------------------------------------------------
// Constants & Globals
// ----------------------------------------------------
#define _POSIX_SOURCE 1 // POSIX compliant source

// HDLC Control/Address Fields (Valores padrão)
#define FLAG 0x7E
#define A_TRX 0x03  // Endereço para comandos (Tx->Rx)
#define A_RCV 0x01  // Endereço para respostas (Rx->Tx)
#define C_SET 0x03  // Set Asynchronous Balanced Mode
#define C_UA 0x07   // Unnumbered Acknowledge
#define C_DISC 0x0B // Disconnect


int alarmEnabled = FALSE;
int alarmCount = 0;
static LinkLayer currentLinkLayer;

// ----------------------------------------------------
// Alarm Handler
// ----------------------------------------------------
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}

// ----------------------------------------------------
// Função Auxiliar: Enviar Frame de Controlo (SET, UA, DISC, etc.)
// ----------------------------------------------------
int sendControlFrame(unsigned char A, unsigned char C)
{
    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = A ^ C; // BCC1
    frame[4] = FLAG;

    int bytes_written = writeBytesSerialPort(frame, 5);
    if (bytes_written != 5)
    {
        printf("Erro ao escrever frame de controlo (A=0x%02X, C=0x%02X)\n", A, C);
        return -1;
    }
    return 0;
}

// ----------------------------------------------------
// Máquina de Estados para Receber Frames de Controlo (SET/UA/DISC)
// ----------------------------------------------------
enum State {
    START_S,
    FLAG_RCV_S,
    A_RCV_S,
    C_RCV_S,
    BCC1_RCV_S,
    STOP_S
};

int receiveControlFrame(unsigned char *frame, unsigned char A_expected, unsigned char C_expected)
{
    enum State state = START_S;
    unsigned char byte;
    int res;
    int i = 0;

    while (state != STOP_S)
    {
        res = readByteSerialPort(&byte);
        if (res == -1)
        {
            perror("readByteSerialPort error");
            return 0;
        }

        if (res == 1)
        {
            switch (state)
            {
            case START_S:
                if (byte == FLAG)
                {
                    frame[i++] = byte;
                    state = FLAG_RCV_S;
                }
                break;

            case FLAG_RCV_S:
                if (byte == A_expected)
                {
                    frame[i++] = byte;
                    state = A_RCV_S;
                }
                else if (byte == FLAG)
                {
                    i = 1;
                }
                else
                {
                    state = START_S;
                    i = 0;
                }
                break;

            case A_RCV_S:
                if (byte == C_expected)
                {
                    frame[i++] = byte;
                    state = C_RCV_S;
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV_S;
                    i = 1;
                }
                else
                {
                    state = START_S;
                    i = 0;
                }
                break;

            case C_RCV_S:
            {
                unsigned char bcc1_calculated = frame[1] ^ frame[2];
                if (byte == bcc1_calculated)
                {
                    frame[i++] = byte;
                    state = BCC1_RCV_S;
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV_S;
                    i = 1;
                }
                else
                {
                    printf("Rx: BCC1 incorreto (esperado: 0x%02X, recebido: 0x%02X)\n", bcc1_calculated, byte);
                    state = START_S;
                    i = 0;
                }
                break;
            }

            case BCC1_RCV_S:
                if (byte == FLAG)
                {
                    frame[i++] = byte;
                    state = STOP_S;
                }
                else
                {
                    state = START_S;
                    i = 0;
                }
                break;

            case STOP_S:
                break;
            }
        }
    }

    return 1; // sucesso
}

// ----------------------------------------------------
// Transmissor: Envia SET e Espera UA (com retransmissão)
// ----------------------------------------------------
int llopen_transmitter(LinkLayer parameters)
{
    int fd = openSerialPort(parameters.serialPort, parameters.baudRate);
    if (fd < 0)
        return -1;

    currentLinkLayer = parameters;

    (void)signal(SIGALRM, alarmHandler);
    unsigned char rx_frame[5];
    int success = 0;

    for (alarmCount = 0; alarmCount < parameters.nRetransmissions && !success;)
    {
        printf("TX: Tentativa %d/%d - Enviando SET...\n", alarmCount + 1, parameters.nRetransmissions);

        sendControlFrame(A_TRX, C_SET);

        alarmEnabled = TRUE;
        alarm(parameters.timeout); 

        while (alarmEnabled && !success)
        {
            if (receiveControlFrame(rx_frame, A_RCV, C_UA))
            {
                printf("TX: UA recebido. Ligação estabelecida.\n");
                success = 1;
                alarm(0); 
            }
        }

        if (!success)
            printf("TX: Timeout. Retransmitindo...\n");
    }

    if (!success)
    {
        printf("TX: Falha após %d tentativas.\n", parameters.nRetransmissions);
        closeSerialPort();
        return -1;
    }

    return fd;
}

// ----------------------------------------------------
// Recetor: Espera SET, Envia UA
// ----------------------------------------------------
int llopen_receiver(LinkLayer parameters)
{
    int fd = openSerialPort(parameters.serialPort, parameters.baudRate);
    if (fd < 0)
        return -1;
    
    currentLinkLayer = parameters;

    unsigned char rx_frame[5];
    printf("RX: À espera da Frame SET...\n");

    while (1) 
    {
        if (receiveControlFrame(rx_frame, A_TRX, C_SET))
        {
            printf("RX: Frame SET recebida.\n");
            break;
        }
       
    }


    if (sendControlFrame(A_RCV, C_UA) < 0)
    {
        printf("RX: Erro ao enviar UA.\n");
        closeSerialPort();
        return -1;
    }

    printf("RX: Ligação estabelecida (UA enviado).\n");
    return fd;
}

// ----------------------------------------------------
// Função Principal (llopen)
// ----------------------------------------------------
int llopen(LinkLayer parameters)
{
    if (parameters.role == LlTx)
        return llopen_transmitter(parameters);
    else
        return llopen_receiver(parameters);
}

// ----------------------------------------------------
// LLWRITE
// ----------------------------------------------------
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implementar envio de tramas I com byte stuffing e retransmissão.
    (void)buf;
    (void)bufSize;

    printf("llwrite(): Ainda não implementado.\n");
    return 0;
}

// ----------------------------------------------------
// LLREAD
// ----------------------------------------------------
int llread(unsigned char *packet)
{
    // TODO: Implementar receção de tramas I com destuffing e validação do BCC2.
    (void)packet;

    printf("llread(): Ainda não implementado.\n");
    return 0;
}

// ----------------------------------------------------
// LLCLOSE 
// ----------------------------------------------------
int llclose()
{
    printf("\n--- Fechando ligação (llclose) ---\n");

    int success = 0;
    unsigned char rx_frame[5];

    if (currentLinkLayer.role == LlTx) 
    {
        (void)signal(SIGALRM, alarmHandler); 

        for (alarmCount = 0; alarmCount < currentLinkLayer.nRetransmissions && !success;)
        {
            printf("TX LLCLOSE: Tentativa %d/%d - Enviando DISC (comando)...\n", alarmCount + 1, currentLinkLayer.nRetransmissions);

            sendControlFrame(A_TRX, C_DISC);

            alarmEnabled = TRUE;
            alarm(currentLinkLayer.timeout); 

            while (alarmEnabled && !success)
            {
               
                if (receiveControlFrame(rx_frame, A_RCV, C_DISC))
                {
                    printf("TX LLCLOSE: DISC (resposta) recebido. Enviando UA...\n");
                    
                   
                    sendControlFrame(A_TRX, C_UA);
                    
                    success = 1;
                    alarm(0);
                }
            }

            if (!success)
                printf("TX LLCLOSE: Timeout. Retransmitindo DISC...\n");
        }

        if (!success)
        {
            printf("TX LLCLOSE: Falha no handshake de fecho após %d tentativas.\n", currentLinkLayer.nRetransmissions);
        }
        else {
            printf("TX LLCLOSE: Handshake de fecho concluído com sucesso.\n");
        }
    }
    else // LlRx
    {
        printf("RX LLCLOSE: À espera da Frame DISC (comando)...\n");

        if (!receiveControlFrame(rx_frame, A_TRX, C_DISC)) 
        {
            printf("RX LLCLOSE: Falha ao receber DISC.\n");
        }
        else
        {
            printf("RX LLCLOSE: DISC (comando) recebido. Enviando DISC (resposta)...\n");

            sendControlFrame(A_RCV, C_DISC);
            
            (void)signal(SIGALRM, alarmHandler);
            alarmCount = 0;
            success = 0; 

            for (alarmCount = 0; alarmCount < currentLinkLayer.nRetransmissions && !success;)
            {
                printf("RX LLCLOSE: Tentativa %d/%d - À espera da Frame UA...\n", alarmCount + 1, currentLinkLayer.nRetransmissions);
                
                alarmEnabled = TRUE;
                alarm(currentLinkLayer.timeout); 
                
                while (alarmEnabled && !success)
                {
                    if (receiveControlFrame(rx_frame, A_TRX, C_UA))
                    {
                        printf("RX LLCLOSE: UA recebido. Handshake de fecho concluído.\n");
                        success = 1;
                        alarm(0);
                    }
                }
                
                if (!success)
                {
                    printf("RX LLCLOSE: Timeout. Retransmitindo DISC...\n");
                    sendControlFrame(A_RCV, C_DISC); 
                }
            }

            if (!success)
            {
                printf("RX LLCLOSE: Falha no handshake de fecho após %d tentativas.\n", currentLinkLayer.nRetransmissions);
            }
            else {
                printf("RX LLCLOSE: Handshake de fecho concluído com sucesso.\n");
            }
        }
    }
    
    if (closeSerialPort() == 0)
    {
        printf("Porta série fechada com sucesso.\n");
        return success ? 0 : -2; 
    }
    else
    {
        printf("Erro ao fechar porta série.\n");
        return -1;
    }
}