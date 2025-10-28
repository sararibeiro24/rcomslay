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

int alarmEnabled = FALSE;
int alarmCount = 0;

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
    frame[3] = A ^ C;
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


// Retorna 1 se a frame esperada for recebida, 0 caso contrário.
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
                    i = 1; // mantém apenas uma FLAG
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

    unsigned char rx_frame[5];
    printf("RX: À espera da Frame SET...\n");

    while (!receiveControlFrame(rx_frame, A_TRX, C_SET))
    {
        // continua à espera
    }

    printf("RX: Frame SET recebida.\n");

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

    // TODO: Implementar handshake de fecho (DISC/UA)
    // Transmissor: envia DISC, espera DISC, envia UA
    // Recetor: espera DISC, envia DISC, espera UA

    if (closeSerialPort() == 0)
    {
        printf("Porta série fechada com sucesso.\n");
        return 0;
    }
    else
    {
        printf("Erro ao fechar porta série.\n");
        return -1;
    }
}
