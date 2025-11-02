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
// Contantes e globais
// ----------------------------------------------------
#define _POSIX_SOURCE 1 // POSIX compliant source

// Fields
#define FLAG 0x7E
#define ESCAPE 0x7D
#define A_TRX 0x03  // Transmitter 
#define A_RCV 0x01  // Receiver
#define C_SET 0x03  // Set Asynchronous Balanced Mode
#define C_UA 0x07   // Unnumbered Acknowledge
#define C_DISC 0x0B // Disconnect

// I-Frame and Supervision Fields 
#define C_I_0  0x00 // I Frame, Ns=0
#define C_I_1  0x40 // I Frame, Ns=1
#define C_RR_0 0x05 // RR Frame, Nr=0 
#define C_RR_1 0x85 // RR Frame, Nr=1 
#define C_REJ_0 0x01 // REJ Frame, Nr=0 
#define C_REJ_1 0x81 // REJ Frame, Nr=1 

// Max Sizes 
#ifndef MAX_PAYLOAD_SIZE
#define MAX_PAYLOAD_SIZE 1024 
#endif
#define MAX_frame_SIZE (MAX_PAYLOAD_SIZE * 2 + 10)


int alarmEnabled = FALSE;
int alarmCount = 0;
static int Ns = 0; 
static int Nr_expected = 0; 
static LinkLayer currentLinkLayer;


// Alarm Handler
alarmHandler(int signal){ 
    alarmEnabled = FALSE;
    alarmCount++;
}

// Função Auxiliar: Enviar Frame de Controlo
int sendControlFrame(unsigned char A, unsigned char C){
    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = A ^ C; // BCC1
    frame[4] = FLAG;

    int bytes_written = writeBytesSerialPort(frame, 5);
    if (bytes_written != 5){
        printf("Erro ao escrever frame de controlo (A=0x%02X, C=0x%02X)\n", A, C);
        return -1;
    }
    return 0;
}


// Função Auxiliar: Enviar Frame de Supervisão (RR/REJ)
static int sendSupervisionFrame(unsigned char C){
    return sendControlFrame(A_RCV, C);
}

// Máquina de Estados
enum State {
    START_S,
    FLAG_RCV_S,
    A_RCV_S,
    C_RCV_S,
    BCC1_RCV_S,
    DATA_RCV_S,
    STOP_S
};

// Receção de Frames de Controlo
int receiveControlFrame(unsigned char *frame, unsigned char A_expected, unsigned char C_expected) {
    enum State state = START_S;
    unsigned char byte;
    int res;
    int i = 0;

    while (state != STOP_S) {
        res = readByteSerialPort(&byte);
        if (res == -1){
            perror("readByteSerialPort error");
            return 0;
        }

        if (alarmEnabled == FALSE && currentLinkLayer.role == LlTx) {
            return 0;
        }

        if (res == 1) {
            switch (state){
                case START_S:
                    if (byte == FLAG){ 
                        frame[i++] = byte;
                        state = FLAG_RCV_S;
                    }
                    break;

                case FLAG_RCV_S:
                    if (byte == A_expected) {
                        frame[i++] = byte;
                        state = A_RCV_S;
                    }
                    else if (byte == FLAG){ 
                        i = 1;
                    }
                    else {
                        state = START_S;
                        i = 0;
                    }
                    break;

                case A_RCV_S:
                    if (byte == C_expected){
                        frame[i++] = byte;
                        state = C_RCV_S;
                    }
                    else if (byte == FLAG){
                        state = FLAG_RCV_S;
                        i = 1;
                    }
                    else{
                        state = START_S;
                        i = 0;
                    }
                    break;

                case C_RCV_S:{
                    unsigned char bcc1_calculated = frame[1] ^ frame[2];
                    if (byte == bcc1_calculated) {
                        frame[i++] = byte;
                        state = BCC1_RCV_S;
                    }
                    else if (byte == FLAG){ 
                        state = FLAG_RCV_S;
                        i = 1;
                    }
                    else {
                        printf("Rx: BCC1 incorreto (esperado: 0x%02X, recebido: 0x%02X). Reiniciando.\n", bcc1_calculated, byte);
                        state = START_S;
                        i = 0;
                    }
                    break;
                }

                case BCC1_RCV_S:
                    if (byte == FLAG) {
                        frame[i++] = byte;
                        state = STOP_S;
                    }
                    else {
                        state = START_S;
                        i = 0;
                    }
                    break;

                case DATA_RCV_S:
                case STOP_S:
                    break;
            }
        }
    }
    return 1;
}

// Transmitter: Envia SET e Espera UA
int llopen_transmitter(LinkLayer parameters){
    int fd = openSerialPort(parameters.serialPort, parameters.baudRate);
    if (fd < 0){
        return -1;
    }

    currentLinkLayer = parameters;

    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, NULL) == -1) {
        perror("Error setting up sigaction for SIGALRM");
        return -1;
    }

    unsigned char rx_frame[5];
    int success = 0;

    for (alarmCount = 0; alarmCount < parameters.nRetransmissions && !success;){
        printf("TX: Tentativa %d/%d - Enviando SET...\n", alarmCount + 1, parameters.nRetransmissions);

        sendControlFrame(A_TRX, C_SET);
        alarmEnabled = TRUE;
        alarm(parameters.timeout); 

        while (alarmEnabled && !success) {
            if (receiveControlFrame(rx_frame, A_RCV, C_UA)) {
                printf("TX: UA recebido. Ligação estabelecida.\n");
                success = 1;
                alarm(0);
            }
        }

        if (!success){
            printf("TX: Timeout. Retransmitindo...\n");
        }
    }

    if (!success){
        printf("TX: Falha após %d tentativas.\n", parameters.nRetransmissions);
        closeSerialPort();
        return -1;
    }

    return fd;
}

// Receiver: Espera SET, Envia UA
int llopen_receiver(LinkLayer parameters){
    int fd = openSerialPort(parameters.serialPort, parameters.baudRate);
    if (fd < 0)
        return -1;
    
    currentLinkLayer = parameters;
    Nr_expected = 0;

    unsigned char rx_frame[5];
    printf("RX: À espera da Frame SET...\n");

    while (1){
        if (receiveControlFrame(rx_frame, A_TRX, C_SET)){
            printf("RX: Frame SET recebida.\n");
            break;
        }
    }

    if (sendControlFrame(A_RCV, C_UA) < 0){
        printf("RX: Erro ao enviar UA.\n");
        closeSerialPort();
        return -1;
    }

    printf("RX: Ligação estabelecida (UA enviado).\n");
    return fd;
}

int llopen(LinkLayer parameters){
    if (parameters.role == LlTx)
        return llopen_transmitter(parameters);
    else
        return llopen_receiver(parameters);
}

// Byte Stuffing / Destuffing
static int byteStuffing(const unsigned char *source, int sourceSize, unsigned char *dest){
    int destIndex = 0;
    for (int i = 0; i < sourceSize; i++){
        if (source[i] == FLAG){
            dest[destIndex++] = ESCAPE;
            dest[destIndex++] = 0x5E;
        }
        else if (source[i] == ESCAPE){
            dest[destIndex++] = ESCAPE;
            dest[destIndex++] = 0x5D;
        }
        else{
            dest[destIndex++] = source[i];
        }
    }
    return destIndex;
}

static int byteDestuffing(const unsigned char *source, int sourceSize, unsigned char *dest){
    int destIndex = 0;
    for (int i = 0; i < sourceSize; i++){
        if (source[i] == ESCAPE){
            i++;
            if (i < sourceSize){
                if (source[i] == 0x5E) dest[destIndex++] = FLAG;
                else if (source[i] == 0x5D) dest[destIndex++] = ESCAPE;
                else return -1;
            }
            else return -1;
        }
        else dest[destIndex++] = source[i];
    }
    return destIndex;
}

static unsigned char calculateBCC2(const unsigned char *buf, int bufSize){
    unsigned char bcc2 = 0x00;
    for (int i = 0; i < bufSize; i++)
        bcc2 ^= buf[i];
    return bcc2;
}

// Receber ACK/REJ Frame
static int receiveAckFrame(int Ns_sent){
    unsigned char C_expected_RR = (Ns_sent == 0) ? C_RR_1 : C_RR_0;
    unsigned char C_expected_REJ = (Ns_sent == 0) ? C_REJ_1 : C_REJ_0;
    unsigned char rx_frame[5];

    while(alarmEnabled){
        enum State state = START_S;
        unsigned char byte;
        int res;
        int i = 0;
        unsigned char A_expected = A_RCV;

        while (state != STOP_S){
            res = readByteSerialPort(&byte);
            if (!alarmEnabled && res == 0) return -2;
            if (res == -1) return -1;

            if (res == 1){
                switch(state){
                    case START_S:
                        if (byte == FLAG){ 
                            state = FLAG_RCV_S; 
                            i = 1; 
                            rx_frame[0] = byte;
                        }
                        break;
                    case FLAG_RCV_S:
                        if (byte == A_expected){ 
                            state = A_RCV_S;
                            i = 2; 
                            rx_frame[1] = byte;
                        }
                        else if (byte == FLAG){
                            i = 1;
                        }
                        else { state = START_S; 
                            i = 0;
                        }
                        break;
                    case A_RCV_S:
                        if (byte == C_expected_RR || byte == C_expected_REJ){ 
                            state = C_RCV_S; i = 3; rx_frame[2] = byte;
                        }
                        else if (byte == FLAG){ 
                            state = FLAG_RCV_S; i = 1; 
                        }
                        else { 
                            state = START_S; i = 0; 
                        }
                        break;
                    case C_RCV_S:{
                        unsigned char bcc1_calculated = rx_frame[1] ^ rx_frame[2];
                        if (byte == bcc1_calculated){ 
                            state = BCC1_RCV_S; i = 4; rx_frame[3] = byte; 
                        }
                        else if (byte == FLAG){ 
                            state = FLAG_RCV_S; i = 1;
                        }
                        else { 
                            state = START_S; i = 0; 
                        }
                        break;
                    }
                    case BCC1_RCV_S:
                        if (byte == FLAG){ 
                            state = STOP_S; 
                            rx_frame[4] = byte;
                        }
                        else { 
                            state = START_S; i = 0;
                        }
                        break;
                    case DATA_RCV_S:
                    case STOP_S:
                        break;
                }
            }
        }

        if (state == STOP_S){
            unsigned char received_C = rx_frame[2];
            alarm(0);
            if (received_C == C_expected_RR) return 1;
            else if (received_C == C_expected_REJ) return 0;
            return -1;
        }
    }
    return -2;
}

// ----------------------------------------------------
// LLWRITE
// ----------------------------------------------------
int llwrite(const unsigned char *buf, int bufSize){
    unsigned char C_I = (Ns == 0) ? C_I_0 : C_I_1;
    unsigned char bcc2;
    int bytes_sent = 0;

    unsigned char frame_data[MAX_PAYLOAD_SIZE + 1];
    unsigned char stuffed_frame_data[MAX_PAYLOAD_SIZE * 2 + 2]; 

    bcc2 = calculateBCC2(buf, bufSize);
    memcpy(frame_data, buf, bufSize);
    frame_data[bufSize] = bcc2;

    int stuffed_data_size = byteStuffing(frame_data, bufSize+1, stuffed_frame_data);

    unsigned char frame_header[4];
    frame_header[0] = FLAG;
    frame_header[1] = A_TRX;
    frame_header[2] = C_I;
    frame_header[3] = A_TRX ^ C_I; 

    int full_frame_size = 4 + stuffed_data_size + 1;
    unsigned char *full_frame = (unsigned char *)malloc(full_frame_size);

    memcpy(full_frame, frame_header, 4);
    memcpy(full_frame + 4, stuffed_frame_data, stuffed_data_size);
    full_frame[full_frame_size-1] = FLAG;

    int success = 0;

    for (alarmCount=0; alarmCount<currentLinkLayer.nRetransmissions && !success;){
        printf("TX LLWRITE (Ns=%d): Tentativa %d/%d - Enviando trama I (tamanho dados: %d)...\n", 
               Ns, alarmCount+1, currentLinkLayer.nRetransmissions, bufSize);

        bytes_sent = writeBytesSerialPort(full_frame, full_frame_size);
        if (bytes_sent != full_frame_size){
            printf("TX LLWRITE: Erro ao escrever frame completa. Abortando.\n");
            free(full_frame);
            return -1;
        }

        alarmEnabled = TRUE;
        alarm(currentLinkLayer.timeout); 

        int ack_status = receiveAckFrame(Ns);
        if (ack_status == 1){ 
            printf("TX LLWRITE (Ns=%d): RR recebido. ACK OK.\n", Ns);
            success = 1;
            alarm(0);
        }
        else if (ack_status == 0){ 
            printf("TX LLWRITE (Ns=%d): REJ recebido. Retransmitindo...\n", Ns);
            alarmCount++;
            alarm(0);
        }
        else if (ack_status == -2){
            printf("TX LLWRITE (Ns=%d): Timeout. Retransmitindo...\n", Ns);
        }
        else{
            printf("TX LLWRITE (Ns=%d): Frame de ACK inválida. Retransmitindo...\n", Ns);
            alarmCount++;
            alarm(0);
        }
    }

    free(full_frame);
    if (success){
        Ns = 1 - Ns;
        return bufSize;
    }
    else{
        printf("TX LLWRITE: Falha na transmissão após %d tentativas (Ns=%d).\n", currentLinkLayer.nRetransmissions, Ns);
        return -1;
    }
}

// ----------------------------------------------------
// LLREAD
// ----------------------------------------------------
int llread(unsigned char *packet){
    while(1){
        unsigned char C_expected = (Nr_expected==0)?C_I_0:C_I_1;
        unsigned char C_other = (Nr_expected==0)?C_I_1:C_I_0;
        unsigned char C_RR_to_send = (Nr_expected==0)?C_RR_1:C_RR_0;
        unsigned char C_REJ_to_send = (Nr_expected==0)?C_REJ_0:C_REJ_1;

        unsigned char frame[MAX_frame_SIZE];
        unsigned char unstuffed_data[MAX_PAYLOAD_SIZE+1];
        int frameIDX=0;
        int data_index=0;

        enum State state=START_S;
        unsigned char byte;
        int res;

        printf("RX LLREAD: À espera da Frame I (Ns=%d)...\n", Nr_expected);

        while(state != STOP_S){
            res = readByteSerialPort(&byte);
            if (res==-1){ perror("llread: Erro de leitura"); return -1;}
            if (res==1){
                if (frameIDX >= MAX_frame_SIZE){
                    printf("RX LLREAD: Buffer excedido. Reiniciando...\n");
                    state=START_S; frameIDX=0; continue;
                }

                switch(state){
                    case START_S: frameIDX=0;
                        if (byte==FLAG){
                            frame[frameIDX++]=byte;
                            state=FLAG_RCV_S;
                        }
                        break;
                    case FLAG_RCV_S:
                        if (byte==A_TRX){ 
                            frame[frameIDX++]=byte; 
                            state=A_RCV_S;
                        }
                        else if (byte==FLAG){
                            frameIDX=1;
                        }
                        else state=START_S;
                        break;
                    case A_RCV_S:
                        if (byte==C_expected || byte==C_other){
                             frame[frameIDX++]=byte;
                             state=C_RCV_S;
                        }
                        else if (byte==C_DISC){ 
                            sendSupervisionFrame(C_DISC);
                             return 0;
                            }
                        else if (byte==FLAG){ 
                            state=FLAG_RCV_S;
                             frameIDX=1;
                            }
                        else state=START_S;
                        break;
                    case C_RCV_S:{
                        unsigned char bcc1_calc = frame[1]^frame[2];
                        if (byte==bcc1_calc){ 
                            frame[frameIDX++]=byte; 
                            data_index=frameIDX; 
                            state=DATA_RCV_S;
                        }
                        else if (byte==FLAG){
                            state=FLAG_RCV_S; 
                            frameIDX=1;
                        }
                        else state=START_S;
                        break;
                    }
                    case DATA_RCV_S: if (byte==FLAG){
                        state=STOP_S;
                    }
                     else{ 
                        frame[frameIDX++]=byte;
                    }
                    break;
                    case BCC1_RCV_S:
                    case STOP_S: break;
                }
            }
        }

        int Ns_received = (frame[2]==C_I_1);
        if (Ns_received != Nr_expected){
            printf("RX LLREAD: Frame duplicada/inesperada (Ns=%d, Esperado=%d). Enviando RR(%d).\n", Ns_received, Nr_expected, Nr_expected);
            unsigned char C_RR_repeat = (Nr_expected==0)?C_RR_0:C_RR_1;
            sendSupervisionFrame(C_RR_repeat);
            return 0;
        }

        int stuffed_data_size = frameIDX - data_index;
        int unstuffed_size = byteDestuffing(frame+data_index, stuffed_data_size, unstuffed_data);

        if (unstuffed_size < 1){
            printf("RX LLREAD: Erro de Destuffing. Enviando REJ(%d).\n", Ns_received);
            sendSupervisionFrame(C_REJ_to_send);
            continue;
        }

        unsigned char bcc2_received = unstuffed_data[unstuffed_size-1];
        int payload_size = unstuffed_size-1;
        unsigned char bcc2_calc = calculateBCC2(unstuffed_data, payload_size);

        if (bcc2_calc != bcc2_received){
            printf("RX LLREAD: Erro BCC2 (Calc=0x%02X, Recebido=0x%02X). Enviando REJ(%d).\n", bcc2_calc, bcc2_received, Ns_received);
            sendSupervisionFrame(C_REJ_to_send);
            continue;
        }

        memcpy(packet, unstuffed_data, payload_size);
        printf("RX LLREAD: Frame I(Ns=%d) válida. Enviando RR(%d).\n", Ns_received, 1-Nr_expected);
        sendSupervisionFrame(C_RR_to_send);
        Nr_expected = 1 - Nr_expected;
        return payload_size;
    }
}

// ----------------------------------------------------
// LLCLOSE
// ----------------------------------------------------
int llclose(){
    printf("\n--- Fechando ligação (llclose) ---\n");

    int success=0;
    unsigned char rx_frame[5];

    if (currentLinkLayer.role==LlTx){
        struct sigaction sa;
        sa.sa_handler = alarmHandler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        if (sigaction(SIGALRM, &sa, NULL)==-1){ perror("sigaction"); return -1;}

        for (alarmCount=0; alarmCount<currentLinkLayer.nRetransmissions && !success;){
            printf("TX LLCLOSE: Tentativa %d/%d - Enviando DISC...\n", alarmCount+1, currentLinkLayer.nRetransmissions);

            sendControlFrame(A_TRX,C_DISC);
            alarmEnabled=TRUE; alarm(currentLinkLayer.timeout);

            while(alarmEnabled && !success){
                if (receiveControlFrame(rx_frame, A_RCV, C_DISC)){
                    printf("TX LLCLOSE: DISC recebido. Enviando UA...\n");
                    sendControlFrame(A_TRX, C_UA);
                    success=1; alarm(0);
                }
            }

            if (!success) printf("TX LLCLOSE: Timeout. Retransmitindo DISC...\n");
        }

        if (!success) printf("TX LLCLOSE: Falha no handshake de fecho.\n");
        else printf("TX LLCLOSE: Handshake concluído.\n");
    }
    else{ // LlRx
        printf("RX LLCLOSE: À espera de DISC...\n");

        if (!receiveControlFrame(rx_frame, A_TRX, C_DISC)) printf("RX LLCLOSE: Falha ao receber DISC.\n");
        else{
            printf("RX LLCLOSE: DISC recebido. Enviando DISC...\n");
            sendControlFrame(A_RCV,C_DISC);

            struct sigaction sa;
            sa.sa_handler=alarmHandler; sigemptyset(&sa.sa_mask); sa.sa_flags=0;
            if (sigaction(SIGALRM,&sa,NULL)==-1){ perror("sigaction"); return -1;}

            alarmCount=0; success=0;

            for (alarmCount=0; alarmCount<currentLinkLayer.nRetransmissions && !success;){
                printf("RX LLCLOSE: Tentativa %d/%d - À espera de UA...\n", alarmCount+1, currentLinkLayer.nRetransmissions);

                alarmEnabled=TRUE; alarm(currentLinkLayer.timeout);

                while(alarmEnabled && !success){
                    if (receiveControlFrame(rx_frame,A_TRX,C_UA)){
                        printf("RX LLCLOSE: UA recebido. Handshake concluído.\n");
                        success=1; alarm(0);
                    }
                }

                if (!success){
                    printf("RX LLCLOSE: Timeout. Retransmitindo DISC...\n");
                    sendControlFrame(A_RCV,C_DISC);
                }
            }

            if (!success) printf("RX LLCLOSE: Falha no handshake de fecho.\n");
            else printf("RX LLCLOSE: Handshake concluído.\n");
        }
    }

    if (closeSerialPort()==0){ printf("Porta série fechada com sucesso.\n"); return success?0:-2;}
    else{ printf("Erro ao fechar porta série.\n"); return -1;}
}
