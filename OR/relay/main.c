/*******************************************************************************
 *
 * Copyright (c) 2018 Dragino
 *
 * http://www.dragino.com
 *
 *******************************************************************************/

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>

#include <sys/ioctl.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
// mac address取得のためにソケットを開く
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <unistd.h>

#include <sys/time.h>
#include <math.h>
// #############################################
// #############################################

#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common

#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegPaDac                                   0x5A // common

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// sx1276 RegModemConfig1
#define SX1276_MC1_BW_125                0x70
#define SX1276_MC1_BW_250                0x80
#define SX1276_MC1_BW_500                0x90
#define SX1276_MC1_CR_4_5            0x02
#define SX1276_MC1_CR_4_6            0x04
#define SX1276_MC1_CR_4_7            0x06
#define SX1276_MC1_CR_4_8            0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01

// sx1276 RegModemConfig2
#define SX1276_MC2_RX_PAYLOAD_CRCON        0x04

// sx1276 RegModemConfig3
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                  0x34

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef LMIC_SX1276
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif LMIC_SX1272
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66

// ----------------------------------------
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD       0x07

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

//----------------------------------------------
//ハローかパケットかの判別
#define HELLO 0b10000000 //128
#define DATA   0b00001000 //8
#define ACK   0b10001000

#define USER_DATA_FRAME_LEN 1500
#define USER_CTRL_FRAME_LEN 100
#define MAX_PAYLOAD_LEN 255

// #############################################
// #############################################

typedef bool boolean;
typedef unsigned char byte;

static const int CHANNEL = 0;

char message[256];

bool sx1272 = true;

byte receivedbytes;

// enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };
// SF6を追加　
// implicitモードをオンにしなければいけない
// DetectionOptimizeのあたいを0x05にする
//
enum sf_t { SF6=6, SF7, SF8, SF9, SF10, SF11, SF12 };
/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1272 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;

// Set spreading factor (SF7 - SF12)
sf_t sf = SF7;

// Set center frequency
uint32_t  freq = 920000000; // in Mhz! (868.1)  868.1->920.0

// byte hello[32] = "world";
int count = 0;
// 時間計測の構造体
clock_t sender,receiver;
// time_t now,later;
struct timespec now,later;

// loraのフレームヘッダの構造体
typedef struct{
    uint8_t type;  // パケット識別
    uint8_t SourceAddr[6]={};
    uint8_t DestAddr[6]={};
    uint8_t SenderAddr[6]={};
    uint8_t len;
    uint16_t seqNum;
    uint16_t checksum;
    uint8_t payload[];
}__attribute__ ((packed)) mac_frame_header_t;

typedef struct{
    uint8_t Hop;
    uint8_t DestAddr[6];
    uint16_t seqNum;
}__attribute__((packed))mac_frame_payload_t;

typedef struct{
    uint8_t srcHop;  // 現在のホップ数
    uint8_t destHop;
    double u;  // 乱数値
    double parameter;  // 待機時間パラメータ
    uint8_t message[]; 
}__attribute__((packed))or_data_packet_t;

// ルーティングテーブル
typedef struct routing_table_entry_struct{
    uint8_t hop;
    uint16_t seq;
    uint8_t addr[6];
    double minLrtt;
    double maxLrtt;
    double Lrtt;
    double smoothedU;
    double smoothedPassiveRandU;
    double smoothedPassiveRandDiff;
    double lastPassiveRandU;
    uint16_t lastSeqNum;
    double dupReceivedRate;
    int firstReceived;
    routing_table_entry_struct* next;
}routing_table_entry_t;

typedef struct{
    routing_table_entry_t* head;
    uint8_t size;
}routing_table_t;

// バックオフテーブル
typedef struct backoff_entry_struct{
    uint8_t srcAddr[6];
    uint16_t seqNum;
    double backoff;
    struct timespec backoff_now;
    backoff_entry_struct* next;
}backoff_entry_t;

typedef struct{
    backoff_entry_t* head;
    uint8_t size;
}backoff_t;

// パケットテーブル
typedef struct packet_table_entry_struct{
    uint8_t srcAddr[6];
    uint16_t seq;
    uint8_t flag;
	bool transmitted;
	uint8_t receiveCount;
	struct timespec txtime;
    packet_table_entry_struct* next;
    mac_frame_header_t* packet;
}packet_table_entry_t;

typedef struct{
    packet_table_entry_t* head;
    uint8_t size;
}packet_table_t;


/*
 * 固定長の配列を宣言しないとバイナリがバグる
 * uint8_tかunsigned charの１バイトで宣言
 * 255はLoRaの最大ペイロードサイズ
 */
uint8_t Hello[255] = {};
uint8_t data[255] = {};
uint8_t Ack[255] = {};

mac_frame_header_t* data_packet = (mac_frame_header_t*)&data;
mac_frame_header_t *Hello_p = (mac_frame_header_t*)&Hello;
mac_frame_header_t* Ack_p = (mac_frame_header_t*)&Ack;


routing_table_t* r_table = NULL;
backoff_t* bo_st = NULL;
packet_table_t* p_table = NULL;

// file open
FILE *fp_rx,*fp_tx,*fp_hop;
char* rx_filename;
char* tx_filename;
char *hop_filename;

// 自分のMACアドレス
uint8_t my_addr[6]={};

// チェックサム用
uint16_t prev_checksum;
uint16_t current_checksum;

// 現在時刻の取得
struct timespec ts_tx,ts_rx;
struct tm tm_tx,tm_rx;
struct timespec after_backoff;

// 転送待機時間算出の乱数値・パラメータ
double smoothedPassiveRandU = 0.0;
double smoothedPassiveRandDiff = 0.0;
// double dupReceivedRate = 0.0;
// double lastPassiveRandU = 0.0;
double parameter = 1000;  // 待機時間パラメータ（ミリ秒）
// uint16_t lastSeqNum;
// int firstReceived = 0; // 0 ･･･ FALSE  1 ･･･ TRUE booleanに直せるなら直す
// int receiveCount = 0;
struct timespec backoff_now;
uint8_t source_addr[6] = {0xb8, 0x27, 0xeb, 0xf6, 0x49, 0xa0}; //べた書きでmacアドレス


/*
 * ノードの固有値
 * LoRaの物理層から取得はできないため、有線LANのものを取得して使う
 */
// MACアドレスの取得_関数
void get_my_mac_addr(){

    int fd;
    struct ifreq ifr;  // <net.if.h>に定義

    // IPv4(AF_INIT),UDP(SOCK_DGRAM),プロトコル(0)
	// 新たに作成されたソケットを参照するディスクリプタを返す
    fd = socket(AF_INET,SOCK_DGRAM,0);

    // sockaddr構造体内(<sys/socket.h>に定義)にあるsa_familyにAF_INET(PF_INIT)を格納
	// AF_INETはIPv4でUDPを扱うのに必要らしい
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name,"eth0",IFNAMSIZ-1);

    // <sys/ioctl.h>に定義
	// ファイルディスクリプタ(fd),MACアドレス取得(SIOCGIFHWADDR)(sockaddr構造体に格納),パラメータ(&ifr)
    ioctl(fd,SIOCGIFHWADDR,&ifr);

    // my_addrに自身のMACアドレスを格納
    for(int i=0;i<6;i++){
	    my_addr[i] = (unsigned char)ifr.ifr_hwaddr.sa_data[i];
    }
    close(fd);
}


// MACアドレスを画面表示_関数
void mac_print_addr(uint8_t* addr){
	printf("mac addr:%02x:%02x:%02x:%02x:%02x:%02x\n",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
}


// MACアドレスを格納_関数(target -> original)(左を右に)
void mac_set_addr(uint8_t*original,uint8_t* target){
    for(int i = 0; i < 6; i++) target[i]=original[i];
}


// MACアドレスに16進数FFを格納_関数(初期化(init関数)に使用)
void mac_set_bc_addr(uint8_t* target){
    for(int i = 0; i < 6; i++) target[i] = 0xff;    
}


/*
 * frame header init function
 */
// HELLOパケット(MACフレームヘッダ)初期化_関数
void mac_tx_hello_frame_header_init(mac_frame_header_t* hdr){

    hdr->type = HELLO;

    mac_set_bc_addr(hdr->SourceAddr);
    mac_set_bc_addr(hdr->DestAddr);
    mac_set_bc_addr(hdr->SenderAddr);

    hdr->len = 0;  // hdr + payload
    hdr->seqNum = 0;
    hdr->checksum = 0;
}


// ACKパケット(MACフレームヘッダ)初期化_関数
void mac_tx_ack_frame_header_init(mac_frame_header_t* hdr){

    hdr->type = ACK;

    mac_set_bc_addr(hdr->SourceAddr);
    mac_set_bc_addr(hdr->DestAddr);

    hdr->len = 0;  // hdr + payload
    hdr->seqNum = 0;
    hdr->checksum = 0;
}


// データパケット(MACフレームヘッダ)初期化_関数
void mac_tx_data_frame_header_init(mac_frame_header_t* hdr){

    hdr->type = DATA;

    mac_set_bc_addr(hdr->SourceAddr);
    mac_set_bc_addr(hdr->DestAddr);

    hdr->len = 0;
    hdr->seqNum = 0;
    hdr->checksum = 0;
}


// HELLOパケット(MACフレームペイロード)初期化_関数
void mac_tx_hello_frame_payload_init(mac_frame_header_t* hdr){

    mac_frame_payload_t* payload = (mac_frame_payload_t*)hdr->payload;

    payload->Hop= 0;
    mac_set_bc_addr(payload->DestAddr);
    payload->seqNum = 0;
}


// データパケット(MACフレームペイロード)初期化_関数
void mac_tx_data_frame_payload_init(mac_frame_header_t* frame){

    or_data_packet_t* data_frame = (or_data_packet_t*)frame->payload;

    data_frame->srcHop = 0;
    data_frame->destHop = 0;
    data_frame->u = 0.0;
    data_frame->parameter = 0.0;
    
    // 255 - MACフレームヘッダ - ORデータパケット
    for(unsigned int i = 0;i < 255 - sizeof(mac_frame_header_t) - sizeof(or_data_packet_t);i++){
        data_frame->message[i] = 0x11;
    }
}


// MACフレームヘッダの画面表示_関数
void print_mac_frame_header(mac_frame_header_t* data){
    printf("mac packet header   : \n");

    if(data->type == HELLO) printf("type                : HELLO\n");
    else if(data->type == DATA) printf("type                : DATA\n");
    else if(data->type == ACK) printf("type                : ACK\n");
    
    printf("Source      ");
    mac_print_addr(data->SourceAddr);

    printf("Destination ");
    mac_print_addr(data->DestAddr);

    printf("length              : %u\n",data->len);
    printf("sequence number     : %u\n",data->seqNum);
    printf("payload             : %s\n",data->payload);
    printf("\n");
}


// MACフレームペイロードの画面表示_関数
void print_mac_frame_payload(mac_frame_header_t* data){

    mac_frame_payload_t* packet = (mac_frame_payload_t*)data->payload;
    
    printf("mac frame payload: \n");
    printf("Hop: %u\n",packet->Hop);

    printf("Destination ");
    mac_print_addr(packet->DestAddr);

    printf("Sequence number: %u\n",packet->seqNum);
    printf("\n");
}


// アドレスの大小比較_関数(左<右 true)
boolean is_smaller_addr(uint8_t* origin,uint8_t* target){
    for(int i = 0 ;i<6;i++){
        if(origin[i]<target[i]){  // addr1 = origin 
            return true;
        }else if(origin[i]>target[i]){
            return false;
        }
    }
    return false;
}


// アドレスの等号比較_関数(左=右 true)
boolean is_same_addr(uint8_t* origin,uint8_t* target){
    for(int i =0;i<6;i++){
        if(origin[i]!=target[i]) return false;
    }
    return true;
}


// ルーティングテーブルから削除_関数
void delete_routing_table(uint8_t* addr){

    routing_table_entry_t* toFree = NULL;
    routing_table_entry_t* current = r_table->head;  // ルーティングテーブルの先頭から
    
    if(!current){
        // ルーティングテーブルに何も無い場合は何もしない
    
    // ルーティングテーブルの先頭を削除(currentはr_table->headである)
    }else if(is_same_addr(current->addr,addr)){
        toFree = r_table->head;
        current->next = toFree->next;
        free(toFree);
        --r_table->size;
    // ルーティングテーブルの移動(currentからcurrent->nextへ)    
    }else{
        while(current->next && is_smaller_addr(current->addr,addr)){
            current = current->next;
        }
    }
    // ルーティングテーブルから削除(current->nextが対象)
    if(current->next && is_same_addr(current->next->addr,addr)){
        toFree = current->next;
        current->next = toFree->next;
        free(toFree);
        --r_table->size;
    }
}


// ルーティングテーブルに追加_関数
void insert_routing_table(uint8_t* addr,uint8_t hop,uint16_t seq){

    routing_table_entry_t* current = r_table->head;  // ルーティングテーブルの先頭から
    routing_table_entry_t* previous = NULL;
    
    // ルーティングテーブルの順序変更(昇順)
    while(current && is_smaller_addr(current->addr,addr)){ 
        previous = current;
        current = current->next;
    }

    // ルーティングテーブルへの最初の挿入(0だから仕事していない)
    if(0){
        printf("new entry\n");
        routing_table_entry_t* new_entry = (routing_table_entry_t* )malloc(sizeof(routing_table_entry_t));
        memset(new_entry, 0, sizeof(routing_table_entry_t));
        mac_set_addr(addr, new_entry->addr);

        new_entry->hop = hop;
        new_entry->seq = seq;
        new_entry->next = NULL;
        (r_table)->size++;
        r_table->head = new_entry;
    
    // 自身のアドレスと比較
    }else if(is_same_addr(my_addr,addr)){
        printf("自身なのでルーティングテーブルにいれません.\n");
    // 自身のアドレスと比較
    }else if(current && is_same_addr(my_addr,addr)){
        printf("自身なのでルーティングテーブルにいれません.\n");
    // currentのアドレスと比較
    }else if(current && is_same_addr(current->addr,addr)){
        // currentのシーケンス番号より大きい場合
    	// 更新
        if(current->seq < seq){
            current->hop = hop;
            current->seq = seq;
            mac_set_addr(addr,current->addr);
        // currentのシーケンス番号と等しく，ホップが小さい場合
    	// 更新
        }else if(current->seq == seq && current->hop > hop){
            current->hop = hop;
            current->seq = seq;
            mac_set_addr(addr,current->addr);
        }
    // current内に無い場合
	// 挿入(routing_table_entry_t* new_entry)
    }else{
        routing_table_entry_t* new_entry = (routing_table_entry_t* )malloc(sizeof(routing_table_entry_t));
        memset(new_entry, 0, sizeof(routing_table_entry_t));
        mac_set_addr(addr, new_entry->addr);

        new_entry->hop = hop;
        new_entry->seq = seq;
        new_entry->next = NULL;
        (r_table)->size++;
        
        // 先頭に挿入(headの変更)
        if(!previous){
            new_entry->next = r_table->head;
            r_table->head = new_entry;
        // 途中に挿入          
        }else{
            new_entry->next = previous->next;
            previous->next = new_entry;
        }
    }

    return ;
} 


// ルーティングテーブルを取得_関数
int get_routing_table(mac_frame_header_t* hello){

    mac_frame_payload_t* current = (mac_frame_payload_t*) hello->payload;
    routing_table_entry_t* current_entry = r_table->head;

    int i = 0;

    while(current_entry){
        current->Hop = current_entry->hop;
        current->seqNum  = current_entry->seq;
        
        mac_set_addr(current_entry->addr,current->DestAddr);
        ++current;
        i++;
        current_entry = current_entry->next;
    }

    printf("\n");

    return i;
}


// csvへの出力_関数(受信パケット(1番目))(time , time , nsec)
void output_data_csv_rx_time(char* filename){

    char time[50];
    char add_time[100];

    if((fp_rx = fopen(filename,"a+")) == NULL){
        printf("can't open file\n");
        exit(1);
    }

    sprintf(time,"%d/%02d/%02d %02d:%02d:%02d",tm_rx.tm_year+1900,tm_rx.tm_mon+1,tm_rx.tm_mday,tm_rx.tm_hour,tm_rx.tm_min,tm_rx.tm_sec);
    sprintf(add_time,"%02d:%02d:%02d",tm_rx.tm_hour,tm_rx.tm_min,tm_rx.tm_sec);

    fprintf(fp_rx,"%s,%s,%09ld,",time,add_time,ts_rx.tv_nsec);

    fclose(fp_rx);
}


// csvへの出力_関数(送信パケット(2番目))(tx_time , time , nsec)
void output_data_csv_tx_time(char* filename){

    char time[50];
    char add_time[100];

    if((fp_tx = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }

    sprintf(time,"%d/%02d/%02d %02d:%02d:%02d",tm_tx.tm_year+1900,tm_tx.tm_mon+1,tm_tx.tm_mday,tm_tx.tm_hour,tm_tx.tm_min,tm_tx.tm_sec);
    sprintf(add_time,"%02d:%02d:%02d",tm_tx.tm_hour,tm_tx.tm_min,tm_tx.tm_sec);

    fprintf(fp_tx,"%s,%s,%09ld,,",time,add_time,ts_tx.tv_nsec);

    fclose(fp_tx);
}


// csvへの出力_関数(ルーティングテーブル(3番目))(srcAddress , hop , seq)
void output_data_csv(FILE *fp,routing_table_entry_t* head, char* filename){
    
    char address[20];
    
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    sprintf(address,"%02x:%02x:%02x:%02x:%02x:%02x",head->addr[0],head->addr[1],head->addr[2],head->addr[3],head->addr[4],head->addr[5]);
    
    fprintf(fp,"%s,%u,%u,",address,head->hop,head->seq);
    
    fclose(fp);
}


// csvへの出力_関数(改行(全て))
void output_data_csv_space(FILE* fp,char* filename){

    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }

    fprintf(fp,"\n");

    fclose(fp);
}


// ルーティングテーブルを画面表示_関数
void print_routing_table(routing_table_t* routing_t){

    routing_table_entry_t* current =routing_t->head;

    int i =1;  // ノード数の表示用

    printf("----------routing table----------\n");
    
    // ルーティングテーブルが無い場合
    if(!current){
        printf("data nothing\n");
        return ;
    }

    // csv出力(3)
    output_data_csv_rx_time(hop_filename);

    // ルーティングテーブルがある場合
    while(current){
        printf("Node%d ",i);

        mac_print_addr(current->addr);

        printf("Hop: %u\n",current->hop);
        printf("Sequence Number: %u\n",current->seq);
        printf("table size: %d\n",routing_t->size);

        i++;

        // csv出力(3)
        output_data_csv(fp_hop,current,hop_filename);
        
        current = current->next;
    }

    // csv出力(改行)
    output_data_csv_space(fp_hop,hop_filename);

    printf("---------------end----------------\n");
}


// データフレームの表示_関数
void print_data_frame(mac_frame_header_t* packet){

    or_data_packet_t* data_frame = (or_data_packet_t*)packet->payload;

    printf("data: \n");
    printf("src hop           : %u\n",data_frame->srcHop);
    printf("dest hop          : %u\n",data_frame->destHop);
}


// ルーティングテーブルへ追加_関数(宛先ノードを追加)
void add_routing_table(mac_frame_header_t* hdr){

    int len = ((hdr->len) - sizeof(mac_frame_header_t) )/sizeof(mac_frame_payload_t);
    
    printf("insert routing table length: %d\n",len);

    mac_frame_payload_t* entry = (mac_frame_payload_t*)hdr->payload;
    
    if(len == 0){ 
        return;
    }else{
        for(int i = 0;i < len;i++){
            insert_routing_table(entry->DestAddr,entry->Hop+1,entry->seqNum);
            mac_print_addr(entry->DestAddr);
            ++entry;
        }
    }
}


// バックオフの大小比較_関数(左<右 true)
boolean is_smaller_backoff(double origin,double target){
    if(origin<target)  return true;
    else if(origin>target) return false;
    return false;
}


// バックオフの等号比較_関数(左=右 true)
boolean is_same_seq(uint16_t origin,uint16_t target){
    if(origin!=target) return false;
    return true;
}


// 使用していない画面表示_関数
void print_packet_table(){

    packet_table_entry_t* current = p_table->head;

    int i =1;

    printf("----------packet table----------\n");
    
    if(!current){
        printf("data nothing\n");
        return ;
    }

    while(current){
        printf("Node%d ",i);

        mac_print_addr(current->srcAddr);

        printf("Sequence Number: %u\n",current->seq);
        printf("flag           : %u\n",current->flag);
        printf("table size     : %d\n",p_table->size);

        i++;
        current = current->next;
    }
    printf("---------------end--------------\n");
}

// バックオフの画面表示_関数
void print_backoff_table(){

    backoff_entry_t* current = bo_st->head;

    int i = 1;

    printf("----------backoff table----------\n");
    
    if(!current){
        printf("data nothing\n");
        return;
    }

    while(current){
        printf("Node%d\n",i);
        printf("src ");

        mac_print_addr(current->srcAddr);

        printf("sequence number: %u\n",current->seqNum);
        printf("backoff time   : %lf\n",current->backoff);
        printf("table size     : %d\n",bo_st->size);

        i++;
        current = current->next;
    }
    printf("---------------end---------------\n");
}


// パケットタイプ判断_関数(HELLO,DATA,ACK)
char* judge_packet_type(mac_frame_header_t* hdr,char* packet_type){

    // HELLOの場合
    if(hdr->type == HELLO){
        strcpy(packet_type,"HELLO");
        return  packet_type;
    // DATAの場合
    }else if(hdr->type == DATA){
        strcpy(packet_type,"DATA");
        return  packet_type;
    // ACKの場合
    }else if(hdr->type == ACK){
        strcpy(packet_type,"ACK");
        return packet_type;
    }
    return NULL;
}


// csvへの出力_関数(受信パケット(1番目))(type , pattern , srcAddress ,　senderAddress ,  destAddress , length , seq , tx_checksum , rx_checksum)
void output_data_csv_rx_hdr(mac_frame_header_t* hdr,char* filename,int pattern){
    
    char type[20];
    char srcaddr[20];
    char destaddr[20];
    char senderaddr[20];
    
    if((fp_rx = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    
    sprintf(srcaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->SourceAddr[0],hdr->SourceAddr[1],hdr->SourceAddr[2],hdr->SourceAddr[3],hdr->SourceAddr[4],hdr->SourceAddr[5]);
    sprintf(senderaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->SenderAddr[0],hdr->SenderAddr[1],hdr->SenderAddr[2],hdr->SenderAddr[3],hdr->SenderAddr[4],hdr->SenderAddr[5]);
    sprintf(destaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->DestAddr[0],hdr->DestAddr[1],hdr->DestAddr[2],hdr->DestAddr[3],hdr->DestAddr[4],hdr->DestAddr[5]);
    
    fprintf(fp_rx,"%s,%d,%s,%s,%s,%u,%u,%u,%u,,",judge_packet_type(hdr,type),pattern,srcaddr,senderaddr,destaddr,hdr->len,hdr->seqNum,prev_checksum,current_checksum);
    fclose(fp_rx);
}


// csvへの出力_関数(受信・送信パケット(1・2番目))(srcHop , destHop , u , parameter)
void output_data_csv_ordata(FILE *fp,or_data_packet_t* hdr,char* filename){

    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }

    fprintf(fp,"%u,%u,%lf,%lf,,",hdr->srcHop,hdr->destHop,hdr->u,hdr->parameter);
    fclose(fp);
}


// csvへの出力_関数(使用していない)(u , randU)
void output_data_csv_u(FILE *fp,or_data_packet_t* hdr,char* filename,routing_table_entry_t* r_current){
    
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }

    fprintf(fp,"%lf,%lf",hdr->u,r_current->smoothedPassiveRandU);
    fclose(fp);
}


// csvへの出力_関数(送信パケット(2番目))(type , srcAddress , senderAddress , destAddress , length , seq , tx_checksum)
void output_data_csv_tx_hdr(mac_frame_header_t* hdr,char* filename){
    
    char type[20];
    char srcaddr[20];
    char destaddr[20];
    char senderaddr[20];
    
    if((fp_tx = fopen(filename,"a+")) == NULL){
        printf("can't open %s\n",filename);
        exit(1);
    }
    
    sprintf(srcaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->SourceAddr[0],hdr->SourceAddr[1],hdr->SourceAddr[2],hdr->SourceAddr[3],hdr->SourceAddr[4],hdr->SourceAddr[5]);
    sprintf(senderaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->SenderAddr[0],hdr->SenderAddr[1],hdr->SenderAddr[2],hdr->SenderAddr[3],hdr->SenderAddr[4],hdr->SenderAddr[5]);
    sprintf(destaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->DestAddr[0],hdr->DestAddr[1],hdr->DestAddr[2],hdr->DestAddr[3],hdr->DestAddr[4],hdr->DestAddr[5]);
    
    fprintf(fp_tx,"%s,%s,%s,%s,%u,%u,%u,,",judge_packet_type(hdr,type),srcaddr,senderaddr,destaddr,hdr->len,hdr->seqNum,hdr->checksum);
    fclose(fp_tx);
}


// パケットテーブルへ追加_関数
packet_table_entry_t* insert_packet_table(uint8_t *srcAddr,uint16_t seq,mac_frame_header_t* packet){//,uint8_t flag){
    
    packet_table_entry_t* current = p_table->head;
    packet_table_entry_t* prev = NULL;

    // currentのアドレス(送信元アドレス)と比較
    while(current && is_smaller_addr(current->srcAddr,srcAddr)){
        prev = current;
        current = current->next;
    }
    
    // currentのアドレス(送信元アドレス)の比較とcurrentのシーケンス番号の比較
    while(current && is_same_addr(current->srcAddr,srcAddr) && current->seq < seq){
        prev = current;
        current = current->next;
    }

    // currentのシーケンス番号と同じ場合
    if(current && is_same_seq(current->seq,seq)){

        // パターン2：受信済みでACK送信してない場合
        // パターン3：受信済みでACK送信してないけど、ACK送信しない場合
        // パターン4：受信済みでACK送信済みの場合

        routing_table_entry_t* r_current = r_table->head;

        // 宛先ノードのホップ数を調べる
        while(r_current && !is_same_addr(r_current->addr,packet->DestAddr)){
            r_current = r_current->next;
        }
        or_data_packet_t* data_p = (or_data_packet_t*)packet->payload;
        
        // データパケット受信
        if(current->flag == DATA){
            // パターン2：暗黙的ACKを受信
            if(r_current->hop >= data_p->destHop){
                current->flag = ACK;
                output_data_csv_rx_hdr(packet,rx_filename,2);
                output_data_csv_ordata(fp_rx,data_p,rx_filename);
            // パターン3：ACK送信しない
            }else{
                output_data_csv_rx_hdr(packet,rx_filename,3);
                output_data_csv_ordata(fp_rx,data_p,rx_filename);
            }
        // パターン4：ACK送信済み
        }else if(current->flag == ACK){
            output_data_csv_rx_hdr(packet,rx_filename,4);
            output_data_csv_ordata(fp_rx,data_p,rx_filename);
        }
        
        output_data_csv_space(fp_rx,rx_filename);
        return current;
    // currentのシーケンス番号と違う場合
    // 同シーケンスのパケット未受信
    }else{

        //パターン1・・未受信でデータ受信
        //パターン5・・未受信でACK受信

        routing_table_entry_t* r_current = r_table->head;
        while(r_current && !is_same_addr(r_current->addr,packet->DestAddr)){
            r_current = r_current->next;
        }

        or_data_packet_t* data_p = (or_data_packet_t*)packet->payload;

        // データパケット受信
        if(packet->type == DATA){
            output_data_csv_rx_hdr(packet,rx_filename,1);
            output_data_csv_ordata(fp_rx,data_p,rx_filename);
        // ACK受信(type == ACK)
        }else{
            output_data_csv_rx_hdr(packet,rx_filename,5);
        }
        
        // パケットテーブルに挿入
        printf("insert packet table\n");
        
        // 新しいパケットテーブルを作成し挿入
        packet_table_entry_t* new_entry = (packet_table_entry_t* )malloc(sizeof(packet_table_entry_t));
        memset(new_entry, 0, sizeof(packet_table_entry_t));
        new_entry->packet = (mac_frame_header_t*)malloc((int)packet->len);
        memcpy(new_entry->packet,packet,(int)packet->len);

        mac_set_addr(srcAddr, new_entry->srcAddr);
        new_entry->seq = seq;
        
        // 1 = DATA , 2 = ACK
        if(packet->type == DATA){
            new_entry->flag = DATA ;
        }else if(packet->type == ACK){
            new_entry->flag = ACK;
        }

        new_entry->next = NULL;
        (p_table)->size++;

        // 先頭に挿入(headの変更)
        if(!prev){
            new_entry->next = p_table->head;
            p_table->head = new_entry;
        // 途中に挿入   
        }else{
            new_entry->next = prev->next;
            prev->next = new_entry;
        }
        
        // csv出力(改行)
        output_data_csv_space(fp_rx,rx_filename);
        
        return new_entry;
    }
}


// バックオフを追加_関数
void insert_backoff(uint8_t* srcAddr,uint16_t seq,double backoff,struct timespec backoff_now){

    backoff_entry_t* current = bo_st->head;
    backoff_entry_t* previous = NULL;

    // currentのバックオフと比較
    while(current && is_smaller_backoff(current->backoff,backoff)){
        previous = current;
        current = current->next;
    }

    // バックオフ追加
    backoff_entry_t* new_entry = (backoff_entry_t* )malloc(sizeof(backoff_entry_t));
    memset(new_entry, 0, sizeof(backoff_entry_t));

    new_entry->backoff = backoff;
    mac_set_addr(srcAddr, new_entry->srcAddr);
    new_entry->seqNum = seq;
    new_entry->backoff_now.tv_sec = backoff_now.tv_sec;
    new_entry->backoff_now.tv_nsec = backoff_now.tv_nsec;
    new_entry->next = NULL;
    (bo_st)->size++;
    
    // 先頭に挿入(headの変更)
    if(!previous){
        new_entry->next = bo_st->head;
        bo_st->head = new_entry;
    // 途中に挿入          
    }else{
        new_entry->next = previous->next;
        previous->next = new_entry;
    }

    // 画面表示
    print_backoff_table();

    return ;
} 


// バックオフを削除_関数
void dequeue_backoff_table(){
    printf("backoff tableを削除します.\n");

    backoff_entry_t* toFree = NULL;
    
    if(bo_st->head){
        toFree = bo_st->head;
        bo_st->head = bo_st->head->next;
        free(toFree);
        --bo_st->size;
        print_backoff_table();
    }
}


// エラー表示_関数
void die(const char *s){
    perror(s);
    exit(1);
}


// ssPin(6)に0Vを出力_関数
void selectreceiver(){
    digitalWrite(ssPin, LOW);
}


// ssPin(6)に5Vを出力_関数
void unselectreceiver(){
    digitalWrite(ssPin, HIGH);
}


// レジスタを読み込む_関数
byte readReg(byte addr){

    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}


// レジスタに書き込む_関数
void writeReg(byte addr, byte value){

    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    
    unselectreceiver();
}


// デバイスのSLEEP・STANDBY・TX・RXの設定
static void opmode (uint8_t mode){
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~OPMODE_MASK) | mode);
}


// LoRaモードの設定（もう一方はFSKモード）
static void opmodeLora(){

    uint8_t u = OPMODE_LORA;

    if (sx1272 == false)
        u |= 0x8;  // TBD: sx1276 high freq
    writeReg(REG_OPMODE, u);
}


// LoRaの設定_関数
void SetupLoRa(){
    
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);

    byte version = readReg(REG_VERSION);

    if(version == 0x22){
        // sx1272
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    }else{
        // sx1276?
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        version = readReg(REG_VERSION);
        if(version == 0x12){
            // sx1276
            printf("SX1276 detected, starting.\n");
            sx1272 = false;
        }else{
            printf("Unrecognized transceiver.\n");
            //printf("Version: 0x%x\n",version);
            exit(1);
        }
    }

    opmode(OPMODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeReg(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeReg(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeReg(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeReg(REG_SYNC_WORD, 0x34);  // LoRaWAN public sync word

    if(sx1272){
        if(sf == SF11 || sf == SF12){
            writeReg(REG_MODEM_CONFIG,0x0B);
        }else{
            writeReg(REG_MODEM_CONFIG,0x0A);
        }
        writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    }else{
        if(sf == SF11 || sf == SF12){
            writeReg(REG_MODEM_CONFIG3,0x0C);
        }else if(sf == SF6){
            /*
            u1_t = mc1 = 0;
            mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
            mc1 |= 0x72;
            * */
            writeReg(REG_MODEM_CONFIG,0x73);
            //RegDetectOptimize
            writeReg(0x31,0xc5);
            //RefDetectionThreshold
            writeReg(0x37,0x0c);
            //
            writeReg(REG_MODEM_CONFIG,0x72);
            writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
        }else{
            writeReg(REG_MODEM_CONFIG3,0x04);
            //
            writeReg(REG_MODEM_CONFIG,0x72);
            writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
        }
        /*
        writeReg(REG_MODEM_CONFIG,0x72);
        writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
        */
    }

    if(sf == SF10 || sf == SF11 || sf == SF12){
        writeReg(REG_SYMB_TIMEOUT_LSB,0x05);
    }else{
        writeReg(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    writeReg(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeReg(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    writeReg(REG_HOP_PERIOD,0xFF);
    writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_BASE_AD));

    writeReg(REG_LNA, LNA_MAX_GAIN);

}


static void configPower(int8_t pw){
    if(sx1272 == false){
        // no boost used for now
        if(pw >= 17){
            pw = 15;
        }else if(pw < 2){
            pw = 2;
        }
        // check board type for BOOST pin
        writeReg(RegPaConfig, (uint8_t)(0x80|(pw&0xf)));
        writeReg(RegPaDac, readReg(RegPaDac)|0x4);

    }else{
        // set PA config (2-17 dBm using PA_BOOST)
        if(pw > 17){
            pw = 17;
        }else if(pw < 2){
            pw = 2;
        }
        writeReg(RegPaConfig, (uint8_t)(0x80|(pw-2)));
    }
}


// 送信モード_関数
void set_txmode(){
    
    opmodeLora();
    // enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY);
    writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08);  // set PA ramp-up time 50 uSec
    configPower(23);
}


// 受信モード_関数
void set_rxmode(){
    
    SetupLoRa();
    opmodeLora();
    opmode(OPMODE_STANDBY);
    opmode(OPMODE_RX);
}


// バッファ書き込み
static void writeBuf(byte addr, byte *value, byte len){

    unsigned char spibuf[256];                                                                          
    spibuf[0] = addr | 0x80;

    for (int i = 0; i < len; i++) {
        spibuf[i + 1] = value[i];
    }

    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, len + 1);
    unselectreceiver();
}


// チェックサムの計算_関数
uint16_t checksum(byte *data,int len){

    register uint32_t sum;
    register uint16_t *ptr;
    register int c;
    
    sum = 0;
    ptr = (uint16_t *)data;
    
    for(c = len;c > 1;c -= 2){
        sum += (*ptr);
        if(sum & 0x80000000){
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
        ptr++;
    }

    if(c == 1){
        uint16_t val;
        val = 0;
        memcpy(&val,ptr,sizeof(uint8_t));
        sum += val;
    }

    while(sum >> 16){
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return (~sum);
}


// チェックサムの比較_関数
boolean calc_checksum(mac_frame_header_t* hdr,int len){

    mac_frame_header_t *tmp = hdr;
    prev_checksum = 0;
    prev_checksum = hdr->checksum;
    
    tmp->checksum = 0;
    current_checksum = checksum((byte*)tmp,len);
    printf("origin(tx):%u\n",prev_checksum);
    printf("rx        :%u\n",current_checksum);
    return current_checksum == prev_checksum;
}


// パケット転送した時間を画面表示_関数
void get_time_rx_now(){
    clock_gettime(CLOCK_REALTIME,&ts_rx);
    localtime_r(&ts_rx.tv_sec,&tm_rx);
    printf("rx time:  %d/%02d/%02d %02d:%02d:%02d.%09ld\n",tm_rx.tm_year+1900,tm_rx.tm_mon+1,tm_rx.tm_mday,tm_rx.tm_hour,tm_rx.tm_min,tm_rx.tm_sec,ts_rx.tv_nsec);
}


// パケットを受信した時間を画面表示_関数
void get_time_tx_now(){
    clock_gettime(CLOCK_REALTIME,&ts_tx);
    localtime_r(&ts_tx.tv_sec,&tm_tx);
    printf("tx time:  %d/%02d/%02d %02d:%02d:%02d.%09ld\n",tm_tx.tm_year+1900,tm_tx.tm_mon+1,tm_tx.tm_mday,tm_tx.tm_hour,tm_tx.tm_min,tm_tx.tm_sec,ts_tx.tv_nsec);
}


// パケット送信_関数
void txlora(byte *frame, byte datalen) {

    //checksum function
    ((mac_frame_header_t*)frame)->checksum = 0;  // 送信するたびに0に初期化しないと値が一致しない
    ((mac_frame_header_t*)frame)->checksum = checksum(frame,(int)datalen);
    printf("checksum: %u\n",((mac_frame_header_t*)frame)->checksum);

    //get_time_now(&ts_tx,&tm_tx);
    get_time_tx_now();

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(REG_IRQ_FLAGS, 0xFF);
    // mask all IRQs but TxDone
    writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);

    // initialize the payload size and address pointers
    writeReg(REG_FIFO_TX_BASE_AD, 0x00);
    writeReg(REG_FIFO_ADDR_PTR, 0x00);
    writeReg(REG_PAYLOAD_LENGTH, datalen);

    //download buffer to the radio FIFO
    writeBuf(REG_FIFO, frame, datalen);
    // now we actually start the transmission
    opmode(OPMODE_TX);
    
    printf("\n");
    
    mac_frame_header_t* p_frame = (mac_frame_header_t*)frame;
    if(p_frame->type == HELLO){
        print_mac_frame_header(p_frame);
        printf("------------------\n");
    }else if(p_frame->type == DATA){
        print_mac_frame_header(p_frame);
        print_data_frame(p_frame);
        printf("------------------\n");
        // csv出力(2)(改行)
        output_data_csv_tx_time(tx_filename);
        output_data_csv_tx_hdr(p_frame,tx_filename);
        output_data_csv_ordata(fp_tx,((or_data_packet_t*)p_frame->payload),tx_filename);
        output_data_csv_space(fp_tx,tx_filename);
    }else if(p_frame->type == ACK){
        print_mac_frame_header(p_frame);
        printf("------------------\n");
        // csv出力(2)(改行)
        output_data_csv_tx_time(tx_filename);
        output_data_csv_tx_hdr(p_frame,tx_filename);
        output_data_csv_space(fp_tx,tx_filename);
    }
    
    for(int i = 0;i<datalen;i++){
        printf("%02x ",frame[i]);
    }

    printf("\n");
    
    usleep(250000);
}


// バックオフ開始時間_関数
void print_time(struct timespec *ts){

    struct tm tm;

    localtime_r(&ts->tv_sec,&tm);
    printf("bo start time:  %d/%02d/%02d %02d:%02d:%02d.%09ld\n",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec,ts->tv_nsec);
}


// バックオフ時間_関数(上の関数と統合)
void print_bo_time(struct timespec *ts){

    struct tm tm;

    localtime_r(&ts->tv_sec,&tm);
    printf("bo time:  %d/%02d/%02d %02d:%02d:%02d.%09ld\n",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec,ts->tv_nsec);
}


// シグモイド関数を計算_関数
double sigmoid(double gain, double x){
    return (1.0 / (1.0 + exp(-gain * x)));
}


// ORのバックオフを計算_関数
double OR_calculate_backoff(uint8_t desthop, uint8_t hop, double parameter, or_data_packet_t* data_p){
    
    struct timespec ts;
    struct tm tm;
    
    clock_gettime(CLOCK_REALTIME,&ts);
    localtime_r(&ts.tv_sec,&tm);
    srand((unsigned int)tm.tm_sec * ((unsigned int)ts.tv_nsec + 1));  // 乱数

    double expected = (double)desthop - 1;    // h_id - 1
    double table = (double)hop;               // h_rd
    double gain = 1.0;                        // α
    double alpha = 1.0;                       // β
    double beta = 1.0;                        // γ
    double difference = (table - expected) - alpha;  // ε_r - β
    double randVal = 0.0;                     // 0から1の小数乱数
    
    double backoff = 0.0;                     // b_r
    
    double max_random_backoff = (parameter) * (sigmoid(gain, difference + beta) - sigmoid(gain,difference));  // T_max × μ_r(ε_r)
    backoff += parameter * sigmoid(gain,difference);  // T_max × ς _r(ε_r)
    printf("difference        :%lf\n",difference);
    printf("max random backoff:%lf\n",max_random_backoff);
    randVal = (double)rand() / RAND_MAX;      // u

    if(max_random_backoff != 0){
        printf("max random backoff !=0\n");
        backoff += randVal * max_random_backoff;  // T_max × (ς _r(ε_r) + u × μ_r(ε_r))
    }else{
        printf("max random backoff :0\n");
        backoff += randVal * (10 * 1000);
    }

    printf("backoff           :%lf\n",backoff);
    printf("randVal = %lf\n",randVal);

    data_p->u = randVal;
    
    return backoff;
}


// 現在時刻を取得_関数
void get_time(struct timespec *ts_a){
    clock_gettime(CLOCK_REALTIME,ts_a);
}


// timespecの差を返す_関数(A:今 , B:送信時間)
double timessub(struct timespec *A, struct timespec *B)
{
    double time;
    struct timespec C;
    
    C.tv_sec  = A->tv_sec  - B->tv_sec;
    C.tv_nsec = A->tv_nsec - B->tv_nsec;
    if(C.tv_nsec < 0){
        C.tv_sec--;
        C.tv_nsec += 1000000000;
    }
    
    time = (double)C.tv_sec + ((double)C.tv_nsec / 1000000000);

    return time;
}

// 乱数値の加重移動平均計算_関数
void calculateSmoothedPassiveRandU(uint16_t newSeqNum, double newRandU, routing_table_entry_t* current){
	printf("calculateSmoothedPassiveRandU関数呼び出し\n");
    printf("newSeqNum = %d\n", newSeqNum);
    printf("newRandU = %lf\n", newRandU);
    
    current->lastPassiveRandU = newRandU;
	current->lastSeqNum = newSeqNum;

    // 重複転送が発生しない場合の重複転送発生確率の計算(単一転送フラグが1の場合のみ)
	if(current->firstReceived == 1){
		printf("calculateSmoothedPassiveRandU関数if分岐(current->firstReceived == 1)\n"); 
        current->dupReceivedRate = (0.9 * current->dupReceivedRate);
        printf("current->dupReceivedRate = %lf\n", current->dupReceivedRate); 
	}

	current->firstReceived = 1;  // 単一転送フラグ(1:単一転送の場合)

    // 初回(乱数値の平滑化の計算無し)
	if(current->smoothedPassiveRandU == 0){
        printf("calculateSmoothedPassiveRandU関数if分岐(current->smoothedPassiveRandU == 0)\n"); 
		current->smoothedPassiveRandU = newRandU;
        printf("current->smoothedPassiveRandU = %lf\n", current->smoothedPassiveRandU); 
	// 2回目以降(ρ:0.95として平滑化の計算)
    }else
	{
        printf("calculateSmoothedPassiveRandU関数else分岐(current->smoothedPassiveRandU == 0)\n"); 
		current->smoothedPassiveRandU = (0.05 * newRandU) + (0.95 * current->smoothedPassiveRandU);
        printf("current->smoothedPassiveRandU = %lf\n", current->smoothedPassiveRandU); 
	}
}


// 乱数値の差の加重移動平均計算_関数
void calculateSmoothedPassiveRandDiff(double newRandDiff, routing_table_entry_t* current){
	printf("calculateSmoothedPassiveRandDiff関数呼び出し\n"); 
    printf("newRandDiff = %lf\n", newRandDiff); 
    
    // 初回(乱数値の差の平滑化の計算無し)
    if(current->smoothedPassiveRandDiff == 0){
        printf("calculateSmoothedPassiveRandDiff関数if分岐(current->smoothedPassiveRandDiff == 0)\n"); 
		current->smoothedPassiveRandDiff = fabs(newRandDiff);
        printf("current->smoothedPassiveRandDiff = %lf\n", current->smoothedPassiveRandDiff); 
	// 2回目以降(ρ:0.9として平滑化の計算)
    }else{
		printf("calculateSmoothedPassiveRandDiff関数else分岐(current->smoothedPassiveRandU == 0)\n"); 
        current->smoothedPassiveRandDiff = (0.1 * fabs(newRandDiff)) + (0.9 * current->smoothedPassiveRandDiff);
        printf("current->smoothedPassiveRandDiff = %lf\n", current->smoothedPassiveRandDiff); 
	}

    // 重複転送発生確率の計算(単一転送フラグ関係無しで計算を行う)
	current->dupReceivedRate = (0.1) + (0.9 * current->dupReceivedRate);
    printf("current->dupReceivedRate = %lf\n", current->dupReceivedRate); 
	current->firstReceived = 0;  // 単一転送フラグ(0:重複転送の場合)
}


// 最小往復リンク時間の計算_関数
void calculateLrtt(struct timespec* now, struct timespec* txtime, routing_table_entry_t* current){
	printf("calculateLrtt関数呼び出し\n"); 
    
    double currentLrtt = timessub(now, txtime);  // 往復リンク時間
    printf("currentLrtt = %lf\n", currentLrtt); 

    // 初回
	if(current->Lrtt == 0){
        printf("calculateLrtt関数if分岐(current->Lrtt == 0)\n"); 
		current->Lrtt = currentLrtt;
		current->minLrtt = currentLrtt;
		current->maxLrtt = currentLrtt;
        printf("current->Lrtt = %lf\n", current->Lrtt); 
        printf("current->minLrtt = %lf\n", current->minLrtt); 
        printf("current->maxLrtt = %lf\n", current->maxLrtt); 
	// 2回目以降
    }else{
        printf("calculateLrtt関数else分岐(current->Lrtt == 0)\n"); 
		current->Lrtt = 0.1 * currentLrtt + 0.9 * current->Lrtt;
		current->minLrtt = fmin(current->minLrtt, currentLrtt);
		current->maxLrtt = fmax(current->maxLrtt, currentLrtt);
        printf("current->Lrtt = %lf\n", current->Lrtt); 
        printf("current->minLrtt = %lf\n", current->minLrtt); 
        printf("current->maxLrtt = %lf\n", current->maxLrtt); 
	}
}

// 1回目に再受信した場合_関数   cstosrcはパケットテーブルだと思ってよい ToDestはrouting_table_entry_t
void handleFirstReceive(routing_table_entry_t* current, packet_table_entry_t* p_current, mac_frame_header_t* packet_p){
    
    or_data_packet_t* data_p = (or_data_packet_t*)(packet_p->payload);
    int diff = current->hop - (data_p->destHop - 1);
    int exTravDist = ((or_data_packet_t*)p_current->packet->payload)->srcHop;

    printf("handleFirstReceive関数呼び出し\n"); 

    // ACKの処理(受信パケットの宛先ホップの方が近い)
    if(0 < diff){
        calculateLrtt(&ts_rx, &(p_current->txtime), current);  // リンクのrtt計算パケットテーブルに送信したときの時間を入れる

        // 受信パケットの送信元ノードまでのホップ数が同じ場合
        if(exTravDist == data_p->srcHop){
            calculateSmoothedPassiveRandU(packet_p->seqNum, data_p->u, current);  // 乱数値の加重平均
            smoothedPassiveRandU = current->smoothedPassiveRandU; // 追加
            //p_current->receiveCount++;//受信カウントを増やす
        }
    }
}


// 2回目に再受信した場合_関数
void handleDuplicationReceive(routing_table_entry_t* current, packet_table_entry_t* p_current, mac_frame_header_t* packet_p){
    
    or_data_packet_t* data_p = (or_data_packet_t*)(packet_p->payload);
    int exTravDist = ((or_data_packet_t*)p_current->packet->payload)->srcHop;  // 自身の送信元ノードのホップ + 1

    printf("handleDuplicationReceive関数呼び出し\n"); 

    // ホップ数が一致しているか判定
    if(exTravDist == data_p->srcHop){
        // 重複して受信した場合
        if(p_current->receiveCount == 1){
            printf("handleDuplicationReceive関数if分岐(p_current->receiveCount == 1)\n"); 
            // routing_table_entryのシーケンス番号と受信パケットのシーケンス番号の比較
            if(current->lastSeqNum == packet_p->seqNum){
                double randDiff = data_p->u - current->lastPassiveRandU;  // 乱数値の差を計算
                printf("randDiff = %lf\n", randDiff);

                calculateSmoothedPassiveRandDiff(randDiff, current);  //receiveCount == 0のときに保存された乱数値（1回目に再受信)とヘッダの乱数値の差を取る(or_data_packetに保存されている乱数値)
                smoothedPassiveRandDiff = current->smoothedPassiveRandDiff;  // 追加
            }
		}
	}
}


// 待機時間パラメータを計算する関数( 待機時間パラメータ parameter : 最小リンク往復遅延時間 current->minLitt : 平滑化した乱数値 current->smoothedPassiveRandU : 平滑化した乱数値の差 current->smoothedPassiveRandDiff )
void calculateParameter(routing_table_entry_t* current, packet_table_entry_t* p_entry){

    double threshold = 0.7;                 // しきい値(重複再受信:u_th)0.2
    double single_threshold = 0.4;          // しきい値(単一再受信:u^single_th)
    double duplicationProbability = 0.01;   // しきい値(重複転送発生確率:p_th)
    double eta = 0.02;                      // η
    double epsilon = 0.04;                  // ε
    double estimatedNode;                   // 推定中継ノード数:N~

    if(smoothedPassiveRandU <= 0.0){
        estimatedNode = 0.0;
    }else{
        estimatedNode = 1.0 / smoothedPassiveRandU - 1.0;
    }

    // 単一再受信の場合の処理
    if(p_entry->receiveCount == 0){
        if(current->dupReceivedRate <= duplicationProbability && smoothedPassiveRandU >= single_threshold)
            parameter = parameter - current->minLrtt / (eta * estimatedNode);
    // 重複再受信の場合の処理
    }else if(p_entry->receiveCount == 1){
        if(2.0 * smoothedPassiveRandDiff >= threshold + epsilon)
            parameter = parameter + current->minLrtt / (eta * estimatedNode);

        else if(2.0 * smoothedPassiveRandDiff < threshold - epsilon)
            parameter = parameter - current->minLrtt / (eta * estimatedNode);
    }

    if(parameter <= 0){
        parameter = 1.0;
    }
    
    printf("\nParameter: %lf , receiveCount: %d\n", parameter, p_entry->receiveCount);
}


// 受信パケットの処理分け_関数(ACK , DATA , HELLO)
void judge_transfer_data(mac_frame_header_t *packet_p){
    
    or_data_packet_t* data_p = (or_data_packet_t*)packet_p->payload;
    routing_table_entry_t* current = r_table->head;

    // 受信パケットがACKの場合
    if(packet_p->type == ACK){
        printf("ACK dataなので転送を中止します.\n");
        insert_packet_table(packet_p->SourceAddr,packet_p->seqNum,packet_p);
    // 受信パケットがDATAの場合
    }else if(packet_p->type == DATA){
            print_mac_frame_header(packet_p);
            print_data_frame(packet_p);
            
        // 宛先ノードが自分の場合
        if(is_same_addr(my_addr,packet_p->DestAddr)){
            printf("%s\n",data_p->message);
            printf("ACKを送信します.\n");
            
            // 宛先ノードのホップ数を調べる
            routing_table_entry_t* r_current = r_table->head;
            while(r_current && !is_same_addr(r_current->addr,packet_p->DestAddr)){
                r_current = r_current->next;
            }
            
            // csv出力(1)
            output_data_csv_rx_hdr(packet_p,rx_filename,1);
            output_data_csv_ordata(fp_rx,((or_data_packet_t*)packet_p->payload),rx_filename);
            output_data_csv_space(fp_rx,rx_filename);
            
            Ack_p->seqNum = packet_p->seqNum;
            mac_set_addr(packet_p->DestAddr,Ack_p->DestAddr);
            mac_set_addr(packet_p->SourceAddr,Ack_p->SourceAddr);

            mac_print_addr(Ack_p->DestAddr);
            mac_print_addr(Ack_p->SourceAddr);
            printf("seqnum: %u\n",Ack_p->seqNum);
            
            // 送信モード
            set_txmode();
            // ACK送信
            txlora((byte*)&Ack,(byte)Ack_p->len);
            // 受信モードに切り替え
            set_rxmode();
        // 宛先ノードが自分ではない場合
        }else{
            printf("transfer data\n");
            
            // 宛先ノードのホップ数を調べる 
            while(current && !is_same_addr(current->addr,packet_p->DestAddr)){
                current = current->next;
            }

            // ルーティングテーブルがある場合
            if(current){
                packet_table_entry_t* p_entry = insert_packet_table(packet_p->SourceAddr,packet_p->seqNum,packet_p); //中継済みかどうか判断する関数が必要
                
                // ACKを受信していない場合
                if(p_entry->flag != ACK){
                    // 転送待機時間の算出
                    double bo = OR_calculate_backoff(data_p->destHop, current->hop, data_p->parameter, (or_data_packet_t*)p_entry->packet->payload);
                    get_time(&after_backoff);
                    print_time(&after_backoff);
                    insert_backoff(packet_p->SourceAddr,packet_p->seqNum,bo,after_backoff);
                }else if(p_entry->transmitted == TRUE){
                    // 1回目に再受信した場合
                    if(p_entry->receiveCount == 0){
                        handleFirstReceive(current, p_entry ,packet_p);
                    // 2回目に再受信した場合
                    }else if(p_entry->receiveCount == 1){
                        handleDuplicationReceive(current, p_entry, packet_p);
                    }

                    calculateParameter(current, p_entry);  // 待機時間パラメータ算出
                    p_entry->receiveCount++;  // 受信カウントを増やす
                }
            }
        }
    // 受信パケットがHELLOの場合
    }else if(packet_p->type == HELLO){
        print_mac_frame_header(packet_p);
        insert_routing_table(packet_p->SourceAddr, 1, packet_p->seqNum);
        add_routing_table(packet_p);
        print_routing_table(r_table);
    // 受信パケットがその他の場合
    }else{
        printf("ここじゃないよ\n");
    }
}


// csvへの出力_関数(受信パケット(1番目))(Packet_RSSI , RSSI , SNR , length)
void output_data_csv_RSSI(int Packet_RSSI,int RSSI,long int SNR,int lora_length,char* filename){
    
    if((fp_rx = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }

    fprintf(fp_rx,"%d,%d,%li,%i,,",Packet_RSSI,RSSI,SNR,lora_length);
    
    fclose(fp_rx);
}


// 受信の確認とペイロード読み取り_関数
boolean receive(char *payload) {
    // パケット受信完了割り込み
    writeReg(REG_IRQ_FLAGS, 0x40);

    int irqflags = readReg(REG_IRQ_FLAGS);
    
    // payload crc:0x20
    // ペイロードCRCエラーが発生した場合
    if((irqflags & 0x20) == 0x20){
        printf("CRC error\n");
        writeReg(REG_IRQ_FLAGS, 0x20);
        return false;
    }else{
        byte currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR);  // 最後に受信したパケットのスタートアドレス
        byte receivedCount = readReg(REG_RX_NB_BYTES);  // 最新受信パケットのペイロードバイト数
        receivedbytes = receivedCount;

        // スタートアドレス読み込み
        writeReg(REG_FIFO_ADDR_PTR, currentAddr);

        // ペイロードの読み取り
        for(int i = 0; i < receivedCount; i++){
            payload[i] = (char)readReg(REG_FIFO);
        }
    }

    return true;
}


// パケットを受信_関数
void receivepacket() {

    long int SNR;
    int rssicorr;
    
    // 入力ピンを読む
    if(digitalRead(dio0) == 1){

        // 受信パケットのメッセージ確認
        if(receive(message)){
            mac_frame_header_t* p = (mac_frame_header_t*)message;
            // パケット受信時刻
            get_time_rx_now();

            // チェックサムが違う場合
            if(!calc_checksum(p,(int)receivedbytes)){
                printf("Length: %i", (int)receivedbytes);
                printf("パケットを破棄します.\n");
            // チェックサム確認後
            }else{
                // パケットSNR
                byte value = readReg(REG_PKT_SNR_VALUE);
                // SNR符号ビットは1
                if(value & 0x80){
                    // 反転して4で割る
                    value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                    SNR = -value;
                }else{
                    // 4で割る
                    SNR = ( value & 0xFF ) >> 2;
                }
                
                if(sx1272){
                    rssicorr = 139;
                }else{
                    rssicorr = 157;
                }
                count++;
                printf("rx count: %d\n",count);
                
                int packetrssi = readReg(0x1A)-rssicorr;
                int rssi = readReg(0x1B)-rssicorr;

                printf("\n");
                printf("------------------\n");
                printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
                printf("Packet RSSI: %d, ", packetrssi);
                printf("RSSI: %d, ",rssi);
                printf("SNR: %li, ", SNR);
                printf("Length: %i", (int)receivedbytes);
                printf("\n");
                
                if(p->type == DATA || p->type == ACK){
                    output_data_csv_rx_time(rx_filename);
                    output_data_csv_RSSI(packetrssi,rssi,SNR,(int)receivedbytes,rx_filename);
                }
                // パケットのタイプ判別
                judge_transfer_data(p);

                // メッセージ表示
                for(int i = 0;i < (int)receivedbytes;i++){
                    printf("%02x ",message[i]);
                }
                printf("\n");
                
            }
        }
    }  // dio0 = 1;
}


// ルーティングテーブル初期化_関数
void routing_table_init(){
    r_table = (routing_table_t *) malloc(sizeof(routing_table_t));
    r_table->size=0;
    r_table->head = NULL;
}


// バックオフテーブル初期化
void backoff_struct_init(){
    bo_st = (backoff_t*)malloc(sizeof(backoff_t));
    bo_st->size=0;
    bo_st->head=NULL;
}


// パケットテーブル初期化
void packet_table_init(){
    p_table = (packet_table_t*)malloc(sizeof(packet_table_t));
    p_table->size=0;
    p_table->head = NULL;
}


// パケットテーブルを確認_関数
packet_table_entry_t* check_packet_table(uint8_t *srcAddr,uint16_t seq){

    packet_table_entry_t* current = p_table->head;
    
    while(current && is_smaller_addr(current->srcAddr,srcAddr)){
        current = current->next;
    }
    
    while(current && is_same_addr(current->srcAddr,srcAddr) && current->seq < seq){
        current = current->next;
    }

    if(current && is_same_seq(current->seq,seq)){
        return current;
    }

    return NULL;
}


// csvへの出力_関数(送信パケット(2番目))(backoff_start_time , nsec , tx_time)
void output_data_csv_backoff_time(char* filename,struct timespec *start_backoff_time,double backoff){
    
    char time[128];
    struct tm ptm;

    if((fp_tx = fopen(filename,"a+")) == NULL){
        printf("can't open %s\n",filename);
        exit(1);
    }
    
    localtime_r(&start_backoff_time->tv_sec,&ptm);
    sprintf(time,"%d/%02d/%02d %02d:%02d:%02d",ptm.tm_year+1900,ptm.tm_mon+1,ptm.tm_mday,ptm.tm_hour,ptm.tm_min,ptm.tm_sec);
    
	fprintf(fp_tx,"%s,%09ld,%lf,",time,start_backoff_time->tv_nsec,backoff);

	fclose(fp_tx);
}


// バックオフの比較_関数
boolean bo_timediff(struct timespec prev_ts, double bo){

    struct timespec next_ts;
    long bo_long = (long)bo;
    
    next_ts.tv_sec = prev_ts.tv_sec + bo_long / 1000;
    long bo_long_nsec = (bo_long - bo_long / 1000 * 1000) * 1000 * 1000;
    next_ts.tv_nsec = prev_ts.tv_nsec + bo_long_nsec;

    // ナノ秒 -> 秒
    while(next_ts.tv_nsec > 1000*1000*1000){
        next_ts.tv_sec++;
        next_ts.tv_nsec -= 1000*1000*1000;
    }
    
    if(now.tv_sec > next_ts.tv_sec){
        return true;
    }else if(now.tv_sec == next_ts.tv_sec){
        if(now.tv_nsec >= next_ts.tv_nsec){
            return true;
        }
    }

    return false;
}


// 時間比較_関数
boolean timediff(){

    if(now.tv_sec > later.tv_sec){
        return false;
    }else if(now.tv_sec == later.tv_sec){
        if(now.tv_nsec >= later.tv_nsec){
            return false;
        }
    }

    return true;
}


// csvの名前入力_関数(受信パケット(1番目))
// file_name = 〇〇.csv
void rx_file_open(char* file_name){

    if((fp_rx = fopen(file_name,"w")) == NULL){
        printf("can't open %s\n",file_name);
        exit(1);
    }
    fprintf(fp_rx,"rx_time,time,nsec,PacketRSSI,RSSI,SNR,lora_length,,Type,pattern,srcAddr,senderAddr,destAddr,length,seq,tx_checksum,rx_checksum,,srcHop,destHop,u,parameter\n");
    fclose(fp_rx);
}


// csvの名前入力_関数(送信パケット(2番目))
// file_name = 〇〇.csv
void tx_file_open(char* file_name){

    if((fp_tx = fopen(file_name,"w")) == NULL){
        printf("can't open %s\n",file_name);
        exit(1);
    }
    fprintf(fp_tx,"backoff_start_time,nsec,backoff_value,tx_time,time,nsec,,Type,srcAddr,senderAddr,destAddr,length,seq,tx_checksum,,srcHop,destHop,u,parameter\n");
    fclose(fp_tx);
}


// csvの名前入力_関数(ルーティングテーブル(3番目))
// file_name = 〇〇.csv
void hop_file_open(char *file_name){

    if((fp_hop = fopen(file_name,"w")) == NULL){
        printf("can't open %s\n",file_name);
        exit(1);
    }
    fprintf(fp_hop,"time,time,nsec,srcAddress,hop,seq\n");
    fclose(fp_hop);
}


// メイン関数
int main (int argc, char *argv[]) {

    if(argv[1]){
        rx_filename = argv[1];
        rx_file_open(rx_filename);
    }else if(!argv[1]){
        printf("パケット受信保存ファイルを設定してください.\n");
        exit(1);
    }
    if(argv[2]){
        tx_filename = argv[2];
        tx_file_open(tx_filename);
    }else if(!argv[2]){
        printf("パケット送信保存ファイルを設定してください.\n");
        exit(1);
    }
    if(argv[3]){
        hop_filename = argv[3];
        hop_file_open(hop_filename);
    }else if(!argv[3]){
        printf("helloホップ数保存ファイルを設定してください.\n");
        exit(1);
    }

    // ルーティングテーブル初期化
    routing_table_init();
    
    // バックオフテーブル初期化
    backoff_struct_init();
    
    // パケットテーブル初期化
    packet_table_init();
    
    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    // クロック周波数
    wiringPiSPISetup(CHANNEL, 500000);
    
    // LoRaを受信モードにセット
    // receiverの設定
    set_rxmode();

    // 送信パケットの初期化
    memset(&Hello[0],0,sizeof(Hello));
    memset(&data[0],0,sizeof(data));
    memset(&Ack[0],0,sizeof(Ack));
    mac_tx_hello_frame_header_init(Hello_p);
    mac_tx_hello_frame_payload_init(Hello_p);
    mac_tx_data_frame_header_init(data_packet);
    mac_tx_data_frame_payload_init(data_packet);
    mac_tx_ack_frame_header_init(Ack_p);

    // 自身のmacアドレスの取得
	get_my_mac_addr();
    printf("my ");
	mac_print_addr(my_addr);
    
    // データ挿入
    // Helloパケット
    Hello_p->type = HELLO;
    mac_set_addr(my_addr,Hello_p->SourceAddr);
    mac_set_addr(my_addr,Hello_p->SenderAddr);
    // ACKパケット
    Ack_p->type = ACK;
    Ack_p->len = sizeof(mac_frame_header_t);
    mac_set_addr(my_addr,Ack_p->SenderAddr);
    // DATAパケット
    data_packet->type = DATA;
    mac_set_addr(my_addr,data_packet->SourceAddr);
    uint8_t dest_addr[6] = {0xb8, 0x27, 0xeb, 0x86, 0x45, 0xd3};
    mac_set_addr(dest_addr,data_packet->DestAddr);
    mac_set_addr(my_addr,data_packet->SenderAddr);
    
    // 時刻を5秒追加
    get_time(&later);
    later.tv_sec += 5;
    
    while(1){
        
        // 現在時刻を取得
        get_time(&now);
        
        // 現在時刻を比較(HELLOパケット送信用)
        if(timediff()){
            
            // パケット受信
            receivepacket(); 
            
            // バックオフテーブルにバックオフがある場合
            if(bo_st->head){
                // バックオフと現在時刻を比較
                if(bo_timediff(bo_st->head->backoff_now,bo_st->head->backoff)){
                    printf("bo_st->head->backoff_now_sec : %ld\n", bo_st->head->backoff_now.tv_sec);
                    printf("bo_st->head->backoff_now_nsec : %ld\n", bo_st->head->backoff_now.tv_nsec);
                    printf("bo_st->head->backoff_sec : %lf\n", bo_st->head->backoff);
                    printf("now_sec : %ld\n", now.tv_sec);
                    printf("now_nsec : %ld\n", now.tv_nsec);
                    
                    // パケットテーブルを確認
                    packet_table_entry_t* p_entry = check_packet_table(bo_st->head->srcAddr,bo_st->head->seqNum);
                    
                    // パケットがACKではない場合
                    if(p_entry->flag != ACK){
                        routing_table_entry_t* r_entry = r_table->head;

                        // 宛先ノードまでのホップ数探索
                        while(r_entry && !is_same_addr(r_entry->addr,p_entry->packet->DestAddr)){
                            r_entry = r_entry->next;
                        }

                        ((or_data_packet_t*)p_entry->packet->payload)->srcHop ++;
                        ((or_data_packet_t*)p_entry->packet->payload)->destHop = r_entry->hop;
                        ((or_data_packet_t*)p_entry->packet->payload)->parameter = parameter;  // 待機時間パラメータを追加
                        
                        printf("データを転送します.\n");
                        
                        get_time(&backoff_now);
                        p_entry->transmitted = TRUE;
                        p_entry->txtime.tv_sec = backoff_now.tv_sec;
                        p_entry->txtime.tv_nsec = backoff_now.tv_nsec;
                        mac_set_addr(my_addr,p_entry->packet->SenderAddr);

                        // 送信モード
                        set_txmode();
                        
                        // csv出力(2)
                        output_data_csv_backoff_time(tx_filename,&bo_st->head->backoff_now,bo_st->head->backoff);
                        
                        // データ送信
                        txlora((byte*)p_entry->packet,(byte)p_entry->packet->len);
                        
                        //受信モードに切り替え
                        set_rxmode();

                    }
                    // バックオフ削除
                    dequeue_backoff_table();
                }
            }
        // HELLOパケット送信
        }else{
            count = 0;
            // 時間測定
            sender=clock();
            
            // 送信モードにセット
            set_txmode();
            
            printf("------------------\n");
            printf("Send packets at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
            
            Hello_p->seqNum++;
            
            int num = get_routing_table(Hello_p);
            int header_len = sizeof(mac_frame_header_t);
            int payload_len = num * sizeof(mac_frame_payload_t);
            printf("%d %d\n",header_len,payload_len);
            
            int total_len = header_len + payload_len;
            printf("total length: %d\n",total_len);
            
            // サイズ確認
            if(total_len >= 255){
                printf("最大ペイロードサイズを超えています.\n");
                return 0;
            }else{
                Hello_p->len = total_len;
                // HELLOパケット送信
                txlora((byte*)&Hello, (byte)total_len);
            }
            
            // 受信モードに切り替え
            set_rxmode();
            
            // 時間計測
            receiver = clock();
            printf("%f秒\n",(double)(receiver-sender)/CLOCKS_PER_SEC);
            
            // 時間の更新
            get_time(&later);
            gettimeofday(&tv_rand,NULL);
            srand(tv_rand.tv_sec * (tv_rand.tv_usec + 1));
            
            // HELLOパケット送信周期
            later.tv_sec +=29;

            printf("now: %ld\n",now.tv_sec);
            printf("later: %ld\n",later.tv_sec);

            later.tv_nsec = rand() % (2* 1000* 1000 * 1000);

            if(later.tv_nsec > 1000*1000*1000){
                later.tv_sec++;
                later.tv_nsec -= 1000*1000*1000;
            }
        }
    }

    return (0);
}
