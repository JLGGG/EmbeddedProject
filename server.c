#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<arpa/inet.h>
#include<sys/socket.h>
#include<time.h>
#include<sys/select.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "bluetooth.h" //bluetooth header
#include "rfcomm.h"    //bluetooth header

#define BUF_SIZE 100
void error_handling(char *buf);
int initport(int fd);
int open_port(void);

int main(int argc, char* argv[])
{
   //bluetooth socket  variable-------------------------------
   struct sockaddr_rc addr = { 0 };
   int bluetoothSock, status, i, j=0, k=0;
   char dest[18] = "98:D3:32:11:18:7F"; //arduino mac address
   //---------------------------------------------------------

   //TCP socket variable
   int serv_sock, clnt_sock, serial_fd, sm2android;
   struct sockaddr_in serv_adr, clnt_adr, sm2android_addr;
   struct timeval timeout;
   fd_set reads, cpy_reads;
   socklen_t adr_sz;
   int fd_max, str_len, fd_num;
   char buf[BUF_SIZE];

   //zigbee port open---------------------------
   serial_fd=open_port();
   if(serial_fd==-1)
      perror("Error opening serial port /dev/ttyUSB0\n");
   else{
      printf("Serial port /dev/ttyUSB0 is open\n");
      if(initport(serial_fd)==-1){
         perror("Error initializing port");
         close(serial_fd);
         return 0;
      }
   }
   //-------------------------------------------

   //android to sm5 socket open---------------
   if(argc!=2){
      printf("Usage : %s <port>\n", argv[0]);
      exit(1);
   }

   memset(buf, 0, sizeof(buf));
   serv_sock=socket(PF_INET, SOCK_STREAM, 0);
   memset(&serv_adr, 0, sizeof(serv_adr));
   serv_adr.sin_family=AF_INET;
   serv_adr.sin_addr.s_addr=htonl(INADDR_ANY);
   serv_adr.sin_port=htons(atoi(argv[1]));
   //-----------------------------------------

   //bluetooth socket open --------------------
   bluetoothSock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

   // set the connection parameters (who to connect to)
   addr.rc_family = AF_BLUETOOTH;
   addr.rc_channel = (uint8_t) 1;
   str2ba( dest, &addr.rc_bdaddr );
   //------------------------------------------

   //sm5->android socket open-------------------
   sm2android=socket(PF_INET, SOCK_STREAM, 0);
   if(sm2android==-1)
      error_handling("sm2android socket() error ");

   memset(&sm2android_addr, 0, sizeof(sm2android_addr));
   sm2android_addr.sin_family=AF_INET;
   sm2android_addr.sin_addr.s_addr=inet_addr("192.168.0.2");
   sm2android_addr.sin_port=htons(atoi("9191"));
   //--------------------------------------------

   if(bind(serv_sock, (struct sockaddr*) &serv_adr, sizeof(serv_adr))==-1)
      error_handling("bind() error");
   if(listen(serv_sock, 5)==-1)
      error_handling("listen() error");

   //select 함수 호출을 사용해서 통신
   //step one-----------------------------
   //파일 디스크립터의 설정
   //검사의 범위 지정
   //타팀아웃의 설정
   //-------------------------------------
   //step two-----------------------------
   //select 함수의 호출
   //-------------------------------------
   //step three---------------------------
   //호출 결과 확인
   //-------------------------------------

   //step one-----------------------------
   FD_ZERO(&reads);
   FD_SET(serv_sock, &reads); //android -> sm5 감시
   FD_SET(serial_fd, &reads); //zigbee 감시
   fd_max=serv_sock+1;
   //-------------------------------------

   //멀티 플렉싱 서버
   while(1){
      cpy_reads=reads;
      timeout.tv_sec=5;
      timeout.tv_usec=5000;

      //step two
      if((fd_num=select(fd_max+1, &cpy_reads, 0, 0, &timeout))==-1)
         break;
      if(fd_num=0)
         continue;

      for(i=0;i<fd_max+1;i++){
         if(FD_ISSET(i, &cpy_reads)){

            //android -> sm5 -> arduino 연결
            if(i==serv_sock){
               adr_sz=sizeof(clnt_adr);
               //안드로이드 앱이 sm5에 연결 요청
               clnt_sock=accept(serv_sock, (struct sockaddr*)&clnt_adr, &adr_sz);
               FD_SET(clnt_sock, &reads);
               if(fd_max<clnt_sock)
                  fd_max=clnt_sock;
               printf("connected client: %d\n", clnt_sock);
            }
            //android -> sm5 -> arduino 통신
            else if(i!=serial_fd){
               str_len=read(i, buf, BUF_SIZE);
               if(str_len==0){
                  FD_CLR(i, &reads);
                  close(i);
                  printf("closed client: %d\n", i);
               }
               else{
                  printf("android -> sm5 -> arduino processing...\n");
                  buf[strlen(buf)]='\0';
                  if(k==0){
                     //bluetooth를 사용해서 arduino에게 데이터 전달
                     connect(bluetoothSock, (struct sockaddr *)&addr, sizeof(addr));
                     k++;
                  }
                  write(bluetoothSock, buf, str_len);
                  //write(i, buf, str_len);
                  printf("%s\n", buf);
                  memset(buf, 0, sizeof(buf));
               }
            }
            //zigbee -> sm5 -> android 통신
            else if(i==serial_fd){
               printf("zigbee -> sm5 -> android processing...\n");
               usleep(5000000);
               char read_buf[128];
               //zigbee serial port로부터 데이터 읽기
               int n1 = read(serial_fd, read_buf, sizeof read_buf);

               // sleep(1);

               if (n1 < 0)
                  fputs("Read failed!\n", stderr);
               else{
                  if(j==0){
                     if(connect(sm2android, (struct sockaddr*)&sm2android_addr, sizeof(sm2android_addr))==-1)
                        perror("sm2android connect() error :");
                     j++;
                  }
                   //android 앱으로 데이터 전송
                   write(sm2android, read_buf, strlen(read_buf));
                    printf("Successfully read from serial port : %s With %d Bytes\n", read_buf,n1);
                    memset(read_buf, 0, sizeof(read_buf));
               }

            }
         }
      }
   }
   close(sm2android); //sm5->android socket close
   close(serial_fd); //zigbee fd close
   close(bluetoothSock); //bluetooth socket close
   close(serv_sock); //server socket close
   return 0;
}
void error_handling(char *buf){
   fputs(buf, stderr);
   fputc('\n', stderr);
   exit(1);
}

int initport(int fd)
{
    int portstatus = 0;

    struct termios options;

    // Get the current options for the port...
    tcgetattr(fd, &options);

    // Set the baud rates to 9600...
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    //options.c_cflag |= SerialDataBitsInterp(8);     /* CS8 - Selects 8 data bits */
    options.c_cflag &= ~CRTSCTS;                      // Disable hardware flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);       // Disable XON XOFF (for transmit and receive)
    //options.c_cflag |= CRTSCTS;                     /* Enable hardware flow control */

    options.c_cc[VMIN] = 1;   //Minimum characters to be read
    options.c_cc[VTIME] = 2;    //Time to wait for data (tenths of seconds)
    options.c_oflag &=~OPOST;
    options.c_iflag &=~(ICANON | ECHO | ECHOE | ISIG);
    // Set the new options for the port...
    tcsetattr(fd, TCSANOW, &options);

    //Set the new options for the port...
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) == -1)
    {
        perror("On tcsetattr:");
        portstatus = -1;
    }
    else
        portstatus = 1;

    return (portstatus);
}

int open_port(void)
{
    int fd; /* File descriptor for the port */
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1)
    {
        /*      Could not open the port.        */
        perror("Open_port: Unable to open /dev/ttyUSB0 --- \n");
    }
    else
    //{message print....};

    return (fd);
}
