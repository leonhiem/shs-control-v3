/*
 * Sun  7 Aug 09:24:53 UTC 2016
 * L.Hiemstra@gmail.com
 *
 * derived from:
 *   i2ctest.cpp
 *   Raspberry Pi I2C interface demo
 *   Byron Jacquot @ SparkFun Electronics>
 *   4/2/2014
 *   https://github.com/sparkfun/Pi_Wedge
 *   This code is beerware; Distributed as-is; no warranty is given.
 *   This code makes use of the Wiring Pi library, which streamlines the interface
 *   You can learn about installing Wiring Pi here: http://wiringpi.com/download-and-install/
 *   The wiringPi SPI API is documented here: https://projects.drogon.net/raspberry-pi/wiringpi/spi-library/
 *
 *   The init call returns a standard file descriptor.  More detailed configuration
 *   of the interface can be performed using ioctl calls on that descriptor.
 *   See the wiringPi SPI implementation (wiringPi/wiringPiSPI.c) for some examples.
 *   Parameters configurable with ioctl are documented here:
 *   http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi/spidev
 *
 * To build this file, I use the command on the Raspberry Pi:
 *   g++ sccmon.cpp -lwiringPi -o sccmon
 */

#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>

#define MAX_RETRIES 10

union semun {
        int val;
        struct semid_ds *buf;
        ushort *array;
};
int initsem(key_t key, int nsems); 



using namespace std;


string battstate_to_str(unsigned char battstate)
{
    switch(battstate) {
      case 3:
        return "NORMAL";
      case 9:
        return "FULL  ";
      case 10:
        return "GONE! ";
      case 11:
        return "EMPTY!";
      default:
        return "??????";
    }
}
string sysstate_to_str(unsigned char sysstate)
{
    switch(sysstate) {
      case 1:
        return "INIT  ";
      case 2:
        return "WAIT  ";
      case 3:
        return "CHARGE";
      case 10:
        return "ERROR ";
      default:
        return "??????";
    }
}
string solarstate_to_str(unsigned char solarstate)
{
    switch(solarstate) {
      case 0:
        return "OK  ";
      case 1:
        return "LOW ";
      case 2:
        return "HIGH";
      default:
        return "????";
    }
}
string loadstate_to_str(unsigned char loadstate)
{
    switch(loadstate) {
      case 0:
        return "OFF OK   ";
      case 1:
        return "ON  OK   ";
      case 2:
        return "OFF FAULT";
      case 3:
        return "ON  FAULT";
      default:
        return "??????";
    }
}
float vsolar_to_volt(unsigned short vsolar)
{
//    float v_adc=(float)vsolar * 3.3 / 1023;
//    float i=v_adc/3000.;
//    float v_solar=i*(120500.); // rt=47e3+47e3+(47e3/2)+3000

//octave:14> 3.3+i*(47e3/2+47e3+47e3)
//ans =  132.55

    float v=(float)vsolar * 132.55 / 1023.;
    return v;
}
float isense_to_amps(unsigned short isense)
{
    // 0.4 @ 4A
    float i=(float)isense * 33. / 1023.;
    return i;
}
float vbatt_to_volt(unsigned short vbatt)
{
    float v=(float)vbatt * 16.5 / 1023.;
    return v;
}

int main(int argc, char* argv[])
{
   int fd, result,i,r;
   unsigned char buffer[100];
   unsigned char buffer_out[100];
   int linecount=0;

   unsigned long minutes;
   unsigned char sysstate,battstate,solarstate,loadstate,SoC,pwm,checksum,checksum_chk;
   unsigned char syncbyte;
   char temperature;
   unsigned short openklem,adc_vsolar,adc_isense_in,adc_vbatt,adc_isense_out;



    /*
     * Hoped that the Linux SPI device driver would do mutual exclusion locking
     * but it doesn't.. So here is an alternative: locking by semaphore (from Stevens)
     */
    key_t key;
    int semid;
    struct sembuf sb;
    sb.sem_num = 0;
    sb.sem_op = -1;  /* set to allocate resource */
    sb.sem_flg = SEM_UNDO;



   if(argc < 2) {
       fprintf(stderr,"Usage: %s channel [loadstate]\n",argv[0]);
       fprintf(stderr,"       channel: 0 or 1 (required)\n");
       fprintf(stderr,"       loadstate: ON or OFF (optional)\n");
       return(0);
   }

   // channel is the wiringPi name for the chip select (or chip enable) pin.
   // Set this to 0 or 1, depending on how it's connected.
   int CHANNEL = atoi(argv[1]);
   string loadstate_str = "NC";
   if(argc > 2) {
       loadstate_str = string(argv[2]);
   }

   if(CHANNEL == 0 || CHANNEL == 1) {
       cout << "Selecting SPI channel: " << CHANNEL << endl;
   } else {
       fprintf(stderr,"channel 0 or 1 !\n");
       return(0);
   }

   if(loadstate_str == "ON" || loadstate_str == "OFF") {
       cout << "Set load to: " << loadstate_str << endl;
   } else if(loadstate_str == "NC") {
       cout << "No change for load state" << endl;
   } else {
       fprintf(stderr,"loadstate set ON or OFF !\n");
       return(0);
   }

   cout << "Initializing" << endl ;

    if ((key = ftok("sccmon.cpp", 'J')) == -1) {
        perror("ftok");
        exit(1);
    }
    /* grab the semaphore set created by seminit: */
    if ((semid = initsem(key, 1)) == -1) {
        perror("initsem");
        exit(1);
    }

   fd = wiringPiSPISetupMode (CHANNEL, 10000, 0);

#define PACKET_SIZE 22
   int cnt=0;
   unsigned char cmd=0;
   while(1) {
               switch(cnt) {
                   case 0:
                       if(loadstate_str == "ON") {
                           cmd = 0xc1;
                           printf("LOAD ___ON___ cmd=0x%x\n",cmd);
                       } else if(loadstate_str == "OFF") {
                           cmd = 0xc0;
                           printf("LOAD ___OFF___ cmd=0x%x\n",cmd);
                       } 
                       break;
                   case 1:
                       cmd = 0x55;
                       printf("master watchdog reset cmd=0x%x\n",cmd);
                       break;
               }
               cnt++;
               if(cnt>=50) cnt=1;  // skip cnt=0



           //printf("Trying to lock...\n");
           sb.sem_op = -1; /* set to allocate resource */
           if (semop(semid, &sb, 1) == -1) {
               perror("semop");
               exit(1);
           }
           //printf("Locked.\n");


           // send sync cmd:
           buffer[0] = 0xa5;
           result = wiringPiSPIDataRW(CHANNEL, buffer, 1);
           usleep(10000);
           //printf("sync:%x\n",buffer[0]);

           for(i=0;i<PACKET_SIZE+1;i++) {
               buffer[0] = cmd; // official command to send
               result = wiringPiSPIDataRW(CHANNEL, buffer, 1);
               buffer_out[i]=buffer[0];
               usleep(10000);
               //printf("%x ",buffer_out[i]);
           }
           //printf("\n");


           sb.sem_op = 1; /* free resource */
           if (semop(semid, &sb, 1) == -1) {
               perror("semop");
               exit(1);
           }
           //printf("Unlocked\n");


           {
               i=1; // offset of 1 because SPI shifting
               syncbyte=buffer_out[i++];

               minutes=((unsigned long)buffer_out[i++])<<16;
               minutes|=((unsigned long)buffer_out[i++])<<8;
               minutes|=((unsigned long)buffer_out[i++])&0xff;

               sysstate=buffer_out[i++];
               battstate=buffer_out[i++];
               solarstate=buffer_out[i++];
               loadstate=buffer_out[i++];
               SoC=buffer_out[i++];

               openklem=((unsigned short)buffer_out[i++])<<8;
               openklem|=((unsigned short)buffer_out[i++])&0xff;

               adc_vsolar=((unsigned short)buffer_out[i++])<<8;
               adc_vsolar|=((unsigned short)buffer_out[i++])&0xff;

               adc_isense_in=((unsigned short)buffer_out[i++])<<8;
               adc_isense_in|=((unsigned short)buffer_out[i++])&0xff;

               adc_vbatt=((unsigned short)buffer_out[i++])<<8;
               adc_vbatt|=((unsigned short)buffer_out[i++])&0xff;

               adc_isense_out=((unsigned short)buffer_out[i++])<<8;
               adc_isense_out|=((unsigned short)buffer_out[i++])&0xff;

               temperature=buffer_out[i++];
               pwm=buffer_out[i++];
               checksum=buffer_out[i];
               checksum_chk=0;
               buffer_out[i]=0;
               for(i=1;i<PACKET_SIZE+1;i++) {
                   checksum_chk+=buffer_out[i];
               }

               if(checksum_chk == checksum && syncbyte == 0xa5) {
                   printf("OK ");
               } else {
                   printf("XX ");
               }
               printf("%d %x %08ld ",CHANNEL,syncbyte,minutes);
               cout << sysstate_to_str(sysstate) << " ";
               cout << battstate_to_str(battstate) << " ";
               cout << solarstate_to_str(solarstate) << " ";
               printf(" %3d ",SoC);

               float Voc     = vsolar_to_volt(openklem);
               float Vpv     = vsolar_to_volt(adc_vsolar);
               float Icharge = isense_to_amps(adc_isense_in);
               float Vbatt   = vbatt_to_volt(adc_vbatt);
               float Iload   = isense_to_amps(adc_isense_out);

               printf("%.02f ",Voc);
               printf("%.02f ",Vpv);
               printf(" %.02f ",Icharge);
               printf("  %.02f ",Vbatt);
               printf(" %.02f ",Vbatt*Icharge);
               printf(" %.02f ",Iload);
               printf(" %d ",temperature);
               printf(" %d  ",pwm);

               cout << loadstate_to_str(loadstate) << endl;

               if((linecount%20)==0) {
fprintf(stderr,"\n");
fprintf(stderr,"  CH     minutes system battst solst SoC  Voc   Vpv  Icharge  Vbatt Pin  Iload temp pwm loadstate\n");
fprintf(stderr,"----------------+------+------+-----+---+-----+-----+-------+------+----+-----+----+----+---------\n");
               }
               linecount++;
           }
   }
   return(0);
}



/*
** initsem() -- inspired by W. Richard Stevens' UNIX Network
** Programming 2nd edition, volume 2, lockvsem.c, page 295.
*/
int initsem(key_t key, int nsems)  /* key from ftok() */
{
        int i;
        union semun arg;
        struct semid_ds buf;
        struct sembuf sb;
        int semid;

        printf("initsem\n");
        semid = semget(key, nsems, IPC_CREAT | IPC_EXCL | 0666);

        if (semid >= 0) { /* we got it first */
                sb.sem_op = 1; sb.sem_flg = 0;
                arg.val = 1;

                printf("pressing return\n");// getchar();

                for(sb.sem_num = 0; sb.sem_num < nsems; sb.sem_num++) {
                        /* do a semop() to "free" the semaphores. */
                        /* this sets the sem_otime field, as needed below. */
                        if (semop(semid, &sb, 1) == -1) {
                                int e = errno;
                                semctl(semid, 0, IPC_RMID); /* clean up */
                                errno = e;
                                return -1; /* error, check errno */
                        }
                }

        } else if (errno == EEXIST) { /* someone else got it first */
                int ready = 0;

                semid = semget(key, nsems, 0); /* get the id */
                if (semid < 0) return semid; /* error, check errno */

                /* wait for other process to initialize the semaphore: */
                arg.buf = &buf;
                for(i = 0; i < MAX_RETRIES && !ready; i++) {
                        semctl(semid, nsems-1, IPC_STAT, arg);
                        if (arg.buf->sem_otime != 0) {
                                ready = 1;
                        } else {
                                sleep(1);
                        }
                }
                if (!ready) {
                        errno = ETIME;
                        return -1;
                }
        } else {
                return semid; /* error, check errno */
        }

        return semid;
}
