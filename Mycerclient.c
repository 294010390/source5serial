#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/ioctl.h>
#include <netinet/tcp.h>
#include <time.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/socket.h> /* for socket(), bind(), and connect() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <stdbool.h>
#include <stdarg.h>
#include <semaphore.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <linux/input.h>
#include "tslib.h"
// #include "nw_vmf.h"
#include "tslib.h"

#include <sys/time.h>
#include <sys/stat.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/ioctl.h>

typedef unsigned long					DWORD;
typedef unsigned char					BYTE;
typedef unsigned short					USHORT;
typedef int								HANDLE;
#define MAX_PATH						260

// touch process
#define SYN_REPORT              		0
#define SYN_CONFIG              		1
#define SYN_MT_REPORT           		2
#define SYN_DROPPED             		3
#define SYN_MAX                 		0xf
#define SYN_CNT                 		(SYN_MAX+1)

#define ABS_MT_TOUCH_MAJOR 0x30 /* Major axis of touching ellipse */

#define ABS_MT_TOUCH_MINOR 0x31 /* Minor axis (omit if circular) */

#define ABS_MT_WIDTH_MAJOR 0x32 /* Major axis of approaching ellipse */

#define ABS_MT_WIDTH_MINOR 0x33 /* Minor axis (omit if circular) */

#define ABS_MT_ORIENTATION 0x34 /* Ellipse orientation */

#define ABS_MT_POSITION_X 0x35 /* Center X ellipse position */

#define ABS_MT_POSITION_Y 0x36 /* Center Y ellipse position */

#define ABS_MT_TOOL_TYPE 0x37 /* Type of touching device */

#define ABS_MT_BLOB_ID 0x38 /* Group a set of packets as a blob */

#define ABS_MT_TRACKING_ID      0x39    /* Unique ID of initiated contact */
#define ABS_MT_PRESSURE         0x3a    /* Pressure on contact area */
#define ABS_MT_DISTANCE         0x3b    /* Contact hover distance */
#define ABS_MT_STATE            0x3c    /* Three states which is down, move, up*/
#define ZF_MAX_X 				0x7FFF
#define ZF_MAX_Y				0X7FFF

enum
{
	EVENT_LEFTDOWN, 			// 0
	EVENT_LEFTUP, 				// 1
	EVENT_MOUSEMOVE, 			// 2
	EVENT_KEYDOWN, 				// 3
	EVENT_KEYUP, 				// 4
	EVENT_LEFTROTATE, 			// 5
	EVENT_RIGHTROTATE 			// 6
};

enum
{
	HARDKEY_MOUSE = 0, 			//
	HARDKEY_HOME = 102, 		//
	HARDKEY_PREVIOUS = 108, 	//
	HARDKEY_NEXT = 103, 		//
	HARDKEY_MEDIA = 226, 		//
	HARDKEY_ROTATE = 1, 		//
	HARDKEY_POWER = 116
};

typedef struct _tagMDCCOMMAND{
	DWORD	dwLengthPackage;	// package length: sizeof(MDCCOMMAND)
	DWORD	dwCommand;			// connect, exit, notify, copy...
	DWORD	dwIndex;		    // re-transfer package index
	DWORD	dwErrorCode;		// error code device GetLastError()
	DWORD	dwFileSize;			// the file's size
	USHORT	event;
	USHORT	key;
	USHORT	x;
	USHORT	y;
	USHORT	steps;
}MDCCOMMAND, *LPMDCCOMMAND;

bool				v_bExitTransfer = false;
bool				v_bmmoveOpen = false;
bool				v_bmmoveStop = true;
int speed_arr[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
int v_Serial = -1;
bool v_bExitSerialMonitor = false;

#define msleep(x)		usleep(x*1000)

void OutputDebugMsg(char *pszFormat, ...)
{
	va_list ArgList;
	char szTemp[MAX_PATH] = "\0";
	char szTime[MAX_PATH] = "\0";

	va_start(ArgList, pszFormat);

	vsprintf(szTemp, pszFormat, ArgList);

	struct timeval logTime;
	double timems;
	gettimeofday(&logTime, NULL);
	long long tv_us = 0;
	tv_us = 1000000*(long long)logTime.tv_sec;
	timems = (tv_us + logTime.tv_usec)/1000.0;
	sprintf(szTime, "%lf	%s", timems, szTemp);
	printf(szTime);
	writefile(szTime);
}

int stop_thread_func(pthread_t *pthread)
{
	if ( *pthread !=0 )
	{
		pthread_join(*pthread, NULL);
	}
	printf("exit thread sus\r\n");
}

int sem_timedwait_millsecs(sem_t *sem, long msecs)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	long secs = msecs/1000;
	msecs = msecs%1000;

	long add = 0;
	msecs = msecs*1000*1000 + ts.tv_nsec;
	add = msecs / (1000*1000*1000);
	ts.tv_sec += (add + secs);
	ts.tv_nsec = msecs%(1000*1000*1000);

	return sem_timedwait(sem, &ts);
}

// jan.li update it for k256 project
int reportkey(int fd, uint16_t type, uint16_t keycode, int32_t value)
{
	struct input_event event;
	memset(&event, 0, sizeof(event));
	gettimeofday(&event.time, 0); gettimeofday(&event.time, 0);
	event.type = type;
	event.code = keycode;
	event.value = value;

	if ( write(fd, &event, sizeof(struct input_event)) < 0 )
	{
        	printf("$$$$$$$$$$$IT_AUTO: write key error\r\n");
        	return -1;
	}
	else
	{
		// printf("$$$$$$$$$$$IT_AUTO: write key success\r\n");
	}
	return 0;
}

// jan.li update it for k256 project
int Key_process(int keycode, int event_type, long x, long y)
{
	// printf("$$$$$$$$$$$IT_AUTO: Key_process and key code is %x\r\n", keycode);
	// printf("$$$$$$$$$$$IT_AUTO: x value = %x\r\n", x);
	// printf("$$$$$$$$$$$IT_AUTO: y value = %x\r\n", y);
    static int k_fd;

	switch (keycode)
	{
		case HARDKEY_HOME:
		case HARDKEY_PREVIOUS:
		case HARDKEY_NEXT:
		case HARDKEY_MEDIA:
		case HARDKEY_ROTATE:
		{
			if ( event_type == EVENT_KEYDOWN )
			{
				k_fd = open("/dev/input/event0", O_RDWR);
				if ( k_fd < 0 )
				{
					printf("$$$$$$$$$$$IT_AUTO: open /dev/event1 error\r\n");
					return k_fd;
				}
				// printf("$$$$$$$$$$$IT_AUTO: EVENT_KEYDOWN\r\n");
				reportkey(k_fd, EV_KEY, keycode, 1);
				reportkey(k_fd, EV_SYN, 0, 0);
			}
			else if ( event_type == EVENT_KEYUP )
			{
				// printf("$$$$$$$$$$$IT_AUTO: EVENT_KEYUP\r\n");
				reportkey(k_fd, EV_KEY, keycode, 0);
				reportkey(k_fd, EV_SYN, 0, 0);
				close(k_fd);
			}
			else if ( event_type == EVENT_LEFTROTATE )
			{
				k_fd = open("/dev/input/event1", O_RDWR);
				if ( k_fd < 0 )
				{
					printf("$$$$$$$$$$$IT_AUTO: open /dev/event2 error\r\n");
					return k_fd;
				}
				// printf("$$$$$$$$$$$IT_AUTO: EVENT_LEFTROTATE\r\n");
				reportkey(k_fd, 2, keycode, -1);
				reportkey(k_fd, EV_SYN, 0, 0);
				close(k_fd);
			}
			else if ( event_type == EVENT_RIGHTROTATE )
			{
				k_fd = open("/dev/input/event1", O_RDWR);
				if ( k_fd < 0 )
				{
					printf("$$$$$$$$$$$IT_AUTO: open /dev/event2 error\r\n");
					return k_fd;
				}
				// printf("$$$$$$$$$$$IT_AUTO: EVENT_RIGHTROTATE\r\n");
				reportkey(k_fd, 2, keycode, 1);
				reportkey(k_fd, EV_SYN, 0, 0);
				close(k_fd);
			}
			else
			{
				/* Do Nothing */
			}
		}
		break;
		case HARDKEY_MOUSE:
		{
			switch (event_type)
			{
				case EVENT_MOUSEMOVE:
				case EVENT_LEFTDOWN:
				{
					// printf("$$$$$$$$$$$IT_AUTO: EVENT_LEFTDOWN\r\n");
					if (v_bmmoveOpen)
					{
						moveprocess(x, y);
					}
					else
					{
						pressprocess(x, y);
						v_bmmoveOpen = true;
					}
					// pressprocess(x, y);
				}
				break;
				case EVENT_LEFTUP:
				{
					// printf("$$$$$$$$$$$IT_AUTO: EVENT_LEFTUP\r\n");
					releaseprocess(x, y);
					v_bmmoveOpen = false;
				}
				break;
				default:
				{
					/* Do Nothing */
				}
				break;
			}
		}
		break;
		default:
		{
			/* Do Nothing */
			// printf("$$$$$$$$$$$IT_AUTO: UNKNOWN DEVICE ACTION\r\n");
		}
		break;
 	}
	return 0;
}

// jan.li update it for k256 project
int hahaprocess(long x, long y, long haha1, long haha2, long haha3, long haha4, long haha5, long haha6)
{
	struct tsdev *ts;
	char *tsdevice = "/dev/input/event0";
	ts = ts_open(tsdevice, 0);
	int fd = ts_fd(ts);
	if ( fd < 0 ) {
		printf("$$$$$$$$$$$IT_AUTO: open /dev/touchscreen error\r\n");
	}
	reportkey(fd, EV_ABS, ABS_MT_POSITION_X, x);
	reportkey(fd, EV_ABS, ABS_MT_POSITION_Y, y);
	reportkey(fd, EV_ABS, ABS_MT_STATE, haha1);
	reportkey(fd, EV_ABS, ABS_MT_TRACKING_ID, haha2);
	reportkey(fd, EV_ABS, ABS_MT_TOUCH_MAJOR, haha3);
	reportkey(fd, EV_ABS, ABS_MT_PRESSURE, haha4);
	reportkey(fd, EV_ABS, ABS_MT_DISTANCE, haha5);
	reportkey(fd, EV_ABS, ABS_MT_ORIENTATION, haha6);
	reportkey(fd, EV_SYN, SYN_MT_REPORT,0);
	reportkey(fd, EV_SYN, 0, 0);
	usleep(1000);
	close(fd);
	return 0;
}

// jan.li update it for k256 project
int hhhhprocess(long x, long y, long haha1, long haha2)
{
	struct tsdev *ts;
	char *tsdevice = "/dev/input/event0";
	ts = ts_open(tsdevice, 0);
	int fd = ts_fd(ts);
	if ( fd < 0 ) {
		printf("$$$$$$$$$$$IT_AUTO: open /dev/touchscreen error\r\n");
	}
    reportkey(fd, EV_ABS, ABS_MT_POSITION_X, x);
    reportkey(fd, EV_ABS, ABS_MT_POSITION_Y, y);
	reportkey(fd, EV_ABS, ABS_MT_STATE, haha1);
	reportkey(fd, EV_ABS, ABS_MT_TRACKING_ID, haha2);
    reportkey(fd, EV_SYN, SYN_MT_REPORT,0);
    reportkey(fd, EV_SYN, 0, 0);
    usleep(1000);
    close(fd);
    return 0;
}

// jan.li update it for k256 project
int pressprocess(long x, long y)
{
	// printf("$$$$$$$$$$$IT_AUTO: pressprocess\r\n");
	OutputDebugMsg("$$$$$$$$$$$IT_AUTO: pressprocess\r\n");
	struct tsdev *ts;
	char *tsdevice = "/dev/input/event2";
	ts = ts_open(tsdevice, 0);
	int fd = ts_fd(ts);
	if ( fd < 0 ) {
		printf("$$$$$$$$$$$IT_AUTO: open /dev/touchscreen error\r\n");
	}
	// jan.li_2017_07_17_marking: NGI2.0 low得分辨率是1280*768，需要公式转换x*1280/800，y*768/480
	// jan.li_2017_08_07_marking: 现在分辨率已经改回800*480，不需要公式转换，直接x,y传进去
	reportkey(fd, EV_ABS, ABS_MT_POSITION_X, /*(800-x)*32767/800*/x);
	reportkey(fd, EV_ABS, ABS_MT_POSITION_Y, /*(480-y)*32767/480*/y);
	reportkey(fd, EV_ABS, ABS_MT_STATE, 0);
	reportkey(fd, EV_ABS, ABS_MT_TRACKING_ID, 0);
	reportkey(fd, EV_ABS, ABS_MT_TOUCH_MAJOR, 2);
	reportkey(fd, EV_ABS, ABS_MT_PRESSURE, 40);
	reportkey(fd, EV_ABS, ABS_MT_DISTANCE, 0);
	reportkey(fd, EV_ABS, ABS_MT_ORIENTATION, 0);
	reportkey(fd, EV_SYN, SYN_MT_REPORT,0);
	reportkey(fd, EV_SYN, 0, 0);
	// usleep(50*1000);
	usleep(1000);
	close(fd);
	return 0;
}

// jan.li update it for k256 project
int moveprocess(long x, long y, long major)
{
	OutputDebugMsg("$$$$$$$$$$$IT_AUTO: moveprocess\r\n");
	struct tsdev *ts;
	char *tsdevice = "/dev/input/event2";
	ts = ts_open(tsdevice, 0);
	int fd = ts_fd(ts);
	if ( fd < 0 ) {
		printf("$$$$$$$$$$$IT_AUTO: open /dev/touchscreen error\r\n");
	}
	// jan.li_2017_07_17_marking: NGI2.0 low得分辨率是1280*768，需要公式转换x*1280/800，y*768/480
	// jan.li_2017_08_07_marking: 现在分辨率已经改回800*480，不需要公式转换，直接x,y传进去
    reportkey(fd, EV_ABS, ABS_MT_POSITION_X, /*(800-x)*32767/800*/x);
    reportkey(fd, EV_ABS, ABS_MT_POSITION_Y, /*(480-y)*32767/480*/y);
	reportkey(fd, EV_ABS, ABS_MT_STATE, 1);
	reportkey(fd, EV_ABS, ABS_MT_TRACKING_ID, 0);
	reportkey(fd, EV_ABS, ABS_MT_TOUCH_MAJOR, 2);
	reportkey(fd, EV_ABS, ABS_MT_PRESSURE, 30);
	reportkey(fd, EV_ABS, ABS_MT_DISTANCE, 0);
	reportkey(fd, EV_ABS, ABS_MT_ORIENTATION, 0);
    reportkey(fd, EV_SYN, SYN_MT_REPORT, 0);
    reportkey(fd, EV_SYN, 0, 0);
	// usleep(50*1000);
    usleep(1000);
    close(fd);
    return 0;
}

// jan.li update it for k256 project
int releaseprocess(long x,long y)
{
	// printf("$$$$$$$$$$$IT_AUTO: releaseprocess\r\n");
	OutputDebugMsg("$$$$$$$$$$$IT_AUTO: releaseprocess\r\n");
    struct tsdev *ts;
    char *tsdevice = "/dev/input/event2";
    ts = ts_open(tsdevice, 0);
    int fd = ts_fd(ts);
    if ( fd < 0 ) {
        printf("$$$$$$$$$$$IT_AUTO: open /dev/touchscreen error\r\n");
    }
	reportkey(fd, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
	// jan.li_2017_07_17_marking: NGI2.0 low得分辨率是1280*768，需要公式转换x*1280/800，y*768/480
	// jan.li_2017_08_07_marking: 现在分辨率已经改回800*480，不需要公式转换，直接x,y传进去
    reportkey(fd, EV_ABS, ABS_MT_POSITION_X, /*(800-x)*32767/800*/x);
    reportkey(fd, EV_ABS, ABS_MT_POSITION_Y, /*(480-y)*32767/480*/y);
	reportkey(fd, EV_ABS, ABS_MT_STATE, 2);
	reportkey(fd, EV_ABS, ABS_MT_TRACKING_ID, 0);
    reportkey(fd, EV_SYN, SYN_MT_REPORT, 1);
    reportkey(fd, EV_SYN, 0, 0);
    // usleep(50*1000);
	usleep(1000);
    close(fd);
    return 0;
}

int open_port(char *pPortString)
{
	if ( pPortString == NULL || *pPortString == '\0' )
	{
		printf("invalid port string\r\n");
		return -1;
	}
	int fd = -1;
	fd = open(pPortString, O_RDWR|O_NOCTTY|O_NDELAY/*|O_NONBLOCK*/);
	if ( fd == -1 )
	{
		printf("open com port %s failed\r\n", pPortString);
		return -1;
	}
	printf("open com port %s sus\r\n", pPortString);

	// block
	fcntl(fd, F_SETFL, 0);

	return (fd);
}

void close_port()
{
	if ( v_Serial != -1 )
	{
		close(v_Serial);
		v_Serial = -1;
	}
	printf("close com port sus\r\n");

	return;
}

int set_speed(int fd, int speed)
{
	int i;
	int   status;
	struct termios opt;
	tcgetattr(fd, &opt);
	bzero(&opt, sizeof(opt));
	for (i=0; i<sizeof(speed_arr)/sizeof(int); i++)
	{
		if ( speed == name_arr[i] )
		{
			printf("found your defined baud rate\r\n");
			cfsetispeed(&opt, speed_arr[i]);
			cfsetospeed(&opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &opt);
			if  ( status != 0 )
			{
				printf("set baud rate failed, errno %d\r\n", errno);
				return -1;
			}

			printf("set baud rate sus\r\n");
			tcflush(fd, TCIOFLUSH);
			return 0;
		}
	}

	printf("not found your defined baud rate\r\n");
	return -1;
}

int set_parity(int fd, int databits, int stopbits, int parity)
{
	struct termios options;
	tcgetattr(fd, &options);
	// options.c_lflag |= ICANON;// \D5\E2\B8\F6\B5ط\BD\D3е\E3\B9\EE\D2죬\B2\BBҪȥ\C9\E8\CB\FC\A1\A3
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CLOCAL | CREAD;
	switch (databits)
	{
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		printf("unsupported data size\r\n");
		return -1;
	}
	switch (stopbits)
	{
	case 1:
		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		printf("unsupported stop bits\r\n");
		return -1;
	}
	switch (parity)
	{
	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB;
		options.c_iflag &= ~INPCK;
		break;
	case 'o':
	case 'O':
		options.c_cflag |= (PARODD | PARENB);
		options.c_iflag |= INPCK | ISTRIP;
		break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		options.c_iflag |= INPCK | ISTRIP;
		break;
	default:
		printf("unsupported parity\r\n");
		return -1;
	}

	options.c_cc[VTIME] = 1;
	options.c_cc[VMIN] = 0;

	tcflush(fd, TCIOFLUSH);
	if ( tcsetattr(fd, TCSANOW, &options) != 0 )
	{
		printf("set parity failed\r\n");
		return -1;
	}

	printf("set parity sus\r\n");
	return 0;
}
/*
void writefile(char *buff)
{
	FILE	*fp = NULL;
	if ( (fp=fopen("/opt/catkmsg.txt", "a+")) == NULL )
	{
		printf("open file error\r\n");
		return;
	}

	fseek(fp, 0, SEEK_END);
	fwrite(buff, 1, strlen(buff), fp);
	fclose(fp);
	fp = NULL;
}*/

#define FILE_MODE 0644
void writefile(char *buff)
{
	int fd;
	fd = open("/opt/catkmsg.txt", O_RDWR|O_CREAT|O_APPEND, FILE_MODE);
	if ( fd < 0 )
	{
		printf("open file error\r\n");
		return;
	}
	int rv;
	rv = write(fd, buff, strlen(buff));
	if ( rv != strlen(buff) )
	{
		printf("write file error\r\n");
	}
	fsync(fd);
	close(fd);
}

static int DoSerialInput()
{
	int status = -1;
	// open com port
	v_Serial = open_port("/dev/ttymxc3");
	if ( v_Serial <= 0 )
	{
		printf("open /dev/ttymxc3 failed\r\n");
		return 0;
	}
	printf("port handle value is %d\r\n", v_Serial);
	// set baud rate
	status = set_speed(v_Serial, 115200);
	if ( status == -1 )
	{
		printf("set serial baud rate failed\r\n");
		close_port();
		return 0;
	}
	printf("set serial baud rate sus\r\n");
	// set parity
	status = set_parity(v_Serial, 8, 1, 'N');
	if ( status == -1 )
	{
		printf("set serial parity failed\r\n");
		close_port();
		return 0;
	}
	printf("set serial parity sus\r\n");

	fd_set fdR;
	struct timeval timeout = {0, 0};

	MDCCOMMAND command;
	memset(&command, 0, sizeof(MDCCOMMAND));
 	while ( !v_bExitSerialMonitor )
 	{
		FD_ZERO(&fdR);
		FD_SET(v_Serial, &fdR);
		timeout.tv_sec = 0;
		timeout.tv_usec = 1000;

		// wait file descriptor change
		// OutputDebugMsg("DoSerialInput: will select\r\n");
		if ( -1 == select(v_Serial + 1, &fdR, NULL, NULL, &timeout) )
		{
			OutputDebugMsg("$$$$$$$$$$$: error from select\r\n");
			break;
		}
		// OutputDebugMsg("DoSerialInput: select sus\r\n");
		// was there any data?
		if ( !FD_ISSET(v_Serial, &fdR) )
		{
			// OutputDebugMsg("DoSerialInput: no any data coming\r\n");
			continue;
		}
		OutputDebugMsg("$$$$$$$$$$$: some data coming\r\n");

		u_long bytes_available = 0;
		int iReturn = ioctl(v_Serial, FIONREAD, &bytes_available);
		if ( iReturn == -1 || bytes_available == 0 )
		{
			OutputDebugMsg("$$$$$$$$$$$: iReturn %d, bytes_available %d, errno %d, double check data length failed\r\n", 
					iReturn, bytes_available, errno);
			continue;
		}
		OutputDebugMsg("$$$$$$$$$$$: double check %d data in serial buffer\r\n", bytes_available);
		do 
		{
			memset(&command, 0, sizeof(MDCCOMMAND));
			BYTE *pDataBuffer = NULL;
			pDataBuffer = &command;
			DWORD dwRetryTimes = 0;

			DWORD dwBytesRead = read(v_Serial, pDataBuffer, sizeof(MDCCOMMAND));
			OutputDebugMsg("$$$$$$$$$$$: read server input command dyBytesRead %d\r\n", dwBytesRead);
			if ( dwBytesRead > 0 && dwBytesRead < sizeof(MDCCOMMAND) )
			{
				// jan.li_2017_07_14_marking:update retry times from 10 to 32 because NGI2.0 low low performance
				while ( dwBytesRead < sizeof(MDCCOMMAND) && dwRetryTimes < 32 )
				{
					DWORD dwExceptionBytesRead = read(v_Serial, pDataBuffer+dwBytesRead, (sizeof(MDCCOMMAND) - dwBytesRead));
					dwBytesRead += dwExceptionBytesRead;
					dwRetryTimes++;
					OutputDebugMsg("$$$$$$$$$$$: exception retry %d, read %d\r\n", dwRetryTimes, dwExceptionBytesRead);
				}
				if ( dwBytesRead != sizeof(MDCCOMMAND) )
				{
					OutputDebugMsg("$$$$$$$$$$$: exception retry 32 times, still error %d\r\n", dwBytesRead);
					break;
				}
			}
			else if ( dwBytesRead <= 0 )
			{
				OutputDebugMsg("$$$$$$$$$$$: exception read server input command 0 byte\r\n");
				continue;
			}
			Key_process(command.key, command.event, command.x, command.y);
			OutputDebugMsg("$$$$$$$$$$$: read server input command sus, dyBytesRead %d, dwIndex %d\r\n", dwBytesRead, command.dwIndex);
			OutputDebugMsg("$$$$$$$$$$$: command value, command.key=%d, command.event=%d, command.x=%d, command.y=%d\r\n", command.key,  						command.event, command.x, command.y);
			// jan.li_2017_02_20_marking: update this below code to deal ioctl return 30, but read 32 problem
			if ( bytes_available < dwBytesRead )
				bytes_available = 0;
			else 
				bytes_available -= dwBytesRead;
			OutputDebugMsg("$$$$$$$$$$$: server input command left %d\r\n", bytes_available);
			if ( bytes_available > 0 && bytes_available < 32 )
				break;
		}while ( bytes_available > 0 );
 	}
	FD_CLR(v_Serial, &fdR);

	close_port();

	OutputDebugMsg("safe to exit DoSerialInput\r\n");

	return 0;
}

int main(int argc, char* argv[])
{
	/*for serial port receive*/
	system("rm -rf /opt/catkmsg.txt");
	sleep(1);
	pthread_t serialThreadID;
	pthread_create(&serialThreadID, NULL, DoSerialInput, NULL);

	// \D5\E2\B8\F6\B5ط\BD\B5\C4scanf̫\B1\E4̬\C1ˣ\AC\CB\FCҲ\BB\E1\B6\C1ȡ\B4\AE\BF\DA\CA\FD\BEݣ\AC\CB\F9\D2\D4\CB\FC\B6\C1\D7\DF\C1ˣ\AC\CEҾ\CDһֱ\B6\C1\B2\BB\B5\BD\A3\AC\B8\E3\C1\CB̫\BE\C3\D5\E2\B8\F6\CE\CA\CC⡣
	// char input[20] = {0};
	// scanf("%s", &input);
	while ( /*strcmp(input, "exit") != 0*/ !v_bExitTransfer )
	{
		// memset(input, 0, sizeof(input));
		// scanf("%s", &input);
		sleep(1);
	}

	v_bExitTransfer = true;
	v_bExitSerialMonitor = true;
	sleep(1);

	return 0;
}
