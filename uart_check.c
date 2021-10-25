/*******************************************************/
/* Program to test serial port throughput              */
/*                                                     */
/*******************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <fcntl.h> // Required for open()
#include <unistd.h> // Required for read(), write(), close()
#include <termios.h> // Required for tcgetattr(), cfsetispeed(), cfsetospeed(), tcflush()
#include <errno.h> // Require for errno
#include <unistd.h> // Required for error codes, among other things
#include <poll.h> // Required for poll()
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>

#define VERSION_MAJOR 1
#define VERSION_MINOR 0

#define DEVLENGTH 128
#define BAUDLENGTH 32

#define RX 1
#define TX 2

static const char *optString = "vd:b:rt";

typedef struct {
	int verbose;			/* enable verbose output */
	char devpath[DEVLENGTH];	/* -d serial device */
	int baudrate;			/* baudrate to use */
	int sermode;			/* serial mode RX or TX */
	int fd;				/* file descriptor */
} Settings_t;

// <-- add_value
#define  FIFO_FROM_UART   "/tmp/from_uart_fifo"
#define  FIFO_TO_UART     "/tmp/to_uart_fifo"
#define  BUFF_SIZE   1024

int   counter = 0;
int   fd_from_uart;
int   fd_to_uart;
char  buff[BUFF_SIZE];

int     handle;

const char *TitleMessage  = "Welcome Serial Port\r\n";
char       Buff[256];
int        RxCount;
int        loop;
int        ending;    
int        key;

Settings_t MySettings;
long rxcount = 0;
long txcount = 0;
// --> add_value

/*************************************************************/
/* parse_args() - Parse command line arguments and set       */
/* values as necessary.                                      */
/* parameters: argc = argument count, argv = argument values */
/*************************************************************/
void parse_args(int argc, char *argv[], Settings_t *settings)
{
	int opt = 0;

	/* set default device path */
	//strncpy(settings->devpath, "/dev/ttyTHS2", DEVLENGTH);
	strncpy(settings->devpath, "/dev/ttyTHS1", DEVLENGTH);
	/* set default baud rate */
	settings->baudrate = 115200;

	opt = getopt( argc, argv, optString );
	while( opt != -1 ) {
		switch( opt ) {
			case 'v':
				// turn on verbose mode
				settings->verbose = 1;
				break;
			case 'd':
				// sets the device path
				strncpy(settings->devpath, argv[2], DEVLENGTH);
				break;
			case 'b':
				// sets the baud rate
				settings->baudrate = atoi(argv[2]);
				break;
			case 'r':
				// sets to serial receive mode
				settings->sermode = RX;
				break;
			case 't':
				// sets to serial transmit mode
				settings->sermode = TX;
				break;
			case 'h':
			case '?':
				printf("sertest version %d.%d\n", VERSION_MAJOR, VERSION_MINOR);
				printf("usage: ./%s [-v] [-d device] [-b baud] -t|-r\n", argv[0]);
				printf("  -v enables verbose mode\n");
				printf("  -d <devicename> sets the serial device\n");
				printf("  -b <baud> sets the baud rate\n");
				printf("  -t sets mode to transmit\n");
				printf("  -r sets mode to receive\n");
				exit( EXIT_FAILURE );
				break;
			default:
				/* You won't actually get here. */
				break;
		}
		opt = getopt( argc, argv, optString );
	}

	if (settings->verbose) {
		printf("Arguments:\n");
		printf("  -v: %d\n", settings->verbose);
		printf("  -d: %s\n", settings->devpath);
		printf("  -b: %d\n", settings->baudrate);
		printf("  -r/t: %d\n", settings->sermode);
	}
}

/*************************************************************/
/* openport() - opens the selected serial port and gets a    */
/* file descriptor                                           */
/*************************************************************/
int openport(Settings_t *settings)
{
    settings->fd = open(settings->devpath, O_RDWR | O_NOCTTY | O_NDELAY);
    if (settings->fd <= 0) {
        char errorString[0xFF];
        strerror_r(errno, errorString, sizeof(errorString));
        fprintf(stderr, "Open device failed: Unable to open device file %s. Error: %s\n", settings->devpath, errorString);
        return -1;
    } else {
	if (settings->verbose) {
		fprintf(stderr, "Got file descriptor: %d\n", settings->fd);
	}
    }

    // Reset the serial device file descriptor for non-blocking read / write
    fcntl(settings->fd, F_SETFL, 0);

    return 0;
}

/*************************************************************/
/* configureport() - configures the selected serial port     */
/*************************************************************/
int configureport(Settings_t *settings)
{
    // Modify the settings on the serial device (baud rate, 8n1, receiver enabled, ignore modem status, no flow control) and apply them
    struct termios deviceOptions;
    memset (&deviceOptions, 0, sizeof deviceOptions);
    tcgetattr(settings->fd, &deviceOptions);

    /* Set the baud rates... */
    speed_t MyBaud;
    switch (settings->baudrate) {
	case 9600:
		MyBaud = B9600;
		break;
	case 57600:
		MyBaud = B57600;
		break;
	case 115200:
		MyBaud = B115200;
		break;
	case 230400:
		MyBaud = B230400;
		break;
	case 460800:
		MyBaud = B460800;
		break;
	case 921600:
		MyBaud = B921600;
		break;
	case 1000000:
		MyBaud = B1000000;
		break;
	case 2000000:
		MyBaud = B2000000;
		break;
	case 3000000:
		MyBaud = B3000000;
		break;
	default:
		MyBaud = B9600;
    }

    //cfsetispeed(&deviceOptions, settings->baudrate);
    //cfsetospeed(&deviceOptions, settings->baudrate);
    cfsetispeed(&deviceOptions, MyBaud);
    cfsetospeed(&deviceOptions, MyBaud);

    // Set character length
    deviceOptions.c_cflag &= ~CSIZE;       // Mask the character size bits
    deviceOptions.c_cflag |= CS8;

    // No parity
    deviceOptions.c_cflag &= ~PARENB;
    deviceOptions.c_cflag |= PARODD;

    // Stick parity flag
    #ifdef __USE_MISC
    deviceOptions.c_cflag |= CMSPAR;
	#endif

    // ignores parity errors and passes bytes regardless
    deviceOptions.c_iflag |= (IGNPAR);

    //causes parity errors to be 'marked' in the input stream using special characters.
    //If IGNPAR is enabled, a NUL character (0x00) is sent to your program before every character with a parity error.
    //Otherwise, a DEL (0x7F) and NUL character is sent along with the bad character.
    deviceOptions.c_iflag |= (PARMRK);

    //enable checking and stripping of the parity bit
    //NOTE: PARMRK and INPCK are exclusive of each other
    //options.c_iflag |= (INPCK);
    //options.c_iflag |= (ISTRIP);

    // One stop bit
    deviceOptions.c_cflag &= ~CSTOPB;

    /* No hardware flow control */
	#ifdef __USE_MISC
    deviceOptions.c_cflag &= ~CRTSCTS;
	#endif

    // Raw input, no echo
    deviceOptions.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | IEXTEN | ISIG);
    deviceOptions.c_iflag &=  ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | INPCK );

    // Raw output
    deviceOptions.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                ONOCR | OFILL | OLCUC | OPOST);

    // Wait timeout for each character
    deviceOptions.c_cc[VMIN] = 1;
    deviceOptions.c_cc[VTIME] = 0;
	
    /* Enable the receiver and set local mode... */
    deviceOptions.c_cflag |= (CLOCAL | CREAD);

    tcflush(settings->fd, TCIFLUSH);
    if (tcsetattr(settings->fd, TCSANOW, &deviceOptions) != 0) {
        close(settings->fd);
        char errorString[0xFF];
        strerror_r(errno, errorString, sizeof(errorString));
        fprintf(stderr, "Create failed: Unable to set options on device. Error: %s\n", errorString);
        return -1;
    }
    if (settings->verbose) {
    	printf("Configured serial device file descriptor for 8n1 and no flow control: %s\n", settings->devpath);
    }
    return 0;
}

/*************************************************************/
/* readBytes() - reads bytes from the selected serial port   */
/*************************************************************/
ssize_t readBytes(int deviceFileDescriptor, void* destination, ssize_t size)
{
    ssize_t bytesRead = 0;
    while (bytesRead < (ssize_t)size) {
        ssize_t result = read(deviceFileDescriptor, destination + bytesRead, size - bytesRead);
        if (result <= 0 && errno != EWOULDBLOCK) {
            char errorString[0xFF];
            strerror_r(errno, errorString, sizeof(errorString));
            fprintf(stderr, "Failed to read from device descriptor. Error: %s\n", errorString);
            return result;
        }
        if (result <= 0 && errno != EAGAIN) {
            char errorString[0xFF];
            strerror_r(errno, errorString, sizeof(errorString));
            fprintf(stderr, "Error: %s\n", errorString);
            return result;
        }
        bytesRead += result;
    }
    return bytesRead;
}

/*************************************************************/
/* writeBytes() - writes bytes to the selected serial port   */
/*************************************************************/
ssize_t writeBytes(int deviceFileDescriptor, const void* source, ssize_t size) {
    ssize_t bytesWritten = 0;
    while (bytesWritten < size) {
        ssize_t result = write(deviceFileDescriptor, source + bytesWritten, size - bytesWritten);
        if (result <= 0 && errno != EWOULDBLOCK) {
            char errorString[0xFF];
            strerror_r(errno, errorString, sizeof(errorString));
            fprintf(stderr, "Failed to write to serial device descriptor. Error: %s\n", errorString);
            return result;
        }
        bytesWritten += result;
    }

    return bytesWritten;
}
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

int main_menu(void)
{
    int key;

    printf("\n\n");
    printf("-------------------------------------------------\n");
    printf("                    MAIN MENU\n");
    printf("-------------------------------------------------\n");
    printf(" 1. apple                                        \n");
    printf(" 2. person(banana)                               \n");
    printf(" 3. bicycle                                      \n");
    printf(" 4. dog                                          \n");
    printf(" 5. truck                                        \n");

    printf(" a. Turn Left                                    \n");
    printf(" b. Turn Right                                   \n");
    printf(" c. Forward                                      \n");
    printf(" d. backward                                     \n");
    printf(" i. stop                                         \n");
    printf(" I. speed up +10                                 \n");
    printf(" D. speed down -10                               \n");


    printf("-------------------------------------------------\n");
    printf(" q. Motor Control application QUIT               \n");
    printf("-------------------------------------------------\n");
    printf("\n\n");
 
    printf("SELECT THE COMMAND NUMBER : ");

    key=getch();
 
    return key;
}

// <-- add_function
void* rx_function(void *data)
{
	// Reset the serial device file descriptor for blocking read / write
	fcntl(MySettings.fd, F_SETFL, 0);

	if (configureport(&MySettings) != 0) {
		exit(-1);
	}

	// Begin reading
	char testchar;
	char expected_char = 'A';

	fprintf(stderr, "Waiting for data...\n");

	while (MySettings.fd > 0) {
		// Wait until there is data to read or a timeout happens
		struct pollfd pollOptions;
		pollOptions.fd = MySettings.fd;
		pollOptions.events = POLLIN;
		pollOptions.revents = 0;
		poll(&pollOptions, 1, 200);
		ssize_t result = readBytes(MySettings.fd, &testchar, 1);
		if (result != 1) {
			if (errno != EWOULDBLOCK) {
				char errorString[0xFF];
				strerror_r(errno, errorString, sizeof(errorString));
				fprintf(stderr, "Error reading data (%s). Exiting...\n", errorString);
				break;
			}
		}
		rxcount++;
		printf("command recived : %c\n", testchar);

		// print status
		if (rxcount % 100 == 0) {
			fprintf(stderr, "rx: %ld\n", rxcount);
		}
	}
	
}

void* tx_function(void *data)
{
	// Reset the serial device file descriptor for non-blocking read / write
	fcntl(MySettings.fd, F_SETFL, O_NONBLOCK);

	if (configureport(&MySettings) != 0) {
		exit(-1);
	}

	// Begin writing
	char testchar;
	while ((key=main_menu()) != 0) {
		// Wait until port is writeable
		struct pollfd pollOptions;
		pollOptions.fd = MySettings.fd;
		pollOptions.events = POLLOUT;
		pollOptions.revents = 0;
		poll(&pollOptions, 1, 200);

		testchar = key;
		printf("testchar : %c\n", testchar);

		// write the data
		ssize_t result = writeBytes(MySettings.fd, &testchar, 1);
		if (result != 1) {
			if (errno != EWOULDBLOCK) {
				char errorString[0xFF];
				strerror_r(errno, errorString, sizeof(errorString));
				fprintf(stderr, "Error writing data (%s). Exiting...\n", errorString);
				break;
			}
		}
		txcount++;
		
		if (testchar == 'q')
		{
			printf("system quit.");

		}
		
		if (key > 'Z') {
			// wrap around
			key = 'A';
		}
		//usleep(50);

		// print status
		if (txcount % 100 == 0) {
			fprintf(stderr, "tx: %ld\n", txcount);
		}
	}
}

// void *t_function(void *data)
// {
//     int id;
//     int i = 0;
//     id = *((int *)data);

//     //from uart
//     while(1)
//     {
//         /* ---- RX mode ---- */
// 		if (MySettings.sermode == RX) {
// 			rx_function();
// 		}
//         /* ---- TX mode ---- */
// 		else{
// 			tx_function();
// 		}
// 	}
// }


// --> add_function

/*************************************************************/
/* Main function                                             */
/*************************************************************/
int main(int argc, char *argv[])
{
	// Settings_t MySettings;
	// long rxcount = 0;
	// long txcount = 0;

	// <-- add local value
	int key;
	pthread_t p_thread;
    int thr_id;
    int status;
    int rx = 1;
    int tx = 2;
	// --> local value
	// thr_id = pthread_create(&p_thread[0], NULL, t_function, (void *)&a);
    // if (thr_id < 0)
    // {
    //     perror("thread create error : ");
    //     exit(0);
    // }
    //for uart 
    // if ( -1 == ( fd_from_uart = open( FIFO_FROM_UART, O_RDWR)))
    // {
    //     if ( -1 == mkfifo( FIFO_FROM_UART, 0666))
    //     {
    //         perror( "mkfifo() run error");
    //         exit( 1);
    //     }

    //     if ( -1 == ( fd_from_uart = open( FIFO_FROM_UART, O_RDWR)))
    //     {
    //         perror( "open() error");
    //         exit( 1);
    //     }
    // }
    // if ( -1 == ( fd_to_uart = open( FIFO_TO_UART, O_RDWR)))
    // {
    //     if ( -1 == mkfifo( FIFO_TO_UART, 0666))
    //     {
    //         perror( "mkfifo() run error");
    //         exit( 1);
    //     }

    //     if ( -1 == ( fd_to_uart = open( FIFO_TO_UART, O_RDWR)))
    //     {
    //         perror( "open() error");
    //         exit( 1);
    //     }
    // }
	// --> add 

	/* parse the command line arguments */
	parse_args(argc, argv, &MySettings);

	/* open the port */
	if (openport(&MySettings) != 0) {
		return -1;
	}
	

	// --> add thread
	if (MySettings.sermode == RX) {
		thr_id = pthread_create(&p_thread, NULL, rx_function, (void *)&rx);
		if (thr_id < 0)
		{
			perror("thread create error : ");
			exit(0);
		}
		printf("flag = %d\n", MySettings.sermode);
	}

	if (MySettings.sermode == TX) {
		thr_id = pthread_create(&p_thread, NULL, tx_function, (void *)&tx);
		if (thr_id < 0)
		{
			perror("thread create error : ");
			exit(0);
		}
		printf("flag = %d\n", MySettings.sermode);
	}

	pthread_join(p_thread, (void*)&rx);
	pthread_join(p_thread, (void*)&tx);

	// /* ---- RX mode ---- */
	// if (MySettings.sermode == RX) {
	// 	rx_function();
	// }
	// /* ---- TX mode ---- */
	// else{
	// 	tx_function();
	// }
	// <-- add thread

	// /* ---- RX mode ---- */
	// if (MySettings.sermode == RX) {

	// 	// Reset the serial device file descriptor for blocking read / write
	// 	fcntl(MySettings.fd, F_SETFL, 0);

	// 	if (configureport(&MySettings) != 0) {
	// 		return -1;
	// 	}

	// 	// Begin reading
	// 	char testchar;
	// 	char expected_char = 'A';

	// 	fprintf(stderr, "Waiting for data...\n");

	// 	while (MySettings.fd > 0) {
	// 		// Wait until there is data to read or a timeout happens
	// 		struct pollfd pollOptions;
	// 		pollOptions.fd = MySettings.fd;
	// 		pollOptions.events = POLLIN;
	// 		pollOptions.revents = 0;
	// 		poll(&pollOptions, 1, 200);
	// 		ssize_t result = readBytes(MySettings.fd, &testchar, 1);
	// 		if (result != 1) {
	// 			if (errno != EWOULDBLOCK) {
	// 				char errorString[0xFF];
	// 				strerror_r(errno, errorString, sizeof(errorString));
	// 				fprintf(stderr, "Error reading data (%s). Exiting...\n", errorString);
	// 				break;
	// 			}
	// 		}
	// 		rxcount++;
	// 		printf("command recived : %c\n", testchar);

	// 		// print status
	// 		if (rxcount % 100 == 0) {
	// 			fprintf(stderr, "rx: %ld\n", rxcount);
	// 		}
	// 	}
	// }

	// /* ---- TX mode ---- */
	// if (MySettings.sermode == TX) {

	// 	// Reset the serial device file descriptor for non-blocking read / write
	// 	fcntl(MySettings.fd, F_SETFL, O_NONBLOCK);

	// 	if (configureport(&MySettings) != 0) {
	// 		return -1;
	// 	}

	// 	// Begin writing
	// 	char testchar;
	// 	while ((key=main_menu()) != 0) {
	// 		// Wait until port is writeable
	// 		struct pollfd pollOptions;
	// 		pollOptions.fd = MySettings.fd;
	// 		pollOptions.events = POLLOUT;
	// 		pollOptions.revents = 0;
	// 		poll(&pollOptions, 1, 200);

	// 		testchar = key;
	// 		printf("testchar : %c\n", testchar);

	// 		// write the data
	// 		ssize_t result = writeBytes(MySettings.fd, &testchar, 1);
	// 		if (result != 1) {
	// 			if (errno != EWOULDBLOCK) {
	// 				char errorString[0xFF];
	// 				strerror_r(errno, errorString, sizeof(errorString));
	// 				fprintf(stderr, "Error writing data (%s). Exiting...\n", errorString);
	// 				break;
	// 			}
	// 		}
	// 		txcount++;
			

	// 		if (key > 'Z') {
	// 			// wrap around
	// 			key = 'A';
	// 		}
	// 		//usleep(50);

	// 		// print status
	// 		if (txcount % 100 == 0) {
	// 			fprintf(stderr, "tx: %ld\n", txcount);
	// 		}
	// 	}
	// }

	fprintf(stderr, "ERROR- you must select a mode with -r or -t\n");
	return 0;
}
