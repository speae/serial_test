/*******************************************************/
/* Program to test serial port throughput              */
/*                                                     */
/*******************************************************/

// <-- move to uart_check.hpp file
#include "uart_ex.hpp"
// --> move to uart_check.hpp file

// <-- Seperate header file and main cpp file
// #include <stdlib.h>
// #include <string.h>
// #include <inttypes.h>
// #include <time.h>
// #include <fcntl.h> // Required for open()
// #include <unistd.h> // Required for read(), write(), close()
// #include <termios.h> // Required for tcgetattr(), cfsetispeed(), cfsetospeed(), tcflush()
// #include <errno.h> // Require for errno
// #include <unistd.h> // Required for error codes, among other things
// #include <poll.h> // Required for poll()
// #include <sys/types.h>
// #include <sys/stat.h>

// #include <iostream>
// #include <thread>
// #include <vector>
// #include <cstdio>

// #define VERSION_MAJOR 1
// #define VERSION_MINOR 0

// #define DEVLENGTH 128
// #define BAUDLENGTH 32

// // <-- add constant
// #define NONE_FIFO 0
// #define USE_FIFO 0

// #define RX 1
// #define TX 2
// #define BUFF_SIZE 1024

// using namespace std;

// struct Settings_t{
// 	int verbose;			/* enable verbose output */
// 	char devpath[DEVLENGTH];	/* -d serial device */
// 	int baudrate;			/* baudrate to use */
// 	int sermode;			/* serial mode RX or TX */
// 	int fd;				/* file descriptor */
// };
// Settings_t MySettings;
// // // --> add constant

// class Uart_remote{
// 	private:	
// 		#if USE_FIFO == 1
// 		#define UART_FIFO
// 			struct fifo_file
// 			{
// 				string FROM_UART;
// 				string TO_UART;
// 			};
// 		#else
// 			const char* optString = "vd:b:rt";
// 			char buff[BUFF_SIZE];
// 			int counter = 0;
// 			int fd_from_uart;
// 			int fd_to_uart;
// 			int handle;
// 			int port_check;
// 			int port_cfg_chk;

// 			string TitleMessage  = "Welcome Serial Port\r\n";
// 			char Buff[256];
// 			int RxCount;
// 			int loop;
// 			int ending;   

// 		#endif
		
// 	public:
// 		long rxcount = 0;
// 		long txcount = 0;
		
// 		#if USE_FIFO == 1
// 		#define UART_FIFO
// 			void set_fifo_protocol(struct fifo_file* fifo);
// 		#else
			

// 			// <-- add global value
// 			struct Uart_control
// 			{
// 				int key;
// 				int thr_id;
// 				int status;
// 				int rx = 1;
// 				int tx = 2;
// 			};
// 			Uart_control uart_c;

// 			void set_openport(Settings_t*);
// 			int get_openport();
// 			void set_configureport(Settings_t*);
// 			int get_configureport();
// 			void parse_args(int, char**, Settings_t*);
			
// 			/*************************************************************/
// 			/* readBytes() - reads bytes from the selected serial port   */
// 			/*************************************************************/
// 			ssize_t readBytes(int deviceFileDescriptor, void* destination, ssize_t size)
// 			{
// 				ssize_t bytesRead = 0;
// 				while (bytesRead < (ssize_t)size) {
// 					ssize_t result = read(deviceFileDescriptor, destination + bytesRead, size - bytesRead);
// 					if (result <= 0 && errno != EWOULDBLOCK) {
// 						char errorString[0xFF];
// 						strerror_r(errno, errorString, sizeof(errorString));
// 						std::fprintf(stderr, "Failed to read from device descriptor. Error: %s\n", errorString);
// 						return result;
// 					}
// 					if (result <= 0 && errno != EAGAIN) {
// 						char errorString[0xFF];
// 						strerror_r(errno, errorString, sizeof(errorString));
// 						std::fprintf(stderr, "Error: %s\n", errorString);
// 						return result;
// 					}
// 					bytesRead += result;
// 				}
// 				return bytesRead;
// 			}

// 			/*************************************************************/
// 			/* writeBytes() - writes bytes to the selected serial port   */
// 			/*************************************************************/
// 			ssize_t writeBytes(int deviceFileDescriptor, const void* source, ssize_t size) {
// 				ssize_t bytesWritten = 0;
// 				while (bytesWritten < size) {
// 					ssize_t result = write(deviceFileDescriptor, source + bytesWritten, size - bytesWritten);
// 					if (result <= 0 && errno != EWOULDBLOCK) {
// 						char errorString[0xFF];
// 						strerror_r(errno, errorString, sizeof(errorString));
// 						std::fprintf(stderr, "Failed to write to serial device descriptor. Error: %s\n", errorString);
// 						return result;
// 					}
// 					bytesWritten += result;
// 				}

// 				return bytesWritten;
// 			}

// 			int getch(void)
// 			{
// 				struct termios oldattr, newattr;
// 				int ch;
// 				tcgetattr( STDIN_FILENO, &oldattr );
// 				newattr = oldattr;
// 				newattr.c_lflag &= ~( ICANON | ECHO );
// 				// TCSANOW :즉시 변경사항 반영 
// 				tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
// 				ch = getchar();
// 				cout << "ch : " << ch << endl;
// 				tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
// 				return ch;
// 			}

// 			int main_menu(void)
// 			{
// 				int key;

// 				cout << endl << endl;
// 				cout << "-------------------------------------------------" << endl;
// 				cout << "                    MAIN MENU                    " << endl;
// 				cout << "-------------------------------------------------" << endl;

// 				cout << " a. Turn Left                                    " << endl;
// 				cout << " b. Turn Right                                   " << endl;
// 				cout << " c. Forward                                      " << endl;
// 				cout << " d. backward                                     " << endl;
// 				cout << " i. stop                                         " << endl;

// 				cout << "-------------------------------------------------" << endl;
// 				cout << " q. Motor Control application QUIT               " << endl;
// 				cout << "-------------------------------------------------" << endl;
// 				cout << endl << endl;
			
// 				cout << "SELECT THE COMMAND NUMBER : " << endl;

// 				key = getch();
// 				cout << "key : " << key << endl;

// 				return key;
// 			}

// 		#endif
// };

// //--> add global value

// /*************************************************************/
// /* parse_args() - Parse command line arguments and set       */
// /* values as necessary.                                      */
// /* parameters: argc = argument count, argv = argument values */
// /*************************************************************/
// void Uart_remote::parse_args(int argc, char** argv, Settings_t* settings)
// {
// 	int opt = 0;

// 	/* set default device path */
// 	//strncpy(settings->devpath, "/dev/ttyTHS2", DEVLENGTH);
// 	strncpy(settings->devpath, "/dev/ttyTHS1", DEVLENGTH);
// 	/* set default baud rate */
// 	settings->baudrate = 115200;
// 	opt = getopt( argc, argv, optString );
// 	cout << "opt : " << opt << endl;
// 	while( opt != -1 ) {
// 		switch( opt ) {
// 			case 'v':
// 				// turn on verbose mode
// 				settings->verbose = 1;
// 				break;
// 			case 'd':
// 				// sets the device path
// 				strncpy(settings->devpath, argv[2], DEVLENGTH);
// 				break;
// 			case 'b':
// 				// sets the baud rate
// 				settings->baudrate = atoi(argv[2]);
// 				break;
// 			case 'r':
// 				// sets to serial receive mode
// 				settings->sermode = RX;
// 				break;
// 			case 't':
// 				// sets to serial transmit mode
// 				settings->sermode = TX;
// 				break;
// 			case 'h':
// 			case '?':
// 				cout << "sertest version " << VERSION_MAJOR << VERSION_MINOR << endl;
// 				cout << "usage: ./" << argv[0] << "[-v] [-d device] [-b baud] -t|-r" << endl;
// 				cout << "  -v enables verbose mode" << endl;
// 				cout << "  -d <devicename> sets the serial device" << endl;
// 				cout << "  -b <baud> sets the baud rate" << endl;
// 				cout << "  -t sets mode to transmit" << endl;
// 				cout << "  -r sets mode to receive" << endl;
// 				exit( EXIT_FAILURE );
// 				break;
// 			default:
// 				/* You won't actually get here. */
// 				break;
// 		}
// 		opt = getopt( argc, argv, optString );
// 	}

// 	if (settings->verbose == 1) {
// 		cout << "Arguments:\n";
// 		cout << "  -v: " << settings->verbose << endl;
// 		cout << "  -d: " << settings->devpath << endl;
// 		cout << "  -b: " << settings->baudrate << endl;
// 		cout << "  -r/t: " << settings->sermode << endl;
// 	}
// }

// /*************************************************************/
// /* openport() - opens the selected serial port and gets a    */
// /* file descriptor                                           */
// /*************************************************************/
// void Uart_remote::set_openport(Settings_t *settings)
// {
//     settings->fd = open(settings->devpath, O_RDWR | O_NOCTTY | O_NDELAY);
//     if (settings->fd <= 0) {
//         char errorString[0xFF];
//         strerror_r(errno, errorString, sizeof(errorString));
//         std::fprintf(stderr, "Open device failed: Unable to open device file %s. Error: %s\n", settings->devpath, errorString);
//         port_check = -1;
//     } else {
// 	if (settings->verbose) {
// 		std::fprintf(stderr, "Got file descriptor: %d\n", settings->fd);
// 	}
//     }

//     // Reset the serial device file descriptor for non-blocking read / write
//     fcntl(settings->fd, F_SETFL, 0);

//     port_check = 0;
// }

// int Uart_remote::get_openport()
// {
//     return port_check;
// }

// /*************************************************************/
// /* configureport() - configures the selected serial port     */
// /*************************************************************/
// void Uart_remote::set_configureport(Settings_t *settings)
// {
//     // Modify the settings on the serial device (baud rate, 8n1, receiver enabled, ignore modem status, no flow control) and apply them
//     struct termios deviceOptions;
//     memset (&deviceOptions, 0, sizeof deviceOptions);
//     tcgetattr(settings->fd, &deviceOptions);

//     /* Set the baud rates... */
//     speed_t MyBaud;
//     switch (settings->baudrate) {
// 	case 9600:
// 		MyBaud = B9600;
// 		break;
// 	case 57600:
// 		MyBaud = B57600;
// 		break;
// 	case 115200:
// 		MyBaud = B115200;
// 		break;
// 	case 230400:
// 		MyBaud = B230400;
// 		break;
// 	case 460800:
// 		MyBaud = B460800;
// 		break;
// 	case 921600:
// 		MyBaud = B921600;
// 		break;
// 	case 1000000:
// 		MyBaud = B1000000;
// 		break;
// 	case 2000000:
// 		MyBaud = B2000000;
// 		break;
// 	case 3000000:
// 		MyBaud = B3000000;
// 		break;
// 	default:
// 		MyBaud = B9600;
//     }

//     //cfsetispeed(&deviceOptions, settings->baudrate);
//     //cfsetospeed(&deviceOptions, settings->baudrate);
//     cfsetispeed(&deviceOptions, MyBaud);
//     cfsetospeed(&deviceOptions, MyBaud);

//     // Set character length
//     deviceOptions.c_cflag &= ~CSIZE;       // Mask the character size bits
//     deviceOptions.c_cflag |= CS8;

//     // No parity
//     deviceOptions.c_cflag &= ~PARENB;
//     deviceOptions.c_cflag |= PARODD;

//     // Stick parity flag
//     #ifdef __USE_MISC
//     	deviceOptions.c_cflag |= CMSPAR;
// 	#endif

//     // ignores parity errors and passes bytes regardless
//     deviceOptions.c_iflag |= (IGNPAR);

//     //causes parity errors to be 'marked' in the input stream using special characters.
//     //If IGNPAR is enabled, a NUL character (0x00) is sent to your program before every character with a parity error.
//     //Otherwise, a DEL (0x7F) and NUL character is sent along with the bad character.
//     deviceOptions.c_iflag |= (PARMRK);

//     //enable checking and stripping of the parity bit
//     //NOTE: PARMRK and INPCK are exclusive of each other
//     //options.c_iflag |= (INPCK);
//     //options.c_iflag |= (ISTRIP);

//     // One stop bit
//     deviceOptions.c_cflag &= ~CSTOPB;

//     /* No hardware flow control */
// 	#ifdef __USE_MISC
//     	deviceOptions.c_cflag &= ~CRTSCTS;
// 	#endif

//     // Raw input, no echo
//     deviceOptions.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | IEXTEN | ISIG);
//     deviceOptions.c_iflag &=  ~(IGNBRK | BRKINT | PARMRK | ISTRIP
//                 | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | INPCK );

//     // Raw output
//     deviceOptions.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
//                 ONOCR | OFILL | OLCUC | OPOST);

//     // Wait timeout for each character
//     deviceOptions.c_cc[VMIN] = 1;
//     deviceOptions.c_cc[VTIME] = 0;

//     /* Enable the receiver and set local mode... */
//     deviceOptions.c_cflag |= (CLOCAL | CREAD);

//     tcflush(settings->fd, TCIFLUSH);
//     if (tcsetattr(settings->fd, TCSANOW, &deviceOptions) != 0) {
//         close(settings->fd);
//         char errorString[0xFF];
//         strerror_r(errno, errorString, sizeof(errorString));
//         std::fprintf(stderr, "Create failed: Unable to set options on device. Error: %s\n", errorString);
//         port_cfg_chk = -1;
//     }
//     if (settings->verbose == 1) {
//     	cout << "Configured serial device file descriptor for 8n1 and no flow control: " << settings->devpath << endl;
//     }
//     port_cfg_chk = 0;
// }

// int Uart_remote::get_configureport(){
// 	return port_cfg_chk;
// }
// --> Seperate header file and main cpp file

void rx_function(Settings_t* settings)
{
	Uart_remote u_remote;

	// Reset the serial device file descriptor for blocking read / write
	fcntl(settings->fd, F_SETFL, 0);

	u_remote.set_configureport(settings);
	if (u_remote.get_configureport() != 0) {
		exit(-1);
	}
	// if (configureport(&MySettings)) != 0) {
	// 	exit(-1);
	// }

	// Begin reading
	char testchar;
	char testBuffer[256];
	//vector<char> drive_string;
	//vector<char>::iterator driver_cmd;

	char expected_char = 'A';

	std::fprintf(stderr, "Waiting for data...\n");

	while (settings->fd > 0) {
		// poll() : linux multiple IO
		// Wait until there is data to read or a timeout happens
		// struct pollfd pollOptions;
		// pollOptions.fd = MySettings.fd;
		// // read is not blocked
		// pollOptions.events = POLLIN;
		// pollOptions.revents = 0;
		// poll(&pollOptions, 1, 200);
		ssize_t result = u_remote.readBytes(settings->fd, &testchar, 1);
		if (result != 1) {
			if (errno != EWOULDBLOCK) {
				char errorString[0xFF];
				strerror_r(errno, errorString, sizeof(errorString));
				fprintf(stderr, "Error reading data (%s). Exiting...\n", errorString);
				break;
			}
		}
		u_remote.rxcount++;
		// sprintf(testBuffer, "%c\n", testchar);
		// printf("command recived : %s\n", testBuffer);
		//drive_string.push_back(testchar);
		printf("command recived : %c\n", testchar);
		if (testchar == 'q')
		{
			printf("system quit.\n");
			exit(-1);
		}
		
		// print status
		// for (driver_cmd = drive_string.begin(); driver_cmd != drive_string.end(); driver_cmd++){
		// 	cout << "received message : " << *driver_cmd << endl;
		// }
		if (u_remote.rxcount % 100 == 0) {
			fprintf(stderr, "rx: %ld\n", u_remote.rxcount);
		}
	}
}

void tx_function(Settings_t* settings)
{
	Uart_remote u_remote;

	// Reset the serial device file descriptor for non-blocking read / write
	fcntl(settings->fd, F_SETFL, O_NONBLOCK);

	u_remote.set_configureport(settings);
	if (u_remote.get_configureport() != 0) {
		exit(-1);
	}

	// Begin writing
	char testchar;
	while ((u_remote.uart_c.key=u_remote.main_menu()) != 0) {
		// Wait until port is writeable
		// struct pollfd pollOptions;
		// pollOptions.fd = MySettings.fd;
		// // write is not blocked
		// pollOptions.events = POLLOUT;
		// pollOptions.revents = 0;
		// poll(&pollOptions, 1, 200);

		testchar = u_remote.uart_c.key;
		printf("testchar : %c\n", testchar);

		// write the data
		ssize_t result = u_remote.writeBytes(settings->fd, &testchar, 1);
		if (result != 1) {
			if (errno != EWOULDBLOCK) {
				char errorString[0xFF];
				strerror_r(errno, errorString, sizeof(errorString));
				fprintf(stderr, "Error writing data (%s). Exiting...\n", errorString);
				break;
			}
		}
		u_remote.txcount++;
		
		if (testchar == 'q')
		{
			printf("system quit.\n");
			exit(-1);
		}
		
		if (u_remote.uart_c.key > 'Z') {
			// wrap around
			u_remote.uart_c.key = 'A';
		}
		//usleep(50);

		// print status
		if (u_remote.txcount % 100 == 0) {
			fprintf(stderr, "tx: %ld\n", u_remote.txcount);
		}
	}
}

// --> add function
#if USE_FIFO == 1
void Uart_remote::get_fifo_protocol(struct fifo_file* fifo){
    fifo->FROM_UART = "/tmp/from_uart_fifo";
    fifo->TO_UART = "/tmp/to_uart_fifo";
}
#endif

// --> add_function

/*************************************************************/
/* Main function                                             */
/*************************************************************/
int main(int argc, char *argv[])
{
	// <-- add local value
	thread rx_t;
    thread tx_t;
	Uart_remote u_remote;
	Settings_t settings;

	/* parse the command line arguments */
	u_remote.parse_args(argc, argv, &settings);
	
	// /* open the port */
	u_remote.set_openport(&settings);
	if (u_remote.get_openport() != 0) {
		return -1;
	}
	// if (openport(&MySettings) != 0) {
	// 	return -1;
	// }
	

	// --> add thread
	if (settings.sermode == RX) {
		thread rx_t(rx_function, &settings);
		cout<< "flag = " << settings.sermode << endl;
		rx_t.join();	
	}

	if (settings.sermode == TX) {
		thread tx_t(tx_function, &settings);
		cout << "flag = " << settings.sermode << endl;
		tx_t.join();
	}
	
	fprintf(stderr, "ERROR- you must select a mode with -r or -t\n");
	return 0;
}
