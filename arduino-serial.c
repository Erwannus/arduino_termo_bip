
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>

/*
 * Arduino Serial Lib Demo
 *  
 * 
 */


//__________________________________________________________________________________
// arduino-serial-lib -- simple library for reading/writing serial ports
//
// 2006-2013, Tod E. Kurt, http://todbot.com/blog/
//


#include <stdio.h>    // Standard input/output definitions 
#include <unistd.h>   // UNIX standard function definitions 
#include <fcntl.h>    // File control definitions 
#include <errno.h>    // Error number definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <string.h>   // String function definitions 
#include <sys/ioctl.h>
#include <stdint.h> // Standard types 

// uncomment this to debug reads
//#define SERIALPORTDEBUG 

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NONBLOCK );
    
    if (fd == -1)  {
        perror("serialport_init: Unable to open port ");
        return -1;
    }
    
    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;
    
    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

//______________________________________________________________________
int serialport_close( int fd )
{
    return close( fd );
}

//______________________________________________________________________
int serialport_writebyte( int fd, uint8_t b)
{
    int n = write(fd,&b,1);
    if( n!=1)
        return -1;
    return 0;
}

//______________________________________________________________________
int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
    if( n!=len ) {
        perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0;
}

//________________________________________________________________________________
int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            if( timeout==0 ) return -2;
            continue;
        }
#ifdef SERIALPORTDEBUG  
        printf("serialport_read_until: i=%d, n=%d b='%c'\n",i,n,b[0]); // debug
#endif
        buf[i] = b[0]; 
        i++;
    } while( b[0] != until && i < buf_max && timeout>0 );

    buf[i] = 0;  // null terminate the string
    return 0;
}

//___________________________________________________________________________
int serialport_flush(int fd)
{
    sleep(2); //required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}


//____________________________________________________________________________
//			PROGRAMME PRINCIPAL
// le fait d'inclure les fonctions de la bibliothèque directement dans le programme
// permet de s'affranchir de la rajouter à l'édition des liens
int main(int argc, char **argv)
{
    //############
    // int socket_serv, err;
    // struct sockaddr_in addr;
    // socket_serv = socket(AF_INET,SOCK_DGRAM,0);
    // if (socket_serv<0) { exit(1); }
    // addr.sin_family = AF_INET;
    // addr.sin_port = htons(9000);
    // addr.sin_addr.s_addr = htonl(INADDR_ANY);
    // err = bind(socket_serv, (struct sockaddr *) &addr, sizeof(struct sockaddr_in));
    // if (err<0) { exit(1); }

    // char msg[64];
    // socklen_t len, flen;
    // struct sockaddr_in from;
    // flen = sizeof(struct sockaddr_in);
    // printf("En attente de connexion");
    // fflush(stdout);
    // len = recvfrom(socket_serv, msg, sizeof(msg), 0, (struct sockaddr*) &from, &flen);
    // if (len<0) { exit(1); }
    // printf(" %s", msg);




	char buffer[100];					// un buffer
	int i;
	
	// ouverture du port à 115200 bauds
	int fd = serialport_init("/dev/ttyACM0", 115200);
	if (fd==-1) return -1;
	
	// boucle
	for ( ; ; ){
		//	lecture d'une ligne
		serialport_read_until(fd, buffer, '\r', 99, 10000);

		// suppression de la fin de ligne
		for (i=0 ; buffer[i]!='\r' && i<100 ; i++);
		buffer[i] = 0;
	
		// écriture du résultat
		printf("%s", buffer);


        FILE *file = fopen("index.txt", "w"); 
 
        // Check if the file was successfully opened 
        if (file == NULL) { 
            printf("Error opening file!\n"); 
            return 1; 
        } 
    
        // Write to the file 
        fprintf(file, "%s", buffer); 
    
        // Close the file 
        fclose(file); 
    
	}
	
	// fermeture du port
	serialport_flush(fd);
	serialport_close(fd);
		
	return 0;
}

