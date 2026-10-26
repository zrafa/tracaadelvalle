
/*
 * leer giro de ruedas .c : obtiene datos crudos del giro de ruedas con una marca de tiempo
 *              en microsegundos y milisegundos por cada dato
 *              Al terminar (ej: con control+c cierra correctamente el device)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/timeb.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* PUERTO SERIAL de ublox (verificar con dmesg o ls /dev/ */
#define serial_dev "/dev/ttyUSB0"

int serial_port;
struct termios tty;
struct sigaction old_action;

void sigint_handler(int sig_no)
{
    printf("finalizamos..\n");
    close(serial_port);
    sigaction(SIGINT, &old_action, NULL);
    kill(0, SIGINT);
}


long long int microsegundos()
{
	struct timeval timer_usec;
	long long int timestamp_usec;
	if (!gettimeofday(&timer_usec, NULL)) {
		timestamp_usec = ((long long int) timer_usec.tv_sec) * 
			1000000ll + (long long int) timer_usec.tv_usec;
  	} else
		 timestamp_usec = -1;

	return timestamp_usec;
}


void serial_init(void)
{
	/* Read in existing settings, and handle any error */
	if(tcgetattr(serial_port, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		exit (1);
	}

	tty.c_cflag &= ~PARENB; /* Clear parity bit */
	tty.c_cflag &= ~CSTOPB; /* Clear stop field, only one stop bit */
	tty.c_cflag &= ~CSIZE; /* Clear all bits that set the data size */ 
	tty.c_cflag |= CS8; /* 8 bits per byte */
	tty.c_cflag &= ~CRTSCTS; /* Disable RTS/CTS */
	tty.c_cflag |= CREAD | CLOCAL; /* Turn on READ & ignore ctrl */

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; /* Disable echo */
	tty.c_lflag &= ~ECHOE; /* Disable erasure */
	tty.c_lflag &= ~ECHONL; /* Disable new-line echo */
	tty.c_lflag &= ~ISIG; /* Disable interpretation of INTR, etc */
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* off s/w flow ctrl */
	/* Disable any special handling of received bytes */
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 

	tty.c_oflag &= ~OPOST; /* no interpretation of output bytes */
	tty.c_oflag &= ~ONLCR; /* no conversion of newline */

	tty.c_cc[VTIME] = 10;    /* Wait for up to 1s */
	tty.c_cc[VMIN] = 0;

	/* baud rate to be 9600 */
	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);

	/* Save tty settings, also checking for error */
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, 
			strerror(errno));
		exit(1);
	}

}

int main() 
{

	/* para control C o SIGTERM (kill) */
	struct sigaction action;
	memset(&action, 0, sizeof(action));
	action.sa_handler = &sigint_handler;
	sigaction(SIGINT, &action, &old_action);
	sigaction(SIGTERM, &action, NULL);


	serial_port = open(serial_dev, O_RDWR);
	serial_init();

	char read_buf [1024];
	int num_bytes;
	char *t;
	int c;

	while (1) {
		memset(&read_buf, '\0', sizeof(read_buf));
		num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
  		if (num_bytes < 0) {
      			printf("Error reading: %s", strerror(errno));
  			close(serial_port);
			exit(0);
		}
			
		t = read_buf;
		c = 0;
		while (c < num_bytes) {
			if ((*t) == '\n') {
				printf(" %lld %lld ", microsegundos(), microsegundos()/1000);
				fflush(0);
			}
			printf("%c",*t);
			c++;
			t++;
		}
	}


	close(serial_port);
	return 0;
}

