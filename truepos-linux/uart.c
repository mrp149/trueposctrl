// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#ifdef TEST
#define UART_NAME  "/dev/ttyUSB0"
#else
#define UART_NAME  "/dev/ttyS0"
#endif

static int serial_port =  -1;
static struct termios tty;

int uart_init() {

  // close the port if it is already open
  if(serial_port != -1) {
      printf("Error the port is already opened %i. Closing the port\n", serial_port);
      close(serial_port);
      serial_port = -1;
      return 1;
  }

  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  serial_port = open(UART_NAME, O_RDWR);
  if(serial_port == -1) {
      printf("Error on the port opening: %i: %s\n", errno, strerror(errno));
      return 1;
  }

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      close(serial_port);
      serial_port = -1;
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
#ifdef TEST
  cfsetispeed(&tty, B4800);
  cfsetospeed(&tty, B4800);
#else
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);
#endif

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      close(serial_port);
      serial_port = -1;
      return 1;
  }
  return 0;
}


// Stop setrial port
int uart_stop() {

  if (serial_port != -1) {
      close(serial_port);
      serial_port = -1;
      return 1;
  }
  return 0;
}


// Write to serial port
int uart_tx(char * msg, size_t ln) {

  if (serial_port != -1)
      return write(serial_port, msg, ln);
  return 0;
}


// Read from setrial port
int uart_rx() {
  char ch;

  if (serial_port == -1)
      return EOF;
  if(read(serial_port, &ch, sizeof(ch)) != sizeof(ch))
      return EOF;
  return ch;
}


#ifdef TEST_UART

int main(){
  char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };

  uart_init();

  uart_tx(msg, sizeof(msg));

  while(1){
     printf("%c", uart_rx());
  }

  uart_stop();

  return 0; // success
}

#endif
