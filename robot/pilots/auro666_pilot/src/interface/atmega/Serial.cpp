#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <stdlib.h>
#include <interface/atmega/Serial.hpp>

Tserial::Tserial() {
}

Tserial::~Tserial() {
}

int Tserial::connect() {
    return connect("//dev//ttyUSB0");
}

int Tserial::connect(char *device, int rate_arg, serial_parity parity_arg, bool verbose) {
    v = verbose;
    struct termios terminalAttributes;
    fd_serialport = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_FSYNC);
    if (fd_serialport < 0) {
        perror(device);
        exit(-1);
    }
    long DATABITS;
    long STOPBITS = 1;
    long PARITYON;
    long PARITY;
    long BAUD;
    int Data_Bits = 8;
    int Stop_Bits = 1;

    //memset(&terminalAttributes, 0, sizeof(struct termios));// clear terminalAttributes data
    tcgetattr(fd_serialport, &terminalAttributes);
    terminalAttributes.c_cflag |= (CLOCAL | CREAD);
    switch (parity_arg) {
        case 0:
        default: //none
            terminalAttributes.c_cflag &= ~PARENB; // Clear parity enable
            break;
        case 1:
            terminalAttributes.c_cflag |= PARENB; // Parity enable
            terminalAttributes.c_cflag |= PARODD; // Enable odd parity
            break;
        case 2:
            terminalAttributes.c_cflag |= PARENB; // Parity enable
            terminalAttributes.c_cflag &= ~PARODD; // Turn off odd parity = even
    } //end of switch parity
    switch (Data_Bits) {
        case 7:
            terminalAttributes.c_cflag |= CS7;
            break;
        case 8:
        default:
            terminalAttributes.c_cflag |= CS8;
            break;
    } //end of switch data_bits
    int n;
    switch (rate_arg) {
        case 1200:
            n = cfsetispeed(&terminalAttributes, B1200);
            n += cfsetospeed(&terminalAttributes, B1200);
            break;
        case 2400:
            n = cfsetispeed(&terminalAttributes, B2400);
            n += cfsetospeed(&terminalAttributes, B2400);
            break;
        case 4800:
            n = cfsetispeed(&terminalAttributes, B4800);
            n += cfsetospeed(&terminalAttributes, B4800);
            break;
        case 9600:
            n = cfsetispeed(&terminalAttributes, B9600);
            n += cfsetospeed(&terminalAttributes, B9600);
            break;
        case 19200:
        default:
            n = cfsetispeed(&terminalAttributes, B19200);
            n += cfsetospeed(&terminalAttributes, B19200);
            break;
        case 38400:
            n = cfsetispeed(&terminalAttributes, B38400);
            n += cfsetospeed(&terminalAttributes, B38400);
            break;
        case 57600: // witilt
            n = cfsetispeed(&terminalAttributes, B57600);
            n += cfsetospeed(&terminalAttributes, B57600);
            break;
    }
    switch (Stop_Bits) {// Set stop bits
        case 1:
        default:
            terminalAttributes.c_cflag &= ~CSTOPB; // Not 2 stop bits = One stop bit
            break;
        case 2:
            terminalAttributes.c_cflag |= CSTOPB; // Two stop bits
            break;
    } //end of switch stop bits

    //terminalAttributes.c_cflag=BAUD | DATABITS |  STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;

    //terminalAttributes.c_iflag = IGNPAR |  ONLCR;
    //terminalAttributes.c_oflag = OPOST;
    // terminalAttributes.c_cc[VTIME] = 0;
    //terminalAttributes.c_cc[VMIN] = 1;
    // Non blocking return immediately with data
    /*
    terminalAttributes.c_cc[VMIN] = 0;
    terminalAttributes.c_cc[VTIME] = 0;

    // Local flags
    terminalAttributes.c_lflag = 0;  // No local flags
    terminalAttributes.c_lflag &= ~ICANON; // Don't canonicalise
    terminalAttributes.c_lflag &= ~ECHO; // Don't echo
    terminalAttributes.c_lflag &= ~ECHOK; // Don't echo

    // Control flags
    terminalAttributes.c_cflag &= ~CRTSCTS; // Disable RTS/CTS
    terminalAttributes.c_cflag |= CLOCAL; // Ignore status lines
    terminalAttributes.c_cflag |= CREAD; // Enable receiver
    terminalAttributes.c_cflag |= HUPCL; // Drop DTR on close

    // oflag - output processing
    options.c_oflag &= ~OPOST; // No output processing
    options.c_oflag &= ~ONLCR; // Don't convert linefeeds

    // iflag - input processing
    terminalAttributes.c_iflag |= IGNPAR; // Ignore parity
    terminalAttributes.c_iflag &= ~ISTRIP; // Don't strip high order bit
    terminalAttributes.c_iflag |= IGNBRK; // Ignore break conditions
    terminalAttributes.c_iflag &= ~INLCR; // Don't Map NL to CR
    terminalAttributes.c_iflag &= ~ICRNL; // Don't Map CR to NL
    terminalAttributes.c_iflag |= (IXON | IXOFF | IXANY); // xon/xoff flow control
     */
    tcsetattr(fd_serialport, TCSANOW, &terminalAttributes);
    fcntl(fd_serialport, F_SETFL, FNDELAY);

    //tcflush(fd_serialport, TCOFLUSH);
    tcflush(fd_serialport, TCIFLUSH); // Clear the line

    return fd_serialport;
}

int Tserial::connect(char *device) {
    struct termios terminalAttributes;

    /*
     * http://linux.die.net/man/2/open
     *
     * Open the serial port
     * read/write
     * not become the process's controlling terminal
     * When possible, the file is opened in nonblocking mode
     *
     */
    fd_serialport = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_FSYNC);

    // clear terminalAttributes data
    memset(&terminalAttributes, 0, sizeof (struct termios));

    /*    http://linux.die.net/man/3/termios
     *
     *  control modes: c_cflag flag constants:
     *
     * 57600 bauds
     * 8 bits per word
     * Ignore modem control lines.
     * Enable receiver.
     */

    terminalAttributes.c_cflag = B57600 | CS8 | CLOCAL | CREAD;

    /*
     * input modes: c_iflag flag constants:
     *
     * Ignore framing errors and parity errors.
     * (XSI) Map NL to CR-NL on output.
     */
    terminalAttributes.c_iflag = IGNPAR | ONLCR;

    /*
     * output modes: flag constants defined in POSIX.1
     *
     * Enable implementation-defined output processing.
     */

    terminalAttributes.c_oflag = OPOST;

    /*
     * Canonical and noncanonical mode
     *
     * min time
     * min bytes to read
     */

    //terminalAttributes.c_lflag = ICANON;
    terminalAttributes.c_cc[VTIME] = 0;
    terminalAttributes.c_cc[VMIN] = 1;

    /*
     * http://linux.die.net/man/3/tcsetattr
     * Set the port to our state
     *
     * the change occurs immediately
     */

    tcsetattr(fd_serialport, TCSANOW, &terminalAttributes);
    fcntl(fd_serialport, F_SETFL, FNDELAY);
    /*
     * http://linux.die.net/man/3/tcflush
     *
     * flushes data written but not transmitted.
     * flushes data received but not read.
     */

    tcflush(fd_serialport, TCOFLUSH);
    tcflush(fd_serialport, TCIFLUSH);

    return fd_serialport;
}

void Tserial::disconnect(void) {
    close(fd_serialport);
    //printf("\nPort 1 has been CLOSED and %d is the file description\n", fd_serialport);
}

int Tserial::sendArray(char *buffer, int len) {
    int n = write(fd_serialport, buffer, len);
    if (v == true) {
        printf("\n sucessfully sent %d chars", n);
    }
    return n;
}

int Tserial::getArray(char *buffer, int len) {
    //int n1=bytesToRead();
    int n = read(fd_serialport, buffer, len);
    if (v == true) {

        if (n == -1)printf("\n Array of %d read failed with status %d ", len, n);
        else printf("\n Array of %d read succesfull with status %d ", len, n);
        //printf(" bytesToRead %d ",n1);
    }
    return n;
}

void Tserial::clear() {
    tcflush(fd_serialport, TCIFLUSH);
    tcflush(fd_serialport, TCOFLUSH);
}

int Tserial::bytesToRead() {
    int bytes = 0;
    ioctl(fd_serialport, FIONREAD, &bytes);
    if (v == true) {
        printf("\nbytesToRead %d ", bytes);
    }
    return bytes;
}

/* -------------------------------------------------------------------- */
/* --------------------------    sendChar     ------------------------- */

/* -------------------------------------------------------------------- */
void Tserial::sendChar(char data) {
    sendArray(&data, 1);
    if (v == true)printf("\nsent %c", data);
}

/* -------------------------------------------------------------------- */
/* --------------------------    getChar      ------------------------- */

/* -------------------------------------------------------------------- */
char Tserial::getChar(void) {
    char c;

    if (v == true) {
        int n1 = bytesToRead();
        //   printf("\nbytesToRead %d ",n1);
    }
    int i = read(fd_serialport, &c, sizeof (char));
    if (v == true) {
        if (i == -1)printf(" read failed with status %d ", i);
        else printf(" read succesfull with status %d ", i);
    }
    return (c);
}

