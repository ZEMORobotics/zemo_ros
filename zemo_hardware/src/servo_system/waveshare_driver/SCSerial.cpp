#include "zemo_hardware/servo_system/waveshare_driver/SCSerial.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>


bool baud_to_speed(int baud_rate, speed_t &speed){
    switch (baud_rate) {
        case 9600:
            speed = B9600;
            return true;
        case 19200:
            speed = B19200;
            return true;
        case 38400:
            speed = B38400;
            return true;
        case 57600:
            speed = B57600;
            return true;
        case 115200:
            speed = B115200;
            return true;
        case 500000:
            speed = B500000;
            return true;
        case 1000000:
            speed = B1000000;
            return true;

        default:
            return false;
    }
}

timeval make_timeout(int timeout_ms){
    if(timeout_ms < 0)
        timeout_ms = 0;

    timeval timeout{};

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    return timeout;
}


SCSerial::SCSerial(){
    IOTimeOut = 100;
    fd = -1;
    txBufLen = 0;
}

SCSerial::SCSerial(u8 End) : SCS(End){
    IOTimeOut = 100;
    fd = -1;
    txBufLen = 0;
}

SCSerial::SCSerial(u8 End, u8 Level): SCS(End, Level){
    IOTimeOut = 100;
    fd = -1;
    txBufLen = 0;
}


bool SCSerial::begin(int baudRate, const char *serialPort, unsigned int txEnableGpio, unsigned int rxEnableGpio){
    if(serialPort == nullptr) return false;

    if(fd != -1) end();

    speed_t baud;
    if(!baud_to_speed(baudRate, baud)) return false;

    fd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);

    if(fd == -1) return false;

    if(tcgetattr(fd, &orgopt) != 0){
        close(fd);
        fd = -1;
        return false;
    }

    curopt = orgopt;

    /*
     * Raw binary mode first.
     */
    cfmakeraw(&curopt);

    
    /*
     * 8 data bits
     * no parity
     * 1 stop bit
     * receiver enabled
     * ignore modem-control lines
     */
    curopt.c_cflag &= ~PARENB;
    curopt.c_cflag &= ~CSTOPB;
    curopt.c_cflag &= ~CSIZE;
    curopt.c_cflag &= ~CRTSCTS;

    curopt.c_cflag |= CS8;
    curopt.c_cflag |= CREAD;
    curopt.c_cflag |= CLOCAL;

    curopt.c_iflag &= ~(IXON | IXOFF | IXANY);
    curopt.c_cc[VMIN] = 0;
    curopt.c_cc[VTIME] = 0;

    if(cfsetispeed(&curopt, baud) != 0){
        close(fd);
        fd = -1;
        return false;
    }

    if(cfsetospeed(&curopt, baud) != 0){
        close(fd);
        fd = -1;
        return false;
    }

    if(tcsetattr(fd, TCSANOW, &curopt) != 0){
        close(fd);
        fd = -1;
        return false;
    }

    /*
     * Throw away stale data that might have been left
     * in the UART buffers before opening/configuring it.
     */
    if(tcflush(fd, TCIOFLUSH) != 0){
        close(fd);
        fd = -1;
        return false;
    }

    txBufLen = 0;

	gpio_chip_ = gpiod_chip_open_by_name("gpiochip0");
	if(!gpio_chip_){
		end();
		return false;
	}

	tx_enable_ = gpiod_chip_get_line(gpio_chip_, txEnableGpio);
	rx_enable_ = gpiod_chip_get_line(gpio_chip_, rxEnableGpio);

	if(!tx_enable_ || !rx_enable_){
		end();
		return false;
	}

	if(gpiod_line_request_output(tx_enable_, "scservo_tx_enable", 0) < 0){
		end();
		return false;
	}

	if(gpiod_line_request_output(rx_enable_, "scservo_rx_enable", 1) < 0){
		end();
		return false;
	}
    return true;
}

void SCSerial::setTxMode(){
	gpiod_line_set_value(rx_enable_, 0);
	gpiod_line_set_value(tx_enable_, 1);
}

void SCSerial::setRxMode(){
	gpiod_line_set_value(tx_enable_, 0);
	gpiod_line_set_value(rx_enable_, 1);
}

int SCSerial::setBaudRate(int baudRate){
    if (fd == -1)
        return -1;

    speed_t baud;
    if (!baud_to_speed(baudRate, baud))
        return -1;

    termios options{};

    if(tcgetattr(fd, &options) != 0) return -1;
    

    /*
     * Do not change baud rate while bytes are still
     * physically being transmitted.
     */
    while (tcdrain(fd) != 0) {
        if(errno == EINTR)
            continue;
        return -1;
    }

    if(cfsetispeed(&options, baud) != 0) return -1;
    if(cfsetospeed(&options, baud) != 0) return -1;
    if(tcsetattr(fd, TCSANOW, &options) != 0) return -1;

    curopt = options;

    return 1;
}


int SCSerial::readSCS(unsigned char *nDat, int nLen){
    if (fd == -1) return -1;
    if (nDat == nullptr) return -1;
    if (nLen < 0) return -1;
    if (nLen == 0) return 0;

    int rvLen = 0;
	setRxMode();

    while(rvLen < nLen){
        fd_set read_set;

        FD_ZERO(&read_set);
        FD_SET(fd, &read_set);

        timeval timeout = make_timeout(IOTimeOut);

        const int result = select(fd + 1, &read_set, nullptr, nullptr, &timeout);

        if(result > 0){
            const ssize_t n = read(fd, nDat + rvLen, static_cast<size_t>(nLen - rvLen));

            if(n > 0){
                rvLen += static_cast<int>(n);
                continue;
            }

            if (n == 0) return rvLen;
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;

            return -1;
        }

        //Timeout.
        if(result == 0) return rvLen;

        //select() interrupted by a signal.
        if(errno == EINTR) continue;

        return -1;
    }

    return rvLen;
}


int SCSerial::writeSCS(unsigned char *nDat, int nLen){
    if(nDat == nullptr) return -1;
    if(nLen < 0) return -1;
    if(nLen == 0) return txBufLen;
    if(nLen > SCSERVO_BUFFER_SIZE - txBufLen) return -1;

    std::memcpy(txBuf + txBufLen, nDat, static_cast<size_t>(nLen));

    txBufLen += nLen;
    return txBufLen;
}


int SCSerial::writeSCS(unsigned char bDat){
    if(txBufLen >= SCSERVO_BUFFER_SIZE) return -1;

    txBuf[txBufLen++] = bDat;
    return txBufLen;
}


void SCSerial::rFlushSCS(){
    if(fd == -1) return;
    tcflush(fd, TCIFLUSH);
}


void SCSerial::wFlushSCS(){
    if(fd == -1) return;
	if(txBufLen <= 0) return;

    setTxMode();

    int total_written = 0;

    while(total_written < txBufLen){
        fd_set write_set;

        FD_ZERO(&write_set);
        FD_SET(fd, &write_set);

        timeval timeout = make_timeout(IOTimeOut);
        const int result = select(fd + 1, nullptr, &write_set, nullptr, &timeout);

        if(result > 0){
            const ssize_t n = write(fd, txBuf + total_written, static_cast<size_t>(txBufLen - total_written));

            if(n > 0){
                total_written += static_cast<int>(n);
                continue;
            }

            if(n == 0) continue;
            if(errno == EINTR) continue;
            if(errno == EAGAIN || errno == EWOULDBLOCK) continue;

            txBufLen = 0;
            return;
        }

        if(result == 0){
            txBufLen = 0;
            return;
        }

        if(errno == EINTR) continue;

		while(tcdrain(fd) != 0){
			if(errno == EINTR) continue;

			setRxMode();
			txBufLen = 0;
			return;
		}

		setRxMode();
        txBufLen = 0;
        return;
    }

    /*
     * write() only guarantees that the kernel accepted
     * the bytes. tcdrain() waits until the UART has
     * physically transmitted them.
     *
     * This will be especially important once TX_ENABLE /
     * RX_ENABLE GPIO handling is added.
     */
    while (tcdrain(fd) != 0) {
        if (errno == EINTR) continue;
        break;
    }
	setRxMode();

    txBufLen = 0;
}


void SCSerial::end() noexcept{

	if(tx_enable_){
        gpiod_line_set_value(tx_enable_, 0);
        gpiod_line_release(tx_enable_);
        tx_enable_ = nullptr;
    }

    if(rx_enable_){
        gpiod_line_set_value(rx_enable_, 0);
        gpiod_line_release(rx_enable_);
        rx_enable_ = nullptr;
    }

    if(gpio_chip_){
        gpiod_chip_close(gpio_chip_);
        gpio_chip_ = nullptr;
    }

    if(fd == -1) return;

    close(fd);
    fd = -1;
    txBufLen = 0;
}