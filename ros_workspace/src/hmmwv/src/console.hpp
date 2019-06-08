#pragma once

#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>

class Console {

private:
    int m_tty;

public:
    Console(const std::string tty) {
        m_tty = open(tty.c_str(), O_RDWR | O_NOCTTY);
        if (m_tty < 0) {
            perror("open");
        }

        struct termios settings;
        tcgetattr(m_tty, &settings);
        cfsetospeed(&settings, B115200);
        cfsetispeed(&settings, B0);
        settings.c_cflag &= ~PARENB;	                        // no parity
        settings.c_cflag &= ~CSTOPB;	                        // one stop bit
        settings.c_cflag &= ~CSIZE;		                        // 8 bits per character
        settings.c_cflag |= CS8;
        settings.c_cflag &= ~CRTSCTS;	                        // Disable hardware flow control
        settings.c_cflag |= CREAD;		                        // Enable receiver
        settings.c_iflag &= ~(IXON | IXOFF | IXANY);            // Disable start/stop I/O control
        settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    // Disable user terminal features
        settings.c_oflag &= ~OPOST;		                        // Disable output postprocessing
        settings.c_cc[VMIN]  = 0;
        settings.c_cc[VTIME] = 0;
        if (tcsetattr(m_tty, TCSANOW, &settings) != 0) {
            perror("tcsetattr");
            printf("1\n");
        }
        tcflush(m_tty, TCOFLUSH);
    }

    ~Console() {
        close(m_tty);
    }

    ssize_t send(const uint16_t cmd) {
        unsigned char buf[2];
        buf[0] = cmd >> 8;
        buf[1] = cmd;
        printf("0x%x, 0x%x\n", buf[0], buf[1]);
        return send(buf, 2);
    }

    ssize_t send(const unsigned char *cmd, const size_t length) {
        return write(m_tty, cmd, length);
    }

};
