#pragma once

#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstdio>

class Console {

private:
    int m_tty;

public:
    Console(const std::string tty) {
        struct termios settings;

        m_tty = open(tty.c_str(), O_RDWR | O_NONBLOCK);
        if (m_tty < 0) {
            perror("open");
            exit(EXIT_FAILURE);
        }

        if (tcgetattr(m_tty, &settings) < 0) {
            perror("failed to get terminal attributes");
            exit(EXIT_FAILURE);
        }

        cfsetospeed(&settings, B57600);
        cfsetispeed(&settings, B57600);

        settings.c_cflag &= ~PARENB;	                        // no parity
        settings.c_cflag &= ~CSTOPB;	                        // one stop bit
        settings.c_cflag &= ~CSIZE;		                        // 8 bits per character
        settings.c_cflag |= CS8;

        settings.c_cflag &= ~CRTSCTS;	                        // Disable hardware flow control

        settings.c_cflag |= CREAD | CLOCAL;                     // Enable receiver and ignore ctrl lines
        settings.c_iflag &= ~(IXON | IXOFF | IXANY);            // Disable software flow control

        settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    // Disable user terminal features, make raw
        settings.c_oflag &= ~OPOST;		                        // Disable output postprocessing, make raw

        settings.c_cc[VMIN]  = 0;
        settings.c_cc[VTIME] = 0;

        tcsetattr(m_tty, TCSANOW, &settings); 
        if (tcsetattr(m_tty, TCSAFLUSH, &settings) < 0) {
            perror("failed to set tcsetattr");
            exit(EXIT_FAILURE);
        }

        // wait to seconds to avoid that arduino resets due to enabled DTR
        sleep(2);
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
