#ifndef _SERIAL_STREAM_HPP
#define _SERIAL_STREAM_HPP

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class serial_stream
{
private:
    int fd;
    std::string device_name;
    bool connected;
    int read_success;

public:
    serial_stream(void)
    {
        connected = false;
        read_success = 0;
    }
    ~serial_stream(void)
    {
        ss_close();
    }
    void set_name(std::string name)
    {
        device_name = name;
    }
    void ss_open(void)
    {
        fd = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        fcntl(fd, F_SETFL, 0);
        //load configuration
        struct termios conf_tio;
        tcgetattr(fd, &conf_tio);
        //set baudrate
        speed_t BAUDRATE = B1000000;
        cfsetispeed(&conf_tio, BAUDRATE);
        cfsetospeed(&conf_tio, BAUDRATE);
        //non canonical, non echo back
        conf_tio.c_lflag &= ~(ECHO | ICANON);
        //non blocking
        conf_tio.c_cc[VMIN] = 0;
        conf_tio.c_cc[VTIME] = 0;
        //store configuration
        tcsetattr(fd, TCSANOW, &conf_tio);
        if (fd >= 0)
        {
            connected = true;
        }
        else
        {
            connected = false;
        }
    }
    void ss_write(std::string data0)
    {
        if (connected)
        {
            int rec = write(fd, data0.c_str(), data0.size());
            if (rec < 0)
            {
                connected = false;
                ss_close();
            }
        }
    }
    std::string ss_read(void)
    {
        if (connected)
        {
            char buf[256] = {0};
            int recv_data = read(fd, buf, sizeof(buf));
            if (recv_data > 0)
            {
                read_success++;
                std::string recv_string = buf;
                return buf;
            }
            else
            {
                return "";
            }
        }
    }
    void ss_close(void)
    {
        close(fd);
        connected = false;
    }
    bool ss_connected(void)
    {
        return connected;
    }
    bool ss_status(void)
    {
        if (read_success > 0)
        {
            read_success = 0;
            return true;
        }
        else
            return false;
    }
};

#endif