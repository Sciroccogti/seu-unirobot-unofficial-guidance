#include "gpio.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include "common.hpp"

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000)
#define MAX_BUF 64

std::map<std::string, gpio::pin_number> gpio::gpio_map = {
    {"GPIO0", gpio::PIN_GPIO0},
    {"GPIO1", gpio::PIN_GPIO1},
    {"GPIO2", gpio::PIN_GPIO2},
    {"GPIO3", gpio::PIN_GPIO3}
};

gpio::gpio(pin_number gpio)
    : io_(gpio)
{
    opened_ = gpio_export();
}

bool gpio::gpio_export()
{
    int fileDescriptor, length;
    char commandBuffer[MAX_BUF];
    char fnBuffer[MAX_BUF];

    fileDescriptor = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG << "gpio_export unable to open gpio: "<< io_ <<ENDL;
        return false;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", io_);

    snprintf(fnBuffer, sizeof(fnBuffer), SYSFS_GPIO_DIR  "/gpio%d", io_);
    if(access(fnBuffer, F_OK) != 0)
    {
        if (write(fileDescriptor, commandBuffer, length) != length) 
        {
            LOG << "gpio export error!" <<ENDL;
            return false;
        }
    }
    close(fileDescriptor);
    return true;
}

bool gpio::gpio_unexport()
{
    if(!opened_) return false;
    int fileDescriptor, length;
    char commandBuffer[MAX_BUF];

    fileDescriptor = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG << "gpio_export unable to open gpio: "<< io_ <<ENDL;
        return false;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", io_);
    if (write(fileDescriptor, commandBuffer, length) != length) 
    {
        LOG << "gpio unexport error!" <<ENDL;
        return false ;
    }
    close(fileDescriptor);
    return true;
}

bool gpio::set_direction(pin_direction dir)
{
    if(!opened_) return false;
    int fileDescriptor;
    char commandBuffer[MAX_BUF];
    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR  "/gpio%d/direction", io_);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG << "gpio_export unable to open gpio: "<< io_ <<ENDL;
        return false;
    }

    if (dir == PIN_OUTPUT) 
    {
        if (write(fileDescriptor, "out", 4) != 4) 
        {
            LOG << "gpio set direction error!" <<ENDL;
            return false ;
        }
    }
    else 
    {
        if (write(fileDescriptor, "in", 3) != 3) 
        {
            LOG << "gpio set direction error!" <<ENDL;
            return false ;
        }
    }
    close(fileDescriptor);
    return true;
}

bool gpio::set_value(pin_value v)
{
    if(!opened_) return false;
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", io_);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG << "gpio_export unable to open gpio: "<< io_ <<ENDL;
        return false;
    }

    if (v == PIN_HIGH) 
    {
        if (write(fileDescriptor, "1", 2) != 2) 
        {
            LOG << "gpio set value error!" <<ENDL;
            return false ;
        }
    }
    else 
    {
        if (write(fileDescriptor, "0", 2) != 2) 
        {
            LOG << "gpio set value error!" <<ENDL;
            return false ;
        }
    }
    close(fileDescriptor);
    return 0;
}

bool gpio::set_edge(char *edge)
{
    if(!opened_) return false;
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/edge", io_);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG << "gpio_export unable to open gpio: "<< io_ <<ENDL;
        return false;
    }

    if (write(fileDescriptor, edge, strlen(edge) + 1) != ((int)(strlen(edge) + 1))) 
    {
        LOG << "gpio set edge error!" <<ENDL;
        return false ;
    }
    close(fileDescriptor);
    return true;
}

int gpio::get_value()
{
    if(!opened_) return -1;
    int fileDescriptor;
    char commandBuffer[MAX_BUF];
    char ch;
    int res;

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", io_);

    fileDescriptor = open(commandBuffer, O_RDONLY);
    if (fileDescriptor < 0) 
    {
        LOG << "gpio_export unable to open gpio: "<< io_ <<ENDL;
        return -1;
    }

    if (read(fileDescriptor, &ch, 1) != 1) 
    {
        LOG << "gpio get value error!" <<ENDL;
        return -1;
     }

    if (ch != '0') 
    {
        res = 1;
    } else 
    {
        res = 0;
    }

    close(fileDescriptor);
    return res;
}