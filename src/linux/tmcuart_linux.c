#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include "board/misc.h" 
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#include "internal.h"

static uint32_t baud;
static uint32_t bit_time;
static uint8_t uart_pin;
static uint8_t uart_active = 0;
static uint8_t read_data[10];
static uint8_t has_data;

static struct task_wake tmcuart_wake;

// GPIO
#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000) /* GPIO controller */

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock


uint8_t swuart_calcCRC(uint8_t* datagram, uint8_t datagram_length)
{
    int i, j;
    uint8_t crc = 0;
    uint8_t currentByte;
    for (i = 0; i < datagram_length; i++)
    {                              // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    }     // for message byte

    return crc;
}

static inline void 
wait(struct timespec *timer, long ns)
{
    struct timespec current;

    timer->tv_nsec += ns;
    uint32_t sec = timer->tv_nsec / NSECS;
    timer->tv_sec += sec;
    timer->tv_nsec -= sec * NSECS;
    
    while (1)
    {
        clock_gettime(CLOCK_MONOTONIC, &current);
        if (current.tv_sec > timer->tv_sec)
            break;

        if (current.tv_sec == timer->tv_sec && current.tv_nsec >= timer->tv_nsec)
            break;
    }
}

void
command_tmcuart_device_send(uint32_t *args)
{
    if (uart_active)
        return;

    uart_active = 1;

    has_data = 0;

    uint8_t is_read = args[2];

    uint32_t send_count = 3;

    uint8_t send_data[8];
    send_data[0] = 0x55;
    send_data[1] = args[0];
    send_data[2] = args[1];

    if (!is_read)
    {
        send_data[2] |= 0x80;
        send_data[3] = 0xFF & (args[3]>>24);
        send_data[4] = 0xFF & (args[3]>>16);
        send_data[5] = 0xFF & (args[3]>>8);
        send_data[6] = 0xFF & args[3];
        send_count = 7;
    }

    send_data[send_count] = swuart_calcCRC(send_data, send_count);
    send_count++;

    // set UART IDLE (high)
    INP_GPIO(uart_pin);
    OUT_GPIO(uart_pin);
    GPIO_SET = 1<<uart_pin;

    struct timespec timer;
    clock_gettime(CLOCK_MONOTONIC, &timer);

    wait(&timer, bit_time * 5);

    for (uint32_t i = 0; i < send_count; i++)
    {
        GPIO_CLR = 1<<uart_pin;
        wait(&timer, bit_time);

        for (uint32_t b = 0; b < 8; b++)
        {
            if ((send_data[i] >> b) & 0x1)
                GPIO_SET = 1<<uart_pin;
            else
                GPIO_CLR = 1<<uart_pin;

            wait(&timer, bit_time);
        }

        GPIO_SET = 1<<uart_pin;

        wait(&timer, bit_time);
    }

    wait(&timer, bit_time);

    INP_GPIO(uart_pin);
    
    wait(&timer, bit_time * 4);

    if (is_read)
    {
        // sync
        uint32_t sync_try = 0;
        while (1)
        {
            if (GET_GPIO(uart_pin) == 0)
                break;

            if (sync_try++ >= 640)
            {
                output("tmcuart_linux: read timeout");
                uart_active = 0;
                sched_wake_task(&tmcuart_wake);
                return;
            }
            
            wait(&timer, bit_time / 10);
        }

        has_data = 1;

        for (uint32_t i = 0; i < 8; i++)
        {
            if (i != 0)
            {
                // sync START bit
                sync_try = 0;
                while (sync_try <= 10)
                {
                    if (GET_GPIO(uart_pin) == 0)
                        break;
                    
                    sync_try++;
                    wait(&timer, bit_time / 10);
                }
            }

            clock_gettime(CLOCK_MONOTONIC, &timer);

            wait(&timer, bit_time);

            for (uint32_t b = 0; b < 8; b++)
            {
                if (GET_GPIO(uart_pin))
                    read_data[i] |= 1 << b;
                else
                    read_data[i] &= ~(1 << b);
                
                wait(&timer, bit_time);
            }

            // STOP bit
            wait(&timer, bit_time/5);
        }

        wait(&timer, bit_time * 4);
    }
    
    uart_active = 0;
    sched_wake_task(&tmcuart_wake);
}
DECL_COMMAND(command_tmcuart_device_send, "tmcuart_device_send addr=%u reg=%u read=%c write=%u");

// Report completed response message back to host
void
tmcuart_device_task(void)
{
    if (!sched_check_wake(&tmcuart_wake))
        return;

    uint32_t data = 0;
    uint8_t success = 0;

    if (has_data)
    {
        uint8_t crc = swuart_calcCRC(read_data, 7);

        if (crc == read_data[7])
        {
            success = 1;
            data = read_data[3] << 24 | read_data[4] << 16 | read_data[5] << 8 | read_data[6];
        }
        else
        {
            output("tmcuart_device_response: bad crc");
        }
    }

    sendf("tmcuart_device_response success=%c read=%u", success, data);
}
DECL_TASK(tmcuart_device_task);

void
command_config_tmcuart_device(uint32_t *args)
{
    int  mem_fd;
    void *gpio_map;

    uart_pin = args[0];
    baud = args[1];
    bit_time = NSECS / baud;

    // setup gpio
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        shutdown("tmcuart_linux: failed to open /dev/mem");
    }

    gpio_map = mmap(
        NULL,
        BLOCK_SIZE,
        PROT_READ|PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        GPIO_BASE
    );

    close(mem_fd);

    if (gpio_map == MAP_FAILED) {
        shutdown("tmcuart_linux: failed to mmap gpio");
    }

    gpio = (volatile unsigned *)gpio_map;

    output("tmcuart_linux: init pin=%u baud=%u bit_time=%u", uart_pin, baud, bit_time);
}
DECL_COMMAND(command_config_tmcuart_device, "config_tmcuart_device uart_pin=%u baud=%u");

void
tmcuart_device_shutdown(void)
{
}
DECL_SHUTDOWN(tmcuart_device_shutdown);