#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include "circ_buf.h"
#include "crsf_protocol.h"
//#include "shmem.h"
#include "utils.h"
#include "config.h"
#include "control.h"
#include "crsf2net.h"
#include "main_update.h"

#define UART_DEVICE "/dev/ttyS0"
#define BAUD_RATE 115200

#define UDP_PORT 7300

#define BUFFER_SIZE 64

#define VERSION "1.0"

#define CRSF_PID_FILE "/tmp/crsf-bridge.pid"

extern int verbose;
int rc_mode = 0;     // RC mode flag
int diagnostic = 0;  // Diagnostic flag

struct crsf_cfg {
    char *ip_addr;
    int ip_port;
    char *uart;
    int baudrate;
    struct circular_buf cbuf;
};

typedef void (*process_func_t)(int uart_fd, int udp_sock, struct circular_buf *cbuf, const char *ip_addr, uint16_t udp_port);

static process_func_t process_connection_func = NULL;
static int extra_messages_cnt = 0;
static int extra_message_len[VRX_TABLE_COUNT];
static uint8_t extra_messages[VRX_TABLE_COUNT][CRSF_MAX_PACKET_SIZE];
static global_status_t *global_status_ptr;
#define DSCP_EF 0x2e  // 46
#define IPTOS_EF (DSCP_EF << 2)

#define STATE_SOURCE 0
#define STATE_LENGTH 1
#define STATE_TYPE 2
#define STATE_PAYLOAD 3
#define STATE_CRC 4

struct parser_state {
    int state;
    int length;
    int expected_length;
    int payload_length;
    int payload_pos;
    uint64_t packets;  // parsed packets
    uint64_t errs;     // unparsed packets
    uint8_t buffer[CRSF_MAX_PACKET_SIZE];
    uint8_t type;
    uint8_t *payload;
};

struct parser_state net_parser;
struct parser_state uart_parser;

struct cbuf_item {
    uint8_t buf[CRSF_MAX_PACKET_SIZE];
    size_t len;
};

#define CBUF_NUM 3

#define MAYBE_UNUSED __attribute__((unused))
static volatile int run;
static pthread_t crsf_thread;

static void prepare_extra_messages()
{
    uint16_t *table;

    if (extra_messages_cnt)
        return;
    for (int band = 0; band < global_status_ptr->bands; band++) {
        extra_messages[band][0] = 0xee;  // Address
        extra_messages[band][1] = 1 + 2 + 1 + 16 * 2 + 1;  // Msg len
                                                    //type(1) MCP header(2) + band(1) + vrx_table(16*2) + CRC(1)
        extra_messages[band][2] = 0x7a;  // Type MCP
        extra_messages[band][3] = 0xc8;  // MCP header
        extra_messages[band][4] = 0x00;  // MCP ID
        extra_messages[band][5] = global_status_ptr->vrx_table[band].band;
        table = (uint16_t *)&extra_messages[band][6];
        for (int i = 0; i < 16; i++)
            table[i] = global_status_ptr->vrx_table[band].table[i];
        uint8_t crc = crc8_data(&extra_messages[band][2], 1 + 2 + 1 + 16 * 2);
        extra_messages[band][6+32] = crc;
        extra_message_len[band] = 6 + 32 + 1;
    }
    extra_messages_cnt = global_status_ptr->bands;
}

#if 1
static int setup_uart(const char *device, int MAYBE_UNUSED baud_rate)
{
    int uart_fd;
    struct termios tty_config;
    int try;

    printf("Openning %s at %d\n", device, baud_rate);
    try = 0;
    while (run) {
        try++;
        printf("Open UART try %d\n", try);
        uart_fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (uart_fd < 0) {
            perror("Error opening UART");
            set_status_line(SEVERITY_ERROR, "Can't access RC");
            sleep(2);
            continue;
        }
        if (tcgetattr(uart_fd, &tty_config) != 0) {
            set_status_line(SEVERITY_ERROR, "Can't access RC");
            perror("Error tcgetattr");
            close(uart_fd);
            return -1;
        }

        tty_config.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        tty_config.c_iflag = IGNPAR;
        tty_config.c_oflag = 0;
        tty_config.c_lflag = 0;
        tcflush(uart_fd, TCIFLUSH);
        if (tcsetattr(uart_fd, TCSANOW, &tty_config) != 0) {
            set_status_line(SEVERITY_ERROR, "Can't access RC");
            perror("Error tcsetattr");
            close(uart_fd);
            return -1;
        }
        int flags = fcntl(uart_fd, F_GETFL, 0);
        if (flags != -1) {
            flags |= O_NONBLOCK;
            if (fcntl(uart_fd, F_SETFL, flags) == -1)
                perror("UART: Set non-blocking mode");
        } else {
            perror("UART: Get descriptor flags");
        }
        if (verbose)
            printf("UART configured on %s with baud rate %d\n", device, baud_rate);
        if (try > 1)
            set_status_line(SEVERITY_NOTIFICATION, "Ready");
        return uart_fd;
    }
    return -1;
}
#else
static int setup_uart(const char *device, int baud_rate)
{
    int uart_fd;
    struct termios tty_config;
    // You should save the original settings to restore them later
    struct termios tty_old_config;
    int try;

    printf("Openning %s at %d\n", device, baud_rate);
    try = 0;
    while (run) {
        try++;
        uart_fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (uart_fd < 0) {
            perror("Error opening UART");
            if (errno == ENOENT || errno == ENODEV || errno == EPERM) {
                sleep(2);
                continue;
            }
            return -1;
        }

        if (tcgetattr(uart_fd, &tty_config) != 0) {
            perror("Error tcgetattr");
            close(uart_fd);
            return -1;
        }
        // Save old settings for restoration
        tty_old_config = tty_config;

        // Set input and output baud rate
        cfsetispeed(&tty_config, baud_rate);
        cfsetospeed(&tty_config, baud_rate);

        // Raw mode: no input or output processing
        tty_config.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem status lines
        tty_config.c_cflag &= ~PARENB;           // No parity
        tty_config.c_cflag &= ~CSTOPB;           // 1 stop bit
        tty_config.c_cflag &= ~CSIZE;            // Clear character size mask
        tty_config.c_cflag |= CS8;               // 8 data bits

        tty_config.c_iflag = 0;  // Disable all input processing
        tty_config.c_oflag = 0;  // Disable all output processing
        tty_config.c_lflag = 0;  // Disable all local modes

        // Set control characters for non-blocking read
        tty_config.c_cc[VMIN] = 1;
        tty_config.c_cc[VTIME] = 0;

        // Apply the changes immediately
        if (tcsetattr(uart_fd, TCSANOW, &tty_config) != 0) {
            perror("Error tcsetattr");
            close(uart_fd);
            return -1;
        }
        // Flush any pending data after setting the new configuration
        tcflush(uart_fd, TCIFLUSH);

        // This part is for demonstration and would need the restore logic
        if (verbose)
            printf("UART configured on %s with baud rate %d\n", device, baud_rate);
        return uart_fd;
    }
    return -1;
}
#endif

static void parser_init(struct parser_state *parser)
{
    parser->state = STATE_SOURCE;
    parser->length = 0;
    parser->expected_length = 0;
}

static int parser(struct parser_state *parser, uint8_t byte)
{
    uint64_t timestamp;
    static uint64_t last_timestamp = 0;

    timestamp = get_timestamp();
    if (last_timestamp) {
        if (parser->state != STATE_SOURCE && (timestamp - last_timestamp) > 20) {
            printf("diff ts: %ld state %d\n", timestamp - last_timestamp, parser->state);
            parser->state = STATE_SOURCE;
            parser->errs++;
        }
    }
    last_timestamp = timestamp;

    switch (parser->state) {
        case STATE_SOURCE:
            if (byte == CRSF_ADDRESS_RADIO_TRANSMITTER || byte == CRSF_ADDRESS_CRSF_TRANSMITTER ||
                byte == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                parser->buffer[0] = byte;
                parser->state = STATE_LENGTH;
            }
            break;
        case STATE_LENGTH:
            if (byte < 3 || byte > (CRSF_MAX_PAYLOAD_LEN + 2)) {
                parser->length = 0;
                parser->state = STATE_SOURCE;
                parser->errs++;
                break;
            }
            parser->expected_length = byte;
            parser->buffer[1] = byte;
            parser->length = 2;
            parser->expected_length = byte + 2;
            parser->payload_length = byte - 2;
            parser->payload_pos = 0;
            parser->state = STATE_TYPE;
            break;
        case STATE_TYPE:
            parser->buffer[parser->length++] = byte;
            parser->type = byte;
            parser->state = STATE_PAYLOAD;
            break;
        case STATE_PAYLOAD:
            if (!parser->payload_pos)
                parser->payload = &parser->buffer[parser->length];
            parser->payload_pos++;
            parser->buffer[parser->length++] = byte;
            if (parser->payload_pos >= parser->payload_length) {
                // Message complete
                parser->state = STATE_CRC;
                parser->packets++;
            }
            break;
        case STATE_CRC:
            parser->buffer[parser->length++] = byte;
            parser->state = STATE_SOURCE;
            uint8_t crc = crc8_data(&parser->buffer[2], parser->payload_length + 1);
            if (crc == byte) {
                parser->packets++;
                return parser->length;  // Return length of complete message
            } else {
                parser->errs++;
                return -parser->length;  // Return length of complete message
            }

        default:
            parser->state = STATE_SOURCE;  // Reset state on error
            parser->errs++;
            break;
    }
    return 0;  // Message not complete yet
}

static void process_rc_packet(struct parser_state *parser)
{
    if (global_status_ptr) {
        if (parser->type == 0x16 && parser->payload_length >= 22) {
            memcpy(&global_status_ptr->channels, parser->payload, parser->payload_length);
            global_status_ptr->channels_updated = 1;  // Indicate new data available
        }
    }
}

static void process_connection_rc(int uart_fd, int udp_sock, struct circular_buf *cbuf, const char *ip_addr, uint16_t udp_port)
{
    char buffer[BUFFER_SIZE];
    ssize_t bytes_read;
    struct sockaddr_in from, to;
    socklen_t from_len;
    int index = -1;

    struct pollfd fds[2];
    fds[0].fd = uart_fd;
    fds[0].events = POLLIN;
    fds[1].fd = udp_sock;
    fds[1].events = POLLIN;

    to.sin_family = AF_INET;
    to.sin_port = htons(udp_port);
    if (inet_pton(AF_INET, ip_addr, &to.sin_addr) <= 0) {
        perror("Invalid IP address");
        close(udp_sock);
        return;
    }
    tcflush(uart_fd, TCIOFLUSH);

    if (verbose)
        printf("Waiting for data...\n");
    while (run) {
        int ret = poll(fds, 2, 1000);
        if (ret < 0) {
            perror("Error polling");
            if (errno == EINTR)
                continue;
            break;
        }
        if (diagnostic) {
            printf("UDP  packets: %lu errors: %lu\n", net_parser.packets, net_parser.errs);
            printf("UART packets: %lu errors: %lu\r\033[A", uart_parser.packets,
                   uart_parser.errs);
        }
        if (ret == 0)
            continue;

        if (fds[0].revents & POLLIN) {
            bytes_read = read(uart_fd, buffer, sizeof(buffer));
            if (_unlikely(bytes_read <= 0)) {
                if (!bytes_read)
                    printf("UART closed the connection.\n");
                else
                    perror("Error reading from UART");
                return;
            }
            for (int i = 0; i < bytes_read; i++) {
                int packet_length = parser(&uart_parser, buffer[i]);
                uint8_t *buf;
                int bytes_written;

                if (!packet_length)
                    continue;
                if (_unlikely(extra_messages_cnt)) {
                    if (index < 0)
                        index = 0;
                    if (index < extra_messages_cnt) {
                        sendto(udp_sock, extra_messages[index], extra_message_len[index], 0,
                                (struct sockaddr *)&to, sizeof(to));
                        index++;
                        continue;
                    }
                    extra_messages_cnt = 0;
                    global_status_ptr->bands = 0;
                    index = -1;
                }
                if (sendto(udp_sock, uart_parser.buffer, uart_parser.length, 0,
                            (struct sockaddr *)&to, sizeof(to)) < 0) {
                    perror("Error sending over UDP");
                    continue;
                }
                if (packet_length > 0) {
                    process_rc_packet(&uart_parser);
                }
                if (verbose) {
                    printf("UART -> UDP: sent %d bytes\n", uart_parser.length);
                    if (verbose > 1)
                        dump("UART data", uart_parser.buffer, uart_parser.length);
                }
                if (cbuf_empty(cbuf))
                    continue;
                buf = cbuf_get_ptr(cbuf);
                if (verbose) {
                    printf("Writing to UART %d bytes\n", buf[1]);
                    if (verbose > 1)
                        dump("Data", buf, buf[1]);
                }
                bytes_written = write(uart_fd, buf, buf[1] + 2);
                if (bytes_written > 0) {
                    cbuf_drop(cbuf);
                    continue;
                }
                if (!bytes_written) {
                    perror("UART closed");
                } else {
                    perror("UART write error");
                    if (errno == EINTR)
                        continue;
                }
                return;
            }
        }

        if (fds[1].revents & POLLIN) {
            from_len = sizeof(struct sockaddr_in);
            bytes_read =
                recvfrom(udp_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
            if (bytes_read <= 0) {
                perror("Error reading from UDP");
                break;
            }
            for (int i = 0; i < bytes_read; i++) {
                int len = parser(&net_parser, buffer[i]);
                if (len <= 0)
                    continue;
                cbuf_put(cbuf, net_parser.buffer);
                if (verbose) {
                    printf("UDP(from %s): received %d bytes\n", inet_ntoa(from.sin_addr),
                            net_parser.length);
                    if (verbose > 1)
                        dump("UDP data", net_parser.buffer, net_parser.length);
                }
            }
        }
    }
    if (!run) {
        printf("Connection closed by user.\n");
    }
    close(udp_sock);
}

static int main_loop(char *peer_ip, uint16_t udp_port, struct circular_buf *cbuf, const char *dev, int baudrate)
{
    struct sockaddr_in sock_addr;
    int udp_sock;
    int uart_fd;
    int rc = 0;

    while (run) {
        uart_fd = setup_uart(dev, baudrate);
        if (uart_fd < 0)
            return -1;
        udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sock < 0) {
            perror("Socket creation error");
            close(uart_fd);
            return -1;
        }
        int flags = fcntl(udp_sock, F_GETFL, 0);
        if (flags != -1) {
            flags |= O_NONBLOCK;
            if (fcntl(udp_sock, F_SETFL, flags) == -1)
                perror("UDP: Set non-blocking mode");
        } else {
            perror("UDP: Get descriptor flags");
        }
        int tos = IPTOS_EF;
        // Setup Expedited Forwarding (EF).
        if (setsockopt(udp_sock, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) < 0)
            perror("setsockopt IP_TOS failed");
        int priority = 6;

        if (setsockopt(udp_sock, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) < 0)
            perror("setsockopt SO_PRIORITY failed");
        sock_addr.sin_family = AF_INET;
        sock_addr.sin_port = htons(udp_port);
        sock_addr.sin_addr.s_addr = INADDR_ANY;
        rc = bind(udp_sock, (const struct sockaddr *)&sock_addr, sizeof(sock_addr));
        if (rc < 0) {
            perror("Error binding UDP socket");
            close(udp_sock);
            run = 0;
            break;
        }
        process_connection_func(uart_fd, udp_sock, cbuf, peer_ip, udp_port);
        close(udp_sock);
        close(uart_fd);
    }
    if (verbose)
        printf("Client stopped\n");
    return rc;
}

static void *crsf_thread_func(void *arg)
{
    struct crsf_cfg *cfg = (struct crsf_cfg *)arg;
    int uart_fd;
    if (!arg)
        return NULL;
    main_loop(cfg->ip_addr, cfg->ip_port, &cfg->cbuf, cfg->uart, cfg->baudrate);
    cbuf_destroy(&cfg->cbuf);
    free(arg);
    return NULL;
}

int crsf_start(global_status_t *status_ptr, app_config_t *app_config)
{
    struct crsf_cfg *cfg;
    uint8_t *cbuffers;

    cfg = malloc(sizeof(struct crsf_cfg));
    if (!cfg)
        return 1;
    cfg->ip_addr = app_config->antenna_ip;
    cfg->ip_port = app_config->crsf_port;
    cfg->uart = app_config->uart_dev;
    cfg->baudrate = app_config->baudrate;
    global_status_ptr = status_ptr;
    process_connection_func = process_connection_rc;
    cbuffers = (uint8_t *)malloc(CRSF_MAX_PACKET_SIZE * CBUF_NUM * sizeof(uint8_t));
    cbuf_init(&cfg->cbuf, cbuffers, CBUF_NUM, CRSF_MAX_PACKET_SIZE * sizeof(uint8_t));
    run = 1;
    return pthread_create(&crsf_thread, NULL, crsf_thread_func, (void *)cfg);
}

void crsf_stop(void)
{
    run = 0;
    printf("Wait for crsf finish\n");
    pthread_join(crsf_thread, NULL);
    printf("Crsf finished\n");
}

void crsf_send_vrxtable(void)
{
    prepare_extra_messages();
}