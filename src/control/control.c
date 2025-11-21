#include <stdio.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include "common.h"
#include "control.h"
#include "config.h"
#include "protocol.h"
#include "main_update.h"

struct buffer {
    struct rotator_protocol protocol_msg;
    union protocol {
        struct rotator_status status;
        struct rotator_command command;
    } payload;
};

struct control_config {
    int tcp_port;
    int server_sock;
    int client_sock;
    int run;
    global_status_t *status;
    uint64_t status_timestamp;
};

static struct control_config ctl_cfg;
//static struct antenna_status antenna_status;
static global_status_t *global_status_ptr = NULL;
#define BUFFER_SIZE 100
#define STATUS_TIMEOUT 2000

void control_send_message()
{
    uint8_t buffer[sizeof(struct rotator_protocol) + sizeof(struct rotator_command) + 1];
    struct rotator_protocol *msg = (struct rotator_protocol *)buffer;
    struct rotator_command *command = (struct rotator_command *)&msg->payload;
    uint8_t *crc = buffer + sizeof(struct rotator_protocol) + sizeof(struct rotator_command);

    if (!global_status_ptr->connect_status)
        return;
    if (ctl_cfg.client_sock < 0)
        return;
    msg->start_byte = PROTOCOL_START_BYTE;
    msg->version = PROTOCOL_VERSION;
    msg->type = MESSAGE_TYPE_COMMAND;
    msg->length = sizeof(struct rotator_command);
    msg->timestamp = get_timestamp();
    command->position = global_status_ptr->encoder_cnt;
    command->switches = global_status_ptr->switch_status;

    *crc = crc8_data(&buffer[offsetof(struct rotator_protocol, timestamp)],
                     offsetof(struct rotator_protocol, payload) -
                     offsetof(struct rotator_protocol, timestamp) +
                     sizeof(struct rotator_command));

    ssize_t bytes_written;
    
    bytes_written = write(ctl_cfg.client_sock, buffer,
          sizeof(struct rotator_protocol) + sizeof(struct rotator_command) + 1);

    LOG2("Sent %d bytes message: length=%d, CRC=0x%x\n", (int)bytes_written, msg->length, *crc);
    dump(NULL, buffer, sizeof(struct rotator_protocol) + sizeof(struct rotator_command) + 1);
    if (bytes_written < 0)
        perror("Error writing to pipe");
}

static inline void close_fd(int *fd)
{
    if (fd && *fd >= 0) {
        close(*fd);
        *fd = -1;
    }
}

void process_message(struct buffer *buf)
{
    if (buf->protocol_msg.type == MESSAGE_TYPE_STATUS) {
//        LOG1("Angle: %d Power = %d\n", buf->payload.status.angle, buf->payload.status.status);
        global_status_ptr->angle = buf->payload.status.angle;
        global_status_ptr->power_status = buf->payload.status.status;
        global_status_ptr->updated = 1;
        global_status_ptr->vbat = buf->payload.status.vbat;
        update_status();
    }
}

#define RESET state = STATE_START_BYTE
static int parse_byte(uint8_t byte)
{
    static enum {
        STATE_START_BYTE,
        STATE_VERSION,
        STATE_TYPE,
        STATE_LENGTH,
        STATE_TIMESTAMP,
        STATE_PAYLOAD,
        STATE_CRC
    } state = STATE_START_BYTE;
    static int expected_length;
    static int current_length;
    static struct buffer buffer;
    static int wrong_message = 0;

    switch (state) {
        case STATE_START_BYTE:
            if (byte != PROTOCOL_START_BYTE)
                break;
            buffer.protocol_msg.start_byte = byte;
            expected_length = 1;
            state = STATE_VERSION;
            break;
        case STATE_VERSION:
            wrong_message = 0;
            memset(&buffer, 0, sizeof(struct buffer));
            if (byte != PROTOCOL_VERSION) {
                fprintf(stderr, "Unsupported protocol version: %d\n", byte);
                RESET;
                break;
            }
            buffer.protocol_msg.version = byte;
            expected_length = 1;
            state = STATE_TYPE;
            break;
        case STATE_TYPE:
            if (byte != MESSAGE_TYPE_STATUS && byte != MESSAGE_TYPE_COMMAND) {
                fprintf(stderr, "Unsupported message type: %d\n", byte);
                RESET;  // reset state
                break;
            }
            if (byte != MESSAGE_TYPE_STATUS) {
                fprintf(stderr, "Type status only allowed here\n");
                wrong_message = 1;
            }
            buffer.protocol_msg.type = byte;
            state = STATE_LENGTH;
            break;
        case STATE_LENGTH:
            buffer.protocol_msg.length = byte;
            if (buffer.protocol_msg.length != sizeof(struct rotator_status)) {
                fprintf(stderr, "Invalid payload length: %d\n", buffer.protocol_msg.length);
                RESET;  // reset state
                break;
            }
            state = STATE_TIMESTAMP;
            expected_length = 4;  // 4 bytes for timestamp
            current_length = 0;
            break;
        case STATE_TIMESTAMP:
            buffer.protocol_msg.timestamp = byte << 24 | buffer.protocol_msg.timestamp >> 8;
            current_length++;
            if (expected_length == current_length) {
                state = STATE_PAYLOAD;
                expected_length = buffer.protocol_msg.length;
                current_length = 0;
            }
            break;
        case STATE_PAYLOAD: {
            uint8_t *buf = (uint8_t *)&buffer.payload;
            buf[current_length++] = byte;
            if (current_length == expected_length) {
                expected_length = 1;  // CRC byte
                state = STATE_CRC;
            }
            break;
        }
        case STATE_CRC: {
            RESET;  // reset state
            if (wrong_message)
                break;
            uint8_t *buf = (uint8_t *)&buffer.protocol_msg;
            uint8_t crc = crc8_data(&buf[offsetof(struct rotator_protocol, timestamp)],
                                    buffer.protocol_msg.length + 4);
            if (byte != crc) {
                fprintf(stderr, "Invalid CRC\n");
                break;
            }
            process_message(&buffer);
            return 1;
        }
    }
    return 0;
}
#undef RESET

static void *control_thread_func(void *arg)
{
    struct control_config *cfg = (struct control_config *)arg;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int opt = 1;

    cfg->server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (cfg->server_sock < 0) {
        perror("Socket creation error");
        return NULL;
    }
    if (setsockopt(cfg->server_sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
        perror("Can't set socket opts SO_REUSEADDR | SO_REUSEPORT");

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(cfg->tcp_port);

    if (bind(cfg->server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Binding socket error");
        close_fd(&cfg->server_sock);
        return NULL;
    }

    if (listen(cfg->server_sock, 5) < 0) {
        perror("Listen error");
        close_fd(&cfg->server_sock);
        return NULL;
    }

    printf("TCP server started on port %d\n", cfg->tcp_port);

    while (cfg->run) {
        cfg->client_sock =
            accept(cfg->server_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (cfg->client_sock < 0) {
            perror("Error accepting connection");
            continue;
        }
        printf("Client connected\n");
        cfg->status->connect_status = 1;
        control_send_message();
        while (cfg->run && cfg->status->connect_status) {
            uint8_t buffer[BUFFER_SIZE];
            ssize_t bytes_read;

            struct pollfd fds[2];
            fds[0].fd = cfg->client_sock;
            fds[0].events = POLLIN;

            LOG1("Waiting for data...\n");
            while (cfg->run) {
                int ret = poll(fds, 1, 1000);
                if (ret < 0) {
                    perror("Error polling");
                    break;
                }
                if (ret == 0) {
                    if (cfg->status_timestamp) {
                        uint64_t timestamp;

                        timestamp = get_timestamp();
                        if (timestamp - cfg->status_timestamp > STATUS_TIMEOUT) {
                            cfg->status->connect_status = 0;
                            printf("Antenna Status timeout (%ld)\n", timestamp - cfg->status_timestamp);
                            break;
                        }
                    }
                }

                if (fds[0].revents & POLLIN) {
                    bytes_read = read(cfg->client_sock, buffer, sizeof(buffer));
                    if (bytes_read > 0) {
                        for (ssize_t i = 0; i < bytes_read; i++) {
                            if (parse_byte(buffer[i]))
                                cfg->status_timestamp = get_timestamp();
                        }
//                        LOG2("Received %d bytes from antenna\n", (int)bytes_read);
                        dump(NULL, buffer, bytes_read);
                    } else if (bytes_read == 0) {
                        printf("TCP peer closed the connection.\n");
                        cfg->status->connect_status = 0;
                        break;
                    }
                }
            }
        }
        cfg->status->connect_status = 0;

        close_fd(&cfg->client_sock);
        printf("Client disconnected\n");
    }

    close_fd(&cfg->server_sock);
    printf("Server stopped\n");
    return NULL;
}

static pthread_t control_thread;
extern int verbose;
int control_start(global_status_t *status_ptr, app_config_t *app_config)
{
    ctl_cfg.run = 1;
    verbose = 1;
    ctl_cfg.status = status_ptr;
    global_status_ptr = status_ptr;
    ctl_cfg.tcp_port = app_config->ctl_port;
    return pthread_create(&control_thread, NULL, control_thread_func, (void *)&ctl_cfg);
}

void control_stop(void)
{
    ctl_cfg.run = 0;
    close_fd(&ctl_cfg.server_sock);
    close_fd(&ctl_cfg.client_sock);
    printf("Wait for control finish\n");
    pthread_join(control_thread, NULL);
    printf("Control finished\n");
}