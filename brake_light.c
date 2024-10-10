#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <lcm/lcm.h>
#include <unistd.h>
#include "exlcm/exlcm_message_t.h"
#include "exlcm/exlcm_heartbeat_t.h"
#include "exlcm/exlcm_object_t.h"

static int mode = 0;
static int detectedObject = 0;
static int id = 0;
static int64_t last_message_ts = 0;
static exlcm_message_t convoy_msg;
static exlcm_object_t object_msg;

int64_t get_timestamp_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}


void send_heartbeat(lcm_t *lc, int8_t entity_id) {
    exlcm_heartbeat_t hb_msg;
    hb_msg.timestamp = get_timestamp_us();
    hb_msg.ID = entity_id;

    exlcm_heartbeat_t_publish(lc, "HEARTBEAT", &hb_msg);
    printf("Heartbeat sent at %lld\n", (long long)hb_msg.timestamp);
}

void send_warning_message(lcm_t *lc, int8_t entity_id) {
    exlcm_message_t warning_msg;
    warning_msg.timestamp = get_timestamp_us();
    warning_msg.ID = entity_id;
    warning_msg.name = "BrakeLightSimulator";
    warning_msg.detectedObject = 1;
    warning_msg.mode = mode;

    exlcm_message_t_publish(lc, "MESSAGE", &warning_msg);
    printf("Warning sent: Danger ahead at %lld\n", (long long)warning_msg.timestamp);
}

void message_handler_convoy(const lcm_recv_buf_t *rbuf, const char *channel,
                     const exlcm_message_t *msg, void *userdata) {
    //printf("Received message on channel %s: Mode %d, ID: %d, Danger ahead: %d\n", channel, msg->mode, msg->ID, msg->detectedObject);
    convoy_msg.detectedObject = msg->detectedObject;
    convoy_msg.ID = msg->ID;
    convoy_msg.mode = msg->mode;
    convoy_msg.name = msg->name;
    convoy_msg.timestamp = msg->timestamp;
}

void message_handler_object(const lcm_recv_buf_t *rbuf, const char *channel,
                     const exlcm_object_t *msg, void *userdata) {
    //printf("Received message on channel %s: Name: %s, DetectedObject: %d\n", channel, msg->name, msg->detectedObject);
    object_msg.detectedObject = msg->detectedObject;
    object_msg.name = msg->name;
    object_msg.timestamp = msg->timestamp;
}

void process_scenario(lcm_t *lc, int8_t entity_id) {
    int64_t current_ts = get_timestamp_us();
    switch (mode) {
        case 0:
            if (object_msg.detectedObject) {
                printf("Mode 0: Object detected! Brake lights ON!\n");
            } else {
                printf("Mode 0: No action.\n");
            }
            break;

        case 1:
            if (object_msg.detectedObject) {
                printf("Mode 1: Object detected! Brake lights ON! Sending warning message.\n");
                send_warning_message(lc, entity_id);
                last_message_ts = current_ts;
            } else {
                if (current_ts - last_message_ts >= 1000000) {
                    printf("Mode 1: No object detected. Sending heartbeat.\n");
                    send_heartbeat(lc, entity_id);
                    last_message_ts = current_ts;
                }
            }
            break;

        case 2:
            if (convoy_msg.ID != entity_id && convoy_msg.detectedObject == 1) {
                printf("Brake lights ON! Repeating warning message.\n");
                send_warning_message(lc, entity_id);
                last_message_ts = current_ts;
            }
            else {
                if (current_ts - last_message_ts >= 1000000) {
                    printf("Mode 2: Sending heartbeat.\n");
                    send_heartbeat(lc, entity_id);
                    last_message_ts = current_ts;
                }
            }
            break;
    
        default:
            printf("Unknown mode: %d\n", mode);
            break;
    }
}

int main(int argc, char **argv) {
    lcm_t *lc = lcm_create(NULL);
    if (!lc) {
        fprintf(stderr, "Error creating LCM instance\n");
        return 1;
    }

    if (argc < 3) {
        id = atoi(argv[1]);
    }
    else if (argc > 2) {
        id = atoi(argv[1]);
        if (atoi(argv[2]) > 2) {
            mode = 0;
        }
        else
            mode = atoi(argv[2]);
    }

    printf("Entity with ID: %d started with mode %d\n\n", id, mode);

    exlcm_object_t_subscribe(lc, "OBJECT", &message_handler_object, lc);
    exlcm_message_t_subscribe(lc, "MESSAGE", &message_handler_convoy, lc);

    last_message_ts = get_timestamp_us();

    while (1) {

        lcm_handle_timeout(lc, 1000);

        process_scenario(lc, id);
    }

    lcm_destroy(lc);
    return 0;
}
