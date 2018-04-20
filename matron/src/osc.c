/*
 * osc.c
 *
 * user OSC device, send/receive arbitrary OSC within lua scripts
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include <lo/lo.h>
#include <dns_sd.h>

#include "args.h"
#include "events.h"
#include "oracle.h"

static lo_server_thread st;
static DNSServiceRef dnssd_ref;

static int osc_receive(const char *path, const char *types,
			lo_arg **argv, int argc, lo_message msg, void *user_data);
static void lo_error_handler(int num, const char *m, const char *path);

void osc_init(void) {
    // receive
    st = lo_server_thread_new("10111", lo_error_handler);
    lo_server_thread_add_method(st, NULL, NULL, osc_receive, NULL);
    lo_server_thread_start(st);

    DNSServiceRegister(&dnssd_ref,
        0,
        0,
        "norns",
        "_osc._udp",
        NULL,
        NULL,
        htons(lo_server_thread_get_port(st)),
        0,
        NULL,
        NULL,
        NULL);
}

void osc_deinit(void) {
    DNSServiceRefDeallocate(dnssd_ref);
    lo_server_thread_free(st);
}

void osc_send(const char *host, const char *port, const char *path, lo_message msg) {
    lo_address address = lo_address_new(host, port);
    if (!address) {
        fprintf(stderr, "failed to create lo_address\n");
        return;
    }
    lo_send_message(address, path, msg);
    lo_address_free(address);
}

int osc_receive(const char *path,
                       const char *types,
                       lo_arg **argv,
                       int argc,
                       lo_message msg,
                       void *user_data)
{
    (void)types;
    (void)argv;
    (void)argc;
    (void)user_data;

    union event_data *ev = event_data_new(EVENT_OSC);

    ev->osc_event.path = (char *) malloc(strlen(path) + 1);
    strcpy(ev->osc_event.path, path);

    ev->osc_event.msg = lo_message_clone(msg);

    lo_address source = lo_message_get_source(msg);
    const char *host = lo_address_get_hostname(source);
    const char *port = lo_address_get_port(source);

    ev->osc_event.from_host = (char *) malloc(strlen(host) + 1);
    strcpy(ev->osc_event.from_host, host);

    ev->osc_event.from_port = (char *) malloc(strlen(port) + 1);
    strcpy(ev->osc_event.from_port, port);

    event_post(ev);

    return 0;
}

void lo_error_handler(int num, const char *m, const char *path) {
    printf("liblo error %d in path %s: %s\n", num, path, m);
    fflush(stdout);
}
