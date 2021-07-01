#pragma once
#include "imxrt.h"

extern uint16_t tx_packet_size;
extern uint16_t rx_packet_size;

typedef struct transfer_struct transfer_t;
struct transfer_struct {
        transfer_t *next;
        uint32_t status;
        uint32_t pointer0;
        uint32_t pointer1;
        uint32_t pointer2;
        uint32_t pointer3;
        uint32_t pointer4;
        struct transfer_struct *callback_param;
};

#ifdef __cplusplus
extern "C" {
#endif
	extern transfer_t *tx_free_list, *rx_cmd_busy_list, *rx_dout_busy_list;
void usb_init(void);
void usb_init_serialnumber(void);

void usb_config_rx(uint32_t ep, uint32_t packet_size, int do_zlp, void (*cb)(transfer_t *));
void usb_config_tx(uint32_t ep, uint32_t packet_size, int do_zlp, void (*cb)(transfer_t *));
void usb_config_rx_iso(uint32_t ep, uint32_t packet_size, int mult, void (*cb)(transfer_t *));
void usb_config_tx_iso(uint32_t ep, uint32_t packet_size, int mult, void (*cb)(transfer_t *));

void usb_prepare_transfer(transfer_t *transfer, uint32_t len);
void usb_transmit(int endpoint_number, transfer_t *transfer);
void usb_receive(int endpoint_number, transfer_t *transfer);
uint32_t usb_transfer_status(const transfer_t *transfer);

void usb_start_sof_interrupts(int interface);
void usb_stop_sof_interrupts(int interface);

extern void (*usb_timer0_callback)(void);
extern void (*usb_timer1_callback)(void);
extern void usb_rx_cmd_ack(transfer_t *t);
extern void usb_rx_dout_ack(transfer_t *t);
int tx_uas_response(transfer_t *xfer, int ep, int len);
extern int usb_uas_interface_alt;

static inline void *transfer_buffer(struct transfer_struct *t)
{
        return (void *)(t->pointer0 & ~0xfff);
}

static inline int transfer_length(transfer_t *t)
{
	return rx_packet_size - ((t->status >> 16) & 0x7FFF);
}

struct transfer_struct *get_frame(struct transfer_struct **list);
struct transfer_struct *get_frame_noblock(struct transfer_struct **list);
void put_frame(struct transfer_struct **list, struct transfer_struct *t);

#ifdef __cplusplus
}
#endif
