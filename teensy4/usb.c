#include "imxrt.h"
#include "usb_dev.h"
#define USB_DESC_LIST_DEFINE
#include "usb_desc.h"
#include "core_pins.h" // for delay()
#include "avr/pgmspace.h"
#include <string.h>
#include "debug/printf.h"
#include "scsi.h"

typedef struct endpoint_struct endpoint_t;

struct endpoint_struct {
	uint32_t config;
	uint32_t current;
	transfer_t *next;
	uint32_t status;
	uint32_t pointer0;
	uint32_t pointer1;
	uint32_t pointer2;
	uint32_t pointer3;
	uint32_t pointer4;
	uint32_t reserved;
	uint32_t setup0;
	uint32_t setup1;
	transfer_t *first_transfer;
	transfer_t *last_transfer;
	void (*callback_function)(transfer_t *completed_transfer);
	uint32_t unused1;
};

endpoint_t endpoint_queue_head[(NUM_ENDPOINTS+1)*2] __attribute__ ((used, aligned(4096)));
transfer_t endpoint0_transfer_data __attribute__ ((used, aligned(32)));
transfer_t endpoint0_transfer_ack  __attribute__ ((used, aligned(32)));
extern volatile uint8_t usb_high_speed;
uint16_t tx_packet_size = 0;
uint16_t rx_packet_size = 0;
#define RX_CMD_NUM 4
#define RX_DOUT_NUM 8
#define TX_NUM 8
static transfer_t rx_cmd_transfer[RX_CMD_NUM] __attribute__ ((used, aligned(32)));
static transfer_t rx_dout_transfer[RX_DOUT_NUM] __attribute__ ((used, aligned(32)));
static transfer_t tx_transfer[TX_NUM] __attribute__((used, aligned(32)));
DMAMEM static uint8_t rx_cmd_buf[RX_CMD_NUM][16384] __attribute__ ((used, aligned(4096)));
DMAMEM static uint8_t rx_dout_buf[RX_DOUT_NUM][16384] __attribute__ ((used, aligned(4096)));
DMAMEM static uint8_t txbuf[TX_NUM][16384] __attribute__ ((used, aligned(4096)));

static uint32_t endpoint0_notify_mask = 0;
static uint32_t endpointN_notify_mask = 0;

volatile uint8_t usb_configuration = 0; // non-zero when USB host as configured device
volatile uint8_t usb_high_speed = 0;    // non-zero if running at 480 Mbit/sec speed
static uint8_t endpoint0_buffer[8];
extern uint8_t usb_descriptor_buffer[]; // defined in usb_desc.c
extern const uint8_t usb_config_descriptor_480[];
extern const uint8_t usb_config_descriptor_12[];

static uint8_t reply_buffer[8];
int usb_uas_interface_alt;

transfer_t *tx_free_list, *rx_cmd_busy_list, *rx_dout_busy_list;

typedef union {
	struct {
		union {
			struct {
				uint8_t bmRequestType;
				uint8_t bRequest;
			};
			uint16_t wRequestAndType;
		};
		uint16_t wValue;
		uint16_t wIndex;
		uint16_t wLength;
	};
	struct {
		uint32_t word1;
		uint32_t word2;
	};
	uint64_t bothwords;
} setup_t;

static void show_desc(const char *prefix, transfer_t *t)
{
	printf("%s %08x: NEXT %08x STATUS %08x P0 %08x P1 %08x P2 %08x P3 %08x P4 %08x PARAM %08x\n",
	       prefix, t, t->next, t->status, t->pointer0, t->pointer1,
	       t->pointer2, t->pointer3, t->pointer4, t->callback_param);
}

void show_rx_descs(void)
{
	int i;
	for(i = 0; i < RX_CMD_NUM; i++)
		show_desc("RX", rx_cmd_transfer + i);
}

void show_tx_descs(void)
{
	int i;
	for(i = 0; i < TX_NUM; i++)
		show_desc("TX", tx_transfer + i);
}

void dump_frames(const char *prefix, struct transfer_struct *head)
{
	printf("%s: %s:", __func__, prefix);
	do {
		printf(" -> %x", head);
	} while(head && (head = head->next));
	printf("\n");
}

struct transfer_struct *get_frame(struct transfer_struct **list)
{
	struct transfer_struct *ret = NULL;
//	printf("%s\n", __func__);
//	dump_frames("X", *list);
	do {
		__disable_irq();
		if (*list) {
			ret = *list;
			*list = (*list)->next;
		}
		__enable_irq();
	} while(!ret);
//	printf("%s: %08x\n", __func__, ret);
	return ret;
}

struct transfer_struct *get_frame_noblock(struct transfer_struct **list)
{
	struct transfer_struct *ret = NULL;
	__disable_irq();
	if (*list) {
		ret = *list;
		*list = (*list)->next;
	}
	__enable_irq();
	return ret;
}

void put_frame(struct transfer_struct **list, struct transfer_struct *t)
{
	struct transfer_struct *tmp;
//	printf("%s: %08x\n", __func__, t);

	__disable_irq();
	if (!*list) {
		t->next = 0;
		*list = t;
		goto out;
	}

	tmp = *list;
	do {
		if (!tmp->next) {
			t->next = 0;
			tmp->next = t;
			break;
		}
	} while((tmp = tmp->next));
out:
//	dump_frames("X", *list);
	__enable_irq();
}

void usb_rx_cmd_ack(struct transfer_struct *t)
{
	usb_prepare_transfer(t, rx_packet_size);
	arm_dcache_delete(transfer_buffer(t), rx_packet_size);
	usb_receive(UAS_CMD_ENDPOINT, t);
}

void usb_rx_dout_ack(struct transfer_struct *t)
{
	usb_prepare_transfer(t, rx_packet_size);
	arm_dcache_delete(transfer_buffer(t), rx_packet_size);
	usb_receive(UAS_DOUT_ENDPOINT, t);
}

int tx_uas_response(transfer_t *xfer, int ep, int len)
{
	usb_prepare_transfer(xfer, len);
	if (len)
		arm_dcache_flush_delete(transfer_buffer(xfer), len);
	usb_transmit(ep, xfer);

	uint32_t status = usb_transfer_status(xfer);
	if (!(status & 0x80)) {
		if (status & 0x68) {
			// TODO: what if status has errors???
			printf("ERROR status = %x, ms=%u\n",
			       status, systick_millis_count);
		}
	}
	return 0;
}

static void rx_cmd_event(transfer_t *t)
{
	put_frame(&rx_cmd_busy_list, t);
}

// called by USB interrupt when any packet is received
static void rx_dout_event(transfer_t *t)
{
	put_frame(&rx_dout_busy_list, t);
}

static void tx_complete(transfer_t *t)
{
	put_frame(&tx_free_list, t);
}

static void usb_msc_configure(void)
{
	int i;

	if (usb_high_speed) {
		tx_packet_size = UAS_TX_SIZE_480;
		rx_packet_size = UAS_RX_SIZE_480;
	} else {
		tx_packet_size = UAS_TX_SIZE_12;
		rx_packet_size = UAS_RX_SIZE_12;
	}

	memset(tx_transfer, 0, sizeof(tx_transfer));
	memset(rx_cmd_transfer, 0, sizeof(rx_cmd_transfer));
	memset(rx_dout_transfer, 0, sizeof(rx_dout_transfer));
	memset(rx_cmd_buf, 0, sizeof(rx_cmd_buf));
	memset(rx_dout_buf, 0, sizeof(rx_dout_buf));
	memset(txbuf, 0, sizeof(txbuf));

	for(i = 0; i < TX_NUM; i++) {
		struct transfer_struct *t = tx_transfer + i;
		t->pointer0 = 0 * 4096 + (uint32_t)(txbuf + i);
		t->pointer1 = 1 * 4096 + (uint32_t)(txbuf + i);
		t->pointer2 = 2 * 4096 + (uint32_t)(txbuf + i);
		t->pointer3 = 3 * 4096 + (uint32_t)(txbuf + i);
		put_frame(&tx_free_list, t);;
	}

	usb_config_rx(UAS_CMD_ENDPOINT, rx_packet_size, 0, rx_cmd_event); // size same 12 & 480
	usb_config_rx(UAS_DOUT_ENDPOINT, rx_packet_size, 0, rx_dout_event); // size same 12 & 480
	usb_config_tx(UAS_STAT_ENDPOINT, tx_packet_size, 0, tx_complete);
	usb_config_tx(UAS_DIN_ENDPOINT, tx_packet_size, 0, tx_complete);

	for (i = 0; i < RX_CMD_NUM; i++) {
		struct transfer_struct *t = rx_cmd_transfer + i;
		t->pointer0 = 0 * 4096 + (uint32_t)(rx_cmd_buf + i);
		t->pointer1 = 1 * 4096 + (uint32_t)(rx_cmd_buf + i);
		t->pointer2 = 2 * 4096 + (uint32_t)(rx_cmd_buf + i);
		t->pointer3 = 3 * 4096 + (uint32_t)(rx_cmd_buf + i);
		usb_rx_cmd_ack(t);
	}

	for (i = 0; i < RX_DOUT_NUM; i++) {
		struct transfer_struct *t = rx_dout_transfer + i;
		t->pointer0 = 0 * 4096 + (uint32_t)(rx_dout_buf + i);
		t->pointer1 = 1 * 4096 + (uint32_t)(rx_dout_buf + i);
		t->pointer2 = 2 * 4096 + (uint32_t)(rx_dout_buf + i);
		t->pointer3 = 3 * 4096 + (uint32_t)(rx_dout_buf + i);
		usb_rx_dout_ack(t);
	}

}

static void usb_flush(void)
{
	uint32_t mask = (1 << UAS_DIN_ENDPOINT) | \
		(1 << UAS_DOUT_ENDPOINT) |	  \
		(1 << UAS_CMD_ENDPOINT) |	  \
		(1 << UAS_STAT_ENDPOINT);

	mask |= (mask << 16);

	int i = 0;
	do {
		USB1_ENDPTFLUSH |= mask;
		while(USB1_ENDPTFLUSH & mask);
	} while((USB1_ENDPTSTATUS & mask) && i++ < 10);

	tx_free_list = NULL;
	rx_cmd_busy_list = NULL;
	rx_dout_busy_list = NULL;
}

static void run_callbacks(endpoint_t *ep)
{
	transfer_t *first = ep->first_transfer;

	if (first == NULL)
		return;

	// count how many transfers are completed, then remove them from the endpoint's list
	uint32_t count = 0;
	transfer_t *t = first;
	while (1) {
		if (t->status & (1<<7)) {
			ep->first_transfer = t;
			break;
		}
		count++;
		t = (transfer_t *)t->next;
		if ((uint32_t)t == 1) {
			ep->first_transfer = NULL;
			ep->last_transfer = NULL;
			break;
		}
	}
	// do all the callbacks
	while (count) {
		transfer_t *next = (transfer_t *)first->next;
		ep->callback_function(first);
		first = next;
		count--;
	}
}

static void endpoint0_transmit(const void *data, uint32_t len, int notify)
{
	if (len > 0) {
		// Executing A Transfer Descriptor, page 3182
		endpoint0_transfer_data.next = (transfer_t *)1;
		endpoint0_transfer_data.status = (len << 16) | (1<<7);
		uint32_t addr = (uint32_t)data;
		endpoint0_transfer_data.pointer0 = addr; // format: table 55-60, pg 3159
		endpoint0_transfer_data.pointer1 = addr + 4096;
		endpoint0_transfer_data.pointer2 = addr + 8192;
		endpoint0_transfer_data.pointer3 = addr + 12288;
		endpoint0_transfer_data.pointer4 = addr + 16384;
		//  Case 1: Link list is empty, page 3182
		endpoint_queue_head[1].next = &endpoint0_transfer_data;
		endpoint_queue_head[1].status = 0;
		USB1_ENDPTPRIME |= (1<<16);
		while (USB1_ENDPTPRIME) ;
	}
	endpoint0_transfer_ack.next = (transfer_t *)1;
	endpoint0_transfer_ack.status = (1<<7) | (notify ? (1 << 15) : 0);
	endpoint0_transfer_ack.pointer0 = 0;
	endpoint_queue_head[0].next = &endpoint0_transfer_ack;
	endpoint_queue_head[0].status = 0;
	USB1_ENDPTCOMPLETE = (1<<0) | (1<<16);
	USB1_ENDPTPRIME |= (1<<0);
	endpoint0_notify_mask = (notify ? (1 << 0) : 0);
	while (USB1_ENDPTPRIME) ;
}

static void endpoint0_receive(void *data, uint32_t len, int notify)
{
	if (len > 0) {
		// Executing A Transfer Descriptor, page 3182
		endpoint0_transfer_data.next = (transfer_t *)1;
		endpoint0_transfer_data.status = (len << 16) | (1<<7);
		uint32_t addr = (uint32_t)data;
		endpoint0_transfer_data.pointer0 = addr; // format: table 55-60, pg 3159
		endpoint0_transfer_data.pointer1 = addr + 4096;
		endpoint0_transfer_data.pointer2 = addr + 8192;
		endpoint0_transfer_data.pointer3 = addr + 12288;
		endpoint0_transfer_data.pointer4 = addr + 16384;
		//  Case 1: Link list is empty, page 3182
		endpoint_queue_head[0].next = &endpoint0_transfer_data;
		endpoint_queue_head[0].status = 0;
		USB1_ENDPTPRIME |= (1<<0);
		while (USB1_ENDPTPRIME) ;
	}
	endpoint0_transfer_ack.next = (transfer_t *)1;
	endpoint0_transfer_ack.status = (1<<7) | (notify ? (1 << 15) : 0);
	endpoint0_transfer_ack.pointer0 = 0;
	endpoint_queue_head[1].next = &endpoint0_transfer_ack;
	endpoint_queue_head[1].status = 0;
	USB1_ENDPTCOMPLETE = (1<<0) | (1<<16);
	USB1_ENDPTPRIME |= (1<<16);
	endpoint0_notify_mask = (notify ? (1 << 16) : 0);
	while (USB1_ENDPTPRIME) ;
}

static void endpoint0_complete(void)
{
}

static void endpoint0_setup(uint64_t setupdata)
{
	setup_t setup;
	uint32_t endpoint, dir, ctrl;
	const usb_descriptor_list_t *list;

	setup.bothwords = setupdata;
	switch (setup.wRequestAndType) {
	  case 0x0500: // SET_ADDRESS
		endpoint0_receive(NULL, 0, 0);
		USB1_DEVICEADDR = USB_DEVICEADDR_USBADR(setup.wValue) | USB_DEVICEADDR_USBADRA;
		return;
	  case 0x0900: // SET_CONFIGURATION
		usb_flush();
		scsi_reset();
		usb_configuration = setup.wValue;
		// configure all other endpoints
		#if defined(ENDPOINT2_CONFIG)
		USB1_ENDPTCTRL2 = ENDPOINT2_CONFIG;
		#endif
		#if defined(ENDPOINT3_CONFIG)
		USB1_ENDPTCTRL3 = ENDPOINT3_CONFIG;
		#endif
		#if defined(ENDPOINT4_CONFIG)
		USB1_ENDPTCTRL4 = ENDPOINT4_CONFIG;
		#endif
		#if defined(ENDPOINT5_CONFIG)
		USB1_ENDPTCTRL5 = ENDPOINT5_CONFIG;
		#endif
		#if defined(ENDPOINT6_CONFIG)
		USB1_ENDPTCTRL6 = ENDPOINT6_CONFIG;
		#endif
		#if defined(ENDPOINT7_CONFIG)
		USB1_ENDPTCTRL7 = ENDPOINT7_CONFIG;
		#endif
		usb_msc_configure();
		endpoint0_receive(NULL, 0, 0);
		return;
	  case 0x0880: // GET_CONFIGURATION
		reply_buffer[0] = usb_configuration;
		endpoint0_transmit(reply_buffer, 1, 0);
		return;
	  case 0x0080: // GET_STATUS (device)
		reply_buffer[0] = 0;
		reply_buffer[1] = 0;
		endpoint0_transmit(reply_buffer, 2, 0);
		return;
	  case 0x0082: // GET_STATUS (endpoint)
		endpoint = setup.wIndex & 0x7F;
		if (endpoint > 7) break;
		dir = setup.wIndex & 0x80;
		ctrl = *((uint32_t *)&USB1_ENDPTCTRL0 + endpoint);
		reply_buffer[0] = 0;
		reply_buffer[1] = 0;
		if ((dir && (ctrl & USB_ENDPTCTRL_TXS)) || (!dir && (ctrl & USB_ENDPTCTRL_RXS))) {
			reply_buffer[0] = 1;
		}
		endpoint0_transmit(reply_buffer, 2, 0);
		return;
	  case 0x0302: // SET_FEATURE (endpoint)
		endpoint = setup.wIndex & 0x7F;
		if (endpoint > 7) break;
		dir = setup.wIndex & 0x80;
		if (dir) {
			*((volatile uint32_t *)&USB1_ENDPTCTRL0 + endpoint) |= USB_ENDPTCTRL_TXS;
		} else {
			*((volatile uint32_t *)&USB1_ENDPTCTRL0 + endpoint) |= USB_ENDPTCTRL_RXS;
		}
		endpoint0_receive(NULL, 0, 0);
		return;
	  case 0x0102: // CLEAR_FEATURE (endpoint)
		endpoint = setup.wIndex & 0x7F;
		if (endpoint > 7) break;
		dir = setup.wIndex & 0x80;
		if (dir) {
			*((volatile uint32_t *)&USB1_ENDPTCTRL0 + endpoint) &= ~USB_ENDPTCTRL_TXS;
		} else {
			*((volatile uint32_t *)&USB1_ENDPTCTRL0 + endpoint) &= ~USB_ENDPTCTRL_RXS;
		}
		endpoint0_receive(NULL, 0, 0);
		return;
	  case 0x0680: // GET_DESCRIPTOR
	  case 0x0681:
		for (list = usb_descriptor_list; list->addr != NULL; list++) {
			if (setup.wValue == list->wValue && setup.wIndex == list->wIndex) {
				uint32_t datalen;
				if ((setup.wValue >> 8) == 3) {
					// for string descriptors, use the descriptor's
					// length field, allowing runtime configured length.
					datalen = *(list->addr);
				} else {
					datalen = list->length;
				}
				if (datalen > setup.wLength) datalen = setup.wLength;

				// copy the descriptor, from PROGMEM to DMAMEM
				if (setup.wValue == 0x200) {
					// config descriptor needs to adapt to speed
					const uint8_t *src = usb_config_descriptor_12;
					if (usb_high_speed) src = usb_config_descriptor_480;
					memcpy(usb_descriptor_buffer, src, datalen);
				} else if (setup.wValue == 0x700) {
					// other speed config also needs to adapt
					const uint8_t *src = usb_config_descriptor_480;
					if (usb_high_speed) src = usb_config_descriptor_12;
					memcpy(usb_descriptor_buffer, src, datalen);
					usb_descriptor_buffer[1] = 7;
				} else {
					memcpy(usb_descriptor_buffer, list->addr, datalen);
				}
				// prep transmit
				arm_dcache_flush_delete(usb_descriptor_buffer, datalen);
				endpoint0_transmit(usb_descriptor_buffer, datalen, 0);
				return;
			}
		}
		break;
	  case 0x0B01: // SET_INTERFACE (alternate setting)
		  if (setup.wIndex > 1)
			  break;
		  usb_uas_interface_alt = setup.wValue;
		  endpoint0_receive(NULL, 0, 0);
		  return;
	  case 0x0A81: // GET_INTERFACE (alternate setting)
		  endpoint0_buffer[0] = usb_uas_interface_alt;
		  endpoint0_transmit(endpoint0_buffer, 1, 0);
		  return;
	}
        USB1_ENDPTCTRL0 = 0x000010001; // stall
}

static void isr(void)
{
	uint32_t status = USB1_USBSTS;
	USB1_USBSTS = status;

	if (status & USB_USBSTS_UI) {
		uint32_t setupstatus = USB1_ENDPTSETUPSTAT;

		while (setupstatus) {
			USB1_ENDPTSETUPSTAT = setupstatus;
			setup_t s;
			do {
				USB1_USBCMD |= USB_USBCMD_SUTW;
				s.word1 = endpoint_queue_head[0].setup0;
				s.word2 = endpoint_queue_head[0].setup1;
				printf(":");
			} while (!(USB1_USBCMD & USB_USBCMD_SUTW));
			USB1_USBCMD &= ~USB_USBCMD_SUTW;
			printf("setup %08lX %08lX\n", s.word1, s.word2);
			USB1_ENDPTFLUSH = (1<<16) | (1<<0); // page 3174
			while (USB1_ENDPTFLUSH & ((1<<16) | (1<<0))) ;
			endpoint0_notify_mask = 0;
			endpoint0_setup(s.bothwords);
			setupstatus = USB1_ENDPTSETUPSTAT; // page 3175
		}

		uint32_t completestatus = USB1_ENDPTCOMPLETE;
		if (completestatus) {
			USB1_ENDPTCOMPLETE = completestatus;
			if (completestatus & endpoint0_notify_mask) {
				endpoint0_notify_mask = 0;
				endpoint0_complete();
			}
			completestatus &= endpointN_notify_mask;

			if (completestatus) {
				// transmit:
				uint32_t tx = completestatus >> 16;
				while (tx) {
					int p=__builtin_ctz(tx);
					run_callbacks(endpoint_queue_head + p * 2 + 1);
					tx &= ~(1 << p);
				}

				// receive:
				uint32_t rx = completestatus & 0xffff;
				while(rx) {
					int p=__builtin_ctz(rx);
					run_callbacks(endpoint_queue_head + p * 2);
					rx &= ~(1 << p);
				};
			}
		}
	}

	if (status & USB_USBSTS_URI) { // page 3164
		USB1_ENDPTSETUPSTAT = USB1_ENDPTSETUPSTAT; // Clear all setup token semaphores
		USB1_ENDPTCOMPLETE = USB1_ENDPTCOMPLETE; // Clear all the endpoint complete status
		while (USB1_ENDPTPRIME != 0) ; // Wait for any endpoint priming
		USB1_ENDPTFLUSH = 0xFFFFFFFF;  // Cancel all endpoint primed status
		endpointN_notify_mask = 0;
	}

	if (status & USB_USBSTS_PCI)
		usb_high_speed = !!(USB1_PORTSC1 & USB_PORTSC1_HSP);
}

static void usb_endpoint_config(endpoint_t *qh, uint32_t config, void (*callback)(transfer_t *))
{
	memset(qh, 0, sizeof(endpoint_t));
	qh->config = config;
	qh->next = (transfer_t *)1; // Terminate bit = 1
	qh->callback_function = callback;
}

void usb_config_rx(uint32_t ep, uint32_t packet_size, int do_zlp, void (*cb)(transfer_t *))
{
	uint32_t config = (packet_size << 16) | (do_zlp ? 0 : (1 << 29));
	if (ep < 2 || ep > NUM_ENDPOINTS) return;
	usb_endpoint_config(endpoint_queue_head + ep * 2, config, cb);
	if (cb) endpointN_notify_mask |= (1 << ep);
}

void usb_config_tx(uint32_t ep, uint32_t packet_size, int do_zlp, void (*cb)(transfer_t *))
{
	uint32_t config = (packet_size << 16) | (do_zlp ? 0 : (1 << 29));
	if (ep < 2 || ep > NUM_ENDPOINTS) return;
	usb_endpoint_config(endpoint_queue_head + ep * 2 + 1, config, cb);
	if (cb) endpointN_notify_mask |= (1 << (ep + 16));
}

void usb_config_rx_iso(uint32_t ep, uint32_t packet_size, int mult, void (*cb)(transfer_t *))
{
	if (mult < 1 || mult > 3) return;
	uint32_t config = (packet_size << 16) | (mult << 30);
	if (ep < 2 || ep > NUM_ENDPOINTS) return;
	usb_endpoint_config(endpoint_queue_head + ep * 2, config, cb);
	if (cb) endpointN_notify_mask |= (1 << ep);
}

void usb_config_tx_iso(uint32_t ep, uint32_t packet_size, int mult, void (*cb)(transfer_t *))
{
	if (mult < 1 || mult > 3) return;
	uint32_t config = (packet_size << 16) | (mult << 30);
	if (ep < 2 || ep > NUM_ENDPOINTS) return;
	usb_endpoint_config(endpoint_queue_head + ep * 2 + 1, config, cb);
	if (cb) endpointN_notify_mask |= (1 << (ep + 16));
}

void usb_prepare_transfer(transfer_t *transfer, uint32_t len)
{
	transfer->next = (transfer_t *)1;
	transfer->status = (len << 16) | (1<<7);
	transfer->callback_param = transfer;
	transfer->pointer0 &= ~0xfff;
	transfer->pointer1 &= ~0xfff;
}

static void schedule_transfer(endpoint_t *endpoint, uint32_t epmask, transfer_t *transfer)
{
	volatile uint32_t status;

	transfer->status |= (1<<15);

	__disable_irq();
	transfer_t *last = endpoint->last_transfer;
	if (last) {
		last->next = transfer;

		if (USB1_ENDPTPRIME & epmask)
			goto end;

		do {
			USB1_USBCMD |= USB_USBCMD_ATDTW;
			status = USB1_ENDPTSTATUS;
		} while (!(USB1_USBCMD & USB_USBCMD_ATDTW));

		USB1_USBCMD &= ~USB_USBCMD_ATDTW;

		if (status & epmask)
			goto end;
	}

	endpoint->next = transfer;
	endpoint->status = 0;
	USB1_ENDPTPRIME |= epmask;
	endpoint->first_transfer = transfer;
end:
	endpoint->last_transfer = transfer;
	__enable_irq();
}

void usb_transmit(int endpoint_number, transfer_t *transfer)
{
	if (endpoint_number < 2 || endpoint_number > NUM_ENDPOINTS)
		return;

	endpoint_t *endpoint = endpoint_queue_head + endpoint_number * 2 + 1;
	uint32_t mask = 1 << (endpoint_number + 16);

	schedule_transfer(endpoint, mask, transfer);
}

void usb_receive(int endpoint_number, transfer_t *transfer)
{
	if (endpoint_number < 2 || endpoint_number > NUM_ENDPOINTS)
		return;

	endpoint_t *endpoint = endpoint_queue_head + endpoint_number * 2;
	uint32_t mask = 1 << endpoint_number;

	schedule_transfer(endpoint, mask, transfer);
}

uint32_t usb_transfer_status(const transfer_t *transfer)
{
	return transfer->status;
}

FLASHMEM void usb_init(void)
{
	PMU_REG_3P0 = PMU_REG_3P0_OUTPUT_TRG(0x0F) | PMU_REG_3P0_BO_OFFSET(6)
		| PMU_REG_3P0_ENABLE_LINREG;

	usb_init_serialnumber();
	CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON); // turn on clocks to USB peripheral
	USB1_BURSTSIZE = 0x0404;

	if ((USBPHY1_PWD & (USBPHY_PWD_RXPWDRX | USBPHY_PWD_RXPWDDIFF | USBPHY_PWD_RXPWD1PT1
	  | USBPHY_PWD_RXPWDENV | USBPHY_PWD_TXPWDV2I | USBPHY_PWD_TXPWDIBIAS
	  | USBPHY_PWD_TXPWDFS)) || (USB1_USBMODE & USB_USBMODE_CM_MASK)) {
		// USB controller is turned on from previous use
		// reset needed to turn it off & start from clean slate
		USBPHY1_CTRL_SET = USBPHY_CTRL_SFTRST; // USBPHY1_CTRL page 3292
		USB1_USBCMD |= USB_USBCMD_RST; // reset controller
		int count=0;
		while (USB1_USBCMD & USB_USBCMD_RST) count++;
		NVIC_CLEAR_PENDING(IRQ_USB1);
		USBPHY1_CTRL_CLR = USBPHY_CTRL_SFTRST; // reset PHY
		delay(25);
	}

	USBPHY1_CTRL_CLR = USBPHY_CTRL_CLKGATE;
	USBPHY1_PWD = 0;
	USB1_USBMODE = USB_USBMODE_CM(2) | USB_USBMODE_SLOM;
	memset(endpoint_queue_head, 0, sizeof(endpoint_queue_head));
	endpoint_queue_head[0].config = (64 << 16) | (1 << 15);
	endpoint_queue_head[1].config = (64 << 16);
	USB1_ENDPOINTLISTADDR = (uint32_t)&endpoint_queue_head;
	USB1_USBINTR = USB_USBINTR_UE | USB_USBINTR_UEE | /* USB_USBINTR_PCE | */
		USB_USBINTR_URE | USB_USBINTR_SLE;
	//_VectorsRam[IRQ_USB1+16] = &isr;
	attachInterruptVector(IRQ_USB1, &isr);
	NVIC_ENABLE_IRQ(IRQ_USB1);
	USB1_USBCMD = USB_USBCMD_RS;
}
