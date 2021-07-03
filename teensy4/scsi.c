#include <scsi.h>
#include <core_pins.h>
#include <stdio.h>
#include "usb_dev.h"
#include "usb_desc.h"
#include "wiring.h"
#include <Arduino.h>

#define BSYI_PIN 2
#define BSYO_PIN 3
#define SELI_PIN 6
#define CDO_PIN 7
#define IOO_PIN 8
#define REQI_PIN 9
#define IOI_PIN 10
#define MSGI_PIN 11
#define CDI_PIN 12
#define ACKO_PIN 24
#define REQO_PIN 25
#define DB6O_PIN 26
#define DB7O_PIN 27
#define RSTI_PIN 28
#define DBPI_PIN 30
#define DBPO_PIN 31
#define ACKI_PIN 32
#define SELO_PIN 33
#define ATNI_PIN 34
#define MSGO_PIN 35
#define RSTO_PIN 36
#define ATNO_PIN 37
#define DB4O_PIN 38
#define DB5O_PIN 39
#define DB4I_PIN 40
#define DB5I_PIN 41
#define DB2I_PIN 14
#define DB3I_PIN 15
#define DB7I_PIN 16
#define DB6I_PIN 17
#define DB1I_PIN 18
#define DB0I_PIN 19
#define DB2O_PIN 20
#define DB3O_PIN 21
#define DB0O_PIN 22
#define DB1O_PIN 23

#define LED_PIN 13
#define SCSI_BUS_CLEAR_DELAY 800
#define SCSI_ARBITRATION_DELAY 2400
#define SCSI_BUS_SETTLE_DELAY 400

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))

struct scsi_ctx {
	unsigned int support_identify:1;
	unsigned int support_tags:1;
	unsigned int support_sdtr:1;
	unsigned int support_disconnect:1;
	int hostid;
	int hostidmsk;
	int targetid;
} sctx;

struct scsi_tag {
	uint32_t host_tag;
	uint8_t tag;
	int valid:1;
	int sent_read_ready:1;
	int sent_write_ready:1;
} scsi_tags[256];

typedef enum {
	SCSI_PHASE_MIN,
	SCSI_PHASE_MOUT,
	SCSI_PHASE_5,
	SCSI_PHASE_4,
	SCSI_PHASE_STATUS,
	SCSI_PHASE_CMD,
	SCSI_PHASE_DIN,
	SCSI_PHASE_DOUT,
} scsi_phase_t;

const char *phase_names[] = {
	"MSG IN",
	"MSG OUT",
	"5",
	"4",
	"STATUS",
	"COMMAND",
	"DATA IN",
	"DATA OUT",
};

struct usb_msc_cbw {
	uint32_t signature;
	uint32_t tag;
	uint32_t datalen;
	uint8_t flags;
	uint8_t lun;
	uint8_t cbwcblen;
	uint8_t cdb[15];
} __attribute__((packed));

struct usb_msc_csw {
	uint32_t signature;
	uint32_t tag;
	uint32_t data_residue;
	uint8_t status;
} __attribute__((packed));

#define SCSI_DEBUG_CMD		1
#define SCSI_DEBUG_PHASE	2
#define SCSI_DEBUG_UAS		4
#define SCSI_DEBUG_MSC		8
#define SCSI_DEBUG_DUMP		16
#define SCSI_DEBUG_ERROR	32
#define SCSI_DEBUG_PIN		64

#define SCSI_DEBUG_ALL		255

#define DEBUG 0//SCSI_DEBUG_ALL

#define SCSI_DEBUG(level, fmt, ...) do {				\
	if (DEBUG & level & SCSI_DEBUG_UAS)					\
		printf("%8ld.%03ld %20.20s:%-d\033[32m " fmt "\033[0m", millis() / 1000, millis() % 1000, __func__, __LINE__, ##__VA_ARGS__); \
	else if (DEBUG & level & SCSI_DEBUG_ERROR)				\
		printf("%8ld.%03ld %20.20s:%-d\033[31m " fmt "\033[0m", millis() / 1000, millis() % 1000, __func__, __LINE__, ##__VA_ARGS__); \
	else if (DEBUG & level & SCSI_DEBUG_PHASE)				\
		printf("%8ld.%03ld %20.20s:%-d\033[36m " fmt "\033[0m", millis() / 1000, millis() % 1000, __func__, __LINE__, ##__VA_ARGS__); \
	else if (DEBUG & level & DEBUG)						\
		printf("%8ld.%03ld %20.20s:%-d\033[37m " fmt "\033[0m", millis() / 1000, millis() % 1000, __func__, __LINE__, ##__VA_ARGS__); \
	} while(0);

#define SCSI_DEBUG_NOH(level, fmt, ...)					\
	if (level & DEBUG)						\
		printf(fmt, ##__VA_ARGS__);

static void scsi_setup_ports(void)
{
	pinMode(SELI_PIN, INPUT);
	pinMode(BSYI_PIN, INPUT);
	pinMode(RSTI_PIN, INPUT);
	pinMode(REQI_PIN, INPUT);
	pinMode(ACKI_PIN, INPUT);
	pinMode(CDI_PIN, INPUT);
	pinMode(IOI_PIN, INPUT);
	pinMode(MSGI_PIN, INPUT);
	pinMode(ATNI_PIN, INPUT);

	pinMode(DB0I_PIN, INPUT);
	pinMode(DB1I_PIN, INPUT);
	pinMode(DB2I_PIN, INPUT);
	pinMode(DB3I_PIN, INPUT);
	pinMode(DB4I_PIN, INPUT);
	pinMode(DB5I_PIN, INPUT);
	pinMode(DB6I_PIN, INPUT);
	pinMode(DB7I_PIN, INPUT);
	pinMode(DBPI_PIN, INPUT);

	digitalWriteFast(SELO_PIN, LOW);
	digitalWriteFast(BSYO_PIN, LOW);
	digitalWriteFast(RSTO_PIN, LOW);
	digitalWriteFast(REQO_PIN, LOW);
	digitalWriteFast(ACKO_PIN, LOW);
	digitalWriteFast(CDO_PIN, LOW);
	digitalWriteFast(IOO_PIN, LOW);
	digitalWriteFast(MSGO_PIN, LOW);
	digitalWriteFast(ATNO_PIN, LOW);

	digitalWriteFast(DB0O_PIN, LOW);
	digitalWriteFast(DB1O_PIN, LOW);
	digitalWriteFast(DB2O_PIN, LOW);
	digitalWriteFast(DB3O_PIN, LOW);
	digitalWriteFast(DB4O_PIN, LOW);
	digitalWriteFast(DB5O_PIN, LOW);
	digitalWriteFast(DB6O_PIN, LOW);
	digitalWriteFast(DB7O_PIN, LOW);
	digitalWriteFast(DBPO_PIN, LOW);

	pinMode(SELO_PIN, OUTPUT);
	pinMode(BSYO_PIN, OUTPUT);
	pinMode(RSTO_PIN, OUTPUT);
	pinMode(REQO_PIN, OUTPUT);
	pinMode(ACKO_PIN, OUTPUT);
	pinMode(CDO_PIN, OUTPUT);
	pinMode(IOO_PIN, OUTPUT);
	pinMode(MSGO_PIN, OUTPUT);
	pinMode(ATNO_PIN, OUTPUT);

	pinMode(DB0O_PIN, OUTPUT);
	pinMode(DB1O_PIN, OUTPUT);
	pinMode(DB2O_PIN, OUTPUT);
	pinMode(DB3O_PIN, OUTPUT);
	pinMode(DB4O_PIN, OUTPUT);
	pinMode(DB5O_PIN, OUTPUT);
	pinMode(DB6O_PIN, OUTPUT);
	pinMode(DB7O_PIN, OUTPUT);
	pinMode(DBPO_PIN, OUTPUT);

	pinMode(LED_PIN, OUTPUT);
}

static uint8_t scsi_get_data(void)
{
	return ~(GPIO6_PSR >> 16);
}

static const int parity_table[256] = {
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
};

#if 0
static struct scsi_tag *scsi_find_tag(uint32_t host_tag)
{
	struct scsi_tag *ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(scsi_tags); i++) {
		ret = scsi_tags + i;
		if (!(ret->valid))
			continue;
		if (ret->host_tag == host_tag)
			return ret;
	}
	return NULL;
}
#endif

static inline uint32_t get_xfer_tag(struct scsi_xfer *xfer)
{
	return xfer->tag ? xfer->tag->host_tag : -1U;
}


static int scsi_insert_tag(uint32_t host_tag)
{
	struct scsi_tag *ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(scsi_tags); i++) {
		ret = scsi_tags + i;
		if (ret->valid)
			continue;
		ret->host_tag = host_tag;
		ret->tag = i;
		ret->valid = 1;
		return i;
	}

	return -1;
}

static struct scsi_tag *scsi_lookup_tag(int tag)
{
	struct scsi_tag *ret;

	if (tag > ARRAY_SIZE(scsi_tags))
		return NULL;

	ret = scsi_tags + tag;
	if (!ret->valid)
		return NULL;
	return ret;
}

static void scsi_free_tag(int tag)
{
	if (tag > ARRAY_SIZE(scsi_tags))
		return;
	memset(scsi_tags + tag, 0, sizeof(struct scsi_tag));
}

static void scsi_set_hiz(void)
{
	GPIO6_DR_CLEAR |= (0xff << 24);
	digitalWriteFast(DBPO_PIN, LOW);
}

static void scsi_set_data(uint8_t data)
{
	GPIO6_DR_CLEAR |= ~data << 24;
	GPIO6_DR_SET |= data << 24;
	digitalWriteFast(DBPO_PIN, parity_table[data]);
}

static __attribute__((unused)) void dump_scsi(const char *prefix)
{
	// XX SEL BSY RST ACK REQ CD IO MSG ATN
	SCSI_DEBUG(SCSI_DEBUG_PIN, "%s: %02X %s%s%s%s%s%s%s%s%s%s\n",
	       prefix, scsi_get_data(),
	       digitalReadFast(SELI_PIN) ? "" : "SEL ",
	       digitalReadFast(BSYI_PIN) ? "" : "BSY ",
	       digitalReadFast(RSTI_PIN) ? "" : "RST ",
	       digitalReadFast(ACKI_PIN) ? "" : "ACK ",
	       digitalReadFast(REQI_PIN) ? "" : "REQ ",
	       digitalReadFast(CDI_PIN) ? "" : "CD ",
	       digitalReadFast(IOI_PIN) ? "" : "IO ",
	       digitalReadFast(MSGI_PIN) ? "" : "MSG ",
	       digitalReadFast(ATNI_PIN) ? "" : "ATN ",
	       digitalReadFast(DBPI_PIN) ? "" : "DBP ");
}

void scsi_reset(void)
{
	digitalWriteFast(RSTO_PIN, HIGH);
	scsi_set_hiz();
        digitalWriteFast(SELO_PIN, LOW);
	digitalWriteFast(BSYO_PIN, LOW);
	digitalWriteFast(ACKO_PIN, LOW);
	digitalWriteFast(REQO_PIN, LOW);
	digitalWriteFast(CDO_PIN, LOW);
	digitalWriteFast(IOO_PIN, LOW);
        digitalWriteFast(MSGO_PIN, LOW);
	digitalWriteFast(ATNO_PIN, LOW);
	delay(10);
	digitalWriteFast(RSTO_PIN, LOW);
	delay(250);
	memset(&scsi_tags, 0, sizeof(scsi_tags));
}

static int scsi_wait_bus_free(void)
{
	for(;;) {
		digitalWriteFast(BSYO_PIN, LOW);
		delayNanoseconds(800); /* Bus clear delay */
		while(!(digitalReadFast(SELI_PIN) & digitalReadFast(BSYI_PIN)));
		/* start arbitration */
		digitalWriteFast(BSYO_PIN, HIGH);
		scsi_set_data(sctx.hostidmsk);
		delayNanoseconds(2400); /* Arbitration delay */
		if (!(scsi_get_data() & (sctx.hostidmsk-1)))
			break;
	}
	delayNanoseconds(SCSI_BUS_CLEAR_DELAY);
	return 0;
}

static int scsi_select(struct scsi_xfer *xfer, int id)
{
	int i = 250000000 / SCSI_BUS_SETTLE_DELAY;

	digitalWriteFast(SELO_PIN, HIGH);
	if (xfer->outmsgcnt)
		digitalWriteFast(ATNO_PIN, HIGH);
	delayNanoseconds(10 * SCSI_BUS_SETTLE_DELAY);
        scsi_set_data(sctx.hostidmsk | (1 << id));
	delayNanoseconds(10 * SCSI_BUS_SETTLE_DELAY);
	digitalWriteFast(BSYO_PIN, LOW);
	delayNanoseconds(10 * SCSI_BUS_SETTLE_DELAY);

        while (i-- > 0 && digitalReadFast(BSYI_PIN))
		delayNanoseconds(SCSI_BUS_SETTLE_DELAY);

	if (i < 0) {
		SCSI_DEBUG(SCSI_DEBUG_PHASE, "select failed\n");
		digitalWriteFast(SELO_PIN, LOW);
		return 1;
	}
	xfer->id = id;
        digitalWriteFast(SELO_PIN, LOW);
	delayNanoseconds(SCSI_BUS_SETTLE_DELAY);
	SCSI_DEBUG(SCSI_DEBUG_PHASE, "selected target %d\n", id);
	return 0;
}

static scsi_phase_t scsi_get_phase(void)
{
	return GPIO7_PSR & 7;
}

static unsigned int get_cdb_len(struct scsi_xfer *xfer)
{
	return 16; // XXX xfer->cdblen;
}

static void scsi_ack_async(void)
{
	digitalWriteFast(ACKO_PIN, HIGH);
	while(!digitalReadFast(REQI_PIN));
	digitalWriteFast(ACKO_PIN, LOW);
}

static void scsi_handle_cmd(struct scsi_xfer *xfer)
{
	unsigned int i;
	uint8_t *cdb = xfer->cdb;
	SCSI_DEBUG(SCSI_DEBUG_CMD, "%lx: CDB: ", get_xfer_tag(xfer));
	for(i = 0; i < get_cdb_len(xfer);) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;
		if (scsi_get_phase() != SCSI_PHASE_CMD)
			break;

		SCSI_DEBUG_NOH(SCSI_DEBUG_CMD, " %02X", cdb[i]);
		scsi_set_data(cdb[i++]);
		scsi_ack_async();
	}
	scsi_set_hiz();
	SCSI_DEBUG_NOH(SCSI_DEBUG_CMD, "\n");
}

static void scsi_handle_msgout(struct scsi_xfer *xfer)
{
	SCSI_DEBUG(SCSI_DEBUG_DUMP, "%lx: MOUT: (%d/%d)", get_xfer_tag(xfer),
		   xfer->outmsgpos, xfer->outmsgcnt);

	while(xfer->outmsgpos < xfer->outmsgcnt) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;

		if (scsi_get_phase() != SCSI_PHASE_MOUT)
			break;

		if (xfer->outmsgpos + 1 == xfer->outmsgcnt) {
			digitalWriteFast(ATNO_PIN, LOW);
		}

		uint8_t msg = xfer->outmsgs[xfer->outmsgpos++];
		SCSI_DEBUG_NOH(SCSI_DEBUG_DUMP, " %02X", msg);
		scsi_set_data(msg);
		scsi_ack_async();
	}
	scsi_set_hiz();
	SCSI_DEBUG_NOH(SCSI_DEBUG_DUMP, "\n");
}

static int scsi_msg_length(const uint8_t *msg, int len)
{
	switch(msg[0]) {
	case 0x01:
		if (len > 0)
			return msg[1]+2;
		else
			return 0;
	case 0x20 ... 0x2f:
		return 2;
	default:
		return 1;
	}
	return 0;
}

static void scsi_handle_rejected_msg(struct scsi_xfer *xfer)
{
	int i, len = 0, pos = 0;
	uint8_t msg;

	for (i = 0; i < xfer->outmsgcnt; i++) {
		len = scsi_msg_length(xfer->outmsgs + pos, xfer->outmsgcnt - pos);
		if (pos + len >= xfer->outmsgpos)
			break;
		pos += len;
	}
	msg = xfer->outmsgs[pos];
	if ((msg & ~0x47) == SCSI_MSG_IDENTIFY)
		sctx.support_identify = 0;
	else if (msg == SCSI_MSG_SIMPLE_TAG)
		sctx.support_tags = 0;
	if (len)
		memset(xfer->outmsgs + pos, SCSI_MSG_NOP, len);
}

static void scsi_handle_msgin(struct scsi_xfer *xfer)
{

	uint8_t tmp, *p, *msg = xfer->inmsgs;
	int len;

	xfer->inmsgcnt = 0;
	SCSI_DEBUG(SCSI_DEBUG_DUMP, "MSGIN: ");
	for(;;) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;

		delayNanoseconds(5);

		if (scsi_get_phase() != SCSI_PHASE_MIN)
			break;

		tmp = scsi_get_data();

		SCSI_DEBUG_NOH(SCSI_DEBUG_DUMP, " %02X", tmp);
		if (xfer->inmsgcnt < 16) {
			*msg++ = tmp;
			xfer->inmsgcnt++;
		}

		scsi_ack_async();
	}
	SCSI_DEBUG_NOH(SCSI_DEBUG_DUMP, "\n");

	p = xfer->inmsgs;
	for(;;) {
		int rem = xfer->inmsgcnt - (p - xfer->inmsgs);
		if (rem <= 0)
			break;
		len = scsi_msg_length(p, rem);
		if (!len)
			break;

		switch(p[0]) {
		case SCSI_MSG_REJECT:
			scsi_handle_rejected_msg(xfer);
			break;
		case SCSI_MSG_SIMPLE_TAG:
			xfer->tag = scsi_lookup_tag(p[1]);
			if (!xfer->tag) {
				xfer->outmsgs[0] = SCSI_MSG_ABORT;
				xfer->outmsgpos = 0;
				xfer->outmsgcnt = 1;
				digitalWriteFast(ATNO_PIN, HIGH);
				return;
			}
			break;
		case SCSI_MSG_COMPLETE:
			scsi_free_tag(xfer->tag->tag);
			xfer->tag = 0;
			/* fallthrough */
		case SCSI_MSG_DISCONNECT:
			xfer->disconnect_ok = 1;
			break;
		default:
			break;
		}
		p += len;
	}
}

extern uint16_t rx_packet_size, tx_packet_size;

static void uas_send_read_ready(int tag)
{
	struct uas_response_iu *response_iu;
	transfer_t *t = get_frame(&tx_free_list);
	SCSI_DEBUG(SCSI_DEBUG_UAS, "%x: read ready\n", tag);
	response_iu = transfer_buffer(t);
	memset(response_iu, 0, sizeof(*response_iu));
	response_iu->iu_id = IU_ID_READ_READY;
	response_iu->tag = cpu_to_be16(tag);
	tx_uas_response(t, UAS_STAT_ENDPOINT, sizeof(*response_iu));

}
static void uas_read_ready(struct scsi_xfer *xfer)
{

	if (!usb_uas_interface_alt)
		return;

	if (!xfer->tag || xfer->tag->sent_read_ready)
		return;

	xfer->tag->sent_read_ready = 1;
	uas_send_read_ready(xfer->tag->host_tag);
}

static void uas_write_ready(struct scsi_xfer *xfer)
{
	struct uas_response_iu *response_iu;
	transfer_t *t;

	if (!usb_uas_interface_alt)
		return;

	if (!xfer->tag || xfer->tag->sent_write_ready)
		return;

	xfer->tag->sent_write_ready = 1;

	t = get_frame(&tx_free_list);
	SCSI_DEBUG(SCSI_DEBUG_UAS, "%lx: write ready\n", xfer->tag->host_tag);
	response_iu = transfer_buffer(t);
	memset(response_iu, 0, sizeof(*response_iu));
	response_iu->iu_id = IU_ID_WRITE_READY;
	response_iu->tag = be16_to_cpu(xfer->tag->host_tag);
	tx_uas_response(t, UAS_STAT_ENDPOINT, sizeof(*response_iu));
}

static void scsi_handle_data_out(struct scsi_xfer *xfer)
{
	uint8_t *p = NULL;
	int cnt = 0;
	transfer_t *t = NULL;



	for(;;) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;

		delayNanoseconds(5);

		if (scsi_get_phase() != SCSI_PHASE_DOUT)
			break;

		if (!t) {
			uas_write_ready(xfer);
			if (usb_uas_interface_alt)
				t = get_frame(&rx_dout_busy_list);
			else
				t = get_frame(&rx_cmd_busy_list);
			p = transfer_buffer(t);
			cnt = transfer_length(t);
		}

		scsi_set_data(*p++);
		xfer->data_act++;
		scsi_ack_async();

		cnt--;
		if (!cnt) {
			if (usb_uas_interface_alt)
				usb_rx_dout_ack(t);
			else
				usb_rx_cmd_ack(t);
			t = NULL;
		}

	}
	if (t) {
		if (usb_uas_interface_alt)
			usb_rx_dout_ack(t);
		else
			usb_rx_cmd_ack(t);
	}
	scsi_set_hiz();
}

static void scsi_handle_data_in(struct scsi_xfer *xfer)
{
	uint8_t *p = NULL;
	int cnt = 0;
	transfer_t *t = NULL;

	for(;;) {

		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;

		if (scsi_get_phase() != SCSI_PHASE_DIN)
			break;

		if (!t) {
			uas_read_ready(xfer);
			t = get_frame(&tx_free_list);
			p = transfer_buffer(t);
		}
		*p++ = scsi_get_data();
		cnt++;
		xfer->data_act++;
		if (cnt == 16384/*tx_packet_size*/) {
			SCSI_DEBUG(SCSI_DEBUG_PHASE, "%lx: sending %d bytes\n", get_xfer_tag(xfer), cnt);
			tx_uas_response(t, UAS_DIN_ENDPOINT, cnt);
			cnt = 0;
			t = NULL;
			p = NULL;
		}

		scsi_ack_async();
	}
	if (cnt) {
		SCSI_DEBUG(SCSI_DEBUG_PHASE, "%lx: sending %d final bytes\n", get_xfer_tag(xfer), cnt);
		tx_uas_response(t, UAS_DIN_ENDPOINT, cnt);
	}
}

static void uas_send_status(int status, int tag)
{
	struct uas_sense_iu *sense_iu;
	transfer_t *t = get_frame(&tx_free_list);
	sense_iu = transfer_buffer(t);
	memset(sense_iu, 0, sizeof(*sense_iu));
	sense_iu->iu_id = IU_ID_STATUS;
	sense_iu->tag = cpu_to_be16(tag);
	sense_iu->status = status;
	tx_uas_response(t, UAS_STAT_ENDPOINT, 16);

}

static void usb_status_hook(struct scsi_xfer *xfer, uint8_t status)
{

	struct usb_msc_csw *csw;

	transfer_t *t;

	if (!usb_uas_interface_alt && xfer->data_exp != xfer->data_act && status) {
		t = get_frame(&tx_free_list);
		tx_uas_response(t, UAS_DIN_ENDPOINT, 0);
	}

	if (usb_uas_interface_alt) {
		uas_send_status(status, xfer->tag->host_tag);
	} else {
		t = get_frame(&tx_free_list);
		csw = transfer_buffer(t);
		csw->signature = 0x55534253;
		csw->tag = xfer->tag->host_tag;
		csw->data_residue = xfer->data_exp - xfer->data_act;
		csw->status = status ? 1 : 0;
		SCSI_DEBUG(SCSI_DEBUG_MSC, "data residue %ld, expected %d, actual %d\n",
					  csw->data_residue, xfer->data_exp, xfer->data_act);
		tx_uas_response(t, UAS_DIN_ENDPOINT, sizeof(*csw));
	}
}

static void scsi_handle_status(struct scsi_xfer *xfer)
{
	uint8_t status;

	for(;;) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (!digitalReadFast(REQI_PIN)) {
			if (scsi_get_phase() != SCSI_PHASE_STATUS)
				break;

			status = scsi_get_data();
			usb_status_hook(xfer, status);
			SCSI_DEBUG(SCSI_DEBUG_DUMP, "%lx: STATUS: %02x\n", get_xfer_tag(xfer), status);
			scsi_ack_async();
		}
	}
}

static void scsi_handle_phase(struct scsi_xfer *xfer)
{
	int phase = scsi_get_phase();

	SCSI_DEBUG(SCSI_DEBUG_PHASE, "%lx: handle %s\n", get_xfer_tag(xfer), phase_names[phase & 7]);

	while(digitalReadFast(REQI_PIN)) {
		if (digitalReadFast(BSYI_PIN)) {
			SCSI_DEBUG(SCSI_DEBUG_PHASE, "disconnected\n");
			return;
		}
		delayNanoseconds(5);
	}
	switch (phase) {
	case SCSI_PHASE_DOUT:
		scsi_handle_data_out(xfer);
		break;

	case SCSI_PHASE_DIN:
		scsi_handle_data_in(xfer);
		break;

	case SCSI_PHASE_CMD:
		scsi_handle_cmd(xfer);
		break;

	case SCSI_PHASE_STATUS:
		scsi_handle_status(xfer);
		break;

	case SCSI_PHASE_MOUT:
		scsi_handle_msgout(xfer);
		break;

	case SCSI_PHASE_MIN:
		scsi_handle_msgin(xfer);
		break;

	default:
		SCSI_DEBUG(SCSI_DEBUG_ERROR, "%s: unknown phase %d\n", __func__, phase);
		break;
	}
}

static int scsi_transfer(int id, struct scsi_xfer *xfer)
{
	digitalWriteFast(LED_PIN, HIGH);
	if (scsi_wait_bus_free())
		goto out;

	if (scsi_select(xfer, id))
		goto out;

	while(!digitalReadFast(BSYI_PIN))
		scsi_handle_phase(xfer);

	if (!xfer->disconnect_ok && xfer->tag)
		scsi_free_tag(xfer->tag->tag);
	digitalWriteFast(LED_PIN, LOW);
	return 0;
out:
	digitalWriteFast(LED_PIN, LOW);
	return 1;
}

void scsi_initialize(void)
{
	scsi_setup_ports();
	memset(&sctx, 0, sizeof(sctx));
	sctx.hostid = 7;
	sctx.hostidmsk = (1 << sctx.hostid);
	sctx.support_disconnect = 1;
	sctx.support_tags = 1;
	sctx.support_sdtr = 1;
	sctx.support_identify = 1;
	sctx.targetid = 0xff;
}

static void scsi_setup_msgs(struct scsi_xfer *xfer)
{
	uint8_t *msg = xfer->outmsgs;

	xfer->outmsgcnt = 0;
	xfer->outmsgpos = 0;

	if (!sctx.support_identify)
		return;

	if (sctx.support_disconnect && sctx.support_tags)
			*msg++ = 0xc0 | xfer->lun;
		else
			*msg++ = 0x80 | xfer->lun;
	xfer->outmsgcnt++;

	if (sctx.support_tags) {
		*msg++ = 0x20;
		*msg++ = xfer->tag->tag;
		xfer->outmsgcnt+=2;
	}
}

static void do_xfer(struct scsi_xfer *xfer)
{
	int id;
	do {
		xfer->retry = 0;
		xfer->data_act = 0;

		scsi_setup_msgs(xfer);

		if (sctx.targetid == 0xff) {
			for(id = 7; id >= 0; id--) {
				if (sctx.hostid == id)
					continue;
				printf("Scanning ID %d\n", id);
				if (!scsi_transfer(id, xfer)) {
					printf("found device at ID %d\n", id);
					sctx.targetid = id;
					printf("Support: Identify: %d Disconnect: %d Tags: %d\n",
					       sctx.support_identify,
					       sctx.support_disconnect,
					       sctx.support_tags);
					break;
				}
			}
		} else {
			scsi_transfer(sctx.targetid, xfer);
		}
	} while(xfer->retry);

	if (!xfer->disconnect_ok)
		SCSI_DEBUG(SCSI_DEBUG_ERROR, "%lx: unexpected disconnect\n", xfer->tag->host_tag);
}

static void scsi_uas_request(struct uas_command_iu *iu, int len)
{
	struct scsi_xfer xfer = { 0 };
	char tmp[16] = { 0 };
	int tag;

	if (len < sizeof(struct uas_command_iu)) {
		SCSI_DEBUG(SCSI_DEBUG_UAS, "%s: short request (%d bytes)\n", __func__, len);
		return;
	}

	SCSI_DEBUG(SCSI_DEBUG_UAS, "%d: ID %d, len %d, lun %d, prio_attr %x, cdb %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x LUN %02x %02x %02x %02x %02x %02x %02x %02x\n",
	       (iu->tag >> 8) | ((iu->tag & 0xff) << 8), iu->iu_id, iu->len, iu->lun[0], iu->prio_attr,
	       iu->cdb[0], iu->cdb[1], iu->cdb[2], iu->cdb[3], iu->cdb[4],
	       iu->cdb[5], iu->cdb[6], iu->cdb[7], iu->cdb[8], iu->cdb[9],
	       iu->lun[0], iu->lun[1], iu->lun[2], iu->lun[3],
	       iu->lun[4], iu->lun[5], iu->lun[6], iu->lun[7]);

	if (iu->cdb[0] == 0xa0) {
		/*
		 * At least Windows 10 insists on the REPORT LUNS
		 * command. As most of the devices i have are not
		 * supporting that, fake a response which contains
		 * a single LUN. We could of course improve the code
		 * and figure out whether the device support it.
		 */
		transfer_t *t = get_frame(&tx_free_list);
		tmp[3] = 8;
		memcpy(transfer_buffer(t), tmp, sizeof(tmp));
		tx_uas_response(t, UAS_DIN_ENDPOINT, sizeof(tmp));
		int tag = be16_to_cpu(iu->tag);
		uas_send_read_ready(tag);
		uas_send_status(0, tag);
		return;
	}

	xfer.cdb = iu->cdb;
	xfer.lun = iu->lun[1];

	tag = scsi_insert_tag(be16_to_cpu(iu->tag));
	if (tag == -1) {
		printf("no free tag\n"); // XXX: return error code
		return;
	}
	xfer.tag = scsi_lookup_tag(tag);
	do_xfer(&xfer);
}

static void scsi_msc_request(struct usb_msc_cbw *cbw, int len)
{
	struct scsi_xfer xfer = { 0 };
	int tag;

	if (len < sizeof(struct usb_msc_cbw)) {
		SCSI_DEBUG(SCSI_DEBUG_ERROR, "%s: short request (%d bytes)\n", __func__, len);
		return;
	}

	SCSI_DEBUG(SCSI_DEBUG_MSC, "%ld: len %d, lun %d, cdblen %d, datalen %ld, flags %x, cdb %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		   cbw->tag, len, cbw->lun & 0xf, cbw->cbwcblen, cbw->datalen, cbw->flags,
		   cbw->cdb[0], cbw->cdb[1], cbw->cdb[2], cbw->cdb[3], cbw->cdb[4],
		   cbw->cdb[5], cbw->cdb[6], cbw->cdb[7], cbw->cdb[8], cbw->cdb[9]);
	xfer.cdb = cbw->cdb;

	tag = scsi_insert_tag(cbw->tag);
	if (tag == -1) {
		printf("no free tag\n"); // XXX: return error code
		return;
	}
	xfer.tag = scsi_lookup_tag(tag);
	xfer.lun = cbw->lun & 0xf;
	xfer.data_exp = cbw->datalen;
	sctx.support_tags = 0;
	sctx.support_disconnect = 0;
	do_xfer(&xfer);
}

static void scsi_check_reselection(void)
{
	struct scsi_xfer xfer = { 0 };

	if (!digitalReadFast(SELI_PIN) &&
	    !digitalReadFast(IOI_PIN)) {
		uint8_t ids = scsi_get_data();
		if (ids & sctx.hostidmsk) {
			xfer.id = __builtin_ctz(ids & (sctx.hostidmsk-1));
			SCSI_DEBUG(SCSI_DEBUG_PHASE, "reselection from ID %d\n", xfer.id);
			digitalWriteFast(BSYO_PIN, HIGH);
			while(!digitalReadFast(SELI_PIN));
			digitalWriteFast(BSYO_PIN, LOW);
			delayNanoseconds(SCSI_BUS_SETTLE_DELAY);
			while(!digitalReadFast(BSYI_PIN))
				scsi_handle_phase(&xfer);
			if (!xfer.disconnect_ok && xfer.tag)
				scsi_free_tag(xfer.tag->tag);
			SCSI_DEBUG(SCSI_DEBUG_PHASE, "disconnected\n");
		}
	}
}

void usb_msc_loop(void)
{

	transfer_t *t;
	while (1) {
		/* check for reselection */
		scsi_check_reselection();
		t = get_frame_noblock(&rx_cmd_busy_list);
		if (t == LIST_END)
			continue;
		int len = transfer_length(t);

		if (len > 0) {
			if (!usb_uas_interface_alt) {
				scsi_msc_request(transfer_buffer(t), len);
			} else {
				scsi_uas_request(transfer_buffer(t), len);
			}
			usb_rx_cmd_ack(t);
		} else {
			SCSI_DEBUG(SCSI_DEBUG_ERROR, "ZLP!\n");
		}

	}
}

