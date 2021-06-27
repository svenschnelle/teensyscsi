#include <scsi.h>
#include <core_pins.h>
#include <stdio.h>
#include "usb_dev.h"
#include "usb_desc.h"
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

#define SCSI_BUS_CLEAR_DELAY 800
#define SCSI_ARBITRATION_DELAY 2400
#define SCSI_BUS_SETTLE_DELAY 400

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
}

static uint8_t scsi_get_data(void)
{
	delayNanoseconds(5);
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

static __attribute__((noinline)) void scsi_set_data(uint8_t data)
{
	GPIO6_DR_CLEAR |= ~data << 24;
	GPIO6_DR_SET |= data << 24;
	digitalWriteFast(DBPO_PIN, parity_table[data]);
}

static void dump_scsi(const char *prefix)
{
	// XX SEL BSY RST ACK REQ CD IO MSG ATN
	printf("%s: %02X %s%s%s%s%s%s%s%s%s%s\n",
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
//	dump_scsi("RESET");
	scsi_set_data(0);
	digitalWriteFast(RSTO_PIN, HIGH);
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
}

static int scsi_wait_bus_free(void)
{
	do {
		digitalWriteFast(BSYO_PIN, LOW);
		delayNanoseconds(800); /* Bus clear delay */
		while(!(digitalReadFast(SELI_PIN) & digitalReadFast(BSYI_PIN)))
			printf("SELI: %d, BSYI: %d\n",
				digitalReadFast(SELI_PIN),
				digitalReadFast(BSYI_PIN));
		/* start arbitration */
		digitalWriteFast(BSYO_PIN, HIGH);
		scsi_set_data(0x80);
		delayNanoseconds(2400); /* Arbitration delay */
	} while (scsi_get_data() != 0x80); // XXX: check priority
	delayNanoseconds(SCSI_BUS_CLEAR_DELAY);
	return 0;
}

static int scsi_select(int id)
{
	int i = 250000000 / SCSI_BUS_SETTLE_DELAY;

	digitalWriteFast(SELO_PIN, HIGH);
	delayNanoseconds(10 * SCSI_BUS_SETTLE_DELAY);
        scsi_set_data(0x80 | (1 << id));
	delayNanoseconds(10 * SCSI_BUS_SETTLE_DELAY);
	digitalWriteFast(BSYO_PIN, LOW);
	delayNanoseconds(10 * SCSI_BUS_SETTLE_DELAY);

        while (i-- > 0 && digitalReadFast(BSYI_PIN))
		delayNanoseconds(SCSI_BUS_SETTLE_DELAY);

	if (i < 0) {
		printf("select failed\n");
		digitalWriteFast(SELO_PIN, LOW);
		return 1;
	}

        digitalWriteFast(SELO_PIN, LOW);
	delayNanoseconds(SCSI_BUS_SETTLE_DELAY);
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
	delayNanoseconds(100);
	digitalWriteFast(ACKO_PIN, HIGH);
	delayNanoseconds(100);
	while(!digitalReadFast(REQI_PIN));
	delayNanoseconds(100);
	digitalWriteFast(ACKO_PIN, LOW);
	delayNanoseconds(100);
}

static void scsi_handle_cmd(struct scsi_xfer *xfer)
{
	unsigned int i;
	uint8_t *cdb = xfer->cdb;

	for(i = 0; i < get_cdb_len(xfer);) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;
		if (scsi_get_phase() != SCSI_PHASE_CMD)
			break;

		scsi_set_data(cdb[i++]);
		scsi_ack_async();
	}
	scsi_set_data(0);
//	printf("send %d CMD bytes\n", i);
}

static void scsi_handle_msgout(struct scsi_xfer *xfer)
{
	uint8_t msg = 0x80;
	for(;;) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;

		if (scsi_get_phase() != SCSI_PHASE_MOUT)
			break;

		scsi_set_data(msg);
		scsi_ack_async();
		break;
	}
	scsi_set_data(0);
}

static void scsi_handle_msgin(struct scsi_xfer *xfer)
{
	for(;;) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;

		if (scsi_get_phase() != SCSI_PHASE_MIN)
			break;

//			printf("MSGIN: %02x\n", scsi_get_data());
		scsi_ack_async();
		break;
	}
}

extern uint16_t rx_packet_size, tx_packet_size;

static void uas_read_ready(struct scsi_xfer *xfer)
{
	struct uas_response_iu *response_iu;
	transfer_t *t = get_frame(&tx_free_list);

//	printf("%s: %d\n", __func__, get_xfer_tag(xfer));
	response_iu = transfer_buffer(t);
	memset(response_iu, 0, sizeof(*response_iu));
	response_iu->iu_id = IU_ID_READ_READY;
	response_iu->tag = xfer->tag;
	tx_uas_response(t, UAS_STAT_ENDPOINT, sizeof(*response_iu));
}

static void uas_write_ready(struct scsi_xfer *xfer)
{
	struct uas_response_iu *response_iu;
	transfer_t *t = get_frame(&tx_free_list);

//	printf("%s: %d\n", __func__, get_xfer_tag(xfer));
	response_iu = transfer_buffer(t);
	memset(response_iu, 0, sizeof(*response_iu));
	response_iu->iu_id = IU_ID_WRITE_READY;
	response_iu->tag = xfer->tag;
	tx_uas_response(t, UAS_STAT_ENDPOINT, sizeof(*response_iu));
}

static void scsi_handle_data_out(struct scsi_xfer *xfer)
{
	uint8_t *p = NULL;
	int cnt = 0;
	transfer_t *t = NULL;
	int write_ready = 0;

	for(;;) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;

		if (scsi_get_phase() != SCSI_PHASE_DOUT)
			break;


		if (!t) {
			if (!write_ready) {
				write_ready = 1;
				uas_write_ready(xfer);
			}
			t = get_frame(&rx_dout_busy_list);
			p = transfer_buffer(t);
			cnt = transfer_length(t);
		}

		scsi_set_data(*p++);
		scsi_ack_async();

		cnt--;
		if (!cnt) {
			usb_rx_dout_ack(t);
			t = NULL;
		}

	}
	if (t)
		usb_rx_dout_ack(t);
	scsi_set_data(0);
}

static void scsi_handle_data_in(struct scsi_xfer *xfer)
{
	uint8_t *p = NULL;
	int cnt = 0;
	transfer_t *t = NULL;
	int read_ready = 0;

	for(;;) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (digitalReadFast(REQI_PIN))
			continue;

		if (scsi_get_phase() != SCSI_PHASE_DIN)
			break;


		if (!t) {
			t = get_frame(&tx_free_list);
			p = transfer_buffer(t);
		}

		*p++ = scsi_get_data();
		cnt++;
		if (cnt == 16384/*tx_packet_size*/) {

//				printf("sending %d bytes, tag %d\n", cnt, get_xfer_tag(xfer));
			tx_uas_response(t, UAS_DIN_ENDPOINT, cnt);
			if (!read_ready) {
				read_ready = 1;
				uas_read_ready(xfer);
			}

			cnt = 0;
			t = NULL;
			p = NULL;
		}

		scsi_ack_async();
	}

	if (cnt) {
		if (!read_ready) {
			read_ready = 1;
			uas_read_ready(xfer);
		}

		if (!t)
			t = get_frame(&tx_free_list);
//		printf("sending %d bytes (zlp %d), tag %d\n", cnt, zlp, get_xfer_tag(xfer));
		tx_uas_response(t, UAS_DIN_ENDPOINT, cnt);
	}
}

static void scsi_handle_status(struct scsi_xfer *xfer)
{
	struct uas_sense_iu *sense_iu;
	transfer_t *t;
	uint8_t status;

	for(;;) {
		if (digitalReadFast(BSYI_PIN))
			break;

		if (!digitalReadFast(REQI_PIN)) {
			if (scsi_get_phase() != SCSI_PHASE_STATUS)
				break;

			status = scsi_get_data();
			t = get_frame(&tx_free_list);
			sense_iu = transfer_buffer(t);
			memset(sense_iu, 0, sizeof(*sense_iu));
			sense_iu->iu_id = IU_ID_STATUS;
			sense_iu->tag = xfer->tag;
			sense_iu->status = status;
			tx_uas_response(t, UAS_STAT_ENDPOINT, 16);

//			printf("received status %02x, tag %d\n", status, get_xfer_tag(xfer));
			scsi_ack_async();
		}
	}
}

static void scsi_handle_phase(struct scsi_xfer *xfer)
{
	int phase = scsi_get_phase();

//	printf("%s: %s\n", __func__, phase_names[phase & 7]);

	while(digitalReadFast(REQI_PIN)) {
		if (digitalReadFast(BSYI_PIN))
			return;
		delayNanoseconds(SCSI_BUS_SETTLE_DELAY);
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
		printf("%s: unknown phase %d\n", __func__, phase);
		break;
	}
}

int scsi_transfer(struct scsi_xfer *xfer)
{
	if (scsi_wait_bus_free())
		return 1;

	if (scsi_select(xfer->id))
		return 1;

	while(!digitalReadFast(BSYI_PIN))
		scsi_handle_phase(xfer);
	return 0;
}

void scsi_initialize(void)
{
	scsi_setup_ports();
}

static void scsi_uas_request(struct uas_command_iu *iu, int len)
{
	struct scsi_xfer xfer = { 0 };

	if (len < sizeof(struct uas_command_iu)) {
		printf("%s: short request (%d bytes)\n", __func__, len);
		return;
	}
	printf("ID %d, len %d, lun %d, tag %d, prio_attr %x, cdb %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	       iu->iu_id, iu->len, iu->lun[0], (iu->tag >> 8) | ((iu->tag & 0xff) << 8), iu->prio_attr,
	       iu->cdb[0], iu->cdb[1], iu->cdb[2], iu->cdb[3], iu->cdb[4],
	       iu->cdb[5], iu->cdb[6], iu->cdb[7], iu->cdb[8], iu->cdb[9]);

	xfer.cdb = iu->cdb;
	xfer.id = 5; // XXX
	xfer.tag = iu->tag;

	scsi_transfer(&xfer);
//	printf("request done\n");
}

void usb_msc_loop(void)
{
	transfer_t *t;
	while (1) {
		t = get_frame(&rx_cmd_busy_list);
		int len = transfer_length(t);

		if (len > 0) {
			scsi_uas_request(transfer_buffer(t), len);
			usb_rx_cmd_ack(t);
		} else {
			printf("ZLP!\n");
		}
	}
}


