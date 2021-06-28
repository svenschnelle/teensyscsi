#ifndef UAS_SCSI_H
#define UAS_SCSI_H

#include <stdint.h>
#define SCSI_SENSE_BUFFERSIZE 96

struct uas_command_iu {
	uint8_t iu_id;
	uint8_t rsvd1;
	uint16_t tag;
	uint8_t prio_attr;
	uint8_t rsvd5;
	uint8_t len;
	uint8_t rsvd7;
	uint8_t lun[8];
	uint8_t cdb[16];
} __attribute__((__packed__));

struct uas_response_iu {
	uint8_t iu_id;
	uint8_t rsvd1;
	uint16_t tag;
	uint8_t add_response_info[3];
	uint8_t response_code;
} __attribute__((__packed__));

struct uas_sense_iu {
	uint8_t iu_id;
	uint8_t rsvd1;
	uint16_t tag;
	uint16_t status_qual;
	uint8_t status;
	uint8_t rsvd7[7];
	uint16_t len;
	uint8_t sense[SCSI_SENSE_BUFFERSIZE];
} __attribute__((__packed__));

typedef enum {
	IU_ID_COMMAND = 1,
	IU_ID_STATUS = 3,
	IU_ID_RESPONSE = 4,
	IU_ID_TASK_MGMT = 5,
	IU_ID_READ_READY = 6,
	IU_ID_WRITE_READY = 7,
} uas_iu_t;

typedef enum {
	SCSI_MSG_UNKNOWN,
	SCSI_MSG_IDENTIFY,
	SCSI_MSG_TAG,
} scsi_msg_phase_t;

struct scsi_xfer {
	uint8_t id;
	uint8_t *cdb;
	uint8_t status;
	uint16_t tag;
	uint8_t outmsgs[16];
	uint8_t inmsgs[16];
	scsi_msg_phase_t msgphase;
	int inmsgcnt;
	int lun;
	int abortxfr:1;
	int retry:1;
};

#define SCSI_MSG_REJECT 0x07
#define SCSI_MSG_SIMPLE_TAG 0x20
static inline uint16_t get_xfer_tag(struct scsi_xfer *xfer)
{
	return (xfer->tag >> 8) | ((xfer->tag & 0xff) << 8);
}

#ifdef __cplusplus
extern "C" {
#endif

void scsi_initialize(void);
void scsi_reset(void);
#ifdef __cplusplus
}
#endif

#endif
