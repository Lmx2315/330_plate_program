
#include "main.h"

/*
 * These macros allow us to specify TMS state transitions by bits rather than hex bytes.
 * Read the bits from LSBit first to MSBit last (right-to-left).
 */
//#define HEX__(n) 0x##n##LU
//#define B8__(x)	\
	((((x) & 0x0000000FLU) ? (1 << 0) : 0) \
	+(((x) & 0x000000F0LU) ? (1 << 1) : 0) \
	+(((x) & 0x00000F00LU) ? (1 << 2) : 0) \
	+(((x) & 0x0000F000LU) ? (1 << 3) : 0) \
	+(((x) & 0x000F0000LU) ? (1 << 4) : 0) \
	+(((x) & 0x00F00000LU) ? (1 << 5) : 0) \
	+(((x) & 0x0F000000LU) ? (1 << 6) : 0) \
	+(((x) & 0xF0000000LU) ? (1 << 7) : 0))

//#define B8(bits, count) {((uint8_t)B8__(HEX__(bits))), (count)}

//static const struct tms_sequences old_tms_seqs[6][6] = {	/* [from_state_ndx][to_state_ndx] */
	/* value clocked to TMS to move from one of six stable states to another.
	 * N.B. OOCD clocks TMS from LSB first, so read these right-to-left.
	 * N.B. Reset only needs to be 0b11111, but in JLink an even byte of 1's is more stable.
	 * These extra ones cause no TAP state problem, because we go into reset and stay in reset.
	 */

/* to state: */
/*	RESET		 IDLE			DRSHIFT			DRPAUSE			IRSHIFT			IRPAUSE		*/	/* from state: */
//{B8(1111111, 7), B8(0000000, 7), B8(0010111, 7), B8(0001010, 7), B8(0011011, 7), B8(0010110, 7)},/* RESET */
//{B8(1111111, 7), B8(0000000, 7), B8(0100101, 7), B8(0000101, 7), B8(0101011, 7), B8(0001011, 7)},/* IDLE */
//{B8(1111111, 7), B8(0110001, 7), B8(0000000, 7), B8(0000001, 7), B8(0001111, 7), B8(0101111, 7)},/* DRSHIFT */
//{B8(1111111, 7), B8(0110000, 7), B8(0100000, 7), B8(0010111, 7), B8(0011110, 7), B8(0101111, 7)},/* DRPAUSE */
//{B8(1111111, 7), B8(0110001, 7), B8(0000111, 7), B8(0010111, 7), B8(0000000, 7), B8(0000001, 7)},/* IRSHIFT */
//{B8(1111111, 7), B8(0110000, 7), B8(0011100, 7), B8(0010111, 7), B8(0011110, 7), B8(0101111, 7)}, /* IRPAUSE */
//};

//static const struct tms_sequences short_tms_seqs[6][6] = { /* [from_state_ndx][to_state_ndx] */
	/* this is the table submitted by Jeff Williams on 3/30/2009 with this comment:

	OK, I added Peter's version of the state table, and it works OK for
	me on MC1322x. I've recreated the jlink portion of patch with this
	new state table. His changes to my state table are pretty minor in
	terms of total transitions, but Peter feels that his version fixes
	some long-standing problems.
	Jeff

	I added the bit count into the table, reduced RESET column to 7 bits from 8.
	Dick

	state specific comments:
	------------------------
	*->RESET		tried the 5 bit reset and it gave me problems, 7 bits seems to
					work better on ARM9 with ft2232 driver.  (Dick)

	RESET->DRSHIFT add 1 extra clock cycles in the RESET state before advancing.
					needed on ARM9 with ft2232 driver.  (Dick)
					(For a total of *THREE* extra clocks in RESET; NOP.)

	RESET->IRSHIFT add 1 extra clock cycles in the RESET state before advancing.
					needed on ARM9 with ft2232 driver.  (Dick)
					(For a total of *TWO* extra clocks in RESET; NOP.)

	RESET->*		always adds one or more clocks in the target state,
					which should be NOPS; except shift states which (as
					noted above) add those clocks in RESET.

	The X-to-X transitions always add clocks; from *SHIFT, they go
	via IDLE and thus *DO HAVE SIDE EFFECTS* (capture and update).
*/

/* to state: */
/*	RESET		IDLE			DRSHIFT			DRPAUSE			IRSHIFT			IRPAUSE */ /* from state: */
//{B8(1111111, 7), B8(0000000, 7), B8(0010111, 7), B8(0001010, 7), B8(0011011, 7), B8(0010110, 7)}, /* RESET */
//{B8(1111111, 7), B8(0000000, 7), B8(001, 3),	 B8(0101, 4),	 B8(0011, 4),	 B8(01011, 5)}, /* IDLE */
//{B8(1111111, 7), B8(011, 3),	 B8(00111, 5),	 B8(01, 2),		 B8(001111, 6),	 B8(0101111, 7)}, /* DRSHIFT */
//{B8(1111111, 7), B8(011, 3),	 B8(01, 2),		 B8(0, 1),		 B8(001111, 6),	 B8(0101111, 7)}, /* DRPAUSE */
//{B8(1111111, 7), B8(011, 3),	 B8(00111, 5),	 B8(010111, 6),	 B8(001111, 6),	 B8(01, 2)}, /* IRSHIFT */
//{B8(1111111, 7), B8(011, 3),	 B8(00111, 5),	 B8(010111, 6),	 B8(01, 2),		 B8(0, 1)} /* IRPAUSE */
//};

/*
 * The JTAG subsystem defines a number of error codes,
 * using codes between -100 and -199.
 */
#define ERROR_JTAG_INIT_FAILED        (-100)
#define ERROR_JTAG_INVALID_INTERFACE  (-101)
#define ERROR_JTAG_NOT_IMPLEMENTED    (-102)
#define ERROR_JTAG_TRST_ASSERTED      (-103)
#define ERROR_JTAG_QUEUE_FAILED       (-104)
#define ERROR_JTAG_NOT_STABLE_STATE   (-105)
#define ERROR_JTAG_DEVICE_ERROR       (-107)
#define ERROR_JTAG_STATE_INVALID      (-108)
#define ERROR_JTAG_TRANSITION_INVALID (-109)
#define ERROR_JTAG_INIT_SOFT_FAIL     (-110)

/* general failures
 * error codes < 100
 */
#define ERROR_OK						(0)
#define ERROR_NO_CONFIG_FILE			(-2)
#define ERROR_BUF_TOO_SMALL				(-3)
/* see "Error:" log entry for meaningful message to the user. The caller should
 * make no assumptions about what went wrong and try to handle the problem.
 */
#define ERROR_FAIL						(-4)
#define ERROR_WAIT						(-5)

/*
 * OpenJTAG-OpenOCD state conversion
 */
typedef enum openjtag_tap_state {
	OPENJTAG_TAP_INVALID    = -1,
	OPENJTAG_TAP_RESET      = 0,
	OPENJTAG_TAP_IDLE       = 1,
	OPENJTAG_TAP_SELECT_DR  = 2,
	OPENJTAG_TAP_CAPTURE_DR = 3,
	OPENJTAG_TAP_SHIFT_DR   = 4,
	OPENJTAG_TAP_EXIT1_DR   = 5,
	OPENJTAG_TAP_PAUSE_DR   = 6,
	OPENJTAG_TAP_EXIT2_DR   = 7,
	OPENJTAG_TAP_UPDATE_DR  = 8,
	OPENJTAG_TAP_SELECT_IR  = 9,
	OPENJTAG_TAP_CAPURE_IR  = 10,
	OPENJTAG_TAP_SHIFT_IR   = 11,
	OPENJTAG_TAP_EXIT1_IR   = 12,
	OPENJTAG_TAP_PAUSE_IR   = 13,
	OPENJTAG_TAP_EXIT2_IR   = 14,
	OPENJTAG_TAP_UPDATE_IR  = 15,
} openjtag_tap_state_t;

/* OpenJTAG vid/pid */
static uint16_t openjtag_vid = 0x0403;
static uint16_t openjtag_pid = 0x6001;

static char *openjtag_device_desc;

#define OPENJTAG_BUFFER_SIZE        504
#define OPENJTAG_MAX_PENDING_RESULTS    256

struct openjtag_scan_result {
	uint32_t bits;          /* Length in bits*/
	struct scan_command *command;   /* Corresponding scan command */
	uint8_t *buffer;
};

/* USB RX/TX buffers */
static int      usb_tx_buf_offs;
static uint8_t  usb_tx_buf[OPENJTAG_BUFFER_SIZE];
static uint32_t usb_rx_buf_len;
static uint8_t  usb_rx_buf[OPENJTAG_BUFFER_SIZE];

/* Pending readings */
static struct openjtag_scan_result openjtag_scan_result_buffer[OPENJTAG_MAX_PENDING_RESULTS];
static int openjtag_scan_result_count;

static unsigned int ep_in, ep_out;

typedef enum tap_state {
	TAP_INVALID = -1,

	/* Proper ARM recommended numbers */
	TAP_DREXIT2 = 0x0,
	TAP_DREXIT1 = 0x1,
	TAP_DRSHIFT = 0x2,
	TAP_DRPAUSE = 0x3,
	TAP_IRSELECT = 0x4,
	TAP_DRUPDATE = 0x5,
	TAP_DRCAPTURE = 0x6,
	TAP_DRSELECT = 0x7,
	TAP_IREXIT2 = 0x8,
	TAP_IREXIT1 = 0x9,
	TAP_IRSHIFT = 0xa,
	TAP_IRPAUSE = 0xb,
	TAP_IDLE = 0xc,
	TAP_IRUPDATE = 0xd,
	TAP_IRCAPTURE = 0xe,
	TAP_RESET = 0x0f,

} tap_state_t;


enum reset_types {
	RESET_NONE            = 0x0,
	RESET_HAS_TRST        = 0x1,
	RESET_HAS_SRST        = 0x2,
	RESET_TRST_AND_SRST   = 0x3,
	RESET_SRST_PULLS_TRST = 0x4,
	RESET_TRST_PULLS_SRST = 0x8,
	RESET_TRST_OPEN_DRAIN = 0x10,
	RESET_SRST_PUSH_PULL  = 0x20,
	RESET_SRST_NO_GATING  = 0x40,
	RESET_CNCT_UNDER_SRST = 0x80
};

struct scan_field {
	/** The number of bits this field specifies */
	int num_bits;
	/** A pointer to value to be scanned into the device */
	const uint8_t *out_value;
	/** A pointer to a 32-bit memory location for data scanned out */
	uint8_t *in_value;

	/** The value used to check the data scanned out. */
	uint8_t *check_value;
	/** The mask to go with check_value */
	uint8_t *check_mask;
};

/**
 * The inferred type of a scan_command_s structure, indicating whether
 * the command has the host scan in from the device, the host scan out
 * to the device, or both.
 */
enum scan_type {
	/** From device to host, */
	SCAN_IN = 1,
	/** From host to device, */
	SCAN_OUT = 2,
	/** Full-duplex scan. */
	SCAN_IO = 3
};

/**
 * The scan_command provide a means of encapsulating a set of scan_field_s
 * structures that should be scanned in/out to the device.
 */
struct scan_command {
	/** instruction/not data scan */
	bool ir_scan;
	/** number of fields in *fields array */
	int num_fields;
	/** pointer to an array of data scan fields */
	struct scan_field *fields;
	/** state in which JTAG commands should finish */
	tap_state_t end_state;
};

struct statemove_command {
	/** state in which JTAG commands should finish */
	tap_state_t end_state;
};

struct pathmove_command {
	/** number of states in *path */
	int num_states;
	/** states that have to be passed */
	tap_state_t *path;
};

struct runtest_command {
	/** number of cycles to spend in Run-Test/Idle state */
	int num_cycles;
	/** state in which JTAG commands should finish */
	tap_state_t end_state;
};


struct stableclocks_command {
	/** number of clock cycles that should be sent */
	int num_cycles;
};


struct reset_command {
	/** Set TRST output: 0 = deassert, 1 = assert, -1 = no change */
	int trst;
	/** Set SRST output: 0 = deassert, 1 = assert, -1 = no change */
	int srst;
};

struct end_state_command {
	/** state in which JTAG commands should finish */
	tap_state_t end_state;
};

struct sleep_command {
	/** number of microseconds to sleep */
	uint32_t us;
};

struct tms_command {
	/** How many bits should be clocked out. */
	unsigned num_bits;
	/** The bits to clock out; the LSB is bit 0 of bits[0]. */
	const uint8_t *bits;
};

union jtag_command_container {
	struct scan_command *scan;
	struct statemove_command *statemove;
	struct pathmove_command *pathmove;
	struct runtest_command *runtest;
	struct stableclocks_command *stableclocks;
	struct reset_command *reset;
	struct end_state_command *end_state;
	struct sleep_command *sleep;
	struct tms_command *tms;
};

enum jtag_command_type {
	JTAG_SCAN         = 1,
	/* JTAG_TLR_RESET's non-minidriver implementation is a
	 * vestige from a statemove cmd. The statemove command
	 * is obsolete and replaced by pathmove.
	 *
	 * pathmove does not support reset as one of it's states,
	 * hence the need for an explicit statemove command.
	 */
	JTAG_TLR_RESET    = 2,
	JTAG_RUNTEST      = 3,
	JTAG_RESET        = 4,
	JTAG_PATHMOVE     = 6,
	JTAG_SLEEP        = 7,
	JTAG_STABLECLOCKS = 8,
	JTAG_TMS          = 9,
};

struct jtag_command {
	union jtag_command_container cmd;
	enum jtag_command_type type;
	struct jtag_command *next;
};

static int8_t openjtag_get_tap_state(int8_t state)
{

	switch (state) {
		case TAP_DREXIT2:   return OPENJTAG_TAP_EXIT2_DR;
		case TAP_DREXIT1:   return OPENJTAG_TAP_EXIT1_DR;
		case TAP_DRSHIFT:   return OPENJTAG_TAP_SHIFT_DR;
		case TAP_DRPAUSE:   return OPENJTAG_TAP_PAUSE_DR;
		case TAP_IRSELECT:  return OPENJTAG_TAP_SELECT_IR;
		case TAP_DRUPDATE:  return OPENJTAG_TAP_UPDATE_DR;
		case TAP_DRCAPTURE: return OPENJTAG_TAP_CAPTURE_DR;
		case TAP_DRSELECT:  return OPENJTAG_TAP_SELECT_DR;
		case TAP_IREXIT2:   return OPENJTAG_TAP_EXIT2_IR;
		case TAP_IREXIT1:   return OPENJTAG_TAP_EXIT1_IR;
		case TAP_IRSHIFT:   return OPENJTAG_TAP_SHIFT_IR;
		case TAP_IRPAUSE:   return OPENJTAG_TAP_PAUSE_IR;
		case TAP_IDLE:      return OPENJTAG_TAP_IDLE;
		case TAP_IRUPDATE:  return OPENJTAG_TAP_UPDATE_IR;
		case TAP_IRCAPTURE: return OPENJTAG_TAP_CAPURE_IR;
		case TAP_RESET:     return OPENJTAG_TAP_RESET;
		case TAP_INVALID:
		default:            return OPENJTAG_TAP_INVALID;
	}
}

void tap_set_state_impl(tap_state_t new_state)
{
	/* this is the state we think the TAPs are in now, was cur_state */
	state_follower = new_state;
}

static inline void tap_set_state(tap_state_t new_state)
{
	tap_set_state_impl(new_state);
}

tap_state_t tap_get_state()
{
	return state_follower;
}


tap_state_t tap_get_end_state()
{
	return end_state_follower;
}

void tap_set_end_state(tap_state_t new_end_state)
{
	/* this is the state we think the TAPs will be in at completion of the
	 * current TAP operation, was end_state
	*/
	end_state_follower = new_end_state;
}

static int openjtag_buf_write_standard(
	uint8_t *buf, int size, uint32_t *bytes_written)
{
	int retval;

	retval = ftdi_write_data(&buf, size);
	if (retval < 0) {
		*bytes_written = 0;
		x_out("ftdi_write_data:", ftdi_get_error_string());
		return ERROR_JTAG_DEVICE_ERROR;
	}

	*bytes_written += retval;

	return ERROR_OK;
}

int ftdi_write_data(char *buf,int size)
{
	while (size!=0)
	{

		size--;
	}
	return  1;
}


static int openjtag_buf_write(
	uint8_t *buf, int size, uint32_t *bytes_written)
{
	return openjtag_buf_write_standard(buf, size, bytes_written);
}

static int openjtag_buf_read_standard(
	uint8_t *buf, uint32_t qty, uint32_t *bytes_read)
{

	int retval;
	int timeout = 5;

	*bytes_read = 0;

	while ((*bytes_read < qty) && timeout--) {
		retval = ftdi_read_data(buf + *bytes_read,qty - *bytes_read);
		if (retval < 0) {
			*bytes_read = 0;
			x_out("ftdi_read_data:",ftdi_get_error_string());
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += retval;
	}

	return ERROR_OK;
}


static int openjtag_buf_read(uint8_t *buf, uint32_t qty, uint32_t *bytes_read)
{

 return openjtag_buf_read_standard(buf, qty, bytes_read);

}

static int openjtag_sendcommand(uint8_t cmd)
{
	uint32_t written;
	return openjtag_buf_write(&cmd, 1, &written);
}

static int openjtag_speed(int speed)
{
	int clockcmd;
	switch (speed) {
		case 48000:
			clockcmd = 0x00;
			break;
		case 24000:
			clockcmd = 0x20;
			break;
		case 12000:
			clockcmd = 0x40;
			break;
		case 6000:
			clockcmd = 0x60;
			break;
		case 3000:
			clockcmd = 0x80;
			break;
		case 1500:
			clockcmd = 0xA0;
			break;
		case 750:
			clockcmd = 0xC0;
			break;
		case 375:
			clockcmd = 0xE0;
			break;
		default:
			clockcmd = 0xE0;
			Transf("adapter speed not recognized, reverting to 375 kHz");
			break;
	}
	openjtag_sendcommand(clockcmd);

	return ERROR_OK;
}

static int openjtag_init_standard(void)
{
	uint8_t latency_timer;

	/* Open by device description */
	if (openjtag_device_desc == NULL) {
		Transf("no openjtag device description specified using default Open JTAG Project");
		openjtag_device_desc = "Open JTAG Project";
	}

	return ERROR_OK;
}

static int openjtag_init(void)
{
	int ret;

	usb_tx_buf_offs = 0;
	usb_rx_buf_len = 0;
	openjtag_scan_result_count = 0;
	
	ret = openjtag_init_standard();
	
	if (ret != ERROR_OK)
		return ret;

	openjtag_speed(375); /* Start at slowest adapter speed */
	openjtag_sendcommand(0x75); /* MSB */

	return ERROR_OK;
}


static void openjtag_write_tap_buffer(void)
{
	uint32_t written;

	openjtag_buf_write(usb_tx_buf, usb_tx_buf_offs, &written);
	openjtag_buf_read(usb_rx_buf, usb_tx_buf_offs, &usb_rx_buf_len);

	usb_tx_buf_offs = 0;
}

static int openjtag_execute_tap_queue(void)
{
	openjtag_write_tap_buffer();

	int res_count = 0;

	if (openjtag_scan_result_count && usb_rx_buf_len) {

		int count;
		int rx_offs = 0;
		int len;

		/* for every pending result */
		while (res_count < openjtag_scan_result_count) {

			/* get sent bits */
			len = openjtag_scan_result_buffer[res_count].bits;

			count = 0;

			uint8_t *buffer = openjtag_scan_result_buffer[res_count].buffer;

			while (len > 0) {
				if (len <= 8) {
					x_out("bits < 8 buf = 0x",usb_rx_buf[rx_offs]);
					x_out("bits < 8 buf = 0x",usb_rx_buf[rx_offs] >> (8 - len));
					buffer[count] = usb_rx_buf[rx_offs] >> (8 - len);
					len = 0;
				} else {
					buffer[count] = usb_rx_buf[rx_offs];
					len -= 8;
				}

				rx_offs++;
				count++;
			}

			jtag_read_buffer(buffer, openjtag_scan_result_buffer[res_count].command);

			if (openjtag_scan_result_buffer[res_count].buffer)
				free(openjtag_scan_result_buffer[res_count].buffer);

			res_count++;
		}
	}

	openjtag_scan_result_count = 0;

	return ERROR_OK;
}


static void openjtag_add_byte(char buf)
{

	if (usb_tx_buf_offs == OPENJTAG_BUFFER_SIZE) {
		Transf("Forcing execute_tap_queue");
		x_out ("TX Buff offs=", usb_tx_buf_offs);
		openjtag_execute_tap_queue();
	}

	usb_tx_buf[usb_tx_buf_offs] = buf;
	usb_tx_buf_offs++;
}

static void openjtag_add_scan(uint8_t *buffer, int length, struct scan_command *scan_cmd)
{

	/* Ensure space to send long chains */
	/* We add two byte for each eight (or less) bits, one for command, one for data */
	if (usb_tx_buf_offs + (DIV_ROUND_UP(length, 8) * 2) >= OPENJTAG_BUFFER_SIZE) {
		Transf("Forcing execute_tap_queue from scan");
		x_out("TX Buff offs=", usb_tx_buf_offs);
		x_out("TX Buff  len=", DIV_ROUND_UP(length, 8) * 2);
		openjtag_execute_tap_queue();
	}

	openjtag_scan_result_buffer[openjtag_scan_result_count].bits = length;
	openjtag_scan_result_buffer[openjtag_scan_result_count].command = scan_cmd;
	openjtag_scan_result_buffer[openjtag_scan_result_count].buffer = buffer;

	uint8_t command;
	uint8_t bits;
	int count = 0;
	while (length) {

		/* write command */
		command = 6;

		/* last bits? */
		if (length <= 8) {
			/* tms high */
			command |= (1 << 4);

			/* bits to transfer */
			bits = (length - 1);
			command |= bits << 5;
			length = 0;
		} else {
			/* whole byte */

			/* bits to transfer */
			bits = 7;
			command |= (7 << 5);
			length -= 8;
		}

		openjtag_add_byte(command);
		openjtag_add_byte(buffer[count]);
		count++;
	}

	openjtag_scan_result_count++;
}

static void openjtag_execute_reset(struct jtag_command *cmd)
{

	x_out("reset trst:  ",	cmd->cmd.reset->trst);
	x_out("reset srst:  ",	cmd->cmd.reset->srst);

	uint8_t buf = 0x00;

	if (cmd->cmd.reset->trst) {
		buf = 0x03;
	} else {
		buf |= 0x04;
		buf |= 0x05 << 4;
	}

	openjtag_add_byte(buf);
}


static void openjtag_execute_sleep(struct jtag_command *cmd)
{
	jtag_sleep(cmd->cmd.sleep->us);
}

static void openjtag_set_state(uint8_t openocd_state)
{
	int8_t state = openjtag_get_tap_state(openocd_state);

	uint8_t buf = 0;
	buf = 0x01;
	buf |= state << 4;

	openjtag_add_byte(buf);
}

static void openjtag_execute_statemove(struct jtag_command *cmd)
{
	x_out("state move to ", cmd->cmd.statemove->end_state);

	tap_set_end_state(cmd->cmd.statemove->end_state);

	openjtag_set_state(cmd->cmd.statemove->end_state);

	tap_set_state(tap_get_end_state());
}



static void openjtag_execute_scan(struct jtag_command *cmd)
{

	int scan_size, old_state;
	uint8_t *buffer;

	x_out("scan ends in ", tap_state_name(cmd->cmd.scan->end_state));

	/* get scan info */
	tap_set_end_state(cmd->cmd.scan->end_state);
	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);

	/* set state */
	old_state = tap_get_end_state();
	openjtag_set_state(cmd->cmd.scan->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);
	tap_set_state(cmd->cmd.scan->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);
	tap_set_end_state(old_state);

	openjtag_add_scan(buffer, scan_size, cmd->cmd.scan);

	openjtag_set_state(cmd->cmd.scan->ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);
	tap_set_state(cmd->cmd.scan->ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state()) {
		openjtag_set_state(tap_get_end_state());
		tap_set_state(tap_get_end_state());
	}
}

static void openjtag_execute_runtest(struct jtag_command *cmd)
{

	tap_state_t end_state = cmd->cmd.runtest->end_state;
	tap_set_end_state(end_state);

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		openjtag_set_state(TAP_IDLE);
		tap_set_state(TAP_IDLE);
	}

	if (cmd->cmd.runtest->num_cycles > 16)
		Transf("num_cycles > 16 on run test");

	if (cmd->cmd.runtest->num_cycles) {
		uint8_t command;
		command = 7;
		command |= ((cmd->cmd.runtest->num_cycles - 1) & 0x0F) << 4;

		openjtag_add_byte(command);
	}

	tap_set_end_state(end_state);
	if (tap_get_end_state() != tap_get_state()) {
		openjtag_set_state(end_state);
		tap_set_state(end_state);
	}
}

static void openjtag_execute_command(struct jtag_command *cmd)
{
	x_out("openjtag_execute_command:", cmd->type);
	switch (cmd->type) {
	case JTAG_RESET:
			openjtag_execute_reset(cmd);
			break;
	case JTAG_SLEEP:
			openjtag_execute_sleep(cmd);
			break;
	case JTAG_TLR_RESET:
			openjtag_execute_statemove(cmd);
			break;
	case JTAG_SCAN:
			openjtag_execute_scan(cmd);
			break;
	case JTAG_RUNTEST:
			openjtag_execute_runtest(cmd);
			break;
	case JTAG_PATHMOVE:
		/* jlink_execute_pathmove(cmd); break; */
	default:
		Transf("BUG: unknown Open JTAG command type encountered");
		exit(-1);
	}
}

static int openjtag_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		openjtag_execute_command(cmd);
		cmd = cmd->next;
	}

	return openjtag_execute_tap_queue();
}

