#include <stdio.h>
#include <ftdi.h>
#include <unistd.h>
#include <pthread.h>
#include <inttypes.h>
#include "usb-driver.h"
#include "config.h"
#include "jtagkey.h"
#include "jtagmon.h"

#define USBBUFSIZE 1048576
#define JTAG_SPEED 100000
#define BULK_LATENCY 2
#define OTHER_LATENCY 1

static struct ftdi_context ftdic;

static int jtagkey_latency(int latency) {
	static int current = 0;
	int ret;

	if (current != latency) {
		DPRINTF("switching latency\n");
		if ((ret = ftdi_set_latency_timer(&ftdic, latency))  != 0) {
			fprintf(stderr, "unable to set latency timer: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
			return ret;
		}
		
		current = latency;
	}

	return ret;
}

static int jtagkey_init(unsigned short vid, unsigned short pid, unsigned short iface) {
	int ret = 0;
	unsigned char c;

	if ((ret = ftdi_init(&ftdic)) != 0) {
		fprintf(stderr, "unable to initialise libftdi: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}
	
	if ((ret = ftdi_set_interface(&ftdic, iface)) != 0) {
		fprintf(stderr, "unable to set interface: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}

	if ((ret = ftdi_usb_open(&ftdic, vid, pid)) != 0) {
		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}

	if ((ret = ftdi_usb_reset(&ftdic)) != 0) {
		fprintf(stderr, "unable reset device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}

	if ((ret = ftdi_write_data_set_chunksize(&ftdic, USBBUFSIZE))  != 0) {
		fprintf(stderr, "unable to set write chunksize: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}

	if ((ret = ftdi_read_data_set_chunksize(&ftdic, USBBUFSIZE))  != 0) {
		fprintf(stderr, "unable to set read chunksize: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}

	if ((ret = jtagkey_latency(OTHER_LATENCY)) != 0)
		return ret;

	c = 0x00;
	ftdi_write_data(&ftdic, &c, 1);

	if ((ret = ftdi_set_bitmode(&ftdic, JTAGKEY_TCK|JTAGKEY_TDI|JTAGKEY_TMS|JTAGKEY_LED, BITMODE_SYNCBB))  != 0) {
		fprintf(stderr, "unable to enable bitbang mode: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}

	if ((ret = ftdi_set_baudrate(&ftdic, JTAG_SPEED))  != 0) {
		fprintf(stderr, "unable to set baudrate: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}

	if ((ret = ftdi_usb_purge_buffers(&ftdic))  != 0) {
		fprintf(stderr, "unable to purge buffers: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return ret;
	}

	return ret;
}

int jtagkey_open(int num) {
	int ret;

	ret = jtagkey_init(config_usb_vid(num), config_usb_pid(num), config_usb_iface(num));

	if (ret >= 0)
		ret = 0xff;

	return ret;
}

void jtagkey_close(int handle) {
	if (handle == 0xff) {
		ftdi_disable_bitbang(&ftdic);
		ftdi_usb_close(&ftdic);
		ftdi_deinit(&ftdic);
	}
}

#ifdef DEBUG
static void jtagkey_state(unsigned char data) {
	fprintf(stderr,"Pins high: ");

	if (data & JTAGKEY_TCK)
		fprintf(stderr,"TCK ");

	if (data & JTAGKEY_TDI)
		fprintf(stderr,"TDI ");

	if (data & JTAGKEY_TDO)
		fprintf(stderr,"TDO ");

	if (data & JTAGKEY_TMS)
		fprintf(stderr,"TMS ");

	fprintf(stderr,"\n");
}
#endif

struct parallelport_emu {
        union {
                unsigned char val;
                struct databits {
                        unsigned TDI : 1;
                        unsigned TCK : 1;
                        unsigned TMS : 1;
                        unsigned CTRL : 1;
                        unsigned PROG : 1;
                        unsigned d5 : 1;
                        unsigned d6 : 1;
                        unsigned d7 : 1;
                } bits;
        } data;
        union {
                unsigned char val;
                struct statusbits {
                        unsigned b0 : 1;
                        unsigned b1 : 1;
                        unsigned b2 : 1;
                        unsigned SENSE : 1;
                        unsigned TDO : 1;
                        unsigned b5 : 1;
                        unsigned b6 : 1;
                        unsigned b7 : 1;
                } bits;
        } status;

} pp;


unsigned char writebuf[USBBUFSIZE];
int writepos = 0;
unsigned char readbuf;

static void jtagkey_run(unsigned char mustRead)
{
        jtagkey_latency(OTHER_LATENCY);
        DPRINTF("--> write %d\n", writepos);
        ftdi_write_data(&ftdic, writebuf, writepos);
        writepos = 0;
        if(mustRead)
        {
                DPRINTF("<-- read %d\n", 1);
                ftdi_read_data(&ftdic, &readbuf, 1);
        }
}

static void jtagkey_updatepp()
{
        //Emulate random hardware link of D6, BUSY and PE
        pp.status.bits.b7 = !pp.data.bits.d6;
        pp.status.bits.b5 = pp.data.bits.d6;

        //Emulate VREF
        pp.status.bits.SENSE = !pp.data.bits.PROG;
        pp.status.bits.TDO = !pp.data.bits.PROG;
}

int jtagkey_transfer(WD_TRANSFER *tr, int fd, unsigned int request, int ppbase, int ecpbase, int num) 
{
        int i;
        unsigned long port;
        unsigned char val;
        unsigned char mustWrite = 0;

        pp.data.val = 0;
        pp.status.val = 0;

        jtagkey_updatepp();

        for(i=0; i<num; i++)
        {
                port = (unsigned long)tr[i].dwPort - ppbase;
                val = tr[i].Data.Byte;

                //Update pp registers
                switch(port)
                {
                        case PP_DATA:
                                DPRINTF("data port\n");
                                switch(tr[i].cmdTrans)
                                {
                                        case PP_READ:
                                                jtagkey_updatepp();
                                                DPRINTF("< data-read %x\n",pp.data.val);
                                                tr[i].Data.Byte = pp.data.val;
                                                break;
                                        case PP_WRITE:
                                                pp.data.val = val;
                                                mustWrite = 1;
                                                break;
                                }
                                break;
                        case PP_STATUS:
                                DPRINTF("status port\n");
                                switch(tr[i].cmdTrans)
                                {
                                        case PP_READ:
                                                jtagkey_updatepp();
                                                if(writepos > 0)
                                                {
                                                        writebuf[writepos-1] |= JTAGKEY_RD;
                                                }
                                                else
                                                {
                                                        writebuf[0] = JTAGKEY_RD;
                                                        writepos++;
                                                }
                                                jtagkey_run(1);

                                                if(readbuf & JTAGKEY_TDO)
                                                        pp.status.bits.TDO = 1;
                                                else
                                                        pp.status.bits.TDO = 0;

                                                DPRINTF("< status-read %x\n",pp.status.val);

                                                tr[i].Data.Byte = pp.status.val;
                                                break;
                                        case PP_WRITE:
                                                pp.status.val = val;
                                                break;
                                }
                                break;
                }

                //Add to writebuf
                if(mustWrite)
                {
                        unsigned char data = 0x00;
                        if(pp.data.bits.TDI)
                                data |= JTAGKEY_TDI;
                        if(pp.data.bits.TCK)
                                data |= JTAGKEY_TCK;
                        if(pp.data.bits.TMS)
                                data |= JTAGKEY_TMS;


                        DPRINTF("> store-write %x of %x\n",data,pp.data.val);

                        writebuf[writepos] = data;
                        writepos++;

                        mustWrite = 0;
                }

                if(writepos == (USBBUFSIZE-1))
                        jtagkey_run(0);
        }


        return 0;
}
