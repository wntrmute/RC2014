/*
 * The KZ80 is a Z80-based laptop project.
 *
 * Platform features:
 * Z80A @ 7.4MHz
 * 8K EPROM, 56K RAM
 *
 * SIO/2 card
 */
#include <sys/types.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "libz80/z80.h"


/*
 * Right now, the memory isn't pageable, but a future revision
 * will possibly fix that. For now, I'm just going to leave the
 * RAM as the full memory, knowing the bottom 8K isn't used
 * right now.
 */
static uint8_t		 eeprom[8192];
static uint8_t		 ram[65535];

static Z80Context	 cpu_z80;
static volatile int	 done;
static uint8_t		 has_im2;
static uint8_t		 int_recalc = 0;

/* static struct ppide	*ppide; */

static uint16_t		 tstate_steps = 369;     /* RC2014 speed */

/* IRQ source that is live in IM2 */
static uint8_t		 live_irq;

#define IRQ_SIOA        1
#define IRQ_SIOB        2
#define IRQ_CTC		3	/* 3 4 5 6 */


#define	TRACE_MEM	1
#define TRACE_IO	2
#define TRACE_SIO	4
#define TRACE_IRQ	8

static int		trace = 0;
static void		reti_event(void);


static uint8_t
mem_read(int unused, uint16_t addr)
{
	static uint8_t	rstate = 0;
	uint8_t		r;

	if (trace & TRACE_MEM) {
		fprintf(stderr, "MREQ/R %04X ", addr);
	}

	if (addr < 8192) {
		if (trace & TRACE_MEM) {
			fprintf(stderr, "[AT28C256]: ");
		}
		r = eeprom[addr];
	}
	else {
		if (trace & TRACE_MEM) {
			fprintf(stderr, "[CY62256]: ");
		}
		r = ram[addr];
	}

	if (trace & TRACE_MEM) {
		fprintf(stderr, "%02X\n", r);
	}

	if (cpu_z80.M1) {
		/* DD FD CB see the Z80 interrupt manual */
		if (r == 0xDD || r == 0xFD || r == 0xCB) {
			rstate = 2;
			return r;
		}
		/* Look for ED with M1, followed directly by 4D and if so trigger
		   the interrupt chain */
		if (r == 0xED && rstate == 0) {
			rstate = 1;
			return r;
		}
	}

	if (r == 0x4D && rstate == 1)
		reti_event();
	rstate = 0;

	return r;
}


static void
mem_write(int unused, uint16_t addr, uint8_t val)
{
	if (trace & TRACE_MEM) {
		fprintf(stderr, "MREQ/W %04X <- %02X", addr, val);
	}

	if (addr < 8192) {
		/*
		 * In reality, the ROM's write enable line isn't mapped,
		 * so writes are silently ignored.
		 */
		if (trace & TRACE_MEM) {
			fprintf(stderr, " [AT28C256/NOP]\n");
		}
	}
	else {
		if (trace & TRACE_MEM) {
			fprintf(stderr, " [CY62256]\n");
		}
		ram[addr] = val;
	}
}


int
check_chario(void)
{
	fd_set		i, o;
	struct timeval	tv;
	unsigned int	r = 0;

	FD_ZERO(&i);
	FD_SET(0, &i);
	FD_ZERO(&o);
	FD_SET(1, &o);
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if (select(2, &i, NULL, NULL, &tv) == -1) {
		if (errno == EINTR)
			return 0;
		perror("select");
		exit(1);
	}
	if (FD_ISSET(0, &i))
		r |= 1;
	if (FD_ISSET(1, &o))
		r |= 2;
	return r;
}


unsigned int
next_char(void)
{
	char	c;

	if (read(0, &c, 1) != 1) {
		printf("(tty read without ready byte)\n");
		return 0xFF;
	}
	if (c == 0x0A)
		c = '\r';
	return c;
}


void
recalc_interrupts(void)
{
        int_recalc = 1;
}


struct z80_sio_chan {
        uint8_t wr[8];
        uint8_t rr[3];
        uint8_t data[3];
        uint8_t dptr;
        uint8_t irq;
        uint8_t rxint;
        uint8_t txint;
        uint8_t intbits;
#define INT_TX  1
#define INT_RX  2
#define INT_ERR 4
        uint8_t pending;        /* Interrupt bits pending as an IRQ cause */
        uint8_t vector;         /* Vector pending to deliver */
};


static int	sio2_input = 1;
static struct	z80_sio_chan sio[2];

/*
 *      Interrupts. We don't handle IM2 yet.
 */

static void
sio2_clear_int(struct z80_sio_chan *chan, uint8_t m)
{
	if (trace & TRACE_IRQ) {
		fprintf(stderr, "Clear intbits %d %x\n",
				(int)(chan - sio), m);
	}
	chan->intbits &= ~m;
	chan->pending &= ~m;

	/* Check me - does it auto clear down or do you have to reti it ? */
	if (!(sio->intbits | sio[1].intbits)) {
		sio->rr[1] &= ~0x02;
		chan->irq = 0;
	}
	recalc_interrupts();
}


static void
sio2_raise_int(struct z80_sio_chan *chan, uint8_t m)
{
        uint8_t new = (chan->intbits ^ m) & m;
        chan->intbits |= m;
        if ((trace & TRACE_SIO) && new)
                fprintf(stderr, "SIO raise int %x new = %x\n", m, new);
        if (new) {
                if (!sio->irq) {
                        chan->irq = 1;
                        sio->rr[1] |= 0x02;
                        recalc_interrupts();
                }
        }
}


static void
sio2_reti(struct z80_sio_chan *chan)
{
        /* Recalculate the pending state and vectors */
        /* FIXME: what really goes here */
        sio->irq = 0;
        recalc_interrupts();
}


static int
sio2_check_im2(struct z80_sio_chan *chan)
{
        uint8_t	vector = sio[1].wr[2];
        /* See if we have an IRQ pending and if so deliver it and return 1 */
        if (chan->irq) {
                /* Do the vector calculation in the right place */
                /* FIXME: move this to other platforms */
                if (sio[1].wr[1] & 0x04) {
                        /* This is a subset of the real options. FIXME: add
                           external status change */
                        if (sio[1].wr[1] & 0x04) {
                                vector &= 0xF1;
                                if (chan == sio)
                                        vector |= 1 << 3;
                                if (chan->intbits & INT_RX)
                                        vector |= 4;
                                else if (chan->intbits & INT_ERR)
                                        vector |= 2;
                        }
                        if (trace & TRACE_SIO)
                                fprintf(stderr, "SIO2 interrupt %02X\n", vector);
                        chan->vector = vector;
                } else {
                        chan->vector = vector;
                }
                if (trace & (TRACE_IRQ|TRACE_SIO))
                        fprintf(stderr, "New live interrupt pending is SIO (%d:%02X).\n",
                                (int)(chan - sio), chan->vector);
                if (chan == sio)
                        live_irq = IRQ_SIOA;
                else
                        live_irq = IRQ_SIOB;
                Z80INT(&cpu_z80, chan->vector);
                return 1;
        }
        return 0;
}


/*
 *      The SIO replaces the last character in the FIFO on an
 *      overrun.
 */
static void
sio2_queue(struct z80_sio_chan *chan, uint8_t c)
{
	if (trace & TRACE_SIO)
		fprintf(stderr, "SIO %d queue %d: ", (int) (chan - sio), c);
	/* Receive disabled */
	if (!(chan->wr[3] & 1)) {
		fprintf(stderr, "RX disabled.\n");
		return;
	}
	/* Overrun */
	if (chan->dptr == 2) {
		if (trace & TRACE_SIO)
			fprintf(stderr, "Overrun.\n");
		chan->data[2] = c;
		chan->rr[1] |= 0x20;    /* Overrun flagged */
		/* What are the rules for overrun delivery FIXME */
		sio2_raise_int(chan, INT_ERR);
	} else {
		/* FIFO add */
		if (trace & TRACE_SIO)
			fprintf(stderr, "Queued %d (mode %d)\n", chan->dptr, chan->wr[1] & 0x18);
		chan->data[chan->dptr++] = c;
		chan->rr[0] |= 1;
		switch (chan->wr[1] & 0x18) {
			case 0x00:
				break;
			case 0x08:
				if (chan->dptr == 1)
					sio2_raise_int(chan, INT_RX);
				break;
			case 0x10:
			case 0x18:
				sio2_raise_int(chan, INT_RX);
				break;
		}
	}
	/* Need to deal with interrupt results */
}


static void
sio2_channel_timer(struct z80_sio_chan *chan, uint8_t ab)
{
        if (ab == 0) {
                int c = check_chario();

                if (sio2_input) {
                        if (c & 1)
                                sio2_queue(chan, next_char());
                }
                if (c & 2) {
                        if (!(chan->rr[0] & 0x04)) {
                                chan->rr[0] |= 0x04;
                                if (chan->wr[1] & 0x02)
                                        sio2_raise_int(chan, INT_TX);
                        }
                }
        } else {
                if (!(chan->rr[0] & 0x04)) {
                        chan->rr[0] |= 0x04;
                        if (chan->wr[1] & 0x02)
                                sio2_raise_int(chan, INT_TX);
                }
        }
}


static void
sio2_timer(void)
{
        sio2_channel_timer(sio, 0);
        sio2_channel_timer(sio + 1, 1);
}


static void
sio2_channel_reset(struct z80_sio_chan *chan)
{
	chan->rr[0] = 0x2C;
	chan->rr[1] = 0x01;
	chan->rr[2] = 0;
	sio2_clear_int(chan, INT_RX | INT_TX | INT_ERR);
}


static void
sio_reset(void)
{
        sio2_channel_reset(sio);
        sio2_channel_reset(sio + 1);
}


static uint8_t
sio2_read(uint16_t addr)
{
        struct z80_sio_chan *chan = (addr & 2) ? sio + 1 : sio;

        if (!(addr & 1)) {
                /* Control */
                uint8_t r = chan->wr[0] & 007;
                chan->wr[0] &= ~007;

                chan->rr[0] &= ~2;
                if (chan == sio && (sio[0].intbits | sio[1].intbits))
                        chan->rr[0] |= 2;
                if (trace & TRACE_SIO)
                        fprintf(stderr, "sio%c read reg %d = ", (addr & 2) ? 'b' : 'a', r);
                switch (r) {
                case 0:
                case 1:
                        if (trace & TRACE_SIO)
                                fprintf(stderr, "%02X\n", chan->rr[r]);
                        return chan->rr[r];
                case 2:
                        if (chan != sio) {
                                if (trace & TRACE_SIO)
                                        fprintf(stderr, "%02X\n", chan->rr[2]);
                                return chan->rr[2];
                        }
                case 3:
                       /* What does the hw report ?? */
                        fprintf(stderr, "INVALID(0xFF)\n");
                        return 0xFF;
                }
        } else {
                /* FIXME: irq handling */
                uint8_t c = chan->data[0];
                chan->data[0] = chan->data[1];
                chan->data[1] = chan->data[2];
                if (chan->dptr)
                        chan->dptr--;
                if (chan->dptr == 0)
                        chan->rr[0] &= 0xFE;    /* Clear RX pending */
                sio2_clear_int(chan, INT_RX);
                chan->rr[0] &= 0x3F;
                chan->rr[1] &= 0x3F;
                if (trace & TRACE_SIO)
                        fprintf(stderr, "sio%c read data %d\n", (addr & 2) ? 'b' : 'a', c);
                if (chan->dptr && (chan->wr[1] & 0x10))
                        sio2_raise_int(chan, INT_RX);
                return c;
        }
        return 0xFF;
}


static void
sio2_write(uint16_t addr, uint8_t val)
{
        struct z80_sio_chan	*chan = (addr & 2) ? sio + 1 : sio;
        uint8_t			 r;

        if (!(addr & 1)) {
                if (trace & TRACE_SIO)
                        fprintf(stderr, "sio%c write reg %d with %02X\n", (addr & 2) ? 'b' : 'a', chan->wr[0] & 7, val);
                switch (chan->wr[0] & 007) {
                case 0:
                        chan->wr[0] = val;
                       /* FIXME: CRC reset bits ? */
                        switch (val & 070) {
                        case 000:       /* NULL */
                                break;
                        case 010:       /* Send Abort SDLC */
                                /* SDLC specific no-op for async */
                                break;
                        case 020:       /* Reset external/status interrupts */
                                sio2_clear_int(chan, INT_ERR);
                                chan->rr[1] &= 0xCF;    /* Clear status bits on rr0 */
                                break;
                        case 030:       /* Channel reset */
                                if (trace & TRACE_SIO)
                                        fprintf(stderr, "[channel reset]\n");
                                sio2_channel_reset(chan);
                                break;
                        case 040:       /* Enable interrupt on next rx */
                                chan->rxint = 1;
                                break;
                        case 050:       /* Reset transmitter interrupt pending */
                                chan->txint = 0;
                                sio2_clear_int(chan, INT_TX);
                                break;
                        case 060:       /* Reset the error latches */
                                chan->rr[1] &= 0x8F;
                                break;
                        case 070:       /* Return from interrupt (channel A) */
                                if (chan == sio) {
                                        sio->irq = 0;
                                        sio->rr[1] &= ~0x02;
                                        sio2_clear_int(sio, INT_RX | INT_TX | INT_ERR);
                                        sio2_clear_int(sio + 1, INT_RX | INT_TX | INT_ERR);
                                }
                                break;
                        }
                       break;
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                        r = chan->wr[0] & 7;
                        if (trace & TRACE_SIO)
                                fprintf(stderr, "sio%c: wrote r%d to %02X\n",
                                        (addr & 2) ? 'b' : 'a', r, val);
                        chan->wr[r] = val;
                        if (chan != sio && r == 2)
                                chan->rr[2] = val;
                        chan->wr[0] &= ~007;
                        break;
                }
                /* Control */
        } else {
                /* Strictly we should emulate this as two bytes, one going out and
                   the visible queue - FIXME */
                /* FIXME: irq handling */
                chan->rr[0] &= ~(1 << 2);       /* Transmit buffer no longer empty */
                chan->txint = 1;
                /* Should check chan->wr[5] & 8 */
                sio2_clear_int(chan, INT_TX);
                if (trace & TRACE_SIO)
                        fprintf(stderr, "sio%c write data %d\n", (addr & 2) ? 'b' : 'a', val);
                if (chan == sio)
                       write(1, &val, 1);
                else {
//                      write(1, "\033[1m;", 5);
                        write(1, &val,1);
//                      write(1, "\033[0m;", 5);
                }
        }
}


static uint8_t
io_read(int unused, uint16_t addr)
{
	uint8_t	v;
	if (trace & TRACE_IO) {
		fprintf(stderr, "IORQ/R %04X ", addr);
	}
	addr &= 0xFF;

	if ((addr >= 0x80) && (addr <= 0x83)) {
		if (trace & TRACE_IO) {
			fprintf(stderr, " [SIO/2]: ");
		}
		/* return sio2_read(); */
		v = sio2_read(addr & 3);
	}
	else {
		if (trace & TRACE_IO) {
			fprintf(stderr, " [UNKDEV]: ");
		}
		v = 0xFF;
	}

	if (trace & TRACE_IO) {
		fprintf(stderr, "%02X\n", v);
	}
	return v;
}


static void
io_write(int unused, uint16_t addr, uint8_t val)
{
	if (trace & TRACE_IO) {
		fprintf(stderr, "IORQ/W %04X <- %02X: ", addr, val);
	}
	addr &= 0xFF;


	if ((addr >= 0x80) && (addr <= 0x83)) {
		if (trace & TRACE_IO) {
			fprintf(stderr, "[SIO/2]");
		}
		sio2_write(addr & 3, val);
	}
	else {
		if (trace & TRACE_IO) {
			fprintf(stderr, "[UNKDEV]");
		}

	}
}


static void
poll_irq_event(void)
{
	/*
        if (has_im2) {
                if (!live_irq) {
                        !sio2_check_im2(sio) && !sio2_check_im2(sio + 1);
                        // && !ctc_check_im2();
                }
        } else {
                !sio2_check_im2(sio) && !sio2_check_im2(sio + 1);
                // ctc_check_im2();
        }
	 */

	if (sio2_check_im2(sio) == 0) {
		sio2_check_im2(sio + 1);
		
	}
}


static void
reti_event(void)
{
	if (live_irq && (trace & TRACE_IRQ))
		fprintf(stderr, "RETI\n");
	if (has_im2) {
		switch(live_irq) {
		case IRQ_SIOA:
			sio2_reti(sio);
			break;
		case IRQ_SIOB:
			sio2_reti(sio + 1);
			break;
	/*
	 * CTC isn't enabled on the KZ80 (yet!).
		case IRQ_CTC:
		case IRQ_CTC + 1:
		case IRQ_CTC + 2:
		case IRQ_CTC + 3:
			ctc_reti(live_irq - IRQ_CTC);
			break;
	 */
		}
	} else {
		/* If IM2 is not wired then all the things respond at the same
		   time. I think they can also fight over the vector but ignore
		   that */
		sio2_reti(sio);
		sio2_reti(sio + 1);
		/*
		if (have_ctc) {
			ctc_reti(0);
			ctc_reti(1);
			ctc_reti(2);
			ctc_reti(3);
		}
		*/
	}
	live_irq = 0;
	poll_irq_event();
}


static struct termios	saved_term, term;


static void
cleanup(int sig)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
	exit(1);
}


static void
exit_cleanup(void)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
}


static void
usage(void)
{
	fprintf(stderr, "kz80: [-r rompath] [-d tracemask]\n");
	exit(EXIT_FAILURE);
}


int
main(int argc, char *argv[])
{
	static struct timespec	 tc;
	int			 opt;
	int			 fd;
	char			*rompath = "kz80.rom";

	while ((opt = getopt(argc, argv, "d:r:")) != -1) {
		switch (opt) {
		case 'd':
			trace = atoi(optarg);
			break;
		case 'r':
			rompath = optarg;
			break;
		default:
			usage();
		}
	}

	fd = open(rompath, O_RDONLY);	
	if (fd == -1) {
		perror(rompath);
		exit(EXIT_FAILURE);
	}

	if (read(fd, eeprom, 8192) != 8192) {
		fprintf(stderr, "kz80: ROM image should be 8K.\n");
		exit(EXIT_FAILURE);
	}
	close(fd);

	sio_reset();

	/*
	 * No real need for interrupt accuracy so just go with the
	 * timer. If we ever do the UART as timer hack it'll need
	 * addressing!
	 */
    tc.tv_sec = 0;
    tc.tv_nsec = 500000L;

    if (tcgetattr(0, &term) == 0) {
	    saved_term = term;
	    atexit(exit_cleanup);
	    signal(SIGINT, cleanup);
	    signal(SIGQUIT, cleanup);
	    signal(SIGPIPE, cleanup);
	    term.c_lflag &= ~(ICANON|ECHO);
	    term.c_cc[VMIN] = 1;
	    term.c_cc[VTIME] = 0;
	    term.c_cc[VINTR] = 0;
	    term.c_cc[VSUSP] = 0;
	    term.c_cc[VSTOP] = 0;
	    tcsetattr(0, TCSADRAIN, &term);
    }

    Z80RESET(&cpu_z80);
    cpu_z80.ioRead = io_read;
    cpu_z80.ioWrite = io_write;
    cpu_z80.memRead = mem_read;
    cpu_z80.memWrite = mem_write;


     while (!done) {
                int	i;

                /* 36400 T states for base KZ80 - varies for others */
                for (i = 0; i < 100; i++) {
                        Z80ExecuteTStates(&cpu_z80, tstate_steps);
			sio2_timer();
		}
		nanosleep(&tc, NULL);

                if (int_recalc) {
                        /* If there is no pending Z80 vector IRQ but we think
                           there now might be one we use the same logic as for
                           reti */
                        if (!live_irq || !has_im2)
                                poll_irq_event();
                        /* Clear this after because reti_event may set the
                           flags to indicate there is more happening. We will
                           pick up the next state changes on the reti if so */
                        if (!(cpu_z80.IFF1|cpu_z80.IFF2))
                                int_recalc = 0;
                }
     }

    exit(0);
}
