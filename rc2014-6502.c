/*
 *	Platform features
 *
 *	6502 processor card for RC2014 set to invert A15 and an I/O window
 *	at $C000 (silly place for it but that is where the board put it)
 *	Zilog SIO/2 at 0x80-0x83
 *	Motorola 6850 repeats all over 0x40-0x7F (not recommended)
 *	IDE at 0x10-0x17 no high or control access
 *	Memory banking Zeta style 16K page at 0x78-0x7B (enable at 0x7C)
 *	First 512K ROM Second 512K RAM (0-31, 32-63)
 *	Z80 CTC (6502 can never clear an IRQ caused by this so be careful!)
 *	RTC at 0xC0
 *	8085 bitbang port also wired to the M1 line (to test an experimental idea)
 *	16550A at 0xC0 (can't be used with RTC present)
 *
 *	FIXME: Need to emulate 6522 VIA card ? or TMS9918A for IRQ at least
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include "6502.h"
#include "ide.h"
#include "w5100.h"

static uint8_t ramrom[1024 * 1024];	/* Covers the banked card */

static unsigned int bankreg[4];
static uint8_t bankenable;

static uint8_t have_ctc = 0;
static uint8_t rtc = 0;
static uint8_t fast = 0;
static uint8_t wiznet = 0;
static uint8_t iopage = 0xC0;
static uint16_t addrinvert = 0x0000;

static uint8_t fake_m1;

static uint16_t tstate_steps = 200;	/* 4MHz */

/* Who is pulling on the interrupt line */

static uint8_t live_irq;

#define IRQ_SIOA	1
#define IRQ_SIOB	2
#define IRQ_CTC		3
#define IRQ_ACIA	4
#define IRQ_16550A	5
#define IRQ_VIA		6

static nic_w5100_t *wiz;

static volatile int done;

#define TRACE_MEM	1
#define TRACE_IO	2
#define TRACE_IRQ	4
#define TRACE_UNK	8
#define TRACE_SIO	16
#define TRACE_512	32
#define TRACE_RTC	64
#define TRACE_CPU	128
#define TRACE_CTC	256
#define TRACE_ACIA	512
#define TRACE_UART	2048
#define TRACE_VIA	4096

static int trace = 0;

static void reti_event(void);


static int check_chario(void)
{
	fd_set i, o;
	struct timeval tv;
	unsigned int r = 0;

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

static unsigned int next_char(void)
{
	char c;
	if (read(0, &c, 1) != 1) {
		printf("(tty read without ready byte)\n");
		return 0xFF;
	}
	if (c == 0x0A)
		c = '\r';
	return c;
}

static void int_set(int src)
{
	live_irq |= (1 << src);
}

static void int_clear(int src)
{
	live_irq &= ~(1 << src);
}


/* FIXME: need to add 16550A */

static uint8_t acia_status = 2;
static uint8_t acia_config;
static uint8_t acia_char;
static uint8_t acia;
static uint8_t acia_input;
static uint8_t acia_inint = 0;
static uint8_t acia_narrow;

static void acia_irq_compute(void)
{
	if (!acia_inint && acia_config && acia_status & 0x80) {
		if (trace & TRACE_ACIA)
			fprintf(stderr, "ACIA interrupt.\n");
		acia_inint = 1;
		int_set(IRQ_ACIA);
	}
	if (acia_inint) {
		int_clear(IRQ_ACIA);
		acia_inint = 0;
	}
}

static void acia_receive(void)
{
	uint8_t old_status = acia_status;
	acia_status = old_status & 0x02;
	if (old_status & 1)
		acia_status |= 0x20;
	acia_char = next_char();
	if (trace & TRACE_ACIA)
		fprintf(stderr, "ACIA rx.\n");
	acia_status |= 0x81;	/* IRQ, and rx data full */
}

static void acia_transmit(void)
{
	if (!(acia_status & 2)) {
		if (trace & TRACE_ACIA)
			fprintf(stderr, "ACIA tx is clear.\n");
		acia_status |= 0x82;	/* IRQ, and tx data empty */
	}
}

static void acia_timer(void)
{
	int s = check_chario();
	if ((s & 1) && acia_input)
		acia_receive();
	if (s & 2)
		acia_transmit();
	if (s)
		acia_irq_compute();
}

/* Very crude for initial testing ! */
static uint8_t acia_read(uint8_t addr)
{
	if (trace & TRACE_ACIA)
		fprintf(stderr, "acia_read %d ", addr);
	switch (addr) {
	case 0:
		/* bits 7: irq pending, 6 parity error, 5 rx over
		 * 4 framing error, 3 cts, 2 dcd, 1 tx empty, 0 rx full.
		 * Bits are set on char arrival and cleared on next not by
		 * user
		 */
		acia_status &= ~0x80;
		acia_irq_compute();
		if (trace & TRACE_ACIA)
			fprintf(stderr, "acia_status %d\n", acia_status);
		return acia_status;
	case 1:
		acia_status &= ~0x81;	/* No IRQ, rx empty */
		acia_irq_compute();
		if (trace & TRACE_ACIA)
			fprintf(stderr, "acia_char %d\n", acia_char);
		return acia_char;
	default:
		fprintf(stderr, "acia: bad addr.\n");
		exit(1);
	}
}

static void acia_write(uint16_t addr, uint8_t val)
{
	if (trace & TRACE_ACIA)
		fprintf(stderr, "acia_write %d %d\n", addr, val);
	switch (addr) {
	case 0:
		/* bit 7 enables interrupts, bits 5-6 are tx control
		   bits 2-4 select the word size and 0-1 counter divider
		   except 11 in them means reset */
		acia_config = val;
		if ((acia_config & 3) == 3)
			acia_status = 2;
		acia_irq_compute();
		return;
	case 1:
		write(1, &val, 1);
		/* Clear any existing int state and tx empty */
		acia_status &= ~0x82;
		break;
	}
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
#define INT_TX	1
#define INT_RX	2
#define INT_ERR	4
	uint8_t pending;	/* Interrupt bits pending as an IRQ cause */
	uint8_t vector;		/* Vector pending to deliver */
};

static int sio2;
static int sio2_input;
static struct z80_sio_chan sio[2];

/*
 *	Interrupts. We don't handle IM2 yet.
 */

static void sio2_clear_int(struct z80_sio_chan *chan, uint8_t m)
{
	if (trace & TRACE_IRQ) {
		fprintf(stderr, "Clear intbits %d %x\n",
			(int)(chan - sio), m);
	}
	chan->intbits &= ~m;
	chan->pending &= ~m;
	if (!(sio->intbits | sio[1].intbits)) {
		sio->rr[1] &= ~0x02;
		chan->irq = 0;
	}
}

static void sio2_raise_int(struct z80_sio_chan *chan, uint8_t m)
{
	uint8_t new = (chan->intbits ^ m) & m;
	uint8_t vector;
	chan->intbits |= m;
	if ((trace & TRACE_SIO) && new)
		fprintf(stderr, "SIO raise int %x new = %x\n", m, new);
	if (new) {
		if (!sio->irq) {
			chan->irq = 1;
			sio->rr[1] |= 0x02;
			vector = 0; /* sio[1].wr[2]; */
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
		}
	}
}

static int sio2_check_im2(struct z80_sio_chan *chan)
{
	int src;
	/* See if we have an IRQ pending and if so deliver it and return 1 */
	if (chan->irq) {
		/* FIXME: quick fix for now but the vector calculation should all be
		   done here it seems */
		if (sio[1].wr[1] & 0x04)
			chan->vector += (sio[1].wr[2] & 0xF1);
		else
			chan->vector += sio[1].wr[2];
		if (trace & (TRACE_IRQ|TRACE_SIO))
			fprintf(stderr, "New live interrupt pending is SIO (%d:%02X).\n",
				(int)(chan - sio), chan->vector);
		if (chan == sio)
			src = IRQ_SIOA;
		else
			src = IRQ_SIOB;
		int_set(src);
		return 1;
	}
	return 0;
}

static void sio2_reti(struct z80_sio_chan *chan)
{
	/* Recalculate the pending state and vectors */
	/* FIXME: what really goes here */
	sio->irq = 0;
	int_clear(IRQ_SIOA);
	int_clear(IRQ_SIOB);
	sio2_check_im2(chan);
}


/*
 *	The SIO replaces the last character in the FIFO on an
 *	overrun.
 */
static void sio2_queue(struct z80_sio_chan *chan, uint8_t c)
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
		chan->rr[1] |= 0x20;	/* Overrun flagged */
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

static void sio2_channel_timer(struct z80_sio_chan *chan, uint8_t ab)
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
		if (!(chan->rr[1] & 0x04)) {
			chan->rr[1] |= 0x04;
			if (chan->wr[1] & 0x02)
				sio2_raise_int(chan, INT_TX);
		}
	}
}

static void sio2_timer(void)
{
	sio2_channel_timer(sio, 0);
	sio2_channel_timer(sio + 1, 1);
}

static void sio2_channel_reset(struct z80_sio_chan *chan)
{
	chan->rr[0] = 0x2C;
	chan->rr[1] = 0x01;
	chan->rr[2] = 0;
	sio2_clear_int(chan, INT_RX | INT_TX | INT_ERR);
}

static void sio_reset(void)
{
	sio2_channel_reset(sio);
	sio2_channel_reset(sio + 1);
}

static uint8_t sio2_read(uint16_t addr)
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
			chan->rr[0] &= 0xFE;	/* Clear RX pending */
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

static void sio2_write(uint16_t addr, uint8_t val)
{
	struct z80_sio_chan *chan = (addr & 2) ? sio + 1 : sio;
	uint8_t r;
	if (!(addr & 1)) {
		if (trace & TRACE_SIO)
			fprintf(stderr, "sio%c write reg %d with %02X\n", (addr & 2) ? 'b' : 'a', chan->wr[0] & 7, val);
		switch (chan->wr[0] & 007) {
		case 0:
			chan->wr[0] = val;
			/* FIXME: CRC reset bits ? */
			switch (val & 070) {
			case 000:	/* NULL */
				break;
			case 010:	/* Send Abort SDLC */
				/* SDLC specific no-op for async */
				break;
			case 020:	/* Reset external/status interrupts */
				sio2_clear_int(chan, INT_ERR);
				chan->rr[1] &= 0xCF;	/* Clear status bits on rr0 */
				break;
			case 030:	/* Channel reset */
				if (trace & TRACE_SIO)
					fprintf(stderr, "[channel reset]\n");
				sio2_channel_reset(chan);
				break;
			case 040:	/* Enable interrupt on next rx */
				chan->rxint = 1;
				break;
			case 050:	/* Reset transmitter interrupt pending */
				chan->txint = 0;
				sio2_clear_int(chan, INT_TX);
				break;
			case 060:	/* Reset the error latches */
				chan->rr[1] &= 0x8F;
				break;
			case 070:	/* Return from interrupt (channel A) */
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
		chan->rr[0] &= ~(1 << 2);	/* Transmit buffer no longer empty */
		chan->txint = 1;
		/* Should check chan->wr[5] & 8 */
		sio2_clear_int(chan, INT_TX);
		if (trace & TRACE_SIO)
			fprintf(stderr, "sio%c write data %d\n", (addr & 2) ? 'b' : 'a', val);
		write(1, &val, 1);
	}
}

/* UART: very mimimal for the moment */

struct uart16x50 {
    uint8_t ier;
    uint8_t iir;
    uint8_t fcr;
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr;
    uint8_t msr;
    uint8_t scratch;
    uint8_t ls;
    uint8_t ms;
    uint8_t dlab;
    uint8_t irq;
#define RXDA	1
#define TEMT	2
#define MODEM	8
    uint8_t irqline;
};

static struct uart16x50 uart;
static unsigned int uart_16550a;

static void uart_init(struct uart16x50 *uptr)
{
    uptr->dlab = 0;
}

/* Compute the interrupt indicator register from what is pending */
static void uart_recalc_iir(struct uart16x50 *uptr)
{
    if (uptr->irq & RXDA)
        uptr->iir = 0x04;
    else if (uptr->irq & TEMT)
        uptr->iir = 0x02;
    else if (uptr->irq & MODEM)
        uptr->iir = 0x00;
    else {
        uptr->iir = 0x01;	/* No interrupt */
        uptr->irqline = 0;
        int_clear(IRQ_16550A);
        return;
    }
    /* Ok so we have an event, do we need to waggle the line */
    if (uptr->irqline)
        return;
    uptr->irqline = uptr->irq;
    int_set(IRQ_16550A);
}

/* Raise an interrupt source. Only has an effect if enabled in the ier */
static void uart_interrupt(struct uart16x50 *uptr, uint8_t n)
{
    if (uptr->irq & n)
        return;
    if (!(uptr->ier & n))
        return;
    uptr->irq |= n;
    uart_recalc_iir(uptr);
}

static void uart_clear_interrupt(struct uart16x50 *uptr, uint8_t n)
{
    if (!(uptr->irq & n))
        return;
    uptr->irq &= ~n;
    uart_recalc_iir(uptr);
}

static void uart_event(struct uart16x50 *uptr)
{
    uint8_t r = check_chario();
    uint8_t old = uptr->lsr;
    uint8_t dhigh;
    if (r & 1)
        uptr->lsr |= 0x01;	/* RX not empty */
    if (r & 2)
        uptr->lsr |= 0x60;	/* TX empty */
    dhigh = (old ^ uptr->lsr);
    dhigh &= uptr->lsr;		/* Changed high bits */
    if (dhigh & 1)
        uart_interrupt(uptr, RXDA);
    if (dhigh & 0x2)
        uart_interrupt(uptr, TEMT);
}

static void show_settings(struct uart16x50 *uptr)
{
    uint32_t baud;

    if (!(trace & TRACE_UART))
        return;

    baud = uptr->ls + (uptr->ms << 8);
    if (baud == 0)
        baud = 1843200;
    baud = 1843200 / baud;
    baud /= 16;
    fprintf(stderr, "[%d:%d",
            baud, (uptr->lcr &3) + 5);
    switch(uptr->lcr & 0x38) {
        case 0x00:
        case 0x10:
        case 0x20:
        case 0x30:
            fprintf(stderr, "N");
            break;
        case 0x08:
            fprintf(stderr, "O");
            break;
        case 0x18:
            fprintf(stderr, "E");
            break;
        case 0x28:
            fprintf(stderr, "M");
            break;
        case 0x38:
            fprintf(stderr, "S");
            break;
    }
    fprintf(stderr, "%d ",
            (uptr->lcr & 4) ? 2 : 1);

    if (uptr->lcr & 0x40)
        fprintf(stderr, "break ");
    if (uptr->lcr & 0x80)
        fprintf(stderr, "dlab ");
    if (uptr->mcr & 1)
        fprintf(stderr, "DTR ");
    if (uptr->mcr & 2)
        fprintf(stderr, "RTS ");
    if (uptr->mcr & 4)
        fprintf(stderr, "OUT1 ");
    if (uptr->mcr & 8)
        fprintf(stderr, "OUT2 ");
    if (uptr->mcr & 16)
        fprintf(stderr, "LOOP ");
    fprintf(stderr, "ier %02x]\n", uptr->ier);
}

static void uart_write(struct uart16x50 *uptr, uint8_t addr, uint8_t val)
{
    switch(addr) {
    case 0:	/* If dlab = 0, then write else LS*/
        if (uptr->dlab == 0) {
            if (uptr == &uart) {
                putchar(val);
                fflush(stdout);
            }
            uart_clear_interrupt(uptr, TEMT);
            uart_interrupt(uptr, TEMT);
        } else {
            uptr->ls = val;
            show_settings(uptr);
        }
        break;
    case 1:	/* If dlab = 0, then IER */
        if (uptr->dlab) {
            uptr->ms= val;
            show_settings(uptr);
        }
        else
            uptr->ier = val;
        break;
    case 2:	/* FCR */
        uptr->fcr = val & 0x9F;
        break;
    case 3:	/* LCR */
        uptr->lcr = val;
        uptr->dlab = (uptr->lcr & 0x80);
        show_settings(uptr);
        break;
    case 4:	/* MCR */
        uptr->mcr = val & 0x3F;
        show_settings(uptr);
        break;
    case 5:	/* LSR (r/o) */
        break;
    case 6:	/* MSR (r/o) */
        break;
    case 7:	/* Scratch */
        uptr->scratch = val;
        break;
    }
}

static uint8_t uart_read(struct uart16x50 *uptr, uint8_t addr)
{
    uint8_t r;

    switch(addr) {
    case 0:
        /* receive buffer */
        if (uptr == &uart && uptr->dlab == 0) {
            uart_clear_interrupt(uptr, RXDA);
            return next_char();
        }
        break;
    case 1:
        /* IER */
        return uptr->ier;
    case 2:
        /* IIR */
        return uptr->iir;
    case 3:
        /* LCR */
        return uptr->lcr;
    case 4:
        /* mcr */
        return uptr->mcr;
    case 5:
        /* lsr */
        r = check_chario();
        uptr->lsr = 0;
        if (r & 1)
             uptr->lsr |= 0x01;	/* Data ready */
        if (r & 2)
             uptr->lsr |= 0x60;	/* TX empty | holding empty */
        /* Reading the LSR causes these bits to clear */
        r = uptr->lsr;
        uptr->lsr &= 0xF0;
        return r;
    case 6:
        /* msr */
        r = uptr->msr;
        /* Reading clears the delta bits */
        uptr->msr &= 0xF0;
        uart_clear_interrupt(uptr, MODEM);
        return r;
    case 7:
        return uptr->scratch;
    }
    return 0xFF;
}

static int ide = 0;
struct ide_controller *ide0;

static uint8_t my_ide_read(uint16_t addr)
{
	return ide_read8(ide0, addr);
}

static void my_ide_write(uint16_t addr, uint8_t val)
{
	ide_write8(ide0, addr, val);
}

/* Real time clock state machine and related state.

   Give the host time and don't emulate time setting except for
   the 24/12 hour setting.
   
 */
static uint8_t rtcw;
static uint8_t rtcst;
static uint16_t rtcr;
static uint8_t rtccnt;
static uint8_t rtcstate;
static uint8_t rtcreg;
static uint8_t rtcram[32];
static uint8_t rtcwp = 0x80;
static uint8_t rtc24 = 1;
static uint8_t rtcbp = 0;
static uint8_t rtcbc = 0;
static struct tm *rtc_tm;

static uint8_t rtc_read(void)
{
	if (rtcst & 0x30)
		return (rtcr & 0x01) ? 1 : 0;
	return 0xFF;
}

static uint16_t rtcregread(uint8_t reg)
{
	uint8_t val, v;

	switch (reg) {
	case 0:
		val = (rtc_tm->tm_sec % 10) + ((rtc_tm->tm_sec / 10) << 4);
		break;
	case 1:
		val = (rtc_tm->tm_min % 10) + ((rtc_tm->tm_min / 10) << 4);
		break;
	case 2:
		v = rtc_tm->tm_hour;
		if (!rtc24) {
			v %= 12;
			v++;
		}
		val = (v % 10) + ((v / 10) << 4);
		if (!rtc24) {
			if (rtc_tm->tm_hour > 11)
				val |= 0x20;
			val |= 0x80;
		}
		break;
	case 3:
		val = (rtc_tm->tm_mday % 10) + ((rtc_tm->tm_mday / 10) << 4);
		break;
	case 4:
		val = ((rtc_tm->tm_mon + 1) % 10) + (((rtc_tm->tm_mon + 1) / 10) << 4);
		break;
	case 5:
		val = rtc_tm->tm_wday + 1;
		break;
	case 6:
		v = rtc_tm->tm_year % 100;
		val = (v % 10) + ((v / 10) << 4);
		break;
	case 7:
		val = rtcwp ? 0x80 : 0x00;
		break;
	case 8:
		val = 0;
		break;
	default:
		val = 0xFF;
		/* Check */
		break;
	}
	if (trace & TRACE_RTC)
		fprintf(stderr, "RTCreg %d = %02X\n", reg, val);
	return val;
}

static void rtcop(void)
{
	if (trace & TRACE_RTC)
		fprintf(stderr, "rtcbyte %02X\n", rtcw);
	/* The emulated task asked us to write a byte, and has now provided
	   the data byte to go with it */
	if (rtcstate == 2) {
		if (!rtcwp) {
			if (trace & TRACE_RTC)
				fprintf(stderr, "RTC write %d as %d\n", rtcreg, rtcw);
			/* We did a real write! */
			/* Not yet tackled burst mode */
			if (rtcreg != 0x3F && (rtcreg & 0x20))	/* NVRAM */
				rtcram[rtcreg & 0x1F] = rtcw;
			else if (rtcreg == 2)
				rtc24 = rtcw & 0x80;
			else if (rtcreg == 7)
				rtcwp = rtcw & 0x80;
		}
		/* For now don't emulate writes to the time */
		rtcstate = 0;
	}
	/* Check for broken requests */
	if (!(rtcw & 0x80)) {
		if (trace & TRACE_RTC)
			fprintf(stderr, "rtcw makes no sense %d\n", rtcw);
		rtcstate = 0;
		rtcr = 0x1FF;
		return;
	}
	/* Clock burst ? : for now we only emulate time burst */
	if (rtcw == 0xBF) {
		rtcstate = 3;
		rtcbp = 0;
		rtcbc = 0;
		rtcr = rtcregread(rtcbp++) << 1;
		if (trace & TRACE_RTC)
			fprintf(stderr, "rtc command BF: burst clock read.\n");
		return;
	}
	/* A write request */
	if (!(rtcw & 0x01)) {
		if (trace & TRACE_RTC)
			fprintf(stderr, "rtc write request, waiting byte 2.\n");
		rtcstate = 2;
		rtcreg = (rtcw >> 1) & 0x3F;
		rtcr = 0x1FF;
		return;
	}
	/* A read request */
	rtcstate = 1;
	if (rtcw & 0x40) {
		/* RAM */
		if (rtcw != 0xFE)
			rtcr = rtcram[(rtcw >> 1) & 0x1F] << 1;
		if (trace & TRACE_RTC)
			fprintf(stderr, "RTC RAM read %d, ready to clock out %d.\n", (rtcw >> 1) & 0xFF, rtcr);
		return;
	}
	/* Register read */
	rtcr = rtcregread((rtcw >> 1) & 0x1F) << 1;
	if (trace & TRACE_RTC)
		fprintf(stderr, "RTC read of time register %d is %d\n", (rtcw >> 1) & 0x1F, rtcr);
}

static void rtc_write(uint8_t val)
{
	uint8_t changed = val ^ rtcst;
	uint8_t is_read;
	/* Direction */
	if ((trace & TRACE_RTC) && (changed & 0x20))
		fprintf(stderr, "RTC direction now %s.\n", (val & 0x20) ? "read" : "write");
	is_read = val & 0x20;
	/* Clock */
	if (changed & 0x40) {
		/* The rising edge samples, the falling edge clocks receive */
		if (trace & TRACE_RTC)
			fprintf(stderr, "RTC clock low.\n");
		if (!(val & 0x40)) {
			rtcr >>= 1;
			/* Burst read of time */
			rtcbc++;
			if (rtcbc == 8 && rtcbp) {
				rtcr = rtcregread(rtcbp++) << 1;
				rtcbc = 0;
			}
			if (trace & TRACE_RTC)
				fprintf(stderr, "rtcr now %02X\n", rtcr);
		} else {
			if (trace & TRACE_RTC)
				fprintf(stderr, "RTC clock high.\n");
			rtcw >>= 1;
			if ((val & 0x30) == 0x10)
				rtcw |= val & 0x80;
			else
				rtcw |= 0xFF;
			rtccnt++;
			if (trace & TRACE_RTC)
				fprintf(stderr, "rtcw now %02x (%d)\n", rtcw, rtccnt);
			if (rtccnt == 8 && !is_read)
				rtcop();
		}
	}
	/* CE */
	if (changed & 0x10) {
		if (rtcst & 0x10) {
			if (trace & TRACE_RTC)
				fprintf(stderr, "RTC CE dropped.\n");
			rtccnt = 0;
			rtcr = 0;
			rtcw = 0;
			rtcstate = 0;
		} else {
			/* Latch imaginary registers on rising edge */
			time_t t = time(NULL);
			rtc_tm = localtime(&t);
			if (trace & TRACE_RTC)
				fprintf(stderr, "RTC CE raised and latched time.\n");
		}
	}
	rtcst = val;
}


/*
 *	Z80 CTC
 */

struct z80_ctc {
	uint16_t count;
	uint16_t reload;
	uint8_t vector;
	uint8_t ctrl;
#define CTC_IRQ		0x80
#define CTC_COUNTER	0x40
#define CTC_PRESCALER	0x20
#define CTC_RISING	0x10
#define CTC_PULSE	0x08
#define CTC_TCONST	0x04
#define CTC_RESET	0x02
#define CTC_CONTROL	0x01
	uint8_t irq;		/* Only valid for channel 0, so we know
				   if we must wait for a RETI before doing
				   a further interrupt */
};

#define CTC_STOPPED(c)	(((c)->ctrl & (CTC_TCONST|CTC_RESET)) == (CTC_TCONST|CTC_RESET))

struct z80_ctc ctc[4];
uint8_t ctc_irqmask;

static void ctc_reset(struct z80_ctc *c)
{
	c->vector = 0;
	c->ctrl = CTC_RESET;
}

static void ctc_init(void)
{
	ctc_reset(ctc);
	ctc_reset(ctc + 1);
	ctc_reset(ctc + 2);
	ctc_reset(ctc + 3);
	int_clear(IRQ_CTC);
}

/* Flag a channel as interrupting */
static void ctc_interrupt(struct z80_ctc *c)
{
	int i = c - ctc;
	if (c->ctrl & CTC_IRQ) {
		if (!(ctc_irqmask & (1 << i))) {
			ctc_irqmask |= 1 << i;
			if (trace & TRACE_CTC)
				fprintf(stderr, "CTC %d wants to interrupt.\n", i);
		}
	}
}

static void ctc_reti(int ctcnum)
{
	if (ctc_irqmask & (1 << ctcnum)) {
		ctc_irqmask &= ~(1 << ctcnum);
		if (trace & TRACE_IRQ)
			fprintf(stderr, "Acked interrupt from CTC %d.\n", ctcnum);
	}
}


/* Check for a CTC interrupt but also check to see if we have changed state
   and dropped int. This needs careful thought because we might take an interrupt
   and move past it before our CPU emulation notices.
   
   Really we need an Intel 8253 board for RC2014! */

static int ctc_check_im2(void)
{
	if (ctc_irqmask) {
		int i;
		for (i = 0; i < 4; i++) {	/* FIXME: correct order ? */
			if (ctc_irqmask & (1 << i)) {
				uint8_t vector = ctc[0].vector & 0xF8;
				vector += 2 * i;
				if (trace & TRACE_IRQ)
					fprintf(stderr, "New live interrupt is from CTC %d vector %x.\n", i, vector);
				int_set(IRQ_CTC);
				return 1;
			}
		}
	}
	return 0;
}

/* Model the chains between the CTC devices */
static void ctc_receive_pulse(int i);

static void ctc_pulse(int i)
{
	/* Model CTC 2 chained into CTC 3 */
	if (i == 2)
		ctc_receive_pulse(3);
}

/* We don't worry about edge directions just a logical pulse model */
static void ctc_receive_pulse(int i)
{
	struct z80_ctc *c = ctc + i;
	if (c->ctrl & CTC_COUNTER) {
		if (CTC_STOPPED(c))
			return;
		if (c->count >= 0x0100)
			c->count -= 0x100;	/* No scaling on pulses */
		if ((c->count & 0xFF00) == 0) {
			ctc_interrupt(c);
			ctc_pulse(i);
			c->count = c->reload << 8;
		}
	} else {
		if (c->ctrl & CTC_PULSE)
			c->ctrl &= ~CTC_PULSE;
	}
}

/* Model counters */
static void ctc_tick(unsigned int clocks)
{
	struct z80_ctc *c = ctc;
	int i;
	int n;
	int decby;

	for (i = 0; i < 4; i++, c++) {
		/* Waiting a value */
		if (CTC_STOPPED(c))
			continue;
		/* Pulse trigger mode */
		if (c->ctrl & CTC_COUNTER)
			continue;
		/* 256x downscaled */
		decby = clocks;
		/* 16x not 256x downscale - so increase by 16x */
		if (!(c->ctrl & CTC_PRESCALER))
			decby <<= 4;
		/* Now iterate over the events. We need to deal with wraps
		   because we might have something counters chained */
		n = c->count - decby;
		while (n < 0) {
			ctc_interrupt(c);
			ctc_pulse(i);
			if (c->reload == 0)
				n += 256 << 8;
			else
				n += c->reload << 8;
		}
		c->count = n;
	}
}

static void ctc_write(uint8_t channel, uint8_t val)
{
	struct z80_ctc *c = ctc + channel;
	if (c->ctrl & CTC_TCONST) {
		if (trace & TRACE_CTC)
			fprintf(stderr, "CTC %d constant loaded with %02X\n", channel, val);
		c->reload = val;
		if ((c->ctrl & (CTC_TCONST|CTC_RESET)) == (CTC_TCONST|CTC_RESET)) {
			c->count = (c->reload - 1) << 8;
			if (trace & TRACE_CTC)
				fprintf(stderr, "CTC %d constant reloaded with %02X\n", channel, val);
		}
		c->ctrl &= ~CTC_TCONST|CTC_RESET;
	} else if (val & CTC_CONTROL) {
		/* We don't yet model the weirdness around edge wanted
		   toggling and clock starts */
		if (trace & TRACE_CTC)
			fprintf(stderr, "CTC %d control loaded with %02X\n", channel, val);
		c->ctrl = val;
		if ((c->ctrl & (CTC_TCONST|CTC_RESET)) == CTC_RESET) {
			c->count = (c->reload - 1) << 8;
			if (trace & TRACE_CTC)
				fprintf(stderr, "CTC %d constant reloaded with %02X\n", channel, val);
		}
		/* Undocumented */
		if (!(c->ctrl & CTC_IRQ) && (ctc_irqmask & (1 << channel))) {
			ctc_irqmask &= ~(1 << channel);
			if (ctc_irqmask == 0) {
				int_clear(IRQ_CTC);
				if (trace & TRACE_IRQ)
					fprintf(stderr, "CTC %d irq reset.\n", channel);
				/* Is this all that is needed ?? */
			}
		}
	} else {
		if (trace & TRACE_CTC)
			fprintf(stderr, "CTC %d vector loaded with %02X\n", channel, val);
		c->vector = val;
	}
}

static uint8_t ctc_read(uint8_t channel)
{
	uint8_t val = ctc[channel].count >> 8;
	if (trace & TRACE_CTC)
		fprintf(stderr, "CTC %d reads %02x\n", channel, val);
	return val;
}

/*
 *	Minimal beginnings of 6522 VIA emulation
 */

struct via6522 {
	uint8_t irq;
	uint8_t acr;
	uint8_t ifr;
	uint8_t ier;
	uint8_t pcr;
	uint8_t sr;
	uint8_t ora;
	uint8_t orb;
	uint8_t ira;
	uint8_t irb;
	uint8_t ddra;
	uint8_t ddrb;
	uint16_t t1;
	uint16_t t1l;
	uint16_t t2;
	uint8_t t2l;
	/* Pin states rather than registers */
	uint8_t ca;
	uint8_t cb;
};

struct via6522 via;

static void via_recalc_irq(void)
{
	int irq = (via.ier & via.ifr) & 0x7F;
	if (irq)
		via.ifr |= 0x80;
	else
		via.ifr &= 0x7F;
	/* We interrupt if ier and ifr are set */
	/* Note: the pin is inverted but we model irq state not the pin! */
	if ((trace & TRACE_VIA) && irq != via.irq)
		fprintf(stderr, "[VIA IRQ now %02X.]\n", irq);
	via.irq = irq;

	if (via.irq)
		int_set(IRQ_VIA);
	else
		int_clear(IRQ_VIA);
}

static void via_handshake_a(void)
{
}

static void via_handshake_b(void)
{
}

static void via_recalc_outputs(void)
{
}

static void via_recalc_inputs(void)
{
}

static void via_recalc_all(void)
{
	via_recalc_outputs();
	via_recalc_inputs();
	via_recalc_irq();
}

/* Perform time related processing for the VIA */
void via_tick(int clocks)
{
	/* This isn't quite right but it's near enough for the moment */
	if (via.t1) {
		if (clocks >= via.t1) {
			if (trace & TRACE_VIA)
				fprintf(stderr,"[VIA T1 expire.].\n");
			via.ifr |= 0x40;
			via_recalc_irq();
			/* +1 or + 2 ?? */
			if (via.acr & 0x40)
				via.t1 = via.t1l + 1;
			else
				via.t1 = 0;
		}
		else
			via.t1 -= clocks;
	}

	if (via.t2 && !(via.acr & 0x20)) {
		if (clocks >= via.t2) {
			via.ifr |= 0x20;
			via_recalc_irq();
			via.t2 = 0;
			if (trace & TRACE_VIA)
				fprintf(stderr,"[VIA T2 expire.].\n");
		}
		via.t2 -= clocks;
	}
}

uint8_t via_read(uint8_t addr)
{
	uint8_t r;
	if (trace & TRACE_VIA)
		fprintf(stderr, "[VIA read %d: ", addr);
	switch(addr) {
		case 0:
			r = via.irb & ~via.ddrb;
			r |= via.orb & via.ddrb;
			via_handshake_b();
			break;
		case 1:
			r = via.ira & ~via.ddra;
			r |= via.ora & via.ddra;
			via_handshake_a();
			break;
		case 2:
			r = via.ddrb;
			break;
		case 3:
			r = via.ddra;
			break;
		case 4:
			via.ifr &= ~0x40;	/* T1 timeout */
			via_recalc_irq();
			r = via.t1;
			break;
		case 5:
			r = via.t1 >> 8;
			break;
		case 6:
			r = via.t1l;
			break;
		case 7:
			r = via.t1l >> 8;
			break;
		case 8:
			via.ifr &= ~0x20;	/* T2 timeout */
			via_recalc_irq();
			r = via.t2;
			break;
		case 9:
			r = via.t2 >> 8;
			break;
		case 10:
			r = via.sr;
			break;
		case 11:
			r = via.acr;
			break;
		case 12:
			r = via.pcr;
			break;
		case 13:
			r = via.ifr;
			break;
		case 14:
			r = via.ier;
			break;
		default:
		case 15:
			r =  via.ira;
			break;
	}
	if (trace & TRACE_VIA)
		fprintf(stderr, "%02X.]\n", r);
	return r;
}

void via_write(uint8_t addr, uint8_t val)
{
	if (trace & TRACE_VIA)
		fprintf(stderr, "[VIA write %d: %02X.]\n ", addr, val);
	switch(addr) {
		case 0:
			via.orb = val;
			via_recalc_outputs();
			via_handshake_b();
			break;
		case 1:
			via.ora = val;
			via_recalc_outputs();
			break;
		case 2:
			via.ddrb = val;
			via_recalc_all();
			break;
		case 3:
			via.ddra = val;
			via_recalc_all();
			break;
		case 4:
		case 6:
			via.t1l &= 0xFF00;
			via.t1l |= val;
			break;
		case 5:
			via.t1l &= 0xFF;
			via.t1l |= val << 8;
			via.t1 = via.t1l;
			via.ifr &= ~0x40;	/* T1 timeout */
			via_recalc_irq();
			if (trace & TRACE_VIA)
				fprintf(stderr, "[VIA T1 begin %04X.]\n", via.t1);
			break;
		case 7:
			via.t1l &= 0xFF;
			via.t1l |= val << 8;
			break;
		case 8:
			via.t2l = val;
			break;
		case 9:
			via.t2 = val << 8;
			via.t2 |= via.t2l;
			via.ifr &= ~0x20;	/* T2 timeout */
			via_recalc_irq();
			break;
		case 10:
			via.sr = val;
			break;
		case 11:
			via.acr = val;
			break;
		case 12:
			via.pcr = val;
			break;
		case 13:
			via.ifr &= ~val;
			if (via.ifr & 0x7F)
				via.ifr |= 0x80;
			via_recalc_irq();
			break;
		case 14:
			if (val & 0x80)
				via.ier |= val;
			else
				via.ier &= ~val;
			via.ier &= 0x7F;
			via_recalc_irq();
			break;
		case 15:
			via.ora = val;
			break;
	}
}



uint8_t mmio_read_6502(uint8_t addr)
{
	if (trace & TRACE_IO)
		fprintf(stderr, "read %02x\n", addr);
	if ((addr >= 0x80 && addr <= 0x87) && acia && acia_narrow)
		return acia_read(addr & 1);
	if ((addr >= 0x80 && addr <= 0xBF) && acia && !acia_narrow)
		return acia_read(addr & 1);
	if ((addr >= 0x80 && addr <= 0x83) && sio2)
		return sio2_read(addr & 3);
	if ((addr >= 0x10 && addr <= 0x17) && ide)
		return my_ide_read(addr & 7);
	if (addr >= 0x28 && addr <= 0x2C && wiznet)
		return nic_w5100_read(wiz, addr & 3);
	if (addr >= 0x60 && addr <= 0x6F)
		return via_read(addr & 0x0F);
	if (addr == 0xC0 && rtc)
		return rtc_read();
	if (addr >= 0xC0 && addr <= 0xCF && uart_16550a)
		return uart_read(&uart, addr & 0x0F);
	/* Scott Baker is 0x90-93, suggested defaults for the
	   Stephen Cousins boards at 0x88-0x8B. No doubt we'll get
	   an official CTC board at another address  */
	if (addr >= 0x88 && addr <= 0x8B && have_ctc)
		return ctc_read(addr & 3);

	if (trace & TRACE_UNK)
		fprintf(stderr, "Unknown read from port %04X\n", addr);
	return 0xFF;
}

void mmio_write_6502(uint8_t addr, uint8_t val)
{
	if (trace & TRACE_IO)
		fprintf(stderr, "write %02x <- %02x\n", addr, val);
	if ((addr >= 0x80 && addr <= 0x87) && acia && acia_narrow)
		acia_write(addr & 1, val);
	if ((addr >= 0x80 && addr <= 0xBF) && acia && !acia_narrow)
		acia_write(addr & 1, val);
	else if ((addr >= 0x80 && addr <= 0x83) && sio2)
		sio2_write(addr & 3, val);
	else if ((addr >= 0x10 && addr <= 0x17) && ide)
		my_ide_write(addr & 7, val);
	else if (addr >= 0x28 && addr <= 0x2C && wiznet)
		nic_w5100_write(wiz, addr & 3, val);
	else if (addr >= 0x60 && addr <= 0x6F)
		via_write(addr & 0x0F, val);
	/* FIXME: real bank512 alias at 0x70-77 for 78-7F */
	else if (addr >= 0x78 && addr <= 0x7B) {
		bankreg[addr & 3] = val & 0x3F;
		if (trace & TRACE_512)
			fprintf(stderr, "Bank %d set to %d\n", addr & 3, val);
	} else if (addr >= 0x7C && addr <= 0x7F) {
		if (trace & TRACE_512)
			fprintf(stderr, "Banking %sabled.\n", (val & 1) ? "en" : "dis");
		bankenable = val & 1;
	} else if (addr == 0xC0 && rtc)
		rtc_write(val);
	else if (addr >= 0x88 && addr <= 0x8B && have_ctc)
		ctc_write(addr & 3, val);
	else if (addr >= 0xC0 && addr <= 0xCF && uart_16550a)
		uart_write(&uart, addr & 0x0F, val);
	else if (addr == 0x00) {
		printf("trace set to %d\n", val);
		trace = val;
		if (trace & TRACE_CPU)
			log_6502 = 1;
		else
			log_6502 = 0;
	} else if (trace & TRACE_UNK)
		fprintf(stderr, "Unknown write to port %04X of %02X\n", addr, val);
}

/* FIXME: emulate paging off correctly, also be nice to emulate with less
   memory fitted */
uint8_t do_6502_read(uint16_t addr)
{
	uint16_t xaddr = addr ^ addrinvert;
	if (bankenable) {
		unsigned int bank = (xaddr & 0xC000) >> 14;
		if (trace & TRACE_MEM)
			fprintf(stderr, "R %04X[%02X] = %02X\n", addr, (unsigned int) bankreg[bank], (unsigned int) ramrom[(bankreg[bank] << 14) + (xaddr & 0x3FFF)]);
		xaddr &= 0x3FFF;
		return ramrom[(bankreg[bank] << 14) + xaddr];
	}
	/* When banking is off the entire 64K is occupied by repeats of ROM 0 */
	if (trace & TRACE_MEM)
		fprintf(stderr, "R %04X = %02X\n", addr, ramrom[xaddr]);
	return ramrom[xaddr & 0x3FFF];
}

uint8_t read6502(uint16_t addr)
{
	static uint8_t rstate = 0;
	uint8_t r;

	if (addr >> 8 == iopage)
		return mmio_read_6502(addr);

	r = do_6502_read(addr);

	if (fake_m1) {
		/* DD FD CB see the Z80 interrupt manual */
		if (r == 0xDD || r == 0xFD || r== 0xCB) {
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
	if (rstate == 1 && r == 0x4D)
		reti_event();
	rstate = 0;
	return r;
}

uint8_t read6502_debug(uint16_t addr)
{
	/* Avoid side effects for debug */
	if (addr >> 8 == iopage)
		return 0xFF;

	return do_6502_read(addr);
}


void write6502(uint16_t addr, uint8_t val)
{
	uint16_t xaddr = addr ^ addrinvert;

	if (addr >> 8 == iopage) {
		mmio_write_6502(addr, val);
		return;
	}
	if (bankenable) {
		unsigned int bank = (xaddr & 0xC000) >> 14;
		if (trace & TRACE_MEM)
			fprintf(stderr, "W %04X[%02X] = %02X\n", (unsigned int) addr, (unsigned int) bankreg[bank], (unsigned int) val);
		if (bankreg[bank] >= 32) {
			xaddr &= 0x3FFF;
			ramrom[(bankreg[bank] << 14) + xaddr] = val;
		}
		/* ROM writes go nowhere */
		else if (trace & TRACE_MEM)
			fprintf(stderr, "[Discarded: ROM]\n");
	} else {
		if (trace & TRACE_MEM)
			fprintf(stderr, "W: %04X = %02X\n", addr, val);
		else if (trace & TRACE_MEM)
			fprintf(stderr, "[Discarded: ROM]\n");
	}
}

static void poll_irq_event(void)
{
	/* The SIO has IE0/IE1 working internally but not globally */
	!sio2_check_im2(sio) && !sio2_check_im2(sio + 1);
	/* The CTC has nothing wired to IE0/IE1 at all */
	ctc_check_im2();
}

static void reti_event(void)
{
	/* If IM2 is not wired then all the things respond at the same
	   time. I think they can also fight over the vector but ignore
	   that */
	if (sio2) {
		sio2_reti(sio);
		sio2_reti(sio + 1);
	}
	if (have_ctc) {
		ctc_reti(0);
		ctc_reti(1);
		ctc_reti(2);
		ctc_reti(3);
	}
	/* The ACIA and 16550A do not care about reti */
	live_irq &= ~(1 << (IRQ_ACIA | IRQ_16550A));
	poll_irq_event();
}

static void irqnotify(void)
{
	if(live_irq)
		irq6502();
}

static struct termios saved_term, term;

static void cleanup(int sig)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
	done = 1;
}

static void exit_cleanup(void)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
}

static void usage(void)
{
	fprintf(stderr, "rc2014: [-1] [-A] [-a] [-c] [-f] [-R] [-r rompath] [-s] [-w] [-d debug]\n");
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	static struct timespec tc;
	int opt;
	int fd;
	char *rompath = "rc2014-6502.rom";
	char *idepath;

	while ((opt = getopt(argc, argv, "1Aacd:fi:r:sRw")) != -1) {
		switch (opt) {
		case '1':
			uart_16550a = 1;
			acia = 0;
			sio2 = 0;
			break;
		case 'a':
			acia = 1;
			acia_input = 1;
			acia_narrow = 0;
			sio2 = 0;
			uart_16550a = 0;
			break;
		case 'A':
			acia = 1;
			acia_narrow = 1;
			acia_input = 1;
			sio2 = 0;
			uart_16550a = 0;
			break;
		case 'r':
			rompath = optarg;
			break;
		case 's':
			sio2 = 1;
			sio2_input = 1;
			acia = 0;
			uart_16550a = 0;
			break;
		case 'i':
			ide = 1;
			idepath = optarg;
			break;
		case 'c':
			have_ctc = 1;
			break;
		case 'd':
			trace = atoi(optarg);
			break;
		case 'f':
			fast = 1;
			break;
		case 'R':
			rtc = 1;
			break;
		case 'w':
			wiznet = 1;
			break;
		default:
			usage();
		}
	}
	if (optind < argc)
		usage();

	if (acia == 0 && sio2 == 0 && uart_16550a == 0) {
		fprintf(stderr, "rc2014: no UART selected, defaulting to 16550A\n");
		uart_16550a = 1;
	}
	if (rtc && uart_16550a) {
		fprintf(stderr, "rc2014: RTC and 16550A clash at 0xC0.\n");
		exit(1);
	}

	fd = open(rompath, O_RDONLY);
	if (fd == -1) {
		perror(rompath);
		exit(EXIT_FAILURE);
	}
	if (read(fd, ramrom, 524288) != 524288) {
		fprintf(stderr, "rc2014: banked rom image should be 512K.\n");
		exit(EXIT_FAILURE);
	}
	close(fd);

	if (ide) {
		ide0 = ide_allocate("cf");
		if (ide0) {
			int ide_fd = open(idepath, O_RDWR);
			if (ide_fd == -1) {
				perror(idepath);
				ide = 0;
			}
			if (ide_attach(ide0, 0, ide_fd) == 0) {
				ide = 1;
				ide_reset_begin(ide0);
			}
		} else
			ide = 0;
	}

	if (sio2)
		sio_reset();
	if (have_ctc)
		ctc_init();
	if (uart_16550a)
		uart_init(&uart);

	if (wiznet) {
		wiz = nic_w5100_alloc();
		nic_w5100_reset(wiz);
	}

	/* 5ms - it's a balance between nice behaviour and simulation
	   smoothness */
	tc.tv_sec = 0;
	tc.tv_nsec = 5000000L;

	if (tcgetattr(0, &term) == 0) {
		saved_term = term;
		atexit(exit_cleanup);
		signal(SIGINT, cleanup);
		signal(SIGQUIT, cleanup);
		signal(SIGPIPE, cleanup);
		term.c_lflag &= ~(ICANON | ECHO);
		term.c_cc[VMIN] = 0;
		term.c_cc[VTIME] = 1;
		term.c_cc[VINTR] = 0;
		term.c_cc[VSUSP] = 0;
		term.c_cc[VSTOP] = 0;
		tcsetattr(0, TCSADRAIN, &term);
	}

	if (trace & TRACE_CPU)
		log_6502 = 1;

	init6502();
	reset6502();
	hookexternal(irqnotify);

	/* This is the wrong way to do it but it's easier for the moment. We
	   should track how much real time has occurred and try to keep cycle
	   matched with that. The scheme here works fine except when the host
	   is loaded though */

	/* We run 4000000 t-states per second */
	/* We run 200 cycles per I/O check, do that 100 times then poll the
	   slow stuff and nap for 5ms. */
	while (!done) {
		int i;
		/* 36400 T states for base RC2014 - varies for others */
		for (i = 0; i < 100; i++) {
			/* FIXME: should check return and keep adjusting */
			exec6502(tstate_steps);
			if (acia)
				acia_timer();
			if (sio2)
				sio2_timer();
			if (have_ctc)
				ctc_tick(tstate_steps);
			if (uart_16550a)
				uart_event(&uart);
			via_tick(tstate_steps);
		}
		if (wiznet)
			w5100_process(wiz);
		/* Do 5ms of I/O and delays */
		if (!fast)
			nanosleep(&tc, NULL);
		poll_irq_event();
	}
	exit(0);
}
