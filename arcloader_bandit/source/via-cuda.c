// SPDX-License-Identifier: GPL-2.0
/*
 * Device driver for the Cuda and Egret system controllers found on PowerMacs
 * and 68k Macs.
 *
 * The Cuda or Egret is a 6805 microcontroller interfaced to the 6522 VIA.
 * This MCU controls system power, Parameter RAM, Real Time Clock and the
 * Apple Desktop Bus (ADB) that connects to the keyboard and mouse.
 *
 * Copyright (C) 1996 Paul Mackerras.
 */

#include "types.h"
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#ifndef ENXIO
#define ENXIO 6
#define EINVAL 22
#endif

volatile unsigned char *via = NULL;

/* VIA registers - spaced 0x200 bytes apart */
#define RS 0x200      /* skip between registers */
#define B 0           /* B-side data */
#define A RS          /* A-side data */
#define DIRB (2 * RS) /* B-side direction (1=output) */
#define DIRA (3 * RS) /* A-side direction (1=output) */
#define T1CL (4 * RS) /* Timer 1 ctr/latch (low 8 bits) */
#define T1CH (5 * RS) /* Timer 1 counter (high 8 bits) */
#define T1LL (6 * RS) /* Timer 1 latch (low 8 bits) */
#define T1LH (7 * RS) /* Timer 1 latch (high 8 bits) */
#define T2CL (8 * RS) /* Timer 2 ctr/latch (low 8 bits) */
#define T2CH (9 * RS) /* Timer 2 counter (high 8 bits) */
#define SR (10 * RS)  /* Shift register */
#define ACR (11 * RS) /* Auxiliary control register */
#define PCR (12 * RS) /* Peripheral control register */
#define IFR (13 * RS) /* Interrupt flag register */
#define IER (14 * RS) /* Interrupt enable register */
#define ANH (15 * RS) /* A-side data, no handshake */

/*
 * When the Cuda design replaced the Egret, some signal names and
 * logic sense changed. They all serve the same purposes, however.
 *
 *   VIA pin       |  Egret pin
 * ----------------+------------------------------------------
 *   PB3 (input)   |  Transceiver session   (active low)
 *   PB4 (output)  |  VIA full              (active high)
 *   PB5 (output)  |  System session        (active high)
 *
 *   VIA pin       |  Cuda pin
 * ----------------+------------------------------------------
 *   PB3 (input)   |  Transfer request      (active low)
 *   PB4 (output)  |  Byte acknowledge      (active low)
 *   PB5 (output)  |  Transfer in progress  (active low)
 */

/* Bits in Port B data register */
#define TREQ 0x08 /* Transfer request */
#define TACK 0x10 /* Transfer acknowledge */
#define TIP 0x20  /* Transfer in progress */

/* Bits in ACR */
#define SR_CTRL 0x1c /* Shift register control bits */
#define SR_EXT 0x0c  /* Shift on external clock */
#define SR_OUT 0x10  /* Shift out if 1 */

/* Bits in IFR and IER */
#define IER_SET 0x80 /* set bits in IER */
#define IER_CLR 0    /* clear bits in IER */
#define SR_INT 0x04  /* Shift register full/empty */

/* Duration of byte acknowledgement pulse (us) */
#define EGRET_TACK_ASSERTED_DELAY 300
#define EGRET_TACK_NEGATED_DELAY 400

/* Interval from interrupt to start of session (us) */
#define EGRET_SESSION_DELAY 450

#define mcu_is_egret false

/*
 * Low-level I/O routines.
 *
 * Copied from <file:arch/powerpc/include/asm/io.h> (which has no copyright)
 */
static inline int in_8(const volatile unsigned char *addr) {
    int ret;

    __asm__ __volatile__("lbz%U1%X1 %0,%1; twi 0,%0,0; isync" : "=r"(ret) : "m"(*addr));
    return ret;
}

static inline void out_8(volatile unsigned char *addr, int val) {
    __asm__ __volatile__("stb%U0%X0 %1,%0; sync" : "=m"(*addr) : "r"(val));
}

static inline unsigned in_le16(const volatile __u16 *addr) {
    unsigned ret;

    __asm__ __volatile__("lhbrx %0,0,%1; twi 0,%0,0; isync" : "=r"(ret) : "r"(addr), "m"(*addr));

    return ret;
}

static inline unsigned in_be16(const volatile __u16 *addr) {
    unsigned ret;

    __asm__ __volatile__("lhz%U1%X1 %0,%1; twi 0,%0,0; isync" : "=r"(ret) : "m"(*addr));
    return ret;
}

static inline void out_le16(volatile __u16 *addr, int val) {
    __asm__ __volatile__("sthbrx %1,0,%2; sync" : "=m"(*addr) : "r"(val), "r"(addr));
}

static inline void out_be16(volatile __u16 *addr, int val) {
    __asm__ __volatile__("sth%U0%X0 %1,%0; sync" : "=m"(*addr) : "r"(val));
}

static inline unsigned in_le32(const volatile unsigned *addr) {
    unsigned ret;

    __asm__ __volatile__("lwbrx %0,0,%1; twi 0,%0,0; isync" : "=r"(ret) : "r"(addr), "m"(*addr));
    return ret;
}

static inline unsigned in_be32(const volatile unsigned *addr) {
    unsigned ret;

    __asm__ __volatile__("lwz%U1%X1 %0,%1; twi 0,%0,0; isync" : "=r"(ret) : "m"(*addr));
    return ret;
}

static inline void out_le32(volatile unsigned *addr, int val) {
    __asm__ __volatile__("stwbrx %1,0,%2; sync" : "=m"(*addr) : "r"(val), "r"(addr));
}

static inline void out_be32(volatile unsigned *addr, int val) {
    __asm__ __volatile__("stw%U0%X0 %1,%0; sync" : "=m"(*addr) : "r"(val));
}

static inline void sync(void) {
    asm volatile("sync" : : : "memory");
}

static inline void eieio(void) {
    asm volatile("eieio" : : : "memory");
}

static inline void barrier(void) {
    asm volatile("" : : : "memory");
}

static inline bool TREQ_asserted(__u8 portb) {
    return !(portb & TREQ);
}

static inline void assert_TIP(void) {
    if (mcu_is_egret) {
        udelay(EGRET_SESSION_DELAY);
        out_8(&via[B], in_8(&via[B]) | TIP);
    } else
        out_8(&via[B], in_8(&via[B]) & ~TIP);
}

static inline void assert_TIP_and_TACK(void) {
    if (mcu_is_egret) {
        udelay(EGRET_SESSION_DELAY);
        out_8(&via[B], in_8(&via[B]) | TIP | TACK);
    } else
        out_8(&via[B], in_8(&via[B]) & ~(TIP | TACK));
}

static inline void assert_TACK(void) {
    if (mcu_is_egret) {
        udelay(EGRET_TACK_NEGATED_DELAY);
        out_8(&via[B], in_8(&via[B]) | TACK);
    } else
        out_8(&via[B], in_8(&via[B]) & ~TACK);
}

static inline void toggle_TACK(void) {
    out_8(&via[B], in_8(&via[B]) ^ TACK);
}

static inline void negate_TACK(void) {
    if (mcu_is_egret) {
        udelay(EGRET_TACK_ASSERTED_DELAY);
        out_8(&via[B], in_8(&via[B]) & ~TACK);
    } else
        out_8(&via[B], in_8(&via[B]) | TACK);
}

static inline void negate_TIP_and_TACK(void) {
    if (mcu_is_egret) {
        udelay(EGRET_TACK_ASSERTED_DELAY);
        out_8(&via[B], in_8(&via[B]) & ~(TIP | TACK));
    } else
        out_8(&via[B], in_8(&via[B]) | TIP | TACK);
}

static enum cuda_state { idle, sent_first_byte, sending, reading, read_done, awaiting_reply } cuda_state;

static struct adb_request *current_req;
static struct adb_request *last_req;
static unsigned char cuda_rbuf[16];
static unsigned char *reply_ptr;
static int reading_reply;
static int data_index;

static void cuda_start(void);
static int cuda_interrupt(int irq, void *arg);
static void cuda_input(unsigned char *buf, int nb);
void cuda_poll(void);
static int cuda_write(struct adb_request *req);

int cuda_request(struct adb_request *req, void (*done)(struct adb_request *), int nbytes, ...);

#define WAIT_FOR(cond, what) \
    do { \
        int x; \
        for (x = 1000; !(cond); --x) { \
            if (x == 0) { \
                pr_err("Timeout waiting for " what "\n"); \
                return -ENXIO; \
            } \
            udelay(100); \
        } \
    } while (0)

/* Construct and send a cuda request */
int cuda_request(struct adb_request *req, void (*done)(struct adb_request *), int nbytes, ...) {
    va_list list;
    int i;

    if (via == NULL) {
        req->complete = 1;
        return -ENXIO;
    }

    req->nbytes = nbytes;
    req->done = done;
    va_start(list, nbytes);
    for (i = 0; i < nbytes; ++i) req->data[i] = va_arg(list, int);
    va_end(list);
    req->reply_expected = 1;
    return cuda_write(req);
}

static int cuda_write(struct adb_request *req) {
    unsigned long flags;

    if (req->nbytes < 2 || req->data[0] > CUDA_PACKET) {
        req->complete = 1;
        return -EINVAL;
    }
    req->next = NULL;
    req->sent = 0;
    req->complete = 0;
    req->reply_len = 0;

    // spin_lock_irqsave(&cuda_lock, flags);
    if (current_req) {
        last_req->next = req;
        last_req = req;
    } else {
        current_req = req;
        last_req = req;
        if (cuda_state == idle) cuda_start();
    }
    // spin_unlock_irqrestore(&cuda_lock, flags);

    return 0;
}

static void cuda_start(void) {
    /* assert cuda_state == idle */
    if (current_req == NULL) return;
    data_index = 0;
    if (TREQ_asserted(in_8(&via[B]))) return; /* a byte is coming in from the CUDA */

    /* set the shift register to shift out and send a byte */
    out_8(&via[ACR], in_8(&via[ACR]) | SR_OUT);
    out_8(&via[SR], current_req->data[data_index++]);
    if (mcu_is_egret) assert_TIP_and_TACK();
    else
        assert_TIP();
    cuda_state = sent_first_byte;
}

void cuda_poll(void) {
    cuda_interrupt(0, NULL);
}

#define ARRAY_FULL(a, p) ((p) - (a) == (sizeof(a) / sizeof(*a)))

static int cuda_interrupt(int irq, void *arg) {
    unsigned long flags;
    u8 status;
    struct adb_request *req = NULL;
    unsigned char ibuf[16];
    int ibuf_len = 0;
    int complete = 0;
    bool full;

    // spin_lock_irqsave(&cuda_lock, flags);

    /* On powermacs, this handler is registered for the VIA IRQ. But they use
     * just the shift register IRQ -- other VIA interrupt sources are disabled.
     * On m68k macs, the VIA IRQ sources are dispatched individually. Unless
     * we are polling, the shift register IRQ flag has already been cleared.
     */

#ifdef CONFIG_MAC
    if (!arg)
#endif
    {
        if ((in_8(&via[IFR]) & SR_INT) == 0) {
            // spin_unlock_irqrestore(&cuda_lock, flags);
            return 0;
        } else {
            out_8(&via[IFR], SR_INT);
        }
    }

    status = in_8(&via[B]) & (TIP | TACK | TREQ);

    switch (cuda_state) {
    case idle:
        /* System controller has unsolicited data for us */
        (void)in_8(&via[SR]);
    idle_state:
        assert_TIP();
        cuda_state = reading;
        reply_ptr = cuda_rbuf;
        reading_reply = 0;
        break;

    case awaiting_reply:
        /* System controller has reply data for us */
        (void)in_8(&via[SR]);
        assert_TIP();
        cuda_state = reading;
        reply_ptr = current_req->reply;
        reading_reply = 1;
        break;

    case sent_first_byte:
        if (TREQ_asserted(status)) {
            /* collision */
            out_8(&via[ACR], in_8(&via[ACR]) & ~SR_OUT);
            (void)in_8(&via[SR]);
            negate_TIP_and_TACK();
            cuda_state = idle;
            /* Egret does not raise an "aborted" interrupt */
            if (mcu_is_egret) goto idle_state;
        } else {
            out_8(&via[SR], current_req->data[data_index++]);
            toggle_TACK();
            if (mcu_is_egret) assert_TACK();
            cuda_state = sending;
        }
        break;

    case sending:
        req = current_req;
        if (data_index >= req->nbytes) {
            out_8(&via[ACR], in_8(&via[ACR]) & ~SR_OUT);
            (void)in_8(&via[SR]);
            negate_TIP_and_TACK();
            req->sent = 1;
            if (req->reply_expected) {
                cuda_state = awaiting_reply;
            } else {
                current_req = req->next;
                complete = 1;
                /* not sure about this */
                cuda_state = idle;
                cuda_start();
            }
        } else {
            out_8(&via[SR], req->data[data_index++]);
            toggle_TACK();
            if (mcu_is_egret) assert_TACK();
        }
        break;

    case reading:
        full = reading_reply ? ARRAY_FULL(current_req->reply, reply_ptr) : ARRAY_FULL(cuda_rbuf, reply_ptr);
        if (full) (void)in_8(&via[SR]);
        else
            *reply_ptr++ = in_8(&via[SR]);
        if (!TREQ_asserted(status) || full) {
            if (mcu_is_egret) assert_TACK();
            /* that's all folks */
            negate_TIP_and_TACK();
            cuda_state = read_done;
            /* Egret does not raise a "read done" interrupt */
            if (mcu_is_egret) goto read_done_state;
        } else {
            toggle_TACK();
            if (mcu_is_egret) negate_TACK();
        }
        break;

    case read_done:
        (void)in_8(&via[SR]);
    read_done_state:
        if (reading_reply) {
            req = current_req;
            req->reply_len = reply_ptr - req->reply;
            if (req->data[0] == ADB_PACKET) {
                /* Have to adjust the reply from ADB commands */
                if (req->reply_len <= 2 || (req->reply[1] & 2) != 0) {
                    /* the 0x2 bit indicates no response */
                    req->reply_len = 0;
                } else {
                    /* leave just the command and result bytes in the reply */
                    req->reply_len -= 2;
                    memmove(req->reply, req->reply + 2, req->reply_len);
                }
            }
            current_req = req->next;
            complete = 1;
            reading_reply = 0;
        } else {
            /* This is tricky. We must break the spinlock to call
             * cuda_input. However, doing so means we might get
             * re-entered from another CPU getting an interrupt
             * or calling cuda_poll(). I ended up using the stack
             * (it's only for 16 bytes) and moving the actual
             * call to cuda_input to outside of the lock.
             */
            ibuf_len = reply_ptr - cuda_rbuf;
            memcpy(ibuf, cuda_rbuf, ibuf_len);
        }
        reply_ptr = cuda_rbuf;
        cuda_state = idle;
        cuda_start();
        if (cuda_state == idle && TREQ_asserted(in_8(&via[B]))) {
            assert_TIP();
            cuda_state = reading;
        }
        break;

    default: //pr_err("cuda_interrupt: unknown cuda_state %d?\n", cuda_state);
    }
    // spin_unlock_irqrestore(&cuda_lock, flags);
    if (complete && req) {
        void (*done)(struct adb_request *) = req->done;
        sync();
        req->complete = 1;
        /* Here, we assume that if the request has a done member, the
         * struct request will survive to setting req->complete to 1
         */
        if (done) (*done)(req);
    }
    if (ibuf_len) cuda_input(ibuf, ibuf_len);
    return 1;
}

static void cuda_input(unsigned char *buf, int nb) {
    switch (buf[0]) {
    case ADB_PACKET:
#ifdef CONFIG_XMON
        if (nb == 5 && buf[2] == 0x2c) {
            extern int xmon_wants_key, xmon_adb_keycode;
            if (xmon_wants_key) {
                xmon_adb_keycode = buf[3];
                return;
            }
        }
#endif /* CONFIG_XMON */
#ifdef CONFIG_ADB
        adb_input(buf + 2, nb - 2, buf[1] & 0x40);
#endif /* CONFIG_ADB */
        break;

    case TIMER_PACKET:
        /* Egret sends these periodically. Might be useful as a 'heartbeat'
         * to trigger a recovery for the VIA shift register errata.
         */
        break;

    default:
    }
}