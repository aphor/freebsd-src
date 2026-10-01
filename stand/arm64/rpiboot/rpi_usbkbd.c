/*-
 * rpi_usbkbd.c -- a USB keyboard for the loader: RP1's xHCI controllers,
 * polled, and HID boot-protocol keyboards on their root ports.
 *
 * Deliberately minimal, in the style of U-Boot's and EDK2's boot-time USB
 * keyboards, rather than a port of the kernel's USB stack:
 *
 *  - Both RP1 USB controllers (Synopsys dwc3 in host mode, xHCI inside) at
 *    RP1 BAR1 + 0x200000 and + 0x300000; see rpi_pcie.c for how RP1 gets
 *    there.  The dwc3 set-up is the kernel's snps_dwc3 attach path (core
 *    and PHY soft reset, host mode, the quirks the device tree names).
 *  - xHCI by polling: no interrupts, one command ring, one event ring, one
 *    interrupter.  Devices directly on a root port only; a hub is reported
 *    and skipped.
 *  - A device is used if one of its interfaces is a HID boot keyboard
 *    (class 3, subclass 1, protocol 1).  It is put in the boot protocol, so
 *    reports are the fixed 8 bytes (modifiers, reserved, six key codes) and
 *    no report descriptor needs parsing.  US layout.
 *
 * Memory.  The controllers DMA through PCIe, which the device tree does not
 * mark dma-coherent, so everything they read or write -- rings, contexts,
 * descriptors, reports -- is in one region mapped Normal non-cacheable
 * (rpi_mmu_set_nc()), between the heap and the kernel's staging area.  Plain
 * stores to it are ordered before a doorbell (a Device-memory store) by a
 * DSB.  Inbound PCIe is 1:1 (rpi_pcie.c), so a DMA address is the physical
 * address.
 *
 * Before the kernel, rpi_usbkbd_shutdown() halts and resets both
 * controllers, so that nothing writes to that region once it is the
 * kernel's memory.
 *
 * Register and data-structure layouts: the xHCI specification, rev. 1.2
 * (sections cited as xHCI 5.x etc.), as FreeBSD's sys/dev/usb/controller/
 * xhci.h names them; dwc3 registers from sys/dev/usb/controller/dwc3/dwc3.h.
 */

#include <stand.h>
#include <sys/param.h>

#include "bootstrap.h"
#include "librpiboot.h"

/* ---- DMA memory ------------------------------------------------------- */

/*
 * 2 MiB at 192 MiB: inside /memory's first bank, above the heap
 * (0x8000000 + 48 MiB) and below the staging area (0x10000000), and 2 MiB
 * aligned because rpi_mmu_set_nc() works in 2 MiB blocks.
 */
#define	DMA_BASE	0x0c000000UL
#define	DMA_SIZE	0x00200000UL

static uintptr_t dma_next;

static void *
dma_alloc(size_t size, size_t align)
{
	uintptr_t p;

	p = roundup2(dma_next, align);
	if (p + size > DMA_BASE + DMA_SIZE)
		return (NULL);
	dma_next = p + size;
	memset((void *)p, 0, size);
	return ((void *)p);
}

#define	DMB()	__asm __volatile("dsb sy" ::: "memory")

static uint64_t
now_us(void)
{
	uint64_t cnt, frq;

	__asm __volatile("isb; mrs %0, cntpct_el0" : "=r" (cnt));
	__asm __volatile("mrs %0, cntfrq_el0" : "=r" (frq));
	return (frq != 0 ? cnt / (frq / 1000000) : 0);
}

/* ---- dwc3 -------------------------------------------------------------- */

#define	DWC3_GCTL		0xc110
#define	  GCTL_PRTCAPDIR_MASK	(0x3U << 12)
#define	  GCTL_PRTCAPDIR_HOST	(0x1U << 12)
#define	  GCTL_CORESOFTRESET	(1U << 11)
#define	DWC3_GUCTL1		0xc11c
#define	DWC3_GSNPSID		0xc120
#define	DWC3_GUCTL		0xc12c
#define	  GUCTL_HOST_AUTO_RETRY	(1U << 14)
#define	DWC3_GHWPARAMS0		0xc140
#define	  GHWPARAMS0_MODE_MASK	0x3
#define	  GHWPARAMS0_MODE_DRD	0x2
#define	DWC3_GUSB2PHYCFG0	0xc200
#define	  GUSB2_PHYSOFTRST	(1U << 31)
#define	  GUSB2_U2_FREECLK_EXISTS (1U << 30)
#define	  GUSB2_ENBLSLPM	(1U << 8)
#define	  GUSB2_SUSPENDUSB20	(1U << 6)
#define	DWC3_GUSB3PIPECTL0	0xc2c0
#define	  GUSB3_PHYSOFTRST	(1U << 31)
#define	  GUSB3_DISRXDETINP3	(1U << 28)
#define	  GUSB3_SUSPENDUSB3	(1U << 17)

/* ---- xHCI ------------------------------------------------------------- */

/* Capability registers (xHCI 5.3). */
#define	XHCI_CAPLENGTH		0x00	/* 8 bits */
#define	XHCI_HCSPARAMS1		0x04
#define	XHCI_HCSPARAMS2		0x08
#define	XHCI_HCCPARAMS1		0x10
#define	  HCC_CSZ		(1U << 2)
#define	XHCI_DBOFF		0x14
#define	XHCI_RTSOFF		0x18

/* Operational registers (xHCI 5.4), from CAPLENGTH. */
#define	XHCI_USBCMD		0x00
#define	  CMD_RS		(1U << 0)
#define	  CMD_HCRST		(1U << 1)
#define	XHCI_USBSTS		0x04
#define	  STS_HCH		(1U << 0)
#define	  STS_CNR		(1U << 11)
#define	XHCI_PAGESIZE		0x08
#define	XHCI_CRCR		0x18
#define	XHCI_DCBAAP		0x30
#define	XHCI_CONFIG		0x38
#define	XHCI_PORTSC(n)		(0x400 + 0x10 * ((n) - 1))
#define	  PS_CCS		(1U << 0)
#define	  PS_PED		(1U << 1)
#define	  PS_PR			(1U << 4)
#define	  PS_PP			(1U << 9)
#define	  PS_SPEED(v)		(((v) >> 10) & 0xf)
#define	  PS_CHANGE		(0x7fU << 17)	/* CSC..CEC, write 1 to clear */
#define	  PS_PRC		(1U << 21)
#define	  PS_KEEP		(PS_PP | (0x3U << 14) | (0x7U << 25))

/* Runtime registers (xHCI 5.5), interrupter 0. */
#define	XHCI_IMAN		0x20
#define	XHCI_ERSTSZ		0x28
#define	XHCI_ERSTBA		0x30
#define	XHCI_ERDP		0x38
#define	  ERDP_EHB		(1U << 3)

/* Port speeds (PORTSC, default protocol speed IDs). */
#define	SPEED_FS		1
#define	SPEED_LS		2
#define	SPEED_HS		3
#define	SPEED_SS		4

/* TRBs (xHCI 6.4). */
struct trb {
	uint64_t	param;
	uint32_t	status;
	uint32_t	control;
};
#define	TRB_CYCLE		(1U << 0)
#define	TRB_TC			(1U << 1)	/* link: toggle cycle */
#define	TRB_ISP			(1U << 2)
#define	TRB_IOC			(1U << 5)
#define	TRB_IDT			(1U << 6)
#define	TRB_TYPE(t)		((uint32_t)(t) << 10)
#define	TRB_GET_TYPE(c)		(((c) >> 10) & 0x3f)
#define	TRB_DIR_IN		(1U << 16)
#define	TRB_TRT_IN		(3U << 16)
#define	TRB_TRT_OUT		(2U << 16)
#define	TRB_SLOT(s)		((uint32_t)(s) << 24)
#define	TRB_EP(e)		((uint32_t)(e) << 16)
#define	TRB_GET_SLOT(c)		(((c) >> 24) & 0xff)
#define	TRB_GET_EP(c)		(((c) >> 16) & 0x1f)
#define	TRB_GET_CODE(s)		(((s) >> 24) & 0xff)

#define	T_NORMAL		1
#define	T_SETUP			2
#define	T_DATA			3
#define	T_STATUS		4
#define	T_LINK			6
#define	T_ENABLE_SLOT		9
#define	T_DISABLE_SLOT		10
#define	T_ADDRESS_DEVICE	11
#define	T_CONFIGURE_EP		12
#define	T_EVALUATE_CTX		13
#define	T_RESET_EP		14
#define	T_SET_TR_DEQ		16
#define	T_EV_TRANSFER		32
#define	T_EV_CMD_COMPLETE	33
#define	T_EV_PORT_CHANGE	34

#define	CC_SUCCESS		1
#define	CC_STALL		6
#define	CC_SHORT_PACKET		13

#define	RING_TRBS		256	/* one 4 KiB page; the last is a link */

struct ring {
	struct trb	*trbs;
	u_int		 enq;
	uint32_t	 cycle;
};

/* Contexts (xHCI 6.2): slot, endpoints, input control; 32 or 64 bytes. */
#define	DCI_EP0			1

/* ---- state ------------------------------------------------------------ */

#define	NCTRL			2
#define	MAXKBD			4
#define	MAXPORTS		8

struct kbd;

struct xhc {
	const char	*name;
	uintptr_t	 base;		/* capability registers */
	uintptr_t	 op, rt, db;
	u_int		 ctxsz;		/* 32 or 64 */
	u_int		 nports, nslots;
	uint64_t	*dcbaa;
	struct ring	 cmd;
	struct trb	*evt;		/* event ring segment */
	u_int		 evt_deq;
	uint32_t	 evt_cycle;
	bool		 up;
	char		 portinfo[MAXPORTS][48];
};

struct kbd {
	struct xhc	*xhc;
	u_int		 port, speed, slot;
	u_int		 dci;		/* interrupt IN endpoint */
	u_int		 mps;
	void		*out_ctx;
	void		*in_ctx;
	struct ring	 ep0;
	struct ring	 intr;
	uint8_t		*report;	/* DMA */
	uint8_t		 last[8];
	bool		 pending;	/* a Normal TRB is queued */
	uint8_t		 rpt_key;	/* typematic */
	uint64_t	 rpt_at;
};

static struct xhc xhcs[NCTRL] = {
	{ .name = "usb@200000" },
	{ .name = "usb@300000" },
};
static struct kbd kbds[MAXKBD];
static u_int nkbd;
static bool usb_tried;
static const char *usb_why = "not attempted";

/* Input from the keyboards, for the console. */
#define	KQ_SIZE			64
static uint8_t kq[KQ_SIZE];
static u_int kq_head, kq_tail;

/* Event handling hands transfer completions here. */
static struct trb last_cmd_event;
static bool cmd_done;
static struct trb last_ep0_event;
static bool ep0_done;

/* ---- register access -------------------------------------------------- */

static inline uint32_t
rd32(uintptr_t a)
{
	return (*(volatile uint32_t *)a);
}

static inline void
wr32(uintptr_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
}

static inline void
wr64(uintptr_t a, uint64_t v)
{
	/* Low word first; xHCI 5.1 permits two 32-bit writes. */
	wr32(a, (uint32_t)v);
	wr32(a + 4, (uint32_t)(v >> 32));
}

static bool
wait_bits(uintptr_t a, uint32_t mask, uint32_t want, u_int ms)
{
	uint64_t end = now_us() + (uint64_t)ms * 1000;

	while ((rd32(a) & mask) != want)
		if (now_us() > end)
			return (false);
	return (true);
}

/* ---- rings ------------------------------------------------------------ */

static int
ring_init(struct ring *r)
{
	struct trb *link;

	if ((r->trbs = dma_alloc(RING_TRBS * sizeof(struct trb), 4096)) ==
	    NULL)
		return (ENOMEM);
	r->enq = 0;
	r->cycle = 1;
	link = &r->trbs[RING_TRBS - 1];
	link->param = (uint64_t)(uintptr_t)r->trbs;
	link->control = TRB_TYPE(T_LINK) | TRB_TC;	/* cycle bit 0: not yet */
	return (0);
}

/* Queue one TRB; the cycle bit goes last, after a barrier. */
static void
ring_put(struct ring *r, uint64_t param, uint32_t status, uint32_t control)
{
	struct trb *t = &r->trbs[r->enq];

	t->param = param;
	t->status = status;
	DMB();
	t->control = (control & ~TRB_CYCLE) | r->cycle;
	if (++r->enq == RING_TRBS - 1) {
		struct trb *link = &r->trbs[RING_TRBS - 1];

		DMB();
		link->control = (link->control & ~TRB_CYCLE) | r->cycle;
		r->enq = 0;
		r->cycle ^= 1;
	}
}

static void
doorbell(struct xhc *x, u_int slot, u_int target)
{
	DMB();
	wr32(x->db + 4 * slot, target);
}

/* ---- events ----------------------------------------------------------- */

static void kbd_report(struct kbd *k, uint32_t code, uint32_t len);

/* Drain the event ring, dispatching what arrived.  Returns events seen. */
static int
evt_poll(struct xhc *x)
{
	struct trb *e;
	uint32_t c;
	u_int i, n = 0;

	for (;;) {
		e = &x->evt[x->evt_deq];
		c = e->control;
		if ((c & TRB_CYCLE) != x->evt_cycle)
			break;
		DMB();
		switch (TRB_GET_TYPE(c)) {
		case T_EV_CMD_COMPLETE:
			last_cmd_event = *e;
			cmd_done = true;
			break;
		case T_EV_TRANSFER:
			if (TRB_GET_EP(c) == DCI_EP0) {
				last_ep0_event = *e;
				ep0_done = true;
				break;
			}
			for (i = 0; i < nkbd; i++)
				if (kbds[i].xhc == x &&
				    kbds[i].slot == TRB_GET_SLOT(c) &&
				    kbds[i].dci == TRB_GET_EP(c))
					kbd_report(&kbds[i],
					    TRB_GET_CODE(e->status),
					    e->status & 0xffffff);
			break;
		default:
			/* Port status changes and the like: not used. */
			break;
		}
		if (++x->evt_deq == RING_TRBS) {
			x->evt_deq = 0;
			x->evt_cycle ^= 1;
		}
		n++;
	}
	if (n > 0)
		wr64(x->rt + XHCI_ERDP,
		    (uint64_t)(uintptr_t)&x->evt[x->evt_deq] | ERDP_EHB);
	return (n);
}

/* Run a command; returns the completion code (0 on timeout). */
static int
xhc_command(struct xhc *x, uint64_t param, uint32_t control, u_int *slot)
{
	uint64_t end;

	cmd_done = false;
	ring_put(&x->cmd, param, 0, control);
	doorbell(x, 0, 0);
	end = now_us() + 500000;
	while (!cmd_done) {
		evt_poll(x);
		if (now_us() > end)
			return (0);
	}
	if (slot != NULL)
		*slot = TRB_GET_SLOT(last_cmd_event.control);
	return (TRB_GET_CODE(last_cmd_event.status));
}

/* ---- contexts --------------------------------------------------------- */

/* Input context: control context, then slot, then EP0 (DCI 1), ... */
static uint32_t *
in_ctl(struct kbd *k)
{
	return ((uint32_t *)k->in_ctx);
}

static uint32_t *
in_slot(struct kbd *k)
{
	return ((uint32_t *)((uint8_t *)k->in_ctx + k->xhc->ctxsz));
}

static uint32_t *
in_ep(struct kbd *k, u_int dci)
{
	return ((uint32_t *)((uint8_t *)k->in_ctx + k->xhc->ctxsz * (1 + dci)));
}

static void
ctx_clear(struct kbd *k)
{
	memset(k->in_ctx, 0, k->xhc->ctxsz * 33);
}

/* xHCI 6.2.3: endpoint context for a ring. */
static void
ep_ctx(uint32_t *ep, u_int type, u_int mps, u_int interval,
    struct ring *r, u_int avg)
{
	uint64_t deq = (uint64_t)(uintptr_t)r->trbs | r->cycle;

	ep[0] = (interval & 0xff) << 16;
	ep[1] = (3U << 1) | (type << 3) | ((mps & 0xffff) << 16); /* CErr 3 */
	ep[2] = (uint32_t)deq;
	ep[3] = (uint32_t)(deq >> 32);
	ep[4] = (avg & 0xffff) | ((type == 7 ? mps : 0) << 16); /* Max ESIT */
}

/* ---- control transfers ------------------------------------------------ */

#define	UT_READ_DEV		0x80
#define	UT_WRITE_DEV		0x00
#define	UT_WRITE_CLASS_IFACE	0x21
#define	UR_GET_DESCRIPTOR	6
#define	UR_SET_CONFIG		9
#define	UR_SET_PROTOCOL		0x0b
#define	UDESC_DEVICE		1
#define	UDESC_CONFIG		2
#define	UDESC_INTERFACE		4
#define	UDESC_ENDPOINT		5

static void
ep0_recover(struct kbd *k)
{
	struct xhc *x = k->xhc;

	/* A STALL halts EP0: reset it and point it past the failed TD. */
	(void)xhc_command(x, 0, TRB_TYPE(T_RESET_EP) | TRB_SLOT(k->slot) |
	    TRB_EP(DCI_EP0), NULL);
	(void)xhc_command(x, (uint64_t)(uintptr_t)&k->ep0.trbs[k->ep0.enq] |
	    k->ep0.cycle, TRB_TYPE(T_SET_TR_DEQ) | TRB_SLOT(k->slot) |
	    TRB_EP(DCI_EP0), NULL);
}

/*
 * One control transfer on EP0.  data/len: an IN buffer in DMA memory when
 * reqtype has 0x80, otherwise no data stage.  Returns 0 or EIO.
 */
static int
control(struct kbd *k, uint8_t reqtype, uint8_t req, uint16_t value,
    uint16_t index, void *data, uint16_t len)
{
	struct xhc *x = k->xhc;
	uint64_t setup, end;
	uint32_t trt, code;
	bool in = (reqtype & 0x80) != 0;

	setup = (uint64_t)reqtype | ((uint64_t)req << 8) |
	    ((uint64_t)value << 16) | ((uint64_t)index << 32) |
	    ((uint64_t)len << 48);
	trt = len == 0 ? 0 : (in ? TRB_TRT_IN : TRB_TRT_OUT);
	ring_put(&k->ep0, setup, 8, TRB_TYPE(T_SETUP) | TRB_IDT | trt);
	if (len != 0)
		ring_put(&k->ep0, (uint64_t)(uintptr_t)data, len,
		    TRB_TYPE(T_DATA) | (in ? TRB_DIR_IN : 0));
	ring_put(&k->ep0, 0, 0, TRB_TYPE(T_STATUS) | TRB_IOC |
	    ((len != 0 && in) ? 0 : TRB_DIR_IN));

	ep0_done = false;
	doorbell(x, k->slot, DCI_EP0);
	end = now_us() + 500000;
	while (!ep0_done) {
		evt_poll(x);
		if (now_us() > end)
			return (EIO);
	}
	code = TRB_GET_CODE(last_ep0_event.status);
	if (code == CC_SUCCESS || code == CC_SHORT_PACKET)
		return (0);
	if (code == CC_STALL)
		ep0_recover(k);
	return (EIO);
}

/* ---- enumeration ------------------------------------------------------ */

static u_int
ep0_mps_for(u_int speed)
{
	switch (speed) {
	case SPEED_LS:	return (8);
	case SPEED_FS:	return (8);		/* corrected after 8 bytes */
	case SPEED_HS:	return (64);
	default:	return (512);
	}
}

/* xHCI interval exponent (125 us units) for an interrupt endpoint. */
static u_int
intr_interval(u_int speed, u_int binterval)
{
	u_int i, frames;

	if (speed == SPEED_HS || speed >= SPEED_SS)
		return (binterval >= 1 && binterval <= 16 ? binterval - 1 : 3);
	/* LS/FS: bInterval in ms; the largest 2^n * 125 us not above it. */
	frames = (binterval == 0 ? 1 : binterval) * 8;
	for (i = 3; i < 10 && (2U << i) <= frames; i++)
		;
	return (i);
}

static int
kbd_setup(struct xhc *x, u_int port, u_int speed)
{
	struct kbd *k;
	uint8_t *buf, *p, *end;
	u_int slot, code, cfgval, iface, ep, mps, ival, total, i;
	bool found;

	if (nkbd == MAXKBD)
		return (ENOSPC);
	k = &kbds[nkbd];
	memset(k, 0, sizeof(*k));
	k->xhc = x;
	k->port = port;
	k->speed = speed;

	code = xhc_command(x, 0, TRB_TYPE(T_ENABLE_SLOT), &slot);
	if (code != CC_SUCCESS || slot == 0 || slot > x->nslots) {
		snprintf(x->portinfo[port - 1], sizeof(x->portinfo[0]),
		    "Enable Slot failed (code %u)", code);
		return (EIO);
	}
	k->slot = slot;
	if ((k->out_ctx = dma_alloc(x->ctxsz * 32, 64)) == NULL ||
	    (k->in_ctx = dma_alloc(x->ctxsz * 33, 64)) == NULL ||
	    ring_init(&k->ep0) != 0 || ring_init(&k->intr) != 0 ||
	    (k->report = dma_alloc(64, 64)) == NULL ||
	    (buf = dma_alloc(512, 64)) == NULL)
		return (ENOMEM);
	x->dcbaa[slot] = (uint64_t)(uintptr_t)k->out_ctx;

	/* Address Device (xHCI 4.3.3). */
	ctx_clear(k);
	in_ctl(k)[1] = (1U << 0) | (1U << 1);		/* A0 slot, A1 EP0 */
	in_slot(k)[0] = (speed << 20) | (1U << 27);	/* entries: 1 */
	in_slot(k)[1] = port << 16;			/* root hub port */
	ep_ctx(in_ep(k, DCI_EP0), 4, ep0_mps_for(speed), 0, &k->ep0, 8);
	code = xhc_command(x, (uint64_t)(uintptr_t)k->in_ctx,
	    TRB_TYPE(T_ADDRESS_DEVICE) | TRB_SLOT(slot), NULL);
	if (code != CC_SUCCESS) {
		snprintf(x->portinfo[port - 1], sizeof(x->portinfo[0]),
		    "Address Device failed (code %u)", code);
		return (EIO);
	}

	/* Device descriptor: 8 bytes for bMaxPacketSize0, then all of it. */
	if (control(k, UT_READ_DEV, UR_GET_DESCRIPTOR, UDESC_DEVICE << 8, 0,
	    buf, 8) != 0) {
		snprintf(x->portinfo[port - 1], sizeof(x->portinfo[0]),
		    "no device descriptor");
		return (EIO);
	}
	if (speed == SPEED_FS && buf[7] != 8 && buf[7] != 0) {
		/* Evaluate Context with the real EP0 packet size. */
		ctx_clear(k);
		in_ctl(k)[1] = 1U << 1;
		ep_ctx(in_ep(k, DCI_EP0), 4, buf[7], 0, &k->ep0, 8);
		/* Keep the dequeue pointer the controller already has. */
		in_ep(k, DCI_EP0)[2] = 0;
		in_ep(k, DCI_EP0)[3] = 0;
		(void)xhc_command(x, (uint64_t)(uintptr_t)k->in_ctx,
		    TRB_TYPE(T_EVALUATE_CTX) | TRB_SLOT(slot), NULL);
	}
	if (control(k, UT_READ_DEV, UR_GET_DESCRIPTOR, UDESC_DEVICE << 8, 0,
	    buf, 18) != 0)
		return (EIO);
	if (buf[4] == 0x09) {
		snprintf(x->portinfo[port - 1], sizeof(x->portinfo[0]),
		    "hub (not supported; keyboard on a root port only)");
		return (ENOTSUP);
	}

	/* Configuration descriptor: header, then all of it (to 512 bytes). */
	if (control(k, UT_READ_DEV, UR_GET_DESCRIPTOR, UDESC_CONFIG << 8, 0,
	    buf, 9) != 0)
		return (EIO);
	total = buf[2] | (buf[3] << 8);
	if (total > 512)
		total = 512;
	if (control(k, UT_READ_DEV, UR_GET_DESCRIPTOR, UDESC_CONFIG << 8, 0,
	    buf, total) != 0)
		return (EIO);
	cfgval = buf[5];

	/* The first HID boot keyboard interface and its interrupt IN. */
	found = false;
	iface = ep = mps = ival = 0;
	p = buf;
	end = buf + total;
	while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
		if (p[1] == UDESC_INTERFACE && p[0] >= 9) {
			if (found && ep != 0)
				break;
			found = (p[5] == 3 && p[6] == 1 && p[7] == 1);
			iface = p[2];
		} else if (p[1] == UDESC_ENDPOINT && p[0] >= 7 && found &&
		    ep == 0 && (p[2] & 0x80) != 0 && (p[3] & 3) == 3) {
			ep = p[2] & 0xf;
			mps = (p[4] | (p[5] << 8)) & 0x7ff;
			ival = p[6];
		}
		p += p[0];
	}
	if (!found || ep == 0) {
		snprintf(x->portinfo[port - 1], sizeof(x->portinfo[0]),
		    "not a boot keyboard");
		return (ENOTSUP);
	}

	if (control(k, UT_WRITE_DEV, UR_SET_CONFIG, cfgval, 0, NULL, 0) != 0)
		return (EIO);
	/* Boot protocol; a device that refuses is probably in it already. */
	(void)control(k, UT_WRITE_CLASS_IFACE, UR_SET_PROTOCOL, 0, iface,
	    NULL, 0);

	/* Configure Endpoint (xHCI 4.3.5) for the interrupt IN endpoint. */
	k->dci = ep * 2 + 1;
	k->mps = mps == 0 ? 8 : mps;
	ctx_clear(k);
	in_ctl(k)[1] = (1U << 0) | (1U << k->dci);
	memcpy(in_slot(k), k->out_ctx, 16);	/* the controller's slot ctx */
	in_slot(k)[0] = (in_slot(k)[0] & ~(0x1fU << 27)) |
	    ((uint32_t)k->dci << 27);
	in_slot(k)[3] = 0;
	ep_ctx(in_ep(k, k->dci), 7, k->mps, intr_interval(speed, ival),
	    &k->intr, k->mps);
	code = xhc_command(x, (uint64_t)(uintptr_t)k->in_ctx,
	    TRB_TYPE(T_CONFIGURE_EP) | TRB_SLOT(slot), NULL);
	if (code != CC_SUCCESS) {
		snprintf(x->portinfo[port - 1], sizeof(x->portinfo[0]),
		    "Configure Endpoint failed (code %u)", code);
		return (EIO);
	}

	for (i = 0; i < sizeof(k->last); i++)
		k->last[i] = 0;
	snprintf(x->portinfo[port - 1], sizeof(x->portinfo[0]),
	    "keyboard (%s, interface %u, endpoint 0x%x)",
	    speed == SPEED_LS ? "low" : speed == SPEED_FS ? "full" :
	    speed == SPEED_HS ? "high" : "super", iface, 0x80 | ep);
	nkbd++;
	return (0);
}

static void
kbd_queue(struct kbd *k)
{
	if (k->pending)
		return;
	ring_put(&k->intr, (uint64_t)(uintptr_t)k->report, k->mps,
	    TRB_TYPE(T_NORMAL) | TRB_IOC | TRB_ISP);
	k->pending = true;
	doorbell(k->xhc, k->slot, k->dci);
}

/* ---- keys ------------------------------------------------------------- */

static void
kq_put(uint8_t c)
{
	u_int next = (kq_head + 1) % KQ_SIZE;

	if (next != kq_tail) {
		kq[kq_head] = c;
		kq_head = next;
	}
}

static void
kq_puts(const char *s)
{
	while (*s != '\0')
		kq_put((uint8_t)*s++);
}

/* HID usage page 7, 0x04-0x38, unshifted and shifted (US layout). */
static const char keymap[2][0x39] = {
	{ 0, 0, 0, 0, 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
	  'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y',
	  'z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '\r', 0x1b,
	  0x08, '\t', ' ', '-', '=', '[', ']', '\\', 0, ';', '\'', '`', ',',
	  '.', '/' },
	{ 0, 0, 0, 0, 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K',
	  'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y',
	  'Z', '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '\r', 0x1b,
	  0x08, '\t', ' ', '_', '+', '{', '}', '|', 0, ':', '"', '~', '<',
	  '>', '?' },
};
static const char keypad[] = "/*-+\r1234567890.";	/* 0x54-0x63 */
static bool capslock;

static void
key_press(uint8_t mods, uint8_t key)
{
	bool shift = (mods & 0x22) != 0, ctrl = (mods & 0x11) != 0;
	char c = 0;

	if (key >= 0x04 && key <= 0x38) {
		c = keymap[shift][key];
		if (capslock && key <= 0x1d)
			c = keymap[!shift][key];
		if (ctrl && c >= '@' && c <= '~')
			c &= 0x1f;
	} else if (key >= 0x54 && key <= 0x63) {
		c = keypad[key - 0x54];
	} else {
		switch (key) {
		case 0x39: capslock = !capslock; return;
		case 0x4f: kq_puts("\033[C"); return;	/* right */
		case 0x50: kq_puts("\033[D"); return;	/* left */
		case 0x51: kq_puts("\033[B"); return;	/* down */
		case 0x52: kq_puts("\033[A"); return;	/* up */
		case 0x4a: kq_puts("\033[H"); return;	/* home */
		case 0x4d: kq_puts("\033[F"); return;	/* end */
		case 0x4c: c = 0x7f; break;		/* delete */
		default: return;
		}
	}
	if (c != 0)
		kq_put((uint8_t)c);
}

#define	RPT_DELAY_US		500000
#define	RPT_RATE_US		40000

static void
kbd_report(struct kbd *k, uint32_t code, uint32_t residue)
{
	uint8_t *r = k->report;
	u_int i, j;
	bool held;

	k->pending = false;
	if ((code != CC_SUCCESS && code != CC_SHORT_PACKET) ||
	    residue > k->mps || k->mps - residue < 8)
		return;			/* requeued by the poll loop */
	if (r[2] == 0x01)		/* ErrorRollOver: too many keys */
		return;
	for (i = 2; i < 8; i++) {
		if (r[i] == 0)
			continue;
		held = false;
		for (j = 2; j < 8; j++)
			if (k->last[j] == r[i])
				held = true;
		if (!held) {
			key_press(r[0], r[i]);
			k->rpt_key = r[i];
			k->rpt_at = now_us() + RPT_DELAY_US;
		}
	}
	held = false;
	for (i = 2; i < 8; i++)
		if (r[i] == k->rpt_key && k->rpt_key != 0)
			held = true;
	if (!held)
		k->rpt_key = 0;
	memcpy(k->last, r, 8);
}

/* ---- controllers ------------------------------------------------------ */

static void
dwc3_init(uintptr_t b)
{
	uint32_t ghwp0, gctl, phy2, phy3;
	bool drd;

	ghwp0 = rd32(b + DWC3_GHWPARAMS0);
	drd = (ghwp0 & GHWPARAMS0_MODE_MASK) == GHWPARAMS0_MODE_DRD;

	/* snps_dwc3_reset(): core and both PHYs through soft reset. */
	gctl = rd32(b + DWC3_GCTL) | GCTL_CORESOFTRESET;
	wr32(b + DWC3_GCTL, gctl);
	phy2 = rd32(b + DWC3_GUSB2PHYCFG0) | GUSB2_PHYSOFTRST;
	if (drd)
		phy2 &= ~GUSB2_SUSPENDUSB20;
	wr32(b + DWC3_GUSB2PHYCFG0, phy2);
	phy3 = rd32(b + DWC3_GUSB3PIPECTL0) | GUSB3_PHYSOFTRST;
	if (drd)
		phy3 &= ~GUSB3_SUSPENDUSB3;
	wr32(b + DWC3_GUSB3PIPECTL0, phy3);
	delay(1000);
	wr32(b + DWC3_GUSB2PHYCFG0, phy2 & ~GUSB2_PHYSOFTRST);
	wr32(b + DWC3_GUSB3PIPECTL0, phy3 & ~GUSB3_PHYSOFTRST);
	wr32(b + DWC3_GCTL, gctl & ~GCTL_CORESOFTRESET);

	/* snps_dwc3_configure_host(): host mode, IN auto retry. */
	gctl = rd32(b + DWC3_GCTL);
	gctl = (gctl & ~GCTL_PRTCAPDIR_MASK) | GCTL_PRTCAPDIR_HOST;
	wr32(b + DWC3_GCTL, gctl);
	wr32(b + DWC3_GUCTL, rd32(b + DWC3_GUCTL) | GUCTL_HOST_AUTO_RETRY);

	/*
	 * snps_dwc3_do_quirks() for RP1's nodes, which name
	 * snps,dis_rxdet_inp3_quirk and none of the u2/u3 suspend or
	 * freeclk quirks.
	 */
	phy2 = rd32(b + DWC3_GUSB2PHYCFG0) | GUSB2_U2_FREECLK_EXISTS |
	    GUSB2_ENBLSLPM;
	if (drd)
		phy2 |= GUSB2_SUSPENDUSB20;
	wr32(b + DWC3_GUSB2PHYCFG0, phy2);
	phy3 = rd32(b + DWC3_GUSB3PIPECTL0) | GUSB3_DISRXDETINP3;
	if (drd)
		phy3 |= GUSB3_SUSPENDUSB3;
	wr32(b + DWC3_GUSB3PIPECTL0, phy3);
}

static int
xhc_start(struct xhc *x, uintptr_t base)
{
	uint32_t hcs1, hcs2, hcc1, id;
	uint64_t *spa, *erst;
	u_int nsp, i;

	x->base = base;
	id = rd32(base + DWC3_GSNPSID);
	if (id == 0xffffffff || id == 0xdeaddead || id == 0)
		return (ENXIO);		/* nothing answering behind RP1 */
	dwc3_init(base);

	x->op = base + (rd32(base + XHCI_CAPLENGTH) & 0xff);
	x->rt = base + (rd32(base + XHCI_RTSOFF) & ~0x1fU);
	x->db = base + (rd32(base + XHCI_DBOFF) & ~0x3U);
	hcs1 = rd32(base + XHCI_HCSPARAMS1);
	hcs2 = rd32(base + XHCI_HCSPARAMS2);
	hcc1 = rd32(base + XHCI_HCCPARAMS1);
	x->ctxsz = (hcc1 & HCC_CSZ) ? 64 : 32;
	x->nports = MIN((hcs1 >> 24) & 0xff, MAXPORTS);
	x->nslots = MIN(hcs1 & 0xff, 16);

	/* Halt, then reset (xHCI 4.2). */
	wr32(x->op + XHCI_USBCMD, rd32(x->op + XHCI_USBCMD) & ~CMD_RS);
	if (!wait_bits(x->op + XHCI_USBSTS, STS_HCH, STS_HCH, 100))
		return (ETIMEDOUT);
	wr32(x->op + XHCI_USBCMD, CMD_HCRST);
	if (!wait_bits(x->op + XHCI_USBCMD, CMD_HCRST, 0, 1000) ||
	    !wait_bits(x->op + XHCI_USBSTS, STS_CNR, 0, 1000))
		return (ETIMEDOUT);
	if ((rd32(x->op + XHCI_PAGESIZE) & 1) == 0)
		return (ENXIO);		/* not 4 KiB pages */

	wr32(x->op + XHCI_CONFIG, x->nslots);
	if ((x->dcbaa = dma_alloc(2048, 64)) == NULL)
		return (ENOMEM);
	nsp = ((hcs2 >> 21) & 0x1f) << 5 | ((hcs2 >> 27) & 0x1f);
	if (nsp > 0) {
		if ((spa = dma_alloc(nsp * 8, 64)) == NULL)
			return (ENOMEM);
		for (i = 0; i < nsp; i++) {
			void *pg = dma_alloc(4096, 4096);

			if (pg == NULL)
				return (ENOMEM);
			spa[i] = (uint64_t)(uintptr_t)pg;
		}
		x->dcbaa[0] = (uint64_t)(uintptr_t)spa;
	}
	wr64(x->op + XHCI_DCBAAP, (uint64_t)(uintptr_t)x->dcbaa);

	if (ring_init(&x->cmd) != 0)
		return (ENOMEM);
	wr64(x->op + XHCI_CRCR, (uint64_t)(uintptr_t)x->cmd.trbs | 1);

	if ((x->evt = dma_alloc(RING_TRBS * sizeof(struct trb), 4096)) ==
	    NULL || (erst = dma_alloc(16, 64)) == NULL)
		return (ENOMEM);
	erst[0] = (uint64_t)(uintptr_t)x->evt;
	erst[1] = RING_TRBS;
	x->evt_deq = 0;
	x->evt_cycle = 1;
	wr32(x->rt + XHCI_IMAN, 0);		/* polled: no interrupts */
	wr32(x->rt + XHCI_ERSTSZ, 1);
	wr64(x->rt + XHCI_ERDP, (uint64_t)(uintptr_t)x->evt);
	wr64(x->rt + XHCI_ERSTBA, (uint64_t)(uintptr_t)erst);

	DMB();
	wr32(x->op + XHCI_USBCMD, CMD_RS);
	if (!wait_bits(x->op + XHCI_USBSTS, STS_HCH, 0, 100))
		return (ETIMEDOUT);

	/* Power every port (a no-op where the hardware controls power). */
	for (i = 1; i <= x->nports; i++) {
		uint32_t ps = rd32(x->op + XHCI_PORTSC(i));

		if ((ps & PS_PP) == 0)
			wr32(x->op + XHCI_PORTSC(i), (ps & PS_KEEP) | PS_PP);
	}
	x->up = true;
	return (0);
}

/* Reset a USB 2 port and wait for it to be enabled; USB 3 ports train. */
static int
port_enable(struct xhc *x, u_int port)
{
	uintptr_t a = x->op + XHCI_PORTSC(port);
	uint32_t ps = rd32(a);

	if ((ps & PS_PED) == 0) {
		wr32(a, (ps & PS_KEEP) | PS_PR);
		if (!wait_bits(a, PS_PRC, PS_PRC, 500))
			return (ETIMEDOUT);
		delay(20000);		/* reset recovery (USB 2.0 7.1.7.5) */
	}
	ps = rd32(a);
	wr32(a, (ps & PS_KEEP) | (ps & PS_CHANGE));	/* clear changes */
	if ((rd32(a) & (PS_CCS | PS_PED)) != (PS_CCS | PS_PED))
		return (ENXIO);
	return (0);
}

/*
 * After the controller reset, devices reconnect: give them up to 600 ms to
 * appear on any port of any controller, then 100 ms to settle (USB 2.0
 * 7.1.7.3).
 */
static bool
xhc_wait_connect(void)
{
	uint64_t end = now_us() + 600000;
	u_int c, i;

	do {
		for (c = 0; c < NCTRL; c++) {
			if (!xhcs[c].up)
				continue;
			for (i = 1; i <= xhcs[c].nports; i++)
				if (rd32(xhcs[c].op + XHCI_PORTSC(i)) & PS_CCS) {
					delay(100000);
					return (true);
				}
		}
	} while (now_us() < end);
	return (false);
}

static void
xhc_scan(struct xhc *x)
{
	uint32_t ps;
	u_int i, speed;

	for (i = 1; i <= x->nports; i++) {
		ps = rd32(x->op + XHCI_PORTSC(i));
		if ((ps & PS_CCS) == 0)
			continue;
		if (port_enable(x, i) != 0) {
			snprintf(x->portinfo[i - 1], sizeof(x->portinfo[0]),
			    "connected, but the port did not enable");
			continue;
		}
		speed = PS_SPEED(rd32(x->op + XHCI_PORTSC(i)));
		if (kbd_setup(x, i, speed) != 0 && x->portinfo[i - 1][0] == 0)
			snprintf(x->portinfo[i - 1], sizeof(x->portinfo[0]),
			    "device set-up failed");
	}
}

/* ---- the interface ---------------------------------------------------- */

/*
 * Find keyboards.  Called once from main() with the MMU on; quiet unless
 * something is found or something failed after RP1 was found.
 */
void
rpi_usbkbd_init(void)
{
	uintptr_t rp1;
	u_int c, i;
	int error;

	if (usb_tried)
		return;
	usb_tried = true;
	if (!rpi_mmu_enabled()) {
		usb_why = "the MMU is off";
		return;
	}
	if (rpi_pcie_rp1_init(&usb_why) != 0)
		return;
	if (rpi_mmu_set_nc(DMA_BASE, DMA_SIZE) != 0) {
		usb_why = "could not map the DMA region non-cacheable";
		return;
	}
	dma_next = DMA_BASE;
	rp1 = (uintptr_t)rpi_pcie_rp1_base();
	for (c = 0; c < NCTRL; c++) {
		error = xhc_start(&xhcs[c], rp1 + 0x200000 + c * 0x100000);
		if (error != 0)
			printf("USB: %s did not start (error %d)\n",
			    xhcs[c].name, error);
	}
	if (xhc_wait_connect())
		for (c = 0; c < NCTRL; c++)
			if (xhcs[c].up)
				xhc_scan(&xhcs[c]);
	for (i = 0; i < nkbd; i++)
		kbd_queue(&kbds[i]);
	usb_why = nkbd > 0 ? NULL : "no boot keyboard on a root port";
	if (nkbd > 0)
		printf("USB keyboard%s: %u found\n", nkbd > 1 ? "s" : "",
		    nkbd);
}

/* Service the controllers; true if a character is waiting. */
bool
rpi_usbkbd_poll(void)
{
	uint64_t now;
	u_int c, i;

	for (c = 0; c < NCTRL; c++)
		if (xhcs[c].up)
			evt_poll(&xhcs[c]);
	now = now_us();
	for (i = 0; i < nkbd; i++) {
		struct kbd *k = &kbds[i];

		if (!k->pending)
			kbd_queue(k);
		if (k->rpt_key != 0 && now >= k->rpt_at) {
			key_press(k->last[0], k->rpt_key);
			k->rpt_at = now + RPT_RATE_US;
		}
	}
	return (kq_head != kq_tail);
}

int
rpi_usbkbd_getchar(void)
{
	int c;

	if (!rpi_usbkbd_poll())
		return (-1);
	c = kq[kq_tail];
	kq_tail = (kq_tail + 1) % KQ_SIZE;
	return (c);
}

/* Before the kernel: stop and reset the controllers, then RP1's mastering. */
void
rpi_usbkbd_shutdown(void)
{
	struct xhc *x;
	u_int c;

	for (c = 0; c < NCTRL; c++) {
		x = &xhcs[c];
		if (!x->up)
			continue;
		wr32(x->op + XHCI_USBCMD, rd32(x->op + XHCI_USBCMD) & ~CMD_RS);
		(void)wait_bits(x->op + XHCI_USBSTS, STS_HCH, STS_HCH, 100);
		wr32(x->op + XHCI_USBCMD, CMD_HCRST);
		(void)wait_bits(x->op + XHCI_USBCMD, CMD_HCRST, 0, 1000);
		x->up = false;
	}
	nkbd = 0;
	rpi_pcie_shutdown();
}

static int
command_usbinfo(int argc __unused, char *argv[] __unused)
{
	struct xhc *x;
	u_int c, i;

	if (usb_why != NULL)
		printf("USB keyboard: %s\n", usb_why);
	for (c = 0; c < NCTRL; c++) {
		x = &xhcs[c];
		if (x->base == 0)
			continue;
		printf("%s at 0x%lx: %s, %u ports, %u-byte contexts\n",
		    x->name, (unsigned long)x->base, x->up ? "running" : "down",
		    x->nports, x->ctxsz);
		for (i = 0; i < x->nports; i++)
			if (x->portinfo[i][0] != '\0')
				printf("   port %u: %s\n", i + 1, x->portinfo[i]);
	}
	printf("DMA region 0x%lx, %lu bytes used\n", DMA_BASE,
	    (unsigned long)(dma_next > DMA_BASE ? dma_next - DMA_BASE : 0));
	return (CMD_OK);
}
COMMAND_SET(usbinfo, "usbinfo", "show the USB keyboard controllers and ports",
    command_usbinfo);
