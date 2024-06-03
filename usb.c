/* $(CROSS_COMPILE)cc -Wall       -g -o usb usb.c usbstring.c -lpthread
 *	-OR-
 * $(CROSS_COMPILE)cc -Wall -DAIO -g -o usb usb.c usbstring.c -lpthread -laio
 */

/*
 * this is an example pthreaded USER MODE driver implementing a
 * USB Gadget/Device with simple bulk bulk_in/bulk_out functionality.
 * you could implement pda sync software this way, or some usb class
 * protocols (printers, test-and-measurement equipment, and so on).
 *
 * with hardware that also supports isochronous data transfers, this
 * can stream data using multi-buffering and AIO.  that's the way to
 * handle audio or video data, where on-time delivery is essential.
 *
 * needs "gadgetfs" and a supported USB device controller driver
 * in the kernel; this autoconfigures, based on the driver it finds.
 */

#include <errno.h>
#include <fcntl.h>
#include <memory.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <asm/byteorder.h>

#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadgetfs.h>

#ifdef AIO
/* this aio code works with libaio-0.3.106 */
#include <libaio.h>
#endif

#include "usbstring.h"

static int verbose = 3;
static int pattern;

/* Thanks to NetChip Technologies for donating this product ID.
 *
 * DO NOT REUSE THESE IDs with any protocol-incompatible driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures.
 */
#define DRIVER_VENDOR_NUM 0x1004      /* NetChip */
#define DRIVER_ISO_PRODUCT_NUM 0xa4a3 /* user mode iso bulk_out/bulk_in */
#define DRIVER_PRODUCT_NUM 0x61a1     /* user mode bulk_out/bulk_in */

/* NOTE:  these IDs don't imply endpoint numbering; host side drivers
 * should use endpoint descriptors, or perhaps bcdDevice, to configure
 * such things.  Other product IDs could have different policies.
 */

/*-------------------------------------------------------------------------*/

/* these descriptors are modified based on what controller we find */

#define STRINGID_MFGR 1
#define STRINGID_PRODUCT 2
#define STRINGID_SERIAL 3
#define STRINGID_CONFIG 4
#define STRINGID_INTERFACE0 5
#define STRINGID_INTERFACE1 6

static struct usb_device_descriptor device_desc = {
    .bLength = sizeof device_desc,
    .bDescriptorType = USB_DT_DEVICE,

    .bcdUSB = __constant_cpu_to_le16(0x0200),
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    // .bMaxPacketSize0 ... set by gadgetfs
    .idVendor = __constant_cpu_to_le16(DRIVER_VENDOR_NUM),
    .idProduct = __constant_cpu_to_le16(DRIVER_PRODUCT_NUM),
    .iManufacturer = STRINGID_MFGR,
    .iProduct = STRINGID_PRODUCT,
    .iSerialNumber = STRINGID_SERIAL,
    .bNumConfigurations = 1,
};

#define MAX_USB_POWER 1

#define CONFIG_VALUE 3

static const struct usb_config_descriptor config = {
    .bLength = sizeof config,
    .bDescriptorType = USB_DT_CONFIG,

    /* must compute wTotalLength ... */
    .bNumInterfaces = 2,
    .bConfigurationValue = CONFIG_VALUE,
    .iConfiguration = STRINGID_CONFIG,
    .bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
    .bMaxPower = (MAX_USB_POWER + 1) / 2,
};

static struct usb_interface_descriptor interface0 = {
    .bLength = sizeof interface0,
    .bDescriptorType = USB_DT_INTERFACE,

    .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
    .iInterface = STRINGID_INTERFACE0,
};

static struct usb_interface_descriptor interface1 = {
    .bLength = sizeof interface1,
    .bDescriptorType = USB_DT_INTERFACE,

    .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
    .iInterface = STRINGID_INTERFACE1,
};

/* Full speed configurations are used for full-speed only devices as
 * well as dual-speed ones (the only kind with high speed support).
 */

static struct usb_endpoint_descriptor fs_bulk_in_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,

    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    /* NOTE some controllers may need FS bulk max packet size
     * to be smaller.  it would be a chip-specific option.
     */
    .wMaxPacketSize = __constant_cpu_to_le16(64),
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,

    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = __constant_cpu_to_le16(64),
};

/* some devices can handle other status packet sizes */
#define STATUS_MAXPACKET 0x100
#define LOG2_STATUS_POLL_MSEC 3

static struct usb_endpoint_descriptor fs_iso_in_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,

    .bmAttributes = USB_ENDPOINT_XFER_INT,
    .wMaxPacketSize = __constant_cpu_to_le16(STATUS_MAXPACKET),
    .bInterval = (1 << LOG2_STATUS_POLL_MSEC),
};

static const struct usb_endpoint_descriptor *fs_eps[3] = {
    &fs_bulk_in_desc,
    &fs_bulk_out_desc,
    &fs_iso_in_desc,
};

/* High speed configurations are used only in addition to a full-speed
 * ones ... since all high speed devices support full speed configs.
 * Of course, not all hardware supports high speed configurations.
 */

static struct usb_endpoint_descriptor hs_bulk_int_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,

    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,

    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = __constant_cpu_to_le16(512),
    .bInterval = 1,
};

static struct usb_endpoint_descriptor hs_iso_in_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,

    .bmAttributes = USB_ENDPOINT_XFER_ISOC,
    .bInterval = 1,
};

static const struct usb_endpoint_descriptor *hs_eps[] = {
    &hs_bulk_int_desc,
    &hs_bulk_out_desc,
    &hs_iso_in_desc,
};

/*-------------------------------------------------------------------------*/

static char serial[64];

static struct usb_string stringtab[] = {
    {
        STRINGID_MFGR,
        "Licensed to Inc.VaultMicro",
    },
    {
        STRINGID_PRODUCT,
        "hvperf",
    },
    {
        STRINGID_SERIAL,
        serial,
    },
    {
        STRINGID_CONFIG,
        "The Configuration",
    },
    {
        STRINGID_INTERFACE0,
        "Bulk I/O",
    },
    {
        STRINGID_INTERFACE1,
        "Isochronous In",
    },
};

static struct usb_gadget_strings strings = {
    .language = 0x0409, /* "en-us" */
    .strings = stringtab,
};

/*-------------------------------------------------------------------------*/

/* kernel drivers could autoconfigure like this too ... if
 * they were willing to waste the relevant code/data space.
 */

static int HIGHSPEED;
static char *DEVNAME;
static char *EP_IN_NAME, *EP_OUT_NAME, *EP_ISO_IN_NAME;

/* gadgetfs currently has no chunking (or O_DIRECT/zerocopy) support
 * to turn big requests into lots of smaller ones; so this is "small".
 */
#define USB_BUFSIZE (7 * 1024)

static enum usb_device_speed current_speed;

static inline int min(unsigned a, unsigned b) { return (a < b) ? a : b; }

static int iso;
static int interval;
static unsigned iosize;
static unsigned bufsize = USB_BUFSIZE;

static int autoconfig() {
    struct stat statb;

    /* NetChip 2280 PCI device or dummy_hcd, high/full speed */
    if (stat(DEVNAME = "fe980000.usb", &statb) == 0) {
        HIGHSPEED = 1;
        // device_desc.bcdDevice = __constant_cpu_to_le16(0x0107),
        device_desc.bcdDevice = __constant_cpu_to_le16(0x0103);

        fs_bulk_in_desc.bEndpointAddress = hs_bulk_int_desc.bEndpointAddress = USB_DIR_IN | 1;
        EP_IN_NAME = "ep1in";
        fs_bulk_out_desc.bEndpointAddress = hs_bulk_out_desc.bEndpointAddress = USB_DIR_OUT | 2;
        EP_OUT_NAME = "ep2out";

        interface0.bNumEndpoints = 2;

        fs_iso_in_desc.bEndpointAddress = hs_iso_in_desc.bEndpointAddress = USB_DIR_IN | 3;
        fs_iso_in_desc.bmAttributes = hs_iso_in_desc.bmAttributes = USB_ENDPOINT_XFER_ISOC;
        hs_iso_in_desc.wMaxPacketSize = 1024;
        if (hs_iso_in_desc.wMaxPacketSize > 5120) {
            fprintf(stderr, "Iso wMaxPacketSize is 0x1400, 5120bytes\n");
            hs_iso_in_desc.wMaxPacketSize = 5120;
        }
        EP_ISO_IN_NAME = "ep3in";

        /* Atmel AT91 processors, full speed only */
    } else {
        DEVNAME = 0;
        return -ENODEV;
    }
    if (verbose) {
        if (HIGHSPEED) {
            fprintf(stderr, "iso hs wMaxPacket 0x%04x bInterval %02x\n",
                    __le16_to_cpu(hs_iso_in_desc.wMaxPacketSize), hs_iso_in_desc.bInterval);
        }
    }

    return 0;
}

/*-------------------------------------------------------------------------*/

/* full duplex data, with at least three threads: ep0, bulk_out, and bulk_in */

static pthread_t ep0;

static pthread_t bulk_in;
static int bulk_in_fd = -1;

static pthread_t bulk_out;
static int bulk_out_fd = -1;

static pthread_t iso_in;
static int iso_in_fd = -1;

// FIXME no status i/o yet

static void close_fd(void *fd_ptr) {
    int status, fd;

    fd = *(int *)fd_ptr;
    *(int *)fd_ptr = -1;

    /* test the FIFO ioctls (non-ep0 code paths) */
    if (pthread_self() != ep0) {
        status = ioctl(fd, GADGETFS_FIFO_STATUS);
        if (status < 0) {
            /* ENODEV reported after disconnect */
            if (errno != ENODEV && errno != -EOPNOTSUPP)
                perror("get fifo status");
        } else {
            fprintf(stderr, "fd %d, unclaimed = %d\n", fd, status);
            if (status) {
                status = ioctl(fd, GADGETFS_FIFO_FLUSH);
                if (status < 0)
                    perror("fifo flush");
            }
        }
    }

    if (close(fd) < 0)
        perror("close");
}

/* you should be able to open and configure endpoints
 * whether or not the host is connected
 */
static int ep_config(char *name, const char *label, struct usb_endpoint_descriptor *fs,
                     struct usb_endpoint_descriptor *hs) {
    int fd, status;
    char buf[USB_BUFSIZE];

    /* open and initialize with endpoint descriptor(s) */
    fd = open(name, O_RDWR);
    if (fd < 0) {
        status = -errno;
        fprintf(stderr, "%s open %s error %d (%s)\n", label, name, errno, strerror(errno));
        return status;
    }

    /* one (fs or ls) or two (fs + hs) sets of config descriptors */
    *(__u32 *)buf = 1; /* tag for this format */
    memcpy(buf + 4, fs, USB_DT_ENDPOINT_SIZE);
    if (HIGHSPEED) {
        memcpy(buf + 4 + USB_DT_ENDPOINT_SIZE, hs, USB_DT_ENDPOINT_SIZE);
    }
    status = write(fd, buf, 4 + USB_DT_ENDPOINT_SIZE + (HIGHSPEED ? USB_DT_ENDPOINT_SIZE : 0));
    if (status < 0) {
        status = -errno;
        fprintf(stderr, "%s config %s error %d (%s)\n", label, name, errno, strerror(errno));
        close(fd);
        return status;
    } else if (verbose) {
        unsigned long id = pthread_self();
        fprintf(stderr, "%s start %ld fd %d\n", label, id, fd);
    }
    return fd;
}

#define bulk_in_open(name) ep_config(name, __FUNCTION__, &fs_bulk_in_desc, &hs_bulk_int_desc)
#define bulk_out_open(name) ep_config(name, __FUNCTION__, &fs_bulk_out_desc, &hs_bulk_out_desc)
#define iso_in_open(name) ep_config(name, __FUNCTION__, &fs_iso_in_desc, &hs_iso_in_desc)

static unsigned long fill_in_buf(void *buf, unsigned long nbytes) {
#ifdef DO_PIPE
    /* pipe stdin to host */
    nbytes = fread(buf, 1, nbytes, stdin);
    if (nbytes == 0) {
        if (ferror(stdin))
            perror("read stdin");
        if (feof(stdin))
            errno = ENODEV;
    }
#else
    switch (pattern) {
        unsigned i;

    default:
        // FALLTHROUGH
    case 0: /* endless streams of zeros */
        memset(buf, 0, nbytes);
        break;
    case 1: /* mod63 repeating pattern */
        for (i = 0; i < nbytes; i++)
            ((__u8 *)buf)[i] = (__u8)(i % 63);
        break;
    }
#endif
    return nbytes;
}

static int empty_out_buf(void *buf, unsigned long nbytes) {
    unsigned len;

#ifdef DO_PIPE
    /* pipe from host to stdout */
    len = fwrite(buf, nbytes, 1, stdout);
    if (len != nbytes) {
        if (ferror(stdout))
            perror("write stdout");
    }
#else
    unsigned i;
    __u8 expected, *data;

    for (i = 0, data = buf; i < nbytes; i++, data++) {
        switch (pattern) {
        case 0:
            expected = 0;
            break;
        case 1:
            expected = i % 63;
            break;
        default: /* no verify */
            i = nbytes - 1;
            continue;
        }
        if (*data == expected)
            continue;
        fprintf(stderr, "bad OUT byte %d, expected %02x got %02x\n", i, expected, *data);
        for (i = 0, data = 0; i < nbytes; i++, data++) {
            if (0 == (i % 16))
                fprintf(stderr, "%4d:", i);
            fprintf(stderr, " %02x", *data);
            if (15 == (i % 16))
                fprintf(stderr, "\n");
        }
        return -1;
    }
    len = i;
#endif
    memset(buf, 0, nbytes);
    return len;
}

static void *simple_in_thread(void *param) {
    char *name = (char *)param;
    int status;
    char buf[USB_BUFSIZE];

    status = bulk_in_open(name);
    if (status < 0)
        return 0;
    bulk_in_fd = status;

    pthread_cleanup_push(close_fd, &bulk_in_fd);
    do {
        unsigned long len;

        /* original LinuxThreads cancelation didn't work right
         * so test for it explicitly.
         */
        pthread_testcancel();

        len = fill_in_buf(buf, sizeof buf);
        if (len > 0)
            status = write(bulk_in_fd, buf, len);
        else
            status = 0;

    } while (status > 0);
    if (status == 0) {
        if (verbose)
            fprintf(stderr, "done %s\n", __FUNCTION__);
    } else if (verbose > 2 || errno != ESHUTDOWN) /* normal disconnect */
        perror("write");
    fflush(stdout);
    fflush(stderr);
    pthread_cleanup_pop(1);

    return 0;
}

static void *simple_out_thread(void *param) {
    char *name = (char *)param;
    int status;
    char buf[USB_BUFSIZE];

    status = bulk_out_open(name);
    if (status < 0)
        return 0;
    bulk_out_fd = status;

    /* synchronous reads of endless streams of data */
    pthread_cleanup_push(close_fd, &bulk_out_fd);
    do {
        /* original LinuxThreads cancelation didn't work right
         * so test for it explicitly.
         */
        pthread_testcancel();
        errno = 0;
        status = read(bulk_out_fd, buf, sizeof buf);

        if (status < 0)
            break;
        status = empty_out_buf(buf, status);
    } while (status > 0);
    if (status == 0) {
        if (verbose)
            fprintf(stderr, "done %s\n", __FUNCTION__);
    } else if (verbose > 2 || errno != ESHUTDOWN) /* normal disconnect */
        perror("read");
    fflush(stdout);
    fflush(stderr);
    pthread_cleanup_pop(1);

    return 0;
}

static void *(*bulk_in_thread)(void *);
static void *(*bulk_out_thread)(void *);
static void *(*iso_in_thread)(void *);

/*
 * aio is used here to keep i/o queues from emptying very often.  that
 * can increase throughput for non-control transfers, and it also helps
 * isochronous streams avoid packet data dropout.
 */

static unsigned aio_in = 0;
static unsigned aio_out = 0;

/* urgh, this is messy ... should couple it to the io_context  */
static unsigned aio_in_pending, aio_out_pending;

static void queue_release(void *ctx_ptr) { io_destroy(*(io_context_t *)ctx_ptr); }

static int io_run(io_context_t ctx, volatile unsigned *pending) {
    int ret;
    struct io_event e[5];

    /* process iocbs so long as they reissue */
    while (pending) {
        unsigned i;
        struct iocb *iocb;
        io_callback_t io_complete;

        /* wait for at least one event */
        ret = io_getevents(ctx, 1, 5, &e[0], 0);
        if (ret < 0)
            break;
        for (i = 0; i < ret; i++) {
            io_complete = (io_callback_t)e[i].data;
            iocb = (struct iocb *)e[i].obj;
            io_complete(ctx, iocb, e[i].res, e[i].res2);
        }
    }

    return ret;
}

static void in_complete(io_context_t ctx, struct iocb *iocb, long res, long res2) {
    int status;

    if (verbose > 2)
        fprintf(stderr, "%s uiocb %p status %ld %ld\n", __FUNCTION__, iocb, res, res2);

    /* fail on short write _OR_ fault */
    if (res != iocb->u.c.nbytes || res2 != 0)
        goto fail;

    /* get data we'll write to the host */
    iocb->u.c.nbytes = fill_in_buf(iocb->u.c.buf, iosize);
    if (iocb->u.c.nbytes < 0) {
        fprintf(stderr, "%s %p refill fail, %d (%s)\n", __FUNCTION__, iocb, errno, strerror(errno));
        goto clean;
    }

    /* resubmit */
resubmit:
    status = io_submit(ctx, 1, &iocb);
    if (status == 1)
        return;
    fprintf(stderr, "%s %p resubmit fail, %d (%s)\n", __FUNCTION__, iocb, errno, strerror(errno));
    goto clean;

fail:
    if (res < 0)
        errno = -res;
    else if (res2 < 0)
        errno = -res2;
    fprintf(stderr, "%s %p fail %ld/%ld, %d (%s)\n", __FUNCTION__, iocb, res, iocb->u.c.nbytes,
            errno, strerror(errno));
    goto resubmit;
clean:
    aio_in_pending--;
    return;
}

#define BUFFER_COUNT 2

static void *aio_in_thread(void *param) {
    char *name = (char *)param;
    int status;
    io_context_t ctx = 0;
    struct iocb *queue, *iocb[BUFFER_COUNT];
    char *buf[BUFFER_COUNT];
    unsigned i;
    int current_buffer = 0;

    status = iso_in_open(name);
    if (status < 0)
        return 0;
    iso_in_fd = status;
    pthread_cleanup_push(close_fd, &iso_in_fd);

    if (aio_in == 0) {
        aio_in = 1;
    }

    /* initialize i/o queue */
    status = io_setup(aio_in, &ctx);
    if (status < 0) {
        perror("aio_in_thread, io_setup");
        return 0;
    }
    pthread_cleanup_push(queue_release, &ctx);

    queue = alloca(aio_in * sizeof(struct iocb));

    /* allocate buffers */
    for (i = 0; i < BUFFER_COUNT; i++) {
        buf[i] = malloc(iosize);
        if (!buf[i]) {
            fprintf(stderr, "%s can't get buffer[%d]\n", __FUNCTION__, i);
            return 0;
        }
    }

    /* populate and (re)run the queue */
    for (i = 0; i < aio_in; i++) {
        current_buffer = i % BUFFER_COUNT;
        io_prep_pwrite(&queue[i], iso_in_fd, buf[current_buffer],
                       fill_in_buf(buf[current_buffer], iosize), 0);
        io_set_callback(&queue[i], in_complete);
        queue[i].key = USB_DIR_IN;

        struct iocb *iocb_list[] = {&queue[i]};
        status = io_submit(ctx, 1, iocb_list);
        if (status < 0) {
            perror(__FUNCTION__);
            break;
        }
        aio_in_pending++;
        if (verbose > 2)
            fprintf(stderr, "%s submit uiocb %p\n", __FUNCTION__, &queue[i]);
    }

    status = io_run(ctx, &aio_in_pending);
    if (status < 0)
        perror("aio_in_thread, io_run");

    /* clean up */
    for (i = 0; i < BUFFER_COUNT; i++) {
        free(buf[i]);
    }
    fflush(stderr);
    pthread_cleanup_pop(1);
    pthread_cleanup_pop(1);

    return 0;
}

static void out_complete(io_context_t ctx, struct iocb *iocb, long res, long res2) {
    int status;

    if (verbose > 2)
        fprintf(stderr, "%s uiocb %p status %ld %ld\n", __FUNCTION__, iocb, res, res2);

    /* fail on all errors we can see.  (short reads MAY mask faults.) */
    if (res < 0)
        goto fail;
    res = empty_out_buf(iocb->u.c.buf, res);
    if (res < 0)
        goto fail;

    /* resubmit */
resubmit:
    status = io_submit(ctx, 1, &iocb);
    if (status == 1)
        return;
    fprintf(stderr, "aio read %p resubmit fail, %d (%s)\n", iocb, errno, strerror(errno));
    goto clean;

fail:
    errno = -res;
    fprintf(stderr, "aio read %p fail, %d (%s)\n", iocb, errno, strerror(errno));
    goto resubmit;
clean:
    aio_out_pending--;
    return;
}

static pthread_mutex_t io_mutex = PTHREAD_MUTEX_INITIALIZER;

unsigned int calc_iosize() {
    u_int8_t mc = (hs_iso_in_desc.wMaxPacketSize & 0x1800) >> 11;
    u_int32_t mps = (mc == 0) ? hs_iso_in_desc.wMaxPacketSize : ((mc + 1) * 1024);
    return mps;
}

static void start_io(u_int8_t flag) {
    sigset_t allsig, oldsig;
#ifdef AIO
    /* iso uses the same API as bulk/interrupt.  we queue one
     * (u)frame's worth of data per i/o request, and the host
     * polls that queue once per interval.
     */
    switch (current_speed) {
    // dont need Full speed any more
    case USB_SPEED_FULL:
        fprintf(stderr, "current_speed is full speed\n");
        break;
    case USB_SPEED_HIGH:
        /* for iso, we updated bufsize earlier */
        iosize = calc_iosize();
        break;
    default:
        fprintf(stderr, "bogus link speed %d\n", current_speed);
        return;
    }
#endif /* AIO */

    iosize = calc_iosize();
    sigfillset(&allsig);
    errno = pthread_sigmask(SIG_SETMASK, &allsig, &oldsig);
    if (errno < 0) {
        perror("set thread signal mask");
        return;
    }

    pthread_mutex_lock(&io_mutex);

    if (!flag) {
        if (pthread_create(&bulk_in, 0, bulk_in_thread, (void *)EP_IN_NAME) != 0) {
            perror("can't create bulk_in thread");
            goto cleanup;
        } else {
            fprintf(stderr, "%s thread started...\n", EP_IN_NAME);
        }

        if (pthread_create(&bulk_out, 0, bulk_out_thread, (void *)EP_OUT_NAME) != 0) {
            perror("can't create bulk_out thread");
            pthread_cancel(bulk_in);
            bulk_in = ep0;
            goto cleanup;
        } else {
            fprintf(stderr, "%s thread started...\n", EP_OUT_NAME);
        }

    } else {
        if (pthread_create(&iso_in, 0, iso_in_thread, (void *)EP_ISO_IN_NAME) != 0) {
            perror("can't create iso_out thread");
            goto cleanup;
        } else {
            fprintf(stderr, "%s thread started...\n", EP_ISO_IN_NAME);
        }
    }

    /* give the other threads a chance to run before we report
     * success to the host.
     * FIXME better yet, use pthread_cond_timedwait() and
     * synchronize on ep config success.
     */
    sched_yield();

cleanup:
    errno = pthread_sigmask(SIG_SETMASK, &oldsig, 0);
    if (errno != 0) {
        perror("restore sigmask");
        exit(-1);
    }

    pthread_mutex_unlock(&io_mutex);
}

static void stop_io(u_int8_t flag) {
    pthread_mutex_lock(&io_mutex);
    if (!flag) {
        if (!pthread_equal(bulk_in, ep0)) {
            pthread_cancel(bulk_in);
            fprintf(stderr, "%s thread stopped...\n", EP_IN_NAME);
            if (pthread_join(bulk_in, 0) != 0)
                perror("can't join bulk_in thread");
            bulk_in = ep0;
        }

        if (!pthread_equal(bulk_out, ep0)) {
            pthread_cancel(bulk_out);
            fprintf(stderr, "%s thread stopped...\n", EP_OUT_NAME);
            if (pthread_join(bulk_out, 0) != 0)
                perror("can't join bulk_out thread");
            bulk_out = ep0;
        }
    } else {
        if (!pthread_equal(iso_in, ep0)) {
            pthread_cancel(iso_in);
            fprintf(stderr, "%s thread stopped...\n", EP_ISO_IN_NAME);
            if (pthread_join(iso_in, 0) != 0)
                perror("can't join bulk_out thread");
            iso_in = ep0;
        }
    }

    pthread_mutex_unlock(&io_mutex);
}
/*-------------------------------------------------------------------------*/

static char *build_config(char *cp, const struct usb_endpoint_descriptor **ep) {
    struct usb_config_descriptor *c;
    int i, j = 0;

    c = (struct usb_config_descriptor *)cp;

    memcpy(cp, &config, config.bLength);
    cp += config.bLength;
    interface0.iInterface = STRINGID_INTERFACE1;
    memcpy(cp, &interface0, interface0.bLength);
    cp += interface0.bLength;

    for (i = 0; i < interface0.bNumEndpoints; i++) {
        memcpy(cp, ep[i], USB_DT_ENDPOINT_SIZE);
        cp += USB_DT_ENDPOINT_SIZE;
    }

    // next intf
    interface0.bInterfaceNumber = 1;
    interface0.bNumEndpoints = 0;
    memcpy(cp, &interface0, interface0.bLength);
    cp += interface0.bLength;

    interface0.bInterfaceNumber = 1;
    interface0.bNumEndpoints = 1;
    interface0.bAlternateSetting = 1;
    memcpy(cp, &interface0, interface0.bLength);
    cp += interface0.bLength;

    j = i;
    for (i = 0; i < interface0.bNumEndpoints; i++) {
        memcpy(cp, ep[j], USB_DT_ENDPOINT_SIZE);
        cp += USB_DT_ENDPOINT_SIZE;
        j++;
    }

    interface0.bInterfaceNumber = 0;
    interface0.bNumEndpoints = 2;
    interface0.bAlternateSetting = 0;
    interface0.iInterface = STRINGID_INTERFACE0;

    c->wTotalLength = __cpu_to_le16(cp - (char *)c);
    return cp;
}

static int init_device(void) {
    char buf[4096], *cp = &buf[0];
    int fd;
    int status;
    status = autoconfig();
    if (status < 0) {
        fprintf(stderr, "?? don't recognize /dev/gadget %s device\n", iso ? "iso" : "bulk");
        return status;
    }

    fd = open(DEVNAME, O_RDWR);
    if (fd < 0) {
        perror(DEVNAME);
        return -errno;
    }

    *(__u32 *)cp = 0; /* tag for this format */
    cp += 4;

    /* write full then high speed configs */
    cp = build_config(cp, fs_eps);
    if (HIGHSPEED)
        cp = build_config(cp, hs_eps);

    /* and device descriptor at the end */
    memcpy(cp, &device_desc, sizeof device_desc);
    cp += sizeof device_desc;

    status = write(fd, &buf[0], cp - &buf[0]);
    if (status < 0) {
        perror("write dev descriptors");
        close(fd);
        return status;
    } else if (status != (cp - buf)) {
        fprintf(stderr, "dev init, wrote %d expected %d\n", status, cp - buf);
        close(fd);
        return -EIO;
    }
    return fd;
}

static void handle_control(int fd, struct usb_ctrlrequest *setup) {
    int status, tmp;
    __u8 buf[256];
    __u16 value, index, length;

    value = __le16_to_cpu(setup->wValue);
    index = __le16_to_cpu(setup->wIndex);
    length = __le16_to_cpu(setup->wLength);

    if (verbose)
        fprintf(stderr,
                "SETUP %02x.%02x "
                "v%04x i%04x %d\n",
                setup->bRequestType, setup->bRequest, value, index, length);

    /*
    if ((setup->bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
        goto special;
    */

    switch (setup->bRequest) { /* usb 2.0 spec ch9 requests */
    case USB_REQ_GET_DESCRIPTOR:
        fprintf(stderr, "GET DESC");
        if (setup->bRequestType != USB_DIR_IN)
            goto stall;
        switch (value >> 8) {
        case USB_DT_STRING:
            tmp = value & 0x0ff;
            if (verbose > 1)
                fprintf(stderr, "... get string %d lang %04x\n", tmp, index);
            if (tmp != 0 && index != strings.language)
                goto stall;
            status = usb_gadget_get_string(&strings, tmp, buf);
            if (status < 0)
                goto stall;
            tmp = status;
            if (length < tmp)
                tmp = length;
            status = write(fd, buf, tmp);
            if (status < 0) {
                if (errno == EIDRM)
                    fprintf(stderr, "string timeout\n");
                else
                    perror("write string data");
            } else if (status != tmp) {
                fprintf(stderr, "short string write, %d\n", status);
            }
            break;
        default:
            goto stall;
        }
        return;
    case USB_REQ_SET_CONFIGURATION:
        if (setup->bRequestType != USB_DIR_OUT)
            goto stall;
        if (verbose)
            fprintf(stderr, "CONFIG #%d\n", value);

        /* Kernel is normally waiting for us to finish reconfiguring
         * the device.
         *
         * Some hardware can't, notably older PXA2xx hardware.  (With
         * racey and restrictive config change automagic.  PXA 255 is
         * OK, most PXA 250s aren't.  If it has a UDC CFR register,
         * it can handle deferred response for SET_CONFIG.)  To handle
         * such hardware, don't write code this way ... instead, keep
         * the endpoints always active and don't rely on seeing any
         * config change events, either this or SET_INTERFACE.
         */
        switch (value) {
        case CONFIG_VALUE:
            start_io(0);
            break;
        case 0:
            stop_io(0);
            break;
        default:
            /* kernel bug -- "can't happen" */
            fprintf(stderr, "? illegal config\n");
            goto stall;
        }

        /* ... ack (a write would stall) */
        status = read(fd, &status, 0);
        if (status)
            perror("ack SET_CONFIGURATION");
        return;
    case USB_REQ_SET_INTERFACE:
        if (setup->bRequestType != USB_RECIP_INTERFACE) {
            goto stall;
        }
        fprintf(stderr, "SET INTERFACE i:%d, v:%d\n", setup->wIndex, value);
        if (value == 0) {
            fprintf(stderr, "Configuring for alternate setting 0\n");
            stop_io(1);
        } else if (value == 1) {
            fprintf(stderr, "Configuring for default alternate setting 1\n");
            start_io(1);
        } else {
            fprintf(stderr, "Unsupported alternate setting: %d\n", value);
            goto stall;
        }
        status = 0;
        status = read(fd, &status, 0);
        if (status) {
            perror("ack SET_INTERFACE");
        }
        return;
    case USB_REQ_CLEAR_FEATURE:

        status = 0;
        if (ioctl(bulk_in_fd, GADGETFS_CLEAR_HALT) < 0) {
            status = errno;
            perror("reset bulk_in fd");
        }

        if (ioctl(bulk_out_fd, GADGETFS_CLEAR_HALT) < 0) {
            status = errno;
            perror("reset bulk_out fd");
        }
        if (status) {
            goto stall;
        }

        status = read(fd, &status, 0);
        if (status) {
            perror("ack SET_INTERFACE");
        }
        return;
    default:
        goto stall;
    }

stall:
    if (verbose)
        fprintf(stderr, "... protocol stall %02x.%02x\n", setup->bRequestType, setup->bRequest);

    /* non-iso endpoints are stalled by issuing an i/o request
     * in the "wrong" direction.  ep0 is special only because
     * the direction isn't fixed.
     */
    if (setup->bRequestType & USB_DIR_IN)
        status = read(fd, &status, 0);
    else
        status = write(fd, &status, 0);
    if (status != -1)
        fprintf(stderr, "can't stall ep0 for %02x.%02x\n", setup->bRequestType, setup->bRequest);
    else if (errno != EL2HLT)
        perror("ep0 stall");
}

static void signothing(int sig, siginfo_t *info, void *ptr) {
    /* NOP */
    if (verbose > 2)
        fprintf(stderr, "%s %d\n", __FUNCTION__, sig);
}

static const char *speed(enum usb_device_speed s) {
    switch (s) {
    case USB_SPEED_LOW:
        return "low speed";
    case USB_SPEED_FULL:
        return "full speed";
    case USB_SPEED_HIGH:
        return "high speed";
    default:
        return "UNKNOWN speed";
    }
}

/*-------------------------------------------------------------------------*/

/* control thread, handles main event loop  */

#define NEVENT 5
#define LOGDELAY (15 * 60) /* seconds before stdout timestamp */

static void *ep0_thread(void *param) {
    int fd = *(int *)param;
    struct sigaction action;
    time_t now, last;
    struct pollfd ep0_poll;

    bulk_in = bulk_out = iso_in = ep0 = pthread_self();
    pthread_cleanup_push(close_fd, param);

    /* REVISIT signal handling ... normally one pthread should
     * be doing sigwait() to handle all async signals.
     */
    action.sa_sigaction = signothing;
    sigfillset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO;
    if (sigaction(SIGINT, &action, NULL) < 0) {
        perror("SIGINT");
        return 0;
    }
    if (sigaction(SIGQUIT, &action, NULL) < 0) {
        perror("SIGQUIT");
        return 0;
    }

    ep0_poll.fd = fd;
    ep0_poll.events = POLLIN | POLLOUT | POLLHUP;

    /* event loop */
    last = 0;
    for (;;) {
        int tmp;
        struct usb_gadgetfs_event event[NEVENT];
        int connected = 0;
        int i, nevent;

        /* Use poll() to test that mechanism, to generate
         * activity timestamps, and to make it easier to
         * tweak this code to work without pthreads.  When
         * AIO is needed without pthreads, ep0 can be driven
         * instead using SIGIO.
         */
        tmp = poll(&ep0_poll, 1, -1);
        if (verbose) {
            time(&now);
            if ((now - last) > LOGDELAY) {
                char timebuf[26];

                last = now;
                ctime_r(&now, timebuf);
                printf("\n** %s", timebuf);
            }
        }
        if (tmp < 0) {
            /* exit path includes EINTR exits */
            perror("poll");
            break;
        }

        tmp = read(fd, &event, sizeof event);
        if (tmp < 0) {
            if (errno == EAGAIN) {
                sleep(1);
                continue;
            }
            perror("ep0 read after poll");
            goto done;
        }
        nevent = tmp / sizeof event[0];
        if (nevent != 1 && verbose)
            fprintf(stderr, "read %d ep0 events\n", nevent);

        for (i = 0; i < nevent; i++) {
            switch (event[i].type) {
            case GADGETFS_NOP:
                if (verbose)
                    fprintf(stderr, "NOP\n");
                break;
            case GADGETFS_CONNECT:
                connected = 1;
                current_speed = event[i].u.speed;
                if (verbose)
                    fprintf(stderr, "CONNECT %s\n", speed(event[i].u.speed));
                break;
            case GADGETFS_SETUP:
                connected = 1;
                handle_control(fd, &event[i].u.setup);
                break;
            case GADGETFS_DISCONNECT:
                connected = 0;
                current_speed = USB_SPEED_UNKNOWN;
                if (verbose)
                    fprintf(stderr, "DISCONNECT\n");
                stop_io(0);
                break;
            case GADGETFS_SUSPEND:
                // connected = 1;
                if (verbose)
                    fprintf(stderr, "SUSPEND\n");
                break;
            default:
                fprintf(stderr, "* unhandled event %d\n", event[i].type);
            }
        }
        continue;
    done:
        fflush(stdout);
        if (connected)
            stop_io(0);
        break;
    }
    if (verbose)
        fprintf(stderr, "done\n");
    fflush(stdout);

    pthread_cleanup_pop(1);
    return 0;
}

/*-------------------------------------------------------------------------*/

int main(int argc, char **argv) {
    int fd, c, i;

    /* random initial serial number */
    srand((int)time(0));
    for (i = 0; i < sizeof serial - 1;) {
        c = rand() % 127;
        if ((('a' <= c && c <= 'z') || ('0' <= c && c <= '9')))
            serial[i++] = c;
    }

    bulk_in_thread = simple_in_thread;
    bulk_out_thread = simple_out_thread;
    iso_in_thread = aio_in_thread;

    while ((c = getopt(argc, argv, "I:a:i:o:p:r:s:v")) != EOF) {
        switch (c) {
        case 'I': /* ISO */
            iso = 1;
            /* interval is log2(period-in-frames); it's
             * ignored if high bandwidth could kick in
             */
            interval = atoi(optarg);
            continue;
        case 'a': /* aio IN/OUT qlen */
            aio_in = aio_out = atoi(optarg);
            continue;
        case 'i': /* aio IN qlen */
            aio_in = atoi(optarg);
            continue;
        case 'o': /* aio OUT qlen */
            aio_out = atoi(optarg);
            continue;
        case 's': /* iso buffer size */
            /* for iso, "-s 1025" and higher is high bandwidth */
            bufsize = atoi(optarg);
            continue;
        case 'p': /* i/o pattern */
            pattern = atoi(optarg);
            continue;
        case 'r': /* nonrandom serial */
            strncpy(serial, optarg, sizeof serial - 1);
            continue;
        case 'v': /* verbose */
            verbose++;
            continue;
        }
        fprintf(stderr,
                "usage:  %s "
#ifdef AIO
                "[-I interval] [-a num] [-s iso_size] "
#endif
                "[-p pattern] [-r serial] [-v]\n",
                argv[0]);
        return 1;
    }
    if (chdir("/dev/gadget") < 0) {
        perror("can't chdir /dev/gadget");
        return 1;
    }

    fd = init_device();
    if (fd < 0)
        return 1;
    fprintf(stderr, "/dev/gadget/%s ep0 configured\nserial=\"%s\"\n", DEVNAME, serial);
    fflush(stderr);
    (void)ep0_thread(&fd);
    return 0;
}
