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

/*!*********************************************************************
 *   hvperf.c
 *   Version     : V1.1.0
 *   Author      : usiop-vault
 *
 *********************************************************************!*/

#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadgetfs.h>

#include <fcntl.h>
#include <memory.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <errno.h>
#include <libaio.h>
#include <stdlib.h>

#include "usbstring.c"

#define FETCH(_var_)                                                                               \
    memcpy(cp, &_var_, _var_.bLength);                                                             \
    cp += _var_.bLength;

#define CONFIG_VALUE 2

// specific to controller
#define USB_DEV "/dev/gadget/dwc_otg_pcd"
#define EP_BULK_IN_NAME "/dev/gadget/ep1in"
#define EP_ISO_IN_NAME "/dev/gadget/ep2in"
// 128KB
#define BLOCK_SIZE 131072
// 60MB
#define TOTAL_SIZE 62914560

enum {
    STRINGID_MANUFACTURER = 1,
    STRINGID_PRODUCT,
    STRINGID_SERIAL,
    STRINGID_CONFIG_HS,
    STRINGID_CONFIG_LS,
    STRINGID_INTERFACE0,
    STRINGID_MAX
};

static struct usb_string stringtab[] = {
    {
        STRINGID_MANUFACTURER,
        "MyOwnGadget",
    },
    {
        STRINGID_PRODUCT,
        "Custom gadget",
    },
    {
        STRINGID_SERIAL,
        "0001",
    },
    {
        STRINGID_CONFIG_HS,
        "High speed configuration",
    },
    {
        STRINGID_CONFIG_LS,
        "Low speed configuration",
    },
    {
        STRINGID_INTERFACE0,
        "Interface",
    },
    {STRINGID_MAX, NULL},
};

static struct usb_gadget_strings strings = {
    .language = 0x0409, /* en-us */
    .strings = stringtab,
};

static struct usb_endpoint_descriptor bulk_in_desc;
static struct usb_endpoint_descriptor iso_in_desc;
/*
 * Respond to host requests
 */

static unsigned int verbose = 0;
static unsigned int iosize = 2;
static unsigned int aio_in_pending;
static unsigned int aio_in = 2;
static pthread_t bulk_in;
static pthread_t iso_in;
pthread_t ep0;
static int bulk_in_fd = -1;
static int iso_in_fd = -1;

static int ep_config(char *name, const char *label, struct usb_endpoint_descriptor *fs,
                     struct usb_endpoint_descriptor *hs) {
    int fd, status;
    char buf[2048];

    /* open and initialize with endpoint descriptor(s) */
    fd = open(name, O_RDWR);
    if (fd < 0) {
        status = -errno;
        fprintf(stderr, "%s open %s error %d (%s)\n", label, name, errno, strerror(errno));
        return status;
    }

    /* one (fs or ls) or two (fs + hs) sets of config_fs descriptors */
    *(__u32 *)buf = 1; /* tag for this format */
    memcpy(buf + 4, fs, USB_DT_ENDPOINT_SIZE);
    memcpy(buf + 4 + USB_DT_ENDPOINT_SIZE, hs, USB_DT_ENDPOINT_SIZE);
    status = write(fd, buf, 4 + USB_DT_ENDPOINT_SIZE + USB_DT_ENDPOINT_SIZE);

    if (status < 0) {
        status = -errno;
        fprintf(stderr, "%s config_fs %s error %d (%s)\n", label, name, errno, strerror(errno));
        close(fd);
        return status;
    } else if (verbose) {
        unsigned long id = pthread_self();
        fprintf(stderr, "%s start %ld fd %d\n", label, id, fd);
    }
    return fd;
}

static void *(*bulk_in_thread)(void *);
static void *(*iso_in_thread)(void *);

#define bulk_in_open(name) ep_config(name, __FUNCTION__, &bulk_in_desc, &bulk_in_desc)
#define iso_in_open(name) ep_config(name, __FUNCTION__, &iso_in_desc, &iso_in_desc)

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
    switch (0) {
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
    // fprintf(stderr, "%s %p fail %ld/%ld, %d (%s)\n", __FUNCTION__, iocb, res, iocb->u.c.nbytes,
    //         errno, strerror(errno));
    goto resubmit;
clean:
    aio_in_pending--;
    return;
}

static void *simple_in_thread(void *param) {
    char *name = (char *)param;
    int status;
    char buf[2048];

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

static void *aio_in_thread(void *param) {
    char *name = (char *)param;
    int status;
    io_context_t ctx = 0;
    struct iocb *queue, *iocb;
    unsigned i;

    status = iso_in_open(name);
    if (status < 0)
        return 0;
    iso_in_fd = status;
    pthread_cleanup_push(close_fd, &iso_in_fd);

    /* initialize i/o queue */
    status = io_setup(aio_in, &ctx);
    if (status < 0) {
        perror("aio_in_thread, io_setup");
        return 0;
    }
    pthread_cleanup_push(queue_release, &ctx);

    if (aio_in == 0)
        aio_in = 1;

    queue = malloc(aio_in * sizeof(*iocb));
    if (!queue) {
        perror("Unable to allocate memory for I/O queue");
        return NULL;
    }

    /* populate and (re)run the queue */
    for (i = 0, iocb = queue; i < aio_in; i++, iocb++) {
        char *buf = malloc(iosize);

        if (!buf) {
            fprintf(stderr, "%s can't get buffer[%d]\n", __FUNCTION__, i);
            return 0;
        }

        /* host receives the data we're writing */
        io_prep_pwrite(iocb, iso_in_fd, buf, fill_in_buf(buf, iosize), 0);
        io_set_callback(iocb, in_complete);
        iocb->key = USB_DIR_IN;

        status = io_submit(ctx, 1, &iocb);
        if (status < 0) {
            perror(__FUNCTION__);
            break;
        }
        aio_in_pending++;
        if (verbose > 2)
            fprintf(stderr, "%s submit uiocb %p\n", __FUNCTION__, iocb);
    }

    status = io_run(ctx, &aio_in_pending);
    if (status < 0)
        perror("aio_in_thread, io_run");

    /* clean up */
    fflush(stderr);
    pthread_cleanup_pop(1);
    pthread_cleanup_pop(1);

    return 0;
}

static pthread_mutex_t io_mutex = PTHREAD_MUTEX_INITIALIZER;

static void start_io() {
    sigset_t allsig, oldsig;

    pthread_mutex_lock(&io_mutex);

    sigfillset(&allsig);
    errno = pthread_sigmask(SIG_SETMASK, &allsig, &oldsig);
    if (errno < 0) {
        perror("set thread signal mask");
        return;
    }

    if (pthread_create(&bulk_in, NULL, bulk_in_thread, (void *)EP_BULK_IN_NAME) != 0) {
        perror("can't create bulk_in thread");
        goto cleanup;
    } else {
        fprintf(stderr, "%s thread started...\n", EP_BULK_IN_NAME);
    }

    // if (pthread_create(&iso_in, NULL, iso_in_thread, (void *)EP_ISO_IN_NAME) != 0) {
    //     perror("can't create iso_in thread");
    //     pthread_cancel(bulk_in);
    //     bulk_in = ep0;
    //     goto cleanup;
    // } else {
    //     fprintf(stderr, "%s thread started...\n", EP_ISO_IN_NAME);
    // }

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

static void stop_io() {
    pthread_mutex_lock(&io_mutex);

    if (!pthread_equal(bulk_in, ep0)) {
        pthread_cancel(bulk_in);
        fprintf(stderr, "%s thread stopped...\n", EP_BULK_IN_NAME);
        if (pthread_join(bulk_in, NULL) != 0)
            perror("can't join bulk_in thread");
        bulk_in = ep0;
    }

    // if (!pthread_equal(iso_in, ep0)) {
    //     pthread_cancel(iso_in);
    //     fprintf(stderr, "%s thread stopped...\n", EP_ISO_IN_NAME);
    //     if (pthread_join(iso_in, NULL) != 0)
    //         perror("can't join iso_in thread");
    //     iso_in = ep0;
    // }

    pthread_mutex_unlock(&io_mutex);
}

static void handle_setup_request(int fd, struct usb_ctrlrequest *setup) {
    int status;
    uint8_t buffer[512];
    sigset_t allsig, oldsig;

    printf("Setup request %d\n", setup->bRequest);

    switch (setup->bRequest) {
    case USB_REQ_GET_DESCRIPTOR:
        if (setup->bRequestType != USB_DIR_IN)
            goto stall;
        switch (setup->wValue >> 8) {
        case USB_DT_STRING:
            printf("Get string id #%d (max length %d)\n", setup->wValue & 0xff, setup->wLength);
            status = usb_gadget_get_string(&strings, setup->wValue & 0xff, buffer);
            // Error
            if (status < 0) {
                printf("String not found !!\n");
                break;
            } else {
                printf("Found %d bytes\n", status);
            }
            write(fd, buffer, status);
            return;
        default:
            printf("Cannot return descriptor %d\n", (setup->wValue >> 8));
        }
        break;
    case USB_REQ_SET_CONFIGURATION:
        if (setup->bRequestType != USB_DIR_OUT) {
            printf("Bad dir\n");
            goto stall;
        }
        switch (setup->wValue) {
        case CONFIG_VALUE:
            printf("Set config_fs value\n");
            start_io();
            break;
        case 0:
            printf("Disable threads\n");
            stop_io();
            break;
        default:
            printf("Unhandled configuration value %d\n", setup->wValue);
            break;
        }

        // Just ACK
        status = read(fd, &status, 0);
        return;
    case USB_REQ_GET_INTERFACE:
        printf("GET_INTERFACE\n");
        buffer[0] = 0;
        write(fd, buffer, 1);
        return;
    case USB_REQ_SET_INTERFACE:
        printf("SET_INTERFACE\n");
        ioctl(bulk_in_fd, GADGETFS_CLEAR_HALT);
        ioctl(iso_in_fd, GADGETFS_CLEAR_HALT);
        // ACK
        status = read(fd, &status, 0);

        if (status) {
            perror("ack SET_INTERFACE");
        }

        if (setup->wValue == 0) {
            fprintf(stderr, "Configuring for alternate setting 0\n");
            // stop_io(1);
        } else if (setup->wValue == 1) {
            fprintf(stderr, "Configuring for alternate setting 1\n");
            // start_io(1);
        } else {
            fprintf(stderr, "Unsupported alternate setting: %d\n", setup->wValue);
            goto stall;
        }
        return;
    }

stall:
    printf("Stalled\n");
    // Error
    if (setup->bRequestType & USB_DIR_IN)
        read(fd, &status, 0);
    else
        write(fd, &status, 0);
}

static void handle_ep0(int fd) {
    int ret, nevents, i;
    fd_set read_set;
    struct usb_gadgetfs_event events[5];

    bulk_in = iso_in = ep0 = pthread_self();

    while (1) {
        FD_ZERO(&read_set);
        FD_SET(fd, &read_set);

        select(fd + 1, &read_set, NULL, NULL, NULL);

        ret = read(fd, &events, sizeof(events));

        if (ret < 0) {
            printf("Read error %d (%m)\n", ret);
            goto end;
        }

        nevents = ret / sizeof(events[0]);

        printf("%d event(s)\n", nevents);

        for (i = 0; i < nevents; i++) {
            switch (events[i].type) {
            case GADGETFS_CONNECT:
                printf("EP0 CONNECT\n");
                break;
            case GADGETFS_DISCONNECT:
                printf("EP0 DISCONNECT\n");
                break;
            case GADGETFS_SETUP:
                printf("EP0 SETUP\n");
                handle_setup_request(fd, &events[i].u.setup);
                break;
            case GADGETFS_NOP:
            case GADGETFS_SUSPEND:
                break;
            }
        }
    }

end:
    return;
}

int main() {
    int fd = -1, ret, err = -1;
    uint32_t send_size;
    struct usb_config_descriptor config_fs;
    struct usb_config_descriptor config_hs;
    struct usb_device_descriptor device_descriptor;
    struct usb_interface_descriptor intf_desc;
    uint8_t init_config[2048];
    uint8_t *cp;

    bulk_in_thread = simple_in_thread;
    // iso_in_thread = aio_in_thread;

    fd = open(USB_DEV, O_RDWR | O_SYNC);

    if (fd <= 0) {
        printf("Unable to open %s (%m)\n", USB_DEV);
        return 1;
    }

    printf("Start init\n");

    *(uint32_t *)init_config = 0;
    cp = &init_config[4];

    device_descriptor.bLength = USB_DT_DEVICE_SIZE;
    device_descriptor.bDescriptorType = USB_DT_DEVICE;
    device_descriptor.bDeviceClass = USB_CLASS_COMM;
    device_descriptor.bDeviceSubClass = 0;
    device_descriptor.bDeviceProtocol = 0;
    // device_descriptor.bMaxPacketSize0 = 255; Set by driver
    device_descriptor.idVendor = 0xAA;    // My own id
    device_descriptor.idProduct = 0xBB;   // My own id
    device_descriptor.bcdDevice = 0x0200; // Version
    // Strings
    device_descriptor.iManufacturer = STRINGID_MANUFACTURER;
    device_descriptor.iProduct = STRINGID_PRODUCT;
    device_descriptor.iSerialNumber = STRINGID_SERIAL;
    device_descriptor.bNumConfigurations = 1; // Only one configuration

    bulk_in_desc.bLength = USB_DT_ENDPOINT_SIZE;
    bulk_in_desc.bDescriptorType = USB_DT_ENDPOINT;
    bulk_in_desc.bEndpointAddress = USB_DIR_IN | 1;
    bulk_in_desc.bmAttributes = USB_ENDPOINT_XFER_BULK;
    bulk_in_desc.wMaxPacketSize = 512; // HS size

    iso_in_desc.bLength = USB_DT_ENDPOINT_SIZE;
    iso_in_desc.bDescriptorType = USB_DT_ENDPOINT;
    iso_in_desc.bEndpointAddress = USB_DIR_IN | 2;
    iso_in_desc.bmAttributes = USB_ENDPOINT_XFER_ISOC;
    iso_in_desc.wMaxPacketSize = 100; // HS size

    intf_desc.bLength = sizeof(intf_desc);
    intf_desc.bDescriptorType = USB_DT_INTERFACE;
    intf_desc.bInterfaceNumber = 0;
    intf_desc.bAlternateSetting = 0;
    intf_desc.bNumEndpoints = 2;
    intf_desc.bInterfaceClass = USB_CLASS_COMM;
    intf_desc.bInterfaceSubClass = 0;
    intf_desc.bInterfaceProtocol = 0;
    intf_desc.iInterface = STRINGID_INTERFACE0;

    config_hs.bLength = sizeof(config_hs);
    config_hs.bDescriptorType = USB_DT_CONFIG;
    config_hs.wTotalLength =
        config_hs.bLength + intf_desc.bLength + bulk_in_desc.bLength + iso_in_desc.bLength;
    config_hs.bNumInterfaces = 1;
    config_hs.bConfigurationValue = CONFIG_VALUE;
    config_hs.iConfiguration = STRINGID_CONFIG_HS;
    config_hs.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER;
    config_hs.bMaxPower = 1;

    config_fs.bLength = sizeof(config_fs);
    config_fs.bDescriptorType = USB_DT_CONFIG;
    config_fs.wTotalLength =
        config_fs.bLength + intf_desc.bLength + bulk_in_desc.bLength + iso_in_desc.bLength;
    config_fs.bNumInterfaces = 1;
    config_fs.bConfigurationValue = CONFIG_VALUE;
    config_fs.iConfiguration = STRINGID_CONFIG_LS;
    config_fs.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER;
    config_fs.bMaxPower = 1;

    FETCH(config_fs);
    FETCH(intf_desc);
    FETCH(bulk_in_desc);
    FETCH(iso_in_desc);

    FETCH(config_hs);
    FETCH(intf_desc);
    FETCH(bulk_in_desc);
    FETCH(iso_in_desc);

    FETCH(device_descriptor);

    // Configure ep0
    send_size = (uint32_t)cp - (uint32_t)init_config;
    ret = write(fd, init_config, send_size);

    if (ret != send_size) {
        printf("Write error %d (%m)\n", ret);
        goto end;
    }

    printf("ep0 configured\n");

    handle_ep0(fd);

end:
    if (fd != -1)
        close(fd);

    return err;
}
