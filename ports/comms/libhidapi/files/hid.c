/*******************************************************
 HIDAPI - Multi-Platform library for
 communication with HID devices.

 James Buren
 libusb/hidapi Team

 Copyright 2023, All Rights Reserved.

 At the discretion of the user of this library,
 this software may be licensed under the terms of the
 GNU General Public License v3, a BSD-Style license, or the
 original HIDAPI license as outlined in the LICENSE.txt,
 LICENSE-gpl3.txt, LICENSE-bsd.txt, and LICENSE-orig.txt
 files located at the root of the source distribution.
 These files may also be found in the public source
 code repository located at:
        https://github.com/libusb/hidapi .
********************************************************/

/* C */
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <locale.h>
#include <ctype.h>
#include <errno.h>

/* Unix */
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

/* OpenBSD */
#include <dev/usb/usb.h>
#include <dev/usb/usbhid.h>

#include "hidapi.h"

#define UHID_MAX_DEVICES 64
#define UHID_MAX_REPORTS 256

struct hid_device_ {
	int device_handle;
	int blocking;
	wchar_t *last_error_str;
	wchar_t *last_read_error_str;
	struct hid_device_info *device_info;
	size_t poll_handles_length;
	struct pollfd poll_handles[UHID_MAX_REPORTS];
	int report_handles[UHID_MAX_REPORTS];
	char path[USB_MAX_DEVNAMELEN];
};

struct hid_enumerate_data {
	struct hid_device_info *root;
	struct hid_device_info *end;
	uint16_t vendor_id;
	uint16_t product_id;
};

typedef void (*enumerate_devices_callback)
    (const struct usb_device_info *, void *);

static wchar_t *last_global_error_str = NULL;

/* The caller must free the returned string with free(). */
static wchar_t *utf8_to_wchar_t(const char *utf8)
{
	wchar_t *ret = NULL;
	if (utf8) {
		size_t wlen = mbstowcs(NULL, utf8, 0);
		if ((size_t) -1 == wlen) {
			return wcsdup(L"");
		}
		ret = calloc(wlen+1, sizeof(wchar_t));
		if (ret == NULL) {
			/* as much as we can do at this point */
			return NULL;
		}
		mbstowcs(ret, utf8, wlen+1);
		ret[wlen] = 0x0000;
	}
	return ret;
}

/* Makes a copy of the given error message (and decoded according to the
 * currently locale) into the wide string pointer pointed by error_str.
 * The last stored error string is freed.
 * Use register_error_str(NULL) to free the error message completely. */
static void register_error_str(wchar_t **error_str, const char *msg)
{
	free(*error_str);
	*error_str = utf8_to_wchar_t(msg);
}

/* Semilar to register_error_str, but allows passing a format string with
 * va_list args into this function. */
static void register_error_str_vformat(wchar_t **error_str, const char *format,
    va_list args)
{
	char msg[256];
	vsnprintf(msg, sizeof(msg), format, args);
	register_error_str(error_str, msg);
}

/* Set the last global error to be reported by hid_error(NULL).
 * The given error message will be copied (and decoded according to the
 * currently locale, so do not pass in string constants).
 * The last stored global error message is freed.
 * Use register_global_error(NULL) to indicate "no error". */
static void register_global_error(const char *msg)
{
	register_error_str(&last_global_error_str, msg);
}

/* Similar to register_global_error, but allows passing a format string into this function. */
static void register_global_error_format(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	register_error_str_vformat(&last_global_error_str, format, args);
	va_end(args);
}

/* Set the last error for a device to be reported by hid_error(dev).
 * The given error message will be copied (and decoded according to the
 * currently locale, so do not pass in string constants).
 * The last stored device error message is freed.
 * Use register_device_error(dev, NULL) to indicate "no error". */
static void register_device_error(hid_device *dev, const char *msg)
{
	register_error_str(&dev->last_error_str, msg);
}

/* Similar to register_device_error, but you can pass a format string. */
static void register_device_error_format(hid_device *dev,
    const char *format, ...)
{
	va_list args;
	va_start(args, format);
	register_error_str_vformat(&dev->last_error_str, format, args);
	va_end(args);
}

static void register_device_read_error(hid_device *dev, const char *msg)
{
	register_error_str(&dev->last_read_error_str, msg);
}

static void register_device_read_error_format(hid_device *dev, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	register_error_str_vformat(&dev->last_read_error_str, format, args);
	va_end(args);
}

/*
 * Gets the size of the HID item at the given position
 * Returns 1 if successful, 0 if an invalid key
 * Sets data_len and key_size when successful
 */
static int get_hid_item_size(const uint8_t *report_descriptor, uint32_t size,
    unsigned int pos, int *data_len, int *key_size)
{
	int key = report_descriptor[pos];
	int size_code;

	/*
	 * This is a Long Item. The next byte contains the
	 * length of the data section (value) for this key.
	 * See the HID specification, version 1.11, section
	 * 6.2.2.3, titled "Long Items."
	 */
	if ((key & 0xf0) == 0xf0) {
		if (pos + 1 < size)
		{
			*data_len = report_descriptor[pos + 1];
			*key_size = 3;
			return 1;
		}
		*data_len = 0; /* malformed report */
		*key_size = 0;
	}

	/*
	 * This is a Short Item. The bottom two bits of the
	 * key contain the size code for the data section
	 * (value) for this key. Refer to the HID
	 * specification, version 1.11, section 6.2.2.2,
	 * titled "Short Items."
	 */
	size_code = key & 0x3;
	switch (size_code) {
	case 0:
	case 1:
	case 2:
		*data_len = size_code;
		*key_size = 1;
		return 1;
	case 3:
		*data_len = 4;
		*key_size = 1;
		return 1;
	default:
		/* Can't ever happen since size_code is & 0x3 */
		*data_len = 0;
		*key_size = 0;
		break;
	};

	/* malformed report */
	return 0;
}

/*
 * Get bytes from a HID Report Descriptor.
 * Only call with a num_bytes of 0, 1, 2, or 4.
 */
static uint32_t get_hid_report_bytes(const uint8_t *rpt, size_t len,
    size_t num_bytes, size_t cur)
{
	/* Return if there aren't enough bytes. */
	if (cur + num_bytes >= len)
		return 0;

	if (num_bytes == 0)
		return 0;
	else if (num_bytes == 1)
		return rpt[cur + 1];
	else if (num_bytes == 2)
		return (rpt[cur + 2] * 256 + rpt[cur + 1]);
	else if (num_bytes == 4)
		return (
			rpt[cur + 4] * 0x01000000 +
			rpt[cur + 3] * 0x00010000 +
			rpt[cur + 2] * 0x00000100 +
			rpt[cur + 1] * 0x00000001
		);
	else
		return 0;
}

/*
 * Iterates until the end of a Collection.
 * Assumes that *pos is exactly at the beginning of a Collection.
 * Skips all nested Collection, iterates until the end of current level.
 *
 * The return value is non-0 when an end of current Collection is found,
 * 0 when error is occured (broken Descriptor, end of a Collection is found
 * before its begin, or no Collection is found at all).
 */
static int hid_iterate_over_collection(const uint8_t *report_descriptor,
    uint32_t size, unsigned int *pos, int *data_len, int *key_size)
{
	int collection_level = 0;

	while (*pos < size) {
		int key = report_descriptor[*pos];
		int key_cmd = key & 0xfc;

		/* Determine data_len and key_size */
		if (!get_hid_item_size(report_descriptor, size, *pos, data_len,
		    key_size))
			return 0; /* malformed report */

		switch (key_cmd) {
		case 0xa0: /* Collection 6.2.2.4 (Main) */
			collection_level++;
			break;
		case 0xc0: /* End Collection 6.2.2.4 (Main) */
			collection_level--;
			break;
		}

		if (collection_level < 0) {
			/* Broken descriptor or wrong usage of this function,
			 * should be called exactly at the collection start */
			return 0;
		}

		if (collection_level == 0) {
			/* Found it!
			 * Also possible when called not at the collection
			 * start, but should not happen if used correctly */
			return 1;
		}

		*pos += *data_len + *key_size;
	}

	return 0; /* Did not find the end of a Collection */
}

struct hid_usage_iterator {
	unsigned int pos;
	int usage_page_found;
	unsigned short usage_page;
};

/*
 * Retrieves the device's Usage Page and Usage from the report descriptor.
 * The algorithm returns the current Usage Page/Usage pair whenever a new
 * Collection is found and a Usage Local Item is currently in scope.
 * Usage Local Items are consumed by each Main Item (See. 6.2.2.8).
 * The algorithm should give similar results as Apple's:
 *   https://developer.apple.com/documentation/iokit/kiohiddeviceusagepairskey?language=objc
 * Physical Collections are also matched (macOS does the same).
 *
 * This function can be called repeatedly until it returns non-0
 * Usage is found. pos is the starting point (initially 0) and will be updated
 * to the next search position.
 *
 * The return value is 0 when a pair is found.
 * 1 when finished processing descriptor.
 * -1 on a malformed report.
 */
static int get_next_hid_usage(const uint8_t *report_descriptor, uint32_t size,
    struct hid_usage_iterator *ctx, unsigned short *usage_page,
    unsigned short *usage)
{
	int data_len, key_size;
	/* Handle case where no top-level application collection is defined */
	int initial = ctx->pos == 0; 

	int usage_found = 0;

	while (ctx->pos < size) {
		int key = report_descriptor[ctx->pos];
		int key_cmd = key & 0xfc;

		/* Determine data_len and key_size */
		if (!get_hid_item_size(report_descriptor, size, ctx->pos,
		    &data_len, &key_size))
			return -1; /* malformed report */

		switch (key_cmd) {
		case 0x4: /* Usage Page 6.2.2.7 (Global) */
			ctx->usage_page = get_hid_report_bytes(
			    report_descriptor, size, data_len, ctx->pos);
			ctx->usage_page_found = 1;
			break;

		case 0x8: /* Usage 6.2.2.8 (Local) */
			if (data_len == 4) {
				/* Usages 5.5 / Usage Page 6.2.2.7 */
				ctx->usage_page = get_hid_report_bytes(
				    report_descriptor, size, 2, ctx->pos + 2);
				ctx->usage_page_found = 1;
				*usage = get_hid_report_bytes(
				    report_descriptor, size, 2, ctx->pos);
				usage_found = 1;
			}
			else {
				*usage = get_hid_report_bytes(
				    report_descriptor, size, data_len,
				    ctx->pos);
				usage_found = 1;
			}
			break;

		case 0xa0: /* Collection 6.2.2.4 (Main) */
			if (!hid_iterate_over_collection(report_descriptor,
			    size, &ctx->pos, &data_len, &key_size)) {
				return -1;
			}

			/* Valid pair, to report when Collection is found */
			if (usage_found && ctx->usage_page_found) {
				*usage_page = ctx->usage_page;
				return 0;
			}

			break;
		}

		/* Skip over this key and its associated data */
		ctx->pos += data_len + key_size;
	}

	/* If no top-level application collection is found and usage page/usage
	 * pair is found, pair is valid
	 * https://docs.microsoft.com/en-us/windows-hardware/drivers/hid/top-level-collections */
	if (initial && usage_found && ctx->usage_page_found) {
			*usage_page = ctx->usage_page;
			return 0; /* success */
	}

	return 1; /* finished processing */
}

static struct hid_device_info *create_device_info(
    const struct usb_device_info *udi, int ifaceno, const char *path,
    const struct usb_ctl_report_desc *ucrd)
{
	struct hid_device_info *root;
	struct hid_device_info *end;

	root = calloc(1, sizeof(struct hid_device_info));
	if (!root)
		return NULL;

	end = root;

	/* Path */
	end->path = (path) ? strdup(path) : NULL;

	/* Vendor Id */
	end->vendor_id = udi->udi_vendorNo;

	/* Product Id */
	end->product_id = udi->udi_productNo;

	/* Serial Number */
	end->serial_number = utf8_to_wchar_t(udi->udi_serial);

	/* Release Number */
	end->release_number = udi->udi_releaseNo;

	/* Manufacturer String */
	end->manufacturer_string = utf8_to_wchar_t(udi->udi_vendor);

	/* Product String */
	end->product_string = utf8_to_wchar_t(udi->udi_product);

	/* Usage Page */
	end->usage_page = 0;

	/* Usage */
	end->usage = 0;

	/* Interface Number */
	end->interface_number = ifaceno;

	/* Next Device Info */
	end->next = NULL;

	/* Bus Type */
	end->bus_type = HID_API_BUS_USB;

	if (ucrd) {
		uint16_t page;
		uint16_t usage;
		struct hid_usage_iterator usage_iterator;

		page = usage = 0;
		memset(&usage_iterator, 0, sizeof(usage_iterator));

		/*
		 * Parse the first usage and usage page
		 * out of the report descriptor.
		 */
		if (get_next_hid_usage(ucrd->ucrd_data, ucrd->ucrd_size,
		    &usage_iterator, &page, &usage) == 0) {
			end->usage_page = page;
			end->usage = usage;
		}

		/*
		 * Parse any additional usage and usage pages
		 * out of the report descriptor.
		 */
		while (get_next_hid_usage(ucrd->ucrd_data, ucrd->ucrd_size,
		    &usage_iterator, &page, &usage) == 0) {
			/* Create new record for additional usage pairs */
			struct hid_device_info *node = calloc(1,
			    sizeof(struct hid_device_info));

			if (!node)
				continue;

			/* Update fields */
			node->path = (end->path) ? strdup(end->path) : NULL;
			node->vendor_id = end->vendor_id;
			node->product_id = end->product_id;
			node->serial_number = (
			    (end->serial_number) ?
			    wcsdup(end->serial_number) : NULL);
			node->release_number = end->release_number;
			node->manufacturer_string = (
			    (end->manufacturer_string) ?
			    wcsdup(end->manufacturer_string) : NULL);
			node->product_string = (
			    (end->product_string) ?
			    wcsdup(end->product_string) : NULL);
			node->usage_page = page;
			node->usage = usage;
			node->interface_number = end->interface_number;
			node->next = NULL;
			node->bus_type = end->bus_type;

			/* Insert node */
			end->next = node;
			end = node;
		}
	}

	return root;
}

static int set_report(hid_device *dev, const uint8_t *data, size_t length,
    int report)
{
	int device_handle;
	struct usb_ctl_report ucr;

	if (length < 1) {
		register_device_error(dev,
		    "report must be greater than 1 byte");
		return -1;
	}

	device_handle = dev->report_handles[*data];
	if (device_handle < 0) {
		register_device_error_format(dev,
		    "unsupported report id: %hhu", *data);
		return -1;
	}

	length--;
	data++;

	if (length > sizeof(ucr.ucr_data)) {
		register_device_error_format(dev,
		    "report must be less than or equal to %zu bytes",
		    sizeof(ucr.ucr_data));
		return -1;
	}

	ucr.ucr_report = report;
	memcpy(ucr.ucr_data, data, length);

	if (ioctl(device_handle, USB_SET_REPORT, &ucr) == -1) {
		register_device_error_format(dev,
		    "ioctl(USB_SET_REPORT): %s", strerror(errno));
		return -1;
	}

	return (int) (length + 1);
}

static int get_report(hid_device *dev, uint8_t *data, size_t length, int report)
{
	int device_handle;
	struct usb_ctl_report ucr;

	if (length < 1) {
		register_device_error(dev,
		    "report must be greater than 1 byte");
		return -1;
	}

	device_handle = dev->report_handles[*data];
	if (device_handle < 0) {
		register_device_error_format(dev,
		    "unsupported report id: %hhu", *data);
		return -1;
	}

	length--;
	data++;

	if (length > sizeof(ucr.ucr_data)) {
		length = sizeof(ucr.ucr_data);
	}

	ucr.ucr_report = report;

	if (ioctl(device_handle, USB_GET_REPORT, &ucr) == -1) {
		register_device_error_format(dev,
		    "ioctl(USB_GET_REPORT): %s", strerror(errno));
		return -1;
	}

	memcpy(data, ucr.ucr_data, length);
	return (int) (length + 1);
}

int HID_API_EXPORT HID_API_CALL hidapi_hid_init(void)
{
	/* indicate no error */
	register_global_error(NULL);

	return 0;
}

int HID_API_EXPORT HID_API_CALL hid_exit(void)
{
	/* Free global error message */
	register_global_error(NULL);

	return 0;
}

struct hid_device_info HID_API_EXPORT * HID_API_CALL hid_enumerate(
    unsigned short vendor_id, unsigned short product_id)
{
	int i, uhid, use_ucrd;
	char * devpath;
	struct hid_enumerate_data hed;
	int lbus = -1, laddr = -1, lifaceno = -1;
	int ifaceno;
	struct usb_device_info udi;
	struct usb_ctl_report_desc ucrd;
	struct hid_device_info *node;

	if (hidapi_hid_init() == -1)
		return NULL;

	hed.root = NULL;
	hed.end = NULL;

	for (i=0; i < UHID_MAX_DEVICES; i++) {
		asprintf(&devpath, "/dev/uhid%d", i);
		uhid = open(devpath, O_RDONLY | O_CLOEXEC);
		if (uhid < 0) {
			free(devpath);
			continue;
		}

		if(ioctl(uhid, USB_GET_DEVICEINFO, &udi) == -1) {
			register_global_error_format(
		    		"%s: ioctl(USB_GET_DEVICEINFO): %s", devpath,
				strerror(errno)
			);
			close(uhid);
			continue;
		}
		if(ioctl(uhid, USB_GET_INTERFACE_ID, &ifaceno) == -1) {
			register_global_error_format(
		    		"%s: ioctl(USB_GET_INTERFACE_ID): %s", devpath,
				strerror(errno)
			);
			close(uhid);
			continue;
		}

		if (lbus == udi.udi_bus && laddr == udi.udi_addr &&
		    lifaceno == ifaceno) {
			close(uhid);
			continue;
		}
		lbus = udi.udi_bus;
		laddr = udi.udi_addr;
		lifaceno = ifaceno;

		if (vendor_id && vendor_id != udi.udi_vendorNo) {
			close(uhid);
			continue;
		}
		if (product_id && product_id != udi.udi_productNo) {
			close(uhid);
			continue;
		}

		ucrd.ucrd_size = 0;
		use_ucrd = ioctl(uhid, USB_GET_REPORT_DESC, &ucrd);
		close(uhid);

		node = create_device_info(&udi, ifaceno, devpath,
		    (use_ucrd != -1) ? &ucrd : NULL);
		if (!node)
			continue;
		if (!hed.root) {
			hed.root = node;
			hed.end = node;
		} else {
			hed.end->next = node;
			hed.end = node;
		}
		while (hed.end->next)
			hed.end = hed.end->next;
	}
	return hed.root;
}

void HID_API_EXPORT HID_API_CALL hid_free_enumeration(
    struct hid_device_info *devs)
{
	while (devs) {
		struct hid_device_info *next = devs->next;
		free(devs->path);
		free(devs->serial_number);
		free(devs->manufacturer_string);
		free(devs->product_string);
		free(devs);
		devs = next;
	}
}

HID_API_EXPORT hid_device * HID_API_CALL hid_open(unsigned short vendor_id,
    unsigned short product_id, const wchar_t *serial_number)
{
	struct hid_device_info *devs;
	struct hid_device_info *dev;
	char path[USB_MAX_DEVNAMELEN];

	devs = hid_enumerate(vendor_id, product_id);
	if (!devs)
		return NULL;

	*path = '\0';

	for (dev = devs; dev; dev = dev->next) {
		if (dev->vendor_id != vendor_id)
			continue;

		if (dev->product_id != product_id)
			continue;

		if (serial_number && wcscmp(dev->serial_number, serial_number))
			continue;

		strlcpy(path, dev->path, sizeof(path));

		break;
	}

	hid_free_enumeration(devs);

	if (*path == '\0') {
		register_global_error(
		    "Device with requested VID/PID/(SerialNumber) not found");
		return NULL;
	}

	return hid_open_path(path);
}

HID_API_EXPORT hid_device * HID_API_CALL hid_open_path(const char *path)
{
	int i, uhid, refifaceno, ifaceno;
	struct usb_device_info refudi, udi;
	hid_device *dev;
	char *child_dev;
	int rep_id;
	struct pollfd *ph;

	if (hidapi_hid_init() == -1)
		return NULL;

	dev = calloc(1, sizeof(hid_device));
	if (!dev) {
		register_global_error_format(
		    "%s: could not allocate hid_device: %s", path,
		    strerror(errno));
		return NULL;
	}
	dev->poll_handles_length = 0;
	memset(dev->poll_handles, 0x00, sizeof(dev->poll_handles));
	memset(dev->report_handles, 0xff, sizeof(dev->report_handles));

	uhid = open(path, O_RDONLY | O_CLOEXEC);
	if (uhid < 0) {
		register_global_error_format(
		    "%s: can't open reference hid_device: %s", path,
		    strerror(errno));
		goto err;
	}

	if (ioctl(uhid, USB_GET_DEVICEINFO, &refudi) == -1) {
		register_global_error_format(
		    "%s: ioctl(USB_GET_DEVICEINFO): %s", path,
		    strerror(errno));
		close(uhid);
		goto err;
	}
	if (ioctl(uhid, USB_GET_INTERFACE_ID, &refifaceno) == -1) {
		register_global_error_format(
		    "%s: ioctl(USB_GET_INTERFACE_ID): %s", path,
		    strerror(errno));
		close(uhid);
		goto err;
	}
	close(uhid);

	for (i = 0; i < UHID_MAX_DEVICES; i++) {
		asprintf(&child_dev, "/dev/uhid%d", i);

		uhid = open(child_dev, O_RDWR | O_CLOEXEC);
		if (uhid == -1) {
			continue;
		}

		if (ioctl(uhid, USB_GET_DEVICEINFO, &udi) == -1) {
			register_global_error_format(
				"%s: ioctl(USB_GET_DEVICEINFO): %s", path,
				strerror(errno));
			close(uhid);
			continue;
		}
		if(ioctl(uhid, USB_GET_INTERFACE_ID, &ifaceno) == -1) {
			register_global_error_format(
				"%s: ioctl(USB_GET_INTERFACE_ID): %s", path,
				strerror(errno));
			close(uhid);
			continue;
		}

		if (refudi.udi_bus != udi.udi_bus ||
		    refudi.udi_addr != udi.udi_addr ||
		    refifaceno != ifaceno) {
			close(uhid);
			continue;
		}

		if (ioctl(uhid, USB_GET_REPORT_ID, &rep_id) == -1) {
			register_global_error_format(
			    "%s: ioctl(USB_GET_REPORT_ID): %s", child_dev,
			    strerror(errno));
			close(uhid);
			goto err;
		}
		if (dev->report_handles[rep_id] != 0xffffffff) {
			register_global_error_format(
			    "%s: duplicate report id: %d", child_dev, rep_id);
			close(uhid);
			goto err;
		}

		ph = &dev->poll_handles[dev->poll_handles_length++];
		ph->fd = uhid;
		ph->events = POLLIN;
		ph->revents = 0;
		dev->report_handles[rep_id] = uhid;
		if (!dev->device_handle)
			dev->device_handle = uhid;
	}

	dev->blocking = 1;
	dev->last_error_str = NULL;
	dev->device_info = NULL;
	strlcpy(dev->path, path, sizeof(dev->path));

	register_global_error(NULL);
	return dev;

err:
	for (i = 0; i < dev->poll_handles_length; i++)
		close(dev->poll_handles[i].fd);
	free(dev);
	return NULL;
}

int HID_API_EXPORT HID_API_CALL hid_write(hid_device *dev,
    const unsigned char *data, size_t length)
{
	int res, device_handle;

	if (!data || (length == 0)) {
		errno = EINVAL;
		register_device_error(dev, strerror(errno));
		return -1;
	}

	res = *data;
	device_handle = dev->report_handles[*data];
	if (device_handle < 0) {
		register_device_error_format(dev,
		    "unsupported report id: %hhu", *data);
		return -1;
	}

	length--;
	data++;

	res = write(device_handle, data, length);
	register_device_error(dev, (res == -1)? strerror(errno): NULL);
	return res;
}

int HID_API_EXPORT HID_API_CALL hid_read_timeout(hid_device *dev,
    unsigned char *data, size_t length, int milliseconds)
{
	int res;
	size_t i;
	struct pollfd *ph;
	ssize_t n;

	register_device_read_error(dev, NULL);

	res = poll(dev->poll_handles, dev->poll_handles_length, milliseconds);
	if (res == -1) {
		register_device_read_error_format(dev,
		    "error while polling: %s", strerror(errno));
		return -1;
	}

	if (res == 0) {
		/* Timeout, no data to read */
		return 0;
	}

	for (i = 0; i < dev->poll_handles_length; i++) {
		ph = &dev->poll_handles[i];

		if (ph->revents & (POLLERR | POLLHUP | POLLNVAL)) {
			register_device_read_error(dev,
			    "device IO error while polling");
			return -1;
		}

		if (ph->revents & POLLIN)
			break;
	}

	res = 0;
	for (i = 0; i < UHID_MAX_REPORTS; i++) {
		if (dev->report_handles[i] == ph->fd) {
			res = i;
			break;
		}
	}

	*data = (unsigned char)res;

	length--;
	data++;
	n = read(ph->fd, data, length);

	if (n == -1) {
		if (errno == EAGAIN || errno == EINPROGRESS) {
			n = 0;
		} else
			register_device_read_error_format(dev,
			    "error while reading: %s", strerror(errno));
	} else
		n += 1;

	return n;
}

int HID_API_EXPORT HID_API_CALL hid_read(hid_device *dev, unsigned char *data,
    size_t length)
{
	return hid_read_timeout(dev, data, length, (dev->blocking) ? -1 : 0);
}

HID_API_EXPORT const wchar_t* HID_API_CALL hid_read_error(hid_device *dev)
{
	if (dev->last_read_error_str == NULL)
		return L"Success";
	return dev->last_read_error_str;
}

int HID_API_EXPORT HID_API_CALL hid_set_nonblocking(hid_device *dev,
    int nonblock)
{
	dev->blocking = !nonblock;
	return 0;
}

int HID_API_EXPORT HID_API_CALL hid_send_feature_report(hid_device *dev,
    const unsigned char *data, size_t length)
{
	return set_report(dev, data, length, UHID_FEATURE_REPORT);
}

int HID_API_EXPORT HID_API_CALL hid_get_feature_report(hid_device *dev,
    unsigned char *data, size_t length)
{
	return get_report(dev, data, length, UHID_FEATURE_REPORT);
}

int HID_API_EXPORT HID_API_CALL hid_get_input_report(hid_device *dev,
    unsigned char *data, size_t length)
{
	return get_report(dev, data, length, UHID_INPUT_REPORT);
}

void HID_API_EXPORT HID_API_CALL hid_close(hid_device *dev)
{
	if (!dev)
		return;

	free(dev->last_error_str);
	free(dev->last_read_error_str);

	hid_free_enumeration(dev->device_info);

	for (size_t i = 0; i < dev->poll_handles_length; i++)
		close(dev->poll_handles[i].fd);

	free(dev);
}

int HID_API_EXPORT_CALL hid_get_manufacturer_string(hid_device *dev,
    wchar_t *string, size_t maxlen)
{
	struct hid_device_info *hdi;

	if (!string || !maxlen) {
		register_device_error(dev, "Zero buffer/length");
		return -1;
	}

	hdi = hid_get_device_info(dev);
	if (!dev)
		return -1;

	if (hdi->manufacturer_string) {
		wcsncpy(string, hdi->manufacturer_string, maxlen);
		string[maxlen - 1] = L'\0';
	} else {
		string[0] = L'\0';
	}

	return 0;
}

int HID_API_EXPORT_CALL hid_get_product_string(hid_device *dev,
    wchar_t *string, size_t maxlen)
{
	struct hid_device_info *hdi;

	if (!string || !maxlen) {
		register_device_error(dev, "Zero buffer/length");
		return -1;
	}

	hdi = hid_get_device_info(dev);
	if (!dev)
		return -1;

	if (hdi->product_string) {
		wcsncpy(string, hdi->product_string, maxlen);
		string[maxlen - 1] = L'\0';
	} else {
		string[0] = L'\0';
	}

	return 0;
}

int HID_API_EXPORT_CALL hid_get_serial_number_string(hid_device *dev,
    wchar_t *string, size_t maxlen)
{
	struct hid_device_info *hdi;

	if (!string || !maxlen) {
		register_device_error(dev, "Zero buffer/length");
		return -1;
	}

	hdi = hid_get_device_info(dev);
	if (!dev)
		return -1;

	if (hdi->serial_number) {
		wcsncpy(string, hdi->serial_number, maxlen);
		string[maxlen - 1] = L'\0';
	} else {
		string[0] = L'\0';
	}

	return 0;
}

struct hid_device_info HID_API_EXPORT * HID_API_CALL hid_get_device_info(
    hid_device *dev)
{
	int ifaceno;
	struct usb_device_info udi;
	struct usb_ctl_report_desc ucrd;
	int use_ucrd;
	struct hid_device_info *hdi;

	if (dev->device_info)
		return dev->device_info;

	if (ioctl(dev->device_handle, USB_GET_DEVICEINFO, &udi) == -1) {
		register_device_error_format(dev,
		    "ioctl(USB_GET_DEVICEINFO): %s", strerror(errno));
		return NULL;
	}
	if(ioctl(dev->device_handle, USB_GET_INTERFACE_ID, &ifaceno) == -1) {
		register_device_error_format(dev,
		    "ioctl(USB_GET_INTERFACE_ID): %s", strerror(errno));
		return NULL;
	}

	use_ucrd = ioctl(dev->device_handle, USB_GET_REPORT_DESC, &ucrd);
	hdi = create_device_info(&udi, ifaceno, dev->path,
	    (use_ucrd != -1) ? &ucrd : NULL);
	if (!hdi) {
		register_device_error(dev, "failed to create device info");
		return NULL;
	}

	dev->device_info = hdi;

	return hdi;
}

int HID_API_EXPORT_CALL hid_get_indexed_string(hid_device *dev,
    int string_index, wchar_t *string, size_t maxlen)
{
	return -1;
}

int HID_API_EXPORT_CALL hid_get_report_descriptor(hid_device *dev,
    unsigned char *buf, size_t buf_size)
{
	struct usb_ctl_report_desc ucrd;

	ucrd.ucrd_size = 0;
	if (ioctl(dev->device_handle, USB_GET_REPORT_DESC, &ucrd) == -1) {
		register_device_error_format(dev,
		    "ioctl(USB_GET_REPORT_DESC): %s", strerror(errno));
		return -1;
	}

	if ((size_t) ucrd.ucrd_size < buf_size)
		buf_size = (size_t) ucrd.ucrd_size;

	memcpy(buf, ucrd.ucrd_data, buf_size);

	return (int) buf_size;
}

HID_API_EXPORT const wchar_t* HID_API_CALL hid_error(hid_device *dev)
{
	if (dev) {
		if (dev->last_error_str == NULL)
			return L"Success";
		return dev->last_error_str;
	}

	if (last_global_error_str == NULL)
		return L"Success";
	return last_global_error_str;
}

HID_API_EXPORT const struct hid_api_version* HID_API_CALL hid_version(void)
{
	static const struct hid_api_version api_version = {
		.major = HID_API_VERSION_MAJOR,
		.minor = HID_API_VERSION_MINOR,
		.patch = HID_API_VERSION_PATCH
	};

	return &api_version;
}

HID_API_EXPORT const char* HID_API_CALL hid_version_str(void)
{
	return HID_API_VERSION_STR;
}
