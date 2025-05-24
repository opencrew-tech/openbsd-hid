COMMENT =	library for communicating with USB and Bluetooth HID devices

# XXX Any consumer should be patched to use hidapi_hid_init() instead of
# hid_init(3) to avoid conflict with base usbhid(3).

V =		0.15.0
PKGNAME =	libhidapi-${V}

DIST_TUPLE =	github libusb hidapi hidapi-${V} .

SHARED_LIBS +=	hidapi-libusb		1.1 # 0.0
SHARED_LIBS +=	hidapi-openbsd		1.1 # 0.0
SHARED_LIBS +=	hidapi			1.1 # 0.0

CATEGORIES =	comms

# GPLv3, BSD-style, or more liberal original HIDAPI license
# chosen at the discretion of the user of HIDAPI
PERMIT_PACKAGE =	Yes

WANTLIB += c iconv pthread usb-1.0

MODULES =	devel/cmake

LIB_DEPENDS =	converters/libiconv \
		devel/libusb1

CONFIGURE_ARGS+=-DHIDAPI_BUILD_HIDTEST=ON

DOCDIR =	${PREFIX}/share/doc/hidapi


post-extract:
	@mkdir ${WRKSRC}/openbsd
	@cp ${FILESDIR}/{CMakeLists.txt,hid.c} ${WRKSRC}/openbsd/
	@cp ${FILESDIR}/hidapi-openbsd.pc.in ${WRKSRC}/pc/

post-install:
	${INSTALL_DATA_DIR} ${DOCDIR}
	cd ${WRKSRC}; ${INSTALL_DATA} LICENSE* README.md AUTHORS.txt ${DOCDIR}/
	ln -sf libhidapi-libusb.so.${LIBhidapi-openbsd_VERSION} \
		${PREFIX}/lib/libhidapi.so.${LIBhidapi-openbsd_VERSION}

do-test:
	cd ${WRKBUILD}/hidtest && ./hidtest

.include <bsd.port.mk>
