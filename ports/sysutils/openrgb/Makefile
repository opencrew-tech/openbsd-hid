COMMENT=	open source RGB lighting control

V=0.9
ID=b5f46e3f1de03595656c682fc2f068b66e735e63
DATE="2023-07-10 01:45:06"
PKGNAME=	openrgb-${V}

DIST_TUPLE=	gitlab CalcProgrammer1 OpenRGB release_${V} .

CATEGORIES=	sysutils

HOMEPAGE=	https://openrgb.org/

MAINTAINER=	Fabien Romano <fabien@openbsd.org>

#GPLv2
PERMIT_PACKAGE=	Yes

WANTLIB += ${COMPILER_LIBCXX} GL Qt5Core Qt5Gui Qt5Widgets c hidapi-openbsd
WANTLIB += m mbedcrypto mbedtls mbedx509 usb-1.0

FIX_CRLF_FILES= \
	Controllers/CorsairHydroController/CorsairHydroControllerDetect.cpp

COMPILER=	base-clang ports-gcc base-gcc

MODULES=	devel/qmake \
		x11/qt5

RUN_DEPENDS=	devel/desktop-file-utils \
		x11/gtk+4,-guic
LIB_DEPENDS=	comms/libhidapi \
		devel/libusb1 \
		security/polarssl

NO_TEST=	Yes

WRKDIST=	${WRKDIR}/OpenRGB-release_${V}-${ID}

SUBST_VARS+=	ID DATE
post-patch:
	${SUBST_CMD} ${WRKSRC}/OpenRGB.pro
	${SUBST_CMD} ${WRKSRC}/AutoStart/AutoStart-Linux.cpp

.include <bsd.port.mk>
