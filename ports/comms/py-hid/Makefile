COMMENT=	Ctypes bindings to comms/hidapi

MODPY_DISTV=	1.0.8
DISTNAME=	hid-${MODPY_DISTV}
PKGNAME=	py-${DISTNAME}

CATEGORIES=	comms

HOMEPAGE=	https://github.com/apmorton/pyhidapi

MAINTAINER=	Fabien Romano <fabien@openbsd.org>

# MIT
PERMIT_PACKAGE=	Yes

MODULES=	lang/python

# Get source from pypi.org
MODPY_PI=	Yes
MODPY_PYBUILD=	setuptools

LIB_DEPENDS=	comms/libhidapi

.include <bsd.port.mk>
