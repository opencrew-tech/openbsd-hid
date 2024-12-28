COMMENT =	musical key detection for digital audio

V=2.2.8
DIST_TUPLE =	github mixxxdj libkeyfinder ${V} .

SHARED_LIBS +=	keyfinder 0.0

CATEGORIES =	audio

HOMEPAGE =	https://mixxxdj.github.io/libkeyfinder/

MAINTAINER =	Fabien Romano <fabien@openbsd.org>

# GPLv3
PERMIT_PACKAGE =	Yes

WANTLIB += ${COMPILER_LIBCXX} fftw3 m

COMPILER =		base-clang ports-gcc base-gcc

MODULES =		devel/cmake

BUILD_DEPENDS=		devel/catch2
LIB_DEPENDS =		math/fftw3

.include <bsd.port.mk>
