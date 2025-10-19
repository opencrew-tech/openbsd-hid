COMMENT=	library for access to DJ record libraries

DIST_TUPLE=	github xsco libdjinterop 0.26.1 .

SHARED_LIBS+=	djinterop 0.0

CATEGORIES=	audio

MAINTAINER=	Fabien Romano <fabien@openbsd.org>

# LGPLv3
PERMIT_PACKAGE=	Yes

WANTLIB += ${COMPILER_LIBCXX} m sqlite3 z

COMPILER=	base-clang ports-gcc base-gcc

MODULES =	devel/cmake

BUILD_DEPENDS=	devel/boost
LIB_DEPENDS=	databases/sqlite3

.include <bsd.port.mk>
