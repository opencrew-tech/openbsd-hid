COMMENT =	microbenchmark support library

DIST_TUPLE =	github google benchmark v1.9.4 .

CATEGORIES =	devel

MAINTAINER =	Fabien Romano <fabien@openbsd.org>

# Apache 2.0
PERMIT_PACKAGE =	Yes

COMPILER =		base-clang ports-gcc base-gcc

MODULES =		devel/cmake

BUILD_DEPENDS =		devel/gtest

CONFIGURE_ARGS+=	-DBENCHMARK_USE_BUNDLED_GTEST=OFF

.include <bsd.port.mk>
