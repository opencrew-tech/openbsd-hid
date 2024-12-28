COMMENT =	microbenchmark support library

V=1.9.1
DIST_TUPLE =	github google benchmark v${V} .

CATEGORIES =	devel

MAINTAINER =	Fabien Romano <fabien@openbsd.org>

# Apache 2.0
PERMIT_PACKAGE =	Yes

COMPILER =		base-clang ports-gcc base-gcc

MODULES =		devel/cmake

TEST_DEPENDS =		devel/gtest

CONFIGURE_ARGS+=	-DBENCHMARK_USE_BUNDLED_GTEST=OFF

.include <bsd.port.mk>
