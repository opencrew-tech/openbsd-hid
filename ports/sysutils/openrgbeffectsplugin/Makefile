COMMENT=	effects plugin for OpenRGB

V=0.9
ID=f1411e178a0024e3da8d178bbd0f0b60425e35fd
DATE="2023-08-23 19:50:38"
PKGNAME=	openrgbeffectsplugin-${V}

QCODEEDITORID=	a9aab24c7970a38d14bc79939306d9d3ba78cf61
SIMPLEXNOISEID=	97e62c5b5e26c8edabdc29a6b0a277192be3746c

DIST_TUPLE=	gitlab OpenRGBDevelopers OpenRGBEffectsPlugin release_${V} .
DIST_TUPLE+=	gitlab CalcProgrammer1 OpenRGB release_${V} .
DIST_TUPLE+=	github justxi QCodeEditor ${QCODEEDITORID} .
DIST_TUPLE+=	github SRombauts SimplexNoise ${SIMPLEXNOISEID} .

SHARED_LIBS +=  OpenRGBEffectsPlugin      0.0 # 1.0

CATEGORIES=	sysutils

MAINTAINER=	Fabien Romano <fabien@openbsd.org>

#GPLv2
PERMIT_PACKAGE=	Yes

WANTLIB += ${COMPILER_LIBCXX} GL Qt5Core Qt5Gui Qt5Widgets m openal

COMPILER=	base-clang ports-gcc base-gcc

MODULES=	devel/qmake \
		x11/qt5

RUN_DEPENDS=	sysutils/openrgb
LIB_DEPENDS=	audio/openal

NO_TEST=	Yes

WRKDIST=	${WRKDIR}/OpenRGBEffectsPlugin-release_${V}-${ID}

SUBST_VARS+=	ID DATE

post-extract:
	rm -rf ${WRKDIST}/OpenRGB && \
	mv ${WRKDIR}/OpenRGB-release_${V}* ${WRKDIST}/OpenRGB
	rm -rf ${WRKDIST}/Dependencies/QCodeEditor && \
	mv ${WRKDIR}/QCodeEditor* ${WRKDIST}/Dependencies/QCodeEditor
	rm -rf ${WRKDIST}/Dependencies/SimplexNoise && \
	mv ${WRKDIR}/SimplexNoise* ${WRKDIST}/Dependencies/SimplexNoise

post-patch:
	${SUBST_CMD} ${WRKSRC}/OpenRGBEffectsPlugin.pro

.include <bsd.port.mk>
