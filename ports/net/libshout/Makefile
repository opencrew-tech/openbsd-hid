COMMENT=	library for communicating with an icecast server

DISTNAME=	libshout-2.4.6

SHARED_LIBS +=  shout                7.0      # 6.0

CATEGORIES=	net audio

HOMEPAGE=	https://icecast.org/

MAINTAINER=	Moritz Grimm <mgrimm@mrsserver.net>

# LGPLv2+
PERMIT_PACKAGE=	Yes

WANTLIB += c crypto m ogg pthread speex ssl theora>=1 vorbis>=4

SITES=		https://downloads.xiph.org/releases/libshout/

LIB_DEPENDS=	audio/libogg \
		audio/libvorbis \
		audio/speex \
		multimedia/libtheora

FAKE_FLAGS=	docdir='${PREFIX}/share/examples/libshout'

SEPARATE_BUILD=	Yes

CONFIGURE_STYLE= gnu

.include <bsd.port.mk>
