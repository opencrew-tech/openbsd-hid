COMMENT=	DJ software to perform live mixes

DIST_TUPLE =	github mixxxdj mixxx 2.5.0 .

CATEGORIES =	audio

HOMEPAGE =	https://mixxx.org/

MAINTAINER =	Jonathan Armani <armani@openbsd.org>

# GPLv2+, some included dependencies with less strong licenses, see LICENSE
PERMIT_PACKAGE =	Yes

WANTLIB += ${COMPILER_LIBCXX} FLAC GL ICE Qt6Concurrent Qt6Core
WANTLIB += Qt6DBus Qt6Gui Qt6Network Qt6OpenGL Qt6PrintSupport
WANTLIB += Qt6Sql Qt6Test Qt6Widgets Qt6Xml SM SoundTouch X11
WANTLIB += Xext absl_bad_optional_access absl_bad_variant_access
WANTLIB += absl_base absl_city absl_civil_time absl_cord absl_cord_internal
WANTLIB += absl_cordz_functions absl_cordz_handle absl_cordz_info
WANTLIB += absl_crc32c absl_crc_cord_state absl_crc_cpu_detect
WANTLIB += absl_crc_internal absl_debugging_internal absl_decode_rust_punycode
WANTLIB += absl_demangle_internal absl_demangle_rust absl_die_if_null
WANTLIB += absl_examine_stack absl_exponential_biased absl_flags_commandlineflag
WANTLIB += absl_flags_commandlineflag_internal absl_flags_config
WANTLIB += absl_flags_internal absl_flags_marshalling absl_flags_private_handle_accessor
WANTLIB += absl_flags_program_name absl_flags_reflection absl_graphcycles_internal
WANTLIB += absl_hash absl_hashtablez_sampler absl_int128 absl_kernel_timeout_internal
WANTLIB += absl_log_entry absl_log_globals absl_log_initialize
WANTLIB += absl_log_internal_check_op absl_log_internal_conditions
WANTLIB += absl_log_internal_fnmatch absl_log_internal_format
WANTLIB += absl_log_internal_globals absl_log_internal_log_sink_set
WANTLIB += absl_log_internal_message absl_log_internal_nullguard
WANTLIB += absl_log_internal_proto absl_log_severity absl_log_sink
WANTLIB += absl_low_level_hash absl_malloc_internal absl_random_distributions
WANTLIB += absl_random_internal_platform absl_random_internal_pool_urbg
WANTLIB += absl_random_internal_randen absl_random_internal_randen_hwaes
WANTLIB += absl_random_internal_randen_hwaes_impl absl_random_internal_randen_slow
WANTLIB += absl_random_internal_seed_material absl_random_seed_gen_exception
WANTLIB += absl_random_seed_sequences absl_raw_hash_set absl_raw_logging_internal
WANTLIB += absl_spinlock_wait absl_stacktrace absl_status absl_statusor
WANTLIB += absl_str_format_internal absl_strerror absl_string_view
WANTLIB += absl_strings absl_strings_internal absl_symbolize absl_synchronization
WANTLIB += absl_throw_delegate absl_time absl_time_zone absl_utf8_for_code_point
WANTLIB += absl_vlog_config_internal avcodec avformat avutil c
WANTLIB += chromaprint crypto djinterop ebur128 glib-2.0 gobject-2.0
WANTLIB += hidapi-libusb id3tag keyfinder lilv-0 m mad mp3lame
WANTLIB += mp4v2 ogg opus opusfile portaudio portmidi protobuf-lite
WANTLIB += qt6keychain rubberband sndfile sqlite3 ssl swresample
WANTLIB += tag upower-glib usb-1.0 vorbis vorbisenc vorbisfile
WANTLIB += wavpack xkbcommon z

COMPILER =		base-clang ports-gcc

MODULES =		devel/cmake \
			x11/qt6

# devel/benchmark needed to build tests
BUILD_DEPENDS=		audio/libdjinterop \
			devel/benchmark \
			devel/microsoft-gsl

RUN_DEPENDS=		audio/vamp-plugin-sdk \
			devel/desktop-file-utils \
			x11/gtk+4,-guic

# XXX
# LIB_DEPENDS audio/faad not needed for audio/mixxx ?
# LIB_DEPENDS audio/libopenmpt not needed for audio/mixxx ?
# LIB_DEPENDS net/libshout not needed for audio/mixxx ?
LIB_DEPENDS =		audio/chromaprint \
			audio/faad \
			audio/lame \
			audio/libdjinterop \
			audio/libebur128 \
			audio/libid3tag \
			audio/libkeyfinder \
			audio/libmad \
			audio/libopenmpt \
			audio/libogg \
			audio/libsndfile \
			audio/libvorbis \
			audio/lilv \
			audio/opusfile \
			audio/portaudio-svn \
			audio/portmidi \
			audio/rubberband \
			audio/soundtouch \
			audio/taglib \
			audio/wavpack \
			comms/libhidapi \
			databases/sqlite3 \
			devel/libusb1 \
			devel/protobuf \
			graphics/ffmpeg \
			multimedia/libmp4v2 \
			net/libshout \
			security/qtkeychain,qt6 \
			sysutils/upower

TEST_DEPENDS =		devel/gtest

# XXX
# -DOPTIMIZE:STRING=portable \
# -DBATTERY:BOOL=OFF \
# -DFAAD:BOOL=OFF \
# -DMODPLUG:BOOL=ON \
# -DMAD:BOOL=ON
CONFIGURE_ARGS+=	-DINSTALL_USER_UDEV_RULES:BOOL=OFF

CONFIGURE_ENV+=		QT_DIR=${MODQT6_LIBDIR}/cmake

# if debug packages are a good idea, extra configure args may be necessary
# and DEBUG_PACKAGES will commonly be set to ${BUILD_PACKAGES}
#DEBUG_CONFIGURE_ARGS =	???
#DEBUG_PACKAGES = ${BUILD_PACKAGES}

# Assume you have one multiple choice flavor: 1 2 3 and switches a b.
# You would write
#
#FLAVORS =	1 2 3 a b
#FLAVOR ?=
# grab multiple choice value
#CHOICE = ${FLAVOR:Na:Nb}
# check that CHOICE is 1 OR 2 OR 3, or error out
#.if ${CHOICE} == "1"
# code for 1
#.elif ${CHOICE} == "2"
# code for 2
#.elif ${CHOICE} == "3"
# code for 3
#.else
#ERRORS += "Fatal: Conflicting flavor: ${FLAVOR}"
#.endif
# check for switches
#.if ${FLAVOR:Ma}
# code for a
#.endif
#.if ${FLAVOR:Mb}
# code for b
#.endif

.include <bsd.port.mk>
