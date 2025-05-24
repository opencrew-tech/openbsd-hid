COMMENT=	DJ software to perform live mixes

DIST_TUPLE =	github mixxxdj mixxx 2.5.1 .

CATEGORIES =	audio

HOMEPAGE =	https://mixxx.org/

MAINTAINER =	Jonathan Armani <armani@openbsd.org>

# GPLv2+, some included dependencies with less strong licenses, see LICENSE
PERMIT_PACKAGE =	Yes

WANTLIB += ${COMPILER_LIBCXX} FLAC GL ICE Qt6Concurrent
WANTLIB += Qt6Core Qt6Core5Compat Qt6DBus Qt6Gui Qt6LabsQmlModels
WANTLIB += Qt6Network Qt6OpenGL Qt6PrintSupport Qt6Qml
WANTLIB += Qt6QmlMeta Qt6QmlModels Qt6QmlWorkerScript
WANTLIB += Qt6Quick Qt6QuickControls2 Qt6QuickControls2Basic
WANTLIB += Qt6QuickControls2BasicStyleImpl Qt6QuickControls2Fusion
WANTLIB += Qt6QuickControls2FusionStyleImpl Qt6QuickControls2Impl
WANTLIB += Qt6QuickLayouts Qt6QuickShapes Qt6QuickTemplates2
WANTLIB += Qt6QuickWidgets Qt6ShaderTools Qt6Sql Qt6Svg
WANTLIB += Qt6SvgWidgets Qt6Test Qt6Widgets Qt6Xml SM
WANTLIB += SoundTouch X11 Xext absl_base absl_city absl_civil_time
WANTLIB += absl_cord absl_cord_internal absl_cordz_functions
WANTLIB += absl_cordz_handle absl_cordz_info absl_crc32c
WANTLIB += absl_crc_cord_state absl_crc_cpu_detect absl_crc_internal
WANTLIB += absl_debugging_internal absl_decode_rust_punycode
WANTLIB += absl_demangle_internal absl_demangle_rust
WANTLIB += absl_die_if_null absl_examine_stack absl_exponential_biased
WANTLIB += absl_flags_commandlineflag absl_flags_commandlineflag_internal
WANTLIB += absl_flags_config absl_flags_internal absl_flags_marshalling
WANTLIB += absl_flags_private_handle_accessor absl_flags_program_name
WANTLIB += absl_flags_reflection absl_graphcycles_internal
WANTLIB += absl_hash absl_hashtablez_sampler absl_int128
WANTLIB += absl_kernel_timeout_internal absl_leak_check
WANTLIB += absl_log_globals absl_log_initialize absl_log_internal_check_op
WANTLIB += absl_log_internal_conditions absl_log_internal_fnmatch
WANTLIB += absl_log_internal_format absl_log_internal_globals
WANTLIB += absl_log_internal_log_sink_set absl_log_internal_message
WANTLIB += absl_log_internal_nullguard absl_log_internal_proto
WANTLIB += absl_log_internal_structured_proto absl_log_severity
WANTLIB += absl_log_sink absl_low_level_hash absl_malloc_internal
WANTLIB += absl_random_distributions absl_random_internal_entropy_pool
WANTLIB += absl_random_internal_platform absl_random_internal_randen
WANTLIB += absl_random_internal_randen_hwaes
WANTLIB += absl_random_internal_randen_hwaes_impl
WANTLIB += absl_random_internal_randen_slow absl_random_internal_seed_material
WANTLIB += absl_random_seed_gen_exception absl_random_seed_sequences
WANTLIB += absl_raw_hash_set absl_raw_logging_internal
WANTLIB += absl_spinlock_wait absl_stacktrace absl_status
WANTLIB += absl_statusor absl_str_format_internal absl_strerror
WANTLIB += absl_string_view absl_strings absl_strings_internal
WANTLIB += absl_symbolize absl_synchronization absl_throw_delegate
WANTLIB += absl_time absl_time_zone absl_tracing_internal
WANTLIB += absl_utf8_for_code_point absl_vlog_config_internal
WANTLIB += avcodec avformat avutil c chromaprint djinterop
WANTLIB += ebur128 glib-2.0 gobject-2.0 hidapi-libusb
WANTLIB += id3tag keyfinder lilv-0 m mad modplug mp3lame
WANTLIB += mp4v2 ogg opus opusfile portaudio portmidi
WANTLIB += protobuf-lite qt6keychain rubberband shout
WANTLIB += sndfile sqlite3 swresample tag upower-glib
WANTLIB += usb-1.0 vorbis vorbisenc vorbisfile wavpack
WANTLIB += xkbcommon z

COMPILER =		base-clang ports-gcc

MODULES =		devel/cmake \
			x11/qt6

BUILD_DEPENDS=		devel/benchmark \
			devel/microsoft-gsl

RUN_DEPENDS=		audio/vamp-plugin-sdk \
			devel/desktop-file-utils \
			x11/gtk+4,-guic

LIB_DEPENDS =		audio/chromaprint \
			audio/lame \
			audio/libdjinterop \
			audio/libebur128 \
			audio/libid3tag \
			audio/libkeyfinder \
			audio/libmad \
			audio/libmodplug \
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
			sysutils/upower \
			x11/qt6/qt5compat \
			x11/qt6/qtshadertools

TEST_DEPENDS =		devel/gtest

CONFIGURE_ARGS+=	-DINSTALL_USER_UDEV_RULES:BOOL=OFF

CONFIGURE_ENV+=		QT_DIR=${MODQT6_LIBDIR}/cmake

.include <bsd.port.mk>
