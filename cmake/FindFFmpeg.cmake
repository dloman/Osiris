FIND_PATH(
  FFMPEG_INCLUDE_DIRS
  NAMES
    libavutil/avutil.h
    libavformat/avformat.h
    libavdevice/avdevice.h
    libswscale/swscale.h
    libavcodec/avfft.h
    libswresample/swresample.h
  PATHS
    ${FFMPEG_ROOT}/include
    /usr/local/include
  )

IF(NOT FFMPEG_INCLUDE_DIRS)
  MESSAGE(FATAL_ERROR "Could not find FFMPEG headers")
ENDIF()

FIND_LIBRARY(X264
  NAMES
  x264
  )

FIND_LIBRARY(ZLIB
  NAMES
  z
  )

FIND_LIBRARY(LZMA
  NAMES
  lzma
  )

FIND_LIBRARY(X11
  NAMES
  X11
  )

  IF (NOT X264)
    MESSAGE(FATAL_ERROR "Could not find FFMPEG x264")
  ENDIF()

  FIND_LIBRARY(MFX
    NAMES
    mfx
    )

  IF (MFX)
    LIST(APPEND FFMPEG_VIDEO_ENCODER ${MFX})
  ENDIF()

  FIND_LIBRARY(DL
    NAMES
    dl
    )

  LIST(APPEND FFMPEG_VIDEO_ENCODER ${X264})

FIND_LIBRARY(FFMPEG_LIBRARY_AVCODEC
  NAMES
    avcodec
  PATHS
    ${FFMPEG_ROOT}/lib
    /usr/local/lib
)

FIND_LIBRARY(FFMPEG_LIBRARY_AVDEVICE
  NAMES
    avdevice
  PATHS
    ${FFMPEG_ROOT}/lib
    /usr/local/lib
)

FIND_LIBRARY(FFMPEG_LIBRARY_AVFORMAT
  NAMES
    avformat
  PATHS
    ${FFMPEG_ROOT}/lib
    /usr/local/lib
)

FIND_LIBRARY(FFMPEG_LIBRARY_AVUTIL
  NAMES
    avutil
  PATHS
    ${FFMPEG_ROOT}/lib
    /usr/local/lib
)

FIND_LIBRARY(FFMPEG_LIBRARY_SWSCALE
  NAMES
    swscale
  PATHS
    ${FFMPEG_ROOT}/lib
    /usr/local/lib
)

FIND_LIBRARY(FFMPEG_LIBRARY_SWRESAMPLE
  NAMES
    swresample
  PATHS
    ${FFMPEG_ROOT}/lib
    /usr/local/lib
)

# For OSX
FIND_LIBRARY(BZ2 bz2)
FIND_LIBRARY(CORE_FOUNDATION CoreFoundation)
FIND_LIBRARY(CORE_MEDIA CoreMedia)
FIND_LIBRARY(CORE_VIDEO CoreVideo)
FIND_LIBRARY(VIDEO_DECODE_ACCELERATION VideoDecodeAcceleration)
FIND_LIBRARY(VIDEO_TOOL_BOX VideoToolBox)
FIND_LIBRARY(SECURITY Security)
FIND_LIBRARY(ICONV NAMES iconv)
FIND_LIBRARY(XCBSHM xcb-shm)

FIND_LIBRARY(DL dl)
FIND_LIBRARY(LZMA lzma)
IF (BZ2)
  LIST(APPEND OPTIONAL_LIBRARIES ${BZ2})
ENDIF()

IF(CORE_FOUNDATION)
  LIST(APPEND OPTIONAL_LIBRARIES ${CORE_FOUNDATION})
ENDIF()

IF(CORE_MEDIA)
  LIST(APPEND OPTIONAL_LIBRARIES ${CORE_MEDIA})
ENDIF()

IF(CORE_VIDEO)
  LIST(APPEND OPTIONAL_LIBRARIES ${CORE_VIDEO})
ENDIF()

IF(VIDEO_DECODE_ACCELERATION)
  LIST(APPEND OPTIONAL_LIBRARIES ${VIDEO_DECODE_ACCELERATION})
ENDIF()

IF(VIDEO_TOOL_BOX)
  LIST(APPEND OPTIONAL_LIBRARIES ${VIDEO_TOOL_BOX})
ENDIF()

IF(SECURITY)
  LIST(APPEND OPTIONAL_LIBRARIES ${SECURITY})
ENDIF()

IF(ICONV)
  LIST(APPEND OPTIONAL_LIBRARIES ${ICONV})
ENDIF()

SET(
  FFMPEG_LIBRARIES
  z
  ${OPTIONAL_LIBRARIES}
  ${FFMPEG_LIBRARY_AVFORMAT}
  ${FFMPEG_LIBRARY_AVCODEC}
  ${FFMPEG_LIBRARY_AVDEVICE}
  ${FFMPEG_LIBRARY_AVUTIL}
  ${FFMPEG_LIBRARY_SWSCALE}
  ${FFMPEG_LIBRARY_SWRESAMPLE}
  ${FFMPEG_VIDEO_ENCODER}
  ${DL}
  ${LZMA}
  ${X264}
  ${XCBSHM}
  )

IF(NOT FFMPEG_LIBRARIES)
  MESSAGE(FATAL_ERROR "Could not find FFMPEG libraries")
ENDIF()

INCLUDE_DIRECTORIES(${FFMPEG_INCLUDE_DIRS})
