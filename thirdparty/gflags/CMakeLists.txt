INCLUDE_DIRECTORIES(
  .
  gflags
)

SET(SRC
  gflags.cc
  gflags_reporting.cc
  gflags_completions.cc
)

IF(WIN32)
  INCLUDE_DIRECTORIES(
    ../msinttypes
    ../pthreads-w32/include
  )
  LIST(APPEND SRC windows_port.cc)
ENDIF(WIN32)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "-fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)


ADD_LIBRARY(gflags ${SRC})
TARGET_LINK_LIBRARIES(gflags pthread)

INSTALL(TARGETS gflags
        EXPORT  CeresExport
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib${LIB_SUFFIX}
        ARCHIVE DESTINATION lib${LIB_SUFFIX})

