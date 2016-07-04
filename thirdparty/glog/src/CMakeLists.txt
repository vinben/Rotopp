IF (WIN32)
  INCLUDE_DIRECTORIES(
    windows
  )

  IF (NOT MINGW)
    INCLUDE_DIRECTORIES(
      ../../msinttypes
    )
  ENDIF (NOT MINGW)

ELSE (WIN32)
  INCLUDE_DIRECTORIES(
    .
  )
ENDIF (WIN32)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "-fPIC")
  #  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused -Wno-type-limits -Wno-empty-body")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

IF (WIN32)
  ADD_LIBRARY(glog
              logging.cc
              windows/port.cc
              raw_logging.cc
              utilities.cc
              vlog_is_on.cc)
ELSE (WIN32)
  ADD_LIBRARY(glog
              demangle.cc
              logging.cc
              raw_logging.cc
              signalhandler.cc
              symbolize.cc
              utilities.cc
              vlog_is_on.cc)
  TARGET_LINK_LIBRARIES(glog gflags)
ENDIF (WIN32)

MACRO (GLOG_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  TARGET_LINK_LIBRARIES(${NAME} glog gflags pthread googletest)
  SET_TARGET_PROPERTIES(${NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${LIBMV_TESTS_OUTPUT_DIR})
  ADD_TEST(${NAME} ${LIBMV_TESTS_OUTPUT_DIR}/${NAME})
ENDMACRO (GLOG_TEST)

INSTALL(TARGETS glog
        EXPORT  CeresExport
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib${LIB_SUFFIX}
        ARCHIVE DESTINATION lib${LIB_SUFFIX})

