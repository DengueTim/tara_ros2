FIND_PATH(Tara_INCLUDE_DIRS Tara.h xunit_lib_tara.h
        /usr/local/tara-sdk/include
        )

SET(TARA_LIB_NAMES  )
FIND_LIBRARY(ECON_TARA_LIBRARY
        NAMES libecon_tara.so
        PATHS /usr/local/tara-sdk/lib
        )

FIND_LIBRARY(ECON_XUNIT_LIBRARY
        NAMES libecon_xunit.so
        PATHS /usr/local/tara-sdk/lib
        )

IF (Tara_INCLUDE_DIRS AND ECON_TARA_LIBRARY AND ECON_XUNIT_LIBRARY)
    SET(Tara_FOUND TRUE)
    SET(Tara_LIBS ${ECON_TARA_LIBRARY} ${ECON_XUNIT_LIBRARY})
    MESSAGE(NOTICE "-- Found eCon Tara libs.")
ENDIF (Tara_INCLUDE_DIRS AND ECON_TARA_LIBRARY AND ECON_XUNIT_LIBRARY)

IF(NOT Tara_FOUND)
    MESSAGE(NOTICE "Tara libs not found")
    IF (Tara_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find REQUIRED Tara libs")
    ENDIF (Tara_FIND_REQUIRED)
ENDIF(NOT Tara_FOUND)