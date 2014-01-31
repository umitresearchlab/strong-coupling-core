CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(SRCS
    src/scConnector.cpp
    src/scConstraint.cpp
    src/scEquation.cpp
    src/scLockConstraint.cpp
    src/scQuat.cpp
    src/scSlave.cpp
    src/scSolver.cpp
    src/scVec3.cpp
)

SET(HEADERS
    include/scConnector.h
    include/scConstraint.h
    include/scEquation.h
    include/scLockConstraint.h
    include/scQuat.h
    include/scSlave.h
    include/scSolver.h
    include/scVec3.h
)

INCLUDE_DIRECTORIES(${UMFPACK_INCLUDE_DIR} include)
LINK_DIRECTORIES(${UMFPACK_LIBRARY_DIR})

ADD_LIBRARY(sc ${HEADERS} ${SRCS})

SET_TARGET_PROPERTIES( sc
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# OSX
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    INCLUDE_DIRECTORIES(/opt/local/include)
    LINK_DIRECTORIES(/opt/local/lib)
    TARGET_LINK_LIBRARIES(sc m umfpack amd cblas SuiteSparse)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# LINUX
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    TARGET_LINK_LIBRARIES(sc m umfpack amd cblas atlas)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

ADD_SUBDIRECTORY(test)

INSTALL(TARGETS sc
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
