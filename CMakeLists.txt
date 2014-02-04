CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(SRCS
    src/Connector.cpp
    src/Constraint.cpp
    src/Equation.cpp
    src/LockConstraint.cpp
    src/Quat.cpp
    src/Slave.cpp
    src/Solver.cpp
    src/Vec3.cpp
    src/Mat3.cpp
    src/JacobianElement.cpp
)

SET(HEADERS
    include/Connector.h
    include/Constraint.h
    include/Equation.h
    include/LockConstraint.h
    include/Quat.h
    include/Slave.h
    include/Solver.h
    include/Vec3.h
    include/Mat3.h
    include/JacobianElement.h
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
