# Minimum cmake version required
cmake_minimum_required(VERSION 3.8)

# Project configuration
project(TESTS)

# Header files
set (RP3D_TESTS_HEADERS
    "Test.h"
    "TestSuite.h"
    "tests/collision/TestAABB.h"
    "tests/collision/TestWorldQueries.h"
    "tests/collision/TestDynamicAABBTree.h"
    "tests/collision/TestHalfEdgeStructure.h"
    "tests/collision/TestPointInside.h"
    "tests/collision/TestRaycast.h"
    "tests/collision/TestTriangleVertexArray.h"
    "tests/collision/TestConvexMesh.h"
    "tests/collision/TestHeightField.h"
    "tests/collision/TestTriangleMesh.h"
    "tests/containers/TestArray.h"
    "tests/containers/TestMap.h"
    "tests/containers/TestSet.h"
    "tests/containers/TestStack.h"
    "tests/containers/TestDeque.h"
    "tests/mathematics/TestMathematicsFunctions.h"
    "tests/mathematics/TestMatrix2x2.h"
    "tests/mathematics/TestMatrix3x3.h"
    "tests/mathematics/TestQuaternion.h"
    "tests/mathematics/TestTransform.h"
    "tests/mathematics/TestVector2.h"
    "tests/mathematics/TestVector3.h"
    "tests/engine/TestRigidBody.h"
    "tests/systems/TestDynamicSystem.h"
    "tests/utils/TestQuickHull.h"
)

# Source files
set (RP3D_TESTS_SOURCES
    "main.cpp"
    "Test.cpp"
    "TestSuite.cpp"
)

# Create the tests executable
add_executable(tests ${RP3D_TESTS_HEADERS} ${RP3D_TESTS_SOURCES})

# Headers
target_include_directories(reactphysics3d PUBLIC
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(tests reactphysics3d)

add_test(Test tests)
