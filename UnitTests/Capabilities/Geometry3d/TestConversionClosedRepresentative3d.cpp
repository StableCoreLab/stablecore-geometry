#include <cmath>
#include <ostream>
#include <string>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/ConversionFixturesClosed3d.h"

using Geometry::BrepConversionIssue3d;
using Geometry::ConvertToBrepBody;
using Geometry::SCPoint3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronBrepBodyConversion3d;

using namespace Geometry::Test::ConversionFixtures;
// Closed tetrahedron and prism representative-average repair variants.
struct SupportMismatchRepresentativeCase
{
    const char* name;
    PolyhedronBody (*build)();
};

void PrintTo(const SupportMismatchRepresentativeCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class ClosedTetrahedronDualVerticesTest : public ::testing::TestWithParam<SupportMismatchRepresentativeCase>
{
};

TEST_P(ClosedTetrahedronDualVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 4);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 4);
    ASSERT_EQ(result.body.EdgeCount(), 6);

    const double s = 1e-5;
    const double expectedV1X = s + (2e-7 - 1e-7 + 0.0) / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && std::abs(point.z - 1.2e-6) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            ASSERT_LT(std::abs(point.x - expectedV1X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
    ASSERT_EQ(sharedV1Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedTetrahedronDualVerticesTest,
    ::testing::Values(
        SupportMismatchRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesBody},
        SupportMismatchRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDuplicateLoopBody},
        SupportMismatchRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<SupportMismatchRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedTetrahedronAllVerticesTest : public ::testing::TestWithParam<SupportMismatchRepresentativeCase>
{
};

TEST_P(ClosedTetrahedronAllVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 4);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 4);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 4);
    ASSERT_EQ(result.body.EdgeCount(), 6);

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV3X = 0.5 * s + 1.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV3Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && point.z > 0.5e-6 && point.z < 2.0e-6 && point.x > 0.5 * s)
        {
            ++sharedV1Count;
            ASSERT_LT(std::abs(point.x - expectedV1X), 1e-10);
        }
        if (point.z > 0.7 * s && point.x > 0.3 * s && point.x < 0.7 * s)
        {
            ++sharedV3Count;
            ASSERT_LT(std::abs(point.x - expectedV3X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV1Count, 1);
    ASSERT_EQ(sharedV3Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedTetrahedronAllVerticesTest,
    ::testing::Values(
        SupportMismatchRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesBody},
        SupportMismatchRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDuplicateLoopBody},
        SupportMismatchRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<SupportMismatchRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedPrismDualVerticesTest : public ::testing::TestWithParam<SupportMismatchRepresentativeCase>
{
};

TEST_P(ClosedPrismDualVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 5);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 5);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 6);
    ASSERT_EQ(result.body.EdgeCount(), 9);

    const double s = 1e-5;
    const double expectedV4X = s + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s && point.z > 0.5 * s &&
            point.z < 1.5 * s)
        {
            ++sharedV4Count;
            ASSERT_LT(std::abs(point.x - expectedV4X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
    ASSERT_EQ(sharedV4Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedPrismDualVerticesTest,
    ::testing::Values(
        SupportMismatchRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedPrismDualVerticesBody},
        SupportMismatchRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDuplicateLoopBody},
        SupportMismatchRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<SupportMismatchRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedPrismAllVerticesTest : public ::testing::TestWithParam<SupportMismatchRepresentativeCase>
{
};

TEST_P(ClosedPrismAllVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 5);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 5);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 6);
    ASSERT_EQ(result.body.EdgeCount(), 9);

    const double s = 1e-5;
    const double expectedV1X = s + 1.0e-7 / 3.0;
    const double expectedV4X = s + 2.0e-7 / 3.0;

    std::size_t sharedV1Count = 0;
    std::size_t sharedV4Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s)
        {
            ++sharedV1Count;
            ASSERT_LT(std::abs(point.x - expectedV1X), 1e-10);
        }
        if (std::abs(point.y) < 1e-12 && point.x > 0.5 * s && point.x < 1.5 * s && point.z > 0.5 * s &&
            point.z < 1.5 * s)
        {
            ++sharedV4Count;
            ASSERT_LT(std::abs(point.x - expectedV4X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV1Count, 1);
    ASSERT_EQ(sharedV4Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedPrismAllVerticesTest,
    ::testing::Values(
        SupportMismatchRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedPrismAllVerticesBody},
        SupportMismatchRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDuplicateLoopBody},
        SupportMismatchRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<SupportMismatchRepresentativeCase>& info) {
        return std::string(info.param.name);
    });
