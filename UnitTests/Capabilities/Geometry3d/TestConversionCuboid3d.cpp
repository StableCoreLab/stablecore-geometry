#include <cmath>
#include <ostream>
#include <string>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/ConversionFixtures3d.h"

using Geometry::BrepConversionIssue3d;
using Geometry::ConvertToBrepBody;
using Geometry::SCPoint3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronBrepBodyConversion3d;

using namespace Geometry::Test::ConversionFixtures;

struct ClosedCuboidRepresentativeCase
{
    const char* name;
    PolyhedronBody (*build)();
};

void PrintTo(const ClosedCuboidRepresentativeCase& scenario, std::ostream* stream)
{
    *stream << scenario.name;
}

class ClosedCuboidAllVerticesTest : public ::testing::TestWithParam<ClosedCuboidRepresentativeCase>
{
};

TEST_P(ClosedCuboidAllVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);
    AssertClosedCuboidAllVerticesRepresentativeTargets(result.body);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedCuboidAllVerticesTest,
    ::testing::Values(
        ClosedCuboidRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedCuboidAllVerticesBody},
        ClosedCuboidRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDuplicateLoopBody},
        ClosedCuboidRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<ClosedCuboidRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedCuboidDualVerticesTest : public ::testing::TestWithParam<ClosedCuboidRepresentativeCase>
{
};

TEST_P(ClosedCuboidDualVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-12 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            ASSERT_LT(std::abs(point.x - expectedV6X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
    ASSERT_EQ(sharedV6Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedCuboidDualVerticesTest,
    ::testing::Values(
        ClosedCuboidRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedCuboidDualVerticesBody},
        ClosedCuboidRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDuplicateLoopBody},
        ClosedCuboidRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<ClosedCuboidRepresentativeCase>& info) {
        return std::string(info.param.name);
    });

class ClosedCuboidTripleVerticesTest : public ::testing::TestWithParam<ClosedCuboidRepresentativeCase>
{
};

TEST_P(ClosedCuboidTripleVerticesTest, PreservesRepresentativeAverageTarget)
{
    const PolyhedronBody body = GetParam().build();
    ASSERT_FALSE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(body);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);

    const double s = 1e-5;
    const double w = 2.0 * s;
    const double expectedV1X = w;
    const double expectedV6X = w + 1.0e-7 / 3.0;

    std::size_t sharedV0Count = 0;
    std::size_t sharedV1Count = 0;
    std::size_t sharedV6Count = 0;
    for (std::size_t i = 0; i < result.body.VertexCount(); ++i)
    {
        const SCPoint3d point = result.body.VertexAt(i).Point();
        if (std::abs(point.x) < 1e-10 && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV0Count;
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y) < 1e-12 && std::abs(point.z) < 1e-12)
        {
            ++sharedV1Count;
            ASSERT_LT(std::abs(point.x - expectedV1X), 1e-10);
        }
        if (point.x > 1.5 * s && point.x < 2.5 * s && std::abs(point.y - s) < 1e-12 && std::abs(point.z - s) < 1e-12)
        {
            ++sharedV6Count;
            ASSERT_LT(std::abs(point.x - expectedV6X), 1e-10);
        }
    }

    ASSERT_EQ(sharedV0Count, 1);
    ASSERT_EQ(sharedV1Count, 1);
    ASSERT_EQ(sharedV6Count, 1);
}

INSTANTIATE_TEST_SUITE_P(
    Conversion3d, ClosedCuboidTripleVerticesTest,
    ::testing::Values(
        ClosedCuboidRepresentativeCase{"Base", &BuildSupportMismatchNearEqualClosedCuboidTripleVerticesBody},
        ClosedCuboidRepresentativeCase{"DuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDuplicateLoopBody},
        ClosedCuboidRepresentativeCase{"DualDuplicateLoop", &BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDualDuplicateLoopBody}),
    [](const ::testing::TestParamInfo<ClosedCuboidRepresentativeCase>& info) {
        return std::string(info.param.name);
    });
