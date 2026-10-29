#include <cmath>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/ConversionFixtures3d.h"
#include "Support/Fixtures3d.h"

using Geometry::BrepConversionIssue3d;
using Geometry::ConvertToBrepBody;
using Geometry::ConvertToTriangleMesh;
using Geometry::MeshConversionIssue3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronBrepBodyConversion3d;
using Geometry::PolyhedronMeshConversion3d;

using namespace Geometry::Test::ConversionFixtures;

// 验证封闭单位立方体转换为三角网格后的三角面数和总面积。
TEST(Conversion3dCapabilityTest, UnitCubePolyhedronBodyConvertsToTriangleMesh)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());
    ASSERT_EQ(cubeBody.FaceCount(), 6);

    const PolyhedronMeshConversion3d result = ConvertToTriangleMesh(cubeBody);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(result.mesh.IsValid());
    // 六个四边形面各拆为两个三角面。
    ASSERT_EQ(result.mesh.TriangleCount(), 12);
    // 单位立方体的表面积为 6.0。
    ASSERT_LT(std::abs(result.mesh.SurfaceArea() - 6.0), 1e-10);
}

// 验证有效多面体转换为平面直线 B-rep 后保留面数量。
TEST(Conversion3dCapabilityTest, UnitCubePolyhedronBodyConvertsToBrepBody)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());
    ASSERT_EQ(cubeBody.FaceCount(), 6);

    const PolyhedronBrepBodyConversion3d result = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.ShellCount(), 1);
    ASSERT_TRUE(result.body.ShellAt(0).IsClosed());
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);
}

// 验证倾斜立方体可以转换为 B-rep。
TEST(Conversion3dCapabilityTest, SkewedCubePolyhedronBodyConvertsToBrepBody)
{
    PolyhedronBody skewedBody;
    ASSERT_NO_THROW(skewedBody = BuildSkewedUnitCubeBody());
    ASSERT_TRUE(skewedBody.IsValid());
    ASSERT_EQ(skewedBody.FaceCount(), 6);

    PolyhedronBrepBodyConversion3d result{};
    ASSERT_NO_THROW(result = ConvertToBrepBody(skewedBody));
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.issue, BrepConversionIssue3d::None);

    bool bodyValid = false;
    ASSERT_NO_THROW(bodyValid = result.body.IsValid());
    ASSERT_TRUE(bodyValid);

    ASSERT_EQ(result.body.FaceCount(), 6);
    ASSERT_EQ(result.body.VertexCount(), 8);
    ASSERT_EQ(result.body.EdgeCount(), 12);

    std::size_t shellCount = 0;
    ASSERT_NO_THROW(shellCount = result.body.ShellCount());
    ASSERT_EQ(shellCount, 1u);
}
