#pragma once
#include <string>
#include <vector>
#include "Brep/SCBrepBody.h"
#include "Brep/MeshConversion.h"
#include "Brep/PolyhedronBody.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    enum class SCSectionIssue3d
    {
        None,
        InvalidPlane,
        InvalidBody,
        MeshConversionFailed,
        CoplanarGeometryUnsupported,
        OpenContour,
        NonManifoldContour,
        InvalidContour
    };

    enum class SCSectionFaceRebuildIssue3d
    {
        None,
        InvalidSection,
        InvalidPolygon
    };

    enum class SCSectionBodyRebuildIssue3d
    {
        None,
        InvalidSection,
        FaceRebuildFailed
    };

    struct GEOMETRY_API SCSectionBodySetRebuildResult3d
    {
        bool success{false};
        SCSectionBodyRebuildIssue3d issue{SCSectionBodyRebuildIssue3d::None};
        std::vector<std::size_t> rootPolygonIndices{};
        std::vector<PolyhedronBody> bodies{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            if (!success)
            {
                return true;
            }

            if (bodies.size() != rootPolygonIndices.size())
            {
                return false;
            }

            for (const PolyhedronBody& body : bodies)
            {
                if (!body.IsValid(eps))
                {
                    return false;
                }
            }

            return true;
        }
    };

    enum class SCSectionContentKind3d
    {
        Empty,
        Curve,
        Area,
        Mixed
    };

    struct GEOMETRY_API SCSectionPolyline3d
    {
        bool closed{false};
        std::vector<SCPoint3d> points{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            if (closed ? points.size() < 3 : points.size() < 2)
            {
                return false;
            }

            for (std::size_t i = 0; i < points.size(); ++i)
            {
                if (!points[i].IsValid())
                {
                    return false;
                }

                if (i > 0 && points[i].AlmostEquals(points[i - 1], eps))
                {
                    return false;
                }
            }

            return true;
        }
    };

    struct GEOMETRY_API SCPolyhedronSection3d
    {
        bool success{false};
        SCSectionIssue3d issue{SCSectionIssue3d::None};
        std::vector<SCLineSegment3d> segments{};
        std::vector<SCSectionPolyline3d> contours{};
        std::vector<SCPolygon2d> polygons{};
        SCPoint3d origin{};
        SCVector3d uAxis{};
        SCVector3d vAxis{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            if (!success)
            {
                return true;
            }

            if (!origin.IsValid() || !uAxis.IsValid() || !vAxis.IsValid() || uAxis.Length() <= eps ||
                vAxis.Length() <= eps)
            {
                return false;
            }

            for (const SCLineSegment3d& segment : segments)
            {
                if (!segment.IsValid(eps))
                {
                    return false;
                }
            }

            for (const SCSectionPolyline3d& contour : contours)
            {
                if (!contour.IsValid(eps))
                {
                    return false;
                }
            }

            for (const SCPolygon2d& polygon : polygons)
            {
                if (!polygon.IsValid())
                {
                    return false;
                }
            }

            return true;
        }
    };

    struct GEOMETRY_API SCSectionFaceRebuildResult3d
    {
        bool success{false};
        SCSectionFaceRebuildIssue3d issue{SCSectionFaceRebuildIssue3d::None};
        struct FaceMapping
        {
            std::size_t outerPolygonIndex{0};
            std::vector<std::size_t> holePolygonIndices{};
        };
        std::vector<PolyhedronFace3d> faces{};
        std::vector<FaceMapping> mappings{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            if (!success)
            {
                return true;
            }

            if (faces.size() != mappings.size())
            {
                return false;
            }

            for (const PolyhedronFace3d& face : faces)
            {
                if (!face.IsValid(eps))
                {
                    return false;
                }
            }

            return true;
        }
    };

    struct GEOMETRY_API SCSectionBodyRebuildResult3d
    {
        bool success{false};
        SCSectionBodyRebuildIssue3d issue{SCSectionBodyRebuildIssue3d::None};
        PolyhedronBody body{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            return !success || body.IsValid(eps) || body.IsEmpty();
        }
    };

    struct GEOMETRY_API SCSectionBrepFaceRebuildResult3d
    {
        bool success{false};
        SCSectionFaceRebuildIssue3d issue{SCSectionFaceRebuildIssue3d::None};
        std::vector<SCBrepFace> faces{};

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            if (!success)
            {
                return true;
            }

            for (const SCBrepFace& face : faces)
            {
                if (!face.IsValid(tolerance))
                {
                    return false;
                }
            }

            return true;
        }
    };

    struct GEOMETRY_API SCSectionBrepBodyRebuildResult3d
    {
        bool success{false};
        SCSectionBodyRebuildIssue3d issue{SCSectionBodyRebuildIssue3d::None};
        SCBrepBody body{};

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            return !success || body.IsValid(tolerance) || body.IsEmpty();
        }
    };

    struct GEOMETRY_API SCSectionBrepBodySetRebuildResult3d
    {
        bool success{false};
        SCSectionBodyRebuildIssue3d issue{SCSectionBodyRebuildIssue3d::None};
        std::vector<std::size_t> rootPolygonIndices{};
        std::vector<SCBrepBody> bodies{};

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            if (!success)
            {
                return true;
            }

            if (rootPolygonIndices.size() != bodies.size())
            {
                return false;
            }

            for (const SCBrepBody& body : bodies)
            {
                if (!body.IsValid(tolerance))
                {
                    return false;
                }
            }

            return true;
        }
    };

    struct GEOMETRY_API SCSectionTopologyNode3d
    {
        std::size_t polygonIndex{0};
        std::size_t parentIndex{static_cast<std::size_t>(-1)};
        std::size_t depth{0};
        std::vector<std::size_t> children{};
    };

    class GEOMETRY_API SCSectionTopology3d
    {
    public:
        [[nodiscard]] bool IsValid() const
        {
            return valid_;
        }

        [[nodiscard]] std::size_t Count() const
        {
            return nodes_.size();
        }

        [[nodiscard]] bool IsEmpty() const
        {
            return nodes_.empty();
        }

        [[nodiscard]] const SCSectionTopologyNode3d& Node(std::size_t index) const
        {
            return nodes_.at(index);
        }

        [[nodiscard]] std::size_t ParentOf(std::size_t index) const
        {
            return nodes_.at(index).parentIndex;
        }

        [[nodiscard]] const std::vector<std::size_t>& ChildrenOf(std::size_t index) const
        {
            return nodes_.at(index).children;
        }

        [[nodiscard]] const std::vector<std::size_t>& Roots() const
        {
            return roots_;
        }

        [[nodiscard]] std::string DebugString() const;

    private:
        friend GEOMETRY_API SCSectionTopology3d BuildSectionTopology(const SCPolyhedronSection3d& section, double eps);

        bool valid_{false};
        std::vector<SCSectionTopologyNode3d> nodes_{};
        std::vector<std::size_t> roots_{};
    };

    struct GEOMETRY_API SCSectionComponent3d
    {
        std::size_t rootPolygonIndex{0};
        std::vector<std::size_t> polygonIndices{};
        std::vector<std::size_t> faceIndices{};
    };

    struct GEOMETRY_API SCSectionComponents3d
    {
        bool valid{false};
        std::vector<SCSectionComponent3d> components{};

        [[nodiscard]] bool IsValid() const
        {
            return valid;
        }
    };

    struct GEOMETRY_API SCSectionMeshConversionResult3d
    {
        bool success{false};
        MeshConversionIssue3d issue{MeshConversionIssue3d::None};
        TriangleMesh mesh{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            return !success || mesh.IsValid(eps);
        }
    };

    struct GEOMETRY_API SCSectionMeshSetConversionResult3d
    {
        bool success{false};
        MeshConversionIssue3d issue{MeshConversionIssue3d::None};
        std::vector<std::size_t> rootPolygonIndices{};
        std::vector<TriangleMesh> meshes{};

        [[nodiscard]] bool IsValid(double eps = Geometry::kDefaultEpsilon) const
        {
            if (!success)
            {
                return true;
            }

            if (meshes.size() != rootPolygonIndices.size())
            {
                return false;
            }

            for (const TriangleMesh& mesh : meshes)
            {
                if (!mesh.IsValid(eps))
                {
                    return false;
                }
            }

            return true;
        }
    };

    using SCSectionBodySetRebuild3d = SCSectionBodySetRebuildResult3d;
    using SCSectionFaceRebuild3d = SCSectionFaceRebuildResult3d;
    using SCSectionBodyRebuild3d = SCSectionBodyRebuildResult3d;
    using SCSectionBrepFaceRebuild3d = SCSectionBrepFaceRebuildResult3d;
    using SCSectionBrepBodyRebuild3d = SCSectionBrepBodyRebuildResult3d;
    using SCSectionBrepBodySetRebuild3d = SCSectionBrepBodySetRebuildResult3d;
    using SCSectionMeshConversion3d = SCSectionMeshConversionResult3d;
    using SCSectionMeshSetConversion3d = SCSectionMeshSetConversionResult3d;

    [[nodiscard]] GEOMETRY_API SCPolyhedronSection3d Section(const PolyhedronBody& body,
                                                           const SCPlane& plane,
                                                           const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCPolyhedronSection3d Section(const SCBrepBody& body,
                                                           const SCPlane& plane,
                                                           const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCSectionFaceRebuildResult3d RebuildSectionFaces(const SCPolyhedronSection3d& section,
                                                                              double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionBrepFaceRebuildResult3d
    RebuildSectionBrepFaces(const SCPolyhedronSection3d& section, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionBodyRebuildResult3d RebuildSectionBody(const SCPolyhedronSection3d& section,
                                                                             double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionBrepBodyRebuildResult3d
    RebuildSectionBrepBody(const SCPolyhedronSection3d& section, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionBrepBodySetRebuildResult3d
    RebuildSectionBrepBodies(const SCPolyhedronSection3d& section, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionBodySetRebuildResult3d
    RebuildSectionBodies(const SCPolyhedronSection3d& section, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionTopology3d BuildSectionTopology(const SCPolyhedronSection3d& section,
                                                                      double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionMeshConversionResult3d
    ConvertSectionToTriangleMesh(const SCPolyhedronSection3d& section, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionMeshSetConversionResult3d
    ConvertSectionToTriangleMeshes(const SCPolyhedronSection3d& section, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionContentKind3d ClassifySectionContent(const SCPolyhedronSection3d& section,
                                                                           double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCSectionComponents3d BuildSectionComponents(const SCPolyhedronSection3d& section,
                                                                          double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry



