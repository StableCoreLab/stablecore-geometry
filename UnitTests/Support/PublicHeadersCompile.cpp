// Compiles the complete installed public header surface in a consumer-like target.
#include "Core/PathGeometry.h"
#include "Core/ArcOps3d.h"
#include "Core/Offset.h"
#include "Geometry.h"
#include "Brep/BodyBoolean.h"
#include "Brep/BrepConversion.h"
#include "Brep/BrepEditing.h"
#include "Brep/Healing.h"
#include "Brep/MeshConversion.h"
#include "Brep/MeshOps.h"
#include "Brep/MeshRepair.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Brep/PolyhedronLoop3d.h"
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepCoedge.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepLoop.h"
#include "Brep/SCBrepShell.h"
#include "Brep/SCBrepVertex.h"
#include "Brep/Tessellation.h"
#include "Brep/Topology.h"
#include "Core/Algorithms.h"
#include "Core/AxisOps.h"
#include "Core/Boolean.h"
#include "Core/Editing.h"
#include "Core/GeometryApi.h"
#include "Core/GeometryEvidence.h"
#include <type_traits>
#include <cstddef>

template <typename T>
constexpr bool HasGeometryResultContract =
    std::is_same_v<decltype(T::state), Geometry::SCGeometryOperationState> &&
    std::is_same_v<decltype(T::payloadValid), bool> &&
    std::is_same_v<decltype(T::evidence), Geometry::SCGeometryEvidenceStatus> &&
    offsetof(T,state)<offsetof(T,payloadValid) &&
    offsetof(T,payloadValid)<offsetof(T,payload) && offsetof(T,payload)<offsetof(T,evidence);
#include "Core/GeometryTypes.h"
#include "Core/GeometryTypesFwd.h"
#include "Core/GeometryTypesPrimitives.h"
#include "Core/Intersection.h"
#include "Core/Measure.h"
#include "Core/Metrics.h"
#include "Core/PointOps.h"
#include "Core/PolygonRelation2d.h"
#include "Core/Projection.h"
#include "Core/Relation.h"
#include "Core/Results.h"
#include "Core/Sampling.h"
#include "Core/SearchPoly.h"
#include "Core/Section.h"
#include "Core/ShapeOps.h"
#include "Core/Snap3d.h"
#include "Core/Tangent.h"
#include "Core/Transform.h"
#include "Core/Triangulation.h"
#include "Core/Validation.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCBoxTree2d.h"
#include "Geometry2d/SCCircle2d.h"
#include "Geometry2d/SCEllipse2d.h"
#include "Geometry2d/SCKDTree2d.h"
#include "Geometry2d/SCLine2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCMultiPolyline2d.h"
#include "Geometry2d/SCPathOps.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Geometry2d/SCRectangle2d.h"
#include "Geometry2d/SCSegmentSearch2d.h"
#include "Geometry2d/SegmentTypes.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/ISCSurface.h"
#include "Geometry3d/SCBoxTree3d.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCOffsetSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/SCRuledSurface.h"
#include "Geometry3d/SCSegmentSearch3d.h"
#include "Geometry3d/TriangleMesh.h"
#include "Serialize/GeometryText.h"
#include "Support/Epsilon.h"
#include "Support/Geometry2d/Normalize2.h"
#include "Support/Geometry2d/Predicate2.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCMatrix2d.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCTransform2d.h"
#include "Types/Geometry2d/SCVector2.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCArcSegment3d.h"
#include "Types/Geometry3d/SCGeometryTypes3d.h"
#include "Types/Geometry3d/SCDirection3d.h"
#include "Types/Geometry3d/SCIntervald.h"
#include "Types/Geometry3d/SCLine3d.h"
#include "Types/Geometry3d/SCLineSegment3d.h"
#include "Types/Geometry3d/SCMatrix3d.h"
#include "Types/Geometry3d/SCPlane.h"
#include "Types/Geometry3d/SCPlaneFrame3d.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCRay3d.h"
#include "Types/Geometry3d/SCTransform3d.h"
#include "Types/Geometry3d/SCTriangle3d.h"
#include "Types/Geometry3d/SCVector3.h"

static_assert(true);

static_assert(HasGeometryResultContract<Geometry::SCLinePlaneIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPlanePlaneIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLineBrepEdgeIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLineBrepVertexIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPlaneBrepEdgeIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPlaneBrepVertexIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLineSurfaceIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLineCurveIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLineCurveOnSurfaceIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPlaneCurveIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPlaneCurveOnSurfaceIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLinePolyhedronFaceIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLineBrepFaceIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLineBrepBodyIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLinePolyhedronBodyIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCLineTriangleMeshIntersection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCBrepVertexProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCBrepFaceProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPolyhedronFaceProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCBrepBodyProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPolyhedronBodyProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCCurveProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCCurveOnSurfaceProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCSurfaceProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCBrepEdgeProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCTriangleMeshProjection3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPointContainment3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCPointPlaneSide3dWithStatus>);
static_assert(HasGeometryResultContract<Geometry::SCDistanceResult3dWithStatus>);
