# 鏈€缁堝懡鍚嶈鍒?
鏈枃瀹氫箟褰撳墠鍙戝竷鐗?Geometry 鍏叡闈㈢殑鍛藉悕瑙勫垯銆?
## 鎬诲垯

- `SC` 鍙繚鐣欑粰宸ョ▼韬唤浣跨敤锛屼緥濡備粨搴撳悕銆佽В鍐虫柟妗堝悕銆丆Make target銆佸寘鍚嶅拰鍐呴儴鏋勫缓鏍囪瘑銆?- 鍏叡绫诲瀷鍚嶃€佸叕鍏卞嚱鏁板悕銆佸叕鍏辨暟鎹粨鏋勪笉浣跨敤 `SC` 鍓嶇紑銆?- 榛樿鍙戝竷闈笉浣跨敤 `ISC`銆?- 闇€瑕佽法 DLL 鐨勫叕鍏?ABI 绋冲畾鏁版嵁缁撴瀯缁х画閫氳繃 `GEOMETRY_API` 瀵煎嚭銆?- 鍏叡闈㈡寜鐩綍鍜岃涔夊懡鍚嶏紝涓嶆寜浜у搧鍓嶇紑缁勭粐銆?
## 鍛藉悕绌洪棿

- 褰撳墠鍏叡鍛藉悕绌洪棿涓?`Geometry`銆?- `Geometry::Sdk` 鍙厑璁镐綔涓鸿縼绉绘湡闂寸殑涓存椂鍒悕銆?- 鏈枃妗ｉ€傜敤浜庢簮鐮佹爲涓殑鏂囦欢銆佺被鍨嬨€佺粨鏋滃拰鍏叡 API锛屼笉閫傜敤浜庡畨瑁呭悗鐨勭洰褰曞墠缂€銆?
### 鍏煎绛栫暐

- 鍙繚鐣欎竴涓粠 `Geometry::Sdk` 鎸囧悜 `Geometry` 鐨勮繃娓″埆鍚嶃€?- 涓嶆仮澶?`SC*` 鎴?`ISC*` 鍏煎绫汇€?- 涓嶆妸鍏煎澶存斁杩涢粯璁ゅ畨瑁呴潰銆?- 杩佺Щ鏈熺粨鏉熷悗鍒犻櫎鍒悕銆?
## 婧愮爜 Include/Core

淇濈暀锛?
- `GeometryApi.h`
- `GeometryTypes.h`
- `Algorithms.h`
- `AxisOps.h`
- `Boolean.h`
- `Editing.h`
- `Intersection.h`
- `Measure.h`
- `Metrics.h`
- `Offset.h`
- `Projection.h`
- `Relation.h`
- `Results.h`
- `Sampling.h`
- `SearchPoly.h`
- `Section.h`
- `ShapeOps.h`
- `Transform.h`
- `Validation.h`

鍒犻櫎锛?
- `SC*` 鍏煎鍚嶅瓧
- `ISC*` 鍏煎鍚嶅瓧

## 婧愮爜 Include/Geometry2d

淇濈暀锛?
- `SCArcSegment2d.h`
- `SCBoxTree2d.h`
- `SCCircle2d.h`
- `SCEllipse2d.h`
- `SCKDTree2d.h`
- `SCLineSegment2d.h`
- `SCMultiPolygon2d.h`
- `SCMultiPolyline2d.h`
- `SCPathOps.h`
- `SCPolygon2d.h`
- `SCPolyline2d.h`
- `SCRectangle2d.h`
- `ISCSegment2d.h`
- `SCSegmentSearch2d.h`

鍒犻櫎锛?
- `SCArcSegment2d.h`
- `SCCircle2d.h`
- `SCLineSegment2d.h`
- `SCMultiPolygon2d.h`
- `SCMultiPolyline2d.h`
- `SCSegment2d.h`
- `ISCPolygon2d.h`
- `ISCPolyline2d.h`

## 婧愮爜 Include/Geometry3d

淇濈暀锛?
- `ISCCurve3d.h`
- `SCCurveOnSurface.h`
- `SCLineCurve3d.h`
- `SCNurbsCurve3d.h`
- `SCNurbsSurface.h`
- `SCOffsetSurface.h`
- `SCPlaneSurface.h`
- `SCRuledSurface.h`
- `ISCSurface.h`
- `TriangleMesh.h`

鍒犻櫎锛?
- `ISCCurve3d.h`

## 婧愮爜 Include/Brep

淇濈暀锛?
- `BodyBoolean.h`
- `BrepBody.h`
- `BrepCoedge.h`
- `BrepConversion.h`
- `BrepEdge.h`
- `BrepEditing.h`
- `BrepFace.h`
- `BrepLoop.h`
- `BrepShell.h`
- `BrepVertex.h`
- `Healing.h`
- `MeshConversion.h`
- `MeshOps.h`
- `MeshRepair.h`
- `PolyhedronBody.h`
- `PolyhedronFace3d.h`
- `PolyhedronLoop3d.h`
- `Tessellation.h`
- `Topology.h`

鍒犻櫎锛?
- 涓嶅啀淇濈暀棰濆鐨勫叕鍏卞寘瑁呭ご

## 婧愮爜 Include/Support 涓?Include/Types

淇濈暀锛?
- `Support/Epsilon.h`
- `Support/Geometry2d/Normalize2.h`
- `Support/Geometry2d/Predicate2.h`
- `Types/Geometry2d/*`
- `Types/Geometry3d/*`
- `Types/Detail/Segment2Detail.h`

鍒犻櫎锛?
- 鏃犵敤鏁存暟鍒悕鍜屽吋瀹硅緟鍔?- 鎵€鏈?`SC*` / `ISC*` 鍏煎澶?
## ABI 璇存槑

- 闇€瑕佽法 DLL 鐨勫叕鍏卞€肩被鍨嬶紝搴旂户缁斁鍦ㄥ鍑哄ご閲屽苟淇濇寔 `GEOMETRY_API`銆?- 涓嶈涓轰簡 DLL 鍙鎬х粰鍊肩被鍨嬮澶栧姞 `SC` 鍓嶇紑銆?- 鍙緵鍐呴儴浣跨敤鐨勭被鍨嬶紝搴旀斁鍏?detail 澶存垨婧愭枃浠讹紝鑰屼笉鏄户缁斁杩涙簮鐮?`Include/`銆?
