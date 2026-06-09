#pragma once

#include <cstddef>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Core/GeometryTypesPrimitives.h"
#include "Brep/SCBrepLoop.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCPlane.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    class GEOMETRY_API SCBrepFace
    {
    public:
        SCBrepFace() = default;
        SCBrepFace(std::shared_ptr<ISCSurface> supportSurface,
                 SCBrepLoop outerLoop,
                 std::vector<SCBrepLoop> holes = {},
                 SCCurveOnSurface outerTrim = {},
                 std::vector<SCCurveOnSurface> holeTrims = {})
            : supportSurface_(std::move(supportSurface)),
              outerLoop_(std::move(outerLoop)),
              holes_(std::move(holes)),
              outerTrim_(std::move(outerTrim)),
              holeTrims_(std::move(holeTrims))
        {
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            if (supportSurface_ == nullptr || !supportSurface_->IsValid(tolerance) || !outerLoop_.IsValid())
            {
                return false;
            }

            if ((outerTrim_.SupportSurface() != nullptr && !outerTrim_.IsValid(tolerance)) ||
                (!holeTrims_.empty() && holeTrims_.size() != holes_.size()))
            {
                return false;
            }

            for (std::size_t i = 0; i < holes_.size(); ++i)
            {
                if (!holes_[i].IsValid())
                {
                    return false;
                }

                if (i < holeTrims_.size() && holeTrims_[i].SupportSurface() != nullptr &&
                    !holeTrims_[i].IsValid(tolerance))
                {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] const ISCSurface* SupportSurface() const
        {
            return supportSurface_.get();
        }

        [[nodiscard]] SCBrepLoop OuterLoop() const
        {
            return outerLoop_;
        }

        [[nodiscard]] std::size_t HoleCount() const
        {
            return holes_.size();
        }

        [[nodiscard]] SCBrepLoop HoleAt(std::size_t index) const
        {
            return holes_.at(index);
        }

        [[nodiscard]] const std::vector<SCBrepLoop>& HoleLoops() const
        {
            return holes_;
        }

        [[nodiscard]] const SCCurveOnSurface& OuterTrim() const
        {
            return outerTrim_;
        }

        [[nodiscard]] const std::vector<SCCurveOnSurface>& HoleTrims() const
        {
            return holeTrims_;
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            if (outerTrim_.IsValid())
            {
                SCBox3d bounds = outerTrim_.Bounds();
                for (const SCCurveOnSurface& trim : holeTrims_)
                {
                    const SCBox3d trimBounds = trim.Bounds();
                    if (!trimBounds.IsValid())
                    {
                        continue;
                    }

                    bounds.ExpandToInclude(trimBounds.MinPoint());
                    bounds.ExpandToInclude(trimBounds.MaxPoint());
                }
                return bounds;
            }

            return supportSurface_ != nullptr ? supportSurface_->Bounds() : SCBox3d{};
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCBrepFace{holeCount=" << HoleCount() << "}";
            return stream.str();
        }

    private:
        std::shared_ptr<ISCSurface> supportSurface_{};
        SCBrepLoop outerLoop_{};
        std::vector<SCBrepLoop> holes_{};
        SCCurveOnSurface outerTrim_{};
        std::vector<SCCurveOnSurface> holeTrims_{};
    };
}  // namespace Geometry





