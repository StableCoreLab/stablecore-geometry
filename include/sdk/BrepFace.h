#pragma once

#include <cstddef>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/BrepLoop.h"
#include "sdk/CurveOnSurface.h"

namespace geometry::sdk
{
class GEOMETRY_API BrepFace
{
public:
    BrepFace() = default;
    BrepFace(
        std::shared_ptr<Surface> supportSurface,
        BrepLoop outerLoop,
        std::vector<BrepLoop> holes = {},
        CurveOnSurface outerTrim = {},
        std::vector<CurveOnSurface> holeTrims = {})
        : supportSurface_(std::move(supportSurface)),
          outerLoop_(std::move(outerLoop)),
          holes_(std::move(holes)),
          outerTrim_(std::move(outerTrim)),
          holeTrims_(std::move(holeTrims))
    {
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const
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

            if (i < holeTrims_.size() && holeTrims_[i].SupportSurface() != nullptr && !holeTrims_[i].IsValid(tolerance))
            {
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] const Surface* SupportSurface() const
    {
        return supportSurface_.get();
    }

    [[nodiscard]] BrepLoop OuterLoop() const
    {
        return outerLoop_;
    }

    [[nodiscard]] std::size_t HoleCount() const
    {
        return holes_.size();
    }

    [[nodiscard]] BrepLoop HoleAt(std::size_t index) const
    {
        return holes_.at(index);
    }

    [[nodiscard]] const std::vector<BrepLoop>& HoleLoops() const
    {
        return holes_;
    }

    [[nodiscard]] const CurveOnSurface& OuterTrim() const
    {
        return outerTrim_;
    }

    [[nodiscard]] const std::vector<CurveOnSurface>& HoleTrims() const
    {
        return holeTrims_;
    }

    [[nodiscard]] Box3d Bounds() const
    {
        return supportSurface_ != nullptr ? supportSurface_->Bounds() : Box3d{};
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "BrepFace{holeCount=" << HoleCount() << "}";
        return stream.str();
    }

private:
    std::shared_ptr<Surface> supportSurface_{};
    BrepLoop outerLoop_{};
    std::vector<BrepLoop> holes_{};
    CurveOnSurface outerTrim_{};
    std::vector<CurveOnSurface> holeTrims_{};
};
} // namespace geometry::sdk
