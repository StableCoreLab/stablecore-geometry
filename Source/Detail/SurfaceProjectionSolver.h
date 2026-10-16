#pragma once

#include <array>
#include <bit>
#include <map>
#include <vector>
#include "Core/Projection.h"
#include "Geometry3d/SCNurbsSurface.h"

namespace Geometry::Detail
{
    struct SurfaceSearchBudget
    {
        SCGeometrySearchStatistics statistics{};
        bool bounded{false};
        struct PositionGroup
        {
            struct Entry { std::array<double,4> key{}; SCPoint3d point{}; bool occupied{false}; };
            std::vector<Entry> entries{};
            std::size_t count{0};
            static std::size_t Hash(const std::array<double,4>& key)
            {
                const auto bits=[](double x) { return x==0 ? std::uint64_t{0} : std::bit_cast<std::uint64_t>(x); };
                auto h=bits(key[0]) ^ std::rotl(bits(key[1]),13) ^ std::rotl(bits(key[2]),29) ^ std::rotl(bits(key[3]),47);
                h^=h>>30; h*=0xbf58476d1ce4e5b9ULL; h^=h>>27;
                return static_cast<std::size_t>(h^(h>>31));
            }
            Entry* Slot(const std::array<double,4>& key)
            {
                auto index=Hash(key)&(entries.size()-1);
                auto* data=entries.data();
                const auto* k=key.data();
                while (data[index].occupied) {
                    const auto* other=data[index].key.data();
                    if (k[0]==other[0] && k[1]==other[1] && k[2]==other[2] && k[3]==other[3]) break;
                    index=(index+1)&(entries.size()-1);
                }
                return data+index;
            }
            const SCPoint3d* Find(const std::array<double,4>& key)
            {
                if (entries.empty()) return nullptr;
                auto* slot=Slot(key);
                return slot->occupied?&slot->point:nullptr;
            }
            void Insert(const std::array<double,4>& key, const SCPoint3d& point)
            {
                if (entries.empty()) entries.resize(16);
                if ((count+1)*2>entries.size()) {
                    auto previous=std::move(entries);
                    entries.resize(previous.size()*2);
                    for (const auto& entry : previous) if (entry.occupied) *Slot(entry.key)=entry;
                }
                auto* slot=Slot(key);
                if (!slot->occupied) ++count;
                *slot={key,point,true};
            }
        };
        std::vector<PositionGroup> positionGroups{};
        std::vector<std::size_t> positionGroupIndices{};

        void PreparePositionCache(const SCNurbsSurface& surface)
        {
            if (surface.DegreeU() != 1 || surface.DegreeV() != 1) return;
            const auto nu = surface.ControlPointCountU(), nv = surface.ControlPointCountV();
            positionGroupIndices.resize(nu * nv);
            std::map<std::array<double, 16>, std::size_t> groups;
            for (std::size_t v = 1; v < nv; ++v) for (std::size_t u = 1; u < nu; ++u) {
                std::array<double, 16> key{};
                std::size_t entry = 0;
                for (std::size_t j = v-1; j <= v; ++j) for (std::size_t i = u-1; i <= u; ++i) {
                    const auto index = j * nu + i;
                    const auto p = surface.ControlPoints()[index];
                    key[entry++] = p.x; key[entry++] = p.y; key[entry++] = p.z;
                    key[entry++] = surface.Weights()[index];
                }
                const auto [it, inserted] = groups.emplace(key, groups.size());
                if (inserted) positionGroups.emplace_back();
                positionGroupIndices[v * nu + u] = it->second;
            }
        }

        SCPoint3d Position(const SCNurbsSurface& surface, double u, double v)
        {
            std::array<double, 4> key{};
            PositionGroup* group = nullptr;
            if (!positionGroups.empty()) {
                const auto basis = [](const auto& knots, std::size_t count, double t, double* values) {
                    return EvaluateValidatedLinearNurbsBasis(knots,count,t,values);
                };
                const auto su = basis(surface.KnotsU(), surface.ControlPointCountU(), u, key.data());
                const auto sv = basis(surface.KnotsV(), surface.ControlPointCountV(), v, key.data()+2);
                if (std::isfinite(key[0]) && std::isfinite(key[1]) && std::isfinite(key[2]) && std::isfinite(key[3])) {
                    group = &positionGroups[positionGroupIndices[sv * surface.ControlPointCountU() + su]];
                    const auto* found = group->Find(key);
                    if (found != nullptr) {
                        return *found;
                    }
                }
            }
            if (!Consume(0)) return {std::numeric_limits<double>::quiet_NaN(),0,0};
            const auto point=EvaluateValidatedNurbsSurface(surface,u,v,0).point;
            if (group!=nullptr && point.IsValid()) group->Insert(key,point);
            return point;
        }

        SCSurfaceEval3d Evaluate(const SCNurbsSurface& surface, double u, double v, int order)
        {
            if (order==0) { SCSurfaceEval3d result{}; result.point=Position(surface,u,v); return result; }
            if (!Consume(order)) {
                SCSurfaceEval3d invalid{};
                invalid.point.x=std::numeric_limits<double>::quiet_NaN();
                return invalid;
            }
            return EvaluateValidatedNurbsSurface(surface,u,v,order);
        }

        bool Consume(int order)
        {
            if (bounded && (statistics.positionEvaluations >= 2000000 ||
                (order > 0 && statistics.derivativeEvaluations >= 1000000)))
            {
                statistics.searchIncomplete = true;
                return false;
            }
            ++statistics.positionEvaluations;
            if (order > 0) ++statistics.derivativeEvaluations;
            return true;
        }
    };

    // 内部候选允许平方不可表示；公开投影只在平方有效时发布。
    [[nodiscard]] SCSurfaceProjection3d SolveSurfaceProjection(
        const SCPoint3d& point, const ISCSurface& surface,
        const SCGeometryTolerance3d& tolerance, SurfaceSearchBudget& budget);
}
