#pragma once

#include <mbgl/util/geometry.hpp>
#include <mbgl/util/mat3.hpp>
#include <mbgl/util/mat4.hpp>

namespace mbgl {
namespace util {

enum class IntersectionResult : int {
    Separate,
    Intersects,
    Contains,
};

class AABB {
public:
    AABB() noexcept;
    AABB(const vec3& min_, const vec3& max_) noexcept;

    constexpr vec3 closestPoint(const vec3& point) const noexcept;

    // Computes the shortest manhattan distance to the provided point
    vec3 distanceXYZ(const vec3& point) const noexcept;

    // Creates an aabb covering one quadrant of the aabb on XZ-plane.
    AABB quadrant(int idx) const noexcept;

    bool intersects(const AABB& aabb) const noexcept;

    bool operator==(const AABB& aabb) const noexcept;
    bool operator!=(const AABB& aabb) const noexcept;

    vec3 min;
    vec3 max;
};

class Frustum {
public:
    Frustum(const std::array<vec3, 8>& points_, const std::array<vec4, 6>& planes_);

    // Isomaps : `pixelsPerMeter` remet le Z des coins dans les unités-tuile des AABB élevées. La matrice de
    // projection consomme un monde en (x,y pixels, z MÈTRES) — getWorldToCamera post-multiplie z par
    // pixelsPerMeter — donc son inverse rend des z en mètres ; le facteur commun scale/worldSize ne vaut que
    // pour x,y. Sans cette correction le frustum est écrasé en Z (÷pixelsPerMeter, ~×4 aux latitudes alpines)
    // et cull À TORT toute boîte dont le terrain dépasse ~œil/4 (invisible en amont : AABB toujours à z=0).
    static Frustum fromInvProjMatrix(
        const mat4& invProj, double worldSize, double zoom, bool flippedY = false, double pixelsPerMeter = 1.0);

    // Performs conservative intersection test using separating axis theorem.
    // Some accuracy is traded for better performance. False positive rate is < 1%
    IntersectionResult intersects(const AABB& aabb) const;

    // Performs precise intersection test using separating axis theorem.
    // It is possible run only edge cases that were not covered in intersects()
    IntersectionResult intersectsPrecise(const AABB& aabb, bool edgeCasesOnly = false) const;

    // Intersection test that accounts for the aabb's elevation (min/max z), for
    // testing terrain against the view volume. `intersects` and `intersectsPrecise`
    // flatten the box to z = 0; this tests all three axes, via a plane/aabb halfspace
    // test against each frustum plane (the p-vertex/n-vertex method), matching
    // maplibre-gl-js Aabb::intersectsFrustum. Use this only when z may be non-zero:
    // it is a little more work and the flat fast paths are enough otherwise.
    IntersectionResult intersectsElevated(const AABB& aabb) const;

    const std::array<vec3, 8>& getPoints() const { return points; }
    const std::array<vec4, 6>& getPlanes() const { return planes; }

private:
    struct Projection {
        vec3 axis;
        Point<double> projection;
    };

    AABB bounds;
    std::array<vec3, 8> points;
    std::array<vec4, 6> planes;
    std::array<Projection, 12> projections;
};

} // namespace util
} // namespace mbgl
