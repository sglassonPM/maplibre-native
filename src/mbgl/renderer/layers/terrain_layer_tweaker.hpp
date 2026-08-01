#pragma once

#include <mbgl/util/noncopyable.hpp>
#include <mbgl/shaders/shader_defines.hpp>

#include <memory>
#include <string>

namespace mbgl {

class RenderTerrain;
class LayerGroupBase;
class PaintParameters;

namespace gfx {
class UniformBuffer;
using UniformBufferPtr = std::shared_ptr<UniformBuffer>;
} // namespace gfx

/**
 * Terrain layer specific tweaker - updates UBOs for terrain rendering
 * Note: This is NOT a LayerTweaker because terrain is not a regular layer
 */
class TerrainLayerTweaker : util::noncopyable {
public:
    /// Isomaps : sinkMeters > 0 = mode SOUS-COUCHE anti-fissures (« terrain-under ») — la nappe est
    /// enfoncée de sinkMeters (translation d'élévation dans la matrice, mètres exagérés) et les
    /// liaisons DEM/satellite sont lues dans les cartes DÉDIÉES de la sous-couche (ids partagés avec
    /// la surface → cartes séparées obligatoires).
    explicit TerrainLayerTweaker(const RenderTerrain* terrain_, double sinkMeters_ = 0.0)
        : terrain(terrain_),
          sinkMeters(sinkMeters_) {}

    ~TerrainLayerTweaker() = default;

    void execute(LayerGroupBase&, const PaintParameters&);

protected:
#if MLN_UBO_CONSOLIDATION
    gfx::UniformBufferPtr drawableUniformBuffer;
#endif

    const RenderTerrain* terrain = nullptr;
    double sinkMeters = 0.0;
};

} // namespace mbgl
