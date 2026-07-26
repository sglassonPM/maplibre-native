#pragma once

#include <mbgl/renderer/layer_tweaker.hpp>
#include <mbgl/tile/tile_id.hpp>
#include <mbgl/util/containers.hpp>
#include <mbgl/util/hash.hpp>

#include <string>
#include <memory>

namespace mbgl {
namespace gfx { class Texture2D; }

/**
    Symbol layer specific tweaker
 */
class SymbolLayerTweaker : public LayerTweaker {
public:
    SymbolLayerTweaker(std::string id_, Immutable<style::LayerProperties> properties)
        : LayerTweaker(std::move(id_), properties) {}
    ~SymbolLayerTweaker() override = default;

    void execute(LayerGroupBase&, const PaintParameters&) override;

private:
    gfx::UniformBufferPtr evaluatedPropsUniformBuffer;

    // Texture 1x1 liee aux slots DEM/profondeur des symboles quand il n'y a PAS de terrain :
    // le shader symbole declare toujours demTexture/depthTexture + depthSampler (occlusion terrain),
    // il faut donc lier quelque chose a ces slots meme sans terrain, sinon la validation Metal
    // (Debug) echoue « missing Sampler binding ». dem_enabled=0 => la texture n'est jamais utilisee.
    std::shared_ptr<gfx::Texture2D> placeholderTerrainTexture;

#if MLN_UBO_CONSOLIDATION
    gfx::UniformBufferPtr drawableUniformBuffer;
    gfx::UniformBufferPtr tilePropsUniformBuffer;
#endif
};

} // namespace mbgl
