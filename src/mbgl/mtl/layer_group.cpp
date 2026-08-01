#include <mbgl/mtl/layer_group.hpp>

#include <mbgl/gfx/drawable_tweaker.hpp>
#include <mbgl/gfx/renderable.hpp>
#include <mbgl/gfx/renderer_backend.hpp>
#include <mbgl/gfx/upload_pass.hpp>
#include <mbgl/mtl/context.hpp>
#include <mbgl/mtl/drawable.hpp>
#include <mbgl/mtl/render_pass.hpp>
#include <mbgl/renderer/paint_parameters.hpp>
#include <mbgl/shaders/mtl/shader_program.hpp>
#include <mbgl/util/convert.hpp>

#include <Metal/Metal.hpp>

#include <optional>

namespace mbgl {
namespace mtl {

LayerGroup::LayerGroup(int32_t layerIndex_, std::size_t initialCapacity, std::string name_, bool renderToTerrain_)
    : mbgl::LayerGroup(layerIndex_, initialCapacity, std::move(name_), renderToTerrain_) {}

void LayerGroup::upload(gfx::UploadPass& uploadPass) {
    if (!enabled) {
        return;
    }

#if !defined(NDEBUG)
    const auto debugGroup = uploadPass.createDebugGroup(getName() + "-upload");
#endif

    visitDrawables([&](gfx::Drawable& drawable) {
        if (drawable.getEnabled()) {
            auto& drawableMTL = static_cast<mtl::Drawable&>(drawable);
            drawableMTL.upload(uploadPass);
        }
    });
}

void LayerGroup::render(RenderOrchestrator&, PaintParameters& parameters) {
    if (!enabled || !getDrawableCount() || !parameters.renderPass) {
        return;
    }

#if !defined(NDEBUG)
    const auto debugGroup = parameters.encoder->createDebugGroup(getName() + "-render");
#endif

    auto& renderPass = static_cast<RenderPass&>(*parameters.renderPass);
    auto& context = static_cast<Context&>(parameters.context);
    const auto& renderable = renderPass.getDescriptor().renderable;

    // Isomaps : état de profondeur des drawables 3D (maillage terrain). Drawable::draw SAUTE tout le
    // bloc depth-stencil pour is3D (« handled by the layer group ») et TileLayerGroup le gère — mais ce
    // LayerGroup SIMPLE (celui du terrain) ne posait RIEN : les tuiles héritaient de l'état laissé par
    // le rendu précédent → test de profondeur aléatoire → l'ORDRE d'arrivée décidait (mesuré : sommet
    // correct, puis ÉCRASÉ par les tuiles du fond arrivées après). Convention reversed-Z du terrain
    // (near→1, far→0, clear 0, cf. mtl/terrain.hpp) → GreaterEqual + écriture.
    std::optional<MTLDepthStencilStatePtr> state3d, state3dNoDepth;
    const auto& getState3D = [&](bool depth) -> const MTLDepthStencilStatePtr& {
        if (depth) {
            if (!state3d) {
                const auto depthMode = gfx::DepthMode{.func = gfx::DepthFunctionType::GreaterEqual,
                                                      .mask = gfx::DepthMaskType::ReadWrite};
                state3d = context.makeDepthStencilState(depthMode, gfx::StencilMode::disabled(), renderable);
            }
            return *state3d;
        }
        if (!state3dNoDepth) {
            state3dNoDepth = context.makeDepthStencilState(
                gfx::DepthMode::disabled(), gfx::StencilMode::disabled(), renderable);
        }
        return *state3dNoDepth;
    };

    bool bindUBOs = false;
    visitDrawables([&](gfx::Drawable& drawable) {
        if (!drawable.getEnabled() || !drawable.hasRenderPass(parameters.pass)) {
            return;
        }

        if (!bindUBOs) {
            uniformBuffers.bindMtl(renderPass);
            bindUBOs = true;
        }

        for (const auto& tweaker : drawable.getTweakers()) {
            tweaker->execute(drawable, parameters);
        }

        // Restreint au groupe « terrain » (surface principale) : le jumeau « terrain-depth » rend dans sa
        // propre cible (pack de profondeur en COULEUR, pas de remap reversed-Z, pas d'attache depth) —
        // lui imposer cet état serait incorrect (et invalide côté Metal sans attache de profondeur).
        if (drawable.getIs3D() && getName() == "terrain") {
            renderPass.setDepthStencilState(getState3D(drawable.getEnableDepth()));
            // BIAIS DE PROFONDEUR PAR NIVEAU DE ZOOM : pendant la fenêtre où un parent (repli) et ses
            // enfants coexistent, leurs surfaces quasi-coplanaires se départagent PIXEL PAR PIXEL là où
            // elles se croisent à quelques cm (« micro-trous » dans la tuile floue laissant voir la nette
            // derrière — mesuré). Reversed-Z (GreaterEqual) : un epsilon POSITIF par niveau met la tuile
            // la plus FINE devant, déterministiquement, sur les seuls quasi-ex-æquo (l'ordre réel des
            // vraies occlusions, à des mètres d'écart, n'est pas affecté).
            const float levelBias = drawable.getTileID()
                                        ? static_cast<float>(drawable.getTileID()->canonical.z)
                                        : 0.0f;
            renderPass.getMetalEncoder()->setDepthBias(levelBias * 2.0f, 0.0f, 0.0f);
        }

        drawable.draw(parameters);
    });
}

} // namespace mtl
} // namespace mbgl
