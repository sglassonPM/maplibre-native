#include <mbgl/renderer/layers/terrain_layer_tweaker.hpp>

#include <mbgl/gfx/context.hpp>
#include <mbgl/gfx/drawable.hpp>
#include <mbgl/renderer/layer_group.hpp>
#include <mbgl/renderer/paint_parameters.hpp>
#include <mbgl/renderer/render_terrain.hpp>
#include <mbgl/map/transform_state.hpp>
#include <mbgl/shaders/terrain_layer_ubo.hpp>
#include <mbgl/shaders/shader_defines.hpp>
#include <mbgl/util/constants.hpp>
#include <mbgl/util/convert.hpp>
#include <mbgl/util/mat4.hpp>
#include <mbgl/util/logging.hpp>

#include <algorithm>
#include <cmath>

namespace mbgl {

using namespace shaders;

void TerrainLayerTweaker::execute(LayerGroupBase& layerGroup, const PaintParameters& parameters) {
    if (layerGroup.empty() || !terrain) {
        return;
    }

    auto& context = parameters.context;

#if defined(DEBUG)
    const auto label = layerGroup.getName() + "-update-uniforms";
    const auto debugGroup = parameters.encoder->createDebugGroup(label.c_str());
#endif

    // Get terrain properties; per the style spec, exaggeration is applied as-is
    // (default 1.0 renders true-scale elevation)
    const float exaggeration = terrain->getExaggeration();

    // Skirt depth (u_ele_delta), en mètres RÉELS. Baseline PARTAGÉE : les shaders Vulkan/WebGPU
    // (Android) l'appliquent telle quelle à TOUS les bords → on la garde courte (8 m) pour ne pas
    // créer de rideaux partout. Le shader Metal (plus avancé : raccord d'arête + skirtFactor PAR
    // bord) la module lui-même : plein uniquement aux frontières de LOD (×~4.2) et ×exagération,
    // pour ~40 m effectifs à exagération 1.2 là où c'est nécessaire (micro-blancs en terrain raide),
    // quasi nul ailleurs. PAS la formule gl-js getSkirtLength (1/5 largeur tuile → rideaux à z11,
    // cracks rouverts à z15). Une profondeur métrique est le bon unit (le crack = décalage d'altitude).
    const float elevationOffset = 8.0f;

    // Populate layer-level UBO with terrain properties
    auto& layerUniforms = layerGroup.mutableUniformBuffers();
    const TerrainEvaluatedPropsUBO propsUBO = {.unpack = terrain->getDEMUnpackVector(),
                                               .exaggeration = exaggeration,
                                               .elevation_offset = elevationOffset,
                                               .pad1 = 0.0f,
                                               .pad2 = 0.0f};
    layerUniforms.createOrUpdate(idTerrainEvaluatedPropsUBO, &propsUBO, context);

    // Isomaps BRUME : position mercator [0,1] de la camera + latitude (calculees une fois) — servent a
    // convertir, PAR TUILE, la distance camera→sommet en metres reels dans le repere local de la tuile.
    constexpr double kEarthCircumference = 40075016.686;
    constexpr double kPi = 3.14159265358979323846;
    const LatLng camLatLng = parameters.state.getLatLng(); // centre écran — sert à la latitude (mètres)
    const double cosLat = std::cos(camLatLng.latitude() * kPi / 180.0);
    // Position mercator de l'ŒIL (pas le centre écran). Sinon la distance est mesurée depuis la CIBLE du
    // regard (au loin) → premier plan embrumé + cible nette, exactement l'inverse du voulu. L'œil est
    // près du premier plan → celui-ci reste net, le lointain (cible) s'embrume.
    double camMercX, camMercY;
    const auto freeCam = parameters.state.getFreeCameraOptions();
    if (freeCam.position) {
        camMercX = (*freeCam.position)[0];
        camMercY = (*freeCam.position)[1];
    } else {
        camMercX = (camLatLng.longitude() + 180.0) / 360.0;
        const double sinLat = std::sin(camLatLng.latitude() * kPi / 180.0);
        camMercY = 0.5 - std::log((1.0 + sinLat) / (1.0 - sinLat)) / (4.0 * kPi);
    }

    // Isomaps BRUME altitude-aware : la brume est basée sur la distance HORIZONTALE œil→fragment. À haute
    // altitude, TOUT le terrain visible est loin horizontalement → la brume recouvrait tout l'écran. On
    // repousse le DÉBUT de brume avec l'altitude : fogStart = max(8 km, altitude × 2). À 8000 m elle
    // commence vers ~16 km → le proche/moyen reste net, le lointain (horizon) s'embrume ; à basse
    // altitude, 8 km comme avant. Transmis par fog_params.w (ce slot portait le zoom de tuile, DIAG off).
    // NB : le facteur ×2 est un curseur (×4 embrumait trop loin en altitude, à re-régler au besoin).
    double camAltM = 0.0;
    if (const auto loc = freeCam.getLocation()) {
        camAltM = loc->altitude;
    }
    const float fogStartM = static_cast<float>(std::max(8000.0, camAltM * 2.0));

#if MLN_UBO_CONSOLIDATION
    int i = 0;
    std::vector<TerrainDrawableUBO> drawableUBOVector(layerGroup.getDrawableCount());
#endif

    // Visit each drawable to populate per-drawable UBOs
    visitLayerGroupDrawables(layerGroup, [&](gfx::Drawable& drawable) {
        if (!drawable.getTileID()) {
            return;
        }

        const UnwrappedTileID tileID = drawable.getTileID()->toUnwrapped();

        // Calculate transformation matrix for this terrain tile
        // This uses the same matrix calculation as other layers
        mat4 matrix = parameters.matrixForTile(tileID);

        // Isomaps BRUME : exprime la camera dans le repere local (0..8192) de CETTE tuile + metres/unite.
        const double fogN = static_cast<double>(1ull << tileID.canonical.z);
        const float fogCamX = static_cast<float>(
            (camMercX * fogN - static_cast<double>(tileID.wrap) * fogN - tileID.canonical.x) * 8192.0);
        const float fogCamY = static_cast<float>((camMercY * fogN - tileID.canonical.y) * 8192.0);
        const float fogMPerUnit = static_cast<float>(kEarthCircumference * cosLat / fogN / 8192.0);

#if !MLN_UBO_CONSOLIDATION
        auto& drawableUniforms = drawable.mutableUniformBuffers();
#endif

#if MLN_UBO_CONSOLIDATION
        drawableUBOVector[i] = {
#else
        const TerrainDrawableUBO drawableUBO = {
#endif
            .matrix = util::cast<float>(matrix),
            .dem_coords = terrain->getDrawableDemCoords(*drawable.getTileID()),
            .edge_dz = terrain->getDrawableEdgeDz(*drawable.getTileID()),
            .map_coords = terrain->getDrawableMapCoords(*drawable.getTileID()),
            .fog_params = {fogCamX, fogCamY, fogMPerUnit, fogStartM}
        };

#if !MLN_UBO_CONSOLIDATION
        drawableUniforms.createOrUpdate(idTerrainDrawableUBO, &drawableUBO, context);
#endif

#if MLN_UBO_CONSOLIDATION
        drawable.setUBOIndex(i++);
#endif
    });

#if MLN_UBO_CONSOLIDATION
    const size_t drawableUBOVectorSize = sizeof(TerrainDrawableUBO) * drawableUBOVector.size();
    if (!drawableUniformBuffer || drawableUniformBuffer->getSize() < drawableUBOVectorSize) {
        drawableUniformBuffer = context.createUniformBuffer(
            drawableUBOVector.data(), drawableUBOVectorSize, false, true);
    } else {
        drawableUniformBuffer->update(drawableUBOVector.data(), drawableUBOVectorSize);
    }

    layerUniforms.set(idTerrainDrawableUBO, drawableUniformBuffer);
#endif
}

} // namespace mbgl
