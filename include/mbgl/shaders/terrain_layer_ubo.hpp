#pragma once

#include <mbgl/shaders/layer_ubo.hpp>

namespace mbgl {
namespace shaders {

struct alignas(16) TerrainDrawableUBO {
    /*  0 */ std::array<float, 4 * 4> matrix;
    /* 64 */ std::array<float, 4> dem_coords; // scale, x offset, y offset into the bound DEM
                                              // tile ({1,0,0,0} unless an ancestor is bound)
    /* 80 */ std::array<float, 4> edge_dz;    // pas de raccord par arete (W, E, N, S) : pas de la grille
                                              // de sommets de la voisine plus grossiere, en unites
                                              // locales 0..8192 ; 0 = voisine de meme grille ou plus fine
    /* 96 */ std::array<float, 4> map_coords; // Isomaps : transform UV pour la tuile raster basemap
                                              // ({1/scale, dx/scale, dy/scale, 0}) quand une tuile
                                              // ANCETRE est bindee (fallback chargement) ; {1,0,0,0}
                                              // = tuile exacte (uv inchange). Cf. RenderTerrain.
    /* 112 */ std::array<float, 4> fog_params; // Isomaps BRUME : position de la CAMERA dans le repere
                                               // local de cette tuile (x,y en unites 0..8192) + metres
                                               // par unite locale (z) → le vertex calcule sa distance
                                               // reelle en metres a la camera. w = libre.
    /* 128 */
};
static_assert(sizeof(TerrainDrawableUBO) == 8 * 16);

struct alignas(16) TerrainTilePropsUBO {
    /*  0 */ std::array<float, 2> dem_tl;
    /*  8 */ float dem_scale;
    /* 12 */ float pad1;
    /* 16 */
};
static_assert(sizeof(TerrainTilePropsUBO) == 16);

/// Evaluated properties that do not depend on the tile
struct alignas(16) TerrainEvaluatedPropsUBO {
    /*  0 */ std::array<float, 4> unpack; // DEM unpack vector for the source's encoding
    /* 16 */ float exaggeration;
    /* 20 */ float elevation_offset;
    /* 24 */ float basemap_direct; // Isomaps : 1 = mode basemap direct (u_map = tuile raster uploadée,
                                   // remap d'ancêtre, overlay drapé par-dessus) ; 0 = drapage RTT
                                   // classique. Le shader GL en a besoin pour choisir le flip Y
                                   // (une cible RTT est inversée en OpenGL, une texture uploadée non).
    /* 28 */ float pad2;
    /* 32 */
};
static_assert(sizeof(TerrainEvaluatedPropsUBO) == 32);

} // namespace shaders
} // namespace mbgl
