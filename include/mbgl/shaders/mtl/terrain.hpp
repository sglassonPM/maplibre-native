#pragma once

#include <mbgl/shaders/terrain_layer_ubo.hpp>
#include <mbgl/shaders/shader_source.hpp>
#include <mbgl/shaders/mtl/shader_program.hpp>

namespace mbgl {
namespace shaders {

constexpr auto terrainShaderPrelude = R"(

enum {
    idTerrainDrawableUBO = idDrawableReservedVertexOnlyUBO,
    idTerrainTilePropsUBO = idDrawableReservedFragmentOnlyUBO,
    idTerrainEvaluatedPropsUBO = drawableReservedUBOCount,
    terrainUBOCount
};

struct alignas(16) TerrainDrawableUBO {
    /*  0 */ float4x4 matrix;
    /* 64 */ float4 dem_coords;
    /* 80 */ float4 edge_dz; // deficit de resolution DEM de la voisine sur chaque arete (W,E,N,S)
    /* 96 */
};
static_assert(sizeof(TerrainDrawableUBO) == 6 * 16, "wrong size");

struct alignas(16) TerrainTilePropsUBO {
    /*  0 */ float2 dem_tl;
    /*  8 */ float dem_scale;
    /* 12 */ float pad1;
    /* 16 */
};
static_assert(sizeof(TerrainTilePropsUBO) == 16, "wrong size");

/// Evaluated properties that do not depend on the tile
struct alignas(16) TerrainEvaluatedPropsUBO {
    /*  0 */ float4 unpack; // DEM unpack vector for the source's encoding
    /* 16 */ float exaggeration;
    /* 20 */ float elevation_offset;
    /* 24 */ float pad1;
    /* 28 */ float pad2;
    /* 32 */
};
static_assert(sizeof(TerrainEvaluatedPropsUBO) == 32, "wrong size");

)";

template <>
struct ShaderSource<BuiltIn::TerrainShader, gfx::Backend::Type::Metal> {
    static constexpr auto name = "TerrainShader";
    static constexpr auto vertexMainFunction = "vertexMain";
    static constexpr auto fragmentMainFunction = "fragmentMain";

    static const std::array<AttributeInfo, 1> attributes;
    static constexpr std::array<AttributeInfo, 0> instanceAttributes{};
    static const std::array<TextureInfo, 2> textures;

    static constexpr auto prelude = terrainShaderPrelude;
    static constexpr auto source = R"(

struct VertexStage {
    short4 pos [[attribute(0)]]; // xy = position, z = drapeau de jupe, w = drapeaux d'arete
};

struct FragmentStage {
    float4 position [[position, invariant]];
    float2 uv;
    float elevation;
};

FragmentStage vertex vertexMain(thread const VertexStage vertx [[stage_in]],
                                device const uint32_t& uboIndex [[buffer(idGlobalUBOIndex)]],
                                device const TerrainDrawableUBO* drawableVector [[buffer(idTerrainDrawableUBO)]],
                                device const TerrainEvaluatedPropsUBO& props [[buffer(idTerrainEvaluatedPropsUBO)]],
                                texture2d<float, access::sample> demTexture [[texture(0)]],
                                sampler demSampler [[sampler(0)]]) {

    device const TerrainDrawableUBO& drawable = drawableVector[uboIndex];

    // The mesh was generated with coordinates from 0 to EXTENT (8192)
    float2 pos = float2(vertx.pos.xy);
    float2 uv = pos / 8192.0;

    // Decode the DEM and interpolate in meters via the shared helper (the packed
    // Terrain-RGB/Terrarium DEM cannot be hardware-filtered, so it is sampled
    // NEAREST and interpolated after decoding, matching maplibre-gl-js and the
    // elevated layers). Map into the bound DEM tile; an ancestor tile is bound as
    // a fallback while this tile's own DEM loads. dem_coords.w = DEM dimension.
    float elevation = get_elevation(pos, demTexture, demSampler, drawable.dem_coords, props.unpack,
                                    drawable.dem_coords.w, props.exaggeration, 1.0);

    // Raccord des aretes : sur un bord dont la voisine est plus grossiere de edge_dz niveaux
    // de resolution DEM, on remplace l'altitude par l'interpolation lineaire entre les deux
    // points de la grille GROSSIERE qui encadrent le sommet. L'arete devient le meme segment
    // que celui de la voisine -> plus de marche, plus de trou. pos.w porte les drapeaux d'arete.
    const int edgeFlags = int(vertx.pos.w);
    const float texStep = 8192.0 / drawable.dem_coords.w; // un texel DEM en unites de pos
    float dzY = 0.0; // aretes ouest/est : la position varie en y
    if ((edgeFlags & 1) != 0) { dzY = max(dzY, drawable.edge_dz.x); }
    if ((edgeFlags & 2) != 0) { dzY = max(dzY, drawable.edge_dz.y); }
    float dzX = 0.0; // aretes nord/sud : la position varie en x
    if ((edgeFlags & 4) != 0) { dzX = max(dzX, drawable.edge_dz.z); }
    if ((edgeFlags & 8) != 0) { dzX = max(dzX, drawable.edge_dz.w); }
    if (dzY > 0.0) {
        const float s = texStep * exp2(dzY);
        const float y0 = floor(pos.y / s) * s;
        const float y1 = min(y0 + s, 8192.0);
        const float e0 = get_elevation(float2(pos.x, y0), demTexture, demSampler, drawable.dem_coords,
                                       props.unpack, drawable.dem_coords.w, props.exaggeration, 1.0);
        const float e1 = get_elevation(float2(pos.x, y1), demTexture, demSampler, drawable.dem_coords,
                                       props.unpack, drawable.dem_coords.w, props.exaggeration, 1.0);
        elevation = mix(e0, e1, (s > 0.0) ? (pos.y - y0) / s : 0.0);
    }
    if (dzX > 0.0) {
        const float s = texStep * exp2(dzX);
        const float x0 = floor(pos.x / s) * s;
        const float x1 = min(x0 + s, 8192.0);
        const float e0 = get_elevation(float2(x0, pos.y), demTexture, demSampler, drawable.dem_coords,
                                       props.unpack, drawable.dem_coords.w, props.exaggeration, 1.0);
        const float e1 = get_elevation(float2(x1, pos.y), demTexture, demSampler, drawable.dem_coords,
                                       props.unpack, drawable.dem_coords.w, props.exaggeration, 1.0);
        elevation = mix(e0, e1, (s > 0.0) ? (pos.x - x0) / s : 0.0);
    }

    // Jupe PAR BORD : elle ne descend a pleine profondeur que sur les aretes en frontiere
    // de LOD (edge_dz > 0), la ou le raccord laisse un residu a couvrir. Sur les 99 % d'aretes
    // qui s'accordent deja, la jupe est quasi nulle -> plus de rideaux visibles a fort pitch.
    // dzX/dzY viennent du raccord ci-dessus (deficit de resolution DEM de la voisine).
    const float skirtFactor = (max(dzX, dzY) > 0.0) ? 1.0 : 0.05;
    const float ele_delta = (float(vertx.pos.z) == 1.0) ? props.elevation_offset * skirtFactor : 0.0;
    float4 position = drawable.matrix * float4(pos.x, pos.y, elevation - ele_delta, 1.0);

    return {
        .position  = position,
        .uv        = uv,
        .elevation = elevation,
    };
}

half4 fragment fragmentMain(FragmentStage in [[stage_in]],
                            device const TerrainEvaluatedPropsUBO& props [[buffer(idTerrainEvaluatedPropsUBO)]],
                            texture2d<float, access::sample> mapTexture [[texture(1)]],
                            sampler mapSampler [[sampler(1)]]) {
#if defined(OVERDRAW_INSPECTOR)
    return half4(1.0);
#endif
    return half4(mapTexture.sample(mapSampler, in.uv));
}
)";
};

} // namespace shaders
} // namespace mbgl
