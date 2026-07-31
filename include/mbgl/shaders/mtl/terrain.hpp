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
    /* 80 */ float4 edge_dz; // pas de raccord par arete (W,E,N,S) : pas de la grille de sommets de la
                             // voisine plus grossiere, en unites locales 0..8192 (0 = pas de raccord)
    /* 96 */ float4 map_coords; // Isomaps : transform UV tuile raster ancetre {1/scale, dx/scale, dy/scale, 0}
    /* 112 */ float4 fog_params; // Isomaps BRUME : camera en repere local tuile (xy, unites 0..8192) + metres/unite (z)
    /* 128 */
};
static_assert(sizeof(TerrainDrawableUBO) == 8 * 16, "wrong size");

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
    float fogDist;   // Isomaps : distance horizontale reelle (metres) du sommet a la camera, pour la brume
    float2 tileUV;   // Isomaps DIAG : uv LOCAL 0..1 dans la tuile (pour le contour de tuile)
    float fogStart;  // Isomaps BRUME : debut de brume (metres), altitude-aware, depuis fog_params.w
    float skirt;     // Isomaps JUPE : profondeur du rideau en metres (0 sur la surface). Sert a ombrer LES
                     // SEULES jupes profondes (frontieres de LOD ~43 m) : un ombrage constant noircissait le
                     // liseré de ~11 m present le long de CHAQUE bord a pitch 0 → quadrillage sombre.
    float tileZoom;  // Isomaps DIAG : zoom canonique de la tuile (map_coords.w, rempli par le tweaker)
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
    // Isomaps : si la tuile raster basemap exacte n'est pas chargee, une tuile ANCETRE (plus grossiere)
    // est bindee ; on remappe l'uv sur la sous-region correspondante. {1,0,0,0} = tuile exacte (no-op).
    uv = uv * drawable.map_coords.x + drawable.map_coords.yz;

    // Decode the DEM and interpolate in meters via the shared helper (the packed
    // Terrain-RGB/Terrarium DEM cannot be hardware-filtered, so it is sampled
    // NEAREST and interpolated after decoding, matching maplibre-gl-js and the
    // elevated layers). Map into the bound DEM tile; an ancestor tile is bound as
    // a fallback while this tile's own DEM loads. dem_coords.w = DEM dimension.
    float elevation = get_elevation(pos, demTexture, demSampler, drawable.dem_coords, props.unpack,
                                    drawable.dem_coords.w, props.exaggeration, 1.0);

    // Raccord des aretes : sur un bord dont la voisine (maillage) est plus grossiere, edge_dz porte le
    // PAS DE SA GRILLE DE SOMMETS en unites locales (0 = pas de raccord). On remplace l'altitude par
    // l'interpolation lineaire entre les deux SOMMETS DE LA VOISINE qui encadrent le notre : l'arete
    // devient le MEME segment que le sien (la pyramide DEM est coherente a ~7 m pres) -> plus de marche.
    // (Avant : alignement sur la grille de TEXELS DEM, ~4x plus fine que la grille de sommets de la
    // voisine -> notre bord suivait la courbure du DEM entre ses sommets -> marches enormes en paroi.)
    // pos.w porte les drapeaux d'arete.
    const int edgeFlags = int(vertx.pos.w);
    float dzY = 0.0; // aretes ouest/est : la position varie en y
    if ((edgeFlags & 1) != 0) { dzY = max(dzY, drawable.edge_dz.x); }
    if ((edgeFlags & 2) != 0) { dzY = max(dzY, drawable.edge_dz.y); }
    float dzX = 0.0; // aretes nord/sud : la position varie en x
    if ((edgeFlags & 4) != 0) { dzX = max(dzX, drawable.edge_dz.z); }
    if ((edgeFlags & 8) != 0) { dzX = max(dzX, drawable.edge_dz.w); }
    if (dzY > 0.0) {
        const float s = dzY;
        const float y0 = floor(pos.y / s) * s;
        const float y1 = min(y0 + s, 8192.0);
        const float e0 = get_elevation(float2(pos.x, y0), demTexture, demSampler, drawable.dem_coords,
                                       props.unpack, drawable.dem_coords.w, props.exaggeration, 1.0);
        const float e1 = get_elevation(float2(pos.x, y1), demTexture, demSampler, drawable.dem_coords,
                                       props.unpack, drawable.dem_coords.w, props.exaggeration, 1.0);
        elevation = mix(e0, e1, (s > 0.0) ? (pos.y - y0) / s : 0.0);
    }
    if (dzX > 0.0) {
        const float s = dzX;
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
    // Isomaps : jupe modulée PAR BORD et proportionnelle à l'exagération (le crack est en mètres
    // exagérés, comme elevation). Aux frontières de LOD (edge_dz>0) le résidu du raccord d'arête est
    // grand → jupe profonde (×4.5). Hors frontière, les bords « accordés » gardent quand même un
    // léger décalage DEM (échantillonnage/floats) qui doit être couvert → jupe modérée (×0.3), pas
    // quasi nulle (sinon micro-blancs qui reviennent en terrain vallonné). Effectif à exagération
    // 1.2 : ~43 m aux frontières, ~2,9 m ailleurs (≈ le test 40 m validé, avec margin). Base = 8 m
    // (Vulkan/Android l'appliquent pleine sur tous les bords, cf. tweaker) → non régressée.
    const float skirtFactor = (max(dzX, dzY) > 0.0) ? 4.5 : 1.2;
    const float ele_delta = (float(vertx.pos.z) == 1.0)
                                ? props.elevation_offset * skirtFactor * props.exaggeration
                                : 0.0;
    float4 position = drawable.matrix * float4(pos.x, pos.y, elevation - ele_delta, 1.0);

    // Isomaps reversed-Z : remap la profondeur clip de GL [-1,1] vers Metal [0,1] INVERSE (near->ndc 1,
    // far->ndc 0). z' = 0.5*(w - z). Le lointain tombe vers 0, zone de haute precision du Depth32Float ->
    // supprime l'effondrement de precision a fort pitch (tuiles lointaines passant DEVANT les proches) SANS
    // remonter le near (donc aucun clipping en navigation). Paire avec GreaterEqual (mtl/drawable.cpp) +
    // clear main pass = 0 (renderer_impl.cpp). Le flicker etait un bug de cover independant (hysteresis).
    position.z = 0.5 * (position.w - position.z);

    // Isomaps BRUME : distance horizontale reelle (metres) du sommet a la camera. La camera est fournie
    // dans le repere local de la tuile (fog_params.xy en unites 0..8192), fog_params.z = metres/unite.
    const float fogDist = length(pos - drawable.fog_params.xy) * drawable.fog_params.z;

    return {
        .position  = position,
        .uv        = uv,
        .elevation = elevation,
        .fogDist   = fogDist,
        .tileUV    = pos / 8192.0,
        .fogStart  = drawable.fog_params.w,
        .skirt     = ele_delta, // profondeur de jupe (m), 0 hors jupe
        .tileZoom  = drawable.map_coords.w,
    };
}

half4 fragment fragmentMain(FragmentStage in [[stage_in]],
                            device const TerrainEvaluatedPropsUBO& props [[buffer(idTerrainEvaluatedPropsUBO)]],
                            texture2d<float, access::sample> mapTexture [[texture(1)]],
                            sampler mapSampler [[sampler(1)]]) {
#if defined(OVERDRAW_INSPECTOR)
    return half4(1.0);
#endif
    const float4 tex = mapTexture.sample(mapSampler, in.uv);

    // Isomaps BRUME atmospherique (facon Mapbox) : net jusqu'a ~fogStart, dense vers ~fogEnd, plafonnee
    // a fogMax pour que les sommets lointains (100-120 km) restent des SILHOUETTES brumeuses, pas un mur
    // opaque. Fondu quadratique (perspective aerienne : monte doucement puis s'accelere).
    const float fogStart = in.fogStart;         // m : debut de brume, altitude-aware (max 8 km, alt x2)
    const float fogEnd = fogStart + 47000.0;    // m : brume ~pleine (largeur de transition ~47 km)
    constexpr float fogMax = 0.85;              // opacite max (garde les silhouettes au-dela)
    const half3 fogColor = half3(0.72, 0.78, 0.84); // bleu-gris desature atmospherique
    float f = clamp((in.fogDist - fogStart) / (fogEnd - fogStart), 0.0, 1.0);
    f = f * f * fogMax;
    half3 rgb = mix(half3(tex.rgb), fogColor, half(f));

    // Isomaps JUPE : ombrage PROPORTIONNEL A LA PROFONDEUR — nul jusqu'a ~15 m (les lisérés de bord normaux
    // restent camoufles par la texture etiree, pas de quadrillage a pitch 0), plein a ~40 m (frontieres de
    // LOD : le grand rideau strie lit comme un pli d'ombre au lieu d'un trou).
    rgb *= half(1.0 - 0.35 * smoothstep(15.0, 40.0, in.skirt));

    // Isomaps DIAG — visualisation du LOD : teinte par niveau de zoom de la tuile + contour.
    // Légende : z≤11 bleu · z12 cyan · z13 vert · z14 jaune · z15 orange · z16 rouge · z≥17 magenta.
    // Zoom lu dans in.tileZoom (map_coords.w, rempli par le tweaker). Repasser à #if 1 pour débugger.
#if 0
    {
        const float z = in.tileZoom;
        half3 zc;
        if (z <= 11.5)      zc = half3(0.15, 0.30, 0.95);
        else if (z <= 12.5) zc = half3(0.10, 0.85, 0.85);
        else if (z <= 13.5) zc = half3(0.20, 0.90, 0.30);
        else if (z <= 14.5) zc = half3(0.95, 0.90, 0.15);
        else if (z <= 15.5) zc = half3(1.00, 0.55, 0.10);
        else if (z <= 16.5) zc = half3(1.00, 0.15, 0.15);
        else                zc = half3(0.90, 0.20, 0.90);
        rgb = mix(rgb, zc, half(0.40));
        // Contour de tuile (uv local proche d'un bord) — liseré sombre.
        const float2 dedge = min(in.tileUV, 1.0 - in.tileUV);
        if (min(dedge.x, dedge.y) < 0.006) rgb = half3(0.05, 0.05, 0.05);
    }
#endif

    return half4(rgb, half(tex.a));
}
)";
};

} // namespace shaders
} // namespace mbgl
