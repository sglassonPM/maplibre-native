// Generated code, do not modify this file!
#pragma once
#include <mbgl/shaders/shader_source.hpp>

namespace mbgl {
namespace shaders {

template <>
struct ShaderSource<BuiltIn::TerrainShader, gfx::Backend::Type::OpenGL> {
    static constexpr const char* name = "TerrainShader";
    static constexpr const char* vertex = R"(out vec2 v_uv;
out vec2 v_tile_uv;
out float v_fog_dist;
out float v_fog_start;
out float v_skirt;

layout (std140) uniform TerrainDrawableUBO {
    highp mat4 u_matrix;
    highp vec4 u_dem_tile_coords;
    highp vec4 u_edge_dz;    // pas de raccord par arete (W,E,N,S), unites locales 0..8192 (0 = pas de raccord)
    highp vec4 u_map_coords; // Isomaps : transform UV tuile raster ancetre {1/scale, dx/scale, dy/scale, 0}
    highp vec4 u_fog_params; // Isomaps BRUME : camera en repere local tuile (xy) + metres/unite (z) + debut (w)
};

layout (std140) uniform TerrainEvaluatedPropsUBO {
    highp vec4 u_unpack;
    highp float u_exaggeration;
    highp float u_elevation_offset;
    highp float u_basemap_direct;
    lowp float props_pad2;
};

layout (location = 0) in vec4 a_pos; // xy = tile position, z = skirt flag, w = edge flags

uniform sampler2D u_dem;

void main() {
    // The mesh was generated with coordinates from 0 to EXTENT (8192)
    vec2 pos = a_pos.xy;
    v_tile_uv = pos / 8192.0;
    // Isomaps : si la tuile raster basemap exacte n'est pas chargee, une tuile ANCETRE (plus grossiere)
    // est bindee ; on remappe l'uv sur la sous-region correspondante. {1,0,0,0} = tuile exacte (no-op).
    v_uv = v_tile_uv * u_map_coords.x + u_map_coords.yz;

    // Decode the DEM and interpolate in meters via the shared prelude helper
    // (the packed Terrain-RGB/Terrarium DEM cannot be hardware-filtered, so it is
    // sampled NEAREST and interpolated after decoding, matching maplibre-gl-js and
    // the elevated layers). Map into the bound DEM tile; an ancestor tile is bound
    // as a fallback while this tile's own DEM loads. dem_coords.w = DEM dimension.
    float elevation = get_elevation(pos, u_dem, u_dem_tile_coords, u_unpack,
                                    u_dem_tile_coords.w, u_exaggeration, 1.0);

    // Raccord des aretes (parite Metal) : sur un bord dont la voisine (maillage) est plus grossiere,
    // edge_dz porte le PAS DE SA GRILLE DE SOMMETS en unites locales (0 = pas de raccord). On remplace
    // l'altitude par l'interpolation lineaire entre les deux SOMMETS DE LA VOISINE qui encadrent le
    // notre : l'arete devient le MEME segment que le sien -> plus de marche. a_pos.w = drapeaux d'arete.
    int edgeFlags = int(a_pos.w);
    float dzY = 0.0; // aretes ouest/est : la position varie en y
    if ((edgeFlags & 1) != 0) { dzY = max(dzY, u_edge_dz.x); }
    if ((edgeFlags & 2) != 0) { dzY = max(dzY, u_edge_dz.y); }
    float dzX = 0.0; // aretes nord/sud : la position varie en x
    if ((edgeFlags & 4) != 0) { dzX = max(dzX, u_edge_dz.z); }
    if ((edgeFlags & 8) != 0) { dzX = max(dzX, u_edge_dz.w); }
    if (dzY > 0.0) {
        float s = dzY;
        float y0 = floor(pos.y / s) * s;
        float y1 = min(y0 + s, 8192.0);
        float e0 = get_elevation(vec2(pos.x, y0), u_dem, u_dem_tile_coords, u_unpack,
                                 u_dem_tile_coords.w, u_exaggeration, 1.0);
        float e1 = get_elevation(vec2(pos.x, y1), u_dem, u_dem_tile_coords, u_unpack,
                                 u_dem_tile_coords.w, u_exaggeration, 1.0);
        elevation = mix(e0, e1, (s > 0.0) ? (pos.y - y0) / s : 0.0);
    }
    if (dzX > 0.0) {
        float s = dzX;
        float x0 = floor(pos.x / s) * s;
        float x1 = min(x0 + s, 8192.0);
        float e0 = get_elevation(vec2(x0, pos.y), u_dem, u_dem_tile_coords, u_unpack,
                                 u_dem_tile_coords.w, u_exaggeration, 1.0);
        float e1 = get_elevation(vec2(x1, pos.y), u_dem, u_dem_tile_coords, u_unpack,
                                 u_dem_tile_coords.w, u_exaggeration, 1.0);
        elevation = mix(e0, e1, (s > 0.0) ? (pos.x - x0) / s : 0.0);
    }

    // Jupe PAR BORD (parite Metal) : pleine profondeur uniquement aux frontieres de LOD
    // (edge_dz > 0, x4.5), moderee ailleurs (x1.2), proportionnelle a l'exageration.
    float skirtFactor = (max(dzX, dzY) > 0.0) ? 4.5 : 1.2;
    float ele_delta = (a_pos.z == 1.0) ? u_elevation_offset * skirtFactor * u_exaggeration : 0.0;
    v_skirt = ele_delta;

    // Isomaps BRUME : distance horizontale reelle (metres) du sommet a la camera (repere local tuile).
    v_fog_dist = length(pos - u_fog_params.xy) * u_fog_params.z;
    v_fog_start = u_fog_params.w;

    // NB : pas de remap reversed-Z ici — propre a Metal (profondeur [0,1] inversee + GreaterEqual).
    gl_Position = u_matrix * vec4(pos.x, pos.y, elevation - ele_delta, 1.0);
}
)";
    static constexpr const char* fragment = R"(in vec2 v_uv;
in vec2 v_tile_uv;
in float v_fog_dist;
in float v_fog_start;
in float v_skirt;

layout (std140) uniform TerrainEvaluatedPropsUBO {
    highp vec4 u_unpack;
    highp float u_exaggeration;
    highp float u_elevation_offset;
    highp float u_basemap_direct;
    lowp float props_pad2;
};

uniform sampler2D u_map;
uniform sampler2D u_drape;

void main() {
    vec4 tex;
    if (u_basemap_direct == 1.0) {
        // Isomaps basemap direct : u_map = tuile raster UPLOADEE (satellite), orientation native ->
        // pas de flip ; v_uv porte le remap d'ancetre. L'overlay vectoriel drape (trace GPX, lignes)
        // est une cible RTT -> flip Y OpenGL, uv locale 1:1 ; les tuiles sans contenu lient un 1x1
        // transparent (alpha 0 = no-op).
        tex = texture(u_map, v_uv);
        vec4 drape = texture(u_drape, vec2(v_tile_uv.x, 1.0 - v_tile_uv.y));
        tex.rgb = mix(tex.rgb, drape.rgb, drape.a);
    } else {
        // Drapage classique : u_map = cible RTT de la tuile (Y inverse en OpenGL).
        tex = texture(u_map, vec2(v_uv.x, 1.0 - v_uv.y));
    }

    // Isomaps BRUME atmospherique (parite Metal) : nette jusqu'a ~fogStart, fondu quadratique,
    // plafonnee pour garder les silhouettes lointaines.
    float fogEnd = v_fog_start + 47000.0;
    float f = clamp((v_fog_dist - v_fog_start) / (fogEnd - v_fog_start), 0.0, 1.0);
    f = f * f * 0.85;
    vec3 rgb = mix(tex.rgb, vec3(0.72, 0.78, 0.84), f);

    // Isomaps JUPE : ombrage proportionnel a la profondeur (nul <15 m, plein ~40 m).
    rgb *= 1.0 - 0.35 * smoothstep(15.0, 40.0, v_skirt);

    fragColor = vec4(rgb, tex.a);

#if defined(OVERDRAW_INSPECTOR)
    fragColor = vec4(1.0);
#endif
}
)";
};

} // namespace shaders
} // namespace mbgl
