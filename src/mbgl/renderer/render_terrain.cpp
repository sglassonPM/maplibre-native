#include <mbgl/renderer/render_terrain.hpp>
#include <mbgl/renderer/update_parameters.hpp>
#include <mbgl/renderer/render_source.hpp>
#include <mbgl/renderer/render_tile.hpp>
#include <mbgl/renderer/render_pass.hpp>
#include <mbgl/renderer/render_tree.hpp>
#include <mbgl/renderer/render_static_data.hpp>
#include <mbgl/renderer/render_orchestrator.hpp>
#include <mbgl/renderer/render_target.hpp>
#include <mbgl/renderer/dem_elevation_provider.hpp>
#include <mbgl/renderer/paint_parameters.hpp>
#include <mbgl/renderer/change_request.hpp>
#include <mbgl/renderer/layer_group.hpp>
#include <mbgl/renderer/layers/terrain_layer_tweaker.hpp>
#include <mbgl/renderer/buckets/hillshade_bucket.hpp>
#include <mbgl/geometry/dem_data.hpp>
#include <mbgl/util/tile_cover.hpp>
#include <mbgl/tile/raster_dem_tile.hpp>
#include <mbgl/tile/raster_tile.hpp>
#include <mbgl/renderer/buckets/raster_bucket.hpp>
#include <mbgl/tile/tile.hpp>
#include <mbgl/gfx/context.hpp>
#include <mbgl/gfx/renderable.hpp>
#include <mbgl/gfx/renderer_backend.hpp>
#include <mbgl/gfx/drawable.hpp>
#include <mbgl/gfx/drawable_impl.hpp>
#include <mbgl/gfx/drawable_builder.hpp>
#include <mbgl/gfx/shader_registry.hpp>
#include <mbgl/gfx/color_mode.hpp>
#include <mbgl/gfx/texture2d.hpp>
#include <mbgl/shaders/shader_source.hpp>
#include <mbgl/shaders/terrain_layer_ubo.hpp>
#include <mbgl/shaders/shader_defines.hpp>
#include <mbgl/shaders/segment.hpp>
#include <mbgl/util/constants.hpp>
#include <mbgl/util/logging.hpp>
#include <mbgl/util/image.hpp>
#include <mbgl/util/mat4.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <functional>
#include <unordered_set>

// Isomaps : compteurs de tuiles (maillage / basemap) exposés au HUD de l'app d'éval via accesseurs C.
namespace {
std::atomic<int> g_isomapsMeshTiles{0};
std::atomic<int> g_isomapsSatTiles{0};
// Isomaps 🧠 : octets GPU réels des textures (mesurés, pas estimés) — pour attribuer le footprint.
std::atomic<long long> g_isomapsSatBytes{0};
std::atomic<long long> g_isomapsDemBytes{0};
} // namespace
extern "C" int isomapsDebugMeshTileCount() { return g_isomapsMeshTiles.load(std::memory_order_relaxed); }
extern "C" int isomapsDebugSatTileCount() { return g_isomapsSatTiles.load(std::memory_order_relaxed); }
extern "C" long long isomapsDebugSatBytes() { return g_isomapsSatBytes.load(std::memory_order_relaxed); }
extern "C" long long isomapsDebugDemBytes() { return g_isomapsDemBytes.load(std::memory_order_relaxed); }

namespace mbgl {

namespace {

// Scale and x/y offset mapping a child tile's local space into the (possibly
// ancestor) DEM tile that covers it: a child dz levels deeper occupies the
// 1/scale sub-square at (dx, dy) of the ancestor.
struct DEMSubTileOffset {
    float scale;
    float dx;
    float dy;
};

DEMSubTileOffset demSubTileOffset(const CanonicalTileID& child, const CanonicalTileID& ancestor) {
    const int dz = child.z - ancestor.z;
    return {static_cast<float>(1u << dz),
            static_cast<float>(child.x - (ancestor.x << dz)),
            static_cast<float>(child.y - (ancestor.y << dz))};
}

} // namespace

RenderTerrain::RenderTerrain(Immutable<style::Terrain::Impl> impl_)
    : impl(std::move(impl_)) {}

RenderTerrain::~RenderTerrain() = default;

std::set<UnwrappedTileID> RenderTerrain::computeMeshCover(const TransformState& state,
                                                         const UpdateParameters& updateParameters) {
    std::set<UnwrappedTileID> out;
    uint8_t demBaseZoom = 0; // zoom max du DEM chargé — sert à borner le raffinement satellite ci-dessous
    if (demSource) {
        const auto renderTiles = demSource->getRawRenderTiles();
        if (!renderTiles->empty()) {
            uint8_t baseZoom = 0;
            uint8_t minLoadedZoom = 30;
            for (const auto& renderTile : *renderTiles) {
                baseZoom = std::max(baseZoom, renderTile.id.canonical.z);
                minLoadedZoom = std::min(minLoadedZoom, renderTile.id.canonical.z);
            }
            demBaseZoom = baseZoom;
            StableElevationProvider elevationProvider(demSource, getExaggeration(), meshTileElevation, meshTileElevationFinal);
            // Meme seuil variable-zoom que les sources (abaisse a 0 dans map_impl) et cover borne.
            const util::TileCoverParameters coverParams{
                .transformState = state,
                .tileLodMinRadius = updateParameters.tileLodMinRadius,
                .tileLodScale = updateParameters.tileLodScale,
                .tileLodPitchThreshold = updateParameters.tileLodPitchThreshold,
                .tileLodMode = updateParameters.tileLodMode,
                .elevationProvider = &elevationProvider};
            // Isomaps : plancher de zoom LARGE (≤8), pas minLoadedZoom. Au repos, seul du DEM z18 reste rendu
            // → Range(18,18) : un nœud que le LOD arrête à z16 (versant profond, plus loin → plus grossier)
            // n'était NI émis NI subdivisé → sous-arbre abandonné → pan de terrain absent (bande bleue nette,
            // maillage=6 mesuré). Émettre grossier est sain : DEM ancêtre (z7/z10 retenu décodé) + drapage
            // ancêtre couvrent ces tuiles.
            const auto cover = util::tileCover(
                coverParams, baseZoom, Range<uint8_t>(std::min<uint8_t>(minLoadedZoom, 8), baseZoom));
            for (const auto& oid : cover) {
                out.insert(UnwrappedTileID(oid.wrap, oid.canonical));
            }
            // Garde-fou : au-dela d'un plafond genereux, on tronque aux tuiles les plus proches
            // du centre (les plus grandes a l'ecran) plutot que de risquer l'OOM.
            constexpr size_t maxMeshTiles = 300; // Isomaps : plafond ferme (recalibré ère 512 : une tuile
            // couvre 2× la portée linéaire → ~¼ du compte 256 pour la même couverture ; 700 laissait ~3 Go
            // de textures 1024² → OOM). Historique : 5000 laissait le dézoom incliné
            // exploser (frustum jusqu'à l'horizon → OOM/crash). 700 borne le coût sans toucher aux vues
            // normales (~150-340 tuiles) ; on garde les tuiles les plus PROCHES du centre (tri par distance).
            if (out.size() > maxMeshTiles) {
                const auto center = TileCoordinate::fromLatLng(baseZoom, state.getLatLng());
                std::vector<UnwrappedTileID> byDist(out.begin(), out.end());
                std::sort(byDist.begin(), byDist.end(), [&](const UnwrappedTileID& a, const UnwrappedTileID& b) {
                    const double sa = static_cast<double>(1u << (baseZoom - a.canonical.z));
                    const double sb = static_cast<double>(1u << (baseZoom - b.canonical.z));
                    const double ax = (a.canonical.x + 0.5) * sa, ay = (a.canonical.y + 0.5) * sa;
                    const double bx = (b.canonical.x + 0.5) * sb, by = (b.canonical.y + 0.5) * sb;
                    const double da = (ax - center.p.x) * (ax - center.p.x) + (ay - center.p.y) * (ay - center.p.y);
                    const double db = (bx - center.p.x) * (bx - center.p.x) + (by - center.p.y) * (by - center.p.y);
                    return da < db;
                });
                out = std::set<UnwrappedTileID>(byDist.begin(), byDist.begin() + maxMeshTiles);
            }
        }
        // Isomaps COARSE-FIRST (lancement) : rien de RENDU encore, mais l'aperçu DEM grossier RETENU
        // (z7/z10) est déjà décodé (demTextures) → mailler tout de suite avec lui. Fond flou immédiat
        // (drapé satellite z2-z10, déjà rendu) au lieu d'un écran bleu ; se raffine dès le premier rendu.
        if (out.empty() && !demTextures.empty()) {
            uint8_t coarseBase = 0;
            uint8_t coarseMin = 30;
            for (const auto& [cid, entry] : demTextures) {
                coarseBase = std::max(coarseBase, cid.canonical.z);
                coarseMin = std::min(coarseMin, cid.canonical.z);
            }
            if (coarseBase > 0) {
                StableElevationProvider elevationProvider(demSource, getExaggeration(), meshTileElevation, meshTileElevationFinal);
                const util::TileCoverParameters coverParams{
                    .transformState = state,
                    .tileLodMinRadius = updateParameters.tileLodMinRadius,
                    .tileLodScale = updateParameters.tileLodScale,
                    .tileLodPitchThreshold = updateParameters.tileLodPitchThreshold,
                    .tileLodMode = updateParameters.tileLodMode,
                    .elevationProvider = &elevationProvider};
                for (const auto& oid :
                     util::tileCover(coverParams, coarseBase, Range<uint8_t>(coarseMin, coarseBase))) {
                    out.insert(UnwrappedTileID(oid.wrap, oid.canonical));
                }
            }
        }
    }

    // NB (Isomaps) : le « raffinement satellite » (+ expandToDeepestCover) est SUPPRIMÉ. Il datait de
    // l'époque où le satellite (LOD variable) descendait plus fin qu'un DEM uniforme z14. Depuis que DEM
    // et satellite partagent le même maxzoom (18) et le même LOD-AGL, le cover DEM atteint z18 tout seul —
    // et l'expansion subdivisait en cascade les tuiles grossières chevauchées : 163 tuiles de maillage AU
    // REPOS pour un écran qui en montre ~10 (mesuré au 🧭 STATUS) → mémoire/chargement gonflés pour rien.

    // NB (Isomaps) : un plafond de zoom par distance a été tenté ici pour alléger le compte au loin,
    // mais il coarsait le maillage EN DESSOUS du DEM chargé → correspondance maillage↔DEM rompue →
    // écran gris. Retiré. L'allègement du compte devra passer par une voie qui préserve l'invariant
    // « maillage ⊆ zooms du DEM » (p.ex. rendre le COVER DEM lui-même plus grossier au loin), pas par
    // un post-plafonnement du maillage. La brume (œil-relative) reste, elle, indépendante et correcte.

    // Hysteresis : une tuile vue dans les HYSTERESIS_FRAMES dernieres frames est conservee
    // meme si le cover brut de cette frame la rate (transitoire au bord du frustum pendant un
    // geste). Filet de securite en plus de la plage d'altitude stable ci-dessus. Borne courte
    // (~0,25 s a 60 fps) pour ne pas trainer de tuiles hors champ ni gonfler les drapages.
    // Isomaps : hystérésis CONSTANTE 60 (~1 s). Le cover oscille (état async des tuiles), une fenêtre
    // longue le lisse (union stable). L'adaptatif (fenêtre courte en mouvement) a été tenté mais un
    // micro-mouvement rétrécissait la fenêtre → les tuiles tenues par la queue 46-60 tombaient d'un coup
    // = disparition immédiate au moindre geste. Constante = stable même en micro-mouvement. NB : c'est un
    // MASQUE ; la vraie racine (cover qui oscille, textures qui ne convergent pas) reste à traiter.
    // Isomaps : fenêtre en TEMPS RÉEL (0,5 s murale) — pas en frames : au repos la carte ne repeint qu'à
    // l'arrivée de tuiles, « 30 frames » s'étalaient sur 15-20 s réelles (maillage gonflé qui drainait
    // lentement). 0,5 s masque toujours l'oscillation du cover (blips de quelques frames) sans empiler
    // les niveaux traversés pendant un zoom continu (cf. OOM 236 tuiles / 3 Go).
    constexpr double kHysteresisSeconds = 0.5;
    const double nowS = util::MonotonicTimer::now().count();
    for (const auto& id : out) {
        meshTileLastVisible[id] = nowS;
    }
    for (auto it = meshTileLastVisible.begin(); it != meshTileLastVisible.end();) {
        if (nowS - it->second > kHysteresisSeconds) {
            it = meshTileLastVisible.erase(it);
        } else {
            out.insert(it->first);
            ++it;
        }
    }

    // Borne le cache d'altitude (l'union monotone accumule chaque tuile visitee par le DFS au
    // fil des pans) : au-dela d'un plafond genereux, on repart de zero — il se reremplit en
    // quelques frames et l'over-coverage transitoire est invisible.
    // Isomaps : JAMAIS de clear TOTAL — le vidage brutal perdait TOUTES les plages d'altitude d'un coup →
    // boîtes de frustum plates → cover effondré (maillage 8) puis reconstruit → OSCILLATION PERMANENTE au
    // repos (8↔100 mesuré au 🧭 STATUS), métronome du « bleu à travers les tuiles ». On n'évince que les
    // entrées FINES (z≥12, re-dérivées instantanément des tuiles chargées) et on GARDE les grossières
    // (peu nombreuses, ce sont elles qui stabilisent les grandes zones).
    constexpr size_t maxElevationCache = 32768;
    if (meshTileElevation.size() > maxElevationCache) {
        for (auto it = meshTileElevation.begin(); it != meshTileElevation.end();) {
            it = (it->first.z >= 12) ? meshTileElevation.erase(it) : std::next(it);
        }
        for (auto it = meshTileElevationFinal.begin(); it != meshTileElevationFinal.end();) {
            it = (it->z >= 12) ? meshTileElevationFinal.erase(it) : std::next(it);
        }
    }

    // Isomaps : PLAFOND FINAL (crash-safety) — appliqué APRÈS l'hystérésis, il borne le total QUOI QU'IL
    // ARRIVE. Le 700 plus haut ne cape que le cover instantané ; l'hystérésis empile ensuite les tuiles
    // vues sur plusieurs frames pendant un geste (mesuré jusqu'à ~2700 → OOM/crash). On garde les tuiles
    // les plus proches du CENTRE écran (ce qu'on regarde) ; le lointain/hors-champ largué est de toute
    // façon noyé dans la brume. NB : ne borne pas assez pour égaler Mapbox (voir découplage drapage) —
    // ici c'est un filet anti-crash, pas l'optimisation finale.
    constexpr size_t kMaxFinalMeshTiles = 350; // recalibré ère 512 (cf. maxMeshTiles ; 900 → OOM à 3 Go)
    if (demBaseZoom > 0 && out.size() > kMaxFinalMeshTiles) {
        const auto center = TileCoordinate::fromLatLng(demBaseZoom, state.getLatLng());
        const auto dist2 = [&](const UnwrappedTileID& t) {
            const int shift = demBaseZoom >= t.canonical.z ? (demBaseZoom - t.canonical.z) : 0;
            const double s = static_cast<double>(1u << shift);
            const double tx = (t.canonical.x + 0.5) * s, ty = (t.canonical.y + 0.5) * s;
            return (tx - center.p.x) * (tx - center.p.x) + (ty - center.p.y) * (ty - center.p.y);
        };
        std::vector<UnwrappedTileID> byDist(out.begin(), out.end());
        std::nth_element(
            byDist.begin(), byDist.begin() + kMaxFinalMeshTiles, byDist.end(),
            [&](const UnwrappedTileID& a, const UnwrappedTileID& b) { return dist2(a) < dist2(b); });
        out = std::set<UnwrappedTileID>(byDist.begin(), byDist.begin() + kMaxFinalMeshTiles);
    }

    return out;
}

std::set<UnwrappedTileID> RenderTerrain::augmentWithFrustumCover(std::set<UnwrappedTileID> tiles,
                                                                const TransformState& state) {
    // La source DEM ne couvre pas toujours tout le frustum visible (coins de l'ecran a
    // fort pitch) : on ajoute les tuiles du frustum qu'elle a ratees, remplies par le DEM
    // d'ancetre dans la boucle de creation. Les cibles de drapage doivent etre creees pour
    // le meme ensemble (voir renderer_impl), sinon la tuile ajoutee est sautee.
    uint8_t maxZ = 0;
    for (const auto& id : tiles) {
        maxZ = std::max(maxZ, id.canonical.z);
    }
    if (maxZ == 0) {
        return tiles;
    }
    const auto frustum = util::tileCover({.transformState = state}, maxZ, Range<uint8_t>(0, maxZ));
    std::set<UnwrappedTileID> cover;
    for (const auto& oid : frustum) {
        cover.insert(UnwrappedTileID(oid.wrap, oid.canonical));
    }
    // Dilatation d'un cran : tileCover peut rater le tout dernier bord (coin bas de l'ecran
    // a fort pitch). On ajoute les voisins 8 de chaque tuile couverte pour que les coins
    // deviennent candidats ; le frustum cull retire ensuite ce qui est vraiment hors champ.
    for (const auto& id : cover) {
        tiles.insert(id);
        const int z = static_cast<int>(id.canonical.z);
        const int n = 1 << z;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                const int ny = static_cast<int>(id.canonical.y) + dy;
                if (ny < 0 || ny >= n) {
                    continue;
                }
                const int nx = ((static_cast<int>(id.canonical.x) + dx) % n + n) % n;
                tiles.insert(UnwrappedTileID(id.wrap, CanonicalTileID(static_cast<uint8_t>(z),
                                                                      static_cast<uint32_t>(nx),
                                                                      static_cast<uint32_t>(ny))));
            }
        }
    }
    return tiles;
}

std::set<UnwrappedTileID> RenderTerrain::expandToDeepestCover(const std::set<UnwrappedTileID>& tileIDs) {
    std::set<UnwrappedTileID> out;
    const std::function<void(const UnwrappedTileID&)> insert = [&](const UnwrappedTileID& id) {
        bool hasDeeper = false;
        for (const auto& other : tileIDs) {
            if (other.isChildOf(id)) {
                hasDeeper = true;
                break;
            }
        }
        if (!hasDeeper) {
            out.insert(id);
            return;
        }
        // A deeper tile overlaps this one: replace it by its children (the
        // children that are themselves in the set are handled at top level)
        for (const auto& childCanonical : id.canonical.children()) {
            const UnwrappedTileID child(id.wrap, childCanonical);
            if (!tileIDs.contains(child)) {
                insert(child);
            }
        }
    };
    for (const auto& id : tileIDs) {
        insert(id);
    }

    return out;
}

void RenderTerrain::rebuildBasemapTextures() {
    basemapTextures.clear();
    if (!basemapSource) {
        return;
    }
    // Une passe sur les tuiles raster chargées → map id -> texture. Maillage et raster partagent le
    // même LOD (Distance) donc les ids coïncident souvent (échantillonnage 1:1, uv terrain = uv tuile).
    auto renderTiles = basemapSource->getRawRenderTiles();
    for (const auto& renderTile : *renderTiles) {
        const auto& tile = renderTile.getTile();
        if (tile.kind != Tile::Kind::Raster) {
            continue;
        }
        const auto* rasterTile = static_cast<const RasterTile*>(&tile);
        if (auto* bucket = rasterTile->getBucket(); bucket && bucket->texture2d) {
            basemapTextures[renderTile.id] = bucket->texture2d;
        }
    }
    // Isomaps ⏱ : l'ensemble des tuiles satellite a changé → les liaisons des drawables sont à re-résoudre.
    std::set<UnwrappedTileID> keys;
    for (const auto& [id, tex] : basemapTextures) {
        keys.insert(id);
    }
    if (keys != lastBasemapKeys) {
        lastBasemapKeys = std::move(keys);
        ++bindingsEpoch;
    }
}

std::shared_ptr<gfx::Texture2D> RenderTerrain::getBasemapTextureForTile(
    const UnwrappedTileID& meshTile, std::array<float, 4>& outMapCoords) const {
    // Tuile exacte : uv 1:1.
    if (const auto it = basemapTextures.find(meshTile); it != basemapTextures.end()) {
        outMapCoords = {{1.0f, 0.0f, 0.0f, 0.0f}};
        return it->second;
    }
    // Fallback : remonter vers les ancêtres jusqu'à une tuile raster chargée (la plus fine dispo), et
    // remapper l'uv sur sa sous-région. Évite les trous/vue traversante pendant le chargement.
    for (int z = static_cast<int>(meshTile.canonical.z) - 1; z >= 0; --z) {
        const uint8_t dz = static_cast<uint8_t>(static_cast<int>(meshTile.canonical.z) - z);
        const UnwrappedTileID ancestor(
            meshTile.wrap,
            CanonicalTileID(static_cast<uint8_t>(z), meshTile.canonical.x >> dz, meshTile.canonical.y >> dz));
        if (const auto it = basemapTextures.find(ancestor); it != basemapTextures.end()) {
            const auto off = demSubTileOffset(meshTile.canonical, ancestor.canonical);
            outMapCoords = {{1.0f / off.scale, off.dx / off.scale, off.dy / off.scale, 0.0f}};
            return it->second;
        }
    }
    return nullptr;
}

void RenderTerrain::update(RenderOrchestrator& orchestrator,
                           gfx::ShaderRegistry& shaders,
                           gfx::Context& context,
                           TexturePool& texturePool,
                           const TransformState& state,
                           const std::shared_ptr<UpdateParameters>& updateParameters,
                           const RenderTree& /*renderTree*/,
                           UniqueChangeRequestVec& changes) {
    // ⏱ Isomaps DIAG fluidité (à retirer) : durée de la mise à jour du maillage terrain.
    const double isomapsMeshT0 = util::MonotonicTimer::now().count();
    // Find the DEM source if we haven't already
    if (!demSource && !impl->sourceID.empty()) {
        demSource = orchestrator.getRenderSource(impl->sourceID);
        if (!demSource) {
            Log::Warning(Event::Render, "Terrain could not find DEM source: " + impl->sourceID);
        }
    }

    // Isomaps : source raster BASEMAP (échantillonnage direct sur le maillage). v1 : explicite
    // uniquement (impl->basemapSourceID) ; vide → nullptr → drapage offscreen historique.
    basemapSource = impl->basemapSourceID.empty() ? nullptr
                                                  : orchestrator.getRenderSource(impl->basemapSourceID);
    rebuildBasemapTextures(); // map id->texture (O(1) au lookup dans la boucle de drawables)

    // Create layer group if we don't have one (including after rebuild)
    if (!layerGroup) {
        if (auto layerGroup_ = context.createLayerGroup(TERRAIN_LAYER_INDEX, /*initialCapacity=*/1, "terrain", false)) {
            layerGroup = std::move(layerGroup_);
            activateLayerGroup(true, changes);
        } else {
            Log::Error(Event::Render, "Failed to create terrain layer group");
            return;
        }
    }

    // Depth-pass twin of the terrain layer group; not activated in the
    // orchestrator, rendered only by renderDepth into the depth target
    if (!depthLayerGroup) {
        depthLayerGroup = context.createLayerGroup(TERRAIN_LAYER_INDEX, /*initialCapacity=*/1, "terrain-depth", false);
    }

    // Create tweaker if we don't have one
    if (!tweaker) {
        tweaker = std::make_unique<TerrainLayerTweaker>(this);
    }

    // If we don't have a DEM source, we can't create terrain drawables
    if (!demSource) {
        return;
    }

    // Get tiles from the DEM source. Isomaps : ne PAS retourner si vide — au lancement, l'aperçu DEM
    // grossier RETENU (z7/z10, décodé plus bas) permet déjà de mailler un fond flou (coarse-first) au
    // lieu de laisser l'écran bleu le temps que les tuiles rendues arrivent.
    auto renderTiles = demSource->getRawRenderTiles();

    // Cast to LayerGroup for addDrawable
    auto* lg = static_cast<LayerGroup*>(layerGroup.get());
    if (!lg) {
        return;
    }

    // Decode and cache the DEM textures of loaded DEM tiles
    ++demUpdateCounter;
    // 🧪 DIAG vanish (à retirer) — compteurs de la boucle de dessin de la frame précédente.
    static int s_drawn = 0, s_skipBM = 0, s_skipDEM = 0;
    static std::vector<std::string> s_flats; // 🕳 tuiles à liaison DEM non propre (id→ancêtre/PLAT)
    for (const auto& renderTile : *renderTiles) {
        const auto& tile = renderTile.getTile();
        if (tile.kind != Tile::Kind::RasterDEM) {
            continue;
        }
        auto* demTile = const_cast<RasterDEMTile*>(static_cast<const RasterDEMTile*>(&tile));
        auto* hillshadeBucket = demTile->getBucket();
        const auto* demData = hillshadeBucket ? &hillshadeBucket->getDEMData() : nullptr;
        if (demData && demData->getImagePtr() && !demData->getImagePtr()->size.isEmpty()) {
            // All tiles come from the same raster-dem source, so they share one
            // encoding and DEM dimension
            demUnpackVector = demData->getUnpackVector();
            demDim = demData->dim;
            if (auto existing = demTextures.find(renderTile.id); existing != demTextures.end()) {
                existing->second.lastUsed = demUpdateCounter;
            } else if (auto texture = createDEMTexture(context, *demData)) {
                // Keep the texture available for elevation sampling by non-draped layers
                demTextures[renderTile.id] = {texture, demData->dim, demUpdateCounter};
                ++bindingsEpoch; // nouveau DEM décodé → liaisons à re-résoudre
            }
        }
    }

    // Isomaps : décode AUSSI l'aperçu DEM grossier RETENU (non rendu, cf. tile_pyramid) dans demTextures, pour
    // qu'il serve d'ANCÊTRE au maillage. Sinon, au pan en altitude, les mailles dont le DEM fin n'est pas encore
    // chargé retombent sur le placeholder PLAT (plaques grises visibles quand on se balade). Avec l'ancêtre
    // grossier : relief flou, jamais plat. Borné aux niveaux grossiers (z<=10) → coût minime (décodage 1×,
    // ensuite simple refresh de lastUsed). Le DEM fin exact, quand il arrive, prime (demTier 2 > 1).
    if (const auto* loaded = demSource->getLoadedTiles()) {
        for (const auto& [oid, tilePtr] : *loaded) {
            if (oid.canonical.z > 10 || !tilePtr || tilePtr->kind != Tile::Kind::RasterDEM) {
                continue;
            }
            const UnwrappedTileID uid(oid.wrap, oid.canonical);
            if (auto it = demTextures.find(uid); it != demTextures.end()) {
                it->second.lastUsed = demUpdateCounter; // garde-le vivant (anti-éviction)
                continue;
            }
            auto* demTile = static_cast<RasterDEMTile*>(tilePtr.get());
            auto* bucket = demTile->getBucket();
            const auto* demData = bucket ? &bucket->getDEMData() : nullptr;
            if (demData && demData->getImagePtr() && !demData->getImagePtr()->size.isEmpty()) {
                demUnpackVector = demData->getUnpackVector();
                demDim = demData->dim;
                if (auto texture = createDEMTexture(context, *demData)) {
                    demTextures[uid] = {texture, demData->dim, demUpdateCounter};
                    ++bindingsEpoch; // aperçu DEM décodé → liaisons à re-résoudre
                }
            }
        }
    }

    // Couverture du maillage : cf. computeMeshCover (tileCover + altitude STABLE + hysteresis).
    // Publiee dans lastRenderedMeshTiles ; renderer_impl la relit pour les cibles de drapage
    // (update() precede render() dans la frame -> drapage et maillage strictement identiques).
    std::set<UnwrappedTileID> meshTiles = computeMeshCover(state, *updateParameters);
    lastRenderedMeshTiles = meshTiles;
    g_isomapsMeshTiles.store(static_cast<int>(meshTiles.size()), std::memory_order_relaxed);
    g_isomapsSatTiles.store(static_cast<int>(basemapTextures.size()), std::memory_order_relaxed);
    {
        // Isomaps 🧠 : octets GPU RÉELS (getDataSize) des deux caches de textures — pour attribuer le
        // footprint (jetsam à ~3,4 Go observé) plutôt que l'estimer par les comptes.
        long long satB = 0;
        for (const auto& [id, tex] : basemapTextures) {
            if (tex) satB += static_cast<long long>(tex->getDataSize());
        }
        long long demB = 0;
        for (const auto& [id, entry] : demTextures) {
            if (entry.texture) demB += static_cast<long long>(entry.texture->getDataSize());
        }
        g_isomapsSatBytes.store(satB, std::memory_order_relaxed);
        g_isomapsDemBytes.store(demB, std::memory_order_relaxed);
    }

    // 🧭 STATUS (diag léger permanent) — 1 ligne / 2 s : zoom, œil, maillage, textures. Suivi longue durée
    // (drain du maillage, charge mémoire textures) sans spam.
    {
        static double s_lastStatus = 0.0;
        const double t = util::MonotonicTimer::now().count();
        if (t - s_lastStatus > 2.0) {
            s_lastStatus = t;
            double eyeASL = 0.0;
            if (const auto l = state.getFreeCameraOptions().getLocation()) {
                eyeASL = l->altitude;
            }
            int drawableCount = 0;
            if (auto* lgc = static_cast<LayerGroup*>(layerGroup.get())) {
                lgc->visitDrawables([&](gfx::Drawable&) { ++drawableCount; });
            }
            std::array<int, 24> meshZ{};
            for (const auto& id : meshTiles) {
                if (id.canonical.z < 24) ++meshZ[id.canonical.z];
            }
            std::string meshHisto;
            for (int zz = 23; zz >= 0; --zz) {
                if (meshZ[zz] > 0) meshHisto += " z" + util::toString(zz) + ":" + util::toString(meshZ[zz]);
            }
            // 🔗 QUALITÉ DES LIAISONS (diag B3) : tiers DEM des drawables (2 = DEM propre, 1 = ancêtre,
            // 0 = placeholder plat) et écart de zoom du satellite lié (0 = exact, k = ancêtre k niveaux
            // au-dessus → k crans de flou). Un paquet de tuiles en tier 1 / satAnc≥3 au REPOS = zone
            // rabotée/délavée (sommets tronqués, rideau de skirt) par liaisons grossières, pas par le LOD.
            int tier0 = 0, tier1 = 0, tier2 = 0;
            std::array<int, 5> satAnc{}; // 0,1,2,3,≥4 niveaux au-dessus
            for (const auto& [tid, tier] : tilesWithDrawables) {
                (tier == 2 ? tier2 : (tier == 1 ? tier1 : tier0))++;
                if (const auto mc = drawableMapCoords.find(tid); mc != drawableMapCoords.end()) {
                    const int up = std::max(0, static_cast<int>(std::lround(-std::log2(mc->second[0]))));
                    ++satAnc[std::min(up, 4)];
                }
            }
            std::string liaisons = " 🔗 dem2/1/0=" + util::toString(tier2) + "/" + util::toString(tier1) +
                                   "/" + util::toString(tier0) + " satAnc0/1/2/3+=" +
                                   util::toString(satAnc[0]) + "/" + util::toString(satAnc[1]) + "/" +
                                   util::toString(satAnc[2]) + "/" + util::toString(satAnc[3] + satAnc[4]);
            if (!s_flats.empty()) {
                liaisons += " 🕳";
                for (const auto& f : s_flats) liaisons += f;
            }
            Log::Warning(Event::Render,
                         "🧭 STATUS zoom=" + util::toString(state.getZoom()) +
                             " œilASL=" + util::toString(static_cast<int>(eyeASL)) +
                             " centreAlt=" + util::toString(static_cast<int>(state.getCenterAltitude())) +
                             " maillage=" + util::toString(meshTiles.size()) +
                             " drawables=" + util::toString(drawableCount) +
                             " dessin(d/sB/sD)=" + util::toString(s_drawn) + "/" + util::toString(s_skipBM) +
                             "/" + util::toString(s_skipDEM) +
                             " satTex=" + util::toString(basemapTextures.size()) +
                             " demTex=" + util::toString(demTextures.size()) + liaisons + " |" + meshHisto);
        }
    }
    // Reset des compteurs de dessin (incrémentés dans la boucle ci-dessous, lus au prochain STATUS).
    s_drawn = s_skipBM = s_skipDEM = 0;
    s_flats.clear();

    // Isomaps : le RETRAIT des drawables sortis du cover est déplacé APRÈS la boucle de création et
    // conditionné à la RE-COUVERTURE (cf. bloc en fin de fonction). Sous budget-temps, retirer d'abord
    // et créer plus tard laissait des trous pendant la bascule (« tuiles qui s'effacent »).
    std::unordered_set<OverscaledTileID> currentTiles;
    for (const auto& id : meshTiles) {
        currentTiles.emplace(id.canonical.z, id.wrap, id.canonical);
    }
    auto* depthLg = static_cast<LayerGroup*>(depthLayerGroup.get());
    // Retain cached DEM textures that are related to the current tile set so they
    // can serve as ancestor fallbacks while exact tiles load (as maplibre-gl-js
    // retains terrain tiles in its source cache); drop unrelated ones.
    // Isomaps ⏱ : test de parenté en O(z) par ENTRÉE (remontée de chaîne parentale + ensemble des
    // ancêtres du maillage) — l'ancien double balayage isChildOf était O(maillage × textures)
    // ≈ 80 000 tests PAR PASSE avec ~500 textures : un des planchers mesurés des saccades.
    std::set<UnwrappedTileID> meshSet;
    std::set<UnwrappedTileID> meshAndAncestors;
    for (const auto& id : meshTiles) {
        meshSet.insert(id);
        for (int zz = static_cast<int>(id.canonical.z); zz >= 0; --zz) {
            meshAndAncestors.emplace(id.wrap, id.canonical.scaledTo(static_cast<uint8_t>(zz)));
        }
    }
    for (auto it = demTextures.begin(); it != demTextures.end();) {
        const UnwrappedTileID& tex = it->first;
        bool related = meshAndAncestors.contains(tex); // tuile du maillage ou ancêtre d'une d'elles
        for (int zz = static_cast<int>(tex.canonical.z) - 1; zz >= 0 && !related; --zz) {
            // descendante d'une tuile du maillage ?
            related = meshSet.contains(UnwrappedTileID(tex.wrap, tex.canonical.scaledTo(static_cast<uint8_t>(zz))));
        }
        it = related ? std::next(it) : demTextures.erase(it);
    }
    // Cap the cache: ancestor/descendant relations accumulate while browsing
    // (zooming makes whole chains "related"), which previously grew past 2GB
    // of DEM textures and overflowed/OOMed. Evict least-recently-used entries
    // that were not used this frame until the cache is back under budget.
    if (demTextures.size() > maxDEMTextures) {
        std::vector<std::pair<uint64_t, UnwrappedTileID>> evictable;
        for (const auto& [id, entry] : demTextures) {
            if (entry.lastUsed != demUpdateCounter) {
                evictable.emplace_back(entry.lastUsed, id);
            }
        }
        std::sort(evictable.begin(), evictable.end());
        for (const auto& [lastUsed, id] : evictable) {
            if (demTextures.size() <= maxDEMTextures) {
                break;
            }
            demTextures.erase(id);
        }
    }

    // Isomaps — raccord des aretes. Pour chaque tuile du maillage, le deficit de zoom
    // de la voisine de chaque cote (ouest, est, nord, sud). Une voisine plus grossiere
    // echantillonne le DEM moins finement : sans alignement, les deux tuiles lisent des
    // altitudes differentes sur leur arete commune et la couture s'ouvre.
    // Raccord des aretes — resolution DEM EFFECTIVE de chaque tuile du maillage : le zoom de
    // son propre DEM s'il est charge, sinon celui du meilleur ancetre disponible. Une tuile fine
    // bordant une grossiere cree une marche d'altitude, donc un trou que la jupe ne couvre pas ;
    // on la comble en alignant l'arete sur la resolution de la voisine.
    meshTileDemZoom.clear();
    for (const auto& id : meshTiles) {
        // Isomaps ⏱ : remontée parentale directe (≤z lookups) au lieu du balayage de toutes les textures.
        if (auto own = demTextures.find(id); own != demTextures.end()) {
            own->second.lastUsed = demUpdateCounter; // touche : protège de l'éviction sous budget-temps
            meshTileDemZoom[id] = static_cast<int>(id.canonical.z);
            continue;
        }
        int best = -1;
        for (int zz = static_cast<int>(id.canonical.z) - 1; zz >= 0; --zz) {
            if (demTextures.contains(UnwrappedTileID(id.wrap, id.canonical.scaledTo(static_cast<uint8_t>(zz))))) {
                best = zz;
                break;
            }
        }
        meshTileDemZoom[id] = best;
    }
    drawableEdgeDz.clear();
    for (const auto& id : meshTiles) {
        std::array<float, 4> dz{{0.0f, 0.0f, 0.0f, 0.0f}};
        const auto self = meshTileDemZoom.find(id);
        if (self != meshTileDemZoom.end() && self->second >= 0) {
            const int z = static_cast<int>(id.canonical.z);
            const int n = 1 << z;
            const std::array<std::pair<int, int>, 4> dirs{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
            for (size_t e = 0; e < dirs.size(); ++e) {
                const int nx = static_cast<int>(id.canonical.x) + dirs[e].first;
                const int ny = static_cast<int>(id.canonical.y) + dirs[e].second;
                if (ny < 0 || ny >= n) {
                    continue;
                }
                const int wx = ((nx % n) + n) % n;
                for (int zz = z; zz >= 0; --zz) {
                    const int sh = z - zz;
                    const UnwrappedTileID cand(id.wrap,
                                               CanonicalTileID(static_cast<uint8_t>(zz),
                                                               static_cast<uint32_t>(wx >> sh),
                                                               static_cast<uint32_t>(ny >> sh)));
                    const auto nb = meshTileDemZoom.find(cand);
                    if (nb != meshTileDemZoom.end()) {
                        // Isomaps : le raccord doit viser la GRILLE DE MAILLAGE de la voisine, pas ses
                        // texels DEM. La voisine rend des segments LINÉAIRES entre SES sommets (pas
                        // 8192·2^(z−zz)/MESH_SIZE en unités locales) ; s'aligner sur la grille de texels
                        // (~4× plus fine) laissait notre bord suivre la courbure du DEM ENTRE ses sommets
                        // → marche de dizaines/centaines de mètres sur les faces raides (« sommet coupé
                        // à l'horizontale » mesuré ; la pyramide DEM, elle, est cohérente : p95 ≈ 7 m).
                        // On passe désormais le PAS DE RACCORD en unités locales (0 = pas de raccord) :
                        // échantillonner aux positions des sommets de la voisine (mêmes valeurs à ±7 m)
                        // et interpoler linéairement entre = le MÊME segment que le sien.
                        if (zz < z) {
                            dz[e] = static_cast<float>(util::EXTENT / MESH_SIZE) * std::exp2(static_cast<float>(z - zz));
                        }
                        break;
                    }
                }
            }
        }
        drawableEdgeDz[OverscaledTileID(id.canonical.z, id.wrap, id.canonical)] = dz;
    }

    // Create terrain drawables for each mesh tile.
    // Isomaps ⏱ BUDGET-TEMPS : créer ~4 ms/drawable × 100+ tuiles dans UNE frame = les pics mesurés de
    // 500-900 ms pendant la bascule. On trie CENTRE D'ABORD et on s'arrête au budget : l'existant reste
    // affiché (échange en place / ancêtres), le reste se crée sur les frames suivantes (repaint demandé).
    constexpr double kMeshBudgetMs = 8.0;
    bool isomapsBudgetOut = false;
    // Budget mesuré depuis le DÉBUT DE LA BOUCLE DE CRÉATION — pas depuis le début d'update() : la phase
    // pré-boucle (computeMeshCover…) peut dépasser 8 ms à elle seule → budget épuisé avant la PREMIÈRE
    // création → famine totale (constaté : dessinés=0 en boucle, drawables figés, repaints infinis).
    const double isomapsCreateT0 = util::MonotonicTimer::now().count();
    std::vector<UnwrappedTileID> orderedMeshTiles(meshTiles.begin(), meshTiles.end());
    {
        const auto ctr = TileCoordinate::fromLatLng(0, state.getLatLng()).p; // centre en mercator [0..1]
        std::sort(orderedMeshTiles.begin(), orderedMeshTiles.end(), [&](const auto& a, const auto& b) {
            const double sa = std::exp2(static_cast<double>(a.canonical.z));
            const double sb = std::exp2(static_cast<double>(b.canonical.z));
            const double dax = (a.canonical.x + 0.5) / sa - ctr.x;
            const double day = (a.canonical.y + 0.5) / sa - ctr.y;
            const double dbx = (b.canonical.x + 0.5) / sb - ctr.x;
            const double dby = (b.canonical.y + 0.5) / sb - ctr.y;
            return dax * dax + day * day < dbx * dbx + dby * dby;
        });
    }
    for (size_t tileIdx = 0; tileIdx < orderedMeshTiles.size(); ++tileIdx) {
        const auto& unwrapped = orderedMeshTiles[tileIdx];
        const OverscaledTileID tileID(unwrapped.canonical.z, unwrapped.wrap, unwrapped.canonical);

        // Fast-skip O(1) : drawable dont les liaisons sont À JOUR (rien de neuf depuis sa résolution), ou
        // déjà au MAXIMUM (DEM propre + satellite exact — rien à améliorer). L'ancien critère « tier 2 »
        // seul ignorait le satellite : DEM arrivé avant sa tuile satellite → lié à l'ancêtre FLOU à vie.
        if (const auto existing = tilesWithDrawables.find(tileID); existing != tilesWithDrawables.end()) {
            const auto itE = drawableEpoch.find(tileID);
            const bool upToDate = itE != drawableEpoch.end() && itE->second == bindingsEpoch;
            bool exactMap = true;
            if (basemapSource) {
                const auto exMap = drawableMapCoords.find(tileID);
                exactMap = exMap != drawableMapCoords.end() && exMap->second[0] >= 1.0f;
            }
            if (upToDate || (existing->second == 2 && exactMap)) {
                drawableEpoch[tileID] = bindingsEpoch; // stampe le cas « au maximum » → plus jamais pendant
                ++s_drawn;
                continue;
            }
        }

        // Budget épuisé : on reporte le reste (plus périphérique, la liste est triée) à la frame suivante.
        // CONVERGENCE : la continuation (repaint) n'est demandée que s'il reste une tuile SANS drawable —
        // la seule dont l'absence fait un trou. Sinon (reste = simples montées en qualité tier<2), on
        // s'arrête là : l'arrivée d'une texture déclenche déjà son propre update. Sans ce test, le seul
        // ENTRETIEN de ~200 tuiles dépassait 8 ms → repaint → re-entretien → boucle infinie AU REPOS
        // (mesuré : UPDATE ×5/s à 55 ms sans toucher l'écran).
        if ((util::MonotonicTimer::now().count() - isomapsCreateT0) * 1000.0 > kMeshBudgetMs) {
            for (size_t j = tileIdx; j < orderedMeshTiles.size(); ++j) {
                const OverscaledTileID rid(
                    orderedMeshTiles[j].canonical.z, orderedMeshTiles[j].wrap, orderedMeshTiles[j].canonical);
                if (!tilesWithDrawables.contains(rid)) {
                    isomapsBudgetOut = true; // trou potentiel → continuer coûte que coûte
                    break;
                }
                const auto itE = drawableEpoch.find(rid);
                if (itE == drawableEpoch.end() || itE->second != bindingsEpoch) {
                    isomapsBudgetOut = true; // liaisons périmées (montée en qualité en attente) → continuer
                    break;
                }
            }
            break;
        }

        // Resolve the DEM texture: the tile's own decoded DEM if available,
        // otherwise the closest cached ancestor as a fallback so the terrain
        // mesh stays up while the tile loads (as maplibre-gl-js does),
        // otherwise the flat placeholder
        std::shared_ptr<gfx::Texture2D> demTexture;
        // {scale, x offset, y offset, DEM dim}: maps tile-local coords (0..EXTENT)
        // into the bound DEM tile's normalized space, matching getTerrainData so
        // the terrain mesh and the elevated layers sample identically. The DEM
        // dimension rides in .w for the shader's get_elevation() call.
        std::array<float, 4> demCoords{{1.0f / util::EXTENT, 0.0f, 0.0f, static_cast<float>(demDim)}};
        uint8_t demTier = 0;

        if (auto cached = demTextures.find(unwrapped); cached != demTextures.end()) {
            cached->second.lastUsed = demUpdateCounter;
            demTexture = cached->second.texture;
            demTier = 2;
        } else {
            // Fall back to the closest cached ancestor DEM.
            // Isomaps ⏱ : remontée parentale directe (≤z lookups) au lieu du balayage complet du cache.
            const UnwrappedTileID* ancestorID = nullptr;
            DEMTextureEntry* ancestorEntry = nullptr;
            UnwrappedTileID ancestorFound(0, {0, 0, 0});
            for (int zz = static_cast<int>(unwrapped.canonical.z) - 1; zz >= 0; --zz) {
                const UnwrappedTileID cand(unwrapped.wrap, unwrapped.canonical.scaledTo(static_cast<uint8_t>(zz)));
                if (auto itA = demTextures.find(cand); itA != demTextures.end()) {
                    ancestorFound = cand;
                    ancestorID = &ancestorFound;
                    ancestorEntry = &itA->second;
                    demTexture = itA->second.texture;
                    break;
                }
            }
            if (!demTexture) {
                // No DEM at all yet: render the mesh flat with the placeholder
                // DEM so the draped map still shows (a briefly flat area is
                // less jarring than a hole in the terrain)
                demTexture = getPlaceholderDEMTexture(context);
                if (!demTexture) {
                    ++s_skipDEM;
                    continue;
                }
            } else {
                ancestorEntry->lastUsed = demUpdateCounter;
                demTier = 1;
                const auto off = demSubTileOffset(unwrapped.canonical, ancestorID->canonical);
                demCoords = {{1.0f / (util::EXTENT * off.scale),
                              off.dx / off.scale,
                              off.dy / off.scale,
                              static_cast<float>(demDim)}};
            }
        }

        // 🕳 DIAG plaques plates (B3) : identité des tuiles dont la liaison DEM n'est pas propre — id +
        // ancêtre réellement lié (ou PLAT = placeholder). Lues dans le STATUS pour vérifier côté serveur.
        if (demTier < 2 && s_flats.size() < 3) {
            std::string what = (demTier == 1) ? "→anc" : "→PLAT";
            s_flats.push_back(" z" + util::toString(static_cast<int>(unwrapped.canonical.z)) + "/" +
                              util::toString(unwrapped.canonical.x) + "/" +
                              util::toString(unwrapped.canonical.y) + what +
                              (demTier == 1 ? util::toString(static_cast<int>(demCoords[0] > 0
                                                                                  ? std::lround(-std::log2(
                                                                                        demCoords[0] * util::EXTENT))
                                                                                  : -1))
                                            : ""));
        }

        // Isomaps : texture raster basemap (échantillonnage direct) + transform UV. On la calcule ICI,
        // avant le check de réutilisation, pour pouvoir recréer le drawable quand une tuile raster PLUS
        // FINE devient dispo (map_coords[0] : 1.0 = exacte, <1 = ancêtre ; plus grand = plus fin).
        std::shared_ptr<gfx::Texture2D> mapTexture;
        std::array<float, 4> mapCoords{{1.0f, 0.0f, 0.0f, 0.0f}};
        if (basemapSource) {
            mapTexture = getBasemapTextureForTile(unwrapped, mapCoords);
            if (!mapTexture) {
                ++s_skipBM;
                continue; // ni exacte ni ancêtre chargé (rare, transitoire tout au début)
            }
        }

        // If a drawable already exists for this tile, keep it until a higher DEM quality tier — OR a
        // finer raster basemap tile (Isomaps) — becomes available.
        if (const auto existing = tilesWithDrawables.find(tileID); existing != tilesWithDrawables.end()) {
            const auto exMap = drawableMapCoords.find(tileID);
            const float existingMapScale = exMap != drawableMapCoords.end() ? exMap->second[0] : 1.0f;
            if (existing->second >= demTier && (!basemapSource || existingMapScale >= mapCoords[0])) {
                drawableEpoch[tileID] = bindingsEpoch; // rien de mieux disponible À CETTE époque
                ++s_drawn;
                continue;
            }
            // Isomaps : ÉCHANGE EN PLACE — les textures se remplacent sur le drawable existant (slots 0=DEM,
            // 1=basemap) et le tweaker relit dem/map coords À CHAQUE frame → aucune recréation nécessaire.
            // L'ancienne recréation (memcpy du maillage + buffers Metal, ×2 avec le jumeau depth), des
            // dizaines de fois par seconde pendant un pan, était une source majeure de saccades.
            drawableDemCoords[tileID] = demCoords;
            drawableMapCoords[tileID] = mapCoords;
            bool swapped = false;
            lg->visitDrawables([&](gfx::Drawable& drawable) {
                if (drawable.getTileID() && *drawable.getTileID() == tileID) {
                    drawable.setTexture(demTexture, 0);
                    if (basemapSource) {
                        drawable.setTexture(mapTexture, 1);
                    }
                    swapped = true;
                }
            });
            if (depthLg) {
                depthLg->visitDrawables([&](gfx::Drawable& drawable) {
                    if (drawable.getTileID() && *drawable.getTileID() == tileID) {
                        drawable.setTexture(demTexture, 0);
                    }
                });
            }
            if (swapped) {
                existing->second = demTier;
                drawableEpoch[tileID] = bindingsEpoch;
                ++s_drawn;
                continue;
            }
            drawableEpoch.erase(tileID);
            // État incohérent (drawable introuvable) → retomber sur la recréation classique.
            lg->removeDrawablesIf(
                [&](gfx::Drawable& drawable) { return drawable.getTileID() && *drawable.getTileID() == tileID; });
            if (depthLg) {
                depthLg->removeDrawablesIf(
                    [&](gfx::Drawable& drawable) { return drawable.getTileID() && *drawable.getTileID() == tileID; });
            }
            tilesWithDrawables.erase(existing);
        }
        drawableDemCoords[tileID] = demCoords;

        // Create terrain drawable. Isomaps : si basemap désignée, mapTexture = tuile raster échantillonnée
        // DIRECTEMENT (calculée plus haut, avec fallback ancêtre) — pas de drapage offscreen. Sinon,
        // drapage historique via la cible offscreen.
        if (!basemapSource) {
            const auto renderTarget = texturePool.getRenderTarget(unwrapped);
            if (!renderTarget) {
                continue;
            }
            mapTexture = renderTarget->getTexture();
        }
        drawableMapCoords[tileID] = mapCoords;
        auto drawable = createDrawableForTile(context, shaders, tileID, demTexture, mapTexture);
        if (drawable) {
            ++s_drawn;
            lg->addDrawable(std::move(drawable));
            tilesWithDrawables[tileID] = demTier;
            drawableEpoch[tileID] = bindingsEpoch;
            if (depthLg) {
                if (auto depthDrawable = createDrawableForTile(
                        context, shaders, tileID, demTexture, nullptr, /*depthPass=*/true)) {
                    depthLg->addDrawable(std::move(depthDrawable));
                }
            }
        }
    }
    // Retrait des drawables SORTIS du cover, seulement quand leur zone est RE-COUVERTE :
    //  - un ancêtre (ou la tuile d'un cover plus grossier) présent AVEC drawable la recouvre, ou
    //  - TOUS les descendants du cover qui la pavent ont leur drawable.
    // Sinon on la garde une frame de plus (mieux un ancien contenu qu'un trou). Chevauchement
    // parent/enfants possible quelques frames : terrain opaque, artefact mineur et transitoire.
    {
        std::vector<OverscaledTileID> removable;
        for (const auto& entry : tilesWithDrawables) {
            const OverscaledTileID& tid = entry.first;
            if (currentTiles.contains(tid)) {
                continue;
            }
            const UnwrappedTileID tuw = tid.toUnwrapped();
            bool covered = false;
            bool ancestorInCover = false; // un ancêtre est dans le cover (drawable prêt ou non)
            for (int zz = static_cast<int>(tuw.canonical.z) - 1; zz >= 0 && !covered; --zz) {
                const CanonicalTileID anc = tuw.canonical.scaledTo(static_cast<uint8_t>(zz));
                const OverscaledTileID ancId(anc.z, tuw.wrap, anc);
                if (currentTiles.contains(ancId)) {
                    ancestorInCover = true;
                    covered = tilesWithDrawables.contains(ancId);
                }
            }
            bool any = false;
            if (!covered) {
                bool all = true;
                for (const auto& id : meshTiles) {
                    if (id.canonical.z > tuw.canonical.z && id.wrap == tuw.wrap &&
                        id.canonical.scaledTo(tuw.canonical.z) == tuw.canonical) {
                        any = true;
                        if (!tilesWithDrawables.contains(OverscaledTileID(id.canonical.z, id.wrap, id.canonical))) {
                            all = false;
                            break;
                        }
                    }
                }
                covered = any && all;
            }
            // ZOMBIES : une tuile SANS AUCUN LIEN avec le cover courant (ni ancêtre dans le cover, ni
            // descendant) a sa zone HORS VUE — elle ne sera jamais « re-couverte », donc l'ancien critère
            // la gardait POUR TOUJOURS, rendue avec des liaisons figées (jamais re-résolues puisque hors
            // maillage). Au retour de la caméra : surface périmée en travers du relief actuel (mesuré :
            // drawables=100 pour maillage=82 au repos ; bande sombre coupant une face, « mauvais ordre »).
            // La retirer ne peut pas créer de trou visible : sa zone n'est pas dans le cover.
            const bool unrelated = !ancestorInCover && !any;
            if (covered || unrelated) {
                removable.push_back(tid);
            }
        }
        if (!removable.empty()) {
            const std::unordered_set<OverscaledTileID> removableSet(removable.begin(), removable.end());
            lg->removeDrawablesIf([&](gfx::Drawable& drawable) {
                return drawable.getTileID() && removableSet.contains(*drawable.getTileID());
            });
            if (depthLg) {
                depthLg->removeDrawablesIf([&](gfx::Drawable& drawable) {
                    return drawable.getTileID() && removableSet.contains(*drawable.getTileID());
                });
            }
            for (const auto& tid : removable) {
                drawableDemCoords.erase(tid);
                drawableMapCoords.erase(tid);
                tilesWithDrawables.erase(tid);
                drawableEpoch.erase(tid);
            }
        }
    }
    if (isomapsBudgetOut) {
        orchestrator.isomapsRequestRepaint(); // créations reportées → garantir une frame de continuation
    }
    // ⏱ Isomaps DIAG fluidité (à retirer) : pics de la mise à jour maillage, agrégé 1 s.
    {
        const double meshMs = (util::MonotonicTimer::now().count() - isomapsMeshT0) * 1000.0;
        static double s_lastMeshLog = 0.0;
        static int s_meshSpikes = 0;
        static double s_meshWorst = 0.0;
        static int s_meshWorstDrawn = 0;
        if (meshMs > 4.0) {
            s_meshSpikes++;
            if (meshMs > s_meshWorst) {
                s_meshWorst = meshMs;
                s_meshWorstDrawn = s_drawn;
            }
        }
        const double nowMesh = util::MonotonicTimer::now().count();
        if (s_meshSpikes > 0 && nowMesh - s_lastMeshLog > 1.0) {
            Log::Warning(Event::Render,
                         "⏱ MESH ×" + std::to_string(s_meshSpikes) + " pire=" +
                             std::to_string(static_cast<int>(s_meshWorst)) + "ms dessinés=" +
                             std::to_string(s_meshWorstDrawn));
            s_meshSpikes = 0;
            s_meshWorst = 0.0;
            s_meshWorstDrawn = 0;
            s_lastMeshLog = nowMesh;
        }
    }
}

float RenderTerrain::getElevation(const UnwrappedTileID& tileID, float x, float y) const {
    if (!demSource) {
        return 0.0f;
    }

    // Isomaps : position normalisée [0,1[ du point demandé, indépendante du zoom de la requête.
    const double nQ = static_cast<double>(1ull << tileID.canonical.z);
    const double gx = (static_cast<double>(tileID.canonical.x) + static_cast<double>(x) / util::EXTENT) / nQ;
    const double gy = (static_cast<double>(tileID.canonical.y) + static_cast<double>(y) / util::EXTENT) / nQ;

    // On prend la tuile DEM chargée la PLUS FINE qui CONTIENT le point — ancêtre OU plus fine. L'ancien
    // test (isChildOf) ne matchait QUE les ancêtres → renvoyait 0 dès que seules des tuiles PLUS FINES que
    // la requête étaient chargées (ex. au zoom avant : z16 chargées, requête z14 → 0 → collision qui lâche).
    // On lit les tuiles CHARGÉES de la pyramide (pas seulement les rendues/frustum) : indispensable à
    // l'anti-collision, qui interroge le terrain HORS ÉCRAN (aperçu grossier DEM retenu, cf. TilePyramid).
    // On prend la PLUS FINE tuile DEM qui CONTIENT le point ET a des données DEM valides.
    const auto* loadedTiles = demSource->getLoadedTiles();
    if (!loadedTiles) {
        return 0.0f;
    }
    RasterDEMTile* demTile = nullptr;
    CanonicalTileID demC(0, 0, 0);
    int bestZoom = -1;
    for (const auto& entry : *loadedTiles) {
        Tile* t = entry.second.get();
        if (!t || t->kind != Tile::Kind::RasterDEM) {
            continue;
        }
        const auto& c = entry.first.canonical;
        const double nC = static_cast<double>(1ull << c.z);
        if (!(gx >= c.x / nC && gx < (c.x + 1) / nC && gy >= c.y / nC && gy < (c.y + 1) / nC) ||
            static_cast<int>(c.z) <= bestZoom) {
            continue;
        }
        auto* candBucket = static_cast<RasterDEMTile*>(t)->getBucket();
        if (!candBucket || !candBucket->getDEMData().getImagePtr() || candBucket->getDEMData().dim <= 0) {
            continue;
        }
        bestZoom = static_cast<int>(c.z);
        demTile = static_cast<RasterDEMTile*>(t);
        demC = c;
    }
    // Isomaps : MÉMOIRE DU MEILLEUR ÉCHANTILLON (cf. header, anti-oscillation renorm/LOD). Cellule ~75 m.
    const uint64_t holdCell = (static_cast<uint64_t>(static_cast<uint32_t>(gx * (1u << 19))) << 32) |
                              static_cast<uint32_t>(gy * (1u << 19));
    const auto held = elevationHold.find(holdCell);
    if (held != elevationHold.end() && static_cast<int>(held->second.z) > bestZoom) {
        return held->second.v; // un DEM plus fin a déjà répondu ici : sa valeur reste la meilleure
    }
    if (!demTile) {
        return 0.0f;
    }
    const auto& demData = demTile->getBucket()->getDEMData();

    // Sous-position [0,1[ du point dans la tuile DEM trouvée (ancêtre ou fine, peu importe).
    const double nD = static_cast<double>(1ull << demC.z);
    const double subX = gx * nD - static_cast<double>(demC.x);
    const double subY = gy * nD - static_cast<double>(demC.y);

    // Bilinear interpolation of the DEM texels, as in maplibre-gl-js Terrain.getDEMElevation
    const float dim = static_cast<float>(demData.dim);
    const float px = util::clamp(static_cast<float>(subX) * dim, 0.0f, dim - 1.0f);
    const float py = util::clamp(static_cast<float>(subY) * dim, 0.0f, dim - 1.0f);
    const auto x0 = static_cast<int32_t>(std::floor(px));
    const auto y0 = static_cast<int32_t>(std::floor(py));
    const float fx = px - static_cast<float>(x0);
    const float fy = py - static_cast<float>(y0);
    const float tl = static_cast<float>(demData.get(x0, y0));
    const float tr = static_cast<float>(demData.get(x0 + 1, y0));
    const float bl = static_cast<float>(demData.get(x0, y0 + 1));
    const float br = static_cast<float>(demData.get(x0 + 1, y0 + 1));
    const float top = tl + (tr - tl) * fx;
    const float bottom = bl + (br - bl) * fx;
    const float value = top + (bottom - top) * fy;
    // Mémorise le meilleur échantillon (à zoom égal, la dernière valeur = données à jour). Borne mémoire.
    if (elevationHold.size() > 100000) {
        elevationHold.clear();
    }
    elevationHold[holdCell] = {static_cast<int8_t>(bestZoom), value};
    return value;
}

float RenderTerrain::getElevationWithExaggeration(const UnwrappedTileID& tileID, float x, float y) const {
    return getElevation(tileID, x, y) * getExaggeration();
}

std::optional<RenderTerrain::TerrainData> RenderTerrain::getTerrainData(const UnwrappedTileID& tileID) const {
    // Find the DEM texture matching the requested tile, or its closest available ancestor
    const UnwrappedTileID* demTileID = nullptr;
    const DEMTextureEntry* entry = nullptr;
    int bestZoom = -1;
    for (const auto& [candidate, candidateEntry] : demTextures) {
        if ((candidate == tileID || tileID.isChildOf(candidate)) &&
            static_cast<int>(candidate.canonical.z) > bestZoom) {
            bestZoom = candidate.canonical.z;
            demTileID = &candidate;
            entry = &candidateEntry;
        }
    }
    if (!entry || !entry->texture) {
        return std::nullopt;
    }

    // Map tile-local coordinates (0..EXTENT) of the requested tile into
    // normalized coordinates (0..1) of the (possibly ancestor) DEM tile
    const auto off = demSubTileOffset(tileID.canonical, demTileID->canonical);

    return TerrainData{
        .demTexture = entry->texture,
        .demCoords = {{1.0f / (util::EXTENT * off.scale), off.dx / off.scale, off.dy / off.scale, 0.0f}},
        .demDim = static_cast<float>(entry->dim),
    };
}

const std::shared_ptr<gfx::Texture2D>& RenderTerrain::getPlaceholderDEMTexture(gfx::Context& context) {
    if (!placeholderDEMTexture) {
        auto image = std::make_shared<PremultipliedImage>(Size{1, 1});
        std::memset(image->data.get(), 0, image->bytes());
        placeholderDEMTexture = context.createTexture2D();
        placeholderDEMTexture->setImage(image);
        placeholderDEMTexture->setSamplerConfiguration({.filter = gfx::TextureFilterType::Nearest,
                                                        .wrapU = gfx::TextureWrapType::Clamp,
                                                        .wrapV = gfx::TextureWrapType::Clamp});
    }
    return placeholderDEMTexture;
}

void RenderTerrain::renderDepth(RenderOrchestrator& orchestrator,
                                const RenderTree& renderTree,
                                PaintParameters& parameters) {
    if (!depthLayerGroup || depthLayerGroup->empty()) {
        return;
    }
    const Size size = parameters.backend.getDefaultRenderable().getSize();
    if (!depthRenderTarget || depthRenderTarget->getTexture()->getSize() != size) {
        depthRenderTarget = parameters.context.createRenderTarget(
            size, gfx::TextureChannelDataType::UnsignedByte, /*stencil=*/false);
        if (!depthRenderTarget) {
            return;
        }
        // Far plane everywhere the terrain does not cover (unpack_depth(1,1,1,1) ~ 1.0)
        depthRenderTarget->setClearColor(Color::white());
        depthRenderTarget->addLayerGroup(depthLayerGroup, /*replace=*/true);
    }

    // The depth twin is intentionally not registered with the orchestrator, so it is
    // skipped by the global upload pass in Renderer::Impl::render(). Its drawables
    // would therefore reach draw() with buffers that were never uploaded — fatal on
    // Metal ("Index buffer not uploaded"). Upload it here, right before rendering.
    {
        const auto uploadPass = parameters.encoder->createUploadPass("terrain-depth-upload",
                                                                    parameters.backend.getDefaultRenderable());
#if !defined(NDEBUG)
        const auto debugGroup = uploadPass->createDebugGroup("terrain-depth-upload");
#endif
        depthLayerGroup->upload(*uploadPass);
    }

    depthRenderTarget->render(orchestrator, renderTree, parameters);
}

const std::shared_ptr<gfx::Texture2D>& RenderTerrain::getDepthTexture(gfx::Context& context) {
    if (depthRenderTarget) {
        return depthRenderTarget->getTexture();
    }
    if (!placeholderDepthTexture) {
        // Far-plane packed depth: symbols compare as visible until the pass runs
        auto image = std::make_shared<PremultipliedImage>(Size{1, 1});
        std::memset(image->data.get(), 0xFF, image->bytes());
        placeholderDepthTexture = context.createTexture2D();
        placeholderDepthTexture->setImage(image);
        placeholderDepthTexture->setSamplerConfiguration({.filter = gfx::TextureFilterType::Nearest,
                                                          .wrapU = gfx::TextureWrapType::Clamp,
                                                          .wrapV = gfx::TextureWrapType::Clamp});
    }
    return placeholderDepthTexture;
}

float RenderTerrain::getExaggeration() const {
    return impl->exaggeration;
}

const std::string& RenderTerrain::getSourceID() const {
    return impl->sourceID;
}

bool RenderTerrain::isEnabled() const {
    return !impl->sourceID.empty();
}

const RenderTerrain::TerrainMesh& RenderTerrain::getMesh(gfx::Context& context) {
    if (!mesh) {
        generateMesh(context);
    }
    return *mesh;
}

void RenderTerrain::generateMesh(gfx::Context& /*context*/) {
    // A regular grid mesh (reused for every tile, displaced by the DEM in the
    // vertex shader) plus a skirt: each tile edge is duplicated into a curtain
    // that the shader drops by u_ele_delta, hiding the cracks between neighbouring
    // tiles at different zoom levels. Ported from maplibre-gl-js Terrain
    // getTerrainMesh()/_buildSkirts().
    const size_t gridSize = MESH_SIZE;
    const size_t vps = gridSize + 1; // vertices per side
    const float step = static_cast<float>(util::EXTENT) / static_cast<float>(gridSize);

    std::vector<int16_t> vertices;
    std::vector<uint16_t> indices;

    // Each vertex is 4 shorts: x, y, skirt flag (0 = surface, 1 = skirt),
    // unused. uv is derived from x,y in the shader, so the 3rd/4th shorts are free
    // to carry the skirt flag (the native analog of gl-js Pos3d.z).
    const auto edgeFlags = [&](float x, float y) -> int16_t {
        int16_t f = 0;
        if (x <= 0.0f) f |= 1;
        if (x >= static_cast<float>(util::EXTENT)) f |= 2;
        if (y <= 0.0f) f |= 4;
        if (y >= static_cast<float>(util::EXTENT)) f |= 8;
        return f;
    };
    const auto addVert = [&](float x, float y, int16_t skirt) {
        vertices.push_back(static_cast<int16_t>(x));
        vertices.push_back(static_cast<int16_t>(y));
        vertices.push_back(skirt);
        vertices.push_back(edgeFlags(x, y));
    };

    // Surface grid
    for (size_t y = 0; y < vps; ++y) {
        for (size_t x = 0; x < vps; ++x) {
            addVert(x * step, y * step, 0);
        }
    }
    for (size_t y = 0; y < gridSize; ++y) {
        for (size_t x = 0; x < gridSize; ++x) {
            const uint16_t topLeft = static_cast<uint16_t>(y * vps + x);
            const uint16_t topRight = static_cast<uint16_t>(topLeft + 1);
            const uint16_t bottomLeft = static_cast<uint16_t>((y + 1) * vps + x);
            const uint16_t bottomRight = static_cast<uint16_t>(bottomLeft + 1);
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }

    // Top/bottom skirt rows (reference the grid's top/bottom edge rows)
    const auto extent = static_cast<float>(util::EXTENT);
    const uint16_t offsetTop = static_cast<uint16_t>(vertices.size() / 4);
    const uint16_t offsetTopEdge = 0;
    const uint16_t offsetBottom = static_cast<uint16_t>(offsetTop + vps);
    const uint16_t offsetBottomEdge = static_cast<uint16_t>(vps * gridSize);
    for (size_t x = 0; x < vps; ++x) {
        addVert(x * step, 0.0f, 1);
    }
    for (size_t x = 0; x < vps; ++x) {
        addVert(x * step, extent, 1);
    }
    for (uint16_t x = 0; x < gridSize; ++x) {
        indices.insert(indices.end(),
                       {static_cast<uint16_t>(offsetBottomEdge + x),
                        static_cast<uint16_t>(offsetBottom + x),
                        static_cast<uint16_t>(offsetBottom + x + 1),
                        static_cast<uint16_t>(offsetBottomEdge + x),
                        static_cast<uint16_t>(offsetBottom + x + 1),
                        static_cast<uint16_t>(offsetBottomEdge + x + 1),
                        static_cast<uint16_t>(offsetTopEdge + x),
                        static_cast<uint16_t>(offsetTop + x + 1),
                        static_cast<uint16_t>(offsetTop + x),
                        static_cast<uint16_t>(offsetTopEdge + x),
                        static_cast<uint16_t>(offsetTopEdge + x + 1),
                        static_cast<uint16_t>(offsetTop + x + 1)});
    }

    // Left/right skirt frames (self-contained strips of paired surface/skirt verts)
    const uint16_t offsetLeft = static_cast<uint16_t>(vertices.size() / 4);
    const uint16_t offsetRight = static_cast<uint16_t>(offsetLeft + vps * 2);
    for (int edge = 0; edge <= 1; ++edge) {
        for (size_t y = 0; y < vps; ++y) {
            for (int16_t z = 0; z <= 1; ++z) {
                addVert(static_cast<float>(edge) * extent, y * step, z);
            }
        }
    }
    for (uint16_t y = 0; y < gridSize * 2; y += 2) {
        indices.insert(indices.end(),
                       {static_cast<uint16_t>(offsetLeft + y),
                        static_cast<uint16_t>(offsetLeft + y + 1),
                        static_cast<uint16_t>(offsetLeft + y + 3),
                        static_cast<uint16_t>(offsetLeft + y),
                        static_cast<uint16_t>(offsetLeft + y + 3),
                        static_cast<uint16_t>(offsetLeft + y + 2),
                        static_cast<uint16_t>(offsetRight + y),
                        static_cast<uint16_t>(offsetRight + y + 3),
                        static_cast<uint16_t>(offsetRight + y + 1),
                        static_cast<uint16_t>(offsetRight + y),
                        static_cast<uint16_t>(offsetRight + y + 2),
                        static_cast<uint16_t>(offsetRight + y + 3)});
    }

    mesh = TerrainMesh{nullptr, // vertexBuffer - created when building the drawable
                       nullptr, // indexBuffer - created when building the drawable
                       vertices.size() / 4,
                       indices.size(),
                       std::move(vertices),
                       std::move(indices)};
}

std::shared_ptr<gfx::Texture2D> RenderTerrain::createDEMTexture(gfx::Context& context, const DEMData& demData) {
    // Get the DEM image data
    const auto& imagePtr = demData.getImagePtr();
    if (!imagePtr || imagePtr->size.isEmpty()) {
        Log::Warning(Event::Render, "DEM data has no image");
        return nullptr;
    }

    // Create a new texture
    auto texture = context.createTexture2D();
    if (!texture) {
        Log::Error(Event::Render, "Failed to create DEM texture");
        return nullptr;
    }

    // Set the image data
    texture->setImage(imagePtr);

    // Nearest filtering: the packed Terrain-RGB/Terrarium DEM cannot be hardware
    // interpolated (blending the encoded bytes does not blend the decoded
    // elevations), so shaders decode each texel and interpolate in meters via
    // get_elevation(). This matches maplibre-gl-js, which binds the DEM NEAREST.
    texture->setSamplerConfiguration({.filter = gfx::TextureFilterType::Nearest,
                                      .wrapU = gfx::TextureWrapType::Clamp,
                                      .wrapV = gfx::TextureWrapType::Clamp});

    return texture;
}

std::unique_ptr<gfx::Drawable> RenderTerrain::createDrawableForTile(gfx::Context& context,
                                                                    gfx::ShaderRegistry& shaders,
                                                                    const OverscaledTileID& tileID,
                                                                    std::shared_ptr<gfx::Texture2D> demTexture,
                                                                    std::shared_ptr<gfx::Texture2D> mapTexture,
                                                                    bool depthPass) {
    // Ensure mesh is generated
    const auto& terrainMesh = getMesh(context);

    if (terrainMesh.vertices.empty() || terrainMesh.indices.empty()) {
        Log::Error(Event::Render, "Terrain mesh is empty, cannot create drawable");
        return nullptr;
    }

    // Get terrain shader
    auto terrainShader = context.getGenericShader(shaders, depthPass ? "TerrainDepthShader" : "TerrainShader");
    if (!terrainShader) {
        // The depth shader is not registered on all backends yet; symbols
        // then sample the far-plane placeholder and stay visible
        if (!depthPass) {
            Log::Error(Event::Render, "Terrain shader not found");
        }
        return nullptr;
    }

    // Create drawable builder
    auto builder = context.createDrawableBuilder(depthPass ? "terrain-depth-tile" : "terrain-tile");
    if (!builder) {
        Log::Error(Event::Render, "Failed to create drawable builder for terrain tile");
        return nullptr;
    }

    // The drape pass uses the Translucent render pass because it renders in
    // forward order (high index = front), unlike Opaque which renders reversed.
    builder->setShader(terrainShader);
    builder->setRenderPass(RenderPass::Translucent);
    if (depthPass) {
        // The depth pass renders packed depth with real depth testing so the
        // nearest surface wins, into the terrain depth target (renderDepth)
        builder->setDepthType(gfx::DepthMaskType::ReadWrite);
        builder->setColorMode(gfx::ColorMode::unblended());
        builder->setEnableDepth(true);
        builder->setIs3D(true);
    } else {
        // The terrain surface is 3D geometry, so it tests and writes depth: nearer
        // terrain occludes farther terrain, and - crucially - occludes the skirt
        // curtains hanging below each tile edge, so the skirts only show through
        // the cracks they exist to fill rather than drawing over the surface.
        //
        // Symbols (and other layers that occlude against terrain via the depth
        // texture) must not main-depth-test against this surface, or they would be
        // culled by the terrain they sit on - the symbol tweaker disables their
        // depth test while terrain is enabled.
        builder->setDepthType(gfx::DepthMaskType::ReadWrite);
        builder->setColorMode(gfx::ColorMode::unblended());
        builder->setEnableDepth(true);
        builder->setIs3D(true);
    }

    // Set vertex data - copy vertices to raw buffer
    std::vector<uint8_t> vertexData(terrainMesh.vertices.size() * sizeof(int16_t));
    std::memcpy(vertexData.data(), terrainMesh.vertices.data(), vertexData.size());
    builder->setRawVertices(std::move(vertexData), terrainMesh.vertexCount, gfx::AttributeDataType::Short4);

    // Set index data and segments
    // Create a single segment covering the entire terrain mesh
    SegmentVector segments;
    segments.emplace_back(0,                       // vertex offset
                          0,                       // index offset
                          terrainMesh.vertexCount, // vertex count
                          terrainMesh.indexCount); // index count

    std::vector<uint16_t> indexData = terrainMesh.indices;
    builder->setSegments(gfx::Triangles(), std::move(indexData), segments.data(), segments.size());

    // Set the DEM texture
    if (demTexture) {
        builder->setTexture(demTexture, 0); // Texture index 0 for DEM
    }

    // The depth pass samples only the DEM and writes packed depth, so it has no
    // map texture by design; only the draped pass binds the drape render target
    if (!depthPass) {
        if (mapTexture) {
            builder->setTexture(mapTexture, 1); // Texture index 1 for map
        } else {
            Log::Warning(Event::Render, "No drape texture for terrain tile " + util::toString(tileID));
        }
    }

    // Flush to create the drawable
    builder->flush(context);

    // Get the drawable
    auto drawables = builder->clearDrawables();
    if (drawables.empty()) {
        Log::Error(Event::Render, "Failed to create terrain drawable for tile");
        return nullptr;
    }

    // Set tile ID on the drawable
    auto& drawable = drawables[0];
    drawable->setTileID(tileID);

    return std::move(drawable);
}

void RenderTerrain::activateLayerGroup(bool activate, UniqueChangeRequestVec& changes) {
    if (layerGroup) {
        if (activate) {
            changes.emplace_back(std::make_unique<AddLayerGroupRequest>(layerGroup));
        } else {
            changes.emplace_back(std::make_unique<RemoveLayerGroupRequest>(layerGroup));
        }
    }
}

} // namespace mbgl
