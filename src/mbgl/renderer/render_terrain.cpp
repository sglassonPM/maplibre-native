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

// 🧪 Isomaps DIAG (à retirer) — compteurs de tuiles exposés au HUD du harness via accesseurs C.
namespace {
std::atomic<int> g_isomapsMeshTiles{0};
std::atomic<int> g_isomapsSatTiles{0};
} // namespace
extern "C" int isomapsDebugMeshTileCount() { return g_isomapsMeshTiles.load(std::memory_order_relaxed); }
extern "C" int isomapsDebugSatTileCount() { return g_isomapsSatTiles.load(std::memory_order_relaxed); }

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
            StableElevationProvider elevationProvider(demSource, getExaggeration(), meshTileElevation);
            // Meme seuil variable-zoom que les sources (abaisse a 0 dans map_impl) et cover borne.
            const util::TileCoverParameters coverParams{
                .transformState = state,
                .tileLodMinRadius = updateParameters.tileLodMinRadius,
                .tileLodScale = updateParameters.tileLodScale,
                .tileLodPitchThreshold = updateParameters.tileLodPitchThreshold,
                .tileLodMode = updateParameters.tileLodMode,
                .elevationProvider = &elevationProvider};
            const auto cover = util::tileCover(coverParams, baseZoom, Range<uint8_t>(minLoadedZoom, baseZoom));
            for (const auto& oid : cover) {
                out.insert(UnwrappedTileID(oid.wrap, oid.canonical));
            }
            // Garde-fou : au-dela d'un plafond genereux, on tronque aux tuiles les plus proches
            // du centre (les plus grandes a l'ecran) plutot que de risquer l'OOM.
            constexpr size_t maxMeshTiles = 700; // Isomaps : plafond ferme. 5000 laissait le dézoom incliné
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
    }

    // Isomaps direct-raster : RAFFINER le maillage là où le SATELLITE a chargé plus fin que le DEM.
    // Le DEM plafonne le maillage à son propre zoom (ex. z14 uniforme), mais le satellite descend plus
    // profond au premier plan (z15+, LOD variable). Résultat sans ceci : la tuile maillage z14 n'a pas
    // de tuile satellite z14 exacte → elle retombe sur un ancêtre grossier → flou (≈30% de match exact
    // mesuré). On ajoute les ids des tuiles satellite chargées puis expandToDeepestCover subdivise
    // chaque tuile DEM grossière recouverte par des tuiles satellite plus profondes → le maillage
    // ÉPOUSE le LOD du satellite au premier plan → match exact (piqué max), tandis que le DEM reste le
    // filet frustum (aucun trou) et domine au loin. Le DEM est échantillonné en ancêtre par tuile.
    if (basemapSource && demBaseZoom > 0) {
        // BORNÉ : on n'ajoute QUE les tuiles satellite plus fines que le DEM et au plus +1 niveau.
        // Sans borne, on avalait les tuiles ancêtres (placeholders z0..z13 que la source garde) →
        // expandToDeepestCover fabriquait un escalier de tuiles grossières de z1 à z15 (n≈400,
        // ~300 gâchées) → lag. Ici : seules les tuiles z=demBaseZoom+1 raffinent le premier plan
        // (×4 local, borné), le DEM domine partout ailleurs → net + fluide.
        const auto satTiles = basemapSource->getRawRenderTiles();
        for (const auto& rt : *satTiles) {
            const uint8_t z = rt.id.canonical.z;
            if (z > demBaseZoom && z <= demBaseZoom + 1) {
                out.insert(rt.id);
            }
        }
        out = expandToDeepestCover(out);
    }

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
    constexpr uint64_t HYSTERESIS_FRAMES = 60;
    for (const auto& id : out) {
        meshTileLastVisible[id] = demUpdateCounter;
    }
    for (auto it = meshTileLastVisible.begin(); it != meshTileLastVisible.end();) {
        if (demUpdateCounter - it->second > HYSTERESIS_FRAMES) {
            it = meshTileLastVisible.erase(it);
        } else {
            out.insert(it->first);
            ++it;
        }
    }

    // Borne le cache d'altitude (l'union monotone accumule chaque tuile visitee par le DFS au
    // fil des pans) : au-dela d'un plafond genereux, on repart de zero — il se reremplit en
    // quelques frames et l'over-coverage transitoire est invisible.
    constexpr size_t maxElevationCache = 8192;
    if (meshTileElevation.size() > maxElevationCache) {
        meshTileElevation.clear();
    }

    // Isomaps : PLAFOND FINAL (crash-safety) — appliqué APRÈS l'hystérésis, il borne le total QUOI QU'IL
    // ARRIVE. Le 700 plus haut ne cape que le cover instantané ; l'hystérésis empile ensuite les tuiles
    // vues sur plusieurs frames pendant un geste (mesuré jusqu'à ~2700 → OOM/crash). On garde les tuiles
    // les plus proches du CENTRE écran (ce qu'on regarde) ; le lointain/hors-champ largué est de toute
    // façon noyé dans la brume. NB : ne borne pas assez pour égaler Mapbox (voir découplage drapage) —
    // ici c'est un filet anti-crash, pas l'optimisation finale.
    constexpr size_t kMaxFinalMeshTiles = 900;
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

    // Get tiles from the DEM source
    auto renderTiles = demSource->getRawRenderTiles();
    if (renderTiles->empty()) {
        return;
    }

    // Cast to LayerGroup for addDrawable
    auto* lg = static_cast<LayerGroup*>(layerGroup.get());
    if (!lg) {
        return;
    }

    // Decode and cache the DEM textures of loaded DEM tiles
    ++demUpdateCounter;
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

    // 🧪 DIAG (à retirer) — aligne-t-on maillage et satellite ? Si zoom max maillage >> zoom max
    // satellite chargé, le premier plan sample un ancêtre grossier = flou.
    if (basemapSource && (demUpdateCounter % 30 == 0)) {
        std::array<int, 24> meshZ{}, satZ{}; // histogrammes par zoom
        for (const auto& id : meshTiles) {
            const int z = static_cast<int>(id.canonical.z);
            if (z >= 0 && z < 24) ++meshZ[z];
        }
        for (const auto& [id, tex] : basemapTextures) {
            const int z = static_cast<int>(id.canonical.z);
            if (z >= 0 && z < 24) ++satZ[z];
        }
        const auto fmt = [](const std::array<int, 24>& a) {
            std::string s;
            for (int z = 23; z >= 0; --z)
                if (a[z] > 0) s += " z" + util::toString(z) + ":" + util::toString(a[z]);
            return s;
        };
        Log::Warning(Event::Render,
                     "🧪 DIAG MAILLAGE n=" + util::toString(meshTiles.size()) + " |" + fmt(meshZ) +
                         "  ||  SATELLITE chargé n=" + util::toString(basemapTextures.size()) + " |" + fmt(satZ));
    }

    // Drop drawables and cached DEM textures for tiles that left the mesh tile
    // set, keeping everything else intact between frames
    std::unordered_set<OverscaledTileID> currentTiles;
    for (const auto& id : meshTiles) {
        currentTiles.emplace(id.canonical.z, id.wrap, id.canonical);
    }
    lg->removeDrawablesIf(
        [&](gfx::Drawable& drawable) { return drawable.getTileID() && !currentTiles.contains(*drawable.getTileID()); });
    auto* depthLg = static_cast<LayerGroup*>(depthLayerGroup.get());
    if (depthLg) {
        depthLg->removeDrawablesIf([&](gfx::Drawable& drawable) {
            return drawable.getTileID() && !currentTiles.contains(*drawable.getTileID());
        });
    }
    for (auto it = tilesWithDrawables.begin(); it != tilesWithDrawables.end();) {
        if (!currentTiles.contains(it->first)) {
            drawableDemCoords.erase(it->first);
            drawableMapCoords.erase(it->first);
            it = tilesWithDrawables.erase(it);
        } else {
            ++it;
        }
    }
    // Retain cached DEM textures that are related to the current tile set so they
    // can serve as ancestor fallbacks while exact tiles load (as maplibre-gl-js
    // retains terrain tiles in its source cache); drop unrelated ones
    for (auto it = demTextures.begin(); it != demTextures.end();) {
        bool related = false;
        for (const auto& current : currentTiles) {
            const UnwrappedTileID unwrapped = current.toUnwrapped();
            if (unwrapped == it->first || unwrapped.isChildOf(it->first) || it->first.isChildOf(unwrapped)) {
                related = true;
                break;
            }
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
        if (demTextures.contains(id)) {
            meshTileDemZoom[id] = static_cast<int>(id.canonical.z);
            continue;
        }
        int best = -1;
        for (const auto& [cand, entry] : demTextures) {
            if (id.isChildOf(cand) && static_cast<int>(cand.canonical.z) > best) {
                best = static_cast<int>(cand.canonical.z);
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
                        if (nb->second >= 0 && self->second > nb->second) {
                            dz[e] = static_cast<float>(self->second - nb->second);
                        }
                        break;
                    }
                }
            }
        }
        drawableEdgeDz[OverscaledTileID(id.canonical.z, id.wrap, id.canonical)] = dz;
    }

    // Create terrain drawables for each mesh tile
    for (const auto& unwrapped : meshTiles) {
        const OverscaledTileID tileID(unwrapped.canonical.z, unwrapped.wrap, unwrapped.canonical);

        // Skip if the tile already has a drawable bound to its own DEM
        if (const auto existing = tilesWithDrawables.find(tileID);
            existing != tilesWithDrawables.end() && existing->second == 2) {
            continue;
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
            // Fall back to the closest cached ancestor DEM
            const UnwrappedTileID* ancestorID = nullptr;
            DEMTextureEntry* ancestorEntry = nullptr;
            int bestZoom = -1;
            for (auto& [candidate, entry] : demTextures) {
                if (candidate != unwrapped && unwrapped.isChildOf(candidate) &&
                    static_cast<int>(candidate.canonical.z) > bestZoom) {
                    bestZoom = candidate.canonical.z;
                    ancestorID = &candidate;
                    ancestorEntry = &entry;
                    demTexture = entry.texture;
                }
            }
            if (!demTexture) {
                // No DEM at all yet: render the mesh flat with the placeholder
                // DEM so the draped map still shows (a briefly flat area is
                // less jarring than a hole in the terrain)
                demTexture = getPlaceholderDEMTexture(context);
                if (!demTexture) {
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

        // Isomaps : texture raster basemap (échantillonnage direct) + transform UV. On la calcule ICI,
        // avant le check de réutilisation, pour pouvoir recréer le drawable quand une tuile raster PLUS
        // FINE devient dispo (map_coords[0] : 1.0 = exacte, <1 = ancêtre ; plus grand = plus fin).
        std::shared_ptr<gfx::Texture2D> mapTexture;
        std::array<float, 4> mapCoords{{1.0f, 0.0f, 0.0f, 0.0f}};
        if (basemapSource) {
            mapTexture = getBasemapTextureForTile(unwrapped, mapCoords);
            if (!mapTexture) {
                continue; // ni exacte ni ancêtre chargé (rare, transitoire tout au début)
            }
        }

        // If a drawable already exists for this tile, keep it until a higher DEM quality tier — OR a
        // finer raster basemap tile (Isomaps) — becomes available, then replace it.
        if (const auto existing = tilesWithDrawables.find(tileID); existing != tilesWithDrawables.end()) {
            const auto exMap = drawableMapCoords.find(tileID);
            const float existingMapScale = exMap != drawableMapCoords.end() ? exMap->second[0] : 1.0f;
            if (existing->second >= demTier && (!basemapSource || existingMapScale >= mapCoords[0])) {
                continue;
            }
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
            lg->addDrawable(std::move(drawable));
            tilesWithDrawables[tileID] = demTier;
            if (depthLg) {
                if (auto depthDrawable = createDrawableForTile(
                        context, shaders, tileID, demTexture, nullptr, /*depthPass=*/true)) {
                    depthLg->addDrawable(std::move(depthDrawable));
                }
            }
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
    return top + (bottom - top) * fy;
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
