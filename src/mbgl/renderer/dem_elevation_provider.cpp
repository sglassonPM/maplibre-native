#include <mbgl/renderer/dem_elevation_provider.hpp>

#include <mbgl/geometry/dem_data.hpp>
#include <mbgl/renderer/buckets/hillshade_bucket.hpp>
#include <mbgl/renderer/render_source.hpp>
#include <mbgl/renderer/render_tile.hpp>
#include <mbgl/tile/raster_dem_tile.hpp>

namespace mbgl {

DEMElevationProvider::DEMElevationProvider(const RenderSource* demSource_, double exaggeration_)
    : demSource(demSource_),
      exaggeration(exaggeration_) {
    if (!demSource) {
        return;
    }
    // Isomaps ⏱ : index construit UNE fois par instance (= par passe de cover) — voir le header.
    // Toujours TOUTES les tuiles CHARGÉES (rendues + retenues hors frustum, ex. aperçu z7/z10
    // anti-collision), pas seulement les rendues : sinon une zone sans tuile DEM RENDUE n'a aucune plage
    // d'altitude → son AABB de frustum reste PLAT au niveau de la mer → à fort pitch près d'un sommet la
    // tuile est jugée hors champ → jamais demandée → jamais élevée (trou auto-entretenu).
    const auto considerIndex = [&](const CanonicalTileID& candidate, const Tile& tile) {
        if (tile.kind != Tile::Kind::RasterDEM) {
            return;
        }
        const auto* demTile = static_cast<const RasterDEMTile*>(&tile);
        const auto* bucket = const_cast<RasterDEMTile*>(demTile)->getBucket();
        if (!bucket) {
            return;
        }
        index.emplace(candidate, &bucket->getDEMData()); // doublons overscalés : premier gagnant, même DEM
    };
    if (const auto* loaded = demSource->getLoadedTiles()) {
        for (const auto& [oid, tilePtr] : *loaded) {
            if (tilePtr) {
                considerIndex(oid.canonical, *tilePtr);
            }
        }
    } else {
        const auto renderTiles = demSource->getRawRenderTiles();
        for (const auto& renderTile : *renderTiles) {
            considerIndex(renderTile.id.canonical, renderTile.getTile());
        }
    }
}

std::optional<Range<double>> DEMElevationProvider::getTileElevationRange(const CanonicalTileID& id) const {
    return getTileElevationRange(id, nullptr);
}

std::optional<Range<double>> DEMElevationProvider::getTileElevationRange(const CanonicalTileID& id,
                                                                         uint8_t* outSourceZ) const {
    if (!demSource) {
        return std::nullopt;
    }

    // The tile's own DEM, or failing that the deepest loaded ancestor: an ancestor's
    // range covers this tile's area, so it stays conservative, just looser.
    // Isomaps ⏱ : remontée de chaîne parentale sur l'index (≤ z lookups) — réponse IDENTIQUE à l'ancien
    // balayage de toutes les tuiles chargées (le premier trouvé en descendant est le plus profond).
    const DEMData* best = nullptr;
    uint8_t bestZoom = 0;
    for (int zz = static_cast<int>(id.z); zz >= 0 && !best; --zz) {
        const auto it = index.find(id.scaledTo(static_cast<uint8_t>(zz)));
        if (it != index.end()) {
            best = it->second;
            bestZoom = static_cast<uint8_t>(zz);
        }
    }

    if (!best) {
        return std::nullopt;
    }

    if (outSourceZ) {
        *outSourceZ = bestZoom;
    }
    // Exaggeration is applied to the mesh in the terrain vertex shader, so the bounds
    // have to carry it too, or an exaggerated peak would still be culled.
    return Range<double>{best->getMinElevation() * exaggeration, best->getMaxElevation() * exaggeration};
}

StableElevationProvider::StableElevationProvider(const RenderSource* demSource_,
                                                 double exaggeration_,
                                                 std::map<CanonicalTileID, Range<double>>& cache_,
                                                 std::set<CanonicalTileID>& finalized_)
    : inner(demSource_, exaggeration_),
      cache(cache_),
      finalized(finalized_) {}

std::optional<Range<double>> StableElevationProvider::getTileElevationRange(const CanonicalTileID& id) const {
    // NB Isomaps : le « cache-first » perf (répondre du cache sans interroger l'inner) a été RÉVERTÉ —
    // même restreint aux plages « définitives », il a coïncidé avec un gel du raffinement (cover convergé
    // à ~6 tuiles → quiescence sur scène incomplète). Retour au comportement éprouvé : inner à chaque
    // requête, union monotone, cache en secours. La perf sera re-tentée isolément, mesurée.
    const auto fresh = inner.getTileElevationRange(id);
    const auto it = cache.find(id);
    if (!fresh) {
        // DEM pas (encore) charge : garder la derniere plage connue plutot que « inconnu »,
        // pour que l'AABB ne s'effondre pas entre deux chargements.
        return it != cache.end() ? std::optional<Range<double>>(it->second) : std::nullopt;
    }
    Range<double> merged = *fresh;
    if (it != cache.end()) {
        merged.min = std::min(merged.min, it->second.min);
        merged.max = std::max(merged.max, it->second.max);
    }
    cache.insert_or_assign(id, merged); // on CACHE la valeur nue (sinon la marge s'accumulerait)

    // Marge de securite sur la valeur RETOURNEE : une boite englobante un peu plus haute et plus
    // basse est plus surement jugee « visible » par le frustum. Sans elle, une tuile de BORD BAS
    // (relief proche penchant vers la camera) dont la boite passe sous le plan bas du frustum est
    // rejetee alors qu'un bout est encore a l'ecran -> elle disparait quand on pan vers le bas
    // (« masquee trop tot »). Appliquee au provider PARTAGE terrain+sources, donc les deux gardent
    // la meme tuile de bord : elle reste maillee ET drapee (pas de trou gris).
    // Isomaps : 150 m (était 800). À pitch 0 le frustum s'élargit en profondeur (jusqu'au niveau de la
    // mer) : une marge de ±800 m faisait passer des tuiles à ~1,5 km hors champ → tapis z18 de 200 tuiles
    // au repos (mesuré) pour un écran qui en montre ~12. 150 m couvre toujours le jeu DEM/exagération.
    constexpr double kElevationMarginMeters = 150.0;
    return Range<double>{merged.min - kElevationMarginMeters, merged.max + kElevationMarginMeters};
}

} // namespace mbgl
