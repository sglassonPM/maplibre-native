#include <mbgl/renderer/dem_elevation_provider.hpp>

#include <mbgl/geometry/dem_data.hpp>
#include <mbgl/renderer/buckets/hillshade_bucket.hpp>
#include <mbgl/renderer/render_source.hpp>
#include <mbgl/renderer/render_tile.hpp>
#include <mbgl/tile/raster_dem_tile.hpp>

namespace mbgl {

DEMElevationProvider::DEMElevationProvider(const RenderSource* demSource_, double exaggeration_)
    : demSource(demSource_),
      exaggeration(exaggeration_) {}

std::optional<Range<double>> DEMElevationProvider::getTileElevationRange(const CanonicalTileID& id) const {
    if (!demSource) {
        return std::nullopt;
    }

    // The tile's own DEM, or failing that the deepest loaded ancestor: an ancestor's
    // range covers this tile's area, so it stays conservative, just looser.
    const DEMData* best = nullptr;
    uint8_t bestZoom = 0;
    const auto consider = [&](const CanonicalTileID& candidate, const Tile& tile) -> bool {
        if (tile.kind != Tile::Kind::RasterDEM) {
            return false;
        }
        const bool covers = candidate == id || id.isChildOf(candidate);
        if (!covers || (best && candidate.z <= bestZoom)) {
            return false;
        }
        const auto* demTile = static_cast<const RasterDEMTile*>(&tile);
        const auto* bucket = const_cast<RasterDEMTile*>(demTile)->getBucket();
        if (!bucket) {
            return false;
        }
        best = &bucket->getDEMData();
        bestZoom = candidate.z;
        return candidate == id; // exact match; nothing looser can improve on it
    };

    // Isomaps : parcourir TOUTES les tuiles CHARGÉES (rendues + retenues hors frustum, ex. aperçu z7/z10
    // anti-collision), pas seulement les rendues. Sinon une zone sans tuile DEM RENDUE n'a aucune plage
    // d'altitude → son AABB de frustum reste PLAT au niveau de la mer → à fort pitch près d'un sommet la
    // tuile est jugée hors champ → jamais demandée → jamais élevée (trou auto-entretenu : faces de sommet
    // « transparentes », fond visible). L'aperçu grossier retenu comble exactement ce vide.
    if (const auto* loaded = demSource->getLoadedTiles()) {
        for (const auto& [oid, tilePtr] : *loaded) {
            if (tilePtr && consider(oid.canonical, *tilePtr)) {
                break;
            }
        }
    } else {
        const auto renderTiles = demSource->getRawRenderTiles();
        for (const auto& renderTile : *renderTiles) {
            if (consider(renderTile.id.canonical, renderTile.getTile())) {
                break;
            }
        }
    }

    if (!best) {
        return std::nullopt;
    }

    // Exaggeration is applied to the mesh in the terrain vertex shader, so the bounds
    // have to carry it too, or an exaggerated peak would still be culled.
    return Range<double>{best->getMinElevation() * exaggeration, best->getMaxElevation() * exaggeration};
}

StableElevationProvider::StableElevationProvider(const RenderSource* demSource_,
                                                 double exaggeration_,
                                                 std::map<CanonicalTileID, Range<double>>& cache_)
    : inner(demSource_, exaggeration_),
      cache(cache_) {}

std::optional<Range<double>> StableElevationProvider::getTileElevationRange(const CanonicalTileID& id) const {
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
