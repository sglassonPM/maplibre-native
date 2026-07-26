#pragma once

#include <mbgl/tile/tile_id.hpp>
#include <mbgl/util/range.hpp>
#include <mbgl/util/tile_cover.hpp>

#include <map>
#include <optional>

namespace mbgl {

class RenderSource;

/// Answers "how high is the terrain here?" for util::tileCover, from the DEM source's
/// currently loaded tiles, so that sources are asked for the tiles the terrain mesh
/// actually needs (see util::TileElevationProvider).
///
/// A tile's own DEM is used when loaded, otherwise the nearest loaded ancestor's range,
/// which contains it. The answer only has to be conservative: too large an elevation
/// range loads a tile that turns out not to be visible, too small drops one that is.
class DEMElevationProvider final : public util::TileElevationProvider {
public:
    /// `demSource` may be null (no terrain, or its source not yet resolved), in which
    /// case every query reports unknown and the cover stays flat.
    explicit DEMElevationProvider(const RenderSource* demSource, double exaggeration);

    std::optional<Range<double>> getTileElevationRange(const CanonicalTileID&) const override;

private:
    const RenderSource* demSource;
    double exaggeration;
};

/// DEMElevationProvider dont chaque reponse est fondue (union monotone) avec la plus large deja
/// vue pour la meme tuile, dans un cache EXTERNE partage. Deux raisons, deux consommateurs :
///  - stabilite : la plage d'altitude ne retrecit plus pendant le (re)chargement du DEM, donc
///    l'AABB d'une tuile ne s'effondre pas et une tuile visible ne clignote pas hors du cover ;
///  - synchronisation terrain/sources : le maillage terrain (RenderTerrain) et le cover des
///    sources (RenderOrchestrator) lisent le DEM a des instants differents de la frame. En
///    partageant le MEME cache, les sources demandent au moins ce que le terrain maille, donc
///    chaque tuile terrain a une tuile satellite a draper (plus de trous magenta au near-bottom).
/// L'union ne peut que grandir : sur-couverture au pire, jamais de trou. Bornee par l'altitude
/// reelle du terrain, donc pas d'emballement. Le cache appartient a RenderTerrain (une carte),
/// qui le vide s'il grossit trop.
class StableElevationProvider final : public util::TileElevationProvider {
public:
    StableElevationProvider(const RenderSource* demSource,
                            double exaggeration,
                            std::map<CanonicalTileID, Range<double>>& cache);

    std::optional<Range<double>> getTileElevationRange(const CanonicalTileID&) const override;

private:
    DEMElevationProvider inner;
    std::map<CanonicalTileID, Range<double>>& cache;
};

} // namespace mbgl
