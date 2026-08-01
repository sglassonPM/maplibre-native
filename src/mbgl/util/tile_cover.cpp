#include <mbgl/math/angles.hpp>
#include <mbgl/math/log2.hpp>
#include <mbgl/util/bounding_volumes.hpp>
#include <mbgl/util/constants.hpp>
#include <mbgl/util/interpolate.hpp>
#include <mbgl/util/tile_coordinate.hpp>
#include <mbgl/util/tile_cover.hpp>
#include <mbgl/util/tile_cover_impl.hpp>

#include <cmath>
#include <functional>
#include <list>

using namespace std::numbers;

namespace mbgl {

namespace {

using ScanLine = const std::function<void(int32_t x0, int32_t x1, int32_t y)>;

// Taken from polymaps src/Layer.js
// https://github.com/simplegeo/polymaps/blob/master/src/Layer.js#L333-L383
struct edge {
    double x0 = 0, y0 = 0;
    double x1 = 0, y1 = 0;
    double dx = 0, dy = 0;

    edge(Point<double> a, Point<double> b) {
        if (a.y > b.y) std::swap(a, b);
        x0 = a.x;
        y0 = a.y;
        x1 = b.x;
        y1 = b.y;
        dx = b.x - a.x;
        dy = b.y - a.y;
    }
};

// scan-line conversion
void scanSpans(edge e0, edge e1, int32_t ymin, int32_t ymax, ScanLine& scanLine) {
    const double y0 = ::fmax(ymin, std::floor(e1.y0));
    const double y1 = ::fmin(ymax, std::ceil(e1.y1));

    // sort edges by x-coordinate
    if ((e0.x0 == e1.x0 && e0.y0 == e1.y0) ? (e0.x0 + e1.dy / e0.dy * e0.dx < e1.x1)
                                           : (e0.x1 - e1.dy / e0.dy * e0.dx < e1.x0)) {
        std::swap(e0, e1);
    }

    // scan lines!
    const double m0 = e0.dx / e0.dy;
    const double m1 = e1.dx / e1.dy;
    const double d0 = e0.dx > 0;       // use y + 1 to compute x0
    const double d1 = e1.dx < 0;       // use y + 1 to compute x1
    for (double y = y0; y < y1; y++) { // NOLINT(clang-analyzer-security.FloatLoopCounter)
        double x0 = m0 * ::fmax(0, ::fmin(e0.dy, y + d0 - e0.y0)) + e0.x0;
        double x1 = m1 * ::fmax(0, ::fmin(e1.dy, y + d1 - e1.y0)) + e1.x0;
        scanLine(static_cast<int32_t>(std::floor(x1)), static_cast<int32_t>(std::ceil(x0)), static_cast<int32_t>(y));
    }
}

// scan-line conversion
void scanTriangle(const Point<double>& a,
                  const Point<double>& b,
                  const Point<double>& c,
                  int32_t ymin,
                  int32_t ymax,
                  ScanLine& scanLine) {
    edge ab = edge(a, b);
    edge bc = edge(b, c);
    edge ca = edge(c, a);

    // sort edges by y-length
    if (ab.dy > bc.dy) {
        std::swap(ab, bc);
    }
    if (ab.dy > ca.dy) {
        std::swap(ab, ca);
    }
    if (bc.dy > ca.dy) {
        std::swap(bc, ca);
    }

    // scan span! scan span!
    if (ab.dy) scanSpans(ca, ab, ymin, ymax, scanLine);
    if (bc.dy) scanSpans(ca, bc, ymin, ymax, scanLine);
}

} // namespace

namespace util {

namespace {

std::vector<UnwrappedTileID> tileCover(const Point<double>& tl,
                                       const Point<double>& tr,
                                       const Point<double>& br,
                                       const Point<double>& bl,
                                       const Point<double>& c,
                                       uint8_t z) {
    const int32_t tiles = 1 << z;

    struct ID {
        int32_t x, y;
        double sqDist;
    };

    std::vector<ID> t;

    // skip the first few allocations, assuming we usually end up with at least a few tiles
    t.reserve(8);

    auto scanLine = [&](int32_t x0, int32_t x1, int32_t y) {
        int32_t x;
        if (y >= 0 && y <= tiles) {
            for (x = x0; x < x1; ++x) {
                const auto dx = x + 0.5 - c.x;
                const auto dy = y + 0.5 - c.y;
                t.emplace_back(ID{.x = x, .y = y, .sqDist = dx * dx + dy * dy});
            }
        }
    };

    // Divide the screen up in two triangles and scan each of them:
    // \---+
    // | \ |
    // +---\.
    scanTriangle(tl, tr, br, 0, tiles, scanLine);
    scanTriangle(br, bl, tl, 0, tiles, scanLine);

    // Sort first by distance, then by x/y.
    std::sort(t.begin(), t.end(), [](const ID& a, const ID& b) noexcept {
        return std::tie(a.sqDist, a.x, a.y) < std::tie(b.sqDist, b.x, b.y);
    });

    // Erase duplicate tile IDs (they typically occur at the common side of both triangles).
    t.erase(std::unique(t.begin(), t.end(), [](const ID& a, const ID& b) { return a.x == b.x && a.y == b.y; }),
            t.end());

    std::vector<UnwrappedTileID> result;
    result.reserve(t.size());
    for (const auto& id : t) {
        result.emplace_back(z, id.x, id.y);
    }
    return result;
}

} // namespace

int32_t coveringZoomLevel(double zoom, style::SourceType type, uint16_t size) noexcept {
    zoom += util::log2(util::tileSize_D / size);
    if (type == style::SourceType::Raster || type == style::SourceType::Video) {
        return static_cast<int32_t>(std::round(zoom));
    } else {
        return static_cast<int32_t>(std::floor(zoom));
    }
}

std::vector<OverscaledTileID> tileCover(const TileCoverParameters& state,
                                        uint8_t z,
                                        const Range<uint8_t> zoomRange,
                                        const std::optional<uint8_t>& overscaledZ) {
    struct Node {
        AABB aabb;
        uint8_t zoom;
        uint32_t x, y;
        int16_t wrap;
        bool fullyVisible;
    };

    struct ResultTile {
        OverscaledTileID id;
        double sqrDist;
    };

    auto childrenOf = [](const Node& node) -> std::vector<Node> {
        std::vector<Node> children(4);
        for (int i = 0; i < 4; i++) {
            const uint32_t childX = (node.x << 1) + (i % 2);
            const uint32_t childY = (node.y << 1) + (i >> 1);

            children[i] = node;
            children[i].aabb = node.aabb.quadrant(i);
            children[i].zoom = node.zoom + 1;
            children[i].x = childX;
            children[i].y = childY;
        }
        return children;
    };

    const auto& transform = state.transformState;
    const double numTiles = std::pow(2.0, z);
    const double worldSize = Projection::worldSize(transform.getScale());
    // Isomaps : avec du TERRAIN, LOD variable TOUJOURS actif (même à pitch bas) → la finesse est gouvernée
    // par la DISTANCE (mesurée en AGL, cf. plus bas), façon Mapbox, et non par le zoom carte (relatif à la
    // mer → flou en montagne). Sans terrain (2D) : inchangé (seuil de pitch).
    const bool allowVariableZoom =
        transform.getPitch() > state.tileLodPitchThreshold || state.elevationProvider != nullptr;
    const uint8_t minZoom = allowVariableZoom ? zoomRange.min : z;
    uint8_t maxZoom = ((state.tileLodMode == TileLodMode::Distance) && allowVariableZoom) ? zoomRange.max : z;
    // Isomaps : PLAFOND DE DRAPAGE. Le LOD AGL par-tuile peut pousser des sommets proches de l'œil bien
    // au-delà du zoom nominal (au dézoom : z17 demandé à idealZoom 13 → ~200 tuiles satellite, 2,5 Go),
    // alors que le maillage terrain — borné par son baseZoom (≈ z+2) — ne drape qu'UNE texture par tuile de
    // maillage : plus fin est inutilisable. Borne : jamais plus fin que z+2 (= ceil(zoom)+1 ; préserve le
    // z18 net validé à zoom 17). Masqué jusqu'ici par le frustum écrasé en Z qui culled ces tuiles à tort.
    if (state.elevationProvider && maxZoom > z + 2) {
        maxZoom = static_cast<uint8_t>(z + 2);
    }
    // Isomaps : RÉFÉRENCE SOL STABLE pour le LOD terrain = altitude du terrain SOUS LA CAMÉRA (une seule
    // valeur). Elle sert à mesurer les distances tuile→caméra en AGL (hauteur au-dessus du SOL, cf. bloc LOD
    // plus bas) au lieu d'ASL (au-dessus de la mer). Ainsi le zoom suit la hauteur au-dessus du sol → net à
    // toute altitude (façon Mapbox), avec la formule STANDARD, sans plafond ad hoc. UNE valeur (pas par tuile)
    // et mise en cache par le StableElevationProvider → jamais nulle après le 1er chargement → pas d'oscillation.
    double cameraGroundM = 0.0;
    if (state.elevationProvider) {
        if (const auto loc = transform.getFreeCameraOptions().getLocation()) {
            const auto tc = TileCoordinate::fromLatLng(static_cast<double>(z), loc->location).p;
            const double side = static_cast<double>(1u << z);
            if (tc.x >= 0.0 && tc.y >= 0.0 && tc.x < side && tc.y < side) {
                if (const auto r = state.elevationProvider->getTileElevationRange(
                        CanonicalTileID(z, static_cast<uint32_t>(tc.x), static_cast<uint32_t>(tc.y)))) {
                    cameraGroundM = 0.5 * (r->min + r->max); // milieu → insensible à la marge ±800 m du provider
                }
            }
        }
    }
    const uint8_t overscaledZoom = std::max(overscaledZ.value_or(z), maxZoom);
    const bool flippedY = transform.getViewportMode() == ViewportMode::FlippedY;

    const auto centerPoint = TileCoordinate::fromScreenCoordinate(
                                 transform, z, {transform.getSize().width / 2.0, transform.getSize().height / 2.0})
                                 .p;

    // Elevation has to reach the frustum in the aabb's units (tiles at zoom z).
    // Renderable heights are in meters and Camera::getWorldToCamera scales them by
    // pixelsPerMeter = worldSize / (cos(lat) * 2pi * R); pixels are then tiles at
    // zoom z scaled by numTiles / worldSize, so worldSize cancels out.
    const double metersToTileUnits = numTiles / (std::cos(util::deg2rad(transform.getLatLng().latitude())) *
                                                 util::M2PI * util::EARTH_RADIUS_M);

    // Isomaps : le CENTRE du LOD est élevé à centerAltitude (≈ terrain au centre via la renormalisation),
    // plus au niveau de la MER. Sinon le numérateur de la formule (camToCenterDistance, ASL) restait gonflé
    // en montagne pendant que le dénominateur (distance tuile) est en AGL → DOUBLE boost avec le zoom
    // AGL-vrai → sur-division ×4-16 (≈200 tuiles à 150 m/sol au lieu de ~15) → mémoire/chargement explosés.
    const vec3 centerCoord = {{centerPoint.x, centerPoint.y, transform.getCenterAltitude() * metersToTileUnits}};

    assert(transform.getFreeCameraOptions().position);
    const vec3 cameraPositionMercator = *transform.getFreeCameraOptions().position;
    const double nominalScale = std::pow(2.0, z);
    const vec3 cameraCoord = vec3Scale(cameraPositionMercator, nominalScale);
    const double cameraToCenterDistanceMercator = vec3Length(vec3Sub(cameraCoord, centerCoord)) / worldSize;

    // Isomaps : pixelsPerMeter (même formule que Camera::getWorldToCamera) pour que le Z du frustum soit
    // dans les mêmes unités-tuile que les AABB élevées — cf. Frustum::fromInvProjMatrix.
    const double pixelsPerMeter = metersToTileUnits * worldSize / numTiles;
    const Frustum frustum =
        Frustum::fromInvProjMatrix(transform.getInvProjectionMatrix(), worldSize, z, flippedY, pixelsPerMeter);

    // Isomaps : référence sol stable (terrain sous la caméra) en unités-tuile — sert au LOD AGL ci-dessous.
    const double cameraGroundZ = cameraGroundM * metersToTileUnits;

    // Isomaps 🌍 : PLAFOND D'HORIZON (terrain seulement). Le plan Mercator est infini : à fort pitch le
    // frustum admet ~2 000 km de terrain (34 tuiles z7 mesurées) qui, sur une Terre courbe, sont SOUS
    // l'horizon — invisibles en vrai, mais chaque tuile coûte sa texture satellite (~4 Mo) → 3,4 Go →
    // jetsam (crash mesuré). Portée optique réelle : d ≈ 3,57·(√h_œil + √h_cible) km ; cible plafonnée à
    // 5 000 m (sommets alpins). Tout nœud entièrement au-delà est coupé (sous-arbre compris). Le plafond
    // croît avec l'altitude de l'œil → au dézoom il cesse naturellement d'agir.
    const double eyeASLMeters = std::max(0.0, cameraCoord[2] / metersToTileUnits);
    const double horizonCapTiles = 3570.0 * (std::sqrt(eyeASLMeters) + 71.0) * metersToTileUnits;

    // The tile's bounds including its terrain: the flat footprint given the height of
    // the DEM covering it. Relief rising towards the camera takes up screen space that
    // the flat footprint does not, so a tile can be in view when its footprint is not;
    // testing the flat box is what leaves the terrain mesh with tiles no source ever
    // requested, and nothing to drape onto them.
    //
    // Only the frustum tests use this. The LOD heuristics below keep using the flat
    // box, matching gl-js, whose distanceToTile2d takes only x and y: elevation should
    // decide what is visible, not how finely it is subdivided. It also keeps this
    // change from perturbing tile selection on maps that have no terrain-shaped reason
    // to change.
    // Marge XY (fraction de l'etendue du noeud) ajoutee a la boite AVANT le test de frustum : une
    // tuile dont le corps est hors ecran mais qui deborde d'un mince liseré au bord (typiquement en
    // bas a fort pitch) etait jugee « separee » et larguee -> liseré non rendu. En elargissant la
    // boite de test, une telle tuile est conservee. N'affecte QUE le test de frustum (les heuristiques
    // de LOD utilisent node.aabb) et s'applique a TOUTES les sources : terrain et imagerie gardent
    // donc la meme tuile de bord (maillage ET drapage), pas de desaccord. Seul le test est elargi ;
    // le maillage/placement reste inchange.
    const auto withFrustumMargin = [&](AABB box) -> AABB {
        // Isomaps : 0.35 (était 0.5, puis 0.25). ±50 % sur-admettait (tapis z18:200 à pitch 0) ; 0.25
        // culled la colonne de BORD DE FUITE au pan alors qu'encore partiellement visible (bande bleue à
        // gauche en allant à droite, ligne de tuile nette). 0.35 = compromis mesuré entre les deux.
        constexpr double kEdgeMargin = 0.35;
        const double mx = (box.max[0] - box.min[0]) * kEdgeMargin;
        const double my = (box.max[1] - box.min[1]) * kEdgeMargin;
        box.min[0] -= mx;
        box.max[0] += mx;
        box.min[1] -= my;
        box.max[1] += my;
        return box;
    };
    const auto elevatedAABB = [&](const Node& node) -> AABB {
        if (!state.elevationProvider) {
            // Pas de terrain (carte plate/vectorielle) : couverture IDENTIQUE a l'amont, SANS marge.
            // La marge n'existe que pour le raccord terrain<->drapage au bord bas a fort pitch ; hors
            // terrain elle ne fait qu'elargir le cover a TOUTES les sources (~x1.75 tuiles), ce qui
            // multiplie la geometrie de lignes des styles vectoriels (165 calques) -> OOM sur device
            // (satellite raster epargne). Constate au profilage (LineBucket::addFeature ~1.3 Go).
            return node.aabb;
        }
        const auto range = state.elevationProvider->getTileElevationRange(CanonicalTileID(node.zoom, node.x, node.y));
        if (!range) {
            return withFrustumMargin(node.aabb); // no DEM loaded here yet: flat, as before
        }
        AABB elevated = node.aabb;
        elevated.min[2] = range->min * metersToTileUnits;
        elevated.max[2] = range->max * metersToTileUnits;
        return withFrustumMargin(elevated);
    };

    // There should always be a certain number of maximum zoom level tiles
    // surrounding the center location
    assert(state.tileLodMinRadius >= 1);
    const double radiusOfMaxLvlLodInTiles = std::max(1.0, state.tileLodMinRadius);

    const auto newRootTile = [&](int16_t wrap) -> Node {
        return {.aabb = AABB({{wrap * numTiles, 0.0, 0.0}}, {{(wrap + 1) * numTiles, numTiles, 0.0}}),
                .zoom = uint8_t(0),
                .x = uint16_t(0),
                .y = uint16_t(0),
                .wrap = wrap,
                .fullyVisible = false};
    };

    // Perform depth-first traversal on tile tree to find visible tiles
    std::vector<Node> stack;
    std::vector<ResultTile> result;
    stack.reserve(128);

    // World copies shall be rendered three times on both sides from closest to farthest
    for (int i = 1; i <= 3; i++) {
        stack.push_back(newRootTile(-i));
        stack.push_back(newRootTile(i));
    }

    stack.push_back(newRootTile(0));

    while (!stack.empty()) {
        Node node = stack.back();
        stack.pop_back();

        // Use cached visibility information of ancestor nodes
        if (!node.fullyVisible) {
            // The flat fast path (frustum.intersects) asserts a zero-elevation box, so
            // an elevated node has to take the 3D path. Nodes stay flat, and keep the
            // cheaper test, until a DEM gives them height via elevatedAABB.
            const AABB testAABB = elevatedAABB(node);
            const bool elevated = testAABB.min[2] != 0.0 || testAABB.max[2] != 0.0;
            const IntersectionResult intersection = elevated ? frustum.intersectsElevated(testAABB)
                                                             : frustum.intersects(testAABB);

            if (intersection == IntersectionResult::Separate) {
                continue;
            }

            node.fullyVisible = intersection == IntersectionResult::Contains;
        }

        bool shouldSplitTile;
        if (state.tileLodMode == TileLodMode::Distance) {
            // Isomaps terrain : distance mesurée en AGL (hauteur au-dessus du SOL). On élève la boîte de la
            // tuile à la RÉFÉRENCE SOL STABLE (cameraGroundZ = terrain sous la caméra, UNE valeur → jamais
            // nulle → PAS d'oscillation), au lieu du plan Z=0 (= mer, donc ASL → flou en montagne). Le ratio
            // (cameraToCenterDistance ASL / distance AGL) applique la correction → la formule STANDARD MapLibre
            // donne le bon zoom à toute altitude (façon Mapbox). Sans terrain (2D) : boîte plate, inchangé.
            AABB lodBox = node.aabb;
            if (state.elevationProvider) {
                // Élévation LOCALE de la tuile (pas une réf unique — sinon en vue large les pentes à une
                // altitude ≠ de la caméra sont mal jugées → plaques grossières). UN seul appel : le provider
                // fait DÉJÀ le repli sur l'ancêtre le plus fin parmi TOUTES les tuiles chargées (rendues +
                // aperçu retenu, cf. DEMElevationProvider) et le Stable garde la dernière plage connue →
                // stable (pas de retour au plat) et local. Ultime repli : terrain sous la caméra.
                // Boîte PLEINE [min,max] (pas l'altitude moyenne) : pour une tuile-FALAISE de 2 km de haut,
                // la moyenne mettait la « distance » à mi-paroi (des km) alors que le haut de la face frôle
                // la caméra → premier plan resté grossier pendant que l'arrière-plan (vallée, plus proche du
                // plan moyen de SES tuiles) était net. distanceXYZ vise le point de la boîte le plus proche
                // = la face la plus proche de la paroi. Stable provider → plages monotones, pas d'oscillation.
                if (const auto r = state.elevationProvider->getTileElevationRange(
                        CanonicalTileID(node.zoom, node.x, node.y))) {
                    lodBox.min[2] = r->min * metersToTileUnits;
                    lodBox.max[2] = r->max * metersToTileUnits;
                } else {
                    lodBox.min[2] = cameraGroundZ;
                    lodBox.max[2] = cameraGroundZ;
                }
            }
            const vec3 camToTileTiles = lodBox.distanceXYZ(cameraCoord);
            // Isomaps 🌍 : cull d'horizon (cf. horizonCapTiles) — distance HORIZONTALE au point le plus
            // proche de la boîte ; un nœud entièrement au-delà de la portée optique est abandonné avec
            // tout son sous-arbre. Terrain seulement (2D inchangé).
            if (state.elevationProvider && std::hypot(camToTileTiles[0], camToTileTiles[1]) > horizonCapTiles) {
                continue;
            }
            const vec3 camToTileMercator = vec3Scale(camToTileTiles, 1.0 / worldSize);
            const double distanceToTileMercator = vec3Length(camToTileMercator);
            double cosPitchToTile = std::max(0.0, camToTileMercator[2] / distanceToTileMercator);
            if (state.elevationProvider) {
                // Plancher d'obliquité en 3D : cos(pitchToTile)→0 suppose une tuile PLATE vue en rasant —
                // faux pour une PAROI vue de face (regard horizontal = incidence perpendiculaire au mur).
                // Sans plancher, les faces proches n'étaient jamais subdivisées (écran rempli de flou).
                cosPitchToTile = std::max(cosPitchToTile, 0.35);
            }
            const double pitchExponent =
                0.5; // 0: constant screen width, 1/2: constant screen area, 1: constant screen height
            double tileScale = std::pow(2.0, node.zoom);
            shouldSplitTile = distanceToTileMercator * tileScale < std::pow(cosPitchToTile, pitchExponent) *
                                                                       cameraToCenterDistanceMercator /
                                                                       state.tileLodScale * nominalScale;
            // Isomaps : BUDGET de tuiles du fork. La formule standard (façon Mapbox) produit le plein compte,
            // que ce rendu ne budgète pas (Mapbox gère via culling/budget GPU) → >900 tuiles = lag. On BORNE
            // le zoom atteignable par la distance — mais en distance AGL (déjà mesurée ci-dessus, boîte élevée
            // à la réf sol), donc correct en montagne (le proche reste net, le lointain retombe → compte borné).
            if (state.elevationProvider) {
                const double tileDistMaxZoom = -std::log2(std::max(1e-9, distanceToTileMercator)) - 4.4;
                if (static_cast<double>(node.zoom) + 1.0 > tileDistMaxZoom) shouldSplitTile = false;
                // PLAFONDS ALIGNÉS SUR LA BRUME (soupape mémoire assumée, façon Mapbox) : au-delà de
                // ~20/40/80 km le rendu est voilé à ~15/50/85 % (fogStart 8 km + 47 km, cf. shader
                // terrain) — inutile d'y payer des tuiles satellite de 4 Mo. Chaque palier borne le zoom
                // du maillage (et donc du satellite drapé, une texture par tuile de maillage).
                // CONTINU (les paliers 20/40/80 → 13/11/9 en marches d'escalier faisaient chuter une tuile
                // de 3 niveaux d'un coup au franchissement d'un seuil : « nette puis très floue quelques ms
                // plus tard » mesuré). Même droite d'ancrage, −2 niveaux par octave de distance au-delà de
                // 20 km, plancher 6.
                // MÊME MÉTRIQUE QUE LA BRUME : la brume du shader travaille en distance HORIZONTALE (à la
                // verticale l'air traversé est mince → pas de voile au nadir). L'ancien plafond en distance
                // 3D écrasait la vue plongeante de haute altitude : à 100 km d'altitude, le sol au nadir est
                // à 100 km en 3D mais 0 km horizontal → non voilé, et pourtant plafonné z7 → « une seule
                // tuile très floue » (mesuré, 40-130 km d'altitude). Ancre également ALIGNÉE sur la loi de
                // la brume (fogStart = max(8 km, alt×2)) : elle croît avec la hauteur de l'œil.
                const double distMeters = vec3Length(camToTileTiles) / metersToTileUnits;
                const double distHorizMeters = std::hypot(camToTileTiles[0], camToTileTiles[1]) / metersToTileUnits;
                const double eyeAGLMeters = std::max(0.0, eyeASLMeters - cameraGroundM);
                const double capAnchorMeters = std::max(20000.0, eyeAGLMeters * 2.0);
                if (distHorizMeters > capAnchorMeters) {
                    const double fogZoomCap = std::max(
                        6.0, 13.0 - 2.0 * std::log2(distHorizMeters / capAnchorMeters));
                    if (static_cast<double>(node.zoom) + 1.0 > fogZoomCap) shouldSplitTile = false;
                }
                // PLANCHER DE CHAMP ULTRA-PROCHE : la formule standard raffine RELATIVEMENT à la distance
                // du CENTRE (+~1,7 niveau max). Depuis une crête en visant l'autre versant (centre loin),
                // le sol sous l'œil plafonnait à « zoom du centre +1,7 » → premier plan flou pendant que
                // le second plan visé est net (mesuré, crête du Goûter). À moins de 500 m de l'œil : on
                // subdivise toujours (borné par maxZoom → coût de quelques tuiles).
                if (distMeters < 500.0) {
                    shouldSplitTile = true;
                }
            }
        } else {
            const vec3 distanceXyz = node.aabb.distanceXYZ(centerCoord);
            const double* longestDim = std::max_element(distanceXyz.data(), distanceXyz.data() + distanceXyz.size());
            assert(longestDim);

            // We're using distance based heuristics to determine if a tile should
            // be split into quadrants or not. radiusOfMaxLvlLodInTiles defines that
            // there's always a certain number of maxLevel tiles next to the map
            // center. Using the fact that a parent node in quadtree is twice the
            // size of its children (per dimension) we can define distance
            // thresholds for each relative level:
            // f(k) = offset + 2 + 4 + 8 + 16 + ... + 2^k
            // This is the same as:
            // f(k) = offset + 2^(k+1)-2
            const double distToSplit = radiusOfMaxLvlLodInTiles + (1 << (maxZoom - node.zoom)) - 2;
            shouldSplitTile = *longestDim * state.tileLodScale < distToSplit;
        }

        // Have we reached the target depth or is the tile too far away to be any split further?
        if (node.zoom == maxZoom || (!shouldSplitTile && node.zoom >= minZoom)) {
            // Perform precise intersection test between the frustum and aabb.
            // This will cull < 1% false positives missed by the original test
            const AABB preciseAABB = elevatedAABB(node);
            const bool preciseElevated = preciseAABB.min[2] != 0.0 || preciseAABB.max[2] != 0.0;
            // intersectsPrecise also flattens to z = 0; the 3D method already covers the
            // elevated case precisely, so reuse it rather than run the flat edge tests.
            const bool visible = node.fullyVisible ||
                                 (preciseElevated
                                      ? frustum.intersectsElevated(preciseAABB) != IntersectionResult::Separate
                                      : frustum.intersectsPrecise(preciseAABB, true) != IntersectionResult::Separate);
            if (visible) {
                const OverscaledTileID id = {
                    node.zoom == maxZoom ? overscaledZoom : node.zoom, node.wrap, node.zoom, node.x, node.y};
                vec3 coordToLoadFirst = (state.tileLodMode == TileLodMode::Distance) ? cameraCoord : centerCoord;
                const double dx = node.wrap * numTiles + node.x + 0.5 - coordToLoadFirst[0];
                const double dy = node.y + 0.5 - coordToLoadFirst[1];

                result.push_back({id, dx * dx + dy * dy});
            }
        } else {
            std::vector<Node> children = childrenOf(node);
            stack.insert(stack.end(), children.begin(), children.end());
        }
    }

    // Sort results by distance
    std::sort(
        result.begin(), result.end(), [](const ResultTile& a, const ResultTile& b) { return a.sqrDist < b.sqrDist; });

    std::vector<OverscaledTileID> ids;
    ids.reserve(result.size());

    for (const auto& tile : result) {
        ids.push_back(tile.id);
    }

    return ids;
}

std::set<UnwrappedTileID> frustumCull(const TileCoverParameters& state, const std::set<UnwrappedTileID>& tiles) {
    if (tiles.empty()) {
        return {};
    }
    const auto& transform = state.transformState;
    const bool flippedY = transform.getViewportMode() == ViewportMode::FlippedY;

    // Express every tile in the units of the deepest one, so its aabb is that tile's
    // footprint scaled up (never down) - the same tile-units-at-a-zoom space the
    // frustum is built in.
    uint8_t refZ = 0;
    for (const auto& id : tiles) {
        refZ = std::max(refZ, id.canonical.z);
    }
    const double numTiles = std::exp2(static_cast<double>(refZ));
    const double worldSize = Projection::worldSize(transform.getScale());
    // Meters to tile-units at refZ; see the same conversion in tileCover.
    const double metersToTileUnits = numTiles / (std::cos(util::deg2rad(transform.getLatLng().latitude())) *
                                                 util::M2PI * util::EARTH_RADIUS_M);
    // Isomaps : même correction d'unités Z du frustum que dans tileCover (cf. Frustum::fromInvProjMatrix).
    const double pixelsPerMeter = metersToTileUnits * worldSize / numTiles;
    const Frustum frustum = Frustum::fromInvProjMatrix(
        transform.getInvProjectionMatrix(), worldSize, refZ, flippedY, pixelsPerMeter);

    std::set<UnwrappedTileID> result;
    for (const auto& id : tiles) {
        const double span = std::exp2(static_cast<double>(refZ - id.canonical.z));
        const double x0 = (id.canonical.x + id.wrap * std::exp2(static_cast<double>(id.canonical.z))) * span;
        const double y0 = id.canonical.y * span;
        AABB aabb({{x0, y0, 0.0}}, {{x0 + span, y0 + span, 0.0}});

        if (state.elevationProvider) {
            if (const auto range = state.elevationProvider->getTileElevationRange(id.canonical)) {
                aabb.min[2] = range->min * metersToTileUnits;
                aabb.max[2] = range->max * metersToTileUnits;
            }
        }

        const bool elevated = aabb.min[2] != 0.0 || aabb.max[2] != 0.0;
        const IntersectionResult intersection = elevated ? frustum.intersectsElevated(aabb) : frustum.intersects(aabb);
        if (intersection != IntersectionResult::Separate) {
            result.insert(id);
        }
    }
    return result;
}

std::vector<UnwrappedTileID> tileCover(const LatLngBounds& bounds_, uint8_t z) {
    if (bounds_.isEmpty() || bounds_.south() > util::LATITUDE_MAX || bounds_.north() < -util::LATITUDE_MAX) {
        return {};
    }

    const LatLngBounds bounds = LatLngBounds::hull({std::max(bounds_.south(), -util::LATITUDE_MAX), bounds_.west()},
                                                   {std::min(bounds_.north(), util::LATITUDE_MAX), bounds_.east()});

    return tileCover(Projection::project(bounds.northwest(), z),
                     Projection::project(bounds.northeast(), z),
                     Projection::project(bounds.southeast(), z),
                     Projection::project(bounds.southwest(), z),
                     Projection::project(bounds.center(), z),
                     z);
}

std::vector<UnwrappedTileID> tileCover(const Geometry<double>& geometry, uint8_t z) {
    std::vector<UnwrappedTileID> result;
    TileCover tc(geometry, z, true);
    while (tc.hasNext()) {
        result.push_back(*tc.next());
    };

    return result;
}

// Taken from https://github.com/mapbox/sphericalmercator#xyzbbox-zoom-tms_style-srs
// Computes the projected tiles for the lower left and upper right points of the bounds
// and uses that to compute the tile cover count
uint64_t tileCount(const LatLngBounds& bounds, uint8_t zoom) noexcept {
    if (zoom == 0) {
        return 1;
    }
    const auto sw = Projection::project(bounds.southwest(), zoom);
    const auto ne = Projection::project(bounds.northeast(), zoom);
    const auto maxTile = std::pow(2.0, zoom);
    const auto x1 = floor(sw.x);
    const auto x2 = ceil(ne.x) - 1;
    const auto y1 = util::clamp(floor(sw.y), 0.0, maxTile - 1);
    const auto y2 = util::clamp(floor(ne.y), 0.0, maxTile - 1);

    const auto dx = x1 > x2 ? (maxTile - x1) + x2 : x2 - x1;
    const auto dy = y1 - y2;
    return static_cast<uint64_t>((dx + 1) * (dy + 1));
}

uint64_t tileCount(const Geometry<double>& geometry, uint8_t z) {
    uint64_t tileCount = 0;

    TileCover tc(geometry, z, true);
    while (tc.next()) {
        tileCount++;
    };
    return tileCount;
}

TileCover::TileCover(const LatLngBounds& bounds_, uint8_t z) {
    LatLngBounds bounds = LatLngBounds::hull({std::max(bounds_.south(), -util::LATITUDE_MAX), bounds_.west()},
                                             {std::min(bounds_.north(), util::LATITUDE_MAX), bounds_.east()});

    if (bounds.isEmpty() || bounds.south() > util::LATITUDE_MAX || bounds.north() < -util::LATITUDE_MAX) {
        bounds = LatLngBounds::world();
    }

    const auto sw = Projection::project(bounds.southwest(), z);
    const auto ne = Projection::project(bounds.northeast(), z);
    const auto se = Projection::project(bounds.southeast(), z);
    const auto nw = Projection::project(bounds.northwest(), z);

    const Polygon<double> p({{sw, nw, ne, se, sw}});
    impl = std::make_unique<TileCover::Impl>(z, p, false);
}

TileCover::TileCover(const Geometry<double>& geom, uint8_t z, bool project /* = true*/)
    : impl(std::make_unique<TileCover::Impl>(z, geom, project)) {}

TileCover::~TileCover() = default;

std::optional<UnwrappedTileID> TileCover::next() {
    return impl->next();
}

bool TileCover::hasNext() {
    return impl->hasNext();
}

} // namespace util
} // namespace mbgl
