#pragma once

#include <mbgl/style/terrain.hpp>
#include <mbgl/util/immutable.hpp>
#include <string>

namespace mbgl {
namespace style {

class Terrain::Impl {
public:
    Impl()
        : sourceID(""),
          exaggeration(1.0f) {}
    Impl(const std::string& sourceID_, float exaggeration_)
        : sourceID(sourceID_),
          exaggeration(exaggeration_) {}

    std::string sourceID;
    float exaggeration;
    // Isomaps : id de la source raster BASEMAP à échantillonner DIRECTEMENT sur le maillage terrain
    // (au lieu de la draper dans une cible offscreen par tuile — coûteux). Vide = fallback auto
    // (source raster drapée principale). Cf. RenderTerrain (raster-on-terrain direct, façon Mapbox).
    std::string basemapSourceID;

    bool operator==(const Impl& other) const {
        return sourceID == other.sourceID && exaggeration == other.exaggeration &&
               basemapSourceID == other.basemapSourceID;
    }
};

} // namespace style
} // namespace mbgl
