#include <mbgl/geometry/dem_data.hpp>
#include <mbgl/math/clamp.hpp>

#include <algorithm>

namespace mbgl {

DEMData::DEMData(const PremultipliedImage& _image, Tileset::RasterEncoding _encoding)
    : dim(_image.size.height),
      // extra two pixels per row for border backfilling on either edge
      stride(dim + 2),
      encoding(_encoding) {
    image = std::make_shared<PremultipliedImage>(Size(static_cast<uint32_t>(stride), static_cast<uint32_t>(stride)));
    if (_image.size.height != _image.size.width) {
        throw std::runtime_error("raster-dem tiles must be square.");
    }

    auto* dest = reinterpret_cast<uint32_t*>(image->data.get()) + stride + 1;
    auto* source = reinterpret_cast<uint32_t*>(_image.data.get());
    for (int32_t y = 0; y < dim; y++) {
        memcpy(dest, source, dim * 4);
        dest += stride;
        source += dim;
    }

    // Isomaps : RÉPARATION NoData AU DÉCODAGE. Les pyramides DEM sans bathymétrie encodent l'eau en
    // NoData (noir terrain-RGB = −10000 m ; mesuré : tuiles z13-z16 entièrement à −9999 sur le Léman).
    // Sans réparation : plage d'élévation à −11 km → boîte de frustum sous terre → tuile jamais émise
    // par le cover → bloc grossier pâle permanent ; et si elle est émise, le maillage plonge en abîme.
    //  • Tuile MIXTE (rive) : chaque pixel NoData prend le MINIMUM VALIDE de la tuile ≈ l'altitude du
    //    plan d'eau — exactement la surface attendue.
    //  • Tuile ENTIÈREMENT NoData (large) : marquée invalide (isAllNoData) — les consommateurs la
    //    traitent comme ABSENTE et se replient sur l'ancêtre (plus grossier mais correct).
    {
        constexpr int32_t kNoDataFloorMeters = -600; // sous la mer Morte : invraisemblable sur Terre
        int32_t minValid = std::numeric_limits<int32_t>::max();
        for (int32_t y = 0; y < dim; y++) {
            for (int32_t x = 0; x < dim; x++) {
                const int32_t v = get(x, y);
                if (v >= kNoDataFloorMeters) {
                    minValid = std::min(minValid, v);
                }
            }
        }
        if (minValid == std::numeric_limits<int32_t>::max()) {
            allNoData = true;
        } else {
            const auto& unpack = getUnpackVector();
            // Ré-encodage inverse de get() : q tel que r*u0 + g*u1 + b*u2 − u3 = minValid, avec
            // r=q>>16, g=(q>>8)&255, b=q&255 (u0/u1/u2 sont en progression ×256 pour les 2 encodages).
            const auto q = static_cast<uint32_t>((static_cast<double>(minValid) + unpack[3]) / unpack[2]);
            auto* px = image->data.get();
            for (int32_t y = 0; y < dim; y++) {
                for (int32_t x = 0; x < dim; x++) {
                    if (get(x, y) < kNoDataFloorMeters) {
                        uint8_t* p = px + idx(x, y) * 4;
                        p[0] = static_cast<uint8_t>((q >> 16) & 0xFF);
                        p[1] = static_cast<uint8_t>((q >> 8) & 0xFF);
                        p[2] = static_cast<uint8_t>(q & 0xFF);
                    }
                }
            }
        }
    }

    // in order to avoid flashing seams between tiles, here we are initially
    // populating a 1px border of pixels around the image with the data of the
    // nearest pixel from the image. this data is eventually replaced when the
    // tile's neighboring tiles are loaded and the accurate data can be
    // backfilled using DEMData#backfillBorder

    auto* data = reinterpret_cast<uint32_t*>(image->data.get());
    for (int32_t x = 0; x < dim; x++) {
        auto rowOffset = stride * (x + 1);
        // left vertical border
        data[rowOffset] = data[rowOffset + 1];

        // right vertical border
        data[rowOffset + dim + 1] = data[rowOffset + dim];
    }

    // top horizontal border with corners
    memcpy(data, data + stride, stride * 4);
    // bottom horizontal border with corners
    memcpy(data + (dim + 1) * stride, data + dim * stride, stride * 4);

    // The elevation range of the tile, used to give the tile a height when testing
    // it against the view frustum (see util::tileCover): terrain rising towards the
    // camera is visible from further away than its flat footprint suggests. Computed
    // once here, on the worker thread that decodes the tile, rather than per frame.
    // The border is excluded: it is a copy of the edge pixels until neighbouring
    // tiles backfill it, so it holds no elevation this tile does not already have.
    if (dim > 0) {
        minElevation = maxElevation = get(0, 0);
        for (int32_t y = 0; y < dim; y++) {
            for (int32_t x = 0; x < dim; x++) {
                const int32_t value = get(x, y);
                minElevation = std::min(minElevation, value);
                maxElevation = std::max(maxElevation, value);
            }
        }
    }
}

// This function takes the DEMData from a neighboring tile and backfills the
// edge/corner data in order to create a one pixel "buffer" of image data around
// the tile. This is necessary because the hillshade formula calculates the
// dx/dz, dy/dz derivatives at each pixel of the tile by querying the 8
// surrounding pixels, and if we don't have the pixel buffer we get seams at
// tile boundaries.
void DEMData::backfillBorder(const DEMData& borderTileData, int8_t dx, int8_t dy) {
    auto& o = borderTileData;

    // Tiles from the same source should always be of the same dimensions.
    assert(dim == o.dim);

    // We determine the pixel range to backfill based which corner/edge
    // `borderTileData` represents. For example, dx = -1, dy = -1 represents the
    // upper left corner of the base tile, so we only need to backfill one pixel
    // at coordinates (-1, -1) of the tile image.
    int32_t xMin = dx * dim;
    int32_t xMax = dx * dim + dim;
    int32_t yMin = dy * dim;
    int32_t yMax = dy * dim + dim;

    if (dx == -1)
        xMin = xMax - 1;
    else if (dx == 1)
        xMax = xMin + 1;

    if (dy == -1)
        yMin = yMax - 1;
    else if (dy == 1)
        yMax = yMin + 1;

    int32_t ox = -dx * dim;
    int32_t oy = -dy * dim;

    auto* dest = reinterpret_cast<uint32_t*>(image->data.get());
    auto* source = reinterpret_cast<uint32_t*>(o.image->data.get());

    for (int32_t y = yMin; y < yMax; y++) {
        for (int32_t x = xMin; x < xMax; x++) {
            dest[idx(x, y)] = source[idx(x + ox, y + oy)];
        }
    }
}

int32_t DEMData::get(const int32_t x, const int32_t y) const {
    const auto& unpack = getUnpackVector();
    const uint8_t* value = image->data.get() + idx(x, y) * 4;
    return static_cast<int32_t>(value[0] * unpack[0] + value[1] * unpack[1] + value[2] * unpack[2] - unpack[3]);
}

const std::array<float, 4>& DEMData::getUnpackVector() const {
    // https://www.mapbox.com/help/access-elevation-data/#mapbox-terrain-rgb
    static const std::array<float, 4> unpackMapbox = {{6553.6f, 25.6f, 0.1f, 10000.0f}};
    // https://aws.amazon.com/public-datasets/terrain/
    static const std::array<float, 4> unpackTerrarium = {{256.0f, 1.0f, 1.0f / 256.0f, 32768.0f}};

    return encoding == Tileset::RasterEncoding::Terrarium ? unpackTerrarium : unpackMapbox;
}

} // namespace mbgl
