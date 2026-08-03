package org.maplibre.android.style.terrain

/**
 * The style's 3D terrain configuration.
 *
 * Terrain drapes the map over the elevation data of a raster-dem source.
 * Per the style specification, `exaggeration` is a vertical multiplier where
 * 1.0 renders true-scale elevation.
 *
 * It is recommended to give terrain its own raster-dem source (even when it
 * uses the same tiles as a hillshade source) so styles stay portable with
 * maplibre-gl-js, which keeps a separate terrain source cache.
 *
 * @property source the id of the raster-dem source providing the elevation data
 * @property exaggeration the vertical exaggeration multiplier, 1.0 by default
 */
class Terrain @JvmOverloads constructor(
    val source: String,
    val exaggeration: Float = 1.0f,
    /**
     * Isomaps — mode « basemap direct » : id d'une source RASTER du style que le maillage terrain
     * échantillonne DIRECTEMENT (façon Mapbox), au lieu du drapage render-to-texture. Net à tous
     * les zooms ; les couches vectorielles restent drapées par-dessus (overlay). null = drapage
     * classique.
     */
    val basemap: String? = null
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Terrain) return false
        return source == other.source && exaggeration == other.exaggeration && basemap == other.basemap
    }

    override fun hashCode(): Int {
        return (31 * source.hashCode() + exaggeration.hashCode()) * 31 + (basemap?.hashCode() ?: 0)
    }

    override fun toString(): String {
        return "Terrain{source=$source, exaggeration=$exaggeration, basemap=$basemap}"
    }
}
