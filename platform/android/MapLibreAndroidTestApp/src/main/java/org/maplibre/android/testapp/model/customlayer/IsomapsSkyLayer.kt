package org.maplibre.android.testapp.model.customlayer

import androidx.annotation.Keep

/**
 * Isomaps : calque CIEL (dégradé + soleil au sud + nuages procéduraux) — port GLES du calque
 * Metal de l'app d'éval iOS. À insérer au-dessus du background et sous tout le reste.
 */
@Keep
object IsomapsSkyLayer {
    init {
        System.loadLibrary("isomaps-sky-layer")
    }

    external fun createContext(): Long

    /** Panorama équirect RGBA8 (soleil pré-recalé au centre = sud) — à pousser AVANT createContext. */
    external fun setPanorama(pixels: java.nio.ByteBuffer, width: Int, height: Int)
}
