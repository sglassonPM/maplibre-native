#pragma once
#include <jni.h>
namespace mbgl {
namespace android {
namespace isomaps_wind {
// Particules de vent Isomaps (GLES3) — port du calque WebGL du site et du calque
// Metal iOS. Le champ u/v est poussé AVANT createContext(), comme le panorama du ciel.
//
// createContext() rend un pointeur de CustomLayerHost à passer à CustomLayer(id, ptr).
jlong createContext();

// Champ u/v d'UNE zone, en RGBA8 : R = u, G = v, A = validité.
// `offset` et `scale` viennent du serveur (km/h = canal * scale + offset) : aucune
// constante d'encodage n'est écrite côté natif.
// Les zones sont poussées de la plus FINE à la plus GROSSIÈRE, puis clearFields()
// entre deux échéances.
void addField(const uint8_t* rgba, int width, int height,
              double ouest, double sud, double est, double nord,
              double offset, double scale);
void clearFields();

// Réglages d'animation, tous pilotés par le serveur.
void setSettings(int count, int trail, double lifeS, double speedFactor, double speedFull);
} // namespace isomaps_wind
} // namespace android
} // namespace mbgl
