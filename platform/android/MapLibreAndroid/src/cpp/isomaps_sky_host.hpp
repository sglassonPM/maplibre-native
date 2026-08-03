#pragma once
#include <jni.h>
namespace mbgl {
namespace android {
namespace isomaps_sky {
// Calque CIEL Isomaps (GLES3) — port du calque Metal de l'app d'éval iOS, intégré au SDK.
// createContext() rend un pointeur de CustomLayerHost à passer à CustomLayer(id, ptr).
jlong createContext();
// Panorama équirect RGBA8 (soleil recalé plein sud) — à pousser AVANT createContext().
void storePanorama(const uint8_t* rgba, int width, int height);
} // namespace isomaps_sky
} // namespace android
} // namespace mbgl
