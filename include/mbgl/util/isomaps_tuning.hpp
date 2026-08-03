#pragma once

// Isomaps — réglages runtime pilotés par l'app (sans passer par le style).

namespace mbgl {
namespace isomaps {

// BRUME SYMBOLES : rideau d'extinction des étiquettes en 3D (mètres).
// Les étiquettes sont nettes jusqu'à max(startMeters, 2 × altitude caméra) — le plancher
// d'altitude évite qu'une vue planétaire ne perde TOUTES ses étiquettes — puis s'éteignent
// linéairement sur widthMeters. startMeters <= 0 restaure la loi par défaut (8 km / 47 km).
void setSymbolFade(float startMeters, float widthMeters);
float getSymbolFadeStartMeters(); // <= 0 si non piloté
float getSymbolFadeWidthMeters();

// PLAFOND DE ZOOM 3D PILOTÉ PAR L'APPAREIL : borne ABSOLUE sur le zoom des tuiles demandées par
// le cover quand le terrain est actif (maillage + sources drapées). Posé par l'app selon la
// mémoire du device (Android 4 Go → 14 ; 6-8 Go → 15-16 ; flagships → 0 = pas de plafond).
// Filet contre les sources sans maxzoom. <= 0 = désactivé (comportement iOS inchangé).
void setTerrainZoomCap(float maxZoom);
float getTerrainZoomCap();

// CAMÉRA CINÉMATIQUE (flyover) : suspend les contraintes INTERACTIVES du terrain — collision,
// renormalisation AGL, rampe de pitch — et fige le plan de référence à 0. La caméra devient
// purement pilotée par l'app (sémantique déterministe : zoom = hauteur d'œil ASL).
void setCinematicCamera(bool on);
bool isCinematicCamera();

} // namespace isomaps
} // namespace mbgl
