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

} // namespace isomaps
} // namespace mbgl
