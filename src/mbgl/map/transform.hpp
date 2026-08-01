#pragma once

#include <mbgl/map/camera.hpp>
#include <mbgl/map/projection_mode.hpp>
#include <mbgl/map/map_observer.hpp>
#include <mbgl/map/mode.hpp>
#include <mbgl/map/transform_state.hpp>
#include <mbgl/util/chrono.hpp>
#include <mbgl/util/geo.hpp>
#include <mbgl/util/noncopyable.hpp>

#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <unordered_map>

namespace mbgl {

class TransformObserver {
public:
    virtual ~TransformObserver() = default;

    static TransformObserver& nullObserver() {
        static TransformObserver observer;
        return observer;
    }

    virtual void onCameraWillChange(MapObserver::CameraChangeMode) {}
    virtual void onCameraIsChanging() {}
    virtual void onCameraDidChange(MapObserver::CameraChangeMode) {}
};

class Transform : private util::noncopyable {
public:
    Transform(TransformObserver& = TransformObserver::nullObserver(),
              ConstrainMode = ConstrainMode::HeightOnly,
              ViewportMode = ViewportMode::Default);

    Transform(const TransformState& state_)
        : observer(TransformObserver::nullObserver()),
          state(state_) {}

    // Map view
    void resize(Size size);

    // Camera
    /** Returns the current camera options. */
    CameraOptions getCameraOptions(const std::optional<EdgeInsets>&) const;

    /** Instantaneously, synchronously applies the given camera options. */
    void jumpTo(const CameraOptions&);
    /** Asynchronously transitions all specified camera options linearly along
        an optional time curve. However, center coordinate is not transitioned
        linearly as, instead, ground speed is kept linear.*/
    void easeTo(const CameraOptions&, const AnimationOptions& = {});
    /** Asynchronously zooms out, pans, and zooms back into the given camera
        along a great circle, as though the viewer is riding a supersonic
        jetcopter.
        Parameter linearZoomInterpolation: when true, there is no additional
        zooming out as zoom is linearly interpolated from current to given
        camera zoom. This is used for easeTo.*/
    void flyTo(const CameraOptions&, const AnimationOptions& = {}, bool linearZoomInterpolation = false);

    // Position

    /** Pans the map by the given amount.
        @param offset The distance to pan the map by, measured in pixels from
            top to bottom and from left to right. */
    void moveBy(const ScreenCoordinate& offset, const AnimationOptions& = {});
    LatLng getLatLng(LatLng::WrapMode = LatLng::Wrapped) const;

    // Bounds

    void setLatLngBounds(LatLngBounds);
    void setMinZoom(double);
    void setMaxZoom(double);

    void setMinPitch(double);
    void setMaxPitch(double);

    // Zoom

    /** Returns the zoom level. */
    double getZoom() const;

    // Bearing

    void rotateBy(const ScreenCoordinate& first, const ScreenCoordinate& second, const AnimationOptions& = {});
    double getBearing() const;

    // Pitch

    double getPitch() const;
    double getRoll() const;
    double getFieldOfView() const;

    // North Orientation
    void setNorthOrientation(NorthOrientation);
    NorthOrientation getNorthOrientation() const;

    // Constrain mode
    void setConstrainMode(ConstrainMode);
    ConstrainMode getConstrainMode() const;

    // Viewport mode
    void setViewportMode(ViewportMode);
    ViewportMode getViewportMode() const;

    // Projection mode
    void setProjectionMode(const ProjectionMode&);
    ProjectionMode getProjectionMode() const;

    // Transitions
    bool inTransition() const;
    void updateTransitions(const TimePoint& now);
    TimePoint getTransitionStart() const { return transitionStart; }
    Duration getTransitionDuration() const { return transitionDuration; }
    void cancelTransitions();

    // Gesture
    void setGestureInProgress(bool);
    bool isGestureInProgress() const { return state.isGestureInProgress(); }

    // Transform state
    const TransformState& getState() const { return state; }
    bool isRotating() const { return state.isRotating(); }
    bool isScaling() const { return state.isScaling(); }
    bool isPanning() const { return state.isPanning(); }

    // Conversion and projection
    ScreenCoordinate latLngToScreenCoordinate(const LatLng&) const;
    LatLng screenCoordinateToLatLng(const ScreenCoordinate&, LatLng::WrapMode = LatLng::Wrapped) const;

    FreeCameraOptions getFreeCameraOptions() const;
    void setFreeCameraOptions(const FreeCameraOptions& options);

    // Isomaps : collision caméra/terrain. `elevationFn` renvoie l'altitude (m au-dessus du niveau de la
    // mer) du terrain sous un point, ou nullopt si indisponible (pas de DEM chargé → pas de contrainte
    // cette frame). L'œil est empêché de descendre sous `minMetersAboveGround` m AU-DESSUS DU SOL, à chaque
    // mouvement de caméra. Injectée car le DEM vit côté render, hors du Transform. minMeters<=0 = désactivé.
    void setTerrainCameraCollision(std::function<std::optional<double>(const LatLng&)> elevationFn,
                                   double minMetersAboveGround);
    // Remonte l'œil s'il passe sous terrainCollisionMinAGL m au-dessus du sol. Appelée en fin de chaque
    // mutation caméra (easeTo/flyTo/setFreeCameraOptions) ET en continu via Map::enforceTerrainCameraConstraints
    // (displayLink) — sinon, au lancement (DEM pas encore chargé), la caméra peut rester sous le terrain.
    void clampEyeAboveTerrain();
    /// Isomaps : reparamétrisation PURE (l'œil ne bouge pas d'un pixel) — centerAltitude glisse vers le
    /// terrain au centre, zoom et centre compensés. Le zoom devient AGL-vrai → vitesses de pan/pinch et
    /// ancre de rotation correctes en montagne (sinon calées sur le niveau de la mer). Jamais pendant un
    /// geste actif (la baseline du geste sauterait).
    void renormalizeCenterAltitudeToTerrain();

    // Frustum
    void setFrustumOffset(const EdgeInsets&);
    EdgeInsets getFrustumOffset();

private:
    TransformObserver& observer;
    TransformState state;

    // Isomaps collision : fonction d'élévation terrain (injectée par le frontend) + hauteur mini œil-sol.
    std::function<std::optional<double>(const LatLng&)> terrainCollisionElevationFn;
    double terrainCollisionMinAGL = 0.0; // mètres ; <= 0 = désactivé
    // Maintien de l'élévation de référence : la sonde perd souvent le terrain (sous l'œil = hors frustum
    // → NUL une frame sur deux). On retient le max récent et on le laisse décroître lentement → contrainte
    // CONTINUE (comble les trous, garde l'œil au-dessus le temps de franchir un relief qu'on vient de voir).
    double collisionRefGroundHold = 0.0;
    // Isomaps : pitch au tick précédent de clampEyeAboveTerrain — sert à détecter qu'un geste est en train
    // d'AUGMENTER le pitch, pour résoudre une violation terrain en bornant le PITCH (le geste bute) plutôt
    // qu'en dézoomant (sinon : bras de fer geste/butée à chaque frame → fuite de zoom + saccades).
    double lastConstraintPitch = -1.0;
    // Isomaps : CLIQUET de pitch pendant un geste — au premier contact avec le relief, l'angle limite est
    // mémorisé et TIENT jusqu'à la fin du geste (fige net). Sans lui, la limite recalculée à chaque tick
    // suit le bruit de la sonde terrain (pas de 75 m, maintien décroissant) → pitch qui tremble.
    double gesturePitchCeiling = std::numeric_limits<double>::infinity();
    // Isomaps : CLIQUET de zoom pendant un geste — quand la butée (paroi/contact) corrige le zoom en plein
    // pinch, le plafond tient jusqu'à la fin du geste. Sans lui, le plancher « sol sous l'œil » SOULÈVE
    // l'œil par-dessus la crête visée, la visée bascule sur le terrain derrière et le zoom repart →
    // franchissement involontaire (mesuré : « je visais le refuge du Goûter, j'ai dépassé la crête »).
    // Relâcher puis re-pincer réévalue : le franchissement reste possible, mais VOLONTAIRE.
    double gestureZoomCeiling = std::numeric_limits<double>::infinity();
    // Mémoire d'élévation : le terrain sous l'œil est souvent hors frustum (non chargé → requête = 0). On
    // retient le MAX vu par cellule spatiale (~100 m) quand il ÉTAIT chargé (à l'écran) ; on le ressort en
    // secours quand la requête live échoue → tout relief approché récemment protège l'œil (le sol est fixe).
    std::unordered_map<int64_t, float> collisionElevCache;

    void startTransition(const CameraOptions&,
                         const AnimationOptions&,
                         const std::function<void(double)>&,
                         const Duration&);

    // We don't want to show horizon: limit max pitch based on edge insets.
    double getMaxPitchForEdgeInsets(const EdgeInsets& insets) const;

    TimePoint transitionStart;
    Duration transitionDuration;
    std::function<bool(const TimePoint)> transitionFrameFn;
    std::function<void()> transitionFinishFn;
};

} // namespace mbgl
