#include <mbgl/map/camera.hpp>
#include <mbgl/map/transform.hpp>
#include <mbgl/util/constants.hpp>
#include <mbgl/util/mat4.hpp>
#include <mbgl/util/math.hpp>
#include <mbgl/util/unitbezier.hpp>
#include <mbgl/util/interpolate.hpp>
#include <mbgl/util/chrono.hpp>
#include <mbgl/util/projection.hpp>
#include <mbgl/math/angles.hpp>
#include <mbgl/math/clamp.hpp>
#include <mbgl/util/logging.hpp>
#include <mbgl/util/platform.hpp>

#include <cstdio>
#include <utility>
#include <numbers>

using namespace std::numbers;

namespace mbgl {

/** Converts the given angle (in radians) to be numerically close to the anchor
 * angle, allowing it to be interpolated properly without sudden jumps. */
namespace {
double _normalizeAngle(double angle, double anchorAngle) {
    if (std::isnan(angle) || std::isnan(anchorAngle)) {
        return 0;
    }

    angle = util::wrap(angle, -pi, pi);
    if (angle == -pi) angle = pi;
    double diff = std::abs(angle - anchorAngle);
    if (std::abs(angle - util::M2PI - anchorAngle) < diff) {
        angle -= util::M2PI;
    }
    if (std::abs(angle + util::M2PI - anchorAngle) < diff) {
        angle += util::M2PI;
    }

    return angle;
}
} // namespace

Transform::Transform(TransformObserver& observer_, ConstrainMode constrainMode, ViewportMode viewportMode)
    : observer(observer_),
      state(constrainMode, viewportMode) {}

// MARK: - Map View

void Transform::resize(const Size size) {
    if (size.isEmpty()) {
        throw std::runtime_error("failed to resize: size is empty");
    }

    if (state.getSize() == size) {
        return;
    }

    observer.onCameraWillChange(MapObserver::CameraChangeMode::Immediate);
    state.setSize(size);
    double scale{state.getScale()};
    double x{state.getX()};
    double y{state.getY()};
    double z{state.getZ()};

    double lat;
    double lon;
    if (state.constrainScreen(scale, lat, lon)) {
        // Turns out that if you resize during a transition any changes made to the state will be ignored :(
        // So if we have to constrain because of a resize and a transition is in progress - cancel the transition!
        if (inTransition()) {
            cancelTransitions();
        }

        // It also turns out that state.setProperties isn't enough if you change the center, you need to set Cc and Bc
        // too, which setLatLngZoom does.
        state.setLatLngZoom(mbgl::LatLng{lat, lon}, state.scaleZoom(scale));
        observer.onCameraDidChange(MapObserver::CameraChangeMode::Immediate);

        return;
    }
    state.constrain(scale, x, y);
    state.setProperties(TransformStateProperties().withScale(scale).withX(x).withY(y).withZ(z));

    observer.onCameraDidChange(MapObserver::CameraChangeMode::Immediate);
}

// MARK: - Camera

CameraOptions Transform::getCameraOptions(const std::optional<EdgeInsets>& padding) const {
    return state.getCameraOptions(padding);
}

/**
 * Change any combination of center, zoom, bearing, and pitch, without
 * a transition. The map will retain the current values for any options
 * not included in `options`.
 */
void Transform::jumpTo(const CameraOptions& camera) {
    easeTo(camera);
}

/**
 * Change any combination of center, zoom, bearing, pitch and edgeInsets, with a
 * smooth animation between old and new values. The map will retain the current
 * values for any options not included in `options`.
 */
void Transform::easeTo(const CameraOptions& inputCamera, const AnimationOptions& animation) {
    CameraOptions camera = inputCamera;

    Duration duration = animation.duration.value_or(Duration::zero());
    if (state.getLatLngBounds() == LatLngBounds() && !isGestureInProgress() && duration != Duration::zero()) {
        // reuse flyTo, without exaggerated animation, to achieve constant ground speed.
        flyTo(camera, animation, true);
        return;
    }

    double zoom = camera.zoom.value_or(getZoom());
    state.constrainCameraAndZoomToBounds(camera, zoom);

    const EdgeInsets& padding = camera.padding.value_or(state.getEdgeInsets());
    LatLng startLatLng = getLatLng(LatLng::Unwrapped);
    const LatLng& unwrappedLatLng = camera.center.value_or(startLatLng);
    const LatLng& latLng = state.getLatLngBounds() != LatLngBounds() ? unwrappedLatLng : unwrappedLatLng.wrapped();

    double bearing = camera.bearing ? util::deg2rad(-*camera.bearing) : getBearing();
    double pitch = camera.pitch ? util::deg2rad(*camera.pitch) : getPitch();
    double fov = camera.fov ? util::deg2rad(*camera.fov) : getFieldOfView();
    double centerAlt = camera.centerAltitude.value_or(state.getCenterAltitude());
    double roll = camera.roll ? util::deg2rad(*camera.roll) : getRoll();

    if (std::isnan(zoom) || std::isnan(bearing) || std::isnan(pitch) || std::isnan(roll) || std::isnan(fov)) {
        if (animation.transitionFinishFn) {
            animation.transitionFinishFn();
        }
        return;
    }

    if (state.getLatLngBounds() == LatLngBounds()) {
        if (isGestureInProgress()) {
            // If gesture in progress, we transfer the wrap rounds from the end
            // longitude into start, so the "scroll effect" of rounding the
            // world is the same while assuring the end longitude remains
            // wrapped.
            const double wrap = unwrappedLatLng.longitude() - latLng.longitude();
            startLatLng = LatLng(startLatLng.latitude(), startLatLng.longitude() - wrap);
        } else {
            // Find the shortest path otherwise.
            startLatLng.unwrapForShortestPath(latLng);
        }
    }
    const double startCenterAlt = state.getCenterAltitude();

    const Point<double> startPoint = Projection::project(startLatLng, state.getScale());
    const Point<double> endPoint = Projection::project(latLng, state.getScale());

    // Constrain camera options.
    zoom = util::clamp(zoom, state.getMinZoom(), state.getMaxZoom());
    pitch = util::clamp(pitch, state.getMinPitch(), state.getMaxPitch());
    fov = util::clamp(fov, state.getMinFieldOfView(), state.getMaxFieldOfView());

    // Minimize rotation by taking the shorter path around the circle.
    bearing = _normalizeAngle(bearing, state.getBearing());
    state.setBearing(_normalizeAngle(state.getBearing(), bearing));

    const double startZoom = state.getZoom();
    const double startBearing = state.getBearing();
    const double startPitch = state.getPitch();
    const double startRoll = state.getRoll();
    const double startFov = state.getFieldOfView();
    state.setProperties(TransformStateProperties()
                            .withPanningInProgress(unwrappedLatLng != startLatLng)
                            .withScalingInProgress(zoom != startZoom)
                            .withRotatingInProgress(bearing != startBearing));
    const EdgeInsets startEdgeInsets = state.getEdgeInsets();

    startTransition(
        camera,
        animation,
        [=, this](double t) {
            Point<double> framePoint = util::interpolate(startPoint, endPoint, t);
            LatLng frameLatLng = Projection::unproject(framePoint, state.zoomScale(startZoom));
            double frameZoom = util::interpolate(startZoom, zoom, t);
            state.setLatLngZoom(frameLatLng, frameZoom);
            state.setCenterAltitude(util::interpolate(startCenterAlt, centerAlt, t));
            if (bearing != startBearing) {
                state.setBearing(util::wrap(util::interpolate(startBearing, bearing, t), -pi, pi));
            }
            if (padding != startEdgeInsets) {
                // Interpolate edge insets
                EdgeInsets edgeInsets;
                state.setEdgeInsets({util::interpolate(startEdgeInsets.top(), padding.top(), t),
                                     util::interpolate(startEdgeInsets.left(), padding.left(), t),
                                     util::interpolate(startEdgeInsets.bottom(), padding.bottom(), t),
                                     util::interpolate(startEdgeInsets.right(), padding.right(), t)});
            }
            double maxPitch = getMaxPitchForEdgeInsets(state.getEdgeInsets());
            if (pitch != startPitch || maxPitch < startPitch) {
                state.setPitch(std::min(maxPitch, util::interpolate(startPitch, pitch, t)));
            }
            if (roll != startRoll) {
                state.setRoll(util::interpolate(startRoll, roll, t));
            }
            if (fov != startFov) {
                state.setFieldOfView(util::interpolate(startFov, fov, t));
            }
            renormalizeCenterAltitudeToTerrain(); // Isomaps : zoom AGL-vrai (vitesses de gestes correctes)
            clampEyeAboveTerrain(); // Isomaps : œil >= sol + minAGL, après tous les setters de la frame
        },
        duration);
}

/** This method implements an “optimal path” animation, as detailed in:

    Van Wijk, Jarke J.; Nuij, Wim A. A. “Smooth and efficient zooming and
        panning.” INFOVIS ’03. pp. 15–22.
        <https://www.win.tue.nl/~vanwijk/zoompan.pdf#page=5>.

    Where applicable, local variable documentation begins with the associated
    variable or function in van Wijk (2003). */
void Transform::flyTo(const CameraOptions& inputCamera,
                      const AnimationOptions& animation,
                      bool linearZoomInterpolation) {
    CameraOptions camera = inputCamera;

    double zoom = camera.zoom.value_or(getZoom());
    state.constrainCameraAndZoomToBounds(camera, zoom);

    const EdgeInsets& padding = camera.padding.value_or(state.getEdgeInsets());
    const LatLng& latLng = camera.center.value_or(getLatLng(LatLng::Unwrapped)).wrapped();
    const double centerAlt = camera.centerAltitude.value_or(state.getCenterAltitude());
    double bearing = camera.bearing ? util::deg2rad(-*camera.bearing) : getBearing();
    double pitch = camera.pitch ? util::deg2rad(*camera.pitch) : getPitch();
    double roll = camera.roll ? util::deg2rad(*camera.roll) : getRoll();
    double fov = camera.fov ? util::deg2rad(*camera.fov) : getFieldOfView();

    if (std::isnan(zoom) || std::isnan(bearing) || std::isnan(pitch) || std::isnan(roll) || std::isnan(fov) ||
        state.getSize().isEmpty()) {
        if (animation.transitionFinishFn) {
            animation.transitionFinishFn();
        }
        return;
    }

    // Determine endpoints.
    LatLng startLatLng = getLatLng(LatLng::Unwrapped).wrapped();
    startLatLng.unwrapForShortestPath(latLng);
    const double startCenterAlt = state.getCenterAltitude();

    const Point<double> startPoint = Projection::project(startLatLng, state.getScale());
    const Point<double> endPoint = Projection::project(latLng, state.getScale());

    // Constrain camera options.
    zoom = util::clamp(zoom, state.getMinZoom(), state.getMaxZoom());
    pitch = util::clamp(pitch, state.getMinPitch(), state.getMaxPitch());
    fov = util::clamp(fov, state.getMinFieldOfView(), state.getMaxFieldOfView());

    // Minimize rotation by taking the shorter path around the circle.
    bearing = _normalizeAngle(bearing, state.getBearing());
    state.setBearing(_normalizeAngle(state.getBearing(), bearing));
    const double startZoom = state.scaleZoom(state.getScale());
    const double startBearing = state.getBearing();
    const double startPitch = state.getPitch();
    const double startRoll = state.getRoll();
    const double startFov = state.getFieldOfView();

    /// w₀: Initial visible span, measured in pixels at the initial scale.
    /// Known henceforth as a <i>screenful</i>.

    double w0 = std::max(state.getSize().width - padding.left() - padding.right(),
                         state.getSize().height - padding.top() - padding.bottom());
    /// w₁: Final visible span, measured in pixels with respect to the initial
    /// scale.
    double w1 = w0 / state.zoomScale(zoom - startZoom);
    /// Length of the flight path as projected onto the ground plane, measured
    /// in pixels from the world image origin at the initial scale.
    double u1 = ::hypot((endPoint - startPoint).x, (endPoint - startPoint).y);

    /** ρ: The relative amount of zooming that takes place along the flight
        path. A high value maximizes zooming for an exaggerated animation, while
        a low value minimizes zooming for something closer to easeTo().

        1.42 is the average value selected by participants in the user study in
        van Wijk (2003). A value of 6<sup>¼</sup> would be equivalent to the
        root mean squared average velocity, V<sub>RMS</sub>. A value of 1
        produces a circular motion. */
    double rho = 1.42;
    if (animation.minZoom || linearZoomInterpolation) {
        double minZoom = util::min(animation.minZoom.value_or(startZoom), startZoom, zoom);
        minZoom = util::clamp(minZoom, state.getMinZoom(), state.getMaxZoom());
        /// w<sub>m</sub>: Maximum visible span, measured in pixels with respect
        /// to the initial scale.
        double wMax = w0 / state.zoomScale(minZoom - startZoom);
        rho = u1 != 0 ? std::sqrt(wMax / u1 * 2) : 1.0;
    }
    /// ρ²
    double rho2 = rho * rho;

    /** rᵢ: Returns the zoom-out factor at one end of the animation.

        @param i 0 for the ascent or 1 for the descent. */
    auto r = [=](double i) {
        /// bᵢ
        double b = (w1 * w1 - w0 * w0 + (i ? -1 : 1) * rho2 * rho2 * u1 * u1) / (2 * (i ? w1 : w0) * rho2 * u1);
        return std::log(std::sqrt(b * b + 1) - b);
    };

    /// r₀: Zoom-out factor during ascent.
    double r0 = u1 != 0 ? r(0) : INFINITY; // Silence division by 0 on sanitize bot.
    double r1 = u1 != 0 ? r(1) : INFINITY;

    // When u₀ = u₁, the optimal path doesn’t require both ascent and descent.
    bool isClose = std::abs(u1) < 0.000001 || !std::isfinite(r0) || !std::isfinite(r1);

    /** w(s): Returns the visible span on the ground, measured in pixels with
        respect to the initial scale.

        Assumes an angular field of view of 2 arctan ½ ≈ 53°. */
    auto w = [=](double s) {
        return (isClose ? std::exp((w1 < w0 ? -1 : 1) * rho * s) : (std::cosh(r0) / std::cosh(r0 + rho * s)));
    };
    /// u(s): Returns the distance along the flight path as projected onto the
    /// ground plane, measured in pixels from the world image origin at the
    /// initial scale.
    auto u = [=](double s) {
        return (isClose ? 0. : (w0 * (std::cosh(r0) * std::tanh(r0 + rho * s) - std::sinh(r0)) / rho2 / u1));
    };
    /// S: Total length of the flight path, measured in ρ-screenfuls.
    double S = (isClose ? (std::abs(std::log(w1 / w0)) / rho) : ((r1 - r0) / rho));

    Duration duration;
    if (animation.duration) {
        duration = *animation.duration;
    } else {
        /// V: Average velocity, measured in ρ-screenfuls per second.
        double velocity = 1.2;
        if (animation.velocity) {
            velocity = *animation.velocity / rho;
        }
        duration = std::chrono::duration_cast<Duration>(std::chrono::duration<double>(S / velocity));
    }
    if (duration == Duration::zero()) {
        // Perform an instantaneous transition.
        jumpTo(camera);
        if (animation.transitionFinishFn) {
            animation.transitionFinishFn();
        }
        return;
    }

    const double startScale = state.getScale();
    state.setProperties(
        TransformStateProperties().withPanningInProgress(true).withScalingInProgress(true).withRotatingInProgress(
            bearing != startBearing));
    const EdgeInsets startEdgeInsets = state.getEdgeInsets();

    startTransition(
        camera,
        animation,
        [=, this](double k) {
            /// s: The distance traveled along the flight path, measured in
            /// ρ-screenfuls.
            double s = k * S;
            double us = k == 1.0 ? 1.0 : u(s);

            // Calculate the current point and zoom level along the flight path.
            Point<double> framePoint = util::interpolate(startPoint, endPoint, us);
            double frameZoom = linearZoomInterpolation ? util::interpolate(startZoom, zoom, k)
                                                       : startZoom + state.scaleZoom(1 / w(s));

            // Zoom can be NaN if size is empty.
            if (std::isnan(frameZoom)) {
                frameZoom = zoom;
            }

            // Convert to geographic coordinates and set the new viewpoint.
            LatLng frameLatLng = Projection::unproject(framePoint, startScale);
            state.setLatLngZoom(frameLatLng, frameZoom);
            state.setCenterAltitude(util::interpolate(startCenterAlt, centerAlt, us));
            if (bearing != startBearing) {
                state.setBearing(util::wrap(util::interpolate(startBearing, bearing, k), -pi, pi));
            }

            if (padding != startEdgeInsets) {
                // Interpolate edge insets
                state.setEdgeInsets({util::interpolate(startEdgeInsets.top(), padding.top(), k),
                                     util::interpolate(startEdgeInsets.left(), padding.left(), k),
                                     util::interpolate(startEdgeInsets.bottom(), padding.bottom(), k),
                                     util::interpolate(startEdgeInsets.right(), padding.right(), k)});
            }

            if (pitch != startPitch) {
                state.setPitch(util::interpolate(startPitch, pitch, k));
            }
            if (roll != startRoll) {
                state.setPitch(util::interpolate(startRoll, roll, k));
            }
            if (fov != startFov) {
                state.setFieldOfView(util::interpolate(startFov, fov, k));
            }
            renormalizeCenterAltitudeToTerrain(); // Isomaps : zoom AGL-vrai (vitesses de gestes correctes)
            clampEyeAboveTerrain(); // Isomaps : œil >= sol + minAGL, après tous les setters de la frame
        },
        duration);
}

// MARK: - Position

void Transform::moveBy(const ScreenCoordinate& offset, const AnimationOptions& animation) {
    ScreenCoordinate centerOffset = {offset.x, offset.y};
    // Isomaps : l'ancien amortissement ×0,6 à ≤300 m/sol est RETIRÉ. Il compensait la dé-projection sur le
    // plan mer qui amplifiait le pan (~5× en montagne : point visé 5× plus loin que le terrain du centre).
    // Depuis que screenCoordinateToTileCoordinate intersecte le plan du centre, la vitesse est écran-vraie
    // (le doigt suit le terrain, façon Mapbox) — l'amortissement par-dessus la rendait très trop lente.

    // Reduce the offset so that it never goes past the horizon. If it goes past
    // the horizon, the pan direction is opposite of the intended direction.
    const double pitch = state.getPitch();
    const double offsetLength = std::hypot(offset.x, offset.y);
    if (pitch > 0.0 && offsetLength > 0.0) {
        const double cameraToCenter = 0.5 * static_cast<double>(state.getSize().height) /
                                      std::tan(state.getFieldOfView() / 2.0);
        const double pixelsToHorizon = std::abs(cameraToCenter / std::tan(pitch));
        constexpr double horizonFactor = 0.75; // must be < 1 to keep the offset short of the horizon
        const double scale = pixelsToHorizon * horizonFactor / offsetLength;
        if (scale < 1.0) {
            centerOffset = {offset.x * scale, offset.y * scale};
        }
    }

    ScreenCoordinate pointOnScreen = state.getEdgeInsets().getCenter(state.getSize().width, state.getSize().height) -
                                     centerOffset;
    // Use unwrapped LatLng to carry information about moveBy direction.
    easeTo(CameraOptions().withCenter(screenCoordinateToLatLng(pointOnScreen, LatLng::Unwrapped)), animation);
}

LatLng Transform::getLatLng(LatLng::WrapMode wrap) const {
    return state.getLatLng(wrap);
}

// MARK: - Zoom

double Transform::getZoom() const {
    return state.getZoom();
}

// MARK: - Bounds

void Transform::setLatLngBounds(LatLngBounds bounds) {
    if (!bounds.valid()) {
        throw std::runtime_error("failed to set bounds: bounds are invalid");
    }
    state.setLatLngBounds(bounds);
}

void Transform::setMinZoom(const double minZoom) {
    if (std::isnan(minZoom)) return;
    state.setMinZoom(minZoom);
}

void Transform::setMaxZoom(const double maxZoom) {
    if (std::isnan(maxZoom)) return;
    state.setMaxZoom(maxZoom);
}

void Transform::setMinPitch(const double minPitch) {
    if (std::isnan(minPitch)) return;
    if (util::deg2rad(minPitch) < util::PITCH_MIN) {
        Log::Warning(Event::General,
                     "Trying to set minimum pitch below the limit (" + std::to_string(util::rad2deg(util::PITCH_MIN)) +
                         " degrees), the value will be clamped.");
    }
    state.setMinPitch(util::deg2rad(minPitch));
}

void Transform::setMaxPitch(const double maxPitch) {
    if (std::isnan(maxPitch)) return;
    if (util::deg2rad(maxPitch) > util::PITCH_MAX) {
        Log::Warning(Event::General,
                     "Trying to set maximum pitch above the limit (" + std::to_string(util::rad2deg(util::PITCH_MAX)) +
                         " degrees), the value will be clamped.");
    }
    state.setMaxPitch(util::deg2rad(maxPitch));
}

void Transform::setFrustumOffset(const EdgeInsets& frustumOffset) {
    state.setFrustumOffset(frustumOffset);
}

EdgeInsets Transform::getFrustumOffset() {
    return state.getFrustumOffset();
}

// MARK: - Bearing

void Transform::rotateBy(const ScreenCoordinate& first,
                         const ScreenCoordinate& second,
                         const AnimationOptions& animation) {
    ScreenCoordinate center = state.getEdgeInsets().getCenter(state.getSize().width, state.getSize().height);
    const ScreenCoordinate offset = first - center;
    const double distance = std::sqrt(std::pow(2, offset.x) + std::pow(2, offset.y));

    // If the first click was too close to the center, move the center of
    // rotation by 200 pixels in the direction of the click.
    if (distance < 200) {
        const double heightOffset = -200;
        const double rotateBearing = std::atan2(offset.y, offset.x);
        center.x = first.x + std::cos(rotateBearing) * heightOffset;
        center.y = first.y + std::sin(rotateBearing) * heightOffset;
    }

    const double bearing = -util::rad2deg(state.getBearing() + util::angle_between(first - center, second - center));
    easeTo(CameraOptions().withBearing(bearing), animation);
}

double Transform::getBearing() const {
    return state.getBearing();
}

// MARK: - Pitch

double Transform::getPitch() const {
    return state.getPitch();
}

double Transform::getRoll() const {
    return state.getRoll();
}

double Transform::getFieldOfView() const {
    return state.getFieldOfView();
}

// MARK: - North Orientation

void Transform::setNorthOrientation(NorthOrientation orientation) {
    state.setNorthOrientation(orientation);
    double scale{state.getScale()};
    double x{state.getX()};
    double y{state.getY()};
    double z{state.getZ()};
    state.constrain(scale, x, y);
    state.setProperties(TransformStateProperties().withScale(scale).withX(x).withY(y).withZ(z));
}

NorthOrientation Transform::getNorthOrientation() const {
    return state.getNorthOrientation();
}

// MARK: - Constrain mode

void Transform::setConstrainMode(mbgl::ConstrainMode mode) {
    state.setConstrainMode(mode);
    double scale{state.getScale()};
    double x{state.getX()};
    double y{state.getY()};
    double z{state.getZ()};
    state.constrain(scale, x, y);
    state.setProperties(TransformStateProperties().withScale(scale).withX(x).withY(y).withZ(z));
}

ConstrainMode Transform::getConstrainMode() const {
    return state.getConstrainMode();
}

// MARK: - Viewport mode

void Transform::setViewportMode(mbgl::ViewportMode mode) {
    state.setViewportMode(mode);
}

ViewportMode Transform::getViewportMode() const {
    return state.getViewportMode();
}

// MARK: - Projection mode

void Transform::setProjectionMode(const ProjectionMode& options) {
    state.setProperties(TransformStateProperties()
                            .withAxonometric(options.axonometric.value_or(state.getAxonometric()))
                            .withXSkew(options.xSkew.value_or(state.getXSkew()))
                            .withYSkew(options.ySkew.value_or(state.getYSkew())));
}

ProjectionMode Transform::getProjectionMode() const {
    return ProjectionMode()
        .withAxonometric(state.getAxonometric())
        .withXSkew(state.getXSkew())
        .withYSkew(state.getYSkew());
}

// MARK: - Transition

void Transform::startTransition(const CameraOptions& camera,
                                const AnimationOptions& animation,
                                const std::function<void(double)>& frame,
                                const Duration& duration) {
    if (transitionFinishFn) {
        transitionFinishFn();
    }

    bool isAnimated = duration != Duration::zero();
    observer.onCameraWillChange(isAnimated ? MapObserver::CameraChangeMode::Animated
                                           : MapObserver::CameraChangeMode::Immediate);

    // Associate the anchor, if given, with a coordinate.
    // Anchor and center points are mutually exclusive, with preference for the
    // center point when both are set.
    std::optional<ScreenCoordinate> anchor = camera.center ? std::nullopt : camera.anchor;
    LatLng anchorLatLng;
    if (anchor) {
        anchor->y = state.getSize().height - anchor->y;
        anchorLatLng = state.screenCoordinateToLatLng(*anchor);
    }

    transitionStart = Clock::now();
    transitionDuration = duration;

    transitionFrameFn = [isAnimated, animation, frame, anchor, anchorLatLng, this](const TimePoint now) {
        float t = isAnimated ? (std::chrono::duration<float>(now - transitionStart) / transitionDuration) : 1.0f;
        if (t >= 1.0) {
            frame(1.0);
        } else {
            util::UnitBezier ease = animation.easing ? *animation.easing : util::DEFAULT_TRANSITION_EASE;
            frame(ease.solve(t, 0.001));
        }

        if (anchor) state.moveLatLng(anchorLatLng, *anchor);

        // At t = 1.0, a DidChangeAnimated notification should be sent from finish().
        if (t < 1.0) {
            if (animation.transitionFrameFn) {
                animation.transitionFrameFn(t);
            }
            observer.onCameraIsChanging();
            return false;
        } else {
            // Indicate that we need to terminate this transition
            return true;
        }
    };

    transitionFinishFn = [isAnimated, animation, this] {
        state.setProperties(
            TransformStateProperties().withPanningInProgress(false).withScalingInProgress(false).withRotatingInProgress(
                false));
        if (animation.transitionFinishFn) {
            animation.transitionFinishFn();
        }
        observer.onCameraDidChange(isAnimated ? MapObserver::CameraChangeMode::Animated
                                              : MapObserver::CameraChangeMode::Immediate);
    };

    if (!isAnimated) {
        auto update = std::move(transitionFrameFn);
        auto finish = std::move(transitionFinishFn);

        transitionFrameFn = nullptr;
        transitionFinishFn = nullptr;

        update(Clock::now());
        finish();
    }
}

bool Transform::inTransition() const {
    return transitionFrameFn != nullptr;
}

void Transform::updateTransitions(const TimePoint& now) {
    // Use a temporary function to ensure that the transitionFrameFn lambda is
    // called only once per update.

    // This addresses the symptoms of
    // https://github.com/mapbox/mapbox-gl-native/issues/11180 where setting a
    // shape source to nil (or similar) in the `onCameraIsChanging` observer
    // function causes `Map::Impl::onUpdate()` to be called which in turn calls
    // this function (before the current iteration has completed), leading to an
    // infinite loop. See https://github.com/mapbox/mapbox-gl-native/issues/5833
    // for a similar, related, issue.
    //
    // By temporarily nulling the `transitionFrameFn` (and then restoring it
    // after the temporary has been called) we stop this recursion.
    //
    // It's important to note that the scope of this change is stop the above
    // crashes. It doesn't address any potential deeper issue (for example
    // user error, how often and when transition callbacks are called).

    auto transition = std::move(transitionFrameFn);
    transitionFrameFn = nullptr;

    if (transition && transition(now)) {
        // If the transition indicates that it is complete, then we should call
        // the finish lambda (going via a temporary as above)
        auto finish = std::move(transitionFinishFn);

        transitionFinishFn = nullptr;
        transitionFrameFn = nullptr;

        if (finish) {
            finish();
        }
    } else if (!transitionFrameFn) {
        // We have to check `transitionFrameFn` is nil here, since a new
        // transition may have been triggered in a user callback (from the
        // transition call above)
        transitionFrameFn = std::move(transition);
    }
}

void Transform::cancelTransitions() {
    if (transitionFinishFn) {
        transitionFinishFn();
    }

    transitionFrameFn = nullptr;
    transitionFinishFn = nullptr;
}

void Transform::setGestureInProgress(bool inProgress) {
    state.setGestureInProgress(inProgress);
}

// MARK: Conversion and projection

ScreenCoordinate Transform::latLngToScreenCoordinate(const LatLng& latLng) const {
    // Isomaps : projeter AU SOL (surface rendue, exagération comprise) et non à z=0 (niveau de la mer).
    // À pitch 0 la différence est invisible (projection verticale) ; dès que pitch > 0, la projection
    // z=0 glisse le long du rayon → annotation qui se déplace avec la caméra et « flotte en l'air »
    // (mesuré : rond bleu de position utilisateur). Sans terrain (fn absente ou point sans DEM) : z=0,
    // comportement historique.
    double groundAltitude = 0.0;
    if (terrainCollisionElevationFn) {
        if (const auto e = terrainCollisionElevationFn(latLng)) {
            groundAltitude = *e;
        }
    }
    vec4 p;
    ScreenCoordinate point = state.latLngToScreenCoordinate(latLng, p, groundAltitude);
    // Point DERRIÈRE le plan caméra (w <= 0) : la division perspective retourne la projection, qui
    // retombe n'importe où dans l'écran (mesuré : rond bleu de position flottant en plein ciel quand
    // l'utilisateur est hors champ). Pas de coordonnée écran valide → loin hors écran, la vue se masque.
    if (p[3] <= 0.0) {
        return {-1e9, -1e9};
    }
    point.y = state.getSize().height - point.y;
    return point;
}

LatLng Transform::screenCoordinateToLatLng(const ScreenCoordinate& point, LatLng::WrapMode wrapMode) const {
    ScreenCoordinate flippedPoint = point;
    flippedPoint.y = state.getSize().height - flippedPoint.y;
    return state.screenCoordinateToLatLng(flippedPoint, wrapMode);
}

double Transform::getMaxPitchForEdgeInsets(const EdgeInsets& insets) const {
    double centerOffsetY = 0.5 * (insets.top() - insets.bottom()); // See TransformState::getCenterOffset.
    if (centerOffsetY == 0.0) {
        return state.getMaxPitch();
    }

    const auto height = state.getSize().height;
    assert(height);
    // Half of fov is the field of view above perspective center.
    const double tangentOfFovAboveCenterAngle = (0.5 + centerOffsetY / height) * 2.0 * tan(getFieldOfView() / 2.0);
    const double fovAboveCenter = std::atan(tangentOfFovAboveCenterAngle);
    return state.getMaxPitch() + getFieldOfView() / 2.0 - fovAboveCenter;
}

FreeCameraOptions Transform::getFreeCameraOptions() const {
    return state.getFreeCameraOptions();
}

void Transform::setFreeCameraOptions(const FreeCameraOptions& options) {
    cancelTransitions();
    state.setFreeCameraOptions(options);
    clampEyeAboveTerrain();
}

// MARK: - Isomaps : collision caméra / terrain

void Transform::setTerrainCameraCollision(std::function<std::optional<double>(const LatLng&)> elevationFn,
                                          double minMetersAboveGround) {
    terrainCollisionElevationFn = std::move(elevationFn);
    terrainCollisionMinAGL = minMetersAboveGround;
}

void Transform::renormalizeCenterAltitudeToTerrain() {
    if (!terrainCollisionElevationFn || !state.valid() || isGestureInProgress()) {
        return; // jamais pendant un geste : la baseline du geste (zoom absolu du pinch) sauterait
    }
    const auto loc = state.getFreeCameraOptions().getLocation();
    if (!loc) {
        return;
    }
    const double eyeAlt = loc->altitude;
    const LatLng c = state.getLatLng();
    const double h0 = state.getCenterAltitude();
    // Isomaps : cible = PREMIÈRE INTERSECTION RAYON-TERRAIN (ce que l'utilisateur regarde), par marche
    // le long du rayon de visée. L'ancien « terrain à l'aplomb du centre » était une itération de point
    // fixe h←ground(centre(h)) : quand le rayon traverse une VALLÉE puis frappe un MASSIF, elle a DEUX
    // attracteurs et fait du ping-pong (mesuré : zoom 13,2 ↔ 15,7, centreAlt 1918 ↔ 4240, re-cover
    // complet à chaque flip = clignotement + deux covers superposés entrelacés par la profondeur).
    // La première intersection est unique et stable par construction.
    const LatLng eyeLL = loc->location;
    double hitT = -1.0;
    double grazeT = -1.0;      // crête FRÔLÉE : passage du rayon à < 150 m au-dessus du terrain
    double grazeGap = 150.0;   // plus petit dégagement rencontré (m)
    constexpr int kRaySteps = 96;
    for (int i = 1; i <= kRaySteps; ++i) {
        const double t = 0.05 + (2.5 - 0.05) * (static_cast<double>(i) / kRaySteps);
        const double rayAlt = eyeAlt + (h0 - eyeAlt) * t; // le rayon passe par (centre, h0), œil fixe
        if (rayAlt <= 0.0) {
            break; // le rayon passe sous le niveau de la mer sans toucher : pas d'intersection utile
        }
        const LatLng p(eyeLL.latitude() + (c.latitude() - eyeLL.latitude()) * t,
                       eyeLL.longitude() + (c.longitude() - eyeLL.longitude()) * t);
        const auto g = terrainCollisionElevationFn(p);
        if (!g || *g <= 1.0) {
            continue;
        }
        if (*g >= rayAlt) {
            hitT = t;
            break;
        }
        // Dégagement faible = le rayon FRÔLE une crête. « Viser une crête n'est pas évident » : le rayon
        // passe quelques mètres au-dessus, la première intersection est le terrain DERRIÈRE → toute la
        // sémantique (renorm, zoom, butée) cible derrière → on franchit le col malgré soi (mesuré,
        // refuge du Goûter). Le premier frôlement devient la cible : zoomer amène À la crête, la butée
        // de contact et le cliquet s'y accrochent ; franchir = relâcher et re-pincer.
        const double gap = rayAlt - *g;
        if (gap < grazeGap && grazeT < 0.0) {
            grazeGap = gap;
            grazeT = t;
        }
    }
    if (grazeT > 0.0 && (hitT < 0.0 || grazeT < hitT)) {
        hitT = grazeT;
    }
    if (hitT < 0.0) {
        return; // pas d'intersection trouvée (ciel/terrain inconnu) → paramétrisation courante conservée
    }
    // Cible = ALTITUDE DU RAYON au paramètre d'impact — PAS l'altitude du sol touché. En visée quasi
    // horizontale (crête à hauteur d'œil mais à des kilomètres), « même altitude » ≠ « même endroit » :
    // viser l'altitude du sol téléportait le centre à quelques mètres de l'œil (mesuré : crête 2 m sous
    // l'œil à 3 km → zoom 22, échelle « 2 m », LOD dégénéré). Avec l'altitude du rayon à t_hit, le
    // glissement homothétique (s = e1/e0 = t_hit) place le centre PILE au point d'impact, œil
    // strictement fixe.
    const double h1ray = eyeAlt + (h0 - eyeAlt) * hitT;
    // Impact trop proche de l'œil (en 3D) : ne pas dégénérer — relever l'œil est le rôle de
    // l'anti-collision, pas du zoom. Distance 3D ≈ t_hit × |œil→centre|.
    const double cosLatR = std::cos(eyeLL.latitude() * M_PI / 180.0);
    const double dxm = (c.longitude() - eyeLL.longitude()) * 111320.0 * cosLatR;
    const double dym = (c.latitude() - eyeLL.latitude()) * 111320.0;
    const double L0 = std::sqrt(dxm * dxm + dym * dym + (eyeAlt - h0) * (eyeAlt - h0));
    if (hitT * L0 < 150.0) {
        return;
    }
    const std::optional<double> ground = h1ray;
    // Convergence : SAUT INSTANTANÉ pour les grands écarts — la reparamétrisation préserve la vue (œil
    // fixe), donc aucun à-coup visuel possible. Lisser les grands deltas (ex. lancement : sol 0 → 2400 m
    // pendant que le DEM s'affine) faisait BALAYER le zoom (14.8→17.1 sur plusieurs secondes) → chaque
    // zoom intermédiaire déclenchait un cover complet → des centaines de tuiles chargées pour rien.
    // Le lissage 20 % ne reste que pour le bruit fin (< 30 m) pour ne pas trembler en balade.
    const double delta = *ground - h0;
    const double alpha = (std::abs(delta) > 30.0) ? 1.0 : 0.2;
    const double h1 = std::min(h0 + delta * alpha, eyeAlt - 5.0);
    const double e0 = eyeAlt - h0;
    const double e1 = eyeAlt - h1;
    if (e0 <= 1.0 || e1 <= 1.0 || std::abs(h1 - h0) < 0.5) {
        return;
    }
    // Reparamétrisation PURE : l'œil et la direction de visée restent STRICTEMENT fixes. Le centre glisse
    // LE LONG DU RAYON à la nouvelle altitude (homothétie de facteur e1/e0 depuis l'œil, car l'offset
    // horizontal œil→centre est ∝ (œil − centre) vertical à visée fixe), et le zoom compense (∝ 2^−zoom).
    const double zoomNew = state.getZoom() + std::log2(e0 / e1);
    if (zoomNew < state.getMinZoom() || zoomNew > state.getMaxZoom()) {
        return; // un clamp déplacerait l'œil → on ne renormalise pas dans les bornes extrêmes
    }
    // ZONE MORTE : en panotant sur du relief accidenté, le rayon central balaie crête/vallée et la cible
    // saute de ±1000 m à chaque micro-geste → sans seuil, CHAQUE relâché reparamétrait le zoom de ±0,5-1
    // → re-cover complet de la pyramide (mesuré : maillage 57↔177, satellite en churn, « dès que je bouge
    // un micro, ça se dégrade »). On ne reparamètre qu'au-delà de 0,5 zoom d'écart (AGL faux d'au plus
    // ×1,4 entre deux resynchronisations — imperceptible sur la vitesse des gestes) ; les grands écarts
    // (survol d'une falaise, lancement) passent toujours en saut instantané.
    if (std::abs(zoomNew - state.getZoom()) < 0.5) {
        return;
    }
    // GARDE : aucune surface terrestre au-dessus de ~10 000 m (Everest × exagération). Une cible
    // fantôme fige toute la paramétrisation AGL — zoom calé sur un plateau imaginaire à 12 km, pan
    // « doigt collé » à 20-40 m/s (mesuré : centreAlt 11 867, œil catapulté à 12 142). On TRACE (pour
    // identifier le chemin fautif) et on REFUSE la renormalisation ce tick.
    if (h1 > 10000.0) {
        Log::Warning(Event::Render,
                     "🛑 renorm fantôme h1=" + util::toString(h1) + " œil=" + util::toString(eyeAlt) +
                         " h0=" + util::toString(h0) + " hitT=" + util::toString(hitT) +
                         " h1ray=" + util::toString(h1ray));
        return;
    }
    const double s = e1 / e0;
    const LatLng c1(eyeLL.latitude() + (c.latitude() - eyeLL.latitude()) * s,
                    eyeLL.longitude() + (c.longitude() - eyeLL.longitude()) * s);
    state.setLatLngZoom(c1, zoomNew);
    state.setCenterAltitude(h1);
}

void Transform::clampEyeAboveTerrain() {
    if (!terrainCollisionElevationFn || terrainCollisionMinAGL <= 0.0 || !state.valid()) {
        return;
    }
    // Position 3D de l'ŒIL (pas le centre du regard) : altitude ASL + point-sol sous l'œil.
    const FreeCameraOptions cam = state.getFreeCameraOptions();
    const std::optional<LatLngAltitude> loc = cam.getLocation();
    if (!loc) {
        return;
    }
    // Élévation de référence = MAX du terrain dans un CERCLE de ~200 m autour du centre (le point regardé,
    // toujours dans le frustum chargé). Un seul point se ferait piéger par une falaise/aiguille juste à côté
    // (ex. face au Grandes Jorasses, 1000 m de verticalité) : on échantillonne le centre + 8 points à 200 m
    // et on garde le plus haut (9 lookups DEM = quelques µs, négligeable). Le terrain sous l'œil est ajouté
    // au max s'il est chargé (souvent hors frustum → 0). Le terrain sous l'œil seul n'était pas fiable.
    const LatLng eyeLL = loc->location;
    const LatLng center = state.getLatLng();
    constexpr double kPi = std::numbers::pi;
    const double cosLat = std::cos(eyeLL.latitude() * kPi / 180.0);
    const double mLat = (center.latitude() - eyeLL.latitude()) * 111320.0;
    const double mLng = (center.longitude() - eyeLL.longitude()) * 111320.0 * cosLat;
    const double distEC = std::sqrt(mLat * mLat + mLng * mLng);
    if (collisionElevCache.size() > 200000) {
        collisionElevCache.clear(); // filet mémoire ; l'élévation est fixe → se re-remplit vite
    }
    std::optional<double> refGround;
    const auto accMax = [&refGround](const std::optional<double>& e) {
        if (e && *e > 1.0 && (!refGround || *e > *refGround)) refGround = e; // >1 m : ignore le 0 hors-frustum
    };
    // Échantillon AVEC mémoire : max(requête live, hauteur mémorisée de la cellule ~100 m) ; on mémorise le
    // max. Ainsi un terrain vu récemment (chargé) reste connu quand l'œil passe dessus/derrière (hors frustum).
    const auto sampleElev = [&](const LatLng& p) -> std::optional<double> {
        const int64_t latC = static_cast<int64_t>(std::llround(p.latitude() / 0.001));
        const int64_t lngC = static_cast<int64_t>(std::llround(p.longitude() / 0.0015));
        const int64_t key = latC * 4000000LL + lngC;
        const std::optional<double> live = terrainCollisionElevationFn(p);
        double best = (live && *live > 1.0) ? *live : -1.0;
        const auto it = collisionElevCache.find(key);
        if (it != collisionElevCache.end() && static_cast<double>(it->second) > best) {
            best = static_cast<double>(it->second);
        }
        if (best > 1.0) {
            collisionElevCache[key] = static_cast<float>(best);
            return best;
        }
        return live;
    };
    // 1) REMPLIR la mémoire avec le terrain VISIBLE (centre proche + le long du regard), SANS l'utiliser pour
    // la contrainte : ainsi, quand l'œil arrivera à l'aplomb de ce relief (devenu hors frustum), sa hauteur
    // sera déjà connue. L'inclure directement dans la contrainte ferait monter l'œil AVANT d'y être = bond.
    if (distEC <= 3000.0) {
        (void)sampleElev(center);
    }
    // Isomaps PAROI DEVANT : en plus de remplir la mémoire, détecter où le RAYON DE VISÉE perce le terrain.
    // Zoomer avance l'œil LE LONG de ce rayon : sans cette détection, l'œil finit DANS une paroi visée (le
    // clamp « sous l'œil » ne voit que le fond du couloir, pas la face devant → rebond puis traversée).
    // Champ proche échantillonné FIN (75 m) : la butée à ~minAGL de la paroi exige de mesurer le perçage
    // plus finement que la butée elle-même, sinon l'œil « rampe » à travers (quantification). Loin : 300 m.
    std::optional<double> wallHitDist; // distance horizontale œil→perçage du rayon (m)
    if (distEC > 1.0) {
        const double eyeAltNow = loc->altitude;
        const double centerAltNow = state.getCenterAltitude();
        const double ux = mLng / distEC, uy = mLat / distEC;
        std::optional<double> prevD, prevGap; // probe précédent au-dessus du terrain (pour interpoler)
        const auto probe = [&](double d) {
            const auto e = sampleElev(LatLng(eyeLL.latitude() + uy * d / 111320.0,
                                             eyeLL.longitude() + ux * d / (111320.0 * cosLat)));
            if (wallHitDist || !e) {
                return;
            }
            // Altitude du rayon œil→centre à la distance horizontale d (interpolation linéaire).
            const double rayAlt = eyeAltNow - (eyeAltNow - centerAltNow) * (d / distEC);
            const double gap = rayAlt - (*e + 40.0); // marge : perçage détecté un peu avant l'impact
            if (gap <= 0.0) {
                // Interpolation entre le dernier probe « au-dessus » et celui-ci → point de perçage
                // sub-échantillon (sinon la frontière saute par pas de 75 m = butée qui tremble).
                if (prevGap && *prevGap > 0.0) {
                    const double f = *prevGap / (*prevGap - gap);
                    wallHitDist = *prevD + f * (d - *prevD);
                } else {
                    wallHitDist = d;
                }
            } else {
                prevD = d;
                prevGap = gap;
            }
        };
        for (double d = 75.0; d <= 1200.0 && d <= distEC; d += 75.0) {
            probe(d);
        }
        for (double d = 1500.0; d <= 3000.0 && d <= distEC; d += 300.0) {
            probe(d);
        }
    }
    // 2) CONTRAINTE = terrain DIRECTEMENT SOUS L'ŒIL (« le sol »), via la mémoire (souvent hors frustum).
    // Petit cercle 120 m pour une falaise/aiguille juste à l'aplomb. minEye = sol + 300 → l'œil se FIGE là
    // quand on descend, SANS rebondir : la référence est le sol STABLE sous l'œil, pas le max d'une large
    // zone (qui attrapait un sommet voisin et repoussait l'œil au-dessus de lui = rebond).
    const double dLat = 120.0 / 111320.0;
    const double dLng = 120.0 / (111320.0 * cosLat);
    accMax(sampleElev(eyeLL));
    for (int i = 0; i < 8; ++i) {
        const double a = static_cast<double>(i) * (kPi / 4.0);
        accMax(sampleElev(LatLng(eyeLL.latitude() + dLat * std::sin(a), eyeLL.longitude() + dLng * std::cos(a))));
    }
    // Filet « vue fraîche » : si le sol sous l'œil est encore inconnu (œil derrière le frustum, mémoire pas
    // remplie là), on retombe sur le terrain du CENTRE (chargé) — moins précis mais évite de plonger dans le
    // sol au tout premier zoom avant d'avoir bougé.
    if (!refGround && distEC <= 3000.0) {
        accMax(sampleElev(center));
    }
    // Maintien : monte INSTANTANÉMENT au max sondé. DESCENTE : RAPIDE (35 %/appel) quand le terrain sous
    // l'œil est CONNU (refGround non NUL) — on survole un sol plus bas fiable, il FAUT redescendre vite,
    // sinon l'œil reste bridé à la hauteur d'un sommet franchi (« figé trop haut » → à fort zoom la vue se
    // casse en un coin de terrain + fond magenta). LENTE (2 %/appel) seulement quand refGround est NUL
    // (terrain hors frustum, inconnu) : on garde la sécurité anti-plongée le temps que la sonde retrouve le sol.
    // GARDE (paire de celle de la renormalisation) : aucune surface terrestre > ~10 000 m — un sol de
    // référence fantôme catapulte l'œil (plancher AGL calé sur un plateau imaginaire). Trace + rejet.
    if (refGround && *refGround > 10000.0) {
        Log::Warning(Event::Render, "🛑 collision fantôme refGround=" + util::toString(*refGround));
        refGround = std::nullopt;
    }
    if (refGround && *refGround > collisionRefGroundHold) {
        collisionRefGroundHold = *refGround;
    }
    const double decayTarget = refGround ? *refGround : 0.0;
    const double decayRate = refGround ? 0.35 : 0.02;
    if (collisionRefGroundHold > decayTarget) {
        collisionRefGroundHold -= (collisionRefGroundHold - decayTarget) * decayRate;
    }
    // NB : centerAltitude est désormais piloté par renormalizeCenterAltitudeToTerrain() (= terrain au
    // centre, reparamétrisation pure qui garde l'œil fixe). L'ancienne relaxation vers 0 est retirée :
    // elle combattrait la renormalisation. Le danger historique (magenta) venait d'un centre flotté
    // AU-DESSUS du terrain sans compensation de zoom — la renormalisation le met PILE au terrain avec
    // zoom compensé, donc cover et vue restent couplés.
    if (collisionRefGroundHold < 1.0) {
        return; // aucun terrain vu récemment → pas de contrainte
    }
    const double minEyeAlt = collisionRefGroundHold + terrainCollisionMinAGL;
    // BRIDAGE DU ZOOM (remplace le float de centerAltitude). L'altitude de l'œil au-dessus du centre est
    // ∝ 2^−zoom (le facteur cos(pitch) est commun et s'annule) : réduire le zoom LÈVE l'œil ET élargit la vue
    // de façon COHÉRENTE — zoom honnête → tuiles correctes → cover ET dessin corrects → zéro magenta. Quand
    // l'œil passe sous sol+minAGL, on plafonne le zoom à la valeur qui remet l'œil pile à sol+minAGL : le zoom
    // « bute » au lieu de figer en montant. Idempotent (si déjà au-dessus, no-op ; borné min/maxZoom).
    const double centerAlt = state.getCenterAltitude();
    const double eyeAboveCenter = loc->altitude - centerAlt;
    double wantAboveCenter = minEyeAlt - centerAlt;
    // Isomaps BUTÉE PAROI : contrainte en 3D LE LONG DU RAYON DE VISÉE — PAS en horizontal vers un centre au
    // niveau de la mer. L'ancienne version horizontale « perçait » le SOL sous l'œil à pitch faible → ratio >1
    // à chaque frame → recul composé exponentiel (éjection à 33 km, flou pendant l'inertie). En 3D : à pitch 0
    // le perçage = le sol sous l'œil à distance AGL (loin le long du rayon) → aucun misfire ; face à une paroi,
    // le perçage est proche → butée. L'œil s'arrête à minAGL du perçage. Zone morte 0,5 % : on ne corrige que
    // les violations franches (pas de micro-corrections = pas de tremblement), le sous-l'œil absolu assure la
    // récupération si un geste a dépassé.
    if (wallHitDist && eyeAboveCenter > 1.0 && distEC > 1.0) {
        const double rayLen = std::sqrt(distEC * distEC + eyeAboveCenter * eyeAboveCenter); // œil→centre en 3D
        const double hit3D = *wallHitDist * (rayLen / distEC); // distance œil→perçage LE LONG du rayon
        const double ratio = std::min(1.25, (rayLen - hit3D + terrainCollisionMinAGL) / rayLen);
        if (ratio > 1.005) {
            wantAboveCenter = std::max(wantAboveCenter, eyeAboveCenter * ratio);
        }
    }
    // Isomaps BUTÉE DE CONTACT : ANGLE MORT de la sonde de paroi (elle démarre à 75 m → les derniers mètres
    // d'un pinch vers une face passaient sous tous les radars : le « sol sous l'œil » ne voit que la vallée
    // en contrebas — mesuré : œil à 2 m du terrain du centre, zoom épinglé à 22, LOD dégénéré, « je
    // n'arrive plus à zoomer »). La distance 3D œil→TERRAIN DU CENTRE est connue exactement (centerAltitude
    // = impact du raycast de renormalisation) : sous minAGL, recul proportionnel (même mécanique bornée
    // ×1,25/tick que la butée paroi — fige sans rebond).
    if (eyeAboveCenter > 0.5 && centerAlt > 1.0) {
        const double rayLen3D = std::sqrt(distEC * distEC + eyeAboveCenter * eyeAboveCenter);
        if (rayLen3D > 0.1 && rayLen3D < terrainCollisionMinAGL) {
            const double ratio = std::min(1.25, terrainCollisionMinAGL / rayLen3D);
            if (ratio > 1.005) {
                wantAboveCenter = std::max(wantAboveCenter, eyeAboveCenter * ratio);
            }
        }
    }
    // Isomaps : pendant un geste qui AUGMENTE le pitch, une violation se résout en BORNANT LE PITCH
    // (centre et zoom fixes, réduire le pitch relève l'œil sur son arc) : le geste « bute » contre le
    // relief au lieu d'un bras de fer dézoom-contre-geste à chaque frame (constaté : bascule au-dessus
    // d'un versant → −3,5 niveaux de zoom en ~2 s, « je me retrouve à 6 000 m », et pitch saccadé).
    // Hors geste de pitch (pincement/pan vers une paroi, inertie), le bridage de ZOOM reste tel quel.
    const double pitchNow = state.getPitch();
    const bool gestureActive = isGestureInProgress();
    if (!gestureActive) {
        gesturePitchCeiling = std::numeric_limits<double>::infinity(); // fin de geste : cliquet relâché
        gestureZoomCeiling = std::numeric_limits<double>::infinity();
    } else if (state.getZoom() > gestureZoomCeiling + 1e-6) {
        // CLIQUET DE ZOOM (cf. header) : la butée a déjà figé ce geste — le pinch incrémental ne peut
        // plus avancer, même si le plancher sous-l'œil a soulevé la caméra par-dessus l'obstacle.
        state.setLatLngZoom(state.getLatLng(), gestureZoomCeiling);
        return;
    } else if (pitchNow > gesturePitchCeiling) {
        // CLIQUET : la limite touchée plus tôt dans CE geste tient — fige net, aucun recalcul (le bruit de
        // la sonde ne fait plus trembler le pitch). Réévaluation complète au prochain tick.
        state.setPitch(gesturePitchCeiling);
        lastConstraintPitch = gesturePitchCeiling;
        return;
    }
    const double pitchPrevTick = lastConstraintPitch; // pitch au tick précédent (dernier état SAIN)
    const bool pitchDriven = gestureActive && pitchPrevTick >= 0.0 && (pitchNow - pitchPrevTick) > 1e-4;
    lastConstraintPitch = pitchNow;
    if (eyeAboveCenter > 1.0 && wantAboveCenter > 1.0 && wantAboveCenter > eyeAboveCenter * 1.005) {
        if (pitchDriven && pitchNow > 0.01) {
            // FIGER, borné PAR CONSTRUCTION : retour au pitch du tick précédent (il était sain, et l'écart
            // = l'incrément du geste, minuscule), armé en plafond (cliquet) pour le reste du geste. Toute
            // correction « exacte » calculée (acos) pouvait sauter de dizaines de degrés en un tick, en
            // plein geste → ancres du recognizer recalculées sur une caméra retournée → caméra cassée
            // (constaté : œil à 0 m, vue planétaire). Si la violation persiste au pitch précédent (terrain
            // affiné sous l'œil), le pitch n'augmente plus → tick suivant = voie zoom, petits pas.
            const double hold = std::min(pitchPrevTick, pitchNow);
            state.setPitch(hold);
            lastConstraintPitch = hold;
            gesturePitchCeiling = std::min(gesturePitchCeiling, hold);
            return;
        }
        const double dz = std::log2(eyeAboveCenter / wantAboveCenter); // < 0 → dézoome juste ce qu'il faut
        const double appliedZoom = util::clamp(state.getZoom() + dz, state.getMinZoom(), state.getMaxZoom());
        if (std::abs(appliedZoom - state.getZoom()) > 0.001) {
            state.setLatLngZoom(state.getLatLng(), appliedZoom);
            if (gestureActive) {
                // ARME le cliquet : la butée a corrigé ce geste → le zoom ne repartira pas au-delà tant
                // que les doigts n'ont pas été relâchés (franchir une crête redevient un choix explicite).
                gestureZoomCeiling = std::min(gestureZoomCeiling, appliedZoom);
            }
        }
    }
}

} // namespace mbgl
