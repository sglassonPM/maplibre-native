#pragma once

#include <mbgl/annotation/annotation_manager.hpp>
#include <mbgl/map/map.hpp>
#include <mbgl/map/map_observer.hpp>
#include <mbgl/map/map_options.hpp>
#include <mbgl/map/mode.hpp>
#include <mbgl/map/transform.hpp>
#include <mbgl/renderer/renderer_frontend.hpp>
#include <mbgl/renderer/renderer_observer.hpp>
#include <mbgl/style/observer.hpp>
#include <mbgl/style/source.hpp>
#include <mbgl/style/style.hpp>
#include <mbgl/util/size.hpp>
#include <mbgl/tile/tile_operation.hpp>

#include <numbers>

namespace mbgl {

class FileSource;
class ResourceTransform;

namespace gfx {
class ShaderRegistry;
} // namespace gfx

namespace util {
class ActionJournal;
} // namespace util

struct StillImageRequest {
    StillImageRequest(Map::StillImageCallback&& callback_)
        : callback(std::move(callback_)) {}

    Map::StillImageCallback callback;
};

class Map::Impl final : public TransformObserver, public style::Observer, public RendererObserver {
public:
    Impl(RendererFrontend&, MapObserver&, std::shared_ptr<FileSource>, const MapOptions&);
    ~Impl() final;

    // TransformObserver
    void onCameraWillChange(MapObserver::CameraChangeMode) final;
    void onCameraIsChanging() final;
    void onCameraDidChange(MapObserver::CameraChangeMode) final;

    // StyleObserver
    void onSourceChanged(style::Source&) final;
    void onUpdate() final;
    void onStyleLoading() final;
    void onStyleLoaded() final;
    void onStyleError(std::exception_ptr) final;
    void onSpriteLoaded(const std::optional<style::Sprite>&) final;
    void onSpriteError(const std::optional<style::Sprite>&, std::exception_ptr) final;
    void onSpriteRequested(const std::optional<style::Sprite>&) final;

    // RendererObserver
    void onInvalidate() final;
    void onResourceError(std::exception_ptr) final;
    void onWillStartRenderingFrame() final;
    void onDidFinishRenderingFrame(RenderMode, bool, bool, const gfx::RenderingStats&) final;
    void onWillStartRenderingMap() final;
    void onDidFinishRenderingMap() final;
    void onStyleImageMissing(const std::string&, const std::function<void()>&) final;
    void onRemoveUnusedStyleImages(const std::vector<std::string>&) final;
    void onRegisterShaders(gfx::ShaderRegistry&) final;

    void onPreCompileShader(shaders::BuiltIn, gfx::Backend::Type, const std::string&) final;
    void onPostCompileShader(shaders::BuiltIn, gfx::Backend::Type, const std::string&) final;
    void onShaderCompileFailed(shaders::BuiltIn, gfx::Backend::Type, const std::string&) final;
    void onGlyphsLoaded(const FontStack&, const GlyphRange&) final;
    void onGlyphsError(const FontStack&, const GlyphRange&, std::exception_ptr) final;
    void onGlyphsRequested(const FontStack&, const GlyphRange&) final;
    void onTileAction(TileOperation op, const OverscaledTileID&, const std::string&) final;
    void onRenderError(std::exception_ptr) final;

    // Map
    void jumpTo(const CameraOptions&);

    bool isRenderingStatsViewEnabled() const;
    void enableRenderingStatsView(bool value);

    MapObserver& observer;
    RendererFrontend& rendererFrontend;
    std::unique_ptr<util::ActionJournal> actionJournal;

    Transform transform;

    const MapMode mode;
    const float pixelRatio;
    const bool crossSourceCollisions;
    const bool fastPFOREnabled;

    MapDebugOptions debugOptions{MapDebugOptions::NoDebug};
    std::unique_ptr<gfx::RenderingStatsView> renderingStatsView;

    std::shared_ptr<FileSource> fileSource;

    std::unique_ptr<style::Style> style;
    AnnotationManager annotationManager;

    bool cameraMutated = false;

    uint8_t prefetchZoomDelta = util::DEFAULT_PREFETCH_ZOOM_DELTA;

    bool loading = false;
    bool rendererFullyLoaded;
    std::unique_ptr<StillImageRequest> stillImageRequest;

    double tileLodMinRadius = 3;
    // Isomaps : 0.4 (au lieu de 1). Le split LOD descend plus profond au premier plan (~z14 a pitch 55
    // au lieu de z13) → plus net, en limitant le nombre de tuiles satellite chargees. 0.2 descendait a
    // z14 mais faisait charger ~250 tuiles z16 (memoire+reseau) → ca ramait. 0.4 = compromis piqué/fluidité.
    double tileLodScale = 0.4;
    // Isomaps : seuil abaisse a 0 -> variable-zoom des qu'on incline, pour TOUTES les sources.
    // Au defaut 60deg, a pitch 45-60 le lointain etait couvert au zoom plein : (1) le terrain
    // explosait en tuiles -> crash memoire, (2) surtout le terrain (force a 0) et le satellite
    // (a 60) DIVERGEAIENT -> tuiles terrain grossieres drapees par des tuiles satellite fines
    // partielles -> trous magenta. A 0, terrain et sources partagent le meme cover variable-zoom
    // -> une tuile satellite par tuile terrain, drapage complet, et cover borne.
    double tileLodPitchThreshold = 0.0;
    double tileLodZoomShift = 0;
    // Isomaps: Distance (et non Default). En Default, le cover à zoom variable est PLAFONNÉ au zoom
    // du centre (maxZoom = z) → à fort pitch le premier plan (proche caméra) reçoit des tuiles au
    // zoom du centre, trop grossières = satellite/relief flous au sol. Distance lève ce plafond
    // (maxZoom = zoomRange.max) → le premier plan descend plus profond = net, pendant que le lointain
    // reste peu profond. Terrain ET sources partagent ce cover (même tileLodMode) → pas de trous.
    TileLodMode tileLodMode = TileLodMode::Distance;
};

// Forward declaration of this method is required for the MapProjection class
CameraOptions cameraForLatLngs(const std::vector<LatLng>& latLngs,
                               const Transform& transform,
                               const EdgeInsets& padding);

} // namespace mbgl
