#include "map_renderer.hpp"

#include <mbgl/gfx/backend_scope.hpp>
#include <mbgl/renderer/renderer.hpp>
#include <mbgl/util/instrumentation.hpp>
#include <mbgl/util/logging.hpp>
#include <mbgl/util/run_loop.hpp>

#include <string>
#include <android/native_window_jni.h>

#include "attach_env.hpp"

// ─── ISOMAPS WATERMARK (attribution incrustée dans le rendu — NON RETIRABLE) ───
// Le logo Isomaps est embarqué dans le binaire (données RGBA compilées) et dessiné
// à chaque frame après le rendu de la carte, en bas à droite. Aucune API publique
// ne permet de le masquer ; le retirer exige de modifier ET recompiler le SDK.
//
// COMPILÉ UNIQUEMENT pour le build SDK vendu (flag ISOMAPS_SDK_WATERMARK, cf. CMakeLists
// + gradle -Pisomaps.watermark=true). Build par défaut / app Isomaps → aucun watermark.
#if defined(ISOMAPS_SDK_WATERMARK) && MLN_RENDER_BACKEND_OPENGL
#include <GLES3/gl3.h>
#include <algorithm>
#include "isomaps_watermark_data.h"
namespace {
GLuint gWmProg = 0, gWmTex = 0, gWmVao = 0, gWmVbo = 0;
GLint gWmRect = -1, gWmTexU = -1;
GLuint isomapsWmShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type); glShaderSource(s, 1, &src, nullptr); glCompileShader(s); return s;
}
void isomapsWatermarkEnsure() {
    if (gWmProg && glIsProgram(gWmProg)) return;               // déjà prêt (contexte valide)
    const char* vs = "#version 300 es\nlayout(location=0) in vec2 a;\nout vec2 v;\nuniform vec4 r;\n"
                     "void main(){ v=vec2(a.x,1.0-a.y); vec2 p=r.xy+a*r.zw; gl_Position=vec4(p*2.0-1.0,0.0,1.0); }";
    const char* fs = "#version 300 es\nprecision mediump float;\nin vec2 v;out vec4 o;uniform sampler2D t;\n"
                     "void main(){ o=texture(t,v); }";
    gWmProg = glCreateProgram();
    glAttachShader(gWmProg, isomapsWmShader(GL_VERTEX_SHADER, vs));
    glAttachShader(gWmProg, isomapsWmShader(GL_FRAGMENT_SHADER, fs));
    glLinkProgram(gWmProg);
    gWmRect = glGetUniformLocation(gWmProg, "r");
    gWmTexU = glGetUniformLocation(gWmProg, "t");
    glGenTextures(1, &gWmTex); glBindTexture(GL_TEXTURE_2D, gWmTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, kIsomapsWatermarkW, kIsomapsWatermarkH, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, kIsomapsWatermarkRGBA);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    const float quad[8] = {0, 0, 1, 0, 0, 1, 1, 1};            // TRIANGLE_STRIP unit quad
    glGenVertexArrays(1, &gWmVao); glBindVertexArray(gWmVao);
    glGenBuffers(1, &gWmVbo); glBindBuffer(GL_ARRAY_BUFFER, gWmVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glBindVertexArray(0);
}
void isomapsWatermarkDraw() {
    GLint vp[4]; glGetIntegerv(GL_VIEWPORT, vp);
    const float fbW = (float)vp[2], fbH = (float)vp[3];
    if (fbW < 2 || fbH < 2) return;
    isomapsWatermarkEnsure();

    // ── SAUVEGARDE de tout l'état GL qu'on va toucher ────────────────────────
    // MapLibre a un Context qui CACHE l'état GL et saute les appels qu'il croit
    // déjà posés. Si on laisse l'état modifié, la frame suivante déraille (rendu
    // corrompu). On restaure donc EXACTEMENT l'état d'avant → le cache reste valide.
    GLint sProg = 0, sVao = 0, sArrayBuf = 0, sActiveTex = 0, sTex0 = 0;
    GLint sBlendSrcRGB = 0, sBlendDstRGB = 0, sBlendSrcA = 0, sBlendDstA = 0;
    GLboolean sBlend = glIsEnabled(GL_BLEND);
    GLboolean sDepthTest = glIsEnabled(GL_DEPTH_TEST);
    GLboolean sStencil = glIsEnabled(GL_STENCIL_TEST);
    GLboolean sCull = glIsEnabled(GL_CULL_FACE);
    GLboolean sDepthMask = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &sDepthMask);
    glGetIntegerv(GL_CURRENT_PROGRAM, &sProg);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &sVao);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &sArrayBuf);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &sActiveTex);
    glGetIntegerv(GL_BLEND_SRC_RGB, &sBlendSrcRGB);
    glGetIntegerv(GL_BLEND_DST_RGB, &sBlendDstRGB);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &sBlendSrcA);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &sBlendDstA);
    glActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &sTex0);

    const float wpx = std::min(std::max(fbW * 0.22f, 90.0f), 340.0f);   // ~22% largeur, borné
    const float hpx = wpx * (float)kIsomapsWatermarkH / (float)kIsomapsWatermarkW;
    const float m = fbW * 0.03f;                                        // marge
    const float rx = (fbW - wpx - m) / fbW, ry = m / fbH, rw = wpx / fbW, rh = hpx / fbH; // bas-droite

    // ── DESSIN ───────────────────────────────────────────────────────────────
    glUseProgram(gWmProg);
    glDisable(GL_DEPTH_TEST); glDepthMask(GL_FALSE); glDisable(GL_STENCIL_TEST); glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindTexture(GL_TEXTURE_2D, gWmTex); glUniform1i(gWmTexU, 0);   // unité déjà GL_TEXTURE0
    glUniform4f(gWmRect, rx, ry, rw, rh);
    glBindVertexArray(gWmVao); glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // ── RESTAURATION à l'identique ───────────────────────────────────────────
    glBindVertexArray((GLuint)sVao);
    glBindBuffer(GL_ARRAY_BUFFER, (GLuint)sArrayBuf);
    glBindTexture(GL_TEXTURE_2D, (GLuint)sTex0);            // sur GL_TEXTURE0
    glActiveTexture((GLenum)sActiveTex);
    glUseProgram((GLuint)sProg);
    glBlendFuncSeparate((GLenum)sBlendSrcRGB, (GLenum)sBlendDstRGB,
                        (GLenum)sBlendSrcA, (GLenum)sBlendDstA);
    if (sBlend) glEnable(GL_BLEND); else glDisable(GL_BLEND);
    if (sDepthTest) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (sStencil) glEnable(GL_STENCIL_TEST); else glDisable(GL_STENCIL_TEST);
    if (sCull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    glDepthMask(sDepthMask);
}
} // namespace
#endif
#include "android_renderer_backend.hpp"
#include "map_renderer_runnable.hpp"

#if MLN_RENDER_BACKEND_OPENGL
#include <sys/system_properties.h>

namespace {
std::string androidSysProp(const char* key) {
    assert(strlen(key) < PROP_NAME_MAX);
    if (__system_property_find(key) == nullptr) {
        return "";
    }
    char prop[PROP_VALUE_MAX + 1];
    __system_property_get(key, prop);
    return prop;
}

bool inEmulator() {
    return androidSysProp("ro.kernel.qemu") == "1" || androidSysProp("ro.boot.qemu") == "1" ||
           androidSysProp("ro.hardware.egl") == "emulation";
}
} // namespace
#endif

namespace mbgl {
namespace android {

MapRenderer::MapRenderer(jni::JNIEnv& _env,
                         const jni::Object<MapRenderer>& obj,
                         jni::jfloat pixelRatio_,
                         const jni::String& localIdeographFontFamily_)
    : javaPeer(_env, obj),
      pixelRatio(pixelRatio_),
      localIdeographFontFamily(localIdeographFontFamily_ ? jni::Make<std::string>(_env, localIdeographFontFamily_)
                                                         : std::optional<std::string>{}),
      threadPool(Scheduler::GetBackground(), {}),
      mailboxData(this) {}

MapRenderer::MailboxData::MailboxData(Scheduler* scheduler_)
    : scheduler(scheduler_) {
    assert(scheduler);
}

std::shared_ptr<Mailbox> MapRenderer::MailboxData::getMailbox() const noexcept {
    if (!mailbox) {
        mailbox = std::make_shared<Mailbox>(*scheduler);
    }
    return mailbox;
}

MapRenderer::~MapRenderer() = default;

void MapRenderer::reset() {
    try {
        destroyed = true;

        if (renderer) {
            // Make sure to destroy the renderer on the GL Thread
            auto self = ActorRef<MapRenderer>(*this, mailboxData.getMailbox());
            self.ask(&MapRenderer::resetRenderer).wait();
        }

        // Lock to make sure there is no concurrent initialisation on the gl thread
        std::scoped_lock lock(initialisationMutex);
        rendererObserver.reset();
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::reset failed: ") + exception.what());
    }
}

ActorRef<Renderer> MapRenderer::actor() const {
    return *rendererRef;
}

std::optional<double> MapRenderer::isomapsQueryTerrainElevation(const mbgl::LatLng& latLng) const {
    // Appelée depuis le thread de la Map (collision caméra du Transform). Pas d'actor : la requête ne
    // lit qu'un instantané immuable publié sous mutex par le thread de rendu — un aller-retour bloquant
    // vers le thread GL par échantillon (le rayon œil→centre est sondé tous les 75 m) gèlerait l'UI.
    std::shared_lock<std::shared_mutex> lock(rendererLifecycleMutex);
    if (!renderer) {
        return std::nullopt;
    }
    return renderer->queryTerrainElevationCrossThread(latLng);
}

void MapRenderer::schedule(std::function<void()>&& scheduled) {
    MLN_TRACE_FUNC();
    try {
        // Create a runnable
        android::UniqueEnv _env = android::AttachEnv();
        auto runnable = std::make_unique<MapRendererRunnable>(*_env, std::move(scheduled));

        // Obtain ownership of the peer (gets transferred to the MapRenderer on the JVM for later GC)
        auto peer = runnable->peer();

        // Queue the event on the Java Peer
        static auto& javaClass = jni::Class<MapRenderer>::Singleton(*_env);
        static auto queueEvent = javaClass.GetMethod<void(jni::Object<MapRendererRunnable>)>(*_env, "queueEvent");
        auto weakReference = javaPeer.get(*_env);
        if (weakReference) {
            MLN_TRACE_ZONE(java);
            weakReference.Call(*_env, queueEvent, peer);
        }

        // Release the c++ peer as it will be destroyed on GC of the Java Peer
        runnable.release();
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::schedule failed: ") + exception.what());
    }
}

void MapRenderer::waitForEmpty([[maybe_unused]] const util::SimpleIdentity tag) {
    try {
        android::UniqueEnv _env = android::AttachEnv();
        static auto& javaClass = jni::Class<MapRenderer>::Singleton(*_env);
        static auto waitForEmpty = javaClass.GetMethod<void()>(*_env, "waitForEmpty");
        if (auto weakReference = javaPeer.get(*_env)) {
            return weakReference.Call(*_env, waitForEmpty);
        }
    } catch (...) {
        Log::Error(Event::Android, "MapRenderer::waitForEmpty failed");
        jni::ThrowJavaError(*android::AttachEnv(), std::current_exception());
    }
}

void MapRenderer::requestRender() {
    try {
        android::UniqueEnv _env = android::AttachEnv();
        static auto& javaClass = jni::Class<MapRenderer>::Singleton(*_env);
        static auto onInvalidate = javaClass.GetMethod<void()>(*_env, "requestRender");
        auto weakReference = javaPeer.get(*_env);
        if (weakReference) {
            weakReference.Call(*_env, onInvalidate);
        }
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::requestRender failed: ") + exception.what());
    }
}

void MapRenderer::requestRender(JNIEnv& env, jni::Local<jni::Object<MapRenderer>>& ref) {
    try {
        static auto& javaClass = jni::Class<MapRenderer>::Singleton(env);
        static auto onInvalidate = javaClass.GetMethod<void()>(env, "requestRender");
        if (ref) {
            ref.Call(env, onInvalidate);
        }
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::requestRender failed: ") + exception.what());
    }
}

void MapRenderer::update(std::shared_ptr<UpdateParameters> params) {
    try {
        // Lock on the parameters
        std::scoped_lock lock(updateMutex);
        updateParameters = std::move(params);
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::update failed: ") + exception.what());
    }
}

void MapRenderer::setObserver(std::shared_ptr<RendererObserver> _rendererObserver) {
    try {
        // Lock as the initialization can come from the main thread or the GL thread first
        std::scoped_lock lock(initialisationMutex);

        rendererObserver = std::move(_rendererObserver);

        // Set the new observer on the Renderer implementation
        if (renderer) {
            renderer->setObserver(rendererObserver.get());
        }
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::setObserver failed: ") + exception.what());
    }
}

void MapRenderer::requestSnapshot(SnapshotCallback callback) {
    auto self = ActorRef<MapRenderer>(*this, mailboxData.getMailbox());
    self.invoke(
        &MapRenderer::scheduleSnapshot,
        std::make_unique<SnapshotCallback>(
            [&, callback = std::move(callback), runloop = util::RunLoop::Get()](PremultipliedImage image) {
                runloop->invoke(
                    [callback = std::move(callback), image = std::move(image), renderer = std::move(this)]() mutable {
                        if (renderer && !renderer->destroyed) {
                            callback(std::move(image));
                        }
                    });
                snapshotCallback.reset();
            }));
}

// Called on OpenGL thread //

void MapRenderer::resetRenderer() {
    {
        // Isomaps : exclure les lecteurs cross-thread (isomapsQueryTerrainElevation) pendant la destruction.
        std::unique_lock<std::shared_mutex> lock(rendererLifecycleMutex);
        renderer.reset();
    }

    if (!asyncRendererCleanup) {
        backend.reset();
    }

    window.reset();
    swapBehaviorFlush = false;
}

void MapRenderer::scheduleSnapshot(std::unique_ptr<SnapshotCallback> callback) {
    snapshotCallback = std::move(callback);

    if (backend) {
        gfx::BackendScope backendGuard{backend->getImpl()};
        backend->enableFramebufferRead(true);
    }

    requestRender();
}

void MapRenderer::render(JNIEnv&) {
    assert(renderer);

    std::shared_ptr<UpdateParameters> params;
    {
        // Lock on the parameters
        std::unique_lock<std::mutex> lock(updateMutex);
        if (!updateParameters) return;

        // Hold on to the update parameters during render
        params = updateParameters;
    }

    // Activate the backend
    assert(backend);
    gfx::BackendScope backendGuard{backend->getImpl()};

    // Ensure that the "current" scheduler on the render thread is
    // this scheduler.
    Scheduler::SetCurrent(this);

    if (framebufferSizeChanged) {
        backend->updateViewPort();
        framebufferSizeChanged = false;
    }

    renderer->render(params);

#if defined(ISOMAPS_SDK_WATERMARK) && MLN_RENDER_BACKEND_OPENGL
    isomapsWatermarkDraw();   // logo Isomaps incrusté (build SDK vendu uniquement)
#endif

    // Deliver the snapshot if requested
    if (snapshotCallback) {
        snapshotCallback->operator()(backend->readFramebuffer());
        snapshotCallback.reset();
    }
}

void MapRenderer::onSurfaceCreated(JNIEnv& env, const jni::Object<AndroidSurface>& surface) {
    // Lock as the initialization can come from the main thread or the GL thread first
    std::scoped_lock lock(initialisationMutex);

    UniqueANativeWindow window_ = nullptr;

    if (surface) {
        window_ = std::unique_ptr<ANativeWindow, std::function<void(ANativeWindow*)>>(
            ANativeWindow_fromSurface(&env, reinterpret_cast<jobject>(surface.get())),
            [](ANativeWindow* window_) { ANativeWindow_release(window_); });
    }

    if (backend) {
        if (backend->createSurface(window_.get())) {
            window = std::move(window_);
            return;
        }

        // The android system will have already destroyed the underlying
        // GL resources if this is not the first initialization and an
        // attempt to clean them up will fail
        gfx::BackendScope backendGuard{backend->getImpl()};
        backend->markContextLost();
    }

    if (renderer) {
        renderer->markContextLost();
    }

    // Reset in opposite order
    {
        // Isomaps : exclure les lecteurs cross-thread (isomapsQueryTerrainElevation) pendant l'échange.
        std::unique_lock<std::shared_mutex> lock(rendererLifecycleMutex);
        renderer.reset();
    }
    backend.reset();
    window = std::move(window_);

    // Create the new backend and renderer
    backend = AndroidRendererBackend::Create(window.get());
    {
        std::unique_lock<std::shared_mutex> lock(rendererLifecycleMutex);
        renderer = std::make_unique<Renderer>(backend->getImpl(), pixelRatio, localIdeographFontFamily);
    }
    rendererRef = std::make_unique<ActorRef<Renderer>>(*renderer, mailboxData.getMailbox());

#if MLN_RENDER_BACKEND_OPENGL
    // If we're running the emulator with the OpenGL backend, we're going to crash eventually,
    // unless we enable this mitigation.
    if (inEmulator()) {
        renderer->enableAndroidEmulatorGoldfishMitigation(true);
    }
#endif

    backend->setSwapBehavior(swapBehaviorFlush ? gfx::Renderable::SwapBehaviour::Flush
                                               : gfx::Renderable::SwapBehaviour::NoFlush);

    // Set the observer on the new Renderer implementation
    if (rendererObserver) {
        renderer->setObserver(rendererObserver.get());
    }
}

void MapRenderer::onSurfaceChanged([[maybe_unused]] JNIEnv& env, jint width, jint height) {
#if MLN_RENDER_BACKEND_OPENGL
    if (!renderer) {
        // In case the surface has been destroyed (due to app back-grounding)
        jni::jobject* nullObj = nullptr;
        onSurfaceCreated(env, jni::Object<AndroidSurface>(nullObj));
    }
#endif

    backend->resizeFramebuffer(width, height);
    framebufferSizeChanged = true;
    requestRender();
}

void MapRenderer::onRendererReset(JNIEnv&) {
    // Make sure to destroy the renderer on the GL Thread
    auto self = ActorRef<MapRenderer>(*this, mailboxData.getMailbox());
    self.ask(&MapRenderer::resetRenderer).wait();
}

// needs to be called on GL thread
void MapRenderer::onSurfaceDestroyed(JNIEnv&) {
    if (backend) {
        backend->destroySurface();
    }

    window.reset();
}

void MapRenderer::setSwapBehaviorFlush(JNIEnv&, jboolean flush) {
    swapBehaviorFlush = flush;

    if (backend) {
        backend->setSwapBehavior(flush ? gfx::Renderable::SwapBehaviour::Flush
                                       : gfx::Renderable::SwapBehaviour::NoFlush);
    }
}

// Static methods //

void MapRenderer::registerNative(jni::JNIEnv& env) {
    // Lookup the class
    static auto& javaClass = jni::Class<MapRenderer>::Singleton(env);

#define METHOD(MethodPtr, name) jni::MakeNativePeerMethod<decltype(MethodPtr), (MethodPtr)>(name)

    // Register the peer
    jni::RegisterNativePeer<MapRenderer>(
        env,
        javaClass,
        "nativePtr",
        jni::MakePeer<MapRenderer, const jni::Object<MapRenderer>&, jni::jfloat, const jni::String&>,
        "nativeInitialize",
        "finalize",
        METHOD(&MapRenderer::render, "nativeRender"),
        METHOD(&MapRenderer::onRendererReset, "nativeReset"),
        METHOD(&MapRenderer::onSurfaceCreated, "nativeOnSurfaceCreated"),
        METHOD(&MapRenderer::onSurfaceChanged, "nativeOnSurfaceChanged"),
        METHOD(&MapRenderer::onSurfaceDestroyed, "nativeOnSurfaceDestroyed"),
        METHOD(&MapRenderer::setSwapBehaviorFlush, "nativeSetSwapBehaviorFlush"));
}

MapRenderer& MapRenderer::getNativePeer(JNIEnv& env, const jni::Object<MapRenderer>& jObject) {
    static auto& javaClass = jni::Class<MapRenderer>::Singleton(env);
    static auto field = javaClass.GetField<jlong>(env, "nativePtr");
    MapRenderer* mapRenderer = reinterpret_cast<MapRenderer*>(jObject.Get(env, field));
    assert(mapRenderer != nullptr);
    return *mapRenderer;
}

} // namespace android
} // namespace mbgl
