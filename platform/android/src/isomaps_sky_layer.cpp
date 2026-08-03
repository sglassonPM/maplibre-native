// Isomaps — CALQUE CIEL Android (GLES3) : port du calque Metal de l'app d'éval iOS
// (MBXViewController.mm, IsomapsSkyLayer). Dégradé horizon→zénith, soleil au SUD (~40°), teinte
// plus chaude au sud / plus froide au nord, nuages procéduraux (fbm sur plan d'altitude).
// Rendu plein écran, inséré au-dessus du background du style et sous tout le reste : le terrain
// opaque le recouvre, il ne reste visible que là où rien n'est dessiné (ciel). Sous l'horizon →
// teinte de la BRUME du terrain (même couleur que le fog du shader terrain) : le lointain se fond.
// Pas de test ni d'écriture de profondeur : l'ordre des calques suffit (peintre).

#include <GLES3/gl3.h>
#include <android/log.h>
#include <jni.h>
#include <mbgl/style/layers/custom_layer.hpp>

#include <cmath>
#include <memory>
#include <vector>

namespace {

const char *LOG_TAG = "IsomapsSkyLayer";

const GLchar *kVertexSource = R"GLSL(#version 300 es
out vec2 v_ndc;
void main() {
    // Triangle plein écran (couvre [-1,1]²), sans VBO : gl_VertexID
    vec2 ndc = vec2((gl_VertexID == 1) ? 3.0 : -1.0, (gl_VertexID == 2) ? 3.0 : -1.0);
    v_ndc = ndc;
    // z = PLAN LOINTAIN (ndc 1.0 = clear GL) : avec le test LEQUAL sans écriture, le ciel ne passe
    // que là où RIEN n'a été dessiné (buffer resté au clear), jamais par-dessus le terrain — quel
    // que soit l'ordre de rendu (même astuce que la variante Metal, en convention z GL standard).
    gl_Position = vec4(ndc, 1.0, 1.0);
}
)GLSL";

// Port fidèle du fragment Metal « skyFragment » (variante procédurale).
const GLchar *kFragmentSource = R"GLSL(#version 300 es
precision highp float;
in vec2 v_ndc;
out vec4 fragColor;

uniform float u_pitch;      // rad, 0 = nadir
uniform float u_bearing;    // rad, 0 = nord, horaire
uniform float u_tanHalfFov; // tan(fov vertical / 2)
uniform float u_aspect;     // largeur / hauteur

float shash(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}
float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = shash(i), b = shash(i + vec2(1.0, 0.0)), c = shash(i + vec2(0.0, 1.0)), d = shash(i + vec2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}
float fbm(vec2 p) {
    float v = 0.0, amp = 0.5;
    for (int i = 0; i < 5; i++) { v += amp * vnoise(p); p = p * 2.03 + 17.17; amp *= 0.5; }
    return v;
}

void main() {
    // Rayon monde par pixel. Axes : x = est, y = nord, z = haut. pitch 0 = nadir.
    float sp = sin(u_pitch), cp = cos(u_pitch);
    float sb = sin(u_bearing), cb = cos(u_bearing);
    vec3 fwd = vec3(sb * sp, cb * sp, -cp);
    vec3 up  = vec3(sb * cp, cb * cp,  sp);
    vec3 rt  = vec3(cb, -sb, 0.0);
    vec3 ray = normalize(fwd + v_ndc.x * u_tanHalfFov * u_aspect * rt + v_ndc.y * u_tanHalfFov * up);

    vec3 fogC = vec3(0.72, 0.78, 0.84); // MÊME teinte que la brume du shader terrain
    // « Sudness » : 1 plein sud, 0 plein nord (composante horizontale du rayon)
    float southness = 0.5 + 0.5 * (-ray.y / max(length(ray.xy), 1e-4));
    vec3 horizonC = mix(vec3(0.62, 0.72, 0.88), vec3(0.93, 0.86, 0.72), southness);
    vec3 zenithC  = mix(vec3(0.10, 0.28, 0.62), vec3(0.22, 0.42, 0.78), southness * 0.7);

    if (ray.z <= 0.0) {
        // Sous l'horizon : brume — ne reste visible qu'au-delà du terrain rendu (raccord avec le fog)
        fragColor = vec4(mix(horizonC, fogC, 0.6), 1.0);
        return;
    }

    vec3 sky = mix(horizonC, zenithC, pow(clamp(ray.z, 0.0, 1.0), 0.42));

    // Soleil au SUD (azimut 180°), élévation ~40° : halo chaud + disque
    vec3 sunDir = vec3(0.0, -0.766, 0.643);
    float d = max(dot(ray, sunDir), 0.0);
    sky += vec3(1.0, 0.90, 0.65) * (0.45 * pow(d, 24.0) + 0.18 * pow(d, 4.0));
    sky += vec3(1.5) * smoothstep(0.9997, 0.99995, d);

    // Nuages : fbm projeté sur un plan d'altitude (perspective naturelle vers l'horizon)
    vec2 cuv = ray.xy / max(ray.z, 0.03) * 0.9;
    float f = fbm(cuv + vec2(13.7, 4.2));
    float cov = smoothstep(0.52, 0.74, f);
    cov *= smoothstep(0.02, 0.10, ray.z); // fondu des nuages dans la brume près de l'horizon
    vec3 cloud = mix(vec3(0.78, 0.80, 0.84), vec3(1.0, 0.99, 0.97), smoothstep(0.5, 0.9, f));
    cloud += vec3(0.20, 0.14, 0.06) * pow(d, 3.0); // face côté soleil réchauffée
    sky = mix(sky, cloud, cov * 0.85);

    fragColor = vec4(sky, 1.0);
}
)GLSL";

// Variante IMAGE (port de skyFragmentImage Metal) : panorama équirectangulaire, soleil
// pré-recalé au CENTRE de l'image = plein SUD. u = azimut (sud -> 0,5), v = élévation
// (+90° -> 0). Sous/près de l'horizon : fondu vers la teinte de brume du terrain.
const GLchar *kFragmentImageSource = R"GLSL(#version 300 es
precision highp float;
in vec2 v_ndc;
out vec4 fragColor;

uniform float u_pitch;
uniform float u_bearing;
uniform float u_tanHalfFov;
uniform float u_aspect;
uniform sampler2D u_sky;

void main() {
    float sp = sin(u_pitch), cp = cos(u_pitch);
    float sb = sin(u_bearing), cb = cos(u_bearing);
    vec3 fwd = vec3(sb * sp, cb * sp, -cp);
    vec3 up  = vec3(sb * cp, cb * cp,  sp);
    vec3 rt  = vec3(cb, -sb, 0.0);
    vec3 ray = normalize(fwd + v_ndc.x * u_tanHalfFov * u_aspect * rt + v_ndc.y * u_tanHalfFov * up);

    vec3 fogC = vec3(0.72, 0.78, 0.84); // MÊME teinte que la brume du shader terrain
    float az = atan(ray.x, ray.y);      // 0 = nord, pi = sud
    float uu = fract((az - 3.14159265) / (2.0 * 3.14159265) + 0.5); // plein sud -> u = 0,5 (soleil)
    float vv = clamp(0.5 - asin(clamp(ray.z, -1.0, 1.0)) / 3.14159265, 0.0, 1.0);
    vec3 img = texture(u_sky, vec2(uu, vv)).rgb;
    // Raccord : fondu vers la brume sous/près de l'horizon (le sol de l'image ne sert jamais)
    float sky = smoothstep(-0.01, 0.10, ray.z);
    fragColor = vec4(mix(fogC, img, sky), 1.0);
}
)GLSL";

// Panorama poussé par le Kotlin (BitmapFactory) AVANT la création du contexte — RGBA8.
std::vector<uint8_t> gPanoramaPixels;
int gPanoramaWidth = 0;
int gPanoramaHeight = 0;

GLuint compileShader(GLenum type, const GLchar *source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == GL_FALSE) {
        GLchar log[1024] = {};
        glGetShaderInfoLog(shader, sizeof(log) - 1, nullptr, log);
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "🌤 CIEL : erreur shader : %s", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

class IsomapsSkyLayer : mbgl::style::CustomLayerHost {
public:
    ~IsomapsSkyLayer() override = default;

    void initialize(const mbgl::style::CustomLayerInitParameters &) override {
        // Panorama iOS présent (poussé par le Kotlin) ? -> variante IMAGE, sinon procédural.
        if (!gPanoramaPixels.empty() && gPanoramaWidth > 0 && gPanoramaHeight > 0) {
            glGenTextures(1, &skyTexture);
            glBindTexture(GL_TEXTURE_2D, skyTexture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, gPanoramaWidth, gPanoramaHeight, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, gPanoramaPixels.data());
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);        // u = azimut, boucle
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // v = élévation
            glBindTexture(GL_TEXTURE_2D, 0);
        }
        program = glCreateProgram();
        vertexShader = compileShader(GL_VERTEX_SHADER, kVertexSource);
        fragmentShader = compileShader(GL_FRAGMENT_SHADER, skyTexture ? kFragmentImageSource : kFragmentSource);
        if (!vertexShader || !fragmentShader) {
            return;
        }
        glAttachShader(program, vertexShader);
        glAttachShader(program, fragmentShader);
        glLinkProgram(program);
        GLint ok = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &ok);
        if (ok == GL_FALSE) {
            GLchar log[1024] = {};
            glGetProgramInfoLog(program, sizeof(log) - 1, nullptr, log);
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "🌤 CIEL : erreur link : %s", log);
            return;
        }
        uSky = glGetUniformLocation(program, "u_sky");
        uPitch = glGetUniformLocation(program, "u_pitch");
        uBearing = glGetUniformLocation(program, "u_bearing");
        uTanHalfFov = glGetUniformLocation(program, "u_tanHalfFov");
        uAspect = glGetUniformLocation(program, "u_aspect");
        glGenVertexArrays(1, &vao);
        ready = true;
        __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "🌤 CIEL : pipeline prêt (%s)", skyTexture ? "image" : "procédural");
    }

    void render(const mbgl::style::CustomLayerRenderParameters &params) override {
        if (!ready) {
            return;
        }
        glUseProgram(program);
        // params.bearing est en DEGRÉS compas (0 = nord, horaire) ; pitch/fov en radians (cf.
        // CustomLayerRenderParameters — bearing = rad2deg(-state.getBearing())).
        const float bearingRad = static_cast<float>(params.bearing * M_PI / 180.0);
        glUniform1f(uPitch, static_cast<float>(params.pitch));
        glUniform1f(uBearing, bearingRad);
        glUniform1f(uTanHalfFov, static_cast<float>(std::tan(params.fieldOfView * 0.5)));
        glUniform1f(uAspect, static_cast<float>(params.width / std::max(params.height, 1.0)));
        if (skyTexture) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, skyTexture);
            glUniform1i(uSky, 0);
        }

        // Test de profondeur SANS écriture, ciel au plan lointain : ne peint que le vide (le
        // terrain, rendu avant dans sa propre passe, a déjà écrit sa profondeur).
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glDisable(GL_STENCIL_TEST);
        glDisable(GL_BLEND);
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
    }

    void contextLost() override { ready = false; program = 0; }

    void deinitialize() override {
        if (program) {
            glDetachShader(program, vertexShader);
            glDetachShader(program, fragmentShader);
            glDeleteShader(vertexShader);
            glDeleteShader(fragmentShader);
            glDeleteProgram(program);
            glDeleteVertexArrays(1, &vao);
        }
        if (skyTexture) {
            glDeleteTextures(1, &skyTexture);
            skyTexture = 0;
        }
        ready = false;
    }

private:
    GLuint program = 0;
    GLuint vertexShader = 0;
    GLuint fragmentShader = 0;
    GLuint vao = 0;
    GLuint skyTexture = 0;
    GLint uPitch = -1, uBearing = -1, uTanHalfFov = -1, uAspect = -1, uSky = -1;
    bool ready = false;
};

void JNICALL nativeSetPanorama(JNIEnv *env, jobject, jobject buffer, jint width, jint height) {
    auto *data = static_cast<uint8_t *>(env->GetDirectBufferAddress(buffer));
    const jlong capacity = env->GetDirectBufferCapacity(buffer);
    if (!data || capacity < jlong(width) * height * 4) {
        __android_log_write(ANDROID_LOG_ERROR, LOG_TAG, "🌤 CIEL : panorama invalide (buffer)");
        return;
    }
    gPanoramaPixels.assign(data, data + size_t(width) * height * 4);
    gPanoramaWidth = width;
    gPanoramaHeight = height;
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "🌤 CIEL : panorama %dx%d reçu", width, height);
}

jlong JNICALL nativeCreateContext(JNIEnv *, jobject) {
    auto layer = std::make_unique<IsomapsSkyLayer>();
    return reinterpret_cast<jlong>(layer.release());
}

} // namespace

extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *) {
    JNIEnv *env = nullptr;
    vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);
    jclass cls = env->FindClass("org/maplibre/android/testapp/model/customlayer/IsomapsSkyLayer");
    JNINativeMethod methods[] = {
        {"createContext", "()J", reinterpret_cast<void *>(&nativeCreateContext)},
        {"setPanorama", "(Ljava/nio/ByteBuffer;II)V", reinterpret_cast<void *>(&nativeSetPanorama)}};
    if (env->RegisterNatives(cls, methods, 2) < 0) {
        env->ExceptionDescribe();
        return JNI_ERR;
    }
    return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT void JNICALL JNI_OnUnload(JavaVM *, void *) {}
