// Isomaps — PARTICULES DE VENT Android (GLES3).
//
// Port du calque WebGL du site (resources/js/map/wind-particles.js) et de son
// pendant Metal iOS (WindParticlesLayer.swift). Même algorithme, même contrat de
// données, mêmes réglages serveur — seul le rendu change.
//
// CE QUI EST REPRIS DES DEUX AUTRES, ET POURQUOI
// ---------------------------------------------
// Le déplacement se calcule en PIXELS ÉCRAN, pas en degrés : Mercator est conforme,
// donc convertir un déplacement pixel en déplacement mercator conserve l'angle, et
// une même vitesse garde la même allure à tous les zooms. Mais la taille du monde
// est PLANCHÉRISÉE au zoom 5 : sans ce plancher, à l'échelle planétaire 512 pixels
// couvrent 40 000 km et un vent de 50 km/h traverse l'écran en quelques secondes.
//
// L'advection est sur le CPU, le GPU ne fait que tracer : la simulation GPU
// classique (ping-pong de textures d'état) imposerait ses propres passes au milieu
// de celle de MapLibre, pour quelques milliers de particules que le CPU advecte
// sans effort.
//
// On échantillonne de la zone la plus FINE à la plus grossière : au bord des Alpes
// une particule change de grille au lieu de disparaître.
//
// Aucune constante de vent n'est écrite ici : domaine, encodage, densité et vitesse
// viennent du serveur.

#include <GLES3/gl3.h>
#include <android/log.h>
#include <jni.h>
#include <mbgl/style/layers/custom_layer.hpp>

#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>

namespace {

const char *LOG_TAG = "IsomapsWindLayer";

// --- Champ de vent d'une zone, déjà décodé -------------------------------------
struct Field {
    double ouest, sud, est, nord;
    int largeur, hauteur;
    std::vector<float> u, v;      // km/h
    std::vector<uint8_t> valide;
};

std::vector<Field> gFields;       // du plus FIN au plus GROSSIER
int gCount = 3000;
int gTrail = 10;
double gLifeS = 2.4;
double gSpeedFactor = 2.0;
double gSpeedFull = 70.0;

// Ralentissement des particules à bas zoom.
//
// Une vitesse constante en PIXELS ÉCRAN garde la même allure à tous les zooms —
// bon sur le terrain. Mais à l'échelle planétaire, 512 pixels couvrent 40 000 km
// et un vent de 50 km/h traverse l'écran en quelques secondes.
//
// ⚠️ La première correction plancherisait la taille du monde au zoom 5. Trop
// brutal : à zoom 0 elle divisait la vitesse écran par 32, soit 3 px/s — une
// traînée d'un pixel sur 400 ms, dont les segments deviennent dégénérés et
// disparaissent. « Si je dézoome trop, j'ai plus rien. »
//
// On borne donc le ralentissement au lieu de la vitesse : au-dessus du zoom de
// référence rien ne change, en dessous on ralentit progressivement mais jamais
// sous un quart de la vitesse nominale. Le champ reste lisible partout.
constexpr double kZoomReference = 5.0;
constexpr double kRalentiMin = 0.25;
constexpr double kDtMax = 0.1;                      // retour d'arrière-plan
constexpr double kHistIntervalS = 0.04;             // cadence de la traînée
constexpr float kDemiEpaisseurPx = 1.6f;

// --- Mercator -------------------------------------------------------------------
inline void versMercator(double lon, double lat, double &x, double &y) {
    const double borne = lat < -85.051129 ? -85.051129 : (lat > 85.051129 ? 85.051129 : lat);
    const double s = std::sin(borne * M_PI / 180.0);
    x = (lon + 180.0) / 360.0;
    y = 0.5 - std::log((1 + s) / (1 - s)) / (4 * M_PI);
}

inline void versLonLat(double x, double y, double &lon, double &lat) {
    lon = x * 360.0 - 180.0;
    lat = 90.0 - 360.0 * std::atan(std::exp((y - 0.5) * 2 * M_PI)) / M_PI;
}

// Vent au point, dans la zone la plus fine qui a une valeur ici.
bool echantillonne(double lon, double lat, double &outU, double &outV) {
    for (const Field &f : gFields) {
        const double fx = (lon - f.ouest) / (f.est - f.ouest) * (f.largeur - 1);
        const double fy = (f.nord - lat) / (f.nord - f.sud) * (f.hauteur - 1);
        if (!(fx >= 0 && fy >= 0 && fx <= f.largeur - 1 && fy <= f.hauteur - 1)) continue;

        const int x0 = int(fx), y0 = int(fy);
        const int x1 = x0 + 1 < f.largeur ? x0 + 1 : f.largeur - 1;
        const int y1 = y0 + 1 < f.hauteur ? y0 + 1 : f.hauteur - 1;
        const double tx = fx - x0, ty = fy - y0;
        const int i00 = y0 * f.largeur + x0, i10 = y0 * f.largeur + x1;
        const int i01 = y1 * f.largeur + x0, i11 = y1 * f.largeur + x1;
        // Un seul coin manquant fausse le bord du domaine : on passe à la zone
        // suivante plutôt que d'interpoler avec du vide.
        if (!f.valide[i00] || !f.valide[i10] || !f.valide[i01] || !f.valide[i11]) continue;

        const double w00 = (1 - tx) * (1 - ty), w10 = tx * (1 - ty);
        const double w01 = (1 - tx) * ty,       w11 = tx * ty;
        outU = f.u[i00] * w00 + f.u[i10] * w10 + f.u[i01] * w01 + f.u[i11] * w11;
        outV = f.v[i00] * w00 + f.v[i10] * w10 + f.v[i01] * w01 + f.v[i11] * w11;
        return true;
    }
    return false;
}

// --- Shaders ---------------------------------------------------------------------
// Le ruban est épaissi APRÈS projection : une ligne GL fait un pixel, invisible sur
// un écran dense au-dessus d'une carte claire.
const GLchar *kVertexSource = R"GLSL(#version 300 es
precision highp float;
layout(location = 0) in vec2 a_a;
layout(location = 1) in vec2 a_b;
layout(location = 2) in float a_t;
layout(location = 3) in float a_cote;
layout(location = 4) in float a_alpha;
uniform mat4 u_matrix;
uniform float u_mondePx;
uniform vec2 u_epaisseur;
out float v_alpha;
void main() {
    vec4 pa = u_matrix * vec4(a_a * u_mondePx, 0.0, 1.0);
    vec4 pb = u_matrix * vec4(a_b * u_mondePx, 0.0, 1.0);
    vec2 na = pa.xy / pa.w;
    vec2 nb = pb.xy / pb.w;
    // Direction toujours de a vers b : la prendre depuis chaque extrémité
    // inverserait la perpendiculaire et le quad se croiserait en sablier.
    vec2 dir = nb - na;
    float len = length(dir);
    vec2 perp = len > 1e-6 ? vec2(-dir.y, dir.x) / len : vec2(0.0, 1.0);
    vec2 base = mix(na, nb, a_t);
    gl_Position = vec4(base + perp * u_epaisseur * a_cote, 0.0, 1.0);
    v_alpha = a_alpha;
}
)GLSL";

const GLchar *kFragmentSource = R"GLSL(#version 300 es
precision mediump float;
in float v_alpha;
out vec4 fragColor;
void main() {
    // Alpha prémultiplié : additif là où les traînées se croisent, sans halo gris.
    fragColor = vec4(vec3(1.0) * v_alpha, v_alpha);
}
)GLSL";

GLuint compileShader(GLenum type, const GLchar *source) {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &source, nullptr);
    glCompileShader(sh);
    GLint ok = GL_FALSE;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (ok == GL_FALSE) {
        GLchar log[1024] = {};
        glGetShaderInfoLog(sh, sizeof(log) - 1, nullptr, log);
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "🌬 VENT : compilation shader : %s", log);
        glDeleteShader(sh);
        return 0;
    }
    return sh;
}

struct Sommet {
    float ax, ay, bx, by, t, cote, alpha;
};

class IsomapsWindLayer : mbgl::style::CustomLayerHost {
public:
    ~IsomapsWindLayer() override = default;

    void initialize(const mbgl::style::CustomLayerInitParameters &) override {
        program = glCreateProgram();
        GLuint vs = compileShader(GL_VERTEX_SHADER, kVertexSource);
        GLuint fs = compileShader(GL_FRAGMENT_SHADER, kFragmentSource);
        if (!vs || !fs) return;
        glAttachShader(program, vs);
        glAttachShader(program, fs);
        glLinkProgram(program);
        GLint ok = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &ok);
        if (ok == GL_FALSE) {
            GLchar log[1024] = {};
            glGetProgramInfoLog(program, sizeof(log) - 1, nullptr, log);
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "🌬 VENT : erreur link : %s", log);
            return;
        }
        uMatrix = glGetUniformLocation(program, "u_matrix");
        uMondePx = glGetUniformLocation(program, "u_mondePx");
        uEpaisseur = glGetUniformLocation(program, "u_epaisseur");
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        ready = true;
        __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "🌬 VENT : pipeline prêt");
    }

    void render(const mbgl::style::CustomLayerRenderParameters &params) override {
        if (!ready || gFields.empty()) return;

        const double maintenant = nowSeconds();
        double dt = dernier == 0 ? 1.0 / 60.0 : maintenant - dernier;
        if (dt > kDtMax) dt = kDtMax;
        dernier = maintenant;

        allouer();
        double ouest, est, sud, nord;
        if (!zoneDeNaissance(params, ouest, est, sud, nord)) return;

        const int nb = avancerEtRemplir(dt, params.zoom, ouest, est, sud, nord);
        if (nb <= 0) return;

        glUseProgram(program);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Sommet) * nb, sommets.data(), GL_DYNAMIC_DRAW);
        const GLsizei stride = sizeof(Sommet);
        for (GLuint i = 0; i < 5; ++i) glEnableVertexAttribArray(i);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void *)0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void *)(2 * sizeof(float)));
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, (void *)(4 * sizeof(float)));
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, stride, (void *)(5 * sizeof(float)));
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, stride, (void *)(6 * sizeof(float)));

        // La matrice arrive en column-major sur 16 doubles : GL_FALSE, pas de transposition.
        float m[16];
        for (int i = 0; i < 16; ++i) m[i] = float(params.projectionMatrix[i]);
        glUniformMatrix4fv(uMatrix, 1, GL_FALSE, m);
        glUniform1f(uMondePx, float(mondePx(params.zoom)));
        glUniform2f(uEpaisseur,
                    kDemiEpaisseurPx / float(params.width) * 2.0f,
                    kDemiEpaisseurPx / float(params.height) * 2.0f);

        // Ces traînées se posent par-dessus la carte : elles n'ont pas de relief.
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glDisable(GL_CULL_FACE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArrays(GL_TRIANGLES, 0, nb);

        glBindVertexArray(0);
        if (!aTrace) {
            aTrace = true;
            __android_log_print(ANDROID_LOG_INFO, LOG_TAG,
                                "🌬 VENT : premier tracé — %d segments, %zu zone(s), zoom %.1f",
                                nb / 6, gFields.size(), params.zoom);
        }
    }

    void contextLost() override { ready = false; }
    void deinitialize() override {
        if (vbo) glDeleteBuffers(1, &vbo);
        if (vao) glDeleteVertexArrays(1, &vao);
        if (program) glDeleteProgram(program);
        ready = false;
    }

private:
    static double nowSeconds() {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + ts.tv_nsec * 1e-9;
    }

    static double mondePx(double zoom) { return 512.0 * std::pow(2.0, zoom); }

    /// Facteur de ralentissement, dans [kRalentiMin, 1].
    static double ralenti(double zoom) {
        if (zoom >= kZoomReference) return 1.0;
        const double f = std::pow(2.0, zoom - kZoomReference);
        return f < kRalentiMin ? kRalentiMin : f;
    }

    static double alea() { return double(std::rand()) / double(RAND_MAX); }

    void allouer() {
        if (int(x.size()) == gCount && trail == gTrail) return;
        trail = gTrail;
        x.assign(gCount, 0); y.assign(gCount, 0);
        age.assign(gCount, 0); vie.assign(gCount, 0); alpha.assign(gCount, 0);
        semee.assign(gCount, 0);
        hist.assign(size_t(gCount) * trail * 2, 0);
        sommets.assign(size_t(gCount) * (trail - 1) * 6, Sommet{});
        tete = 0; histAccum = 0;
    }

    // Intersection de la vue et de l'enveloppe des champs chargés.
    bool zoneDeNaissance(const mbgl::style::CustomLayerRenderParameters &p,
                         double &ouest, double &est, double &sud, double &nord) const {
        double fo = 1e9, fe = -1e9, fs = 1e9, fn = -1e9;
        for (const Field &f : gFields) {
            if (f.ouest < fo) fo = f.ouest;
            if (f.est > fe) fe = f.est;
            if (f.sud < fs) fs = f.sud;
            if (f.nord > fn) fn = f.nord;
        }
        // Étendue visible calculée EN MERCATOR, pas en degrés : la latitude n'y est
        // pas linéaire, et une estimation en degrés fabrique une bande de naissance
        // fausse — d'autant plus fausse qu'on monte en latitude.
        const double m = mondePx(p.zoom);
        double cx, cy;
        versMercator(p.longitude, p.latitude, cx, cy);
        const double demiX = p.width / (2.0 * m);
        const double demiY = p.height / (2.0 * m);
        double lonO, latN, lonE, latS;
        versLonLat(cx - demiX, cy - demiY, lonO, latN);   // y mercator croît vers le SUD
        versLonLat(cx + demiX, cy + demiY, lonE, latS);
        ouest = std::max(lonO, fo);
        est   = std::min(lonE, fe);
        sud   = std::max(latS, fs);
        nord  = std::min(latN, fn);
        return ouest < est && sud < nord;
    }

    void renaitre(int i, double ouest, double est, double sud, double nord) {
        const double lon = ouest + alea() * (est - ouest);
        const double lat = sud + alea() * (nord - sud);
        double mx, my;
        versMercator(lon, lat, mx, my);
        x[i] = mx; y[i] = my; age[i] = 0;
        // Durée de vie dispersée : sans ce jitter, tout un lot disparaît ensemble
        // et l'animation clignote.
        vie[i] = gLifeS * (0.5 + alea());
        semee[i] = 1;
        for (int t = 0; t < trail; ++t) {
            const size_t o = (size_t(t) * gCount + i) * 2;
            hist[o] = mx; hist[o + 1] = my;
        }
    }

    int avancerEtRemplir(double dt, double zoom,
                         double ouest, double est, double sud, double nord) {
        const double k = gSpeedFactor * dt * ralenti(zoom) / mondePx(zoom);
        for (int i = 0; i < gCount; ++i) {
            if (!semee[i]) renaitre(i, ouest, est, sud, nord);
            double lon, lat, u, v;
            versLonLat(x[i], y[i], lon, lat);
            if (!echantillonne(lon, lat, u, v)) { renaitre(i, ouest, est, sud, nord); continue; }
            x[i] += u * k;
            y[i] -= v * k;                     // y mercator croît vers le SUD
            const double vitesse = std::sqrt(u * u + v * v);
            // Un vent nul ne doit pas être invisible, ni aussi marqué qu'une tempête.
            alpha[i] = 0.35 + 0.65 * std::min(1.0, vitesse / gSpeedFull);
            age[i] += dt;
            if (age[i] > vie[i]) { renaitre(i, ouest, est, sud, nord); continue; }
            const size_t o = (size_t(tete) * gCount + i) * 2;
            hist[o] = x[i]; hist[o + 1] = y[i];
        }

        int n = 0;
        for (int seg = 0; seg < trail - 1; ++seg) {
            const int ancien = (tete + 1 + seg) % trail;
            const int recent = (tete + 2 + seg) % trail;
            const float aA = float(seg + 1) / trail;
            const float aB = float(seg + 2) / trail;
            for (int i = 0; i < gCount; ++i) {
                const size_t oa = (size_t(ancien) * gCount + i) * 2;
                const size_t ob = (size_t(recent) * gCount + i) * 2;
                const float xa = float(hist[oa]), ya = float(hist[oa + 1]);
                const float xb = float(hist[ob]), yb = float(hist[ob + 1]);
                // Une particule qui vient de renaître a tout son historique au même
                // point : le segment est vide, on ne l'envoie pas au GPU.
                if (xa == xb && ya == yb) continue;
                if (n + 6 > int(sommets.size())) return n;
                const float al = float(alpha[i]);
                const Sommet s[6] = {
                    {xa, ya, xb, yb, 0.f, -1.f, aA * al},
                    {xa, ya, xb, yb, 0.f,  1.f, aA * al},
                    {xa, ya, xb, yb, 1.f, -1.f, aB * al},
                    {xa, ya, xb, yb, 1.f, -1.f, aB * al},
                    {xa, ya, xb, yb, 0.f,  1.f, aA * al},
                    {xa, ya, xb, yb, 1.f,  1.f, aB * al},
                };
                for (int j = 0; j < 6; ++j) sommets[n++] = s[j];
            }
        }

        histAccum += dt;
        if (histAccum >= kHistIntervalS) {
            histAccum -= kHistIntervalS;
            tete = (tete + 1) % trail;
            for (int i = 0; i < gCount; ++i) {
                const size_t o = (size_t(tete) * gCount + i) * 2;
                hist[o] = x[i]; hist[o + 1] = y[i];
            }
        }
        return n;
    }

    GLuint program = 0, vao = 0, vbo = 0;
    GLint uMatrix = -1, uMondePx = -1, uEpaisseur = -1;
    bool ready = false, aTrace = false;
    double dernier = 0, histAccum = 0;
    int tete = 0, trail = 0;
    std::vector<double> x, y, age, vie, alpha, hist;
    std::vector<uint8_t> semee;
    std::vector<Sommet> sommets;
};

} // namespace

namespace mbgl {
namespace android {
namespace isomaps_wind {

jlong createContext() {
    auto layer = std::make_unique<IsomapsWindLayer>();
    return reinterpret_cast<jlong>(layer.release());
}

void addField(const uint8_t *rgba, int width, int height,
              double ouest, double sud, double est, double nord,
              double offset, double scale) {
    Field f;
    f.ouest = ouest; f.sud = sud; f.est = est; f.nord = nord;
    f.largeur = width; f.hauteur = height;
    const size_t n = size_t(width) * height;
    f.u.resize(n); f.v.resize(n); f.valide.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const uint8_t a = rgba[i * 4 + 3];
        // ⚠️ Un pixel hors domaine a des canaux à zéro, et zéro décodé vaut -153,6
        // km/h : une tempête là où il n'y a pas de donnée. On ne décode que le valide.
        if (a <= 127) { f.valide[i] = 0; f.u[i] = 0; f.v[i] = 0; continue; }
        f.valide[i] = 1;
        f.u[i] = float(rgba[i * 4] * scale + offset);
        f.v[i] = float(rgba[i * 4 + 1] * scale + offset);
    }
    gFields.push_back(std::move(f));
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "🌬 VENT : zone %dx%d reçue (%zu au total)",
                        width, height, gFields.size());
}

void clearFields() { gFields.clear(); }

void setSettings(int count, int trail, double lifeS, double speedFactor, double speedFull) {
    if (count > 0) gCount = count;
    if (trail >= 2) gTrail = trail;
    if (lifeS > 0) gLifeS = lifeS;
    if (speedFactor > 0) gSpeedFactor = speedFactor;
    if (speedFull > 0) gSpeedFull = speedFull;
}

} // namespace isomaps_wind
} // namespace android
} // namespace mbgl
