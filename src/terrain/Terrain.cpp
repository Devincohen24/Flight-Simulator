// =============================================================================
//  terrain/Terrain.cpp -- procedural height field. See Terrain.hpp.
// =============================================================================
#include "terrain/Terrain.hpp"

#include <algorithm>
#include <cmath>

namespace fsim {

double ProceduralTerrain::height(double north, double east) const {
    const double k1 = 2.0 * kPi / p_.wavelength;
    const double k2 = k1 * 2.17; // incommensurate second octave -> non-repeating
    const double k3 = k1 * 4.63; // finer detail

    // Fractal-ish sum of sinusoids; amplitudes fall off with frequency.
    double h = 0.0;
    h += 1.00 * std::sin(k1 * north + 0.0)  * std::cos(k1 * east + 0.0);
    h += 0.45 * std::sin(k2 * north + 1.3)  * std::cos(k2 * east - 0.7);
    h += 0.22 * std::sin(k3 * north - 2.1)  * std::cos(k3 * east + 1.9);
    h *= p_.amplitude;

    // Flatten a disc around the airfield (origin) so there is a runway, blending
    // smoothly out to the rolling terrain.
    const double r = std::sqrt(north * north + east * east);
    double blend = (r - p_.airfieldRadius) / std::max(p_.airfieldFalloff, 1e-6);
    blend = std::clamp(blend, 0.0, 1.0);
    // Smoothstep for a gentle transition.
    blend = blend * blend * (3.0 - 2.0 * blend);

    return p_.baseElevation + blend * h;
}

} // namespace fsim
