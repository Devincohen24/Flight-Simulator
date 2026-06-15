// =============================================================================
//  aircraft/Aircraft.cpp -- load an Aircraft from a JSON data file.
//  Every field is optional and falls back to the in-struct default, so data
//  files can specify only what differs from the sensible baseline.
// =============================================================================
#include "aircraft/Aircraft.hpp"
#include "core/Json.hpp"

namespace fsim {

namespace {

// Read a number by key, leaving the destination untouched if the key is absent.
void readNumber(const JsonValue& obj, const char* key, double& dst) {
    const JsonValue& v = obj.get(key);
    if (v.isNumber()) dst = v.asNumber();
}

Vec3 readVec3(const JsonValue& v, const Vec3& fallback) {
    if (v.isArray() && v.size() == 3)
        return Vec3{v[0].asNumber(), v[1].asNumber(), v[2].asNumber()};
    return fallback;
}

} // namespace

Aircraft Aircraft::load(const std::string& path) {
    const JsonValue root = Json::parseFile(path);
    Aircraft ac;

    if (root.get("name").type() == JsonValue::Type::String)
        ac.name = root["name"].asString();

    // --- Mass & inertia ---
    if (root.contains("mass_properties")) {
        const JsonValue& m = root["mass_properties"];
        double mass = ac.mass.mass;
        double Ixx = 1285.0, Iyy = 1825.0, Izz = 2667.0, Ixz = 0.0;
        readNumber(m, "mass", mass);
        readNumber(m, "Ixx", Ixx);
        readNumber(m, "Iyy", Iyy);
        readNumber(m, "Izz", Izz);
        readNumber(m, "Ixz", Ixz);
        ac.mass = MassProperties::make(mass, Mat3::inertia(Ixx, Iyy, Izz, Ixz));
    }

    // --- Wing geometry ---
    if (root.contains("wing")) {
        const JsonValue& w = root["wing"];
        readNumber(w, "area",   ac.wing.area);
        readNumber(w, "span",   ac.wing.span);
        readNumber(w, "chord",  ac.wing.chord);
        readNumber(w, "oswald", ac.wing.oswald);
    }

    // --- Aerodynamic coefficients ---
    if (root.contains("aero")) {
        const JsonValue& a = root["aero"];
        AeroCoefficients& c = ac.aero;
        readNumber(a, "CL0", c.CL0);     readNumber(a, "CLalpha", c.CLalpha);
        readNumber(a, "CLq", c.CLq);     readNumber(a, "CLde", c.CLde);
        readNumber(a, "CLflap", c.CLflap);
        readNumber(a, "CD0", c.CD0);
        readNumber(a, "CYbeta", c.CYbeta); readNumber(a, "CYdr", c.CYdr);
        readNumber(a, "Clbeta", c.Clbeta); readNumber(a, "Clp", c.Clp);
        readNumber(a, "Clr", c.Clr);       readNumber(a, "Clda", c.Clda);
        readNumber(a, "Cldr", c.Cldr);
        readNumber(a, "Cm0", c.Cm0);       readNumber(a, "Cmalpha", c.Cmalpha);
        readNumber(a, "Cmq", c.Cmq);       readNumber(a, "Cmde", c.Cmde);
        readNumber(a, "Cnbeta", c.Cnbeta); readNumber(a, "Cnp", c.Cnp);
        readNumber(a, "Cnr", c.Cnr);       readNumber(a, "Cnda", c.Cnda);
        readNumber(a, "Cndr", c.Cndr);
        readNumber(a, "alpha_stall", c.alphaStall);
        readNumber(a, "stall_blend_rate", c.stallBlendRate);
    }

    // --- Control-surface limits ---
    if (root.contains("control_limits")) {
        const JsonValue& l = root["control_limits"];
        readNumber(l, "elevator_max", ac.limits.elevatorMax);
        readNumber(l, "aileron_max",  ac.limits.aileronMax);
        readNumber(l, "rudder_max",   ac.limits.rudderMax);
        readNumber(l, "flap_max",     ac.limits.flapMax);
    }

    // --- Propulsion ---
    if (root.contains("propulsion")) {
        const JsonValue& p = root["propulsion"];
        readNumber(p, "max_power",       ac.propulsion.maxPower);
        readNumber(p, "prop_diameter",   ac.propulsion.propDiameter);
        readNumber(p, "prop_efficiency", ac.propulsion.propEfficiency);
        readNumber(p, "idle_power_fraction", ac.propulsion.idlePowerFraction);
        readNumber(p, "idle_rpm",        ac.propulsion.idleRpm);
        readNumber(p, "max_rpm",         ac.propulsion.maxRpm);
        readNumber(p, "spool_tau",       ac.propulsion.spoolTau);
        readNumber(p, "bsfc",            ac.propulsion.bsfc);
        readNumber(p, "fuel_capacity",   ac.propulsion.fuelCapacity);
        ac.propulsion.thrustPoint = readVec3(p.get("thrust_point"), ac.propulsion.thrustPoint);
        ac.propulsion.thrustAxis  = readVec3(p.get("thrust_axis"),  ac.propulsion.thrustAxis);
    }

    // --- Landing gear (array of contact points) ---
    if (root.contains("gear")) {
        const JsonValue& g = root["gear"];
        ac.gear.clear();
        for (std::size_t i = 0; i < g.size(); ++i) {
            const JsonValue& u = g[i];
            GearUnit unit;
            unit.position = readVec3(u.get("position"), unit.position);
            readNumber(u, "stiffness",     unit.stiffness);
            readNumber(u, "damping",       unit.damping);
            readNumber(u, "friction_roll", unit.frictionRoll);
            readNumber(u, "friction_side", unit.frictionSide);
            readNumber(u, "friction_brake", unit.frictionBrake);
            if (u.get("braked").type() == JsonValue::Type::Bool)
                unit.braked = u["braked"].asBool();
            if (u.get("name").type() == JsonValue::Type::String)
                unit.name = u["name"].asString();
            ac.gear.push_back(unit);
        }
    }

    return ac;
}

} // namespace fsim
