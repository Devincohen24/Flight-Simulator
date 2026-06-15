// =============================================================================
//  core/Json.hpp
//
//  A small, dependency-free JSON reader sufficient for loading aircraft data
//  files (objects, arrays, numbers, strings, booleans, null, with nesting and
//  full escape handling). It is deliberately a *reader* only -- the simulator
//  loads data, it does not serialise it. Keeping this in-tree avoids pulling a
//  third-party library into the headless physics core.
//
//  Usage:
//      JsonValue root = Json::parseFile("data/aircraft/cessna172.json");
//      double S = root["aero"]["wing_area"].asNumber();
// =============================================================================
#pragma once

#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace fsim {

class JsonValue {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    JsonValue() : type_(Type::Null) {}
    explicit JsonValue(bool b) : type_(Type::Bool), bool_(b) {}
    explicit JsonValue(double n) : type_(Type::Number), num_(n) {}
    explicit JsonValue(std::string s) : type_(Type::String), str_(std::move(s)) {}

    Type type() const { return type_; }
    bool isObject() const { return type_ == Type::Object; }
    bool isArray()  const { return type_ == Type::Array; }
    bool isNumber() const { return type_ == Type::Number; }

    // Scalar accessors (throw on type mismatch so config errors surface early).
    double asNumber() const {
        if (type_ != Type::Number) throw std::runtime_error("JSON: value is not a number");
        return num_;
    }
    bool asBool() const {
        if (type_ != Type::Bool) throw std::runtime_error("JSON: value is not a bool");
        return bool_;
    }
    const std::string& asString() const {
        if (type_ != Type::String) throw std::runtime_error("JSON: value is not a string");
        return str_;
    }

    // Number with a default if the key is absent (convenience for optional
    // coefficients that default to zero).
    double number(double fallback = 0.0) const {
        return type_ == Type::Number ? num_ : fallback;
    }

    // Object access.
    bool contains(const std::string& key) const {
        return type_ == Type::Object && object_.count(key) > 0;
    }
    const JsonValue& operator[](const std::string& key) const {
        static const JsonValue kNull;
        if (type_ != Type::Object) throw std::runtime_error("JSON: value is not an object");
        auto it = object_.find(key);
        if (it == object_.end())
            throw std::runtime_error("JSON: missing key '" + key + "'");
        return it->second;
    }
    // Object access returning Null (not throwing) when absent.
    const JsonValue& get(const std::string& key) const {
        static const JsonValue kNull;
        if (type_ == Type::Object) {
            auto it = object_.find(key);
            if (it != object_.end()) return it->second;
        }
        return kNull;
    }

    // Array access.
    std::size_t size() const { return type_ == Type::Array ? array_.size() : 0; }
    const JsonValue& operator[](std::size_t i) const {
        if (type_ != Type::Array) throw std::runtime_error("JSON: value is not an array");
        return array_.at(i);
    }

    // --- Builders used by the parser ---
    static JsonValue makeObject() { JsonValue v; v.type_ = Type::Object; return v; }
    static JsonValue makeArray()  { JsonValue v; v.type_ = Type::Array;  return v; }
    void set(const std::string& key, JsonValue v) { object_[key] = std::move(v); }
    void push_back(JsonValue v) { array_.push_back(std::move(v)); }

private:
    Type type_;
    bool bool_{false};
    double num_{0.0};
    std::string str_;
    std::vector<JsonValue> array_;
    std::map<std::string, JsonValue> object_;
};

class Json {
public:
    static JsonValue parse(const std::string& text);
    static JsonValue parseFile(const std::string& path);
};

} // namespace fsim
