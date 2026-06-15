// =============================================================================
//  tests/test_json.cpp -- JSON reader correctness.
// =============================================================================
#include "TestHarness.hpp"
#include "core/Json.hpp"

using namespace fsim;

TEST_CASE(json_scalars_and_nesting) {
    const char* text = R"({
        "name": "test",
        "n": -12.5,
        "exp": 6.02e3,
        "flag": true,
        "nothing": null,
        "nested": { "a": 1, "b": [10, 20, 30] }
    })";
    const JsonValue v = Json::parse(text);
    CHECK(v.isObject());
    CHECK(v["name"].asString() == "test");
    CHECK_NEAR(v["n"].asNumber(), -12.5, 1e-12);
    CHECK_NEAR(v["exp"].asNumber(), 6020.0, 1e-9);
    CHECK(v["flag"].asBool() == true);
    CHECK(v["nested"]["a"].asNumber() == 1.0);
    CHECK(v["nested"]["b"].size() == 3);
    CHECK_NEAR(v["nested"]["b"][2].asNumber(), 30.0, 1e-12);
}

TEST_CASE(json_missing_key_uses_default) {
    const JsonValue v = Json::parse(R"({"a": 1})");
    CHECK(!v.contains("b"));
    CHECK_NEAR(v.get("b").number(99.0), 99.0, 1e-12);  // fallback
    CHECK_NEAR(v.get("a").number(99.0), 1.0,  1e-12);  // present
}

TEST_CASE(json_line_comments_and_strings) {
    const char* text = R"({
        // a leading comment
        "msg": "line\nbreak\ttab",
        "arr": [ ]   // empty array
    })";
    const JsonValue v = Json::parse(text);
    CHECK(v["msg"].asString() == "line\nbreak\ttab");
    CHECK(v["arr"].size() == 0);
}

FSIM_TEST_MAIN()
