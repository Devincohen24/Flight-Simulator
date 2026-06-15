// =============================================================================
//  core/Json.cpp -- recursive-descent JSON parser. See Json.hpp.
// =============================================================================
#include "core/Json.hpp"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fsim {

namespace {

// Stateful cursor over the input text.
class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    JsonValue parse() {
        skipWs();
        JsonValue v = parseValue();
        skipWs();
        if (pos_ != s_.size())
            fail("trailing characters after JSON value");
        return v;
    }

private:
    const std::string& s_;
    std::size_t pos_{0};

    [[noreturn]] void fail(const std::string& msg) const {
        throw std::runtime_error("JSON parse error at offset "
                                 + std::to_string(pos_) + ": " + msg);
    }

    char peek() const { return pos_ < s_.size() ? s_[pos_] : '\0'; }
    char next() { return s_[pos_++]; }
    bool eof() const { return pos_ >= s_.size(); }

    void skipWs() {
        while (!eof()) {
            const char c = s_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') { ++pos_; continue; }
            // Line comments are not standard JSON but are handy in data files.
            if (c == '/' && pos_ + 1 < s_.size() && s_[pos_ + 1] == '/') {
                pos_ += 2;
                while (!eof() && s_[pos_] != '\n') ++pos_;
                continue;
            }
            break;
        }
    }

    void expect(char c) {
        if (eof() || s_[pos_] != c)
            fail(std::string("expected '") + c + "'");
        ++pos_;
    }

    JsonValue parseValue() {
        skipWs();
        if (eof()) fail("unexpected end of input");
        const char c = peek();
        switch (c) {
            case '{': return parseObject();
            case '[': return parseArray();
            case '"': return JsonValue(parseString());
            case 't': case 'f': return parseBool();
            case 'n': return parseNull();
            default:
                if (c == '-' || std::isdigit(static_cast<unsigned char>(c)))
                    return parseNumber();
                fail(std::string("unexpected character '") + c + "'");
        }
    }

    JsonValue parseObject() {
        expect('{');
        JsonValue v = JsonValue::makeObject();
        skipWs();
        if (peek() == '}') { ++pos_; return v; }
        while (true) {
            skipWs();
            if (peek() != '"') fail("expected string key in object");
            std::string key = parseString();
            skipWs();
            expect(':');
            v.set(key, parseValue());
            skipWs();
            const char c = peek();
            if (c == ',') { ++pos_; continue; }
            if (c == '}') { ++pos_; break; }
            fail("expected ',' or '}' in object");
        }
        return v;
    }

    JsonValue parseArray() {
        expect('[');
        JsonValue v = JsonValue::makeArray();
        skipWs();
        if (peek() == ']') { ++pos_; return v; }
        while (true) {
            v.push_back(parseValue());
            skipWs();
            const char c = peek();
            if (c == ',') { ++pos_; continue; }
            if (c == ']') { ++pos_; break; }
            fail("expected ',' or ']' in array");
        }
        return v;
    }

    std::string parseString() {
        expect('"');
        std::string out;
        while (true) {
            if (eof()) fail("unterminated string");
            char c = next();
            if (c == '"') break;
            if (c == '\\') {
                if (eof()) fail("unterminated escape");
                char e = next();
                switch (e) {
                    case '"':  out += '"';  break;
                    case '\\': out += '\\'; break;
                    case '/':  out += '/';  break;
                    case 'b':  out += '\b'; break;
                    case 'f':  out += '\f'; break;
                    case 'n':  out += '\n'; break;
                    case 'r':  out += '\r'; break;
                    case 't':  out += '\t'; break;
                    case 'u': {
                        // Minimal BMP handling: decode \uXXXX to UTF-8.
                        if (pos_ + 4 > s_.size()) fail("bad \\u escape");
                        unsigned code = 0;
                        for (int i = 0; i < 4; ++i) {
                            char h = next();
                            code <<= 4;
                            if (h >= '0' && h <= '9') code |= unsigned(h - '0');
                            else if (h >= 'a' && h <= 'f') code |= unsigned(h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') code |= unsigned(h - 'A' + 10);
                            else fail("bad hex digit in \\u escape");
                        }
                        if (code < 0x80) {
                            out += char(code);
                        } else if (code < 0x800) {
                            out += char(0xC0 | (code >> 6));
                            out += char(0x80 | (code & 0x3F));
                        } else {
                            out += char(0xE0 | (code >> 12));
                            out += char(0x80 | ((code >> 6) & 0x3F));
                            out += char(0x80 | (code & 0x3F));
                        }
                        break;
                    }
                    default: fail("invalid escape character");
                }
            } else {
                out += c;
            }
        }
        return out;
    }

    JsonValue parseNumber() {
        const std::size_t start = pos_;
        if (peek() == '-') ++pos_;
        while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        if (peek() == '.') {
            ++pos_;
            while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        }
        if (peek() == 'e' || peek() == 'E') {
            ++pos_;
            if (peek() == '+' || peek() == '-') ++pos_;
            while (!eof() && std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        }
        const std::string num = s_.substr(start, pos_ - start);
        return JsonValue(std::strtod(num.c_str(), nullptr));
    }

    JsonValue parseBool() {
        if (s_.compare(pos_, 4, "true") == 0)  { pos_ += 4; return JsonValue(true); }
        if (s_.compare(pos_, 5, "false") == 0) { pos_ += 5; return JsonValue(false); }
        fail("invalid literal");
    }

    JsonValue parseNull() {
        if (s_.compare(pos_, 4, "null") == 0) { pos_ += 4; return JsonValue(); }
        fail("invalid literal");
    }
};

} // namespace

JsonValue Json::parse(const std::string& text) {
    return Parser(text).parse();
}

JsonValue Json::parseFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("JSON: cannot open file '" + path + "'");
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse(ss.str());
}

} // namespace fsim
