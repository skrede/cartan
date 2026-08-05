#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_JSON_WRITER_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_JSON_WRITER_H

/// @file json_writer.h
/// @brief Just enough JSON to write one manifest, escaped per RFC 8259.

#include <format>
#include <string>
#include <vector>
#include <string_view>

namespace cartan::bench
{

inline std::string json_string(std::string_view value)
{
    std::string quoted{'"'};
    for (const char character : value)
    {
        switch (character)
        {
        case '"': quoted += "\\\""; break;
        case '\\': quoted += "\\\\"; break;
        case '\n': quoted += "\\n"; break;
        case '\r': quoted += "\\r"; break;
        case '\t': quoted += "\\t"; break;
        default:
            if (static_cast<unsigned char>(character) < 0x20)
            {
                quoted += std::format("\\u{:04x}", static_cast<unsigned int>(character));
            }
            else
            {
                quoted.push_back(character);
            }
        }
    }
    quoted.push_back('"');
    return quoted;
}

inline std::string json_field(std::string_view key, std::string_view value)
{
    return json_string(key) + ": " + json_string(value);
}

/// A value the program could not read is null rather than an empty string: an
/// empty governor and an unread one are different claims.
inline std::string json_optional(std::string_view key, std::string_view value, bool present)
{
    return json_string(key) + ": " + (present ? json_string(value) : std::string{"null"});
}

inline std::string json_object(const std::vector<std::string>& fields, std::string_view indent)
{
    std::string body{"{"};
    for (std::size_t i = 0; i < fields.size(); ++i)
    {
        body += (i == 0 ? "\n" : ",\n");
        body += indent;
        body += "  ";
        body += fields[i];
    }
    body += fields.empty() ? "}" : "\n" + std::string{indent} + "}";
    return body;
}

inline std::string json_array(const std::vector<std::string>& elements, std::string_view indent)
{
    std::string body{"["};
    for (std::size_t i = 0; i < elements.size(); ++i)
    {
        body += (i == 0 ? "\n" : ",\n");
        body += indent;
        body += "  ";
        body += elements[i];
    }
    body += elements.empty() ? "]" : "\n" + std::string{indent} + "]";
    return body;
}

}

#endif
