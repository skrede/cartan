#include "harness.h"

#include <cartan/urdf/build.h>
#include <cartan/urdf/parser.h>

#include <span>
#include <string>
#include <random>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <system_error>

namespace
{

/// Owns a scratch directory only this process can reach, and removes it when
/// the process ends.
class scratch_directory
{
public:
    scratch_directory()
        : m_path(reserve())
    {
    }

    scratch_directory(const scratch_directory&) = delete;

    scratch_directory& operator=(const scratch_directory&) = delete;

    ~scratch_directory()
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_path, ignored);
    }

    const std::filesystem::path& path() const { return m_path; }

private:
    /// create_directory is a single mkdir and reports whether *this* call made
    /// the directory, so a name it accepts is one no other process holds. That
    /// is what keeps a predictable path in a world-writable temporary directory
    /// from being pre-created as a symlink onto a file worth destroying.
    static std::filesystem::path reserve()
    {
        std::random_device entropy;
        for (int attempt = 0; attempt < 64; ++attempt)
        {
            const std::filesystem::path candidate
                = std::filesystem::temp_directory_path()
                / ("cartan-fuzz-" + std::to_string(entropy()) + std::to_string(entropy()));
            std::error_code failure;
            if (std::filesystem::create_directory(candidate, failure) && !failure)
            {
                std::filesystem::permissions(candidate, std::filesystem::perms::owner_all,
                    std::filesystem::perm_options::replace, failure);
                return candidate;
            }
        }
        cartan::fuzzing::require(false, "a private scratch directory can be reserved");
        return {};
    }

    std::filesystem::path m_path;
};

/// The parser's only entry point takes a filesystem path, so the input has to
/// reach it as a file.
const std::filesystem::path& scratch_document()
{
    static const scratch_directory directory;
    static const std::filesystem::path document = directory.path() / "input.urdf";
    return document;
}

bool write_scratch(std::span<const std::uint8_t> input)
{
    std::ofstream out(scratch_document(), std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(input.data()),
        static_cast<std::streamsize>(input.size()));
    // Closed before the state is read: a flush failure surfaces at close, and a
    // half-written document parsed as though it were whole would be reported
    // against the parser.
    out.close();
    return out.good();
}

}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    if (!write_scratch(std::span<const std::uint8_t>(data, size)))
    {
        return 0;
    }
    auto model = cartan::parse_urdf_file<double>(scratch_document());
    if (!model.has_value())
    {
        return 0;
    }
    auto built = cartan::build_chain<double>(model.value());
    if (!built.has_value())
    {
        return 0;
    }
    cartan::fuzzing::consume(built.value().chain);
    return 0;
}
