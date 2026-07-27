#include "harness.h"

#include <cartan/urdf/build.h>
#include <cartan/urdf/parser.h>

#include <string>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <filesystem>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace
{

int process_id()
{
#if defined(_WIN32)
    return _getpid();
#else
    return static_cast<int>(::getpid());
#endif
}

/// The parser's only entry point takes a filesystem path, so the input has to
/// reach it as a file. The name carries the process identifier so two targets
/// fuzzing concurrently cannot write over each other's document.
const std::filesystem::path& scratch_document()
{
    static const std::filesystem::path path
        = std::filesystem::temp_directory_path()
        / ("cartan-fuzz-urdf-" + std::to_string(process_id()) + ".urdf");
    return path;
}

bool write_scratch(const std::uint8_t* data, std::size_t size)
{
    std::ofstream out(scratch_document(), std::ios::binary | std::ios::trunc);
    if (!out)
    {
        return false;
    }
    out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    return out.good();
}

}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    if (!write_scratch(data, size))
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
