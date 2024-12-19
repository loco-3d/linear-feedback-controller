#pragma once

#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>

namespace tests::utils {

/**
 *  @brief Safely close any std::FILE ptr
 *
 *  Architecture Decision (AD): We use a lambda here because of a caveat of
 *  using std::unique_ptr<> with custom deleter. When using a function, the
 *  unique_ptr store the function ptr alongside the ptr to watch for, therefore
 *  doubling its size...
 *
 *  See https://godbolt.org/z/vh8zE4bYo
 *
 *  @param[in] file The file ptr we wish to close
 */
constexpr auto FileSafelyClose = [](std::FILE* const file) noexcept -> void {
  if (file) std::fclose(file);
};

/// Alias use to represents safe file handler (using RAII) returned by FileOpen
using FilePtr = std::unique_ptr<std::FILE, decltype(FileSafelyClose)>;

/**
 *  @brief Wrapper around std::fopen in order to create a unique_ptr
 *
 *  @param[in] name Name of the file, see std::fopen
 *  @param[in] mode File access flags (r, w, r+, w+, ...), see std::fopen
 *
 *  @return std::unique_ptr<> of an std::FILE with deleter to close the file
 *                            automatically
 */
inline auto FileOpen(const char* name, char const* const mode) noexcept
    -> FilePtr {
  return {
      std::fopen(name, mode),
      FileSafelyClose,
  };
}

/**
 *  @brief Dump the file content into a std::string
 *
 *  @warning Reading ends until EOF or if any file operation error happens,
 *           those must be checked by hand afterwards (std::ferror/std::feof)
 *
 *  @param[in] file File we wish to read from
 *  @param[in] chunks Size of the chunks read from the file
 *                    (default to 1024 bytes)
 *
 *  @return std::string The file content store within a string
 */
inline auto FileToString(std::FILE& file,
                         const std::size_t chunks = 1024) noexcept
    -> std::string {
  std::string output(chunks, '\0');

  std::size_t read = 0;
  while ((read = std::fread(output.data() + (output.size() - chunks), 1, chunks,
                            std::addressof(file))) == chunks) {
    output.resize(output.size() + chunks, '\0');
  }

  // Remove the extra stuff
  // This is mandatory otherwise the size won't match the number of byte read
  output.resize(output.size() - (chunks - read));

  return output;
}

/**
 *  @brief Dump the file located at \a path into a std::string
 *
 *  @param[in] path File path we wish to read from
 *
 *  @return std::string The file content store within a string
 *
 *  @throw std::system_error{} When opening the file failed
 */
inline auto FileToString(const std::filesystem::path& path) -> std::string {
  const auto file_ptr = FileOpen(path.c_str(), "r");
  if (file_ptr == nullptr) {
    throw std::system_error{
        errno,
        std::generic_category(),
        path.c_str(),
    };
  }

  return FileToString(*file_ptr);
}

}  // namespace tests::utils
