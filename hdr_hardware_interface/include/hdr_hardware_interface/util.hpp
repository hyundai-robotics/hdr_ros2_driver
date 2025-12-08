/**
 * @brief Lightweight utility helpers shared across HDR projects.
 * This header contains a handful of free functions and constants that are
 * required by both the *HDR Driver* and the ros2_control *SystemInterface*.
 * Keep this file **header‑only** to avoid linker dependencies.
 *
 * @file util.hpp
 * @author HD Hyundai Robotics
 */

#ifndef HDR_UTIL_HPP_
#define HDR_UTIL_HPP_

#include <algorithm>  // std::transform, std::tolower
#include <cctype>     // ::tolower
#include <iostream>   // std::cerr
#include <string>     // std::string
#include <unordered_map>
#include <variant>

namespace util {

// ──────────────────────────────────────────────────────────────────────────────
// Type aliases
// ──────────────────────────────────────────────────────────────────────────────
/// <parameter‑name, parameter‑value‑as‑string>
using ParamMap = std::unordered_map<std::string, std::string>;
/// Variant that holds strongly‑typed parameter results
using ParamValue = std::variant<std::string, int, bool, double>;

// ──────────────────────────────────────────────────────────────────────────────
// Constants
// ──────────────────────────────────────────────────────────────────────────────
/// Minimum controller *system* firmware version supported by this driver.
inline constexpr double kMinSupportedSysVer{60.3200};
/**
 * @brief socket Stream protocol version for handshake negotiation.
 *
 * @details
 * This version string follows Semantic Versioning (major.minor.patch):
 * - Major: Breaking changes (incompatible with different major versions)
 * - Minor: New features (backward compatible)
 * - Patch: Bug fixes (backward compatible)
 *
 * Used in DoHandshake() to verify protocol compatibility between client and server.
 * Only major version must match; minor/patch differences are acceptable.
 */
inline constexpr const char* kStreamSvrVer = "1.0.0";
/// Generic numerical tolerance used across the driver.
inline constexpr double kEpsilon{1e-4};

// ──────────────────────────────────────────────────────────────────────────────
// String helpers
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @param[in] str  Input string.
 *
 * @return Lower‑case copy of @p str.
 *
 * @brief Converts an ASCII string to lower‑case.
 *
 */
[[nodiscard]] inline std::string ToLower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return str;
}

/**
 * @brief Compares two strings ignoring ASCII case.
 * @note Uses simple ASCII *tolower* conversion; UTF‑8 is **not** considered.
 *
 */
[[nodiscard]] inline bool CompareIgnoreCase(const std::string& a, const std::string& b) {
  return ToLower(a) == ToLower(b);
}

// ──────────────────────────────────────────────────────────────────────────────
// Parameter parsing
// ──────────────────────────────────────────────────────────────────────────────

/**
  * @param[in] params       Map of <key, value‑as‑string> pairs.
  * @param[in] key          Parameter name to look up.
  * @param[in] default_val  Fallback value when parsing fails or key is
 missing.
  * @param[in] type_hint    Expected type: "string", "int", or "bool"
 (case‑insensitive).
  *
  * @return Parsed value wrapped in @ref ParamValue.
  *
  * @brief Retrieves and converts a parameter from a string map.
  *
  * @throws std::runtime_error if type conversion fails   // ← 추가 필요
  *
  * @example
  * @code
  * ParamMap params = {{"speed", "100"}, {"enabled", "true"}};
  * auto speed = GetParam(params, "speed", 50, "int");
  * auto enabled = GetParam(params, "enabled", false, "bool");
  * @endcode
  */
inline ParamValue GetParam(const ParamMap& params, const std::string& key,
                           const ParamValue& default_val, const std::string& type_hint) {
  const auto it = params.find(key);
  if (it == params.end()) {
    return default_val;
  }

  const std::string& raw = it->second;

  try {
    const std::string type = ToLower(type_hint);
    if (type == "string") {
      return raw;
    } else if (type == "int") {
      return std::stoi(raw);
    } else if (type == "double") {
      return std::stod(raw);
    } else if (type == "bool") {
      const std::string val = ToLower(raw);
      if (val == "true" || val == "1")
        return true;
      if (val == "false" || val == "0")
        return false;
      throw std::runtime_error("invalid boolean literal");
    }

    throw std::runtime_error("unsupported type hint");
  } catch (const std::exception& e) {
    std::cerr << "[WARN] util::GetParam: could not parse '" << key << "' as " << type_hint
              << " → using default (" << e.what() << ")\n";
    return default_val;
  }
}

static const std::unordered_map<std::string, std::vector<std::string>> kAllowedMap = {
    {"ha006b", {"ha006b", "HA006B-01"}},
    {"hdf7_9", {"hdf7-9", "HH7-02"}},
    {"hdf8_8", {"hdf8-8", "HH8-01"}},
    {"hdr10l_19", {"hdr10l_19", "HDR10L-19"}},
    {"hdr20_17", {"hdr20-17", "HDR20-17"}},
    {"hdr50_22", {"hdr50-22", "HH050-11"}},
    {"hdr220_26", {"hdr220-26", "HS220-01", "HS220-02", "HS220-03"}},
    {"hh020", {"hh020", "HH020-03"}},
    {"hdr35_20", {"hdr35-20", "UH035"}}
};

}  // namespace util

#endif  // HDR_UTIL_HPP_
