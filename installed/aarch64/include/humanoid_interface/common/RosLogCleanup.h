#pragma once

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <functional>
#include <regex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <sys/stat.h>

namespace ros_log_cleanup {
namespace fs = std::filesystem;

constexpr std::time_t synced_epoch = 946684800;

struct Run {
  std::time_t started;
  std::time_t now;
  std::time_t booted;
  fs::path log_directory;
  std::set<fs::path> recorded_files;
};

struct File {
  fs::path path;
  std::string filename;
  uintmax_t disk_size;
  std::time_t mtime_sec;
  long mtime_nsec;
  bool is_synced_epoch;
  bool is_protected_current_run;
  bool is_bag = false;
  bool is_active_bag = false;
  bool has_valid_bag_id = false;
  std::string bag_prefix{};
  std::string split_stream{};
  uint64_t bag_id = 0;
};

inline bool inDirectory(const fs::path& path, const fs::path& directory) {
  if (directory.empty()) return false;
  const auto rel = path.lexically_relative(directory);
  return !rel.empty() && *rel.begin() != "..";
}

inline File describeFile(const fs::path& path, const struct stat& st, const Run& run) {
  File file{path, path.filename().string(), static_cast<uintmax_t>(st.st_blocks) * 512,
            st.st_mtim.tv_sec, st.st_mtim.tv_nsec, st.st_mtime >= synced_epoch, false};
  file.is_active_bag = path.extension() == ".active" && path.stem().extension() == ".bag";
  file.is_bag = file.is_active_bag || path.extension() == ".bag";
  const bool in_run_window = (run.started >= synced_epoch) == (run.now >= synced_epoch) &&
                             (run.started >= synced_epoch) == file.is_synced_epoch &&
                             file.mtime_sec >= run.started && file.mtime_sec <= run.now;
  file.is_protected_current_run = file.is_active_bag || run.recorded_files.count(path) ||
                                  inDirectory(path, run.log_directory) ||
                                  (!file.is_bag && in_run_window);
  if (!file.is_bag) return file;

  const auto stem = (file.is_active_bag ? path.stem() : path.filename()).stem().string();
  const auto sep = stem.rfind('_');
  if (sep == std::string::npos || sep == 0) return file;
  const char* end = stem.data() + stem.size();
  const auto parsed = std::from_chars(stem.data() + sep + 1, end, file.bag_id);
  file.has_valid_bag_id = parsed.ec == std::errc{} && parsed.ptr == end;
  if (!file.has_valid_bag_id) {
    file.bag_id = 0;
    return file;
  }
  file.bag_prefix = stem.substr(0, sep);
  // Recorder regenerates the date for every split, even within a single recording.
  static const std::regex stamp("(^|_)[0-9]{4}(-[0-9]{2}){5}$");
  file.split_stream = std::regex_replace(file.bag_prefix, stamp, "");
  return file;
}

// Files are sorted newest first. A reset or a gap in split IDs ends the round.
template <typename Eligible>
inline std::vector<size_t> newestRound(const std::vector<File>& files, size_t limit, Eligible eligible) {
  std::vector<size_t> batch;
  for (size_t i = 0; i < files.size() && batch.size() < limit; ++i) {
    const auto& file = files[i];
    if (!file.has_valid_bag_id || !eligible(file)) continue;
    if (!batch.empty()) {
      const auto& last = files[batch.back()];
      if (last.is_synced_epoch != file.is_synced_epoch || last.split_stream != file.split_stream ||
          last.bag_id == 0 || file.bag_id != last.bag_id - 1) break;
    }
    batch.push_back(i);
  }
  return batch;
}

struct Scan {
  std::vector<File> files;
  std::set<fs::path> directories;
  uintmax_t total_size = 0;
  size_t skipped = 0;
};

using Exclude = std::function<bool(const fs::path&, bool)>;

inline Scan scan(const fs::path& root, const Run& run, size_t bags_to_keep,
                 bool bags_only, const Exclude& excluded = {}) {
  Scan result;
  for (fs::recursive_directory_iterator it(root), end; it != end; ++it) {
    const auto& path = it->path();
    // Read metadata once without following symlink aliases such as log/latest.
    struct stat st{};
    if (::lstat(path.c_str(), &st) != 0 || S_ISLNK(st.st_mode)) continue;
    const bool directory = S_ISDIR(st.st_mode);
    if (excluded && excluded(path, directory)) {
      ++result.skipped;
      if (directory) it.disable_recursion_pending();
      continue;
    }
    if (directory) {
      if (bags_only) it.disable_recursion_pending();
      else result.directories.insert(path);
      continue;
    }
    if (!S_ISREG(st.st_mode)) continue;
    auto file = describeFile(path, st, run);
    if (!bags_only || file.is_bag) {
      result.total_size += file.disk_size;
      result.files.push_back(std::move(file));
    }
  }

  auto key = [](const File& f) {
    return std::tie(f.is_synced_epoch, f.mtime_sec, f.mtime_nsec, f.bag_id, f.filename, f.path);
  };
  std::sort(result.files.begin(), result.files.end(),
            [&](const File& a, const File& b) { return key(a) > key(b); });

  if (run.now < synced_epoch && run.recorded_files.empty()) {
    // Before the first open, a reboot can put historical bags on either side
    // of 'now'. Only infer the latest continuous round in this boot's window.
    auto in_boot = [&](const File& f) {
      return f.path.parent_path() == root && f.mtime_sec >= run.booted && f.mtime_sec <= run.now;
    };
    for (auto i : newestRound(result.files, bags_to_keep, in_boot))
      result.files[i].is_protected_current_run = true;
  }
  return result;
}

template <typename Remove>
inline void trimBags(const Scan& scan, bool per_run, size_t keep, size_t max_splits, Remove remove) {
  if (per_run) {
    // Existing per-run policy trims only the most recent round. Leave
    // unnumbered bags and all earlier rounds alone.
    const auto batch = newestRound(scan.files, max_splits, [](const File&) { return true; });
    for (size_t n = keep; n < batch.size(); ++n) {
      const auto& f = scan.files[batch[n]];
      if (!f.is_protected_current_run) remove(f.path);
    }
    return;
  }
  size_t retained = std::count_if(scan.files.begin(), scan.files.end(),
                                 [](const File& f) { return f.is_protected_current_run; });
  for (const auto& f : scan.files) {
    if (f.is_protected_current_run) continue;
    if (retained < keep) ++retained;
    else remove(f.path);
  }
}

template <typename Remove>
inline uintmax_t trimToSize(const Scan& scan, uintmax_t maximum, uintmax_t target, Remove remove) {
  uintmax_t size = scan.total_size;
  if (size <= maximum) return size;
  for (auto it = scan.files.rbegin(); it != scan.files.rend() && size > target; ++it) {
    if (!it->is_protected_current_run && remove(it->path)) size -= it->disk_size;
  }
  return size;
}

}  // namespace ros_log_cleanup
