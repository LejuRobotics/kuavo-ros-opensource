# ProxSuite 0.7.2 for velocity IK

The velocity IK solver links the local header-only `kuavo_proxsuite` target.
It needs the existing Eigen3 dependency; it does not search for an installed
ProxSuite package, download code, run upstream installation hooks or change CI.

## Provenance

- Upstream: https://github.com/Simple-Robotics/proxsuite
- Version: 0.7.2, commit `e701292db6b6f0941c2ddf68be3772d0fae30754`.
- Archive: https://codeload.github.com/Simple-Robotics/proxsuite/tar.gz/e701292db6b6f0941c2ddf68be3772d0fae30754
- Archive SHA-256: `21e4b35220fe99c515f267e78e0a2ae564331a462886cf4f05c0710384e0284c`.
- 98 headers are unmodified upstream `include/proxsuite` files.
- `config.hpp`, `deprecated.hpp` and `warning.hpp` are upstream-generated
  headers. Their original templates and license notice are retained in
  `generated-header-templates/`.
- Generator: https://github.com/jrl-umi3218/jrl-cmakemodules at
  `b5ae8e49306840a50ae9c752c5b4040f892c89d8`; archive SHA-256
  `21485098ab289666a7a63ab5ff0998a9491fff7c88b7d3fe3494612efa3a9b42`.
  Template substitutions are listed in `provenance.json`.
- All 101 headers match the ProxSuite installation used for the existing IK
  acceptance build byte for byte. There are no platform binaries or absolute
  build paths in this copy. `headers.sha256` records every header.

The interface retains the validated non-vectorized, non-OpenMP profile.
Python bindings, tests, examples, benchmarks and upstream build modules are
not invoked. Optional serialization headers retain their upstream cereal
dependency; the IK dense QP path does not use them.

ProxSuite's BSD-2-Clause license is in `LICENSE`; original per-file copyright
and license notices, including Hedley, are preserved. The generated-header
templates retain the JRL CMake modules' own license notice.
