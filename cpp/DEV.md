This document has a minimal wiki to explain some of the design choices to help future development.

- tbb is pinned in `vcpkg.json` due to erroring in the [macos-13](https://github.com/contagon/evalio/actions/runs/14037602196/job/39299397169?pr=12) build with "is only available on macOS 10.13 or newer" when ran with 2022.0.0.