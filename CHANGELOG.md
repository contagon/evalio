# Changelog

## [0.5.0](https://github.com/contagon/evalio/compare/v0.4.2...v0.5.0) (2025-11-13)


### ⚠ BREAKING CHANGES

* Some simple nomenclature breaking changes here, fromMat → from_mat toMat → to_mat toEigen → to_eigen
* **pl:** The Pipeline class interface has changed slightly. `add_lidar` now has no return, and `pose` method has been removed. In place of these is a new `save` method for saving poses and features asynchronously with no overhead.

### Features

* **ds:** Add in IMU rates to ImuParams ([#64](https://github.com/contagon/evalio/issues/64)) ([9f9859d](https://github.com/contagon/evalio/commit/9f9859d279ab85212c2148430485c9c381721f05))
* **pl:** Create features and pose estimate buffers ([#54](https://github.com/contagon/evalio/issues/54)) ([21403a0](https://github.com/contagon/evalio/commit/21403a06a1488b8555f594deae3354d265cd6fbf))


### Bug Fixes

* **ci:** Clean up ci cibuildwheel selection and concurrency for docs deployment ([#66](https://github.com/contagon/evalio/issues/66)) ([d767fe4](https://github.com/contagon/evalio/commit/d767fe4d4f2f1547925a5bda7901674cd187ac83))
* **ci:** Fix uploading assets to github releases ([5b9f49e](https://github.com/contagon/evalio/commit/5b9f49ee0576c1514775490c8bad6ec99a2c8239))
* Switch interface to consistent snake_case naming ([#65](https://github.com/contagon/evalio/issues/65)) ([ded06a8](https://github.com/contagon/evalio/commit/ded06a8613829b004b54f7f86ce1086e68b2ba69))

## [0.4.2](https://github.com/contagon/evalio/compare/v0.4.1...v0.4.2) (2025-10-31)


### Features

* Add support for python 3.14 ([#60](https://github.com/contagon/evalio/issues/60)) ([53e2109](https://github.com/contagon/evalio/commit/53e2109cb520daf39f13c5c307402efce58df9c7))
* **cli:** Make title include url in ls command ([#57](https://github.com/contagon/evalio/issues/57)) ([904b194](https://github.com/contagon/evalio/commit/904b19446a4a401f1d423548f7067d6333cd224f))
* **pl:** Type conversions module ([#62](https://github.com/contagon/evalio/issues/62)) ([c629fef](https://github.com/contagon/evalio/commit/c629fef700b043b56a9e0abe528ccd29a2d2b0b7))


### Bug Fixes

* **ci:** Add __init__.py to release-please version bumper ([b350adc](https://github.com/contagon/evalio/commit/b350adc18f63d0afc763563ac658f52609c54206))
* **ci:** Cast release-please output to a boolean ([3e3c173](https://github.com/contagon/evalio/commit/3e3c173f3bba3834c2078974d2017d948d609bc2))
* **ci:** Tweak release output ([4c6787c](https://github.com/contagon/evalio/commit/4c6787ceacb346a6e19a33d5fdd7a290d2c0d0aa))
* Remove free-threaded python builds ([d91297a](https://github.com/contagon/evalio/commit/d91297aeabb8e283060fcf035ba22efd5fe689f9))
