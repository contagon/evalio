# Changelog

## [0.5.0](https://github.com/contagon/evalio/compare/v0.4.2...v0.5.0) (2026-04-10)


### ⚠ BREAKING CHANGES

* Some simple nomenclature breaking changes here, fromMat → from_mat toMat → to_mat toEigen → to_eigen
* **pl:** The Pipeline class interface has changed slightly. `add_lidar` now has no return, and `pose` method has been removed. In place of these is a new `save` method for saving poses and features asynchronously with no overhead.

### Features

* **ds:** Add Boreas and BoreasRT dataset support ([#67](https://github.com/contagon/evalio/issues/67)) ([1a598f6](https://github.com/contagon/evalio/commit/1a598f6818fde578375ed1b698e0fa56275970cb))
* **ds:** Add in IMU rates to ImuParams ([#64](https://github.com/contagon/evalio/issues/64)) ([9f9859d](https://github.com/contagon/evalio/commit/9f9859d279ab85212c2148430485c9c381721f05))
* Fomo dataset ([#68](https://github.com/contagon/evalio/issues/68)) ([a232bee](https://github.com/contagon/evalio/commit/a232beec99d36955c8137b97990aecc4e2e6d649))
* **pl:** add FORM pipeline ([#72](https://github.com/contagon/evalio/issues/72)) ([a1b1a98](https://github.com/contagon/evalio/commit/a1b1a981f165e9cc6664a5eef06a4ef179b5888f))
* **pl:** Create features and pose estimate buffers ([#54](https://github.com/contagon/evalio/issues/54)) ([21403a0](https://github.com/contagon/evalio/commit/21403a06a1488b8555f594deae3354d265cd6fbf))
* **pl:** Direct Lidar Inertial Odometry ([#71](https://github.com/contagon/evalio/issues/71)) ([866feb8](https://github.com/contagon/evalio/commit/866feb8a4433b07ec8034bcac850ecb0faa164c9))


### Bug Fixes

* Add fix for loading time stamps in scientific notation (see enwide) ([#70](https://github.com/contagon/evalio/issues/70)) ([4256395](https://github.com/contagon/evalio/commit/4256395b71ee6de983fe44e14d865f6bf34de3a4))
* Bump all dependencies ([#75](https://github.com/contagon/evalio/issues/75)) ([362ad5b](https://github.com/contagon/evalio/commit/362ad5b1c851f9304473e20f7175568637ba88a3))
* **ci:** Clean up ci cibuildwheel selection and concurrency for docs deployment ([#66](https://github.com/contagon/evalio/issues/66)) ([d767fe4](https://github.com/contagon/evalio/commit/d767fe4d4f2f1547925a5bda7901674cd187ac83))
* **ci:** Fix uploading assets to github releases ([5b9f49e](https://github.com/contagon/evalio/commit/5b9f49ee0576c1514775490c8bad6ec99a2c8239))
* **docs:** Update a bunch of docs ([#77](https://github.com/contagon/evalio/issues/77)) ([4c29e94](https://github.com/contagon/evalio/commit/4c29e94af232d9888d6cf5eff7b46f9d0894b3a4))
* **pl:** Fix LIO-SAM segfaults  ([#76](https://github.com/contagon/evalio/issues/76)) ([96e0832](https://github.com/contagon/evalio/commit/96e083281c2dd4d25641f2d1e87b206a87bdc415))
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
