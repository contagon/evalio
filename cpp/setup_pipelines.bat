:: Call this script from the root of evalio
@ECHO Off

:: ----------------- Pipelines ----------------- ::
:: Create the pipelines-src directory if it doesn't exist
if not exist cpp\bindings\pipelines-src (
    mkdir cpp\bindings\pipelines-src
)
cd "cpp/bindings/pipelines-src"

:: kiss
if not exist kiss-icp (
    git clone https://github.com/PRBonn/kiss-icp.git
)
cd kiss-icp
git stash
git switch --detach v1.2.2
git apply ../../pipelines/kiss_icp.patch
cd ..

:: lio-sam
if not exist LIO-SAM (
    git clone --depth 1 https://github.com/contagon/LIO-SAM.git
)
cd "LIO-SAM"
git stash
git checkout master
cd ..

:: ------------------------- Dependencies ------------------------- ::
cd ../../..
if not exist .vcpkg (
    :: Clone the vcpkg repository
    git clone --depth 1 https://github.com/microsoft/vcpkg.git .vcpkg/
)
:: Bootstrap vcpkg
.\.vcpkg\bootstrap-vcpkg.bat