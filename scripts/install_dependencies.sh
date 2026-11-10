#!/usr/bin/env bash
# Installs SpaceVecAlg -> RBDyn -> mc_rbdyn_urdf into /usr/local.
# Run it from the colcon workspace root; the sources go to ./depends (COLCON_IGNORE).
#
# Stops at the first error, reuses clones already in depends/ and configures a fresh
# build directory every time, so it can be rerun after an interrupted run. (The
# original kept going after `git clone` / `mkdir build` failed on leftovers, and its
# `cd ../..` then cloned RBDyn / mc_rbdyn_urdf into the workspace root, where colcon
# picked them up as packages.)
set -euo pipefail

if [[ ! -d "src" ]]; then
  echo "This script should be run in a colcon workspace. Exiting." >&2
  exit 1
fi

DEPS_DIR="${PWD}/depends"
JOBS="${JOBS:-$(nproc)}"

# name url (default branch; the built commit is printed)
DEPENDENCIES=(
  "SpaceVecAlg https://github.com/jrl-umi3218/SpaceVecAlg.git"
  "RBDyn https://github.com/jrl-umi3218/RBDyn.git"
  "mc_rbdyn_urdf https://github.com/jrl-umi3218/mc_rbdyn_urdf.git"
)

mkdir -p "${DEPS_DIR}"
touch "${DEPS_DIR}/COLCON_IGNORE"

for entry in "${DEPENDENCIES[@]}"; do
  read -r name url <<< "${entry}"
  src="${DEPS_DIR}/${name}"

  if [[ -d "${src}/.git" ]]; then
    echo "Reusing ${src}"
  elif [[ -e "${src}" ]]; then
    echo "${src} exists but is not a git clone; remove it and rerun." >&2
    exit 1
  else
    git clone --recursive "${url}" "${src}"
  fi
  git -C "${src}" submodule update --init --recursive

  echo "Building ${name} at $(git -C "${src}" rev-parse --short HEAD)"
  rm -rf "${src}/build"
  cmake -S "${src}" -B "${src}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_BINDING=OFF \
    -DBUILD_TESTING=OFF
  cmake --build "${src}/build" -j "${JOBS}"
  sudo cmake --install "${src}/build"
done

# Rebuild the library cache
sudo ldconfig
