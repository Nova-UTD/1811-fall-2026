#!/usr/bin/env python3
"""Build the ZED floor-plane extension in a cache, without editing zed_ws."""

import fcntl
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
PATCH = REPO / "ros2_ws/src/jetson_bringup/patches/zed_ros2_v5.2.2_floor_plane.patch"


def source_package(zed_ws):
    matches = [path.parent for path in (zed_ws / "src").rglob("package.xml")
               if path.parent.name == "zed_components"]
    if len(matches) != 1:
        raise RuntimeError(f"expected one zed_components source under {zed_ws / 'src'}, found {len(matches)}")
    source = matches[0]
    version = ET.parse(source / "package.xml").getroot().findtext("version")
    if version != "5.2.2":
        raise RuntimeError(f"ZED floor extension requires zed_components 5.2.2, found {version}")
    return source


def source_fingerprint(source, patch):
    digest = hashlib.sha256(patch.read_bytes())
    for path in sorted(source.rglob("*")):
        if path.is_file():
            digest.update(str(path.relative_to(source)).encode())
            with path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
    # Rebuild the overlay when the locally installed SDK headers change.
    sdk_header = Path("/usr/local/zed/include/sl/Camera.hpp")
    if sdk_header.is_file():
        digest.update(sdk_header.read_bytes())
    return digest.hexdigest()[:20]


def run(command, cwd):
    subprocess.run(command, cwd=cwd, stdout=sys.stderr, check=True)


def prepare(zed_ws, cache_root):
    source = source_package(zed_ws)
    fingerprint = source_fingerprint(source, PATCH)
    cache_root.mkdir(parents=True, exist_ok=True)
    # Multiple launch sessions must not build the same overlay concurrently.
    with (cache_root / ".build.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        overlay = cache_root / fingerprint
        src = overlay / "src"
        component = src / "zed_components"
        setup = overlay / "install/local_setup.bash"
        library = overlay / "install/zed_components/lib/libzed_camera_component.so"
        marker = overlay / ".build_complete"
        src.mkdir(parents=True, exist_ok=True)
        if not component.exists():
            # Patch a private copy; the Jetson's original wrapper stays untouched.
            with tempfile.TemporaryDirectory(prefix=".zed-source-", dir=src) as staging:
                staged = Path(staging)
                shutil.copytree(source, staged / "zed_components")
                run(["git", "apply", "--check", str(PATCH)], staged)
                run(["git", "apply", str(PATCH)], staged)
                (staged / "zed_components").rename(component)
        else:
            # Refuse to overwrite an unexpectedly changed cached source.
            run(["git", "apply", "--reverse", "--check", str(PATCH)], src)
        if not (marker.is_file() and setup.is_file() and library.is_file()):
            print("[jetson] Building isolated ZED floor-plane wrapper (first-time build may take a few minutes)...",
                  file=sys.stderr, flush=True)
            run(["colcon", "--log-base", str(overlay / "log"), "build", "--base-paths", str(src),
                 "--build-base", str(overlay / "build"), "--install-base", str(overlay / "install"),
                 "--packages-select", "zed_components", "--allow-overriding", "zed_components",
                 "--cmake-args", "-DBUILD_TESTING=OFF"], overlay)
            if not setup.is_file() or not library.is_file():
                raise RuntimeError("ZED floor wrapper build did not produce its setup file and library")
            marker.write_text(fingerprint + "\n")
        return setup


def main():
    zed_ws = Path(os.environ.get("ZED_WS", str(Path.home() / "zed_ws"))).expanduser()
    cache_root = Path(os.environ.get("NOVA_ZED_FLOOR_CACHE",
                                     str(Path.home() / ".cache/1811/zed_floor_wrapper"))).expanduser()
    try:
        print(prepare(zed_ws, cache_root))
    except (OSError, RuntimeError, ET.ParseError, subprocess.CalledProcessError) as exc:
        print(f"[jetson] [ERROR] Could not prepare ZED floor-plane wrapper: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
