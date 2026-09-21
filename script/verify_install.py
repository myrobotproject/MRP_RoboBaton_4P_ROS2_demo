#!/usr/bin/env python3
"""Verify the canonical-layout ROS2 merged install against ABI-v2 producers."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import subprocess
import sys

PACKAGE = "robobaton_4p_ros2_demo"
RELEASE_VERSION = (Path(__file__).resolve().parents[1] / "VERSION").read_text(encoding="utf-8").strip()
NODE = "robobaton_sensors_node"
IMU_RATE_MONITOR = "robobaton_imu_rate_monitor"
EXECUTABLES = (NODE, IMU_RATE_MONITOR)
MANIFEST = "abi_manifest.sha256"
PLUGIN_LIBRARY = "librobobaton_nv12_compressed_image_transport.so"
PLUGIN_XML = f"share/{PACKAGE}/robobaton_nv12_compressed_plugins.xml"
ENV_SCRIPT = "robobaton_ros2_env.bash"
PLUGIN_RESOURCE = f"share/ament_index/resource_index/image_transport__pluginlib__plugin/{PACKAGE}"
ROS_TRANSITIVE_RUNTIME_LIBS = (
    "libconsole_bridge.so", "libconsole_bridge.so.1.0",
    "libspdlog.so", "libspdlog.so.1", "libspdlog.so.1.9.2",
    "libfmt.so", "libfmt.so.8", "libfmt.so.8.1.1",
    "libtinyxml2.so", "libtinyxml2.so.9", "libtinyxml2.so.9.0.0",
    "libssl.so", "libssl.so.3", "libcrypto.so", "libcrypto.so.3",
    "libblas.so.3", "liblapack.so.3", "libgfortran.so.5",
)
LIBRARIES = {
    "icm42688": {
        "names": ("libicm42688.so.2.1.0", "libicm42688.so.2", "libicm42688.so"),
        "soname": "libicm42688.so.2",
        "versions": ("ICM42688_X5_2.0", "ICM42688_X5_2.1"),
    },
    "sc132": {
        "names": ("libsc132.so.2.0.1", "libsc132.so.2", "libsc132.so"),
        "soname": "libsc132.so.2",
        "versions": ("LIBSC132_2.0",),
    },
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def readelf(path: Path, *args: str) -> str:
    return subprocess.run(
        ["readelf", *args, str(path)], text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, check=True
    ).stdout


def versions(path: Path) -> set[str]:
    return set(re.findall(r"Name:\s*([^\s]+)", readelf(path, "--version-info", "--wide")))


def soname(path: Path) -> str | None:
    match = re.search(r"Library soname: \[([^\]]+)\]", readelf(path, "-d", "--wide"))
    return match.group(1) if match else None


def needed(path: Path) -> list[str]:
    return re.findall(r"Shared library: \[([^\]]+)\]", readelf(path, "-d", "--wide"))


def dynamic_symbols(path: Path) -> set[str]:
    return set(re.findall(
        r"\b(?:GLOBAL|WEAK)\b\s+\bDEFAULT\b\s+\S+\s+([A-Za-z_][A-Za-z0-9_]*)",
        readelf(path, "--dyn-syms", "--wide"),
    ))


def runpath(path: Path) -> str:
    match = re.search(r"Library runpath: \[([^\]]*)\]", readelf(path, "-d", "--wide"))
    return match.group(1) if match else ""


def verify_relocatable_bash_setup(install_dir: Path) -> None:
    setup_bash = install_dir / "setup.bash"
    if not setup_bash.is_file():
        raise AssertionError(f"missing relocatable setup entry: {setup_bash}")
    hardcoded_prefixes = re.findall(
        r'''(?m)^COLCON_CURRENT_PREFIX=["'](/[^"']+)["']$''',
        setup_bash.read_text(encoding="utf-8", errors="strict"),
    )
    if hardcoded_prefixes:
        raise AssertionError(f"setup.bash embeds absolute underlays: {hardcoded_prefixes}")
    source_result = subprocess.run(
        [
            "/bin/bash", "--noprofile", "--norc", "-c",
            'set -e\nsource "$1/setup.bash"\n'
            'printf "__AMENT_PREFIX_PATH__=%s\\n" "${AMENT_PREFIX_PATH:-}"\n'
            'printf "__COLCON_PREFIX_PATH__=%s\\n" "${COLCON_PREFIX_PATH:-}"',
            "verify-install", str(install_dir),
        ],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="/",
        env={"HOME": "/tmp", "LANG": "C", "PATH": "/usr/bin:/bin"},
    )
    if source_result.returncode != 0 or source_result.stderr:
        raise AssertionError(
            f"setup.bash clean-shell failure rc={source_result.returncode}: "
            f"{source_result.stderr.strip()}"
        )
    expected = str(install_dir)
    for name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        marker = f"__{name}__="
        lines = [line[len(marker):] for line in source_result.stdout.splitlines()
                 if line.startswith(marker)]
        if len(lines) != 1 or [value for value in lines[0].split(":") if value] != [expected]:
            raise AssertionError(f"setup.bash leaked prefixes into {name}: {lines}")


def verify_runtime_env_script(install_dir: Path) -> None:
    env_script = install_dir / ENV_SCRIPT
    if not env_script.is_file() or env_script.is_symlink() or not (env_script.stat().st_mode & 0o111):
        raise AssertionError(f"missing executable runtime env script: {env_script}")
    source_result = subprocess.run(
        [
            "/bin/bash", "--noprofile", "--norc", "-c",
            'set -e\nsource "$1/robobaton_ros2_env.bash"\n'
            'printf "__PREFIX__=%s\\n" "${ROBOBATON_ROS2_INSTALL_PREFIX:-}"\n'
            'printf "__BUFFER__=%s\\n" "${RCUTILS_LOGGING_BUFFERED_STREAM:-}"\n'
            'printf "__FASTDDS__=%s\\n" "${FASTDDS_DEFAULT_PROFILES_FILE:-}"\n'
            'declare -F robobaton_ros2_list_topics >/dev/null\n',
            "verify-env", str(install_dir),
        ],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="/",
        env={"HOME": "/tmp", "LANG": "C", "PATH": "/usr/bin:/bin", "ROBOBATON_ROS_UNDERLAY": ""},
    )
    if source_result.returncode != 0 or source_result.stderr:
        raise AssertionError(
            f"runtime env script source failure rc={source_result.returncode}: "
            f"{source_result.stderr.strip()}"
        )
    expected_profile = install_dir / "share" / PACKAGE / "config" / "fastdds" / "robobaton_shm.xml"
    expected_markers = {
        "__PREFIX__": str(install_dir),
        "__BUFFER__": "0",
        "__FASTDDS__": str(expected_profile),
    }
    observed = dict(line.split("=", 1) for line in source_result.stdout.splitlines() if line.startswith("__"))
    if observed != expected_markers:
        raise AssertionError(f"runtime env script exported unexpected values: {observed}")
    execute_result = subprocess.run(
        [str(env_script), "bash", "-c", 'printf "__EXEC_PREFIX__=%s\\n" "${ROBOBATON_ROS2_INSTALL_PREFIX:-}"'],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="/",
        env={"HOME": "/tmp", "LANG": "C", "PATH": "/usr/bin:/bin", "ROBOBATON_ROS_UNDERLAY": ""},
    )
    if execute_result.returncode != 0 or execute_result.stderr:
        raise AssertionError(
            f"runtime env script execute failure rc={execute_result.returncode}: "
            f"{execute_result.stderr.strip()}"
        )
    if execute_result.stdout.strip() != f"__EXEC_PREFIX__={install_dir}":
        raise AssertionError(f"runtime env script command wrapper failed: {execute_result.stdout!r}")




def required_symbol_versions(spec: dict[str, object]) -> set[str]:
    return set(spec["versions"])

NODE_LIBRARY_KEYS = ("icm42688", "sc132")
# The node only needs the versions of symbols it calls; producers must still provide the complete ABI 2.1 version-node set.
NODE_REQUIRED_VERSIONS = {"ICM42688_X5_2.0", "LIBSC132_2.0"}
PLUGIN_PLATFORM_SONAMES = {"libmultimedia.so.1", "libhbmem.so.1", "libalog.so.1"}


def artifact_names() -> list[str]:
    names = [*EXECUTABLES, PLUGIN_LIBRARY]
    for spec in LIBRARIES.values():
        names.extend(spec["names"])
    return sorted(names)


def write_manifest(runtime_dir: Path) -> None:
    lines = [f"{sha256(runtime_dir / name)}  {name}" for name in artifact_names()]
    (runtime_dir / MANIFEST).write_text("\n".join(lines) + "\n", encoding="utf-8")


def verify_plugin_registration(install_dir: Path) -> None:
    plugin_xml = install_dir / PLUGIN_XML
    plugin_resource = install_dir / PLUGIN_RESOURCE
    if not plugin_xml.is_file() or plugin_xml.is_symlink():
        raise AssertionError(f"missing image_transport plugin XML: {plugin_xml}")
    if not plugin_resource.is_file() or plugin_resource.is_symlink():
        raise AssertionError(f"missing image_transport plugin resource: {plugin_resource}")
    xml_text = plugin_xml.read_text(encoding="utf-8")
    if 'name="robobaton_4p_ros2_demo/compressed_pub"' not in xml_text:
        raise AssertionError("NV12 compressed plugin does not claim compressed transport")
    resource_entries = {line.strip() for line in plugin_resource.read_text(
        encoding="utf-8").splitlines() if line.strip()}
    if PLUGIN_XML not in resource_entries:
        raise AssertionError(f"image_transport plugin resource missing {PLUGIN_XML}")


def verify_manifest(runtime_dir: Path) -> None:
    manifest = runtime_dir / MANIFEST
    if not manifest.is_file() or manifest.is_symlink():
        raise AssertionError(f"missing regular manifest: {manifest}")
    expected_names = set(artifact_names())
    seen: set[str] = set()
    for line in manifest.read_text(encoding="utf-8").splitlines():
        digest, separator, name = line.partition("  ")
        if not separator or name not in expected_names or name in seen:
            raise AssertionError(f"invalid manifest line: {line!r}")
        if sha256(runtime_dir / name) != digest:
            raise AssertionError(f"manifest hash mismatch: {name}")
        seen.add(name)
    if seen != expected_names:
        raise AssertionError(f"manifest inventory mismatch: {sorted(seen)} != {sorted(expected_names)}")


def verify_executable(runtime_dir: Path, install_dir: Path, name: str) -> Path:
    executable = runtime_dir / name
    if not executable.is_file() or executable.is_symlink() or not (executable.stat().st_mode & 0o111):
        raise AssertionError(f"missing executable: {executable}")
    executable_runpath = runpath(executable)
    if "$ORIGIN" not in executable_runpath or str(install_dir) in executable_runpath:
        raise AssertionError(f"{name} RUNPATH is not package-relative: {executable_runpath!r}")
    return executable


def verify(install_dir: Path) -> None:
    runtime_dir = install_dir / "lib" / PACKAGE
    version_file = install_dir / "share" / PACKAGE / "VERSION"
    if not version_file.is_file() or version_file.read_text(encoding="utf-8").strip() != RELEASE_VERSION:
        raise AssertionError(f"missing or invalid ROS2 release VERSION: {version_file}")
    node = verify_executable(runtime_dir, install_dir, NODE)
    verify_executable(runtime_dir, install_dir, IMU_RATE_MONITOR)

    plugin = runtime_dir / PLUGIN_LIBRARY
    if not plugin.is_file() or plugin.is_symlink():
        raise AssertionError(f"missing image_transport plugin library: {plugin}")
    if "robobaton_nv12_compressed_image_transport_get_version" not in dynamic_symbols(plugin):
        raise AssertionError("compressed image plugin does not export its release version getter")

    for name in ROS_TRANSITIVE_RUNTIME_LIBS:
        path = runtime_dir / name
        if not path.is_file():
            raise AssertionError(f"missing ROS2 transitive runtime library: {path}")

    project_needed = {name for name in needed(node)
                      if name.startswith(("libicm42688", "libsc132", "libmultimedia"))}
    expected_project_needed = {LIBRARIES[key]["soname"] for key in NODE_LIBRARY_KEYS}
    if project_needed != expected_project_needed:
        raise AssertionError(
            f"project DT_NEEDED mismatch: {sorted(project_needed)} != {sorted(expected_project_needed)}"
        )
    node_versions = versions(node)
    if not NODE_REQUIRED_VERSIONS.issubset(node_versions):
        raise AssertionError(
            f"node version needs missing: {sorted(NODE_REQUIRED_VERSIONS - node_versions)}"
        )
    old_versions = {value for value in node_versions if value.endswith("_1.0") and
                    value.startswith(("ICM42688_", "LIBSC132_"))}
    if old_versions:
        raise AssertionError(f"node retains old ABI needs: {sorted(old_versions)}")
    # The executable $ORIGIN RUNPATH is checked centrally by verify_executable.

    plugin_needed = {name for name in needed(plugin)
                     if name.startswith(("libmultimedia", "libhbmem", "libalog"))}
    if plugin_needed != PLUGIN_PLATFORM_SONAMES:
        raise AssertionError(
            f"plugin DT_NEEDED mismatch: {sorted(plugin_needed)} != {sorted(PLUGIN_PLATFORM_SONAMES)}"
        )
    plugin_runpath = runpath(plugin)
    if "$ORIGIN" not in plugin_runpath or str(install_dir) in plugin_runpath:
        raise AssertionError(f"plugin RUNPATH is not package-relative: {plugin_runpath!r}")

    for library, spec in LIBRARIES.items():
        paths = [runtime_dir / name for name in spec["names"]]
        for path in paths:
            if not path.is_file():
                raise AssertionError(f"missing {library} package entry: {path}")
        hashes = {sha256(path) for path in paths}
        if len(hashes) != 1:
            raise AssertionError(f"{library} version chain bytes differ: {sorted(hashes)}")
        real = paths[0]
        if soname(real) != spec["soname"]:
            raise AssertionError(f"{library} SONAME mismatch: {soname(real)!r}")
        missing_versions = required_symbol_versions(spec) - versions(real)
        if missing_versions:
            raise AssertionError(f"{library} version nodes missing: {sorted(missing_versions)}")

    verify_plugin_registration(install_dir)
    verify_runtime_env_script(install_dir)
    verify_relocatable_bash_setup(install_dir)
    verify_manifest(runtime_dir)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("install_dir", type=Path)
    parser.add_argument("--write-manifest", action="store_true")
    args = parser.parse_args()
    install_dir = args.install_dir.resolve()
    runtime_dir = install_dir / "lib" / PACKAGE
    if args.write_manifest:
        write_manifest(runtime_dir)
    verify(install_dir)
    print(f"ROS2 ABI-v2 install verified: {install_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, subprocess.CalledProcessError) as error:
        print(f"ROS2 ABI install verification failed: {error}", file=sys.stderr)
        raise SystemExit(1)
