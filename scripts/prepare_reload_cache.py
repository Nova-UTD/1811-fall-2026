#!/usr/bin/env python3
"""Reuse intact cached KISS-ICP dependency archives during a fresh ROS build."""

import gzip
import json
from pathlib import Path, PurePosixPath
import shutil
import sys
import tarfile
import tempfile


# These versions are the ones named in the vendored KISS-ICP CMake files.
DEPENDENCIES = (
    ('sophus', 'Sophus-1.24.6', '1.24.6.tar.gz', 'sophus/sophus.cmake',
     'https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.tar.gz'),
    ('tessil', 'robin-map-1.4.0', 'v1.4.0.tar.gz', 'tsl_robin/tsl_robin.cmake',
     'https://github.com/Tessil/robin-map/archive/refs/tags/v1.4.0.tar.gz'),
)
MAX_ARCHIVE_BYTES = 20 * 1024 * 1024
MAX_EXPANDED_BYTES = 64 * 1024 * 1024
MAX_MEMBERS = 5000


def extract_checked(archive_path, expected_root, destination):
    """Verify gzip integrity and extract only regular files below one root."""
    if archive_path.stat().st_size > MAX_ARCHIVE_BYTES:
        raise ValueError('archive is too large')
    expanded = 0
    with gzip.open(archive_path, 'rb') as stream:
        while chunk := stream.read(1024 * 1024):
            expanded += len(chunk)
            if expanded > MAX_EXPANDED_BYTES:
                raise ValueError('expanded archive is too large')
    with tarfile.open(archive_path, 'r:gz') as archive:
        members = archive.getmembers()
        if not members or len(members) > MAX_MEMBERS:
            raise ValueError('archive has an unexpected number of entries')
        for member in members:
            parts = PurePosixPath(member.name).parts
            if (not parts or parts[0] != expected_root or '..' in parts or
                    member.name.startswith('/') or not (member.isdir() or member.isfile())):
                raise ValueError(f'unsafe archive entry: {member.name}')
            relative = parts[1:]
            if not relative:
                if not member.isdir():
                    raise ValueError('archive root is not a directory')
                continue
            target = destination.joinpath(*relative)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            source = archive.extractfile(member)
            if source is None:
                raise ValueError(f'cannot read archive entry: {member.name}')
            with source, target.open('xb') as output:
                shutil.copyfileobj(source, output)
            target.chmod(member.mode & 0o777)
    if not (destination / 'CMakeLists.txt').is_file():
        raise ValueError('archive has no CMakeLists.txt')


def prepare(source, previous_build, fresh_build):
    cache = fresh_build / '.reload-dependencies'
    metadata_args = []
    builds = [previous_build]
    backup_root = previous_build.parent.parent
    if previous_build.parent.name.startswith('reload.') and backup_root.name == '.reload-backups':
        builds.extend(run / 'build' for run in sorted(backup_root.glob('reload.*'), reverse=True)
                      if run != previous_build.parent)
    for name, root, archive_name, cmake_name, expected_url in DEPENDENCIES:
        declaration = source / 'cpp/kiss_icp/3rdparty' / cmake_name
        if expected_url not in declaration.read_text(encoding='utf-8'):
            print(f'[reload] {name} dependency version changed; using normal CMake discovery.', file=sys.stderr)
            continue
        candidates = [candidate for build in builds for candidate in (
            build / '.reload-dependencies/archives' / f'{name}-{archive_name}',
            build / 'kiss_icp/_deps' / f'{name}-subbuild' / f'{name}-populate-prefix/src' / archive_name,
        ) if candidate.is_file()]
        if not candidates:
            print(f'[reload] No cached {name} archive; CMake may need internet access.', file=sys.stderr)
            continue
        cache.mkdir(parents=True, exist_ok=True)
        target = cache / name
        for archive in candidates:
            try:
                with tempfile.TemporaryDirectory(prefix=f'.{name}-', dir=cache) as temporary:
                    extracted = Path(temporary) / name
                    extracted.mkdir()
                    extract_checked(archive, root, extracted)
                    archives = cache / 'archives'
                    archives.mkdir(exist_ok=True)
                    shutil.copyfile(archive, archives / f'{name}-{archive_name}')
                    extracted.rename(target)
                break
            except (OSError, ValueError, EOFError, gzip.BadGzipFile, tarfile.TarError) as error:
                print(f'[reload] Cached {name} archive is invalid ({error}); trying another copy.', file=sys.stderr)
        else:
            print(f'[reload] No intact {name} archive; CMake may need internet access.', file=sys.stderr)
            continue
        metadata_args.append(f'-DFETCHCONTENT_SOURCE_DIR_{name.upper()}={target}')
        print(f'[reload] Reusing the checked {name} dependency archive.', file=sys.stderr)
    if not metadata_args:
        return None
    metadata = cache / 'reload.meta'
    metadata.write_text(json.dumps({'names': {'kiss_icp': {'cmake-args': metadata_args}}}), encoding='utf-8')
    return metadata


if __name__ == '__main__':
    if len(sys.argv) != 4:
        raise SystemExit('usage: prepare_reload_cache.py <kiss-icp source> <previous build> <fresh build>')
    result = prepare(*(Path(value).resolve() for value in sys.argv[1:]))
    if result is not None:
        print(result)
