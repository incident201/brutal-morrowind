"""Shared discovery and reads for Dark Messiah VPK, GCF and extracted resources.

Paths exposed to conversion stages always start with mm/. Archive filenames do
not identify their contents. Loose resources override archives, as in the game.
"""
from pathlib import Path

from extract_vpk import entries as vpk_entries, extract as vpk_extract, store
from gcf import open_gcf


def archives(root):
    root = Path(root)
    found = set()
    for folder in (root, root / 'vpks', root / 'mm', root / 'mm/vpks'):
        if folder.is_dir():
            for path in folder.iterdir():
                if path.is_file() and (path.name.lower().endswith('_dir.vpk') or path.suffix.lower() == '.gcf'):
                    found.add(path)
    result = sorted(found, key=lambda path: str(path).lower())
    # A selected mm folder and a selected game folder expose the same loose tree.
    content = root / 'mm' if (root / 'mm').is_dir() else root
    has_player_model = (content / 'models/npc/player/first_basic.mdl').is_file()
    has_loose_assets = any((content / folder).is_dir() for folder in ('models', 'materials', 'sound', 'scripts'))
    # Archives can supply the player model while loose files replace individual assets.
    if has_player_model or (found and has_loose_assets):
        result.append(content)
    return result


def source_root(path):
    if archives(path):
        return path
    if path.is_dir():
        matches = sorted(child for child in path.iterdir() if child.is_dir() and archives(child))
        if len(matches) == 1:
            return matches[0]
        if matches:
            names = ', '.join(child.name for child in matches)
            raise ValueError(f'Multiple Dark Messiah folders in {path}: {names}. Select one directly.')
    raise ValueError(f'No Dark Messiah resources (VPK, GCF or extracted mm folder) found in {path}')


def entries(path):
    if path.is_dir():
        for file in sorted(path.rglob('*')):
            if file.is_file() and file.suffix.lower() not in ('.gcf', '.vpk'):
                yield 'mm/' + file.relative_to(path).as_posix(), file
    elif path.suffix.lower() == '.gcf':
        yield from open_gcf(path).entries()
    else:
        yield from vpk_entries(path)


def extract(path, entry):
    if path.is_dir():
        return entry.read_bytes()
    if path.suffix.lower() == '.gcf':
        return open_gcf(path).extract(entry)
    return vpk_extract(path, entry)
