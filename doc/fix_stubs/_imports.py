"""Collect required imports from the parsed tree and merge them into the file header."""

from __future__ import annotations

import re
from typing import TYPE_CHECKING

from ._type_utils import type_module

if TYPE_CHECKING:
    from ._models import ClassI

IMPORT_MODULE_RE = re.compile(r"^\s*import\s+([A-Za-z_][A-Za-z0-9_.]*)")
FROM_IMPORT_RE = re.compile(r"^\s*from\s+([A-Za-z_][A-Za-z0-9_.]*)\s+import")


def collect_required_imports(tree: list[ClassI]) -> list[str]:
    """Parcourt tout l'arbre et renvoie la liste triee des modules a importer,
    a partir des types utilises dans les signatures (params + retour, getter + setter)."""
    modules: set[str] = set()

    def _scan(ov):
        m = type_module(ov.ret)
        if m:
            modules.add(m)
        for _, t in ov.params:
            m = type_module(t)
            if m:
                modules.add(m)

    for cls in tree:
        for fn in cls.functions:
            for ov in fn.overloads:
                _scan(ov)
            for ov in fn.setter_overloads:
                _scan(ov)

    return sorted(modules)


def merge_imports(header: str, required_modules: list[str]) -> str:
    """Ajoute au header les `import X` manquants parmi required_modules,
    sans dupliquer ceux deja presents (via `import X` ou `from X import ...`)."""
    header_lines = header.splitlines() if header else []

    already_covered: set[str] = set()
    for line in header_lines:
        m = IMPORT_MODULE_RE.match(line)
        if m:
            already_covered.add(m.group(1))
            continue
        m = FROM_IMPORT_RE.match(line)
        if m:
            already_covered.add(m.group(1))

    missing = [mod for mod in required_modules if mod not in already_covered]
    if not missing:
        return header

    new_import_lines = [f"import {mod}" for mod in missing]

    # inserer juste apres le dernier import existant, ou apres "from __future__", sinon en tete
    insert_at = 0
    for idx, line in enumerate(header_lines):
        if (
            IMPORT_MODULE_RE.match(line)
            or FROM_IMPORT_RE.match(line)
            or line.startswith("from __future__")
        ):
            insert_at = idx + 1

    new_header_lines = (
        header_lines[:insert_at] + new_import_lines + header_lines[insert_at:]
    )
    return "\n".join(new_header_lines)
