"""Render a parsed (header, tree) pair to a .pyi file string."""

from __future__ import annotations

from ._models import ClassI


def render_file(header: str, tree: list[ClassI]) -> str:
    """Construit le contenu complet du fichier .pyi corrige a partir du header et de l'arbre."""
    parts = []
    if header:
        parts.append(header)
        parts.append("")

    for cls in tree:
        parts.append(str(cls))
        parts.append("")

    content = "\n".join(parts)
    if not content.endswith("\n"):
        content += "\n"
    return content
