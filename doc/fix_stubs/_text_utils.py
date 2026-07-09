"""Low-level text manipulation helpers."""

from __future__ import annotations


def indent_block(text: str, spaces: int) -> str:
    """Indente chaque ligne de `text` de `spaces` espaces."""
    pad = " " * spaces
    return "\n".join(pad + line if line else line for line in text.splitlines())


def read_docstring(lines: list[str], start: int) -> tuple[str, int]:
    """Lit une docstring triple-quote a partir de lines[start]. Retourne (texte, index_apres).

    Gere aussi le cas ou du texte suit directement les \"\"\" d'ouverture sur la meme ligne
    (ex: '\"\"\"Device with handles.'), qui peut arriver quand on re-parse un fichier deja
    corrige par ce script (pour que l'operation reste stable si on la relance)."""
    stripped = lines[start].strip()
    if stripped.count('"""') >= 2 and len(stripped) > 3:
        return stripped.split('"""', 2)[1], start + 1

    opening_text = stripped[3:]
    body = [opening_text] if opening_text else []
    i = start + 1
    n = len(lines)
    while i < n and lines[i].strip() != '"""':
        body.append(lines[i])
        i += 1
    i += 1
    return "\n".join(body), i


def clean_description(desc_lines: list[str]) -> str:
    """Nettoie les lignes de description qui suivent la signature dans la docstring:
    retire les lignes vides en debut/fin, et dedente au minimum d'indentation commun."""
    while desc_lines and desc_lines[0].strip() == "":
        desc_lines.pop(0)
    while desc_lines and desc_lines[-1].strip() == "":
        desc_lines.pop()
    if not desc_lines:
        return ""

    indents = [len(ln) - len(ln.lstrip()) for ln in desc_lines if ln.strip()]
    min_indent = min(indents) if indents else 0
    dedented = [ln[min_indent:] if len(ln) >= min_indent else ln for ln in desc_lines]
    return "\n".join(dedented)
