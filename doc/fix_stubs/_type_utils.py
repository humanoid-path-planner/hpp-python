"""Utilities for mapping Boost.Python type strings to valid Python type annotations."""

from __future__ import annotations

from ._patterns import BUILTIN_TYPES


def quote_type(t: str) -> str:
    """Met entre guillemets (forward reference) tout type qui n'est pas un builtin simple,
    pour eviter que Pyright/mypy tente de le resoudre immediatement (ex: noms qui collisionnent
    avec une fonction de meme nom au niveau module, types dans un autre fichier .pyi, etc.)."""
    if t in BUILTIN_TYPES:
        return t
    if t == "NoneType":
        return "None"
    # certains types Boost.Python (souvent des templates C++ mal resolus) se terminent
    # par un point parasite, ex: "pyhpp.core.path.bindings.SplineB3." -> retirer ce point,
    # sinon la forward-reference n'est pas une expression de type valide pour Pyright
    t = t.rstrip(".")
    # Les expressions de type generiques (ex: typing.Iterator[str], list[bool]) sont deja
    # syntaxiquement valides et ne doivent pas etre encadrees de guillemets: Pyright les
    # interprete correctement telles quelles, et les guillemets les rendraient invalides.
    if "[" in t:
        return t
    return f'"{t}"'


def type_module(t: str) -> str | None:
    """Extrait le module a importer pour un type dotte, ex:
    'pyhpp.core.bindings.Distance' -> 'pyhpp.core.bindings'
    'numpy.ndarray' -> 'numpy'
    'Transition' (pas de point) -> None (type local, pas d'import necessaire)
    'pyhpp.core.path.bindings.SplineB3.' -> 'pyhpp.core.path.bindings' (le point final artefact est ignore)."""
    if t in BUILTIN_TYPES or t == "NoneType":
        return None
    t = t.rstrip(".")
    if "." not in t:
        return None
    return t.rsplit(".", 1)[0]
