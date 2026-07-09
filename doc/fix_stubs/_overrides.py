"""Setter type overrides, method signature overrides, and base class substitutions.

SETTER_TYPE_OVERRIDES
    Boost.Python's add_property() never generates an auto-signature for the setter half of a
    property (only the getter gets one).  This table maps "ClassName.propertyName" -> annotated
    type string, consulted when a setter has no parsable docstring.

METHOD_OVERRIDES
    map_indexing_suite / vector_indexing_suite generate __getitem__, __setitem__, __iter__, etc.
    with generic "object" types and no docstrings.  This table maps "ClassName.method" to an
    explicit list of (ret, [(param_name, param_type), ...]) tuples (one per overload).

BASE_CLASS_SUBSTITUTIONS
    Replaces Boost.Python artifact base classes (unresolvable by Pyright) with native equivalents.
"""

from __future__ import annotations

import json
from pathlib import Path

# Each entry: list of (ret_type, [(param_name, param_type), ...])
# All entries are instance methods (is_method=True implicitly).
# Use "typing.Iterator[str]" etc. for complex type expressions -- quote_type won't double-quote them.
_MethodSig = list[tuple[str, list[tuple[str, str]]]]

METHOD_OVERRIDES: dict[str, _MethodSig] = {
    # HandleMap = std::map<std::string, HandlePtr_t> via map_indexing_suite (manipulation)
    "HandleMap.__getitem__": [("Handle", [("key", "str")])],
    "HandleMap.__setitem__": [("None", [("key", "str"), ("value", "Handle")])],
    "HandleMap.__delitem__": [("None", [("key", "str")])],
    "HandleMap.__iter__": [("typing.Iterator[str]", [])],
    # GripperMap = std::map<std::string, GripperPtr_t> via map_indexing_suite (pinocchio)
    "GripperMap.__getitem__": [("Gripper", [("key", "str")])],
    "GripperMap.__setitem__": [("None", [("key", "str"), ("value", "Gripper")])],
    "GripperMap.__delitem__": [("None", [("key", "str")])],
    "GripperMap.__iter__": [("typing.Iterator[str]", [])],
}

SETTER_TYPE_OVERRIDES: dict[str, str] = {
    "Handle.mask": "list[bool]",
    "Handle.maskComp": "list[bool]",
    "Handle.localPosition": "pyhpp.pinocchio.bindings.Transform3s",
    "Handle.approachingDirection": "numpy.ndarray",
    "Handle.clearance": "float",
    "Handle.name": "str",
}

# Pyright cannot resolve these Boost.Python internal base classes; replace with native equivalents.
# "Boost.Python.enum" behaves like IntEnum at the Python level.
BASE_CLASS_SUBSTITUTIONS: dict[str, str] = {
    "Boost.Python.instance": "object",
    "Boost.Python.enum": "int",
}


def load_method_overrides(path: Path | None) -> dict[str, _MethodSig]:
    """Merge METHOD_OVERRIDES with an optional external JSON file.

    JSON format -- each entry is a single overload or a list of overloads::

        {
            "ClassName.method": {"ret": "RetType", "params": [["name", "Type"], ...]},
            "ClassName.method2": [
                {"ret": "RetType", "params": [["name", "Type"]]},
                {"ret": "RetType2", "params": []}
            ]
        }

    External entries take precedence over the built-in table."""
    merged: dict[str, _MethodSig] = dict(METHOD_OVERRIDES)
    if path is None:
        return merged

    raw = json.loads(path.read_text())
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: expected a JSON object, got {type(raw).__name__}")

    for key, value in raw.items():
        entries = value if isinstance(value, list) else [value]
        sigs: _MethodSig = []
        for entry in entries:
            ret = entry["ret"]
            params = [(p[0], p[1]) for p in entry.get("params", [])]
            sigs.append((ret, params))
        merged[key] = sigs

    return merged


def load_setter_overrides(path: Path | None) -> dict[str, str]:
    """Merge SETTER_TYPE_OVERRIDES with an optional external JSON file.
    The JSON file must be a flat object: {"Class.prop": "type", ...}.
    External entries take precedence over the built-in table."""
    merged = dict(SETTER_TYPE_OVERRIDES)
    if path is not None:
        data = json.loads(path.read_text())
        if not isinstance(data, dict):
            raise ValueError(
                f"{path}: expected a flat JSON object, got {type(data).__name__}"
            )
        merged.update(data)
    return merged


def normalize_base_class(base: str) -> str:
    return BASE_CLASS_SUBSTITUTIONS.get(base, base)
