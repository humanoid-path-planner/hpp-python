"""Parse a .pyi file into a (header, tree) pair ready for rendering.

The tree is a list of ClassI objects.  The first entry may be a pseudo-class
(is_module=True) that carries top-level functions.
"""

from __future__ import annotations
import sys
from pathlib import Path

from ._models import ClassI, Function, Overload
from ._overrides import METHOD_OVERRIDES, SETTER_TYPE_OVERRIDES, normalize_base_class
from ._patterns import (
    ARG_RE,
    CLASS_ATTR_RE,
    CLASS_RE,
    DEF_RE,
    DEF_SELF_FIRST_RE,
    DOCSTRING_OPEN_RE,
    NESTED_ARG1_RE,
    PROPERTY_RE,
    SETTER_RE,
    SIG_RE,
    STATICMETHOD_RE,
)
from ._text_utils import clean_description, read_docstring


def parse_all_signatures(
    docstring: str, method_name: str
) -> list[tuple[str, list[tuple[str, str]], str]] | None:
    """Cherche TOUTES les lignes 'methodName( (Type)arg, ...) -> Ret' dans la docstring.

    Une docstring Boost.Python peut contenir plusieurs signatures (overloads) separees par
    une ligne vide. Pour les properties/setters, Boost.Python ecrit parfois "None" a la place
    du vrai nom de methode: on accepte aussi ce cas.

    Retourne une liste de (type_retour, [(nom, type), ...], description_nettoyee),
    une entree par signature, ou None si aucune signature n'est parsable."""
    lines = docstring.splitlines()

    sig_indices = []
    for idx, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith(method_name + "("):
            matched_name = method_name
        elif stripped.startswith("None("):
            matched_name = "None"
        else:
            continue
        m = SIG_RE.match(stripped)
        if m and m.group("name") == matched_name:
            sig_indices.append(idx)

    if not sig_indices:
        return None

    results = []
    for k, idx in enumerate(sig_indices):
        stripped = lines[idx].strip()
        m = SIG_RE.match(stripped)
        ret = m.group("ret").strip()
        raw_args = [a.strip() for a in m.group("args").split(",") if a.strip()]
        params: list[tuple[str, str]] = []
        for raw in raw_args:
            am = ARG_RE.match(raw)
            if am:
                pname = am.group("name")
                if pname == "self":
                    # certains bindings nomment (par erreur) un parametre "self" alors
                    # que ce n'est pas le premier -> collision avec le vrai self injecte
                    pname = "self_"
                params.append((pname, am.group("type")))
            else:
                return None  # signature illisible -> abandonner le parsing complet

        next_idx = sig_indices[k + 1] if k + 1 < len(sig_indices) else len(lines)
        description = clean_description(lines[idx + 1 : next_idx])
        results.append((ret, params, description))

    return results


def _parse_def_body(
    lines: list[str], i: int, n: int, fc_name: str
) -> tuple[list | None, int, int, str | None]:
    """Parse ce qui suit une ligne 'def name(...):' a l'index i.
    Retourne (parsed_or_None, index_apres, body_end, raw_doc)."""
    j = i + 1
    while j < n and lines[j].strip() == "":
        j += 1

    if j < n and DOCSTRING_OPEN_RE.match(lines[j]):
        raw_doc, after = read_docstring(lines, j)
        parsed = parse_all_signatures(raw_doc, fc_name)
        return parsed, after, after, raw_doc
    if j < n and lines[j].strip() == "...":
        return None, j + 1, j + 1, None
    return None, j, j, None


def _make_overloads(parsed: list, class_name: str | None) -> list[Overload]:
    overloads = []
    for ret, params, description in parsed:
        is_method = False
        if params and class_name:
            first_type = params[0][1]
            if first_type == class_name or first_type.rsplit(".", 1)[-1] == class_name:
                is_method = True
                params = params[1:]
        overloads.append(Overload(ret, params, description, is_method))
    return overloads


def _make_setter_override_overload(type_str: str) -> Overload:
    """Fabrique un Overload synthetique pour un setter de property dont on connait le
    type uniquement via SETTER_TYPE_OVERRIDES (aucune docstring Boost.Python ne le documente).
    Toujours: 1 argument "value", retour None, is_method=True."""
    return Overload(
        ret="None", params=[("value", type_str)], description="", is_method=True
    )


def _apply_method_override(
    fc: Function, override_key: str, method_overrides: dict
) -> bool:
    """Applique un METHOD_OVERRIDES sur fc si la cle est presente.
    Retourne True si un override a ete applique."""
    sigs = method_overrides.get(override_key)
    if sigs is None:
        return False
    fc.overloads = [Overload(ret, params, "", True) for ret, params in sigs]
    fc.from_docstring = True
    return True


def _nested_class_from_fn(fn: Function, outer_name: str) -> str | None:
    """Return nested class name from parsed overloads (primary signal).

    A function belongs to a nested class when its first overload has is_method=False
    and the first parameter is named 'arg1' with type 'OuterClass.NestedClass'.

    Exception: if any OTHER parameter also has that same NestedClass type, the function
    is a static method of the outer class that takes NestedClass data as argument (e.g.
    copy(Splines, Splines)), not an instance method of the nested class.
    """
    marker = outer_name + "."
    all_overloads = list(fn.overloads) + list(fn.setter_overloads)
    for ov in all_overloads:
        if ov.is_method or not ov.params or ov.params[0][0] != "arg1":
            continue
        type_path = ov.params[0][1]
        idx = type_path.rfind(marker)
        if idx == -1:
            continue
        rest = type_path[idx + len(marker) :]
        if not rest or "." in rest:
            continue
        nested_name = rest
        # Guard: if another parameter also carries the same nested class type,
        # this is likely a real static method of the outer class, not an instance method.
        nested_marker = outer_name + "." + nested_name
        if any(
            nested_marker in t or t.endswith("." + nested_name)
            for _, t in ov.params[1:]
        ):
            return None
        return nested_name
    return None


def _nested_class_from_blocks(blocks: list[list[str]], outer_name: str) -> str | None:
    """Fallback: find nested class name from typed `arg1: "outer.Nested"` in raw def lines.

    Used only for non-docstring functions where overloads are unavailable.
    """
    marker = outer_name + "."
    for block in blocks:
        for line in block:
            m = NESTED_ARG1_RE.search(line)
            if m:
                type_path = m.group(1)
                idx = type_path.rfind(marker)
                if idx != -1:
                    rest = type_path[idx + len(marker) :]
                    if rest and "." not in rest:
                        return rest
    return None


def _is_definite_outer_method(fn: Function) -> bool:
    """True only for non-dunder methods that definitively belong to the outer class."""
    if fn.name.startswith("__") and fn.name.endswith("__"):
        return False
    if fn.from_docstring:
        return bool(fn.overloads and fn.overloads[0].is_method)
    # Raw (non-docstring) functions: check for non-static def with self as first arg
    for block in fn.raw_blocks + fn.setter_raw_blocks:
        if any("@staticmethod" in ln for ln in block):
            continue
        for ln in block:
            if DEF_SELF_FIRST_RE.match(ln):
                return True
    return False


def _make_instance_method(fn: Function) -> None:
    """Convert is_method=False overloads with first param 'arg1' into instance method form."""
    for ov in list(fn.overloads) + list(fn.setter_overloads):
        if not ov.is_method and ov.params and ov.params[0][0] == "arg1":
            ov.params = ov.params[1:]
            ov.is_method = True


def _post_process_nested_classes(cls: ClassI) -> None:
    """Detect and move nested-class methods out of a flat ClassI into nested ClassI objects."""
    outer_name = cls.name

    # Step 1: compute hint for each function (nested class name, "" = outer, None = ambiguous)
    hints: list[str | None] = []
    for fn in cls.functions:
        nested = _nested_class_from_fn(fn, outer_name)
        if nested is None and not fn.from_docstring:
            nested = _nested_class_from_blocks(
                fn.raw_blocks + fn.setter_raw_blocks, outer_name
            )
        if nested is not None:
            hints.append(nested)
        elif _is_definite_outer_method(fn):
            hints.append("")
        else:
            hints.append(None)

    if not any(h is not None and h != "" for h in hints):
        return

    # Step 2: fill ambiguous hints from nearest definite neighbour
    n = len(hints)
    filled: list[str] = [""] * n
    for i, h in enumerate(hints):
        if h is not None:
            filled[i] = h
    for i in range(n):
        if hints[i] is not None:
            continue
        prev_h = next(
            (filled[j] for j in range(i - 1, -1, -1) if hints[j] is not None), None
        )
        next_h = next(
            (filled[j] for j in range(i + 1, n) if hints[j] is not None), None
        )
        if prev_h == next_h:
            filled[i] = prev_h if prev_h is not None else ""
        elif prev_h is None:
            filled[i] = next_h if next_h is not None else ""
        elif next_h is None:
            filled[i] = prev_h
        else:
            # Boundary between two blocks: assign to the next block
            filled[i] = next_h

    # Step 3: collect ordered nested class names (first-occurrence order)
    seen: set[str] = set()
    ordered_names: list[str] = []
    for h in filled:
        if h and h not in seen:
            seen.add(h)
            ordered_names.append(h)

    if not ordered_names:
        return

    # Step 4: group functions and convert nested-class methods to instance methods
    outer_fns: list[Function] = []
    nested_fn_map: dict[str, list[Function]] = {name: [] for name in ordered_names}

    for fn, h in zip(cls.functions, filled):
        if h == "":
            outer_fns.append(fn)
        else:
            _make_instance_method(fn)
            nested_fn_map[h].append(fn)

    # Step 5: distribute __instance_size__ extra_lines to nested classes (in order)
    size_extras = [e for e in cls.extra_lines if "__instance_size__" in e]
    other_extras = [e for e in cls.extra_lines if "__instance_size__" not in e]
    cls.extra_lines = other_extras + size_extras[:1]
    nested_sizes = size_extras[1:]

    # Step 6: build nested ClassI objects
    nested_classes: list[ClassI] = []
    for i, name in enumerate(ordered_names):
        nested_cls = ClassI()
        nested_cls.name = name
        nested_cls.functions = nested_fn_map[name]
        if i < len(nested_sizes):
            nested_cls.extra_lines = [nested_sizes[i]]
        nested_classes.append(nested_cls)

    cls.nested_classes = nested_classes
    cls.functions = outer_fns


def extract_tree(
    path: Path,
    setter_overrides: dict[str, str] | None = None,
    method_overrides: dict | None = None,
) -> tuple[str, list[ClassI]]:
    """Retourne (header, tree). Le header contient tout ce qui precede la 1ere classe
    ou la 1ere fonction de niveau module (imports, __future__, __all__, etc.).

    setter_overrides: mapping "ClassName.propName" -> type string, consulte quand un
    setter de property n'a pas de docstring parsable.
    method_overrides: mapping "ClassName.method" -> [(ret, [(name, type), ...])] pour
    les methodes generees sans docstring (ex: map_indexing_suite).__"""
    if setter_overrides is None:
        setter_overrides = SETTER_TYPE_OVERRIDES
    if method_overrides is None:
        method_overrides = METHOD_OVERRIDES

    lines = path.read_text().splitlines()
    tree: list[ClassI] = []
    current: ClassI | None = None
    class_name: str | None = None
    module_root = ClassI(is_module=True)
    module_root.name = ""

    n = len(lines)
    header_end = n
    for idx, ln in enumerate(lines):
        if (
            CLASS_RE.match(ln)
            or DEF_RE.match(ln)
            or PROPERTY_RE.match(ln)
            or STATICMETHOD_RE.match(ln)
            or SETTER_RE.match(ln)
        ):
            header_end = idx
            break
    header = "\n".join(lines[:header_end]).rstrip("\n")

    i = header_end
    while i < n:
        line = lines[i]

        cm = CLASS_RE.match(line)
        if cm:
            class_name = cm.group(1)
            current = ClassI()
            current.name = class_name
            bases_raw = cm.group("bases") or ""
            raw_bases = [b.strip() for b in bases_raw.split(",") if b.strip()]
            current.bases = [normalize_base_class(b) for b in raw_bases]
            tree.append(current)
            i += 1

            j = i
            while j < n and lines[j].strip() == "":
                j += 1
            if j < n and DOCSTRING_OPEN_RE.match(lines[j]):
                doc, after = read_docstring(lines, j)
                current.docstring = clean_description(doc.splitlines())
                i = after
            continue

        if line.strip() and not line[0].isspace() and not cm:
            current = None
            class_name = None

        is_prop = PROPERTY_RE.match(line)
        setter_m = SETTER_RE.match(line)
        static_m = STATICMETHOD_RE.match(line)

        if (
            (is_prop or setter_m or static_m)
            and i + 1 < n
            and DEF_RE.match(lines[i + 1])
        ):
            def_line_idx = i
            def_idx = i + 1
            dm = DEF_RE.match(lines[def_idx])
            fname = dm.group(1)

            parsed, after, body_end, _raw_doc = _parse_def_body(
                lines, def_idx, n, fname
            )
            raw_lines = lines[def_line_idx:body_end]
            i = after

            container = current if current is not None else module_root
            is_method_ctx = class_name if current is not None else None

            if setter_m:
                existing = next(
                    (
                        f
                        for f in container.functions
                        if f.name == fname and f.is_property
                    ),
                    None,
                )
                if existing is None:
                    existing = Function()
                    existing.name = fname
                    existing.is_property = True
                    existing.in_class = current is not None
                    container.functions.append(existing)
                if parsed:
                    existing.setter_overloads = _make_overloads(parsed, is_method_ctx)
                    existing.from_docstring = True
                else:
                    override_key = f"{class_name}.{fname}" if class_name else fname
                    override_type = setter_overrides.get(override_key)
                    if override_type is not None and existing.from_docstring:
                        existing.setter_overloads = [
                            _make_setter_override_overload(override_type)
                        ]
                    elif override_type is not None and not existing.from_docstring:
                        print(
                            f"warning: {override_key}: setter override ignored because "
                            f"the getter itself has no parsable docstring signature -- "
                            f"fix the getter's C++ docstring first (see mask example)",
                            file=sys.stderr,
                        )
                        existing.setter_raw_blocks.append(raw_lines)
                    else:
                        existing.setter_raw_blocks.append(raw_lines)
                continue

            fc = Function()
            fc.name = fname
            fc.raw_lines = raw_lines
            fc.in_class = current is not None
            override_key = f"{class_name}.{fname}" if class_name else fname
            if _apply_method_override(fc, override_key, method_overrides):
                pass  # override applied, parsed is ignored
            elif parsed:
                fc.from_docstring = True
                if fname == "__init__":
                    fc.overloads = [
                        Overload(ret, params[1:], description, True)
                        for ret, params, description in parsed
                    ]
                else:
                    fc.overloads = _make_overloads(parsed, is_method_ctx)
            if is_prop:
                fc.is_property = True
            container.functions.append(fc)
            continue

        dm = DEF_RE.match(line)
        if dm:
            fc_name = dm.group(1)
            def_line_idx = i
            parsed, after, body_end, _raw_doc = _parse_def_body(lines, i, n, fc_name)
            raw_lines = lines[def_line_idx:body_end]
            i = after

            container = current if current is not None else module_root
            is_method_ctx = class_name if current is not None else None

            fc = Function()
            fc.name = fc_name
            fc.raw_lines = raw_lines
            fc.in_class = current is not None

            override_key = f"{class_name}.{fc_name}" if class_name else fc_name
            if _apply_method_override(fc, override_key, method_overrides):
                pass  # override applied, parsed is ignored
            elif parsed:
                fc.from_docstring = True
                if fc_name == "__init__":
                    fc.overloads = [
                        Overload(ret, params[1:], description, True)
                        for ret, params, description in parsed
                    ]
                else:
                    fc.overloads = _make_overloads(parsed, is_method_ctx)

            # Fusionner les occurrences repetees du meme nom dans la meme classe.
            # On ne devine jamais "property" ici (seulement si @property/@X.setter
            # etaient presents litteralement), pour preserver l'idempotence.
            existing = next((f for f in container.functions if f.name == fc_name), None)
            if existing is not None and existing.from_docstring and fc.from_docstring:
                existing.overloads.extend(fc.overloads)
                continue
            if (
                existing is not None
                and existing.from_docstring
                and not fc.from_docstring
            ):
                continue
            if (
                existing is not None
                and not existing.from_docstring
                and not fc.from_docstring
            ):
                existing.raw_blocks.extend(fc.raw_blocks)
                continue

            container.functions.append(fc)
            continue

        if current is not None and CLASS_ATTR_RE.match(line) and line.strip():
            stripped = line.strip()
            if not stripped.startswith("__slots__"):
                current.extra_lines.append(stripped)
            i += 1
            continue

        i += 1

    if module_root.functions:
        tree.insert(0, module_root)

    for cls in tree:
        if not cls.is_module:
            _post_process_nested_classes(cls)

    return header, tree
