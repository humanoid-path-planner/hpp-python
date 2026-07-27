#!/usr/bin/env python3
"""Generate code-oriented Markdown API reference from pyhpp .pyi stubs."""

from __future__ import annotations

import ast
import html as _html
import keyword
import re
import sys
import argparse
from pathlib import Path

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BOOST_SIG_RE = re.compile(r"^[\w][\w:]*\s*\(.*\)\s*->\s*\S")
BOOST_RET_RE = re.compile(r"->\s*(\S+?)(?:\s*:)?\s*$")
SKIP_NAMES = {"__reduce__", "__instance_size__", "__hash__"}
SKIP_BASES = {"object", "instance"}
NOT_INSTANTIABLE = re.compile(
    r"raises an exception|cannot be instantiated from python", re.IGNORECASE
)
PARAM_RE = re.compile(r"^:param\s+(\w+):\s*(.*)")
RETURNS_RE = re.compile(r"^:returns?:\s*(.*)")
TYPE_RE = re.compile(r"^:(?:type|rtype)\b")
SECTION_RE = re.compile(
    r"^(inputs?|outputs?|parameters?|args?|arguments?)s?:?\s*$", re.IGNORECASE
)
KV_RE = re.compile(r"^([A-Za-z]\w*): +(.+)$")

MODULES = [
    (
        "pyhpp/core/bindings.pyi",
        "pyhpp.core",
        "Path planning problem, planners, optimizers, roadmap, steering methods.",
    ),
    (
        "pyhpp/core/path/bindings.pyi",
        "pyhpp.core.path",
        "Concrete path types: StraightPath, PathVector, splines.",
    ),
    (
        "pyhpp/core/path_optimization/bindings.pyi",
        "pyhpp.core.path_optimization",
        "Concrete path optimizer implementations.",
    ),
    (
        "pyhpp/core/problem_target/bindings.pyi",
        "pyhpp.core.problem_target",
        "Problem target types (goal configurations, etc.).",
    ),
    (
        "pyhpp/constraints/bindings.pyi",
        "pyhpp.constraints",
        "Differentiable functions, implicit/explicit constraints, hierarchical solver.",
    ),
    (
        "pyhpp/manipulation/bindings.pyi",
        "pyhpp.manipulation",
        "Manipulation-specific Device, constraint graph, path planners.",
    ),
    (
        "pyhpp/manipulation/urdf/bindings.pyi",
        "pyhpp.manipulation.urdf",
        "URDF/SRDF loading for manipulation devices.",
    ),
    (
        "pyhpp/manipulation/steering_method/bindings.pyi",
        "pyhpp.manipulation.steering_method",
        "Manipulation-specific steering methods.",
    ),
    (
        "pyhpp/pinocchio/bindings.pyi",
        "pyhpp.pinocchio",
        "Robot model (Device), Lie-group utilities.",
    ),
    (
        "pyhpp/pinocchio/urdf/bindings.pyi",
        "pyhpp.pinocchio.urdf",
        "URDF/SRDF loading for pinocchio devices.",
    ),
]


def slug(module_name: str) -> str:
    return module_name.replace(".", "-").removeprefix("pyhpp-")


# ---------------------------------------------------------------------------
# Math / Doxygen → KaTeX
# ---------------------------------------------------------------------------

_MATH_ENV_RE = re.compile(r"\\begin\{(\w+\*?)\}.*?\\end\{\1\}", re.DOTALL)


def _protect_math_envs(raw: str) -> tuple[str, list[str]]:
    stashed: list[str] = []

    def _stash(m: re.Match[str]) -> str:
        stashed.append(m.group(0))
        return f"\x00MATHENV{len(stashed) - 1}\x00"

    return _MATH_ENV_RE.sub(_stash, raw), stashed


def _restore_math_envs(text: str, stashed: list[str]) -> str:
    for i, body in enumerate(stashed):
        text = text.replace(f"\x00MATHENV{i}\x00", body)
    return text


def doxygen_to_katex(text: str) -> str:
    text = re.sub(r"\\begin\{eqnarray(\*?)\}", r"\\begin{align\1}", text)
    text = re.sub(r"\\end\{eqnarray(\*?)\}", r"\\end{align\1}", text)
    text = re.sub(r"\\mbox\s*\{", r"\\text{", text)

    def _collapse(body: str) -> str:
        body = re.sub(r"\s*\n\s*", " ", body)
        return re.sub(r"\s+", " ", body).strip()

    bracket_blocks: list[str] = []

    def _stash_brackets(m: re.Match[str]) -> str:
        bracket_blocks.append(_collapse(m.group(1)))
        return f"\x00BRACKETBLOCK{len(bracket_blocks) - 1}\x00"

    text = re.sub(r"\\\[(.*?)\\\]", _stash_brackets, text, flags=re.DOTALL)

    dollar_blocks: list[str] = []

    def _stash_dollars(m: re.Match[str]) -> str:
        body = m.group(1)
        if "\n" not in body or "\\begin{" not in body:
            return m.group(0)
        dollar_blocks.append(_collapse(body))
        return f"\x00DOLLARBLOCK{len(dollar_blocks) - 1}\x00"

    text = re.sub(
        r"(?<!\$)\$(?!\$)(.*?)(?<!\$)\$(?!\$)", _stash_dollars, text, flags=re.DOTALL
    )

    def _wrap(m: re.Match[str]) -> str:
        env, body = m.group(1), m.group(2)
        body = _collapse(body)
        return f"\n\n$$\\begin{{{env}}}{body}\\end{{{env}}}$$\n\n"

    text = re.sub(r"\\begin\{(\w+\*?)\}(.*?)\\end\{\1\}", _wrap, text, flags=re.DOTALL)

    for i, body in enumerate(bracket_blocks):
        text = text.replace(f"\x00BRACKETBLOCK{i}\x00", f"\n\n$${body}$$\n\n")
    for i, body in enumerate(dollar_blocks):
        text = text.replace(f"\x00DOLLARBLOCK{i}\x00", f"\n\n$${body}$$\n\n")

    return text


# ---------------------------------------------------------------------------
# Docstring parsing
# ---------------------------------------------------------------------------


def _format_line(s: str) -> str | None:
    if TYPE_RE.match(s):
        return None
    m = PARAM_RE.match(s)
    if m:
        return f"**{m.group(1)}** — {m.group(2)}"
    m = RETURNS_RE.match(s)
    if m:
        return f"**Returns** — {m.group(1)}"
    if SECTION_RE.match(s):
        return f"*{s.rstrip(':').capitalize()}:*"
    m = KV_RE.match(s)
    if m and " " not in m.group(1):
        return f"**{m.group(1)}** — {m.group(2)}"
    return s.replace("<", "&lt;").replace(">", "&gt;").replace("|", "\\|")


def parse_overloads(raw: str | None) -> list[tuple[str | None, str]]:
    if not raw:
        return []
    result: list[tuple[str | None, str]] = []
    current_ret: str | None = None
    current: list[str] = []
    for line in raw.splitlines():
        s = line.strip()
        if BOOST_SIG_RE.match(s):
            if current:
                text = "\n".join(current).rstrip()
                if text:
                    result.append((current_ret, text))
                current = []
            m = BOOST_RET_RE.search(s)
            current_ret = m.group(1) if m else None
        elif not s:
            if current and current[-1] != "":
                current.append("")
        else:
            formatted = _format_line(s)
            if formatted is not None:
                current.append(formatted)
    if current:
        text = "\n".join(current).rstrip()
        if text:
            result.append((current_ret, text))
    return result


def overloads_to_cell(overloads: list[tuple[str | None, str]]) -> str:
    if not overloads:
        return ""
    if len(overloads) == 1:
        lines = []
        for line in overloads[0][1].split("\n"):
            s = line.strip()
            if not s:
                continue
            if not re.match(r"^[-*]\s+\\", s):
                s = re.sub(r"^[-*]\s+", "", s)
            lines.append(s)
        return " <br> ".join(lines)
    parts = []
    for i, (_, text) in enumerate(overloads, 1):
        lines = []
        for line in text.split("\n"):
            s = line.strip()
            if not s:
                continue
            if not re.match(r"^[-*]\s+\\", s):
                s = re.sub(r"^[-*]\s+", "", s)
            lines.append(s)
        parts.append(f"**{i}.** {' <br> '.join(lines)}")
    return " <br> ".join(parts)


# ---------------------------------------------------------------------------
# AST helpers
# ---------------------------------------------------------------------------

_KW_PARAM_RE = re.compile(
    r"([,(]\s*)(" + "|".join(re.escape(kw) for kw in keyword.kwlist) + r")(\s*:)"
)


def _parse_stub(stub_path: Path) -> ast.Module | None:
    try:
        source = stub_path.read_text()
        source = re.sub(r"ClassVar\[[^\]]*\.\]", "ClassVar[Any]", source)
        source = _KW_PARAM_RE.sub(r"\1\2_\3", source)

        return ast.parse(source)
    except Exception:
        return None


def get_docstring(node: ast.AST) -> str | None:
    body = getattr(node, "body", None)
    if (
        body
        and isinstance(body[0], ast.Expr)
        and isinstance(body[0].value, ast.Constant)
        and isinstance(body[0].value.value, str)
    ):
        return body[0].value.value
    return None


def has_decorator(node: ast.FunctionDef, name: str) -> bool:
    for d in node.decorator_list:
        if isinstance(d, ast.Name) and d.id == name:
            return True
        if isinstance(d, ast.Attribute) and d.attr == name:
            return True
    return False


def is_setter(node: ast.FunctionDef) -> bool:
    return any(
        isinstance(d, ast.Attribute) and d.attr == "setter" for d in node.decorator_list
    )


def _unparse_ann(ann: ast.expr | None) -> str:
    """Unparse a type annotation node, stripping surrounding forward-ref quotes."""
    if ann is None:
        return ""
    return ast.unparse(ann).strip("'\"")


# ---------------------------------------------------------------------------
# Class index for type → link resolution
# ---------------------------------------------------------------------------

ClassIndex = dict[str, str]  # type name variants → "page.md#anchor"
ClassDefinitionIndex = dict[str, ast.ClassDef]


def _stub_module_name(relative_stub: str) -> str:
    return str(Path(relative_stub).with_suffix("")).replace("/", ".")


def build_class_index(stubs_root: Path) -> ClassIndex:
    """Scan all stubs and map every known class name to its page#anchor."""
    index: ClassIndex = {}
    for rel_stub, module_name, _ in MODULES:
        stub_path = stubs_root / rel_stub
        if not stub_path.exists():
            continue
        tree = _parse_stub(stub_path)
        if tree is None:
            continue
        page = slug(module_name)
        stub_dotted = _stub_module_name(rel_stub)
        for item in tree.body:
            if not isinstance(item, ast.ClassDef):
                continue
            anchor = item.name.lower()
            target = f"{page}.md#{anchor}"
            for key in (
                item.name,
                f"{module_name}.{item.name}",
                f"{stub_dotted}.{item.name}",
            ):
                index[key] = target
            for sub in item.body:
                if not isinstance(sub, ast.ClassDef):
                    continue
                sub_anchor = sub.name.lower()
                sub_target = f"{page}.md#{sub_anchor}"
                for key in (
                    f"{item.name}.{sub.name}",
                    f"{module_name}.{item.name}.{sub.name}",
                ):
                    index.setdefault(key, sub_target)
                # simple name: setdefault so first occurrence wins
                index.setdefault(sub.name, sub_target)
    return index


def build_class_definition_index(stubs_root: Path) -> ClassDefinitionIndex:
    index: ClassDefinitionIndex = {}

    def add_class(class_node: ast.ClassDef, parent: str) -> None:
        qualified_class = f"{parent}.{class_node.name}"
        index[qualified_class] = class_node
        for item in class_node.body:
            if isinstance(item, ast.ClassDef):
                add_class(item, qualified_class)

    for rel_stub, _, _ in MODULES:
        stub_path = stubs_root / rel_stub
        if not stub_path.exists():
            continue
        tree = _parse_stub(stub_path)
        if tree is None:
            continue
        stub_module = _stub_module_name(rel_stub)
        for item in tree.body:
            if isinstance(item, ast.ClassDef):
                add_class(item, stub_module)
    return index


def _type_link(type_str: str, class_index: ClassIndex, current_page: str) -> str:
    """Markdown link for a type (used outside code blocks, e.g. base-class line)."""
    if not type_str:
        return ""
    t = type_str.strip("'\"")
    simple = t.split(".")[-1]
    for key in (t, simple):
        target = class_index.get(key)
        if target:
            page_part, anchor = target.split("#", 1)
            href = f"#{anchor}" if page_part == f"{current_page}.md" else target
            return f"[`{simple}`]({href})"
    return f"`{simple}`"


# Built-in Python types that get hljs coloring inside the <pre> signature block
_HLJS_BUILTINS: frozenset[str] = frozenset(
    {
        "int",
        "float",
        "str",
        "bool",
        "bytes",
        "bytearray",
        "list",
        "dict",
        "tuple",
        "set",
        "frozenset",
        "object",
        "type",
        "complex",
    }
)
_HLJS_LITERALS: frozenset[str] = frozenset({"None", "True", "False"})


def _fmt_ann(ann: str, class_index: ClassIndex, current_page: str) -> str:
    """Format a type annotation for inside a bare <pre> block.

    Returns one of:
    - ``<a href="…">Full.Type</a>``  for known pyhpp types (clickable link)
    - ``<span class="hljs-built_in">int</span>``  for Python builtins
    - ``<span class="hljs-literal">None</span>``  for None / True / False
    - HTML-escaped plain text for everything else (numpy, pinocchio, …)

    The hljs CSS classes apply for coloring even inside a bare <pre> because
    mdbook loads highlight.css (which targets class names globally).
    highlight.js itself never touches this element since it queries 'pre code'
    and we deliberately omit the inner <code>.
    """
    if not ann:
        return ""
    t = ann.strip("'\"")
    simple = t.split(".")[-1]
    for key in (t, simple):
        target = class_index.get(key)
        if target:
            page_part, anchor = target.split("#", 1)
            href = f"#{anchor}" if page_part == f"{current_page}.md" else target
            return f'<a href="{_html.escape(href)}">{_html.escape(t)}</a>'
    if simple in _HLJS_BUILTINS:
        return f'<span class="hljs-built_in">{_html.escape(t)}</span>'
    if simple in _HLJS_LITERALS:
        return f'<span class="hljs-literal">{_html.escape(t)}</span>'
    return _html.escape(t)


# ---------------------------------------------------------------------------
# Base class link helpers
# ---------------------------------------------------------------------------


def _build_stub_module_index() -> dict[str, str]:
    index: dict[str, str] = {}
    for rel_stub, module_name, _ in MODULES:
        index[_stub_module_name(rel_stub)] = slug(module_name)
    return index


def _base_name(base: ast.expr) -> str:
    if isinstance(base, ast.Attribute):
        return ast.unparse(base)
    if isinstance(base, ast.Name):
        return base.id
    return ""


def _resolve_base(
    base: ast.expr,
    current_stub_module: str,
    class_definitions: ClassDefinitionIndex,
) -> str | None:
    raw = _base_name(base)
    if not raw:
        return None
    qualified = raw if "." in raw else f"{current_stub_module}.{raw}"
    return qualified if qualified in class_definitions else None


def _base_links(
    class_node: ast.ClassDef,
    stub_module_index: dict[str, str],
    class_index: ClassIndex,
    current_page: str,
) -> list[str]:
    links = []
    for b in class_node.bases:
        raw = _base_name(b)
        if not raw:
            continue
        class_name = raw.split(".")[-1]
        if class_name in SKIP_BASES:
            continue
        # Try module-path index first, then class_index (handles same-module bases)
        module_path = raw.rsplit(".", 1)[0] if "." in raw else ""
        page = stub_module_index.get(module_path) or stub_module_index.get(raw)
        if page:
            links.append(f"[`{raw}`]({page}.md#{class_name.lower()})")
        else:
            link = _type_link(raw, class_index, current_page)
            links.append(link)
    return links


# ---------------------------------------------------------------------------
# Body grouping — consecutive same-name FunctionDefs become one overload group
# ---------------------------------------------------------------------------

type BodyGroup = list[ast.FunctionDef] | ast.ClassDef
type ClassMethods = dict[str, tuple[list[ast.FunctionDef], str]]


def _group_body(body: list[ast.stmt]) -> list[BodyGroup]:
    result: list[BodyGroup] = []
    pending: list[ast.FunctionDef] = []
    for item in body:
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if pending and item.name == pending[-1].name:
                pending.append(item)
            else:
                if pending:
                    result.append(pending)
                pending = [item]
        elif isinstance(item, ast.ClassDef):
            if pending:
                result.append(pending)
                pending = []
            result.append(item)
        # Assign (class attrs like __slots__, __instance_size__) — skipped
    if pending:
        result.append(pending)
    return result


def _class_mro(
    qualified_class: str,
    class_definitions: ClassDefinitionIndex,
    cache: dict[str, list[str]],
) -> list[str]:
    if qualified_class in cache:
        return cache[qualified_class]

    class_node = class_definitions[qualified_class]
    current_stub_module = qualified_class.rsplit(".", 1)[0]
    bases = [
        resolved
        for base in class_node.bases
        if (resolved := _resolve_base(base, current_stub_module, class_definitions))
    ]
    sequences = [_class_mro(base, class_definitions, cache).copy() for base in bases]
    sequences.append(bases.copy())

    result = [qualified_class]
    while sequences:
        sequences = [sequence for sequence in sequences if sequence]
        if not sequences:
            break
        candidate = next(
            (
                sequence[0]
                for sequence in sequences
                if not any(sequence[0] in other[1:] for other in sequences)
            ),
            None,
        )
        if candidate is None:
            raise ValueError(f"Inconsistent class hierarchy for {qualified_class}")
        result.append(candidate)
        for sequence in sequences:
            if sequence[0] == candidate:
                sequence.pop(0)

    cache[qualified_class] = result
    return result


def _direct_method_groups(
    class_node: ast.ClassDef,
) -> dict[str, list[ast.FunctionDef]]:
    methods: dict[str, list[ast.FunctionDef]] = {}
    for group in _group_body(class_node.body):
        if isinstance(group, ast.ClassDef):
            continue
        node = group[0]
        methods.setdefault(node.name, []).extend(group)
    return methods


def _collect_class_methods(
    qualified_class: str,
    class_definitions: ClassDefinitionIndex,
    cache: dict[str, ClassMethods],
    mro_cache: dict[str, list[str]],
) -> ClassMethods:
    if qualified_class in cache:
        return cache[qualified_class]

    methods: ClassMethods = {}
    for owner in _class_mro(qualified_class, class_definitions, mro_cache):
        for name, group in _direct_method_groups(class_definitions[owner]).items():
            if name == "__init__" and owner != qualified_class:
                continue
            methods.setdefault(name, (group, owner))

    cache[qualified_class] = methods
    return methods


# ---------------------------------------------------------------------------
# Signature rendering — HTML <pre> with embedded <a> links
# ---------------------------------------------------------------------------

_SIG_WIDTH = 72  # chars before switching to multi-line params


def _render_signature_pre(
    group: list[ast.FunctionDef],
    class_index: ClassIndex,
    current_page: str,
) -> str:
    """Render a signature group as a syntax-coloured <pre> with linked types.

    Strategy:
    - Bare <pre> (no inner <code>) → highlight.js selector 'pre code' never
      matches → <a> links survive untouched in the DOM.
    - Manual <span class="hljs-*"> spans → mdbook loads highlight.css globally
      so those CSS rules apply everywhere, giving us keyword / builtin colours
      without triggering the highlight.js JavaScript engine.
    """
    is_multi_overload = len(group) > 1 and not is_setter(group[-1])
    parts: list[str] = []

    for node in group:
        lines_out: list[str] = []

        # --- decorators ---
        if is_multi_overload:
            lines_out.append('<span class="hljs-meta">@typing.overload</span>')
        for d in node.decorator_list:
            name = ast.unparse(d)
            if name == "typing.overload":
                continue
            lines_out.append(f'<span class="hljs-meta">@{_html.escape(name)}</span>')

        # --- parameters (HTML + plain copy for width estimate) ---
        params_html: list[str] = []
        params_plain: list[str] = []

        for arg in node.args.args:
            ann = _unparse_ann(arg.annotation)
            params_plain.append(f"{arg.arg}: {ann}" if ann else arg.arg)
            if arg.arg == "self":
                part = '<span class="hljs-params">self</span>'
            else:
                part = _html.escape(arg.arg)
            if ann:
                part += f": {_fmt_ann(ann, class_index, current_page)}"
            params_html.append(part)

        if node.args.vararg:
            v = node.args.vararg
            ann = _unparse_ann(v.annotation)
            params_plain.append(f"*{v.arg}: {ann}" if ann else f"*{v.arg}")
            pfx = f"*{_html.escape(v.arg)}"
            params_html.append(
                f"{pfx}: {_fmt_ann(ann, class_index, current_page)}" if ann else pfx
            )

        if node.args.kwarg:
            k = node.args.kwarg
            ann = _unparse_ann(k.annotation)
            params_plain.append(f"**{k.arg}: {ann}" if ann else f"**{k.arg}")
            pfx = f"**{_html.escape(k.arg)}"
            params_html.append(
                f"{pfx}: {_fmt_ann(ann, class_index, current_page)}" if ann else pfx
            )

        # --- return type ---
        ret = _unparse_ann(node.returns)
        ret_html = f" -&gt; {_fmt_ann(ret, class_index, current_page)}" if ret else ""

        # --- assemble: single-line or multi-line ---
        plain_sig = (
            f"def {node.name}({', '.join(params_plain)}){' -> ' + ret if ret else ''}"
        )
        kw = '<span class="hljs-keyword">def</span>'
        fn = f'<span class="hljs-title function_">{_html.escape(node.name)}</span>'

        if len(plain_sig) > _SIG_WIDTH and len(params_html) > 1:
            inner = ",\n    ".join(params_html)
            sig = f"{kw} {fn}(\n    {inner},\n){ret_html}"
        else:
            sig = f"{kw} {fn}({', '.join(params_html)}){ret_html}"

        lines_out.append(sig)
        parts.append("\n".join(lines_out))

    return f"<pre>{''.join(chr(10) + chr(10)).join(parts)}</pre>"


# ---------------------------------------------------------------------------
# Table cell rendering — def | description
# ---------------------------------------------------------------------------


def _sig_cell(
    group: list[ast.FunctionDef],
    class_index: ClassIndex,
    current_page: str,
) -> str:
    """Inline <code> signature(s) for the 'def' column of the method table.

    Multiple overloads are separated by <br><br>.  Types are linked via <a>
    or coloured via hljs spans.  Uses <code> (inline) so it stays valid inside
    a Markdown table cell.
    """
    is_multi_ov = len(group) > 1 and not is_setter(group[-1])
    sig_parts: list[str] = []

    for node in group:
        dec_lines: list[str] = []
        if is_multi_ov:
            dec_lines.append('<span class="hljs-meta">@typing.overload</span>')
        for d in node.decorator_list:
            name = ast.unparse(d)
            if name == "typing.overload":
                continue
            dec_lines.append(f'<span class="hljs-meta">@{_html.escape(name)}</span>')

        params_html: list[str] = []
        for arg in node.args.args:
            ann = _unparse_ann(arg.annotation)
            if arg.arg == "self":
                part = '<span class="hljs-params">self</span>'
            else:
                part = _html.escape(arg.arg)
                if ann:
                    part += f": {_fmt_ann(ann, class_index, current_page)}"
            params_html.append(part)

        if node.args.vararg:
            v = node.args.vararg
            ann = _unparse_ann(v.annotation)
            pfx = f"*{_html.escape(v.arg)}"
            params_html.append(
                f"{pfx}: {_fmt_ann(ann, class_index, current_page)}" if ann else pfx
            )

        if node.args.kwarg:
            k = node.args.kwarg
            ann = _unparse_ann(k.annotation)
            pfx = f"**{_html.escape(k.arg)}"
            params_html.append(
                f"{pfx}: {_fmt_ann(ann, class_index, current_page)}" if ann else pfx
            )

        ret = _unparse_ann(node.returns)
        ret_html = f" -&gt; {_fmt_ann(ret, class_index, current_page)}" if ret else ""

        kw = '<span class="hljs-keyword">def</span>'
        fn = f'<span class="hljs-title function_">{_html.escape(node.name)}</span>'
        params_str = ", ".join(params_html)

        sig_line = f"{kw} {fn}({params_str}){ret_html}"
        inner = "<br>".join(dec_lines + [sig_line])
        sig_parts.append(inner)

    return "<code>" + "<br><br>".join(sig_parts) + "</code>"


def _desc_cell(group: list[ast.FunctionDef]) -> str:
    """First non-empty docstring paragraph as plain text for the description column."""
    for ov in group:
        if is_setter(ov):
            continue
        raw = get_docstring(ov)
        if not raw:
            continue
        protected, stashed = _protect_math_envs(raw)
        overloads = parse_overloads(protected)
        if overloads:
            _, text = overloads[0]
            text = _restore_math_envs(text, stashed)
            first_para = text.split("\n\n")[0]
            flat = " ".join(first_para.split())
            return flat.replace("|", "\\|")
    return ""


# ---------------------------------------------------------------------------
# Class rendering
# ---------------------------------------------------------------------------


def render_class(
    class_node: ast.ClassDef,
    class_index: ClassIndex,
    current_page: str,
    stub_module_index: dict[str, str],
    qualified_class: str,
    class_methods: dict[str, ClassMethods],
    heading_level: int = 2,
) -> tuple[list[str], bool]:
    """Render a class and return (lines, has_content)."""
    raw_doc = get_docstring(class_node)
    bases = _base_links(class_node, stub_module_index, class_index, current_page)
    heading = "#" * heading_level

    groups = _group_body(class_node.body)

    method_rows: list[tuple[str, str]] = []
    nested_class_lines: list[str] = []

    for g in groups:
        if isinstance(g, ast.ClassDef):
            sub_lines, _ = render_class(
                g,
                class_index,
                current_page,
                stub_module_index,
                f"{qualified_class}.{g.name}",
                class_methods,
                heading_level + 1,
            )
            nested_class_lines.extend(sub_lines)

    methods = class_methods.get(qualified_class)
    if methods is None:
        methods = {
            name: (group, qualified_class)
            for name, group in _direct_method_groups(class_node).items()
        }
    for group, owner in methods.values():
        node = group[0]
        if node.name in SKIP_NAMES or is_setter(node):
            continue
        if node.name == "__init__":
            if any(
                get_docstring(ov) and NOT_INSTANTIABLE.search(get_docstring(ov))
                for ov in group
            ):
                continue
        sig = _sig_cell(group, class_index, current_page)
        if owner != qualified_class:
            sig += f" *(inherited from {_type_link(owner, class_index, current_page)})*"
        desc = _desc_cell(group)
        method_rows.append((sig, desc))

    has_content = bool(raw_doc or bases or method_rows or nested_class_lines)
    if not has_content:
        return [], False

    lines: list[str] = [f"{heading} `{class_node.name}`\n"]

    meta: list[str] = []
    if bases:
        meta.append(f"*Inherits: {', '.join(bases)}*")
    if meta:
        lines.append("  ".join(meta) + "\n")

    if raw_doc:
        protected, stashed = _protect_math_envs(raw_doc)
        bq_overloads = parse_overloads(protected)
        doc = overloads_to_cell(bq_overloads)
        if doc:
            doc = _restore_math_envs(doc, stashed)
            doc = doxygen_to_katex(doc.replace(" <br> ", "\n\n"))
            quoted = "\n".join(
                f"> {ln}" if ln.strip() else ">" for ln in doc.splitlines()
            )
            lines.append(quoted + "\n")

    if method_rows:
        lines.append("| def | Description |")
        lines.append("|:---|:---|")
        for sig, desc in method_rows:
            lines.append(f"| {sig} | {desc} |")
        lines.append("")

    if nested_class_lines:
        lines.extend(nested_class_lines)

    if heading_level == 2:
        lines.append("---\n")

    return lines, True


# ---------------------------------------------------------------------------
# Module page generation
# ---------------------------------------------------------------------------


def generate_module_md(
    stub_path: Path,
    module_name: str,
    current_stub_module: str,
    class_index: ClassIndex,
    stub_module_index: dict[str, str],
    class_methods: dict[str, ClassMethods],
) -> str:
    tree = _parse_stub(stub_path)
    if tree is None:
        return f"# `{module_name}`\n\n*Failed to parse stub.*\n"

    current_page = slug(module_name)
    lines: list[str] = [f"# `{module_name}`\n"]
    has_content = False

    # Module-level functions
    func_groups: list[list[ast.FunctionDef]] = []
    pending_funcs: list[ast.FunctionDef] = []
    for item in tree.body:
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if pending_funcs and item.name == pending_funcs[-1].name:
                pending_funcs.append(item)
            else:
                if pending_funcs:
                    func_groups.append(pending_funcs)
                pending_funcs = [item]
        elif isinstance(item, ast.ClassDef):
            if pending_funcs:
                func_groups.append(pending_funcs)
                pending_funcs = []
    if pending_funcs:
        func_groups.append(pending_funcs)

    if func_groups:
        func_rows: list[tuple[str, str]] = []
        for fg in func_groups:
            if fg[0].name in SKIP_NAMES:
                continue
            func_rows.append((_sig_cell(fg, class_index, current_page), _desc_cell(fg)))
        if func_rows:
            lines.append("## Functions\n")
            lines.append("| def | Description |")
            lines.append("|:---|:---|")
            for sig, desc in func_rows:
                lines.append(f"| {sig} | {desc} |")
            lines.append("")
            has_content = True

    for item in tree.body:
        if isinstance(item, ast.ClassDef):
            class_lines, ok = render_class(
                item,
                class_index,
                current_page,
                stub_module_index,
                f"{current_stub_module}.{item.name}",
                class_methods,
            )
            if ok:
                lines.extend(class_lines)
                has_content = True

    if not has_content:
        lines.append("*No documented symbols in this module.*\n")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate code-oriented mdbook Markdown from pyhpp .pyi stubs"
    )
    parser.add_argument(
        "--stubs",
        required=True,
        type=Path,
        help="Python site-packages directory (e.g. install/lib/python3.x/site-packages)",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Output directory for generated .md files",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    class_index = build_class_index(args.stubs)
    class_definitions = build_class_definition_index(args.stubs)
    stub_module_index = _build_stub_module_index()
    class_methods: dict[str, ClassMethods] = {}
    mro_cache: dict[str, list[str]] = {}
    for qualified_class in class_definitions:
        _collect_class_methods(
            qualified_class, class_definitions, class_methods, mro_cache
        )

    generated: list[tuple[str, str, str]] = []
    for rel_stub, module_name, description in MODULES:
        stub_path = args.stubs / rel_stub
        if not stub_path.exists():
            print(f"WARNING: {stub_path} not found, skipping", file=sys.stderr)
            continue
        out_file = args.output / f"{slug(module_name)}.md"
        out_file.write_text(
            generate_module_md(
                stub_path,
                module_name,
                _stub_module_name(rel_stub),
                class_index,
                stub_module_index,
                class_methods,
            )
        )
        generated.append((module_name, slug(module_name), description))
        print(f"  {out_file.name}")

    index_lines = [
        "# pyhpp API Reference\n\n",
        "Python bindings for HPP — auto-generated from `.pyi` stubs.\n\n",
        "| Module | Description |\n",
        "|:---|:---|\n",
    ]
    for mod, s, desc in generated:
        index_lines.append(f"| [`{mod}`]({s}.md) | {desc} |\n")
    (args.output / "index.md").write_text("".join(index_lines))
    print("  index.md")


if __name__ == "__main__":
    main()
