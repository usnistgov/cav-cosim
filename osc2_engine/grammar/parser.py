"""
OSC2 Parser: Lark-based parser for the ASAM OpenSCENARIO 2.0 grammar subset.

Parses .osc files into ScenarioIR dataclasses. Supports import resolution
for extension libraries (e.g., lib/carla.osc).
"""

import re
from pathlib import Path

from lark import Lark, Transformer, Token, Tree
from lark.indenter import Indenter

from grammar.ir import (
    ScenarioIR, ActorDecl, ExtendDecl, ModifierDecl, StructDecl,
    ActorInstance, ParamDecl, PhysicalValue, ActionPhase, WaitPhase,
    CompositionPhase, Modifier, Expression, KeepConstraint,
)


class OSC2Indenter(Indenter):
    """Handle Python-like indentation for OSC2 files."""
    NL_type = "_NL"
    OPEN_PAREN_types = ["LPAR"]
    CLOSE_PAREN_types = ["RPAR"]
    INDENT_type = "_INDENT"
    DEDENT_type = "_DEDENT"
    tab_len = 4


class OSC2Transformer(Transformer):
    """Transform Lark parse tree into ScenarioIR dataclasses."""

    # --- Top level ---

    def start(self, items):
        actor_decls = []
        struct_decls = []
        extend_decls = []
        modifier_decls = []
        imports = []
        scenario = None
        for item in items:
            if isinstance(item, ActorDecl):
                actor_decls.append(item)
            elif isinstance(item, StructDecl):
                struct_decls.append(item)
            elif isinstance(item, ExtendDecl):
                extend_decls.append(item)
            elif isinstance(item, ModifierDecl):
                modifier_decls.append(item)
            elif isinstance(item, ScenarioIR):
                scenario = item
            elif isinstance(item, tuple) and item[0] == "__import__":
                imports.append(item[1])
        if scenario is None:
            scenario = ScenarioIR(name="unnamed")
        scenario.actor_decls = actor_decls
        scenario.struct_decls = struct_decls
        scenario.extend_decls = extend_decls
        scenario.modifier_decls = modifier_decls
        scenario.imports = imports
        return scenario

    def import_stmt(self, items):
        dotted = items[0]
        path = ".".join(dotted) if isinstance(dotted, list) else str(dotted)
        return ("__import__", path)

    # --- Actor / Extend / Modifier / Struct declarations ---

    def actor_decl(self, items):
        name = str(items[0])
        parent = str(items[1])
        fields = self._collect_fields(items[2:])
        return ActorDecl(name=name, parent_type=parent, fields=fields)

    def extend_decl(self, items):
        target_type = str(items[0])
        fields = self._collect_fields(items[1:])
        return ExtendDecl(target_type=target_type, fields=fields)

    def modifier_decl(self, items):
        name = str(items[0])
        fields = self._collect_fields(items[1:])
        return ModifierDecl(name=name, fields=fields)

    def struct_decl(self, items):
        name = str(items[0])
        fields = self._collect_fields(items[1:])
        return StructDecl(name=name, fields=fields)

    def _collect_fields(self, items):
        fields = {}
        for item in items:
            if isinstance(item, tuple) and len(item) == 2:
                fields[item[0]] = item[1]
        return fields

    def member(self, items):
        name = str(items[0])
        if len(items) >= 3:
            return (name, items[2])
        else:
            return (name, None)

    def type_ref(self, items):
        return str(items[0])

    # --- Scenario ---

    def scenario_decl(self, items):
        name = str(items[0])
        body = items[1]
        if isinstance(body, ScenarioIR):
            body.name = name
            return body
        return body

    def scenario_body(self, items):
        instances = []
        params = []
        do_block = None
        for item in items:
            if isinstance(item, ActorInstance):
                instances.append(item)
            elif isinstance(item, ParamDecl):
                params.append(item)
            elif isinstance(item, CompositionPhase):
                do_block = item
        return ScenarioIR(
            name="",
            actor_instances=instances,
            params=params,
            do_block=do_block,
        )

    def constrained_instance(self, items):
        name = str(items[0])
        type_name = str(items[1])
        constraints = items[2] if len(items) > 2 and isinstance(items[2], list) else []
        return ActorInstance(instance_name=name, type_name=type_name, constraints=constraints)

    def actor_instance(self, items):
        return ActorInstance(instance_name=str(items[0]), type_name=str(items[1]))

    def param_decl(self, items):
        name = str(items[0])
        type_name = str(items[1])
        default_val = items[2] if len(items) > 2 else None
        unit = default_val.unit if isinstance(default_val, PhysicalValue) else None
        return ParamDecl(name=name, type_name=type_name, default_value=default_val, unit=unit)

    # --- Keep constraints ---

    def with_constraint_block(self, items):
        return [i for i in items if isinstance(i, KeepConstraint)]

    def keep_constraint(self, items):
        return items[0]

    def keep_expr(self, items):
        # items: "it", ".", property_name, COMP_OP, expression
        # After transformer: property_name (str), op (str), value
        prop_name = str(items[0])
        op = str(items[1])
        value = items[2]
        return KeepConstraint(property_name=prop_name, op=op, value=value)

    # --- Do / Composition ---

    def do_block(self, items):
        mode = items[0]
        children = [i for i in items[1:] if i is not None]
        return CompositionPhase(mode=mode, children=children)

    def comp_serial(self, items):
        return "serial"

    def comp_parallel(self, items):
        return "parallel"

    def comp_one_of(self, items):
        return "one_of"

    def composition_block(self, items):
        mode = items[0]
        children = [i for i in items[1:] if i is not None]
        return CompositionPhase(mode=mode, children=children)

    # --- Phase labels ---

    def labeled_phase(self, items):
        label = str(items[0])
        phase = items[1]
        if isinstance(phase, (CompositionPhase, ActionPhase)):
            phase.label = label
        return phase

    # --- Action ---

    def action_stmt(self, items):
        dotted = items[0]
        if len(dotted) >= 2:
            actor_ref = ".".join(dotted[:-1])
            action_name = dotted[-1]
        else:
            actor_ref = ""
            action_name = dotted[0]

        args = {}
        modifiers = []
        for item in items[1:]:
            if isinstance(item, list) and item and isinstance(item[0], Modifier):
                modifiers = item
            elif isinstance(item, list):
                for a in item:
                    if isinstance(a, tuple):
                        args[a[0]] = a[1]

        return ActionPhase(
            actor_ref=actor_ref,
            action_name=action_name,
            args=args,
            modifiers=modifiers,
        )

    # --- Wait ---

    def wait_stmt(self, items):
        return WaitPhase(condition=items[0])

    # --- With block (action modifiers) ---

    def with_block(self, items):
        return [i for i in items if isinstance(i, Modifier)]

    def modifier(self, items):
        name = str(items[0])
        args = {}
        positional = []
        if len(items) > 1 and isinstance(items[1], list):
            for a in items[1]:
                if isinstance(a, tuple):
                    args[a[0]] = a[1]
                else:
                    positional.append(a)
        return Modifier(name=name, args=args, positional_args=positional)

    # --- Argument lists ---

    def arg_list(self, items):
        return list(items)

    def named_arg(self, items):
        return (str(items[0]), items[1])

    def positional_arg(self, items):
        return items[0]

    # --- Expressions ---

    def or_expr(self, items):
        if len(items) == 1:
            return items[0]
        result = items[0]
        for i in range(1, len(items)):
            result = Expression(op="or", left=result, right=items[i])
        return result

    def and_expr(self, items):
        if len(items) == 1:
            return items[0]
        result = items[0]
        for i in range(1, len(items)):
            result = Expression(op="and", left=result, right=items[i])
        return result

    def comparison(self, items):
        return Expression(op=str(items[1]), left=items[0], right=items[2])

    def add(self, items):
        return Expression(op="+", left=items[0], right=items[1])

    def sub(self, items):
        return Expression(op="-", left=items[0], right=items[1])

    def mul(self, items):
        return Expression(op="*", left=items[0], right=items[1])

    def div(self, items):
        return Expression(op="/", left=items[0], right=items[1])

    # --- Atoms ---

    def physical_literal(self, items):
        token_str = str(items[0])
        match = re.match(r'^(-?\d+(?:\.\d+)?)(mps|kph|kmh|deg|km|ms|m|s)$', token_str)
        if match:
            return PhysicalValue(value=float(match.group(1)), unit=match.group(2))
        return PhysicalValue(value=float(token_str), unit="")

    def number_literal(self, items):
        val = str(items[0])
        return float(val) if '.' in val else int(val)

    def string_literal(self, items):
        s = str(items[0])
        if (s.startswith('"') and s.endswith('"')) or (s.startswith("'") and s.endswith("'")):
            return s[1:-1]
        return s

    def true_literal(self, items):
        return True

    def false_literal(self, items):
        return False

    def func_call(self, items):
        dotted = items[0]
        args_list = items[1] if len(items) > 1 else []
        if len(dotted) >= 2:
            obj = ".".join(dotted[:-1])
            method = dotted[-1]
        else:
            obj = None
            method = dotted[0]
        return Expression(
            op="call",
            left=obj,
            value=method,
            right=args_list if isinstance(args_list, list) else [args_list],
        )

    def name_ref(self, items):
        dotted = items[0]
        return Expression(op="ref", value=".".join(dotted))

    def dotted_name(self, items):
        return [str(t) for t in items]


class OSC2Parser:
    """Parse .osc files into ScenarioIR using Lark with import resolution."""

    def __init__(self):
        grammar_path = Path(__file__).parent / "osc2_subset.lark"
        self._lark = Lark(
            grammar_path.read_text(),
            parser="earley",
            postlex=OSC2Indenter(),
            propagate_positions=True,
            maybe_placeholders=False,
        )
        self._transformer = OSC2Transformer()

    def parse_file(self, filepath: str) -> ScenarioIR:
        """Parse an .osc file, resolve imports, and return the scenario IR."""
        text = Path(filepath).read_text()
        ir = self._parse_text(text)

        # Resolve imports relative to the scenario file location
        base_dir = Path(filepath).parent
        self._resolve_imports(ir, base_dir)

        return ir

    def parse_string(self, text: str) -> ScenarioIR:
        """Parse OSC2 text directly (no import resolution)."""
        return self._parse_text(text)

    def _parse_text(self, text: str) -> ScenarioIR:
        if not text.endswith("\n"):
            text += "\n"
        tree = self._lark.parse(text)
        return self._transformer.transform(tree)

    def _resolve_imports(self, scenario_ir, base_dir):
        """Load and merge imported .osc library files."""
        engine_root = Path(__file__).parent.parent

        for imp in scenario_ir.imports:
            if imp == "osc.standard":
                continue  # Built-in standard library (no-op for now)

            # Convert dotted path to file path: lib.carla -> lib/carla.osc
            rel_path = imp.replace(".", "/") + ".osc"

            # Search: relative to scenario file, then relative to engine root
            full_path = base_dir / rel_path
            if not full_path.exists():
                full_path = engine_root / rel_path

            if full_path.exists():
                lib_ir = self._parse_text(full_path.read_text())
                # Merge library declarations into the scenario IR
                scenario_ir.extend_decls.extend(lib_ir.extend_decls)
                scenario_ir.struct_decls.extend(lib_ir.struct_decls)
                scenario_ir.modifier_decls.extend(lib_ir.modifier_decls)
                scenario_ir.actor_decls.extend(lib_ir.actor_decls)
            else:
                import logging
                logging.warning(f"Import not found: {imp} (searched {base_dir / rel_path} and {engine_root / rel_path})")
