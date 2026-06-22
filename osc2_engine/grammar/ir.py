"""
Intermediate Representation (IR) dataclasses for parsed OpenSCENARIO 2.0 files.

These dataclasses form the contract between the parser and the execution engine.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class PhysicalValue:
    """A numeric value with a physical unit (e.g., 35m, 5mps, 30kph, 180deg)."""
    value: float
    unit: str  # "m", "kph", "mps", "s", "km", "ms", "deg"

    def __repr__(self):
        return f"{self.value}{self.unit}"


@dataclass
class Expression:
    """
    Simple expression tree for trigger conditions.

    Examples:
        ego.object_distance(ped) <= 35m
        -> Expression(op="<=",
             left=Expression(op="call", value="object_distance",
                             left="ego", right=[Expression(op="ref", value="ped")]),
             right=Expression(op="literal", value=PhysicalValue(35, "m")))
    """
    op: str  # "<=", ">=", "==", "!=", "<", ">", "and", "or", "not",
             # "call", "literal", "ref", "+", "-", "*", "/"
    left: Any = None
    right: Any = None
    value: Any = None  # For "literal" and "ref" nodes

    def __repr__(self):
        if self.op == "literal":
            return f"Lit({self.value})"
        if self.op == "ref":
            return f"Ref({self.value})"
        if self.op == "call":
            return f"{self.left}.{self.value}({self.right})"
        return f"({self.left} {self.op} {self.right})"


@dataclass
class Modifier:
    """A modifier inside a with: block (e.g., speed(30kph), carla_behavior(mode: "cautious"))."""
    name: str
    args: Dict[str, Any] = field(default_factory=dict)
    positional_args: List[Any] = field(default_factory=list)

    def __repr__(self):
        parts = [f"{k}={v}" for k, v in self.args.items()]
        parts += [repr(a) for a in self.positional_args]
        return f"{self.name}({', '.join(parts)})"


@dataclass
class KeepConstraint:
    """
    A keep() constraint on an actor or struct instance.
    e.g., keep(it.blueprint == "vehicle.lincoln.mkz")
    """
    property_name: str        # "blueprint", "ros_name", etc.
    op: str                   # "==", "in"
    value: Any                # literal value or RangeValue for "in"


@dataclass
class ActionPhase:
    """
    An action invocation on an actor.
    e.g., ego.drive(), ped.walk(), ego.change_speed(target: 0kph)
    """
    actor_ref: str
    action_name: str
    args: Dict[str, Any] = field(default_factory=dict)
    modifiers: List[Modifier] = field(default_factory=list)
    label: Optional[str] = None

    def get_modifier(self, name: str) -> Optional[Modifier]:
        for m in self.modifiers:
            if m.name == name:
                return m
        return None


@dataclass
class WaitPhase:
    """A wait directive: wait <condition>."""
    condition: Expression


@dataclass
class CompositionPhase:
    """
    A composition block: serial, parallel, or one_of.
    Children can be ActionPhase, WaitPhase, or nested CompositionPhase.
    """
    mode: str  # "serial", "parallel", "one_of"
    children: List[Any] = field(default_factory=list)
    label: Optional[str] = None


@dataclass
class ActorDecl:
    """
    Top-level actor type declaration (legacy format).
    e.g., actor ego_car inherits vehicle: ...
    """
    name: str
    parent_type: str
    fields: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ExtendDecl:
    """
    Extend declaration: adds fields to an existing standard type.
    e.g., extend vehicle: blueprint: string = "" ...
    """
    target_type: str
    fields: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ModifierDecl:
    """
    Extension modifier type declaration.
    e.g., modifier carla_behavior: mode: string = "cautious"
    """
    name: str
    fields: Dict[str, Any] = field(default_factory=dict)


@dataclass
class StructDecl:
    """
    Top-level struct declaration (used for sensor configs).
    e.g., struct carla_rgb_camera: ...
    """
    name: str
    fields: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ActorInstance:
    """
    Actor instantiation inside a scenario.
    e.g., ego: vehicle with: keep(it.blueprint == "...")
    """
    instance_name: str
    type_name: str
    constraints: List[KeepConstraint] = field(default_factory=list)


@dataclass
class ParamDecl:
    """
    Parameter declaration inside a scenario.
    e.g., ped_trigger_distance: length = 35m
    """
    name: str
    type_name: str
    default_value: Any = None
    unit: Optional[str] = None


@dataclass
class ScenarioIR:
    """
    Top-level IR representing a complete parsed .osc file.
    """
    name: str
    actor_decls: List[ActorDecl] = field(default_factory=list)
    struct_decls: List[StructDecl] = field(default_factory=list)
    extend_decls: List[ExtendDecl] = field(default_factory=list)
    modifier_decls: List[ModifierDecl] = field(default_factory=list)
    actor_instances: List[ActorInstance] = field(default_factory=list)
    params: List[ParamDecl] = field(default_factory=list)
    do_block: Optional[CompositionPhase] = None
    imports: List[str] = field(default_factory=list)

    def get_actor_decl(self, type_name: str) -> Optional[ActorDecl]:
        for decl in self.actor_decls:
            if decl.name == type_name:
                return decl
        return None

    def get_struct_decl(self, type_name: str) -> Optional[StructDecl]:
        for decl in self.struct_decls:
            if decl.name == type_name:
                return decl
        return None

    def get_extended_fields(self, base_type: str) -> Dict[str, Any]:
        """Merge all extend declarations for a base type."""
        merged = {}
        for ext in self.extend_decls:
            if ext.target_type == base_type:
                merged.update(ext.fields)
        return merged

    def get_param(self, name: str, default=None) -> Any:
        for p in self.params:
            if p.name == name:
                if isinstance(p.default_value, PhysicalValue):
                    return p.default_value.value
                return p.default_value
        return default
