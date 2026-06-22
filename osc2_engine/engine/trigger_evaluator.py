"""
Trigger Evaluator.

Evaluates Expression trees from parsed wait conditions against live CARLA state.
Supports standard OSC2 functions: object_distance(), elapsed()
Legacy functions: distance_to(), distance_to_point()
"""

import logging

from grammar.ir import Expression, PhysicalValue


class TriggerEvaluator:
    """Evaluate Expression trees against live actor state."""

    def __init__(self, actor_manager, params=None):
        self.actors = actor_manager
        self._sim_time = 0.0
        self._params = params or {}

    def advance_time(self, delta):
        """Called by executor each tick to track simulation time."""
        self._sim_time += delta

    def evaluate(self, expr):
        """Recursively evaluate an expression tree."""
        if expr is None:
            return True

        # Pass through non-Expression types (PhysicalValue, int, float, str, etc.)
        if not isinstance(expr, Expression):
            return expr

        op = expr.op

        # Comparison operators
        if op in ("<=", ">=", "==", "!=", "<", ">"):
            left = self.evaluate(expr.left)
            right = self.evaluate(expr.right)
            left_val = self._to_float(left)
            right_val = self._to_float(right)
            if op == "<=":
                return left_val <= right_val
            elif op == ">=":
                return left_val >= right_val
            elif op == "==":
                return left_val == right_val
            elif op == "!=":
                return left_val != right_val
            elif op == "<":
                return left_val < right_val
            elif op == ">":
                return left_val > right_val

        # Logical operators
        if op == "and":
            return self.evaluate(expr.left) and self.evaluate(expr.right)
        if op == "or":
            return self.evaluate(expr.left) or self.evaluate(expr.right)

        # Arithmetic
        if op == "+":
            return self._to_float(self.evaluate(expr.left)) + self._to_float(self.evaluate(expr.right))
        if op == "-":
            return self._to_float(self.evaluate(expr.left)) - self._to_float(self.evaluate(expr.right))
        if op == "*":
            return self._to_float(self.evaluate(expr.left)) * self._to_float(self.evaluate(expr.right))
        if op == "/":
            return self._to_float(self.evaluate(expr.left)) / self._to_float(self.evaluate(expr.right))

        # Literal value
        if op == "literal":
            return expr.value

        # Name reference — handles actor properties like ego.speed
        if op == "ref":
            name = expr.value
            if "." in name:
                parts = name.split(".", 1)
                actor_name, prop = parts[0], parts[1]
                return self._resolve_property(actor_name, prop)
            # Bare name → resolve against scenario params (e.g. d_trigger)
            if name in self._params:
                return self._params[name]
            return name

        # Function call
        if op == "call":
            return self._eval_call(expr)

        raise ValueError(f"Unknown expression op: {op}")

    def _eval_call(self, expr):
        """Evaluate function calls."""
        obj = expr.left
        method = expr.value
        args = expr.right if isinstance(expr.right, list) else [expr.right]

        # Standard: object_distance(actor) or object_distance(point: position_3d(...))
        # Legacy: distance_to(actor), distance_to_point(x:, y:)
        if method in ("object_distance", "distance_to"):
            # Check if first arg is a point (named arg or position_3d call)
            for a in args:
                if isinstance(a, tuple) and a[0] == "point":
                    # object_distance(point: position_3d(x: ..., y: ...))
                    x, y = self._resolve_position_3d(a[1])
                    return self.actors.distance_to_point(obj, x, y)
                if isinstance(a, Expression) and a.op == "call" and a.value == "position_3d":
                    x, y = self._resolve_position_3d(a)
                    return self.actors.distance_to_point(obj, x, y)

            # Default: distance to another actor
            target_name = self._resolve_arg_as_name(args[0])
            return self.actors.distance_between(obj, target_name)

        elif method == "distance_to_point":
            # Legacy: distance_to_point(x: val, y: val)
            x, y = self._resolve_point_args(args)
            return self.actors.distance_to_point(obj, x, y)

        # Standard: elapsed(duration) — time since scenario start
        elif method == "elapsed":
            duration = self._to_float(self.evaluate(args[0]))
            return self._sim_time >= duration

        else:
            raise ValueError(f"Unknown function: {obj}.{method}")

    def _resolve_position_3d(self, expr):
        """Resolve a position_3d(x: val, y: val) call to (x, y)."""
        if isinstance(expr, Expression) and expr.op == "call" and expr.value == "position_3d":
            args = expr.right if isinstance(expr.right, list) else [expr.right]
            return self._resolve_point_args(args)
        if isinstance(expr, list):
            return self._resolve_point_args(expr)
        return 0.0, 0.0

    def _resolve_property(self, actor_name, prop):
        """Resolve an actor property like ego.speed, ego.x, ego.y."""
        if prop == "speed":
            return self.actors.get_speed(actor_name)
        elif prop == "speed_kmh":
            return self.actors.get_speed_kmh(actor_name)
        elif prop in ("x", "y", "z"):
            loc = self.actors.get_location(actor_name)
            return getattr(loc, prop)
        elif prop == "yaw":
            transform = self.actors.get_transform(actor_name)
            return transform.rotation.yaw
        else:
            raise ValueError(f"Unknown property: {actor_name}.{prop}")

    def _resolve_arg_as_name(self, arg):
        """Resolve an argument to an actor name string."""
        if isinstance(arg, Expression):
            if arg.op == "ref":
                return arg.value
            return self.evaluate(arg)
        if isinstance(arg, tuple):
            return arg[1] if isinstance(arg[1], str) else str(arg[1])
        return str(arg)

    def _resolve_point_args(self, args):
        """Extract x, y from named arguments."""
        x = 0.0
        y = 0.0
        for arg in args:
            if isinstance(arg, tuple):
                name, val = arg
                val = self._to_float(val)
                if name == "x":
                    x = val
                elif name == "y":
                    y = val
        return x, y

    def _to_float(self, val):
        """Convert a value to float, handling PhysicalValue."""
        if isinstance(val, PhysicalValue):
            return val.value
        if isinstance(val, (int, float)):
            return float(val)
        if isinstance(val, str):
            try:
                return float(val)
            except ValueError:
                return 0.0
        return float(val)
