"""Target drones in Isaac Sim: visual geometry (build_model) and the dynamics of their Gazebo
models (DYNAMICS: mass and inertia of the SDF, gains of its MulticopterVelocityControl).

Preferred: the Gazebo model itself, converted to USD by import_gz_models.py (assets/<key>.usd).
Fallback: each catalog key becomes a hub, arms and
rotor discs with the same bounding radius as the scenario runner's model_catalog, so the
size seen by the camera is close to Gazebo's. The appearance (colour, hub) is not the same,
which matters for the visual tracker (declared in docs/ISAAC_SIM.md).
"""
import math
import os
import re

from pxr import Gf, Usd, UsdGeom, UsdShade, Sdf

# key: (bounding radius [m], arms, hub size [m], hub colour, rotor colour)
MODELS = {
    "x4": (0.45, 4, (0.20, 0.20, 0.07), (0.15, 0.15, 0.17), (0.05, 0.05, 0.05)),
    "x3": (0.35, 4, (0.16, 0.16, 0.06), (0.75, 0.12, 0.10), (0.08, 0.08, 0.08)),
    "x5": (0.40, 4, (0.18, 0.18, 0.06), (0.10, 0.25, 0.70), (0.08, 0.08, 0.08)),
    "tiltrotor": (0.60, 4, (0.30, 0.12, 0.08), (0.85, 0.85, 0.85), (0.10, 0.10, 0.10)),
}


def _material(stage, path, rgb):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _cube(stage, path, size, center, yaw, mat):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)


def _disc(stage, path, radius, center, mat):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(0.01)
    cyl.CreateAxisAttr("Z")
    UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(*center))
    UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(mat)


# key: mass [kg], diagonal inertia [kg m^2], velocityGain, attitudeGain, angularRateGain,
# maximumLinearAcceleration, maximumLinearVelocity (gz_models/<key>/model.sdf and the models they
# include: X3/X4 from Fuel, x500_base and tiltrotor from PX4). Same meaning as in Gazebo's
# LeeVelocityController: a = velocityGain/mass * dv, torque = -attitudeGain e_R - angularRateGain e_w.
DYNAMICS = {
    "x3": dict(mass=1.5, inertia=(0.0347563, 0.07, 0.0977), vel_gain=(2.7, 2.7, 2.7),
               att_gain=(2.0, 3.0, 0.15), rate_gain=(0.4, 0.52, 0.18), max_acc=(3, 3, 3), max_vel=(10, 10, 5)),
    "x4": dict(mass=3.42, inertia=(0.075, 0.075, 0.148916), vel_gain=(6, 6, 10),
               att_gain=(4, 4, 2), rate_gain=(0.7, 0.7, 0.7), max_acc=(8, 8, 4), max_vel=(10, 10, 5)),
    "x5": dict(mass=2.0, inertia=(0.0216667, 0.0216667, 0.04), vel_gain=(6, 6, 10),
               att_gain=(4, 4, 2), rate_gain=(0.7, 0.7, 0.7), max_acc=(3, 3, 3), max_vel=(10, 10, 5)),
    "tiltrotor": dict(mass=5.0, inertia=(0.197563, 0.1458929, 0.1477), vel_gain=(8.90, 8.90, 14.84),
                      att_gain=(10.54, 7.78, 1.98), rate_gain=(1.84, 1.36, 0.69), max_acc=(3, 3, 3),
                      max_vel=(10, 10, 5)),
}

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")


def build_model(stage, path, model):
    """Defines the model under `path` (an Xform the caller moves) and returns its prim: the
    Gazebo model converted by import_gz_models.py (assets/<key>.usd) when there is one,
    otherwise the simple geometry below."""
    asset = os.path.join(ASSETS, f"{model}.usd")
    if os.path.exists(asset):
        root = UsdGeom.Xform.Define(stage, path)
        visual = stage.DefinePrim(path + "/visual", "Xform")
        visual.GetReferences().AddReference(asset)
        return root.GetPrim()
    if model not in MODELS:
        raise KeyError(f"unknown model '{model}' (known: {sorted(MODELS)})")
    radius, arms, hub, hub_rgb, rotor_rgb = MODELS[model]
    root = UsdGeom.Xform.Define(stage, path)
    hub_mat = _material(stage, path + "/mat_hub", hub_rgb)
    rotor_mat = _material(stage, path + "/mat_rotor", rotor_rgb)
    _cube(stage, path + "/hub", hub, (0, 0, 0), 0.0, hub_mat)
    rotor_r = 0.3 * radius
    arm_len = radius - rotor_r
    for i in range(arms):
        a = math.pi / 4 + i * 2 * math.pi / arms
        c, s = math.cos(a), math.sin(a)
        _cube(stage, f"{path}/arm{i}", (arm_len, 0.025, 0.02), (0.5 * arm_len * c, 0.5 * arm_len * s, 0), a, hub_mat)
        _disc(stage, f"{path}/rotor{i}", rotor_r, (arm_len * c, arm_len * s, 0.04), rotor_mat)
        _cube(stage, f"{path}/leg{i}", (0.015, 0.015, 0.12),
              (0.35 * arm_len * c, 0.35 * arm_len * s, -0.07), 0.0, hub_mat)
    return root.GetPrim()


class Propellers:
    """Visual spin of a target's rotor meshes (the physics does not change: the thrust is applied to
    the body, DynamicDrone.step). Rotors are the prims named `...rotor_<n>_visual` (Gazebo visual
    names kept by import_gz_models.py; fallback geometry: `rotor<n>`), turned about the model z axis
    through the centre of their bounding box. The rate is a visual one: the real ~700 rad/s would
    alias at the camera rate (31 Hz), so hover maps to HOVER_RATE and it scales with sqrt(thrust)."""

    NAME_RE = re.compile(r"rotor_?(\d+)(_visual)?$")
    HOVER_RATE = 30.0      # [rad/s] visual rate at hover (< pi/2 per frame at 31 Hz for 2 blades)
    MAX_RATE = 45.0        # [rad/s]
    SPOOL_TAU = 0.4        # [s] motor spool up/down

    def __init__(self, prim):
        self.rotors = []   # (transform op, base, pivot out, pivot back, direction)
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        it = iter(Usd.PrimRange(prim))
        for p in it:
            m = self.NAME_RE.search(p.GetName())
            if p == prim or not m or not p.IsA(UsdGeom.Xformable):
                continue
            it.PruneChildren()
            box = bbox.ComputeUntransformedBound(p).ComputeAlignedRange()
            if box.IsEmpty():
                continue
            xf = UsdGeom.Xformable(p)
            base = xf.GetLocalTransformation()
            centre = base.Transform(box.GetMidpoint())          # in the parent frame
            op = xf.MakeMatrixXform()
            op.Set(base)
            self.rotors.append((op, base, Gf.Matrix4d().SetTranslate(-centre),
                                Gf.Matrix4d().SetTranslate(centre), 1.0 if int(m.group(1)) % 2 == 0 else -1.0))
        self.rate = 0.0
        self.angle = 0.0
        self._written = 0.0

    def step(self, dt, thrust_ratio):
        """thrust_ratio: collective thrust / weight (0 = motors off)."""
        target = min(self.MAX_RATE, self.HOVER_RATE * math.sqrt(max(thrust_ratio, 0.0)))
        self.rate += min(1.0, dt / self.SPOOL_TAU) * (target - self.rate)
        self.angle = math.fmod(self.angle + self.rate * dt, 2.0 * math.pi)

    def apply(self):
        """Writes the rotor transforms (once per render)."""
        if not self.rotors or self.angle == self._written:
            return                                    # stopped: nothing to write
        deg = math.degrees(self.angle)
        for op, base, out, back, direction in self.rotors:
            spin = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), direction * deg))
            op.Set(base * out * spin * back)
        self._written = self.angle
