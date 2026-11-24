"""Renders a USD model (e.g. isaac/assets/x4.usd) from the front, side and top to PNG files.

    isaac/isaac_python.sh isaac/preview_model.py isaac/assets/x4.usd --out /tmp/x4
"""
import argparse
import math
import os

parser = argparse.ArgumentParser()
parser.add_argument("usd")
parser.add_argument("--out", default="/tmp/preview")
parser.add_argument("--distance", type=float, default=1.6)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdLux.DomeLight.Define(stage, "/World/sky").CreateIntensityAttr(1000.0)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(2500.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 20, 0))
model = stage.DefinePrim("/World/model", "Xform")
model.GetReferences().AddReference(os.path.abspath(args.usd))  # the Isaac python.sh changes the cwd
d = args.distance
views = {"front": (d, 0, 0.3), "side": (0, d, 0.3), "top": (0.01, 0, d)}
for name, pos in views.items():
    cam = rep.create.camera(position=pos, look_at=(0, 0, 0))
    rp = rep.create.render_product(cam, (640, 480))
    ann = rep.AnnotatorRegistry.get_annotator("rgb")
    ann.attach(rp)
    for _ in range(30):
        rep.orchestrator.step()
    img = np.asarray(ann.get_data())[:, :, :3]
    Image.fromarray(img).save(f"{args.out}_{name}.png")
    print(f"[preview] {args.out}_{name}.png", flush=True)
app.close()
