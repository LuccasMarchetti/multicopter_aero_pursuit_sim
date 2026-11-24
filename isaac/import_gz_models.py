"""Converts Gazebo SDF models (the target drones of the catalog) to USD for Isaac Sim.

    isaac/isaac_python.sh isaac/import_gz_models.py x4=<dir of model.sdf> [x3=<dir> ...] [--out DIR]
        [--model-path DIR ...]

Every <visual> of every <link> becomes a prim at pose(link) * pose(visual): COLLADA/OBJ/STL meshes
through omni.kit.asset_converter (materials and textures kept), boxes, cylinders and spheres as
USD primitives, coloured with the SDF <material> (diffuse/ambient or a Gazebo/<Colour> script) when
the mesh brings none. <include> (model://name) is followed recursively at the include pose (or the
included model's own pose, as an SDF merge does). model://name/<path> is looked up in the model
path (--model-path, GZ_SIM_RESOURCE_PATH, the PX4 gz models, the repository gz_models), Fuel
https://.../files/<path> and relative URIs inside the model directory. The result,
<out>/<key>.usd, is what isaac/target_models.py references for that catalog key, so the target
looks as it does in Gazebo (only the visuals; no physics).
"""
import argparse
import asyncio
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument("models", nargs="+", help="chave=diretório do model.sdf")
parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets"))
parser.add_argument("--model-path", nargs="*", default=[], help="pastas com <modelo>/model.sdf")
args = parser.parse_args()

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = [os.path.expanduser(d) for d in args.model_path] + \
    [d for d in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") if d] + [
        os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/models"),
        os.path.join(HERE, "..", "src", "ros2_ws", "src", "aero_pursuit", "gz_models"),   # host stage
        os.path.join(HERE, "..", "ros2_ws", "src", "aero_pursuit", "gz_models")]          # repository
# Gazebo material scripts used by the catalog models (OGRE names -> RGB).
SCRIPT_COLOURS = {"Gazebo/DarkGrey": (0.175, 0.175, 0.175), "Gazebo/Grey": (0.5, 0.5, 0.5),
                  "Gazebo/Black": (0.05, 0.05, 0.05), "Gazebo/White": (1.0, 1.0, 1.0),
                  "Gazebo/Blue": (0.0, 0.0, 1.0), "Gazebo/Red": (1.0, 0.0, 0.0),
                  "Gazebo/Green": (0.0, 1.0, 0.0), "Gazebo/Orange": (1.0, 0.5, 0.0)}

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.kit.asset_converter")
app.update()
import omni.kit.asset_converter as converter  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

from scipy.spatial.transform import Rotation  # noqa: E402

MESH_EXT = (".dae", ".obj", ".stl", ".fbx", ".gltf", ".glb")


def pose_matrix(text):
    """SDF <pose> 'x y z roll pitch yaw' -> 4x4 (column vectors)."""
    v = [float(t) for t in (text or "0 0 0 0 0 0").split()]
    v += [0.0] * (6 - len(v))
    r = Rotation.from_euler("xyz", v[3:6]).as_matrix()   # SDF: extrinsic roll, pitch, yaw
    # Gf matrices act on row vectors: the rotation goes in transposed.
    rot = Gf.Matrix3d(*[float(r[j][i]) for i in range(3) for j in range(3)])
    return Gf.Matrix4d(rot, Gf.Vec3d(*v[:3]))


def find_model(name):
    for d in MODEL_PATH:
        if os.path.isfile(os.path.join(d, name, "model.sdf")):
            return os.path.abspath(os.path.join(d, name))
    return None


def resolve(uri, model_dir):
    """Local file of a mesh URI: model://<model>/<path> in the current model directory when it
    is there, else in the model path; Fuel .../files/<path> and relative paths in it."""
    uri = uri.strip()
    if uri.startswith("model://"):
        name, _, rel = uri[len("model://"):].partition("/")
        if os.path.exists(os.path.join(model_dir, rel)):
            return os.path.join(model_dir, rel)           # the model's own files (Fuel copies)
        return os.path.join(find_model(name) or model_dir, rel)
    if "/files/" in uri:
        return os.path.join(model_dir, uri.split("/files/", 1)[1])
    return uri if os.path.isabs(uri) else os.path.join(model_dir, uri)


def ogre_material(vis, model_dir):
    """Texture and alpha cut of a visual's Gazebo (OGRE) material script, e.g. OakTree/Branch:
    (texture path or None, alpha_rejection threshold 0..1 or None). Gazebo colours the trees of
    the Fuel catalog this way; their COLLADA files reference no texture."""
    name = (vis.findtext("material/script/name") or "").strip()
    if not name:
        return None, None
    dirs = [resolve(u.text or "", model_dir) for u in vis.findall("material/script/uri")]
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if not f.endswith(".material"):
                continue
            text = open(os.path.join(d, f), errors="replace").read()
            m = re.search(r"material\s+" + re.escape(name) + r"\s*\{(.*?)(?=\nmaterial\s|\Z)", text, re.S)
            if not m:
                continue
            tex = re.search(r"\btexture\s+(\S+)", m.group(1))
            cut = re.search(r"alpha_rejection\s+\S+\s+(\d+)", m.group(1))
            path = None
            if tex:
                for t in dirs:
                    if os.path.isfile(os.path.join(t, tex.group(1))):
                        path = os.path.join(t, tex.group(1))
            return path, (int(cut.group(1)) / 255.0 if cut else None)
    return None, None


def apply_texture(asset_path, submesh, texture, cut):
    """Textures the converted materials whose prim name contains the submesh name (the COLLADA
    material of that part, e.g. Branch_1-material): OmniPBR diffuse texture and, with an alpha
    cut, opacity from the texture alpha (leaves)."""
    import shutil
    tex_dir = os.path.join(os.path.dirname(asset_path), "textures")
    os.makedirs(tex_dir, exist_ok=True)
    shutil.copy(texture, tex_dir)
    rel = "./textures/" + os.path.basename(texture)
    st = Usd.Stage.Open(asset_path)
    n = 0
    for p in st.Traverse():
        if not p.IsA(UsdShade.Shader) or submesh.lower() not in str(p.GetPath()).lower():
            continue
        sh = UsdShade.Shader(p)
        sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(rel))
        sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1, 1, 1))
        if cut is not None:
            sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateInput("enable_opacity_texture", Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateInput("opacity_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(rel))
            sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)        # alpha channel
            sh.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(float(cut))
        n += 1
    st.GetRootLayer().Save()
    return n


def visual_colour(vis):
    """RGB of a visual's SDF material (diffuse, else ambient, else a Gazebo script name)."""
    mat = vis.find("material")
    if mat is None:
        return None
    for tag in ("diffuse", "ambient"):
        text = mat.findtext(tag)
        if text:
            return tuple(float(v) for v in text.split()[:3])
    return SCRIPT_COLOURS.get((mat.findtext("script/name") or "").strip())


def collect_visuals(model, model_dir, parent_m, out):
    """(model_dir, visual element, world matrix) of the links of `model` and of its includes."""
    for link in model.findall("link"):
        link_m = pose_matrix(link.findtext("pose")) * parent_m
        for vis in link.findall("visual"):
            out.append((model_dir, vis, pose_matrix(vis.findtext("pose")) * link_m))
    for inc in model.findall("include"):
        uri = (inc.findtext("uri") or "").strip()
        if not uri.startswith("model://"):
            print(f"[import] non-local include ignored: {uri} (download it and pass the model folder)", flush=True)
            continue
        sub_dir = find_model(uri[len("model://"):].split("/")[0])
        if sub_dir is None:
            print(f"[import] model not found in the path: {uri}", flush=True)
            continue
        sub = ET.parse(os.path.join(sub_dir, "model.sdf")).getroot().find("model")
        pose = inc.findtext("pose") or sub.findtext("pose")
        collect_visuals(sub, sub_dir, pose_matrix(pose) * parent_m, out)


async def convert(src, dst):
    ctx = converter.AssetConverterContext()
    ctx.ignore_materials = False
    ctx.ignore_animations = True
    ctx.ignore_camera = True
    ctx.ignore_light = True
    ctx.single_mesh = False
    ctx.smooth_normals = True
    ctx.use_meter_as_world_unit = True
    ctx.create_world_as_default_root_prim = True
    task = converter.get_instance().create_converter_task(src, dst, None, ctx)
    ok = await task.wait_until_finished()
    if not ok:
        raise RuntimeError(f"{src}: {task.get_status()} {task.get_error_message()}")


def needs_native_reader(src):
    """COLLADA exported by MeshLab (VCGLib) or with geometry only: the asset converter segfaults on
    them (tinyxml2, e.g. x8_wing.dae of standard_vtol and the X3 propellers), so native_dae reads
    them (geometry and flat diffuse colours; their textures, rare in the catalog, are dropped)."""
    if not src.lower().endswith(".dae"):
        return False
    with open(src, "r", errors="replace") as f:
        head = f.read(4096)
        rest = head + f.read()
    return "MeshLab" in head or not any(t in rest for t in ("<library_images", "<library_effects"))


def native_dae(src, dst):
    """COLLADA triangles/polylist -> one UsdGeom.Mesh per primitive group, with a
    UsdPreviewSurface of the material's diffuse colour. Raw COLLADA coordinates and <unit>, as the
    converter and Gazebo read them; node transforms ignored (MeshLab writes identity nodes).
    Default prim /World, like the converter output."""
    root = ET.parse(src).getroot()
    nsuri = root.tag[1:].split("}")[0] if root.tag.startswith("{") else ""

    def q(path):
        return "/".join(f"{{{nsuri}}}{t}" if nsuri and t not in (".", "") else t for t in path.split("/"))

    unit = root.find(q("asset/unit"))
    meter = float(unit.get("meter", 1.0)) if unit is not None else 1.0
    # material id -> RGB (effect <diffuse><color>); symbol -> material id (instance_material)
    effects = {}
    for eff in root.iter(q("effect")):
        col = next(eff.iter(q("diffuse")), None)
        col = col.find(q("color")) if col is not None else None
        if col is not None:
            effects[eff.get("id")] = tuple(float(v) for v in col.text.split()[:3])
    mat_rgb = {}
    for mat in root.iter(q("material")):
        inst = mat.find(q("instance_effect"))
        if inst is not None and inst.get("url", "").lstrip("#") in effects:
            mat_rgb[mat.get("id")] = effects[inst.get("url").lstrip("#")]
    symbols = {im.get("symbol"): im.get("target", "").lstrip("#") for im in root.iter(q("instance_material"))}

    stage = Usd.Stage.CreateNew(dst)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, meter)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    looks = {}
    n = 0
    for geom in root.iter(q("geometry")):
        mesh = geom.find(q("mesh"))
        if mesh is None:
            continue
        sources = {s_.get("id"): s_ for s_ in mesh.findall(q("source"))}
        vin = next(i for i in mesh.find(q("vertices")).findall(q("input")) if i.get("semantic") == "POSITION")
        flat = [float(v) for v in sources[vin.get("source").lstrip("#")].find(q("float_array")).text.split()]
        points = [Gf.Vec3f(*flat[i:i + 3]) for i in range(0, len(flat) - 2, 3)]
        for prim in list(mesh.findall(q("triangles"))) + list(mesh.findall(q("polylist"))):
            inputs = prim.findall(q("input"))
            stride = max(int(i.get("offset", 0)) for i in inputs) + 1
            voff = next(int(i.get("offset", 0)) for i in inputs if i.get("semantic") == "VERTEX")
            p = [int(v) for v in prim.find(q("p")).text.split()]
            if prim.tag.endswith("triangles"):
                vcount = [3] * int(prim.get("count"))
            else:
                vcount = [int(v) for v in prim.find(q("vcount")).text.split()]
            m = UsdGeom.Mesh.Define(stage, f"/World/mesh_{n:02d}")
            n += 1
            m.CreatePointsAttr(points)
            m.CreateFaceVertexCountsAttr(vcount)
            m.CreateFaceVertexIndicesAttr(p[voff::stride][:sum(vcount)])
            m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            m.CreateDoubleSidedAttr(True)
            rgb = mat_rgb.get(symbols.get(prim.get("material"), prim.get("material")))
            if rgb is not None:
                if rgb not in looks:
                    looks[rgb] = material(stage, f"/World/Looks/mat_{len(looks):02d}", rgb)
                UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(looks[rgb])
    stage.GetRootLayer().Save()


def run(coro):
    loop = asyncio.get_event_loop()
    fut = asyncio.ensure_future(coro)
    while not fut.done():
        app.update()
        loop.run_until_complete(asyncio.sleep(0.01))
    return fut.result()


def asset_fix(path):
    """Metres per unit and (informative) up axis of a converted asset. Also makes its materials
    opaque: the FBX COLLADA exporter writes <transparent opaque="RGB_ZERO"> + transparency 0
    (opaque, and Gazebo draws it so), which the converter turns into OmniPBR enable_opacity
    with the texture as cutout, leaving the meshes invisible."""
    st = Usd.Stage.Open(path)
    changed = False
    for p in st.Traverse():
        if p.IsA(UsdShade.Shader):
            sh = UsdShade.Shader(p)
            # enable_opacity_texture: the x500 CF.png (dark carbon fibre) also became its own
            # opacity map, leaving the X5 frame almost transparent.
            for name in ("enable_opacity", "enable_opacity_texture"):
                inp = sh.GetInput(name)
                if inp and inp.Get():
                    inp.Set(False)
                    changed = True
            # Mean colour of the diffuse texture as the constant: what shows if the texture does not
            # load (the x500 frame, CF.png, rendered white = diffuse_color_constant 1,1,1).
            tex, const = sh.GetInput("diffuse_texture"), sh.GetInput("diffuse_color_constant")
            if tex and const and tex.Get():
                img = os.path.join(os.path.dirname(path), tex.Get().path)
                try:
                    from PIL import Image
                    mean = Image.open(img).convert("RGB").resize((1, 1), Image.BOX).getpixel((0, 0))
                    const.Set(Gf.Vec3f(*[c / 255.0 for c in mean]))
                    changed = True
                except OSError as e:
                    print(f"[import] textura {img}: {e}", flush=True)
            if p.GetAttribute("info:id").Get() == "UsdPreviewSurface" and sh.GetInput("opacity"):
                sh.GetInput("opacity").Set(1.0)
                changed = True
    if changed:
        st.GetRootLayer().Save()
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    up = UsdGeom.GetStageUpAxis(st)
    return mpu, up


def material(stage, path, rgb):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def build(key, model_dir, out_dir):
    sdf = ET.parse(os.path.join(model_dir, "model.sdf")).getroot()
    model = sdf.find("model")
    mesh_dir = os.path.join(out_dir, f"{key}_meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    out = os.path.join(out_dir, f"{key}.usd")
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/model")
    stage.SetDefaultPrim(root.GetPrim())
    grey = material(stage, "/model/mat_grey", (0.35, 0.35, 0.37))
    colours = {}

    def colour_mat(rgb):
        if rgb is None:
            return grey
        if rgb not in colours:
            colours[rgb] = material(stage, f"/model/mat_{len(colours):02d}", rgb)
        return colours[rgb]

    converted = {}
    placed = set()          # (mesh file, transform): Gazebo visuals that only pick a submesh
    textured = set()        # (mesh file, submesh) already textured from the OGRE script
    n = 0
    visuals = []
    collect_visuals(model, model_dir, Gf.Matrix4d(1.0), visuals)   # top-level <pose>: the spawn's
    for vis_dir, vis, m in visuals:
        geo = vis.find("geometry")
        if geo is None or geo.find("plane") is not None:
            continue                                  # decals (textured planes): left out
        path = f"/model/v{n:02d}_{vis.get('name', 'visual')}".replace(" ", "_")
        n += 1
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTransformOp().Set(m)
        mat = colour_mat(visual_colour(vis))
        mesh = geo.find("mesh")
        if mesh is not None and mesh.find("submesh") is not None:
            src = resolve(mesh.findtext("uri"), vis_dir)
            sub = (mesh.findtext("submesh/name") or "").strip()
            texture, cut = ogre_material(vis, vis_dir)
            place = (src, tuple(round(v, 6) for row in m for v in row))
            if place in placed:
                # Same file at the same place: the mesh is already there (all submeshes); only
                # this part's material is left to apply once the file is converted.
                if texture and (src, sub) not in textured and src in converted:
                    apply_texture(converted[src][0], sub, texture, cut)
                    textured.add((src, sub))
                stage.RemovePrim(path)
                continue
            placed.add(place)
        if mesh is not None:
            # On the visual: weaker than the asset's own materials (COLLADA), used by STL.
            UsdShade.MaterialBindingAPI.Apply(xf.GetPrim()).Bind(mat)
            src = resolve(mesh.findtext("uri"), vis_dir)
            if not src.lower().endswith(MESH_EXT) or not os.path.exists(src):
                print(f"[import] {key}: malha ignorada {src}", flush=True)
                continue
            if src not in converted:
                dst = os.path.join(mesh_dir, os.path.splitext(os.path.basename(src))[0] + ".usd")
                if needs_native_reader(src):
                    print(f"[import] {key}: own reader (MeshLab/geometry only) {src}", flush=True)
                    native_dae(src, dst)
                else:
                    print(f"[import] {key}: convertendo {src}", flush=True)
                    run(convert(src, dst))
                converted[src] = (dst, *asset_fix(dst))
            dst, mpu, up = converted[src]
            sub = (mesh.findtext("submesh/name") or "").strip()
            texture, cut = ogre_material(vis, vis_dir) if sub else (None, None)
            if texture and (src, sub) not in textured:
                print(f"[import] {key}: {sub} <- {os.path.basename(texture)}"
                      f"{f' (recorte alfa {cut:.2f})' if cut is not None else ''}", flush=True)
                apply_texture(dst, sub, texture, cut)
                textured.add((src, sub))
            scale = [float(s) for s in (mesh.findtext("scale") or "1 1 1").split()]
            inner = UsdGeom.Xform.Define(stage, path + "/asset")
            # The converter keeps the COLLADA coordinates (Z up, as Gazebo reads them) even
            # though it tags the stage Y up: no axis change, only the unit.
            inner.AddScaleOp().Set(Gf.Vec3f(*[s * mpu for s in scale]))
            inner.GetPrim().GetReferences().AddReference(os.path.relpath(dst, out_dir))
        elif geo.find("box") is not None:
            size = [float(s) for s in geo.findtext("box/size").split()]
            c = UsdGeom.Cube.Define(stage, path + "/box")
            c.CreateSizeAttr(1.0)
            c.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
        elif geo.find("cylinder") is not None:
            c = UsdGeom.Cylinder.Define(stage, path + "/cylinder")
            c.CreateRadiusAttr(float(geo.findtext("cylinder/radius")))
            c.CreateHeightAttr(float(geo.findtext("cylinder/length")))
            c.CreateAxisAttr("Z")
            UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
        elif geo.find("sphere") is not None:
            c = UsdGeom.Sphere.Define(stage, path + "/sphere")
            c.CreateRadiusAttr(float(geo.findtext("sphere/radius")))
            UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
    stage.GetRootLayer().Save()
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(root.GetPrim()).ComputeAlignedRange()
    size = box.GetSize()
    print(f"[import] {key}: {n} visuals, {len(converted)} meshes -> {out}; box {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m",
          flush=True)


os.makedirs(args.out, exist_ok=True)
for item in args.models:
    key, _, d = item.partition("=")
    build(key, os.path.abspath(os.path.expanduser(d)), args.out)
app.close()
