#!/usr/bin/env python3
"""Convert Gazebo Classic worlds and models (XTDrone sitl_config) to Gazebo Harmonic.

What it does
  * worlds: adds the gz-sim systems PX4 needs (physics @250 Hz, sensors/ogre2,
    IMU, air pressure, NavSat, contact, user commands, scene broadcaster),
    spherical coordinates, a GUI with the Classic camera pose; drops <state>,
    Classic <gui>, <audio_*> and Classic plugins; turns <road> into flat boxes.
  * models: every model://NAME referenced by the worlds (recursively) is copied
    from the search paths and converted the same way; Classic sensors
    (ray -> gpu_lidar, gpu_ray -> gpu_lidar) are updated.
  * materials: OGRE <script> materials (Gazebo/* and model:// .material files)
    become <ambient>/<diffuse>/<specular> plus a PBR albedo map.
  * file://media/... (Classic install media) is rewritten to
    model://xtdrone_classic_media/..., a model directory built from the
    gazebo-classic media folder given with --classic-media.

Usage
  classic_to_gz.py --worlds W1.world [W2.world ...] --model-path DIR [DIR ...]
                   --classic-media GAZEBO_CLASSIC/media --out OUT_DIR
                   [--skip-models NAME ...]
  Writes OUT_DIR/worlds/<name>.sdf and OUT_DIR/models/<model>/.
"""
import argparse
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

MEDIA_MODEL = "xtdrone_classic_media"

# Classic models that are not in osrf/gazebo_models but exist on Gazebo Fuel.
# Mesh references to Classic assets that are not published anywhere; replaced by
# equivalent Gazebo Classic media files.
URI_REMAP = {
    "model://actor_walk/meshes/SKIN_man_green_shirt.dae": "file://media/models/walk.dae",
    "model://actor_walk/meshes/ANIMATION_walking.dae": "file://media/models/walk.dae",
}

FUEL_MODELS = {
    "sonoma_raceway": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sonoma Raceway",
}

WORLD_SYSTEMS = [
    ("gz::sim::systems::Physics", "gz-sim-physics-system", ""),
    ("gz::sim::systems::UserCommands", "gz-sim-user-commands-system", ""),
    ("gz::sim::systems::SceneBroadcaster", "gz-sim-scene-broadcaster-system", ""),
    ("gz::sim::systems::Contact", "gz-sim-contact-system", ""),
    ("gz::sim::systems::Imu", "gz-sim-imu-system", ""),
    ("gz::sim::systems::AirPressure", "gz-sim-air-pressure-system", ""),
    ("gz::sim::systems::AirSpeed", "gz-sim-air-speed-system", ""),
    ("gz::sim::systems::ApplyLinkWrench", "gz-sim-apply-link-wrench-system", ""),
    ("gz::sim::systems::NavSat", "gz-sim-navsat-system", ""),
    ("gz::sim::systems::Magnetometer", "gz-sim-magnetometer-system", ""),
    ("gz::sim::systems::Sensors", "gz-sim-sensors-system", "<render_engine>ogre2</render_engine>"),
]

GUI_TEMPLATE = """<gui fullscreen="false">
  <plugin filename="MinimalScene" name="3D View">
    <gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui>
    <engine>ogre2</engine><scene>scene</scene>
    <ambient_light>0.4 0.4 0.4</ambient_light><background_color>0.8 0.8 0.8</background_color>
    <camera_pose>{camera_pose}</camera_pose>
  </plugin>
  <plugin filename="GzSceneManager" name="Scene Manager"><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="InteractiveViewControl" name="Interactive view control"><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="CameraTracking" name="Camera Tracking"><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="MarkerManager" name="Marker manager"><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="SelectEntities" name="Select Entities"><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="VisualizationCapabilities" name="Visualization Capabilities"><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="Spawn" name="Spawn Entities"><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
  <plugin filename="WorldControl" name="World control">
    <gz-gui><title>World control</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">72</property><property type="double" key="z">1</property><property type="string" key="state">floating</property>
      <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors></gz-gui>
    <play_pause>true</play_pause><step>true</step><start_paused>false</start_paused>
  </plugin>
  <plugin filename="WorldStats" name="World stats">
    <gz-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">110</property><property type="double" key="width">290</property><property type="double" key="z">1</property><property type="string" key="state">floating</property>
      <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors></gz-gui>
    <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations>
  </plugin>
  <plugin filename="EntityTree" name="Entity tree"/>
  <plugin filename="ComponentInspector" name="Component inspector"/>
</gui>"""

DEFAULT_SPHERICAL = """<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>47.397971057728974</latitude_deg>
  <longitude_deg>8.546163739800146</longitude_deg>
  <elevation>0</elevation>
</spherical_coordinates>"""


# ---------------------------------------------------------------------------
# OGRE material scripts
# ---------------------------------------------------------------------------
class MaterialLibrary:
    def __init__(self):
        self.materials = {}  # name -> dict
        self.loaded = set()

    def load(self, path, texture_dirs):
        path = Path(path)
        if path in self.loaded or not path.is_file():
            return
        self.loaded.add(path)
        text = re.sub(r"//.*", "", path.read_text(errors="ignore"))
        tokens = re.findall(r"[{}]|[^\s{}]+", text)
        i = 0
        while i < len(tokens):
            if tokens[i] == "material" and i + 1 < len(tokens):
                name = tokens[i + 1]
                parent = None
                j = i + 2
                if j < len(tokens) and tokens[j] == ":":
                    parent = tokens[j + 1]
                    j += 2
                # find body
                if j >= len(tokens) or tokens[j] != "{":
                    i += 1
                    continue
                depth, k = 0, j
                body = []
                while k < len(tokens):
                    if tokens[k] == "{":
                        depth += 1
                    elif tokens[k] == "}":
                        depth -= 1
                        if depth == 0:
                            break
                    body.append(tokens[k])
                    k += 1
                mat = dict(self.materials.get(parent, {})) if parent else {}
                mat.update(self._parse_body(body, texture_dirs))
                self.materials[name] = mat
                i = k + 1
            else:
                i += 1

    @staticmethod
    def _parse_body(body, texture_dirs):
        mat = {}
        for idx, tok in enumerate(body):
            if tok in ("ambient", "diffuse", "specular", "emissive") and tok not in mat:
                vals = []
                for t in body[idx + 1: idx + 5]:
                    try:
                        vals.append(float(t))
                    except ValueError:
                        break
                if len(vals) >= 3:
                    if len(vals) == 3:
                        vals.append(1.0)
                    mat[tok] = vals
            elif tok == "texture" and "texture" not in mat and idx + 1 < len(body):
                name = body[idx + 1]
                for d in texture_dirs:
                    if (Path(d) / name).is_file():
                        mat["texture"] = Path(d) / name
                        break
        return mat


# ---------------------------------------------------------------------------
# Converter
# ---------------------------------------------------------------------------
class Converter:
    def __init__(self, model_paths, media_dir, out_dir, skip_models):
        self.model_paths = [Path(p) for p in model_paths]
        self.media_dir = Path(media_dir) if media_dir else None
        self.out = Path(out_dir)
        self.skip = set(skip_models)
        self.materials = MaterialLibrary()
        self.pending = []
        self.done = set()
        self.missing = set()
        self.media_files = set()
        # Classic models whose name is taken by an XTDrone vehicle model (e.g. the PX4
        # Classic 'rover' Polaris mesh used as scenery) are copied as <name>_classic.
        self.renamed = {}
        if self.media_dir:
            self.materials.load(
                self.media_dir / "materials/scripts/gazebo.material",
                [self.media_dir / "materials/textures"],
            )

    # -- paths ---------------------------------------------------------------
    def find_model(self, name):
        name = self.renamed.get(name, name)
        for p in self.model_paths:
            if (p / name).is_dir():
                return p / name
        return None

    def model_uri_for_file(self, f):
        """Turn an absolute texture path into a model:// or copy it into media."""
        f = Path(f)
        for p in self.model_paths:
            try:
                rel = f.relative_to(p)
                return self.rewrite_uri("model://" + rel.as_posix())
            except ValueError:
                pass
        if self.media_dir:
            try:
                rel = f.relative_to(self.media_dir)
                self.media_files.add(rel)
                return f"model://{MEDIA_MODEL}/" + rel.as_posix()
            except ValueError:
                pass
        return f.as_uri()

    def require_model(self, name):
        if name in self.skip or name == MEDIA_MODEL:
            return
        if name not in self.done and name not in self.pending:
            self.pending.append(name)

    def rewrite_uri(self, uri):
        uri = URI_REMAP.get(uri.strip(), uri.strip())
        if uri.startswith("file://media/"):
            rel = uri[len("file://media/"):]
            self.media_files.add(Path(rel))
            return f"model://{MEDIA_MODEL}/{rel}"
        m = re.match(r"model://([^/]+)$", uri)
        if m and m.group(1) in FUEL_MODELS and self.find_model(m.group(1)) is None:
            return FUEL_MODELS[m.group(1)]
        m = re.match(r"model://([^/]+)(/.*)?$", uri)
        if m:
            name, sub = m.group(1), m.group(2) or ""
            # files inside a Classic model whose name is an XTDrone vehicle model
            if name in self.skip and sub:
                if self.find_model(name) is not None:
                    new = f"{name}_classic"
                    self.renamed[new] = name
                    self.require_model(new)
                    return f"model://{new}{sub}"
            self.require_model(name)
        return uri

    # -- materials -----------------------------------------------------------
    def convert_material(self, mat_el, model_dir=None):
        script = mat_el.find("script")
        if script is None:
            return
        name_el = script.find("name")
        name = name_el.text.strip() if name_el is not None and name_el.text else ""
        tex_dirs = []
        for uri_el in script.findall("uri"):
            uri = (uri_el.text or "").strip()
            local = self.resolve_local(uri, model_dir)
            if local is None:
                continue
            if local.is_dir():
                for f in sorted(local.glob("*.material")):
                    self.materials.load(f, [local.parent / "textures", local])
                tex_dirs.append(local)
            elif local.suffix == ".material":
                self.materials.load(local, [local.parent.parent / "textures", local.parent])
        mat_el.remove(script)
        mat = self.materials.materials.get(name)
        if mat is None:
            return
        for key in ("ambient", "diffuse", "specular", "emissive"):
            if key in mat and mat_el.find(key) is None:
                ET.SubElement(mat_el, key).text = " ".join(f"{v:g}" for v in mat[key])
        if "diffuse" not in mat and "ambient" in mat and mat_el.find("diffuse") is None:
            ET.SubElement(mat_el, "diffuse").text = " ".join(f"{v:g}" for v in mat["ambient"])
        if "texture" in mat:
            d = mat_el.find("diffuse")
            if d is None:
                d = ET.SubElement(mat_el, "diffuse")
            d.text = "1 1 1 1"
            pbr = ET.SubElement(mat_el, "pbr")
            metal = ET.SubElement(pbr, "metal")
            ET.SubElement(metal, "albedo_map").text = self.model_uri_for_file(mat["texture"])
            ET.SubElement(metal, "metalness").text = "0"
            ET.SubElement(metal, "roughness").text = "0.9"

    def resolve_local(self, uri, model_dir):
        uri = uri.strip()
        if uri.startswith("model://"):
            rest = uri[len("model://"):]
            name = rest.split("/")[0]
            src = self.find_model(name)
            if src is None:
                return None
            return src.parent / rest
        if uri.startswith("file://media/") and self.media_dir:
            return self.media_dir / uri[len("file://media/"):]
        if model_dir is not None and not re.match(r"^\w+://", uri):
            return Path(model_dir) / uri
        return None

    # -- generic element pass ------------------------------------------------
    def convert_tree(self, root, model_dir=None):
        # Actors positioned by XTDrone's ROS actor plugin (init_pose) get that pose directly.
        # The Classic plugin pose rolled the mesh by 90 deg; Gazebo Harmonic does not need it.
        for actor in root.iter("actor"):
            if actor.find("pose") is not None:
                continue
            for pl in actor.findall("plugin"):
                init = pl.findtext("init_pose")
                if init:
                    v = [float(x) for x in init.split()] + [0.0] * 6
                    ET.SubElement(actor, "pose").text = f"{v[0]:g} {v[1]:g} 1 0 0 {v[5]:g}"
                    break
        # Classic plugins are not loadable in gz-sim.
        for parent in list(root.iter()):
            for child in list(parent):
                if child.tag == "plugin" and (child.get("filename", "").startswith("lib")
                                              or child.get("filename", "").endswith(".so")):
                    parent.remove(child)
                elif child.tag in ("audio_source", "audio_sink"):
                    parent.remove(child)
        for mat in root.iter("material"):
            self.convert_material(mat, model_dir)
        for tag in ("uri", "filename", "diffuse", "normal", "albedo_map"):
            for el in root.iter(tag):
                if el.text and ("://" in el.text):
                    el.text = self.rewrite_uri(el.text)
        # Classic-saved worlds carry deprecated empty frame="" attributes and stray
        # <friction>/<contact>/<bounce> directly under <collision> (they belong in <surface>).
        for el in root.iter("pose"):
            if el.get("frame") == "":
                del el.attrib["frame"]
        for col in root.iter("collision"):
            for tag in ("friction", "contact", "bounce"):
                for stray in col.findall(tag):
                    col.remove(stray)
        # SDF 1.9 rejects duplicate sibling names that Classic accepted.
        for parent in root.iter():
            for tag in ("link", "joint", "collision", "visual", "sensor"):
                seen = {}
                for child in parent.findall(tag):
                    name = child.get("name")
                    if name is None:
                        continue
                    if name in seen:
                        seen[name] += 1
                        child.set("name", f"{name}_{seen[name]}")
                        print(f"warning: duplicate {tag} name '{name}' renamed to '{child.get('name')}'",
                              file=sys.stderr)
                    else:
                        seen[name] = 0
        # DART (Gazebo Harmonic's physics) aborts on zero-size shapes, which Classic accepted.
        for tag, fields in (("box", ("size",)), ("cylinder", ("radius", "length")), ("sphere", ("radius",))):
            for shape in root.iter(tag):
                for field in fields:
                    el = shape.find(field)
                    if el is None or not el.text:
                        continue
                    vals = el.text.split()
                    fixed = [v if float(v) > 0 else "0.001" for v in vals]
                    if fixed != vals:
                        print(f"warning: {tag} {field} '{el.text.strip()}' -> '{' '.join(fixed)}' (zero size)",
                              file=sys.stderr)
                        el.text = " ".join(fixed)
        for sensor in root.iter("sensor"):
            t = sensor.get("type")
            if t in ("ray", "gpu_ray"):
                sensor.set("type", "gpu_lidar")
                ray = sensor.find("ray")
                if ray is not None:
                    ray.tag = "lidar"
            elif t == "multicamera":
                sensor.set("type", "camera")
                cams = sensor.findall("camera")
                for extra in cams[1:]:
                    sensor.remove(extra)
            elif t == "depth":
                sensor.set("type", "depth_camera")

    # -- roads -----------------------------------------------------------------
    def convert_roads(self, world):
        for road in list(world.findall("road")):
            name = road.get("name", "road")
            width = float(road.findtext("width", "6"))
            pts = []
            for p in road.findall("point"):
                xyz = [float(v) for v in p.text.split()]
                pts.append(xyz + [0.0] * (3 - len(xyz)))
            model = ET.Element("model", name=f"road_{name}")
            ET.SubElement(model, "static").text = "true"
            link = ET.SubElement(model, "link", name="link")
            mat_src = road.find("material")
            for i in range(len(pts) - 1):
                (x0, y0, z0), (x1, y1, z1) = pts[i][:3], pts[i + 1][:3]
                length = math.hypot(x1 - x0, y1 - y0)
                if length < 1e-6:
                    continue
                yaw = math.atan2(y1 - y0, x1 - x0)
                vis = ET.SubElement(link, "visual", name=f"segment_{i}")
                ET.SubElement(vis, "pose").text = (
                    f"{(x0 + x1) / 2:g} {(y0 + y1) / 2:g} {(z0 + z1) / 2 + 0.005:g} 0 0 {yaw:g}")
                geom = ET.SubElement(ET.SubElement(vis, "geometry"), "box")
                # overlap segments by half a width so corners are closed
                ET.SubElement(geom, "size").text = f"{length + width / 2:g} {width:g} 0.01"
                if mat_src is not None:
                    vis.append(_deepcopy(mat_src))
                else:
                    m = ET.SubElement(vis, "material")
                    ET.SubElement(m, "ambient").text = "0.2 0.2 0.2 1"
                    ET.SubElement(m, "diffuse").text = "0.2 0.2 0.2 1"
            world.remove(road)
            world.append(model)

    # -- worlds ----------------------------------------------------------------
    def convert_world(self, src, name=None):
        tree = ET.parse(src)
        root = tree.getroot()
        root.set("version", "1.9")
        world = root.find("world")
        name = name or Path(src).stem
        world.set("name", name)

        camera_pose = "-6 0 6 0 0.5 0"
        gui = world.find("gui")
        if gui is not None:
            pose = gui.find("camera/pose")
            if pose is not None and pose.text:
                camera_pose = " ".join(pose.text.split())
            world.remove(gui)
        for tag in ("state", "physics"):
            for el in world.findall(tag):
                world.remove(el)
        self.convert_roads(world)
        self.convert_tree(world)

        # Drop scene sub-elements that gz handles differently.
        scene = world.find("scene")
        if scene is not None:
            for tag in ("fog", "origin_visual", "grid"):
                for el in scene.findall(tag):
                    scene.remove(el)

        insert_at = 0
        physics = ET.fromstring(
            '<physics name="4ms" type="ode"><max_step_size>0.004</max_step_size>'
            "<real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>")
        world.insert(insert_at, physics)
        for i, (pname, fname, inner) in enumerate(WORLD_SYSTEMS, start=1):
            world.insert(insert_at + i, ET.fromstring(f'<plugin name="{pname}" filename="{fname}">{inner}</plugin>'))
        # WindEffects is always loaded so the wind can be changed at runtime (/world/<name>/wind).
        world.insert(insert_at + len(WORLD_SYSTEMS) + 1, ET.fromstring(
            '<plugin name="gz::sim::systems::WindEffects" filename="gz-sim-wind-effects-system">'
            '<force_approximation_scaling_factor>1</force_approximation_scaling_factor></plugin>'))
        world.insert(insert_at + len(WORLD_SYSTEMS) + 1, ET.fromstring(GUI_TEMPLATE.format(camera_pose=camera_pose)))
        sc = world.find("spherical_coordinates")
        if sc is None:
            world.append(ET.fromstring(DEFAULT_SPHERICAL))
        elif sc.find("world_frame_orientation") is None:
            ET.SubElement(sc, "world_frame_orientation").text = "ENU"

        out = self.out / "worlds" / f"{name}.sdf"
        out.parent.mkdir(parents=True, exist_ok=True)
        _indent(root)
        tree.write(out, encoding="unicode", xml_declaration=True)
        return out

    # -- models ----------------------------------------------------------------
    def convert_models(self):
        while self.pending:
            name = self.pending.pop(0)
            self.done.add(name)
            src = self.find_model(name)
            if src is None:
                self.missing.add(name)
                continue
            dst = self.out / "models" / name
            if dst.exists():
                shutil.rmtree(dst)
            shutil.copytree(src, dst, ignore=shutil.ignore_patterns("*.sdf", "model.config"))
            sdf_name = self.pick_sdf(src)
            if sdf_name is None:
                # a mesh/texture-only directory referenced by URI (e.g. walker/walk_0.dae)
                self.write_config(src, dst, name)
                continue
            tree = ET.parse(src / sdf_name)
            root = tree.getroot()
            root.set("version", "1.9")
            self.convert_tree(root, model_dir=src)
            _indent(root)
            tree.write(dst / "model.sdf", encoding="unicode", xml_declaration=True)
            self.write_config(src, dst, name)

    @staticmethod
    def pick_sdf(src):
        cfg = src / "model.config"
        if cfg.is_file():
            try:
                best = None
                for el in ET.parse(cfg).getroot().findall("sdf"):
                    ver = tuple(int(x) for x in el.get("version", "0").split(".") if x.isdigit())
                    if (src / el.text.strip()).is_file() and (best is None or ver > best[0]):
                        best = (ver, el.text.strip())
                if best:
                    return best[1]
            except ET.ParseError:
                pass
        sdfs = sorted(src.glob("*.sdf"))
        return sdfs[0].name if sdfs else None

    @staticmethod
    def write_config(src, dst, name):
        desc = ""
        cfg = src / "model.config"
        if cfg.is_file():
            try:
                desc = (ET.parse(cfg).getroot().findtext("description") or "").strip()
            except ET.ParseError:
                pass
        (dst / "model.config").write_text(
            f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
            f'  <sdf version="1.9">model.sdf</sdf>\n  <description>{_xml_escape(desc)} '
            f"(converted from Gazebo Classic for Harmonic)</description>\n</model>\n")

    def place_mesh_textures(self):
        """COLLADA and OBJ (.mtl) files often name their images by bare file name (e.g. landing2.png) while
        the image sits in the model's materials/textures. Classic found it through OGRE
        resource paths; Gazebo Harmonic looks next to the mesh, so copy it there."""
        copied = 0
        for model_dir in (self.out / "models").iterdir():
            if not model_dir.is_dir():
                continue
            images = {}
            for f in model_dir.rglob("*"):
                if f.suffix.lower() in (".png", ".jpg", ".jpeg", ".tga", ".bmp"):
                    images.setdefault(f.name, f)
            refs = [(dae, r) for dae in model_dir.rglob("*.dae")
                    for r in re.findall(r"<init_from>([^<]+)</init_from>", dae.read_text(errors="ignore"))]
            refs += [(mtl, r) for mtl in model_dir.rglob("*.mtl")
                     for r in re.findall(r"^\s*map_\w+\s+(?:-\S+\s+\S+\s+)*(\S+)\s*$",
                                         mtl.read_text(errors="ignore"), re.M)]
            for mesh, ref in refs:
                ref = ref.strip()
                if "://" in ref or ref.startswith("/"):
                    continue
                target = mesh.parent / ref
                if not target.exists() and Path(ref).name in images:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(images[Path(ref).name], target)
                    copied += 1
        if copied:
            print(f"textures placed next to their meshes: {copied}")

    def clean_collision_meshes(self):
        """DART crashes when it builds a collision shape from a COLLADA mesh that contains
        line primitives (SketchUp exports edges as <lines>). Collisions get a copy of the
        mesh without them; visuals keep the original."""
        sdfs = list((self.out / "worlds").glob("*.sdf")) + list((self.out / "models").glob("*/model.sdf"))
        cleaned = {}
        for sdf in sdfs:
            tree = ET.parse(sdf)
            changed = False
            for uri in tree.getroot().findall(".//collision/geometry/mesh/uri"):
                u = (uri.text or "").strip()
                if not (u.startswith("model://") and u.lower().endswith(".dae")):
                    continue
                mesh = self.out / "models" / u[len("model://"):]
                if not mesh.is_file():
                    continue
                if mesh not in cleaned:
                    cleaned[mesh] = _strip_dae_lines(mesh)
                if cleaned[mesh]:
                    uri.text = u[:-4] + "_collision.dae"
                    changed = True
            if changed:
                tree.write(sdf, encoding="unicode", xml_declaration=True)
        n = sum(1 for v in cleaned.values() if v)
        if n:
            print(f"collision meshes without line primitives: {n}")

    def write_media(self):
        if not self.media_files or not self.media_dir:
            return
        dst = self.out / "models" / MEDIA_MODEL
        for rel in sorted(self.media_files):
            s = self.media_dir / rel
            if s.is_file():
                (dst / rel).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(s, dst / rel)
            elif s.is_dir():
                shutil.copytree(s, dst / rel, dirs_exist_ok=True)
            else:
                print(f"warning: media file not found: {rel}", file=sys.stderr)
        (dst / "model.config").write_text(
            f'<?xml version="1.0"?>\n<model>\n  <name>{MEDIA_MODEL}</name>\n  <version>1.0</version>\n'
            "  <description>Gazebo Classic media (textures, meshes) used by converted XTDrone worlds."
            "</description>\n</model>\n")


def _strip_dae_lines(mesh):
    """Write <mesh>_collision.dae without <lines>/<linestrips>; False if there were none."""
    text = mesh.read_text(errors="ignore")
    if not re.search(r"<(lines|linestrips)\b", text):
        return False
    tree = ET.parse(mesh)
    for parent in tree.getroot().iter():
        for child in list(parent):
            if child.tag.split("}")[-1] in ("lines", "linestrips"):
                parent.remove(child)
    ns = re.match(r"\{([^}]*)\}", tree.getroot().tag)
    if ns:
        ET.register_namespace("", ns.group(1))
    tree.write(mesh.with_name(mesh.stem + "_collision.dae"), encoding="unicode", xml_declaration=True)
    return True


def _deepcopy(el):
    return ET.fromstring(ET.tostring(el))


def _xml_escape(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _indent(elem, level=0):
    if hasattr(ET, "indent"):
        ET.indent(elem)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--worlds", nargs="*", default=[])
    ap.add_argument("--models", nargs="*", default=[], help="extra model names to convert")
    ap.add_argument("--model-path", nargs="+", required=True)
    ap.add_argument("--classic-media")
    ap.add_argument("--out", required=True)
    ap.add_argument("--skip-models", nargs="*", default=[])
    args = ap.parse_args()

    conv = Converter(args.model_path, args.classic_media, args.out, args.skip_models)
    for w in args.worlds:
        out = conv.convert_world(w)
        print(f"world  {out}")
    for m in args.models:
        conv.require_model(m)
    conv.convert_models()
    conv.place_mesh_textures()
    conv.clean_collision_meshes()
    conv.write_media()
    print(f"models {len(conv.done) - len(conv.missing)} converted")
    if conv.missing:
        print("missing models (not found in --model-path): " + " ".join(sorted(conv.missing)), file=sys.stderr)


if __name__ == "__main__":
    main()
