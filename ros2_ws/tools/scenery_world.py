#!/usr/bin/env python3
"""Gazebo world with a scenery (src/aero_pursuit/config/scenery/<name>.json): a copy of the world
SDF with one <include> per tree (Fuel models, static). The world keeps its name, so bridges,
TFs and frames do not change. Isaac Sim reads the same file (isaac/aero_world.py --scenery).

    scenery_world.py <world.sdf> <scenery.json> <out.sdf>
"""
import json
import sys
from xml.sax.saxutils import escape

world, scenery, out = sys.argv[1:4]
doc = json.load(open(scenery))
includes = []
for t in doc["trees"]:
    uri = doc["models"][t["model"]]
    includes.append(f'    <include><uri>{escape(uri)}</uri><name>{escape(t["name"])}</name>'
                    f'<pose>{t["x"]} {t["y"]} 0 0 0 {t["yaw"]}</pose><static>true</static></include>')
text = open(world).read()
cut = text.rindex("</world>")
text = text[:cut] + "    <!-- scenery: " + escape(scenery) + " -->\n" + "\n".join(includes) + "\n  " + text[cut:]
open(out, "w").write(text)
print(f"{len(includes)} modelos de cenografia -> {out}")
