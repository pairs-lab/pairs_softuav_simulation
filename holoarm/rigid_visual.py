"""Attach original X2-inspired artwork to the existing breakable rigid frame."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


KIT = Path(__file__).resolve().parent/'models'/'rigid_x2_visual.xml'


def apply_rigid_visual(root):
    kit = ET.parse(KIT).getroot()
    for asset in kit.find('asset'):
        root.find('asset').append(deepcopy(asset))
    root.find('default').append(deepcopy(kit.find('default/default')))
    world = root.find('worldbody')
    core = world.find("body[@name='holoarm_core']")
    assemblies = [core]+[world.find(f"body[@name='{arm}_base']") for arm in ('fl','fr','rl','rr')]
    # Retain all existing physical geometry, masses and collision envelopes.
    # Alpha only affects rendering; adding zero-mass geometry does not alter
    # the model's inertia or fracture attachment topology.
    for body in assemblies:
        for geom in body.iter('geom'):
            geom.set('rgba','0 0 0 0')

    def attach(parent,part,prefix):
        for geom in kit.find(f"worldbody/body[@name='{part}_visual']").findall('geom'):
            clone = deepcopy(geom)
            clone.set('name',f'rigid_x2_{prefix}_{geom.get("name")}')
            parent.append(clone)

    attach(core,'body','body')
    for arm in ('fl','fr','rl','rr'):
        base = world.find(f"body[@name='{arm}_base']")
        motor = base.find(f"body[@name='{arm}_arm_sliding']/body[@name='{arm}_motor']")
        attach(base,'arm',arm)
        attach(motor,'motor',arm)
        # Different blade orientations make the silhouette readable in top view.
        motor.find(f"geom[@name='rigid_x2_{arm}_prop_blade']").set('euler',f'0 0 {0.6 if arm in ("fl","rr") else -0.6}')
