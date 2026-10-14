"""HoLoArm reference appearance with ligaments attached to actual moving sites.

All added model geometry is massless and collision-free. The bowed ligament
curves are presentation geometry derived from actual body/site transforms;
they do not replace the existing spring/damper mechanics.
"""
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


KIT=Path(__file__).resolve().parent/'models'/'soft_holoarm_visual.xml'
ARMS=('fl','fr','rl','rr')


def apply_soft_visual(root):
    kit=ET.parse(KIT).getroot()
    for asset in kit.find('asset'):
        root.find('asset').append(deepcopy(asset))
    root.find('default').append(deepcopy(kit.find('default/default')))
    core=root.find("worldbody/body[@name='holoarm_core']")
    for geom in core.iter('geom'):
        if '_guard_' in geom.get('name',''):
            geom.set('rgba','0.96 0.95 0.90 1')
        else:
            geom.set('rgba','0 0 0 0')
    for tendon in root.find('tendon'):
        tendon.set('rgba','0 0 0 0')

    def attach(parent,part,prefix,offset=None,yaw=0.):
        yaw_q=np.array([math.cos(yaw/2),0.,0.,math.sin(yaw/2)])
        rot=np.array([[math.cos(yaw),-math.sin(yaw),0],
                      [math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        for geom in kit.find(f"worldbody/body[@name='{part}_visual']").findall('geom'):
            clone=deepcopy(geom)
            clone.set('name',f'soft_holoarm_{prefix}_{geom.get("name")}')
            if offset is not None:
                pos=offset+rot@np.fromstring(geom.get('pos','0 0 0'),sep=' ')
                clone.set('pos',' '.join(map(str,pos)))
                local_q=np.fromstring(geom.get('quat','1 0 0 0'),sep=' ')
                quaternion=np.empty(4)
                mujoco.mju_mulQuat(quaternion,yaw_q,local_q)
                clone.set('quat',' '.join(map(str,quaternion)))
            parent.append(clone)

    attach(core,'body','body')
    for arm in ARMS:
        base=core.find(f"body[@name='{arm}_base']")
        sliding=base.find(f"body[@name='{arm}_arm_sliding']")
        motor=sliding.find(f"body[@name='{arm}_motor']")
        offset=np.fromstring(base.get('pos'),sep=' ')
        yaw=float(base.get('euler').split()[2])
        attach(core,'fixed_joint',f'{arm}_fixed',offset,yaw)
        attach(sliding,'arm',arm)
        attach(motor,'motor',arm)
        # The black blades sit between the white guard spokes, making both
        # structures readable rather than superimposing their silhouettes.
        phase=(.45,.65,.35,.8)[ARMS.index(arm)]
        blade_q=np.array([math.cos(phase/2),0.,0.,math.sin(phase/2)])
        blade_rot=np.array([[math.cos(phase),-math.sin(phase),0],
                            [math.sin(phase),math.cos(phase),0],[0,0,1]])
        for blade in range(1,4):
            geom=motor.find(f"geom[@name='soft_holoarm_{arm}_blade_{blade}']")
            pos=blade_rot@np.fromstring(geom.get('pos'),sep=' ')
            local_q=np.fromstring(geom.get('quat','1 0 0 0'),sep=' ')
            quaternion=np.empty(4)
            mujoco.mju_mulQuat(quaternion,blade_q,local_q)
            geom.set('pos',' '.join(map(str,pos)))
            geom.set('quat',' '.join(map(str,quaternion)))


def ligament_paths(model,data,segments=10):
    """Return bowed centre-lines with endpoints fixed to real attachment sites."""
    paths=[]
    core=model.body('holoarm_core').id
    core_rotation=data.xmat[core].reshape(3,3)
    for arm in ARMS:
        body=model.body(f'{arm}_base').id
        q=model.body_quat[body]
        nominal=np.empty(9)
        mujoco.mju_quat2Mat(nominal,q)
        fixed_rotation=core_rotation@nominal.reshape(3,3)
        moving_rotation=data.xmat[body].reshape(3,3)
        for index,direction in enumerate(((0,1,0),(0,-1,0),(0,0,1),(0,0,-1))):
            a=data.site_xpos[model.site(f'{arm}_ligament_fixed_{index}').id]
            b=data.site_xpos[model.site(f'{arm}_ligament_moving_{index}').id]
            direction=np.asarray(direction,dtype=float)
            outward=(fixed_rotation+moving_rotation)@direction
            outward/=max(np.linalg.norm(outward),1e-12)
            midpoint=(a+b)/2+outward*.020
            t=np.linspace(0,1,segments+1)[:,None]
            paths.append((1-t)**2*a+2*t*(1-t)*midpoint+t**2*b)
    return paths


def add_soft_ligaments(scene,model,data):
    """Add curved orange TPU-band artwork to a freshly updated render scene."""
    for path in ligament_paths(model,data):
        for a,b in zip(path[:-1],path[1:]):
            if scene.ngeom>=scene.maxgeom:
                raise RuntimeError('Increase render-scene capacity for soft ligaments')
            geom=scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),
                               np.zeros(3),np.eye(3).ravel(),np.array([1.,.48,.035,1.]))
            mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_CAPSULE,.0022,a,b)
            geom.category=mujoco.mjtCatBit.mjCAT_DECOR
            scene.ngeom+=1
