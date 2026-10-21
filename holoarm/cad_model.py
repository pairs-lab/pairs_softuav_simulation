"""HoLoArm CAD geometry + the frozen one-arm v2 constitutive model.

The old demo's linear springs, 40 mm slides, and decorative ligament builder
are not used. Internal preload transfers the previously agreed neutral hover
assumption; neither material identification nor flight validation is claimed.
"""
from pathlib import Path
import json,hashlib,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

CAD=Path(__file__).resolve().parent/'models'/'holoarm_cad'
FLIGHT=json.loads((CAD/'flight_config.json').read_text(encoding='utf-8'))
RHEO=json.loads((CAD/'rheology.json').read_text())['parameters']
STIFFNESS=json.loads((CAD/'stiffness.json').read_text())
ARM_NUMBERS={'fl':3,'fr':1,'rl':2,'rr':4}

def load_scene():
    scene=json.loads((CAD.parent/'scene_config.json').read_text())
    for group,values in scene.items():
        if not isinstance(values,dict):continue
        for key,value in values.items():
            if not isinstance(value,(float,int)) or not math.isfinite(value):raise ValueError(f'{group}.{key} must be finite')
            if key not in ('front_x_m','center_x_m') and value<=0:raise ValueError(f'{group}.{key} must be positive')
    return scene

def patch_config(cfg):
    scene=load_scene()['impact_wall']
    cfg.setdefault('rigid_model','hard-arm')
    if cfg['rigid_model'] not in ('hard-arm','locked-cad'):
        raise ValueError('rigid_model must be hard-arm or locked-cad')
    for key,value in [('wall_front_x_m',scene['front_x_m']),('wall_thickness_m',scene['thickness_m']),
                      ('wall_width_m',scene['width_m']),('wall_height_m',scene['height_m'])]:cfg.setdefault(key,value)
    for legacy in ('yaw_stiffness_nm_rad','yaw_damping_nms_rad','pitch_stiffness_nm_rad',
                   'pitch_damping_nms_rad','slide_stiffness_n_m','slide_damping_ns_m','wall_friction'):
        cfg.pop(legacy,None)
    cfg.update(mass_kg=FLIGHT['total_mass_kg'],timestep_s=.0004,
        max_motor_thrust_n=FLIGHT['max_motor_thrust_N'],
        motor_time_constant_s=FLIGHT['motor_delay_s'],guard_radius_m=.085,
        contact_time_constant_s=FLIGHT['contact']['solref_s'],
        slide_range_m=.008,bend_limit_deg=math.degrees(.7),
        material_model='frozen_one_arm_v2',contact_model='normal_only_ideal_slip',
        constitutive_parameters=RHEO,
        nonlinear_stiffness_parameters=STIFFNESS,
        notes='User CAD and frozen bench v2; assumed neutral-hover preload and ideal-slip guard contact. The rigid_model setting selects the previous illustrative hard-arm frame or the optional locked-CAD control; neither is an identified fracture model.')
    return cfg

def build_cad_model(kind,cfg,width=640,height=540,environment=None,model_transform=None):
    if kind not in ('soft','rigid'):raise ValueError('kind must be soft or rigid')
    if model_transform is not None:
        raise ValueError('Legacy behavior transforms are disabled for the frozen CAD model. Use the CAD narrow-passage or baseline wall demo.')
    root=ET.parse(CAD/'drone_cad.xml').getroot()
    root.set('model',f'HoLoArm CAD | {kind}')
    root.find('compiler').set('meshdir',str(CAD/'assets'))
    root.find('option').set('timestep',str(cfg['timestep_s']))
    root.find('visual/global').set('offwidth',str(width));root.find('visual/global').set('offheight',str(height))
    aliases={'drone':'holoarm_core','wall':'impact_wall'}
    for arm,i in ARM_NUMBERS.items():
        aliases.update({f'slider{i}':f'{arm}_base',f'x{i}':f'{arm}_slide',
            f'alpha{i}':f'{arm}_bend_yaw',f'beta{i}':f'{arm}_bend_pitch',f'rotor{i}':f'thrust_{arm}'})
    for elem in root.iter():
        for key in ('name','body','body1','body2','joint','site'):
            if elem.get(key) in aliases:elem.set(key,aliases[elem.get(key)])
    world=root.find('worldbody');core=world.find("body[@name='holoarm_core']")
    core.set('pos',f"0 0 {cfg['altitude_m']}")
    wall=world.find("geom[@name='impact_wall']")
    thick=cfg['wall_thickness_m'];height_m=cfg['wall_height_m']
    if not math.isfinite(thick) or thick<=0:raise ValueError('wall_thickness_m must be positive and finite')
    wall.set('pos',f"{cfg['wall_front_x_m']+thick/2} 0 {height_m/2}")
    wall.set('size',f"{thick/2} {cfg['wall_width_m']/2} {height_m/2}");wall.set('condim','1')
    if environment is not None:
        root.set('model','HoLoArm keyboard narrow passage')
        world.remove(wall)
        for original in environment:
            elem=ET.fromstring(ET.tostring(original))
            if elem.tag=='geom' and elem.get('contype','1')!='0':
                elem.set('contype','2');elem.set('conaffinity','1');elem.set('condim','1')
            world.append(elem)
    for geom in root.iter('geom'):
        if geom.get('name','').startswith('guard_contact_'):geom.set('condim','1')
    # Exact damping is implicit in MuJoCo, once; nonlinear stiffness is applied
    # in CADMaterial, once. Skins and CAD meshes are visual and have no mass.
    for arm in ARM_NUMBERS:
        for suffix,key in [('slide','x'),('bend_yaw','alpha'),('bend_pitch','beta_up')]:
            joint=next(e for e in root.iter('joint') if e.get('name')==f'{arm}_{suffix}')
            joint.set('damping',str(RHEO[key][0]));joint.set('stiffness','0')
    if kind=='rigid':
        # Counterfactual: same CAD, masses and rotor layout, arms locked with
        # welds. This is not a measured PLA fracture model or a Skydio X2.
        eq=ET.SubElement(root,'equality')
        for arm in ARM_NUMBERS:
            base=core.find(f"body[@name='{arm}_base']")
            for body in base.iter('body'):
                for j in list(body.findall('joint')):body.remove(j)
            pos=np.fromstring(base.get('pos'),sep=' ');pos[2]+=cfg['altitude_m']
            base.set('pos',' '.join(map(str,pos)));base.insert(0,ET.Element('freejoint',name=f'{arm}_fracture_free'))
            core.remove(base);world.append(base)
            ET.SubElement(eq,'weld',name=f'{arm}_attachment',body1='holoarm_core',body2=f'{arm}_base',
                solref='.001 1',solimp='.99 .999 .001',torquescale='.15')
    actuators=ET.SubElement(root,'actuator')
    for arm,i in ARM_NUMBERS.items():
        spin=FLIGHT['motor_directions'][str(i)]*FLIGHT['yaw_torque_per_thrust_m']
        ET.SubElement(actuators,'general',name=f'motor_{arm}',site=f'thrust_{arm}',
            gear=f'0 0 1 0 0 {spin}',ctrllimited='true',ctrlrange=f"0 {cfg['max_motor_thrust_n']}")
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))

def restoring(q):
    x,a,b=q
    px=STIFFNESS['x'];pa=STIFFNESS['alpha'];pb=STIFFNESS['beta_up' if b>=0 else 'beta_down']
    fs=px['force_saturation_N'];xs=px['displacement_scale_m']
    fx=fs*(-np.expm1(-x/xs)) if x>=0 else (fs/xs)*x
    ma=pa['moment_saturation_Nm'];mb=pb['moment_saturation_Nm']
    return np.array([fx,ma*np.tanh(pa['initial_stiffness_Nm_per_rad']*a/ma),mb*np.tanh(pb['initial_stiffness_Nm_per_rad']*b/mb)])

class CADMaterial:
    def __init__(self,model,data):
        self.m,self.d=model,data
        self.qids=np.array([[model.joint(f'{a}_{s}').qposadr[0] for s in ('slide','bend_yaw','bend_pitch')] for a in ARM_NUMBERS])
        self.dids=np.array([[model.joint(f'{a}_{s}').dofadr[0] for s in ('slide','bend_yaw','bend_pitch')] for a in ARM_NUMBERS])
        pars=np.array([RHEO[s] for s in ['x','alpha','beta_up']])
        self.k=pars[:,1];self.tau=pars[:,2];self.scale=pars[:,3]
        self.reset()

    def reset(self):
        m,d=self.m,self.d
        self.z=d.qpos[self.qids].copy()
        self.reference=np.array([restoring(q) for q in self.z])
        saved=d.ctrl.copy();d.ctrl[:]=m.body_mass.sum()*9.81/4
        mujoco.mj_forward(m,d)
        self.preload=(d.qfrc_bias-d.qfrc_actuator)[self.dids].copy()
        d.ctrl[:]=saved;mujoco.mj_forward(m,d)

    def before_step(self):
        q=self.d.qpos[self.qids].copy();self.previous=q
        force=self.scale*np.array([restoring(v) for v in q])+(1-self.scale)*self.reference+self.k*(q-self.z)
        self.d.qfrc_applied[:]=0
        self.d.qfrc_applied[self.dids]=-force+self.preload

    def after_step(self):
        dt=self.m.opt.timestep;a=-np.expm1(-dt/self.tau)
        self.z=(1-a)*self.z+a*self.previous+(1-self.tau*a/dt)*(self.d.qpos[self.qids]-self.previous)

def provenance():
    return dict(model='CAD HoLoArm, 37 mesh assets, 20 visual skins',
        arm_numbers=ARM_NUMBERS,rheology_sha256=hashlib.sha256((CAD/'rheology.json').read_bytes()).hexdigest(),
        stiffness_sha256=hashlib.sha256((CAD/'stiffness.json').read_bytes()).hexdigest(),
        scope='Frozen bench parameters with assumed hover preload and ideal-slip contact; not a validated flight digital twin.')
