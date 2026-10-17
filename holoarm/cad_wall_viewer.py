"""Interactive wall experiment using the repository Trial and HoLoArm CAD."""
import argparse,time,math
from pathlib import Path
import mujoco
import numpy as np
from collision_study import Trial,load_config,write_results,tilt_deg
from cad_model import ARM_NUMBERS
from holoarm_narrow_passage_viewer import clean_flight_visuals

def run(speed=.6,duration=3.,wall_thickness=None):
    import mujoco.viewer
    cfg=load_config();cfg['post_impact_s']=duration
    if wall_thickness is not None:cfg['wall_thickness_m']=wall_thickness
    while True:
        trial=Trial('soft',speed,cfg,fracture=False).prepare_approach()
        m,d=trial.model,trial.data;paused=False;actions=[];restart=False;view=0;saved=False
        def key(code):
            if code in (32,82,83,70):actions.append(code)
        def save():
            nonlocal saved
            if not trial.history:return
            folder=Path(__file__).resolve().parents[1]/'outputs'/('cad_wall_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%1000000))
            write_results([trial],folder,cfg);saved=True;print('Saved:',folder,flush=True)
        with mujoco.viewer.launch_passive(m,d,key_callback=key,show_left_ui=False,show_right_ui=False) as viewer:
            viewer.cam.azimuth=90;viewer.cam.elevation=-78;viewer.cam.distance=1.15
            while viewer.is_running():
                start=time.perf_counter()
                while actions:
                    code=actions.pop(0)
                    if code==32:paused=not paused
                    elif code==82:restart=True;break
                    elif code==83:save()
                    elif code==70:view=(view+1)%2
                if restart:break
                with viewer.lock():
                    if not paused:
                        for _ in range(round(.25/(60*m.opt.timestep))):trial.step()
                        saved=False
                    if d.time>=cfg['approach_s']+duration:
                        paused=True
                        if not saved:save()
                    mujoco.mj_forward(m,d)
                    clean_flight_visuals(viewer)
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_SKIN]=True
                    viewer.user_scn.ngeom=0
                    for arm,i in ARM_NUMBERS.items():
                        g=viewer.user_scn.geoms[viewer.user_scn.ngeom]
                        mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_LABEL,np.zeros(3),d.site(f'label{i}').xpos,np.eye(3).ravel(),np.ones(4));g.label=str(i);viewer.user_scn.ngeom+=1
                    viewer.cam.lookat[:]=d.qpos[:3]
                    if view:viewer.cam.elevation=-30
                    else:viewer.cam.elevation=-78
                qs=trial.material.qids;values=[]
                for arm,i in sorted(ARM_NUMBERS.items(),key=lambda v:v[1]):
                    j=list(ARM_NUMBERS).index(arm);q=d.qpos[qs[j]]*[1000,180/np.pi,180/np.pi]
                    values.append(f'{i}: x {q[0]:+.3f} mm | a {q[1]:+.1f} | b {q[2]:+.1f} deg')
                warning='JOINT GUARD REACHED' if trial.peak_bend>math.degrees(.69) or trial.peak_slide>7.8 else ''
                if tilt_deg(d,trial.core)>60:warning+=' BODY TILT > 60 DEG'
                impact='not yet' if trial.actual_impact_speed is None else f'{trial.actual_impact_speed:.3f} m/s'
                viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    f'HoLoArm CAD | repository wall experiment\nFrozen K/C | normal-only contact\n'
                    f't={d.time:.3f}s | incoming {speed:.2f} m/s | actual {impact}\n'
                    f'Wall thickness {cfg["wall_thickness_m"]:.3f} m\n'+warning+'\n'+'\n'.join(values),
                    'Space pause | R restart | S save | F view\n0.25x playback; mouse zoom/orbit\nNeutral-hover preload assumed; no flight validation')])
                viewer.sync();time.sleep(max(0,1/60-(time.perf_counter()-start)))
        if trial.history and not saved:save()
        if not restart:return

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--speed',type=float,default=.6);p.add_argument('--duration',type=float,default=3.)
    p.add_argument('--wall-thickness',type=float,help='Full wall thickness in metres; front face stays fixed')
    a=p.parse_args()
    if not math.isfinite(a.speed) or not 0<a.speed<=.9 or not math.isfinite(a.duration) or a.duration<=0:p.error('speed in (0, .9]; duration > 0')
    if a.wall_thickness is not None and (not math.isfinite(a.wall_thickness) or a.wall_thickness<=0):p.error('wall thickness must be positive and finite')
    run(a.speed,a.duration,a.wall_thickness)
