"""Standalone CAD demos. No ROS, WSL or Linux desktop required."""
from pathlib import Path
import os,sys,math,time,subprocess,json
import tkinter as tk
from tkinter import ttk,messagebox

ROOT=Path(__file__).resolve().parent

def main():
    app=tk.Tk();app.title('PAIRS | HoLoArm CAD');app.geometry('760x660')
    pane=ttk.Frame(app,padding=22);pane.pack(fill='both',expand=True)
    ttk.Label(pane,text='HoLoArm CAD | PAIRS Simulation',font=('Segoe UI',20,'bold')).pack(anchor='w')
    ttk.Label(pane,text='CAD của HoLoArm • K, C và bộ nhớ vật liệu v2 giữ nguyên',wraplength=670).pack(anchor='w',pady=(8,20))
    row=ttk.Frame(pane);row.pack(fill='x')
    speed=tk.StringVar(value='.6');gap=tk.StringVar(value='.46')
    ttk.Label(row,text='Tốc độ (m/s)').grid(row=0,column=0,pady=6,sticky='w')
    ttk.Combobox(row,textvariable=speed,values=['.3','.6','.9'],width=9).grid(row=0,column=1,padx=12)
    ttk.Label(row,text='Bề rộng khe (m)').grid(row=1,column=0,pady=6,sticky='w')
    ttk.Entry(row,textvariable=gap,width=11).grid(row=1,column=1,padx=12)
    scene=json.loads((ROOT/'holoarm/models/scene_config.json').read_text())
    wall=tk.StringVar(value=str(scene['impact_wall']['thickness_m']))
    passage=tk.StringVar(value=str(scene['passage']['wall_thickness_x_m']))
    ttk.Label(row,text='Dày tường va đập (m)').grid(row=2,column=0,pady=6,sticky='w')
    ttk.Entry(row,textvariable=wall,width=11).grid(row=2,column=1,padx=12)
    ttk.Label(row,text='Dày vật cản khe theo X (m)').grid(row=3,column=0,pady=6,sticky='w')
    ttk.Entry(row,textvariable=passage,width=11).grid(row=3,column=1,padx=12)
    def launch(mode):
        try:
            v=float(speed.get());g=float(gap.get());tw=float(wall.get());tp=float(passage.get())
            if not math.isfinite(v) or not .1<=v<=.9 or not math.isfinite(g) or not .3<=g<=1:raise ValueError('Tốc độ 0.1–0.9 m/s; khe 0.3–1.0 m.')
            if not all(math.isfinite(x) and x>0 for x in [tw,tp]):raise ValueError('Bề dày phải là số dương, đơn vị m.')
            if mode=='gap':args=['holoarm/holoarm_narrow_passage_viewer.py','--gap',str(g),'--speed',str(v),'--passage-thickness',str(tp)]
            elif mode=='wall':args=['holoarm/cad_wall_viewer.py','--speed',str(v),'--wall-thickness',str(tw)]
            else:
                out=ROOT/'outputs'/('cad_comparison_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%1000000))
                args=['holoarm/wall_collision_comparison.py','--impact-speed',str(v),'--wall-thickness',str(tw),'--no-fracture','--output',str(out/'comparison.mp4'),'--results-dir',str(out)]
                messagebox.showinfo('Xuất video','Đang chạy và xuất video ở cửa sổ dòng lệnh. Hoàn tất rồi mở thư mục outputs để xem.')
            subprocess.Popen([sys.executable,*args],cwd=ROOT)
        except Exception as e:messagebox.showerror('Không chạy được',str(e))
    for text,mode in [('1. BAY QUA KHE HẸP — điều khiển bàn phím','gap'),('2. VA TƯỜNG — quan sát trực tiếp','wall'),('3. XUẤT VIDEO — HoLoArm CAD mềm / tay cứng X2-style','comparison')]:
        ttk.Button(pane,text=text,command=lambda mode=mode:launch(mode)).pack(fill='x',pady=7,ipady=6)
    ttk.Label(pane,text='Khe: giữ I/K, J/L để bay; U/O lên/xuống; Q/E xoay.\nThả phím để hãm; H giữ vị trí; R lặp lại; Space dừng; F đổi góc nhìn.\nVa tường dùng Space, R, S và F. Kết quả do mô phỏng tự tính.',wraplength=670).pack(anchor='w',pady=12)
    def outputs():
        folder=ROOT/'outputs';folder.mkdir(exist_ok=True)
        if os.name=='nt':os.startfile(str(folder))
        else:print(folder)
    ttk.Button(pane,text='Mở thư mục kết quả',command=outputs).pack(anchor='w')
    app.mainloop()

if __name__=='__main__':main()
