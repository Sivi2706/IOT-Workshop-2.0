"""OpenRocket CSV playback ground station, inspired by a telemetry grid dashboard.

Standalone Tkinter desktop app (Windows Python usually includes Tkinter).
A firmware/PlatformIO build is NOT needed to replay OpenRocket CSV data.
Run from the PlatformIO root:
    py ground_station/dashboard.py --demo
    py ground_station/dashboard.py --csv "C:/path/to/OpenRocket-export.csv"

Signal is N/A because OpenRocket doesn't simulate received radio strength.
Rocket attitude is animated only from exported orientation fields.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from telemetry import Flight, Measurement, write_synthetic_demo

BG='#030506'; PANEL='#050607'; WHITE='#f1f4f8'; MUTED='#91a3b7'
BLUE='#1717ed'; GREEN='#079b40'; LINE='#f5f7fc'; CYAN='#62c9e9'
FONT='Arial'


def fmt_time(seconds):
    m, s=divmod(max(0.,seconds),60)
    return f'{int(m)}:{s:04.1f}'


def rotate_points(points,cx,cy,angle,scale):
    co,si=math.cos(angle),math.sin(angle)
    result=[]
    for x,y in points:
        result.extend((cx+scale*(x*co-y*si),cy+scale*(x*si+y*co)))
    return result


class PlotCanvas(tk.Canvas):
    def __init__(self,parent,**kw):
        super().__init__(parent,bg=PANEL,highlightthickness=0,**kw)
        self.field=None; self.flight=None; self.seconds=0; self.bounds=None
        self.bind('<Configure>',lambda _:self.render())

    def set_flight(self,flight,field):
        self.flight,self.field=flight,field
        vals=[v for v in flight.series.get(field,[]) if v is not None]
        if vals:
            lo,hi=min(vals),max(vals)
            padding=max(1.0,abs(lo)*.02) if abs(hi-lo)<1e-8 else (hi-lo)*.07
            self.bounds=(lo-padding,hi+padding)
        else:self.bounds=None
        self.render()

    def update_time(self,seconds):
        self.seconds=seconds; self.render()

    def render(self):
        self.delete('all')
        w,h=self.winfo_width(),self.winfo_height()
        if w<75 or h<70:return
        flight=self.flight
        if self.field=='rssi':
            self.create_text(w/2,h/2,text='NO RF MEASUREMENT\nIN OPENROCKET',fill=MUTED,
                             font=(FONT,11,'bold'),justify='center')
            return
        if flight is None:return
        history=flight.history(self.field,self.seconds)
        if self.bounds is None:
            self.create_text(w/2,h/2,text='NOT EXPORTED',fill=MUTED,font=(FONT,11,'bold'))
            return
        left,top,right,bottom=51,10,w-10,h-27
        if right<=left or bottom<=top:return
        vmin,vmax=self.bounds
        self.create_line(left,top,left,bottom,right,bottom,fill=MUTED,width=1)
        for k in range(5):
            val=vmin+(vmax-vmin)*(4-k)/4
            y=top+(bottom-top)*k/4
            self.create_line(left-5,y,left,y,fill=WHITE)
            self.create_text(left-8,y,text=f'{val:.0f}' if abs(val)>=10 else f'{val:.1f}',
                             anchor='e',fill=MUTED,font=(FONT,8))
        dur=max(0.01,flight.duration)
        for k in range(5):
            x=left+(right-left)*k/4
            self.create_line(x,bottom,x,bottom+4,fill=WHITE)
            self.create_text(x,bottom+13,text=f'{dur*k/4:.0f}',fill=MUTED,font=(FONT,8))
        if len(history)>=2:
            coords=[]
            for t,val in history:
                x=left+max(0,min(1,t/dur))*(right-left)
                y=bottom-(val-vmin)/(vmax-vmin)*(bottom-top)
                coords.extend([x,y])
            if len(coords)>=4:
                self.create_line(*coords,fill=LINE,width=1.6,smooth=False)
                self.create_oval(coords[-2]-3,coords[-1]-3,coords[-2]+3,coords[-1]+3,
                                 fill=CYAN,outline='')


class AttitudeCanvas(tk.Canvas):
    BODY=[(-13,-80),(0,-112),(13,-80),(13,53),(23,78),(42,103),(16,103),
          (16,111),(-16,111),(-16,103),(-42,103),(-23,78),(-13,53)]
    def __init__(self,parent,axis,**kw):
        super().__init__(parent,bg=PANEL,highlightthickness=0,**kw)
        self.axis=axis;self.angle=None
        self.bind('<Configure>',lambda _:self.render())

    def set_angle(self,angle):
        self.angle=angle;self.render()

    def render(self):
        self.delete('all')
        w,h=self.winfo_width(),self.winfo_height()
        if w<70 or h<70:return
        if self.angle is None:
            self.create_text(w/2,h/2,text='ORIENTATION\nNOT EXPORTED',fill=MUTED,
                             font=(FONT,10,'bold'),justify='center')
            return
        scale=min(w/135.,h/255.,1.65)
        cx,cy=w/2,h/2
        if self.axis=='roll':
            # Rear view of four fins; nozzle as central circular body.
            radius=21*scale; length=54*scale
            a=math.radians(self.angle)
            for k in range(4):
                th=a+k*math.pi/2
                dx,dy=math.cos(th),math.sin(th)
                self.create_line(cx+radius*dx,cy+radius*dy,
                                 cx+length*dx,cy+length*dy,
                                 fill=WHITE,width=max(2,int(3.5*scale)))
            self.create_oval(cx-radius,cy-radius,cx+radius,cy+radius,
                             fill=WHITE,outline=WHITE)
            # Orange roll marker gives a clear phase reference.
            dx,dy=math.cos(a),math.sin(a)
            self.create_line(cx+radius*dx,cy+radius*dy,cx+length*dx,cy+length*dy,
                             fill='#fbb05d',width=max(2,int(3.5*scale)))
        else:
            # Screen sign chosen so a positive angle rotates the nose clockwise.
            points=rotate_points(self.BODY,cx,cy,math.radians(self.angle),scale)
            self.create_polygon(*points,fill=WHITE,outline=WHITE)
            # Orange side marker helps orientation even when shape is symmetric.
            mark=rotate_points([(12,-15),(12,20)],cx,cy,math.radians(self.angle),scale)
            self.create_line(*mark,fill='#fbb05d',width=max(2,int(scale*2)))


class Card(tk.Frame):
    def __init__(self,parent,title,kind,field=None):
        super().__init__(parent,bg=PANEL,highlightbackground='#aeb1b7',highlightthickness=1)
        self.title=title;self.field=field;self.kind=kind
        self.columnconfigure(0,weight=1)
        self.rowconfigure(2,weight=1)
        tk.Label(self,text=title,anchor='w',bg=PANEL,fg=WHITE,
                 font=(FONT,15,'bold'),padx=7,pady=4).grid(row=0,column=0,sticky='ew')
        self.value=tk.Label(self,text='—',anchor='w',bg=PANEL,fg=WHITE,
                            font=(FONT,17,'bold'),padx=8,pady=4,
                            highlightbackground='#aaadb0',highlightthickness=1)
        self.value.grid(row=1,column=0,sticky='ew',padx=8,pady=(0,3))
        if kind=='plot':
            self.visual=PlotCanvas(self)
        else:
            self.visual=AttitudeCanvas(self,kind)
        self.visual.grid(row=2,column=0,sticky='nsew',padx=9,pady=(3,10))

    def set_data(self,f: Flight,m:Measurement,t:float):
        val={'rssi':None,'altitude':m.altitude,'velocity':m.velocity,
             'pressure':m.pressure_mbar,'temperature':m.temperature_c,
             'pitch':m.pitch,'yaw':m.yaw,'roll':m.roll}.get(self.field)
        units={'rssi':'dBm','altitude':'m','velocity':'m/s','pressure':'mbar',
               'temperature':'°C','pitch':'°','yaw':'°','roll':'°'}
        if val is None:
            self.value.config(text='N/A')
        else:
            self.value.config(text=f'{val:,.2f} {units.get(self.field,"")}')
        if self.kind=='plot':
            self.visual.update_time(t)
        else:
            self.visual.set_angle(val)


class MiniXYPlot(tk.Canvas):
    def __init__(self,parent,xfield,yfield,title):
        super().__init__(parent,bg=PANEL,highlightthickness=0)
        self.xfield=xfield;self.yfield=yfield;self.title=title;self.flight=None;self.t=0
        self.bind('<Configure>',lambda _:self.paint())

    def update(self,flight,t):self.flight=flight;self.t=t;self.paint()

    def paint(self):
        self.delete('all')
        w,h=self.winfo_width(),self.winfo_height()
        if w<60:return
        self.create_text(12,12,text=self.title,anchor='nw',fill=WHITE,font=(FONT,13,'bold'))
        f=self.flight
        if f is None:return
        xseries=f.times if self.xfield=='time' else f.series.get(self.xfield)
        yseries=f.series.get(self.yfield)
        if not xseries or not yseries or not any(v is not None for v in xseries):
            self.create_text(w/2,h/2,text='NOT IN CSV',fill=MUTED,font=(FONT,13,'bold'))
            return
        records=[(x-f.t0 if self.xfield=='time' else x,y,t-f.t0) for t,x,y in zip(f.times,xseries,yseries)
                 if x is not None and y is not None]
        if not records:return
        xmin,xmax=min(r[0] for r in records),max(r[0] for r in records)
        ymin,ymax=min(r[1] for r in records),max(r[1] for r in records)
        xpad=max(1.0,(xmax-xmin)*0.1);ypad=max(1.0,(ymax-ymin)*0.1)
        xmin-=xpad;xmax+=xpad;ymin-=ypad;ymax+=ypad
        l,top,right,bottom=40,44,w-22,h-30
        def project(x,y):
            return (l+(x-xmin)/(xmax-xmin)*(right-l),
                    bottom-(y-ymin)/(ymax-ymin)*(bottom-top))
        self.create_rectangle(l,top,right,bottom,outline=MUTED)
        allcoords=[];nowcoords=[];last=None
        for x,y,t in records:
            xy=project(x,y)
            allcoords.extend(xy)
            if t<=self.t:
                nowcoords.extend(xy);last=xy
        if len(allcoords)>3:self.create_line(*allcoords,fill='#3b5360',width=1)
        if len(nowcoords)>3:self.create_line(*nowcoords,fill=CYAN,width=2)
        if last:
            self.create_oval(last[0]-5,last[1]-5,last[0]+5,last[1]+5,fill='#ffba57',outline='')
        self.create_text(l,bottom+9,text=f'{xmin:.0f}',fill=MUTED,anchor='nw')
        self.create_text(right,bottom+9,text=f'{xmax:.0f}',fill=MUTED,anchor='ne')
        self.create_text(l-4,top,text=f'{ymax:.0f}',fill=MUTED,anchor='ne')
        self.create_text(l-4,bottom,text=f'{ymin:.0f}',fill=MUTED,anchor='se')


class TrajectoryWindow(tk.Toplevel):
    def __init__(self,parent):
        super().__init__(parent,bg=BG)
        self.title('Rocket position | OpenRocket CSV trajectory')
        self.geometry('900x475')
        self.plots=[]
        if parent.flight and parent.flight.has_position:
            # X: East and Y: North (plan view), and time vs altitude (vertical profile)
            selections=[('east','north','GROUND TRACK | EAST vs NORTH (m)'),
                        ('time','altitude','ALTITUDE vs TIME (s / m)')]
        else:
            selections=[('time','altitude','ALTITUDE vs TIME (s / m)')]
        for x,y,name in selections:
            widget=MiniXYPlot(self,x,y,name)
            widget.pack(side='left',fill='both',expand=True,padx=7,pady=7)
            self.plots.append(widget)
        self.parent=parent
        self.protocol('WM_DELETE_WINDOW',self.destroy)

    def update_plot(self,flight,t):
        for widget in self.plots:widget.update(flight,t)


# Rotation matrices and a compact polygonal 3D rocket, drawn with the
# standard Tk Canvas so no external OpenGL engine is needed.
def mmul(a,b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotz(a):
    c,s=math.cos(a),math.sin(a)
    return [[c,-s,0],[s,c,0],[0,0,1]]


def roty(a):
    c,s=math.cos(a),math.sin(a)
    return [[c,0,s],[0,1,0],[-s,0,c]]


def rotx(a):
    c,s=math.cos(a),math.sin(a)
    return [[1,0,0],[0,c,-s],[0,s,c]]


def transpose(a):return [[a[j][i] for j in range(3)] for i in range(3)]


def apply(a,p):return tuple(sum(a[i][j]*p[j] for j in range(3)) for i in range(3))


def orientation_matrix(f,m):
    # Live MPU6050 mode supplies an already calibrated, model-frame quaternion.
    # Use the complete rotation rather than Euler angles to preserve attitude.
    q = getattr(m, 'quaternion', None)
    if q is not None:
        w,x,y,z = q
        return [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]
    # OpenRocket supplies zenith and azimuth: body Z points in that world direction.
    # Direct pitch/yaw columns have different semantics; use XYZ fixed-axis angles.
    if f.attitude_source=='zenith' and m.zenith is not None and m.azimuth is not None:
        r=mmul(rotz(math.radians(m.azimuth)),roty(math.radians(m.zenith)))
    elif m.pitch is not None and m.yaw is not None:
        r=mmul(roty(math.radians(m.yaw)),rotx(math.radians(m.pitch)))
    else:r=[[1,0,0],[0,1,0],[0,0,1]]
    return mmul(r,rotz(math.radians(m.roll or 0.0)))


class Rocket3DWindow(tk.Toplevel):
    def __init__(self,parent):
        super().__init__(parent,bg=BG)
        self.title('3D Rocket Attitude | Live sensor' if getattr(parent.flight,'is_live',False) else '3D Rocket Attitude | Simulation replay')
        self.geometry('550x650')
        self.canvas=tk.Canvas(self,bg=BG,highlightthickness=0)
        self.canvas.pack(fill='both',expand=True)
        self.parent=parent
        self.canvas.bind('<Configure>',lambda _:self.paint())
        self.data=None
        self.zero_orientation=None

    def update_plot(self,f,m):
        self.data=(f,m)
        if self.zero_orientation is None:
            self.zero_orientation=orientation_matrix(f,f.sample(0))
        self.paint()

    def paint(self):
        c=self.canvas;c.delete('all')
        if not self.data:return
        f,m=self.data
        w,h=max(50,c.winfo_width()),max(50,c.winfo_height())
        live=getattr(f,'is_live',False)
        c.create_text(15,13,anchor='nw',text='3D ATTITUDE • LIVE' if live else '3D ATTITUDE • REPLAY',fill=WHITE,font=(FONT,17,'bold'))
        c.create_text(15,42,anchor='nw',text='Relative to Nano calibration origin' if live else 'Relative to the first CSV sample',fill=MUTED,font=(FONT,11))
        if not f.has_attitude:
            c.create_text(w/2,h/2,text='NO ATTITUDE DATA IN CSV',fill=MUTED,font=(FONT,13))
            return
        # relative rotation: R_world(t) * inverse(R_world(0))
        relative=mmul(orientation_matrix(f,m),transpose(self.zero_orientation))
        # Camera basis: right, up, camera-facing depth.
        right=(.8,.6,0.0); up=(-.32,.43,.85); view=(-.51,.68,-.53)
        scale=min(w,h)*.20;cx=w*.50;cy=h*.53
        def vecdot(a,b):return sum(x*y for x,y in zip(a,b))
        def project(v):
            v=apply(relative,v)
            return (cx+scale*vecdot(v,right),cy-scale*vecdot(v,up),vecdot(v,view))
        pieces=[]
        sectors=12;radius=.29
        lower=[(radius*math.cos(2*math.pi*i/sectors),radius*math.sin(2*math.pi*i/sectors),-1.4)
               for i in range(sectors)]
        upper=[(x,y,1.0) for x,y,_ in lower]
        tip=(0,0,1.75)
        for i in range(sectors):
            j=(i+1)%sectors
            pieces.append(([lower[i],lower[j],upper[j],upper[i]], '#cbd4df' if i%2 else '#f3f6fb'))
            pieces.append(([upper[i],upper[j],tip],'#ee693a' if i%2 else '#ff9a63'))
        for a in range(4):
            ang=a*math.pi/2
            dx,dy=math.cos(ang),math.sin(ang)
            v=lambda r,z:(r*dx,r*dy,z)
            pieces.append(([v(.24,-.65),v(.85,-1.28),v(.24,-1.45)],
                           '#ffa657' if a==0 else '#6aa9e6'))
        to_draw=[]
        for xyz,fill in pieces:
            projected=[project(v) for v in xyz]
            depth=sum(p[2] for p in projected)/len(projected)
            coords=[q for p in projected for q in p[:2]]
            to_draw.append((depth,coords,fill))
        for _,coords,fill in sorted(to_draw,key=lambda z:z[0]):
            c.create_polygon(*coords,fill=fill,outline='#202d40',width=1)
        c.create_text(w/2,h-65,text=f'T + {m.time:0.1f} s',fill=WHITE,font=(FONT,17,'bold'))
        c.create_text(w/2,h-36,text='Camera fixed • live MPU6050 quaternion' if live else 'Camera fixed • OpenRocket attitude',fill=MUTED,font=(FONT,11))


class GroundStation(tk.Tk):
    def __init__(self,flight:Flight,autoplay=True,speed=1.0):
        super().__init__()
        self.title('OPENROCKET | FLIGHT TELEMETRY GROUND STATION')
        self.geometry('1420x860'); self.minsize(970,690)
        self.configure(bg=GREEN)
        self.flight=flight;self.playing=autoplay;self.speed=speed
        self.current=0.0;self.last_wall=time.monotonic()
        self.dragging=False;self.trajectory=None;self.rocket3d=None
        self._build()
        self._set_flight(flight)
        self.after(60,self.tick)

    def _build(self):
        border=tk.Frame(self,bg=BG)
        border.pack(fill='both',expand=True,padx=8,pady=8)
        self.time_label=tk.Label(border,text='FLIGHT TIME: 0:00.0',font=(FONT,29,'bold'),
                                 bg=BLUE,fg=WHITE,anchor='center',height=2)
        self.time_label.pack(fill='x',padx=2,pady=(2,5))
        tk.Frame(border,bg=WHITE,height=3).pack(fill='x',padx=2,pady=(0,6))
        control=tk.Frame(border,bg=BG)
        control.pack(fill='x',padx=6,pady=(0,6))
        def btn(text,fn):
            return tk.Button(control,text=text,command=fn,fg=WHITE,bg='#243147',
                             font=(FONT,10,'bold'),activeforeground=WHITE,activebackground='#385671',
                             relief='flat',padx=9,pady=3)
        btn('OPEN CSV',self.open_csv).pack(side='left',padx=(0,5))
        self.play_btn=btn('PAUSE',self.toggle_play)
        self.play_btn.pack(side='left',padx=(0,5))
        btn('RESTART',self.restart).pack(side='left',padx=(0,5))
        btn('TRAJECTORY',self.open_trajectory).pack(side='left',padx=(0,5))
        btn('3D ROCKET',self.open_3d).pack(side='left',padx=(0,12))
        tk.Label(control,text='SPEED',fg=MUTED,bg=BG,font=(FONT,9,'bold')).pack(side='left')
        self.speed_var=tk.StringVar(value='1×')
        speedbox=ttk.Combobox(control,textvariable=self.speed_var,width=6,state='readonly',
                              values=['0.25×','0.5×','1×','2×','4×','8×'])
        speedbox.pack(side='left',padx=(5,13))
        speedbox.bind('<<ComboboxSelected>>',lambda _:self.change_speed())
        self.timeline=tk.Scale(control,orient='horizontal',from_=0,to=100,resolution=.1,
                               showvalue=False,bg=BG,fg=WHITE,troughcolor='#40566c',
                               highlightthickness=0,bd=0,length=250)
        self.timeline.pack(side='left',fill='x',expand=True,padx=(0,6))
        self.timeline.bind('<ButtonPress-1>',lambda _:self.set_dragging(True))
        self.timeline.bind('<ButtonRelease-1>',self.stop_dragging)
        self.timeline.configure(command=self.seek)
        self.duration_label=tk.Label(control,text='',bg=BG,fg=WHITE,font=(FONT,10))
        self.duration_label.pack(side='right')
        self.status=tk.Label(border,text='',anchor='w',bg=BG,fg=MUTED,
                             font=(FONT,10),padx=8)
        self.status.pack(side='bottom',fill='x',pady=(1,4))
        self.grid_frame=tk.Frame(border,bg=BG)
        self.grid_frame.pack(fill='both',expand=True,padx=2,pady=(0,5))
        for col in range(4):self.grid_frame.columnconfigure(col,weight=1,uniform='col')
        for row in range(2):self.grid_frame.rowconfigure(row,weight=1,uniform='row')
        specs=[('SIGNAL STRENGTH','plot','rssi'),('ALTITUDE','plot','altitude'),
               ('VERTICAL VELOCITY','plot','velocity'),('TEMPERATURE','plot','temperature'),
               ('PRESSURE','plot','pressure'),('PITCH / TILT X','pitch','pitch'),
               ('YAW / TILT Y','yaw','yaw'),('AXIAL ROLL','roll','roll')]
        # For velocity field display use the exported/derived vertical velocity series.
        self.cards=[]
        for i,(label,kind,field) in enumerate(specs):
            card=Card(self.grid_frame,label,kind,field)
            card.grid(row=i//4,column=i%4,sticky='nsew',padx=4,pady=4)
            self.cards.append(card)

    def _set_flight(self,f):
        self.flight=f;self.current=0.0;self.timeline.configure(to=max(0.1,f.duration))
        self.duration_label.config(text=f' / {fmt_time(f.duration)}')
        for card in self.cards:
            if card.kind=='plot':
                which={'velocity':f.velocity_key,'temperature':'temperature',
                       'pressure':'pressure','altitude':'altitude','rssi':'rssi'}[card.field]
                card.visual.set_flight(f,which)
        self._update_all()
        self._sync_status()

    def _sync_status(self):
        f=self.flight
        caveat=' • '.join(f.warnings[:2])
        self.status.config(text=f'SOURCE: {f.title} | {len(f.times)} frames | '
                          f'VELOCITY: {f.velocity_note} | RSSI: N/A (no RF model) | {caveat}')

    def _update_all(self):
        f=self.flight;m=f.sample(self.current)
        self.time_label.config(text=f'FLIGHT TIME: {fmt_time(self.current)}')
        for card in self.cards:card.set_data(f,m,self.current)
        if self.trajectory is not None:
            try:
                if self.trajectory.winfo_exists():self.trajectory.update_plot(f,self.current)
                else:self.trajectory=None
            except tk.TclError:self.trajectory=None
        if self.rocket3d is not None:
            try:
                if self.rocket3d.winfo_exists():self.rocket3d.update_plot(f,m)
                else:self.rocket3d=None
            except tk.TclError:self.rocket3d=None
        if not self.dragging:
            self.timeline.set(self.current)

    def tick(self):
        now=time.monotonic()
        elapsed=min(0.3,max(0,now-self.last_wall))
        self.last_wall=now
        if self.playing and not self.dragging:
            self.current=min(self.flight.duration,self.current+elapsed*self.speed)
            if self.current>=self.flight.duration:
                self.playing=False;self.play_btn.config(text='PLAY')
        self._update_all()
        self.after(75,self.tick)

    def change_speed(self):
        self.speed=float(self.speed_var.get().replace('×',''))

    def toggle_play(self):
        if self.current>=self.flight.duration:self.current=0
        self.playing=not self.playing
        self.play_btn.config(text='PAUSE' if self.playing else 'PLAY')
        self.last_wall=time.monotonic()

    def restart(self):
        self.current=0;self.playing=True;self.play_btn.config(text='PAUSE')
        self.last_wall=time.monotonic();self._update_all()

    def set_dragging(self,value):self.dragging=value

    def seek(self,value):
        if self.dragging:
            self.current=float(value);self._update_all()

    def stop_dragging(self,_event):
        self.current=float(self.timeline.get());self.dragging=False;self._update_all()
        self.last_wall=time.monotonic()

    def open_csv(self):
        path=filedialog.askopenfilename(title='OpenRocket Flight Data CSV',
                 filetypes=[('CSV / TXT','*.csv *.txt'),('All files','*.*')])
        if not path:return
        try:f=Flight.from_csv(path)
        except Exception as e:
            messagebox.showerror('Unable to load OpenRocket CSV',str(e));return
        self.playing=True;self.play_btn.config(text='PAUSE')
        self._set_flight(f);self.last_wall=time.monotonic()
        if self.rocket3d is not None:
            try:self.rocket3d.zero_orientation=None
            except tk.TclError:pass

    def open_trajectory(self):
        if self.trajectory is not None:
            try:
                if self.trajectory.winfo_exists():self.trajectory.lift();return
            except tk.TclError:pass
        self.trajectory=TrajectoryWindow(self)
        self.trajectory.update_plot(self.flight,self.current)

    def open_3d(self):
        if self.rocket3d is not None:
            try:
                if self.rocket3d.winfo_exists():self.rocket3d.lift();return
            except tk.TclError:pass
        self.rocket3d=Rocket3DWindow(self)
        self.rocket3d.update_plot(self.flight,self.flight.sample(self.current))


def main(argv=None):
    parser=argparse.ArgumentParser(description='OpenRocket flight-data telemetry playback dashboard')
    options=parser.add_mutually_exclusive_group(required=True)
    options.add_argument('--csv',help='Path to CSV exported by OpenRocket')
    options.add_argument('--demo',action='store_true',help='Play clearly labeled synthetic sample (not OpenRocket)')
    options.add_argument('--port',help='LIVE physical Nano MPU6050 serial port, e.g. COM4')
    parser.add_argument('--baud',type=int,default=115200,help='Serial baud rate for --port')
    parser.add_argument('--nose-axis',choices=['+x','-x','+y','-y','+z','-z'],default='+z',
                        help='MPU6050 axis pointing to imaginary rocket nose (live mode)')
    parser.add_argument('--turn',type=int,choices=[0,90,180,270],default=0,
                        help='Rotation of model around nose axis for physical mounting')
    parser.add_argument('--view',choices=['imu','telemetry'],default='imu',
                        help='Start with live IMU graphs or original telemetry panels')
    parser.add_argument('--record',help='Optional path to save received Q packets as CSV')
    parser.add_argument('--no-3d',action='store_true',help='Do not open live 3D rocket automatically')
    parser.add_argument('--speed',type=float,default=1.0,choices=[.25,.5,1.,2.,4.,8.])
    args=parser.parse_args(argv)
    if args.port:
        # Defer serial and live dashboard imports: offline CSV works without pyserial.
        try:
            from live_dashboard import run_live
            return run_live(args)
        except (OSError, ValueError, ImportError) as exc:
            print(f'LIVE SERIAL ERROR: {exc}',file=sys.stderr)
            return 1
    try:
        if args.demo:
            file=Path(__file__).resolve().parent/'synthetic_demo_not_openrocket.csv'
            if not file.exists():write_synthetic_demo(file)
        else:file=Path(args.csv).expanduser()
        flight=Flight.from_csv(file)
    except (OSError,ValueError) as exc:
        print(f'CSV LOAD ERROR: {exc}',file=sys.stderr)
        return 1
    if args.demo:flight.warnings.insert(0,'SYNTHETIC EXAMPLE, NOT OPENROCKET DATA')
    app=GroundStation(flight,autoplay=True,speed=args.speed)
    app.speed_var.set(f'{args.speed:g}×')
    app.mainloop()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
