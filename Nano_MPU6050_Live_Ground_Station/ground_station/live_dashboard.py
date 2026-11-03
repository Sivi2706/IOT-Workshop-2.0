"""LIVE Arduino Nano + MPU6050 source for the existing eight-card dashboard.

Start from project root (Windows / PlatformIO):
    py ground_station/dashboard.py --port COM4
    py ground_station/dashboard.py --port COM4 --nose-axis +x --turn 90

Accepts V1 and V2 Nano firmware packets:
V1: Q,millis,qw,qx,qy,qz,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps
V2: Q,millis,qw,qx,qy,qz,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,still

Telemetry-only bench-top visualization. Never controls deployment hardware.
"""
from __future__ import annotations

import csv
from collections import deque
from dataclasses import dataclass
import math
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox

from attitude_math import alignment, euler_degrees, map_sensor_to_rocket, parse_packet
from dashboard import BG, BLUE, CYAN, FONT, GREEN, LINE, MUTED, PANEL, WHITE, Card, Rocket3DWindow, fmt_time

# 45 seconds at ~50 samples/second; limit memory during long demonstrations.
HISTORY_LIMIT = 3000
GRAPH_SECONDS = 45.0
IDENTITY = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class LiveFrame:
    time: float
    timestamp_ms: int
    quaternion: tuple[float,float,float,float]
    pitch: float
    yaw: float
    roll: float
    accel: tuple[float,float,float]
    gyro: tuple[float,float,float]
    stationary: bool | None

    def get(self, field: str):
        if field in ('pitch', 'yaw', 'roll'):
            return getattr(self, field)
        if field == 'ax': return self.accel[0]
        if field == 'ay': return self.accel[1]
        if field == 'az': return self.accel[2]
        if field == 'gx': return self.gyro[0]
        if field == 'gy': return self.gyro[1]
        if field == 'gz': return self.gyro[2]
        if field == 'accel_mag': return math.sqrt(sum(v*v for v in self.accel))
        if field == 'gyro_mag': return math.sqrt(sum(v*v for v in self.gyro))
        return None


@dataclass
class LiveMeasurement:
    """Minimal data shape expected by the existing polygonal 3D rocket view."""
    time: float
    quaternion: tuple[float,float,float,float] = IDENTITY
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


class LiveFlight:
    """Store sensor packets for graphs; do not synthesize unsupported flight data."""
    is_live = True
    has_attitude = True
    attitude_source = 'quaternion'
    has_position = False

    def __init__(self, nose_axis='+z', turn=0):
        self.align = alignment(nose_axis, turn)
        self.frames: deque[LiveFrame] = deque(maxlen=HISTORY_LIMIT)
        self.start_ms: int | None = None
        self.last_ms: int | None = None
        self.seconds = 0.0
        self.received = 0

    def reset_history(self):
        self.frames.clear()
        self.start_ms = None
        self.last_ms = None
        self.seconds = 0.0

    def add(self, packet) -> LiveFrame:
        t_ms, q_sensor, accel, gyro, stationary = packet
        if self.last_ms is not None and t_ms + 200 < self.last_ms:
            # Nano reset; do not draw a graph across two separate sessions.
            self.reset_history()
        if self.start_ms is None: self.start_ms = t_ms
        self.last_ms = t_ms
        self.seconds = max(0.0, (t_ms-self.start_ms)*0.001)
        # V1 already emits a calibration-relative WORLD-frame quaternion.
        # V2 emits a calibration-relative SENSOR-frame quaternion, so only V2
        # supports the optional --nose-axis/--turn mounting transform.
        q_model = q_sensor if stationary is None else map_sensor_to_rocket(q_sensor, self.align)
        x,y,z = euler_degrees(q_model)
        f = LiveFrame(time=self.seconds, timestamp_ms=t_ms,
                      quaternion=q_model, pitch=x, yaw=y, roll=z,
                      accel=accel, gyro=gyro, stationary=stationary)
        self.frames.append(f)
        self.received += 1
        return f

    def sample(self, seconds=0):
        # The Nano's quaternion already starts from the calibrated identity.
        if seconds == 0 or not self.frames:
            return LiveMeasurement(time=0.0)
        f=self.frames[-1]
        return LiveMeasurement(time=f.time, quaternion=f.quaternion,
                               pitch=f.pitch, yaw=f.yaw, roll=f.roll)

    def history(self, field: str, seconds=GRAPH_SECONDS, limit=260):
        if not self.frames:return []
        cutoff=self.frames[-1].time-seconds
        result=[]
        for f in reversed(self.frames):
            if f.time < cutoff: break
            v=f.get(field)
            if v is not None:result.append((f.time,v))
        result.reverse()
        if len(result)>limit:
            step=max(1,math.ceil(len(result)/limit))
            result=result[::step]+([] if result[-1] == result[::step][-1] else [result[-1]])
        return result


class NanoSerialPump:
    """Bounded nonblocking line receiver, testable with a fake serial object."""
    def __init__(self, port):
        self.port=port
        self.buffer=bytearray()
        self.dropped=0
        self.legacy_seen=False

    def poll(self):
        """Return (packets, status lines) without blocking the Tk UI thread."""
        available=self.port.in_waiting
        if available:
            chunk=self.port.read(min(available,8192))
            self.buffer.extend(chunk)
        if len(self.buffer)>32768:
            self.buffer.clear()
            self.dropped+=1
            return [], ['# WARNING: serial buffer overflow; discarding incomplete data']
        packets=[];statuses=[]
        for _ in range(220):
            index=self.buffer.find(b'\n')
            if index<0:break
            raw=bytes(self.buffer[:index])
            del self.buffer[:index+1]
            line=raw.decode('ascii','replace').strip()
            if not line:continue
            if line.startswith('#'):
                statuses.append(line)
            elif line.startswith('Q,'):
                count = len(line.split(','))
                if count not in (12, 13):
                    self.dropped += 1
                    continue
                if count == 12 and not self.legacy_seen:
                    statuses.append('# INFO: V1 12-field packets accepted; stationary flag unavailable. Use V2 for nose-axis mapping.')
                    self.legacy_seen=True
                packet=parse_packet(line)
                if packet is not None:packets.append(packet)
                else:self.dropped+=1
        return packets,statuses


class LivePlotCanvas(tk.Canvas):
    def __init__(self,parent):
        super().__init__(parent,bg=PANEL,highlightthickness=0)
        self.history=[];self.available=True;self.title=''
        self.bind('<Configure>',lambda _:self.render())

    def update_history(self, history, available=True, title=''):
        self.history,self.available,self.title=history,available,title
        self.render()

    def render(self):
        self.delete('all')
        w,h=self.winfo_width(),self.winfo_height()
        if w<90 or h<80:return
        if not self.available:
            self.create_text(w/2,h/2,text='N/A\nNOT IN NANO PACKET',fill=MUTED,
                             font=(FONT,11,'bold'),justify='center')
            return
        if len(self.history)<2:
            self.create_text(w/2,h/2,text='WAITING FOR LIVE DATA',fill=MUTED,
                             font=(FONT,11,'bold'))
            return
        left,top,right,bottom=53,11,w-11,h-28
        times,values=zip(*self.history)
        vmax=max(values);vmin=min(values)
        pad=max((vmax-vmin)*0.11,0.04 if max(abs(vmax),abs(vmin))<3 else 0.25)
        if vmax-vmin<1e-8:pad=max(pad,abs(vmax)*0.03,0.03)
        vmin-=pad;vmax+=pad
        last_t=times[-1]
        xmin=max(0.0,last_t-GRAPH_SECONDS);xmax=max(GRAPH_SECONDS,last_t)
        xspan=xmax-xmin
        self.create_line(left,top,left,bottom,right,bottom,fill=MUTED)
        for i in range(5):
            v=vmax-(vmax-vmin)*i/4
            y=top+(bottom-top)*i/4
            self.create_line(left-4,y,left,y,fill=MUTED)
            self.create_text(left-8,y,text=f'{v:.1f}' if abs(v)<100 else f'{v:.0f}',
                             fill=MUTED,anchor='e',font=(FONT,8))
        for i in range(5):
            tick=xmin+xspan*i/4
            px=left+(right-left)*i/4
            self.create_line(px,bottom,px,bottom+4,fill=MUTED)
            self.create_text(px,bottom+13,text=f'{tick:.0f}',fill=MUTED,font=(FONT,8))
        xy=[]
        for t,v in self.history:
            px=left+(t-xmin)/xspan*(right-left)
            py=bottom-(v-vmin)/(vmax-vmin)*(bottom-top)
            xy.extend((px,py))
        if len(xy)>=4:
            self.create_line(*xy,fill=LINE,width=1.6,smooth=False)
            self.create_oval(xy[-2]-3,xy[-1]-3,xy[-2]+3,xy[-1]+3,
                             fill=CYAN,outline='')


TELEMETRY_CARDS = [
    ('SIGNAL STRENGTH', 'rssi', 'dBm'),
    ('ALTITUDE', 'altitude', 'm'),
    ('VERTICAL VELOCITY', 'velocity', 'm/s'),
    ('TEMPERATURE', 'temperature', '°C'),
    ('PRESSURE', 'pressure', 'mbar'),
]
IMU_CARDS = [
    ('ACCELERATION X', 'ax', 'g'),
    ('ACCELERATION Y', 'ay', 'g'),
    ('ACCELERATION Z', 'az', 'g'),
    ('ANGULAR RATE |GYRO|', 'gyro_mag', '°/s'),
    ('GYRO Z (BODY)', 'gz', '°/s'),
]


class LiveGroundStation(tk.Tk):
    def __init__(self, serial_port, port_name, nose_axis='+z', turn=0,
                 view='imu', record=None, auto_3d=True):
        super().__init__()
        self.title('LIVE NANO / MPU6050 | ROCKET TELEMETRY GROUND STATION')
        self.geometry('1450x880');self.minsize(970,690)
        self.configure(bg=GREEN)
        self.serial=serial_port
        self.port_name=port_name
        self.pump=NanoSerialPump(serial_port)
        self.flight=LiveFlight(nose_axis,turn)
        self.mode=view
        self.rocket3d=None
        self.last_frame_wall=None
        self.last_msg='WAITING FOR NANO TO INITIALISE'
        self.log=None
        self.writer=None
        if record:
            path=Path(record).expanduser()
            path.parent.mkdir(parents=True,exist_ok=True)
            self.log=path.open('w',newline='',encoding='utf-8')
            self.writer=csv.writer(self.log)
            self.writer.writerow(['millis','qw','qx','qy','qz','ax_g','ay_g','az_g',
                                  'gx_dps','gy_dps','gz_dps','stationary'])
        self._build()
        self.protocol('WM_DELETE_WINDOW',self.shutdown)
        self.after(60,self.tick)
        if auto_3d:self.after(800,self.open_3d)

    def _build(self):
        outer=tk.Frame(self,bg=BG)
        outer.pack(fill='both',expand=True,padx=8,pady=8)
        self.time_label=tk.Label(outer,text='LIVE SENSOR TIME: 0:00.0',bg=BLUE,fg=WHITE,
                                 font=(FONT,29,'bold'),height=2)
        self.time_label.pack(fill='x',padx=2,pady=(2,5))
        tk.Frame(outer,bg=WHITE,height=3).pack(fill='x',padx=2,pady=(0,7))
        actions=tk.Frame(outer,bg=BG)
        actions.pack(fill='x',padx=6,pady=(0,8))
        def button(title, action):
            w=tk.Button(actions,text=title,command=action,bg='#243147',fg=WHITE,
                        font=(FONT,10,'bold'),relief='flat',padx=10,pady=5)
            w.pack(side='left',padx=(0,6))
            return w
        button('ZERO ORIGIN (Z)',lambda:self.command('Z'))
        button('CALIBRATE (C)',lambda:self.command('C'))
        button('3D ROCKET',self.open_3d)
        button('CLEAR GRAPHS',self.clear_graphs)
        self.switch_btn=button('SHOW TELEMETRY VIEW' if self.mode=='imu' else 'SHOW IMU VIEW',
                               self.switch_view)
        tk.Label(actions,text='LIVE USB / NO RF OR FIREBASE',fg=MUTED,bg=BG,
                 font=(FONT,10,'bold')).pack(side='right',padx=4)
        self.status=tk.Label(outer,text=self.last_msg,anchor='w',bg=BG,fg=MUTED,
                             font=(FONT,10),padx=7)
        self.status.pack(side='bottom',fill='x',pady=3)
        self.board=tk.Frame(outer,bg=BG)
        self.board.pack(fill='both',expand=True,padx=2,pady=(0,4))
        for i in range(4):self.board.columnconfigure(i,weight=1,uniform='col')
        for i in range(2):self.board.rowconfigure(i,weight=1,uniform='row')
        self.cards=[]
        for i in range(8):
            field = 'pitch' if i==5 else 'yaw' if i==6 else 'roll' if i==7 else None
            if field:
                label={'pitch':'PITCH / +X ROTATION','yaw':'YAW / +Y ROTATION',
                       'roll':'AXIAL ROLL / +Z'}[field]
                card=Card(self.board,label,field,field)
            else:
                label,key,_=(IMU_CARDS if self.mode=='imu' else TELEMETRY_CARDS)[i]
                card=Card(self.board,label,'plot',key)
                card.visual.destroy()
                card.visual=LivePlotCanvas(card)
                card.visual.grid(row=2,column=0,sticky='nsew',padx=9,pady=(3,10))
            card.grid(row=i//4,column=i%4,sticky='nsew',padx=4,pady=4)
            self.cards.append(card)

    def clear_graphs(self):
        self.flight.reset_history()
        self.last_msg='GRAPH HISTORY CLEARED; CURRENT SENSOR ORIGIN RETAINED'

    def switch_view(self):
        self.mode='telemetry' if self.mode=='imu' else 'imu'
        self.switch_btn.config(text='SHOW TELEMETRY VIEW' if self.mode=='imu' else 'SHOW IMU VIEW')
        self.refresh()

    def command(self,letter):
        try:
            self.serial.write((letter+'\n').encode('ascii'))
            self.last_msg=('ZERO REQUEST SENT TO NANO' if letter=='Z' else
                           'GYRO CALIBRATION REQUESTED; KEEP SENSOR MOTIONLESS')
        except Exception as exc:
            self.last_msg=f'COMMAND FAILED: {exc}'

    def open_3d(self):
        if self.rocket3d is not None:
            try:
                if self.rocket3d.winfo_exists():
                    self.rocket3d.lift()
                    return
            except tk.TclError:pass
        self.rocket3d=Rocket3DWindow(self)
        self.rocket3d.update_plot(self.flight,self.flight.sample(1))

    def tick(self):
        try:
            packets, messages=self.pump.poll()
            if messages:
                for line in messages:
                    self.last_msg=line.lstrip('# ').strip()
                    if 'ORIGIN_ZEROED' in line:
                        # Keep the graph from before the command? No: a new origin is a new session.
                        self.flight.reset_history()
                    if 'CALIBRATING' in line:
                        self.last_frame_wall=None
            if packets and self.last_msg == 'WAITING FOR NANO TO INITIALISE':
                self.last_msg = 'RECEIVING SENSOR DATA'
            for p in packets:
                self.flight.add(p)
                self.last_frame_wall=time.monotonic()
                if self.writer:
                    ms,q,accel,gyro,still=p
                    self.writer.writerow([ms,*q,*accel,*gyro,1 if still else 0])
            if self.log and packets:self.log.flush()
            self.refresh()
        except Exception as err:
            self.last_msg=f'SERIAL CONNECTION LOST: {err}'
            try:self.serial.close()
            except Exception:pass
            self.refresh()
            messagebox.showerror('Nano serial connection lost',str(err))
            return
        self.after(70,self.tick)

    def refresh(self):
        f=self.flight
        frame=f.frames[-1] if f.frames else None
        self.time_label.config(text=f'LIVE SENSOR TIME: {fmt_time(f.seconds)}')
        for i,card in enumerate(self.cards):
            if i<5:
                label,key,units=(IMU_CARDS if self.mode=='imu' else TELEMETRY_CARDS)[i]
                # Update original card label without reconstructing its layout.
                card.winfo_children()[0].config(text=label)
                value=frame.get(key) if frame else None
                card.value.config(text=f'{value:,.2f} {units}' if value is not None else 'N/A')
                card.visual.update_history(f.history(key),available=key in ('ax','ay','az','gx','gy','gz','gyro_mag','accel_mag'))
            else:
                field=('pitch','yaw','roll')[i-5]
                val=frame.get(field) if frame else None
                card.value.config(text=f'{val:+.2f} °' if val is not None else 'N/A')
                card.visual.set_angle(val)
        if self.rocket3d is not None:
            try:
                if self.rocket3d.winfo_exists():
                    self.rocket3d.update_plot(f,f.sample(1))
                else:self.rocket3d=None
            except tk.TclError:self.rocket3d=None
        if self.last_frame_wall is None:
            stream='CALIBRATING / WAITING'
        else:
            age=time.monotonic()-self.last_frame_wall
            stream='LIVE' if age < 1 else f'STALE ({age:.1f}s since last packet)'
        still='?' if frame is None or frame.stationary is None else 'YES' if frame.stationary else 'NO'
        self.status.config(text=f'{self.port_name} • {stream} • {f.received} packets • '
                          f'STATIONARY: {still}  |  {self.last_msg}  |  '
                          f'NO ALTITUDE/PRESSURE/RSSI FROM MPU6050')

    def shutdown(self):
        try:self.serial.close()
        except Exception:pass
        if self.log:
            try:self.log.close()
            except Exception:pass
        self.destroy()


def run_live(args):
    import serial
    try:
        conn=serial.Serial(args.port,args.baud,timeout=0)
    except serial.SerialException as exc:
        raise OSError(f'Cannot open {args.port}. Close PlatformIO Serial Monitor and 3D rocket viewer: {exc}') from exc
    try:
        app=LiveGroundStation(conn,args.port,nose_axis=args.nose_axis,turn=args.turn,
                              view=args.view,record=args.record,auto_3d=not args.no_3d)
        app.mainloop()
    finally:
        if conn.is_open:conn.close()
    return 0
