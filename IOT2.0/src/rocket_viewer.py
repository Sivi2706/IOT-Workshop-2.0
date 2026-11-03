
"""VPython Nano+MPU6050 rocket viewer: corrected reference and sensor axes.

From VS Code's PlatformIO project root:
    py src/rocket_viewer.py --demo
    py src/rocket_viewer.py --port COM4 --nose-axis=+z

If MPU6050 +X points along the imagined rocket nose: --nose-axis=+x
If the axes seem rotated around the nose: add --turn 90 (or 180/270).

Hardware must run V2 main.cpp, which transmits SENSOR-REFERENCE deltas.
"""
import argparse
import math
import time
from attitude_math import (alignment,conjugate,delta_axis_angle,euler_degrees,
                           map_sensor_to_rocket,multiply,parse_packet,q_from_euler)


def choose_port(requested):
    if requested:
        return requested
    import serial.tools.list_ports
    ports=list(serial.tools.list_ports.comports())
    if len(ports)==1:
        return ports[0].device
    print('Available serial ports:')
    for p in ports:
        print(f' {p.device}: {p.description}')
    raise SystemExit('Specify --port COM4 (or the COM port listed in Device Manager)')


def rocket_scene():
    from vpython import canvas,vec,color,box,cylinder,cone,compound,arrow,label
    scene=canvas(title='NANO MPU6050 | CORRECTED 3D ROCKET ATTITUDE',
                 width=1170,height=760,background=vec(.045,.065,.11),
                 center=vec(0,0,0),range=4.2)
    scene.up=vec(0,0,1)
    scene.camera.pos=vec(6,-9,5)
    scene.camera.axis=vec(-6,9,-5)
    scene.caption='Starting...  |  Drag scene to rotate view; scroll to zoom.\n'

    # WORLD axes (fixed). Physical sensor axes become aligned to these at zero.
    for axis,clr,txt in [(vec(1,0,0),color.red,'Model +X (PITCH axis)'),
                         (vec(0,1,0),color.green,'Model +Y (YAW axis)'),
                         (vec(0,0,1),color.cyan,'Model +Z (NOSE / ROLL axis)')]:
        arrow(pos=vec(0,0,0),axis=axis*3.0,shaftwidth=.038,color=clr)
        label(pos=axis*3.6,text=txt,height=11,box=False,opacity=0,color=clr)

    # Fixed ghost shows where the rocket was at its last zeroed orientation.
    cylinder(pos=vec(0,0,-1.35),axis=vec(0,0,2.7),radius=.15,
             color=vec(.52,.59,.68),opacity=.15)
    cone(pos=vec(0,0,1.35),axis=vec(0,0,.8),radius=.15,
         color=vec(.52,.59,.68),opacity=.15)
    parts=[]
    parts.append(cylinder(pos=vec(0,0,-1.4),axis=vec(0,0,2.8),radius=.21,
                          color=vec(.85,.89,.95)))
    parts.append(cone(pos=vec(0,0,1.4),axis=vec(0,0,.84),radius=.21,
                      color=vec(1.,.31,.16)))
    parts.append(cylinder(pos=vec(0,0,-1.4),axis=vec(0,0,.24),radius=.245,
                          color=vec(.25,.55,.82)))
    parts.append(cylinder(pos=vec(0,0,.50),axis=vec(0,0,.12),radius=.216,
                          color=vec(.25,.55,.82)))
    # Asymmetric fin makes rotation around the model's +Z longitudinal axis visible.
    parts.append(box(pos=vec(.36,0,-1.0),size=vec(.72,.09,.72),color=vec(1.,.45,.10)))
    parts.append(box(pos=vec(-.36,0,-1.0),size=vec(.72,.09,.72),color=vec(.29,.62,.91)))
    parts.append(box(pos=vec(0,.36,-1.0),size=vec(.09,.72,.72),color=vec(.29,.62,.91)))
    parts.append(box(pos=vec(0,-.36,-1.0),size=vec(.09,.72,.72),color=vec(.29,.62,.91)))
    parts.append(box(pos=vec(.21,0,.15),size=vec(.022,.09,.96),color=vec(1.,.45,.10)))
    # Short coloured MODEL-LOCAL axes attached to rocket: show pitch/yaw/roll.
    parts.append(cylinder(pos=vec(0,0,0),axis=vec(.8,0,0),radius=.035,color=color.red))
    parts.append(cylinder(pos=vec(0,0,0),axis=vec(0,.8,0),radius=.035,color=color.green))
    parts.append(cylinder(pos=vec(0,0,0),axis=vec(0,0,2.43),radius=.025,color=color.cyan))
    rocket=compound(parts,origin=vec(0,0,0))
    return scene,rocket


def main():
    p=argparse.ArgumentParser(description='MPU6050 3D rocket attitude with sensor-axis mapping')
    p.add_argument('--port',default=None,help='Arduino serial port, e.g. COM4')
    p.add_argument('--baud',type=int,default=115200)
    p.add_argument('--demo',action='store_true',help='Animate without Nano connected')
    p.add_argument('--nose-axis',default='+z',
                   choices=['+z','-z','+x','-x','+y','-y'],
                   help='MPU6050 axis pointing toward the imaginary rocket nose at zero (default +z)')
    p.add_argument('--turn',type=int,default=0,choices=[0,90,180,270],
                   help='Additional rotation about rocket +Z to orient sensor X/Y relative to camera')
    args=p.parse_args()
    align=alignment(args.nose_axis,args.turn)

    ser=None
    if not args.demo:
        import serial
        port=choose_port(args.port)
        print(f'Connecting to {port} at {args.baud} baud. Close PlatformIO Serial Monitor first.')
        try:
            ser=serial.Serial(port,args.baud,timeout=0)
        except serial.SerialException as err:
            raise SystemExit(f'Cannot open port: {err}') from err

    from vpython import rate,vec
    scene,rocket=rocket_scene()
    previous=(1.,0.,0.,0.)
    latest_sensor=(1.,0.,0.,0.)
    last_packet=time.monotonic()
    last_screen=0.
    last_ms=0
    accel=(0.,0.,1.)
    gyro=(0.,0.,0.)
    stationary=None
    status='DEMO' if args.demo else 'WAITING FOR NANO BOOT/CALIBRATION'
    rx=bytearray()
    demo_ref=(1.,0.,0.,0.)
    start=time.monotonic()
    version_seen=False

    def keyboard(evt):
        nonlocal demo_ref,status
        k=(evt.key or '').lower()
        if k=='z':
            if ser:
                ser.write(b'Z\n')
                status='REQUESTED NEW ORIGIN'
            else:
                t=time.monotonic()-start
                demo_ref=q_from_euler(.48*math.sin(.7*t),.38*math.sin(.42*t),
                                      .62*math.sin(.29*t))
                status='DEMO ORIGIN RESET'
        elif k=='c':
            if ser:
                ser.write(b'C\n')
                status='RECALIBRATING; HOLD STILL'
            else:
                demo_ref=(1.,0.,0.,0.)
                status='DEMO ORIGIN RESET'

    scene.bind('keydown',keyboard)
    print('Keyboard: Z = zero current pose; C = full gyro calibration (keep board still).')
    print('For rocket +Z nose, board +Z normal = --nose-axis=+z, +X edge = --nose-axis=+x.')
    print('Use the moving orange fin and the RGB axes to identify movement directions.')
    try:
        while True:
            rate(60)
            now=time.monotonic()
            if args.demo:
                t=now-start
                raw=q_from_euler(.48*math.sin(.7*t),.38*math.sin(.42*t),
                                 .62*math.sin(.29*t))
                latest_sensor=multiply(conjugate(demo_ref),raw)
                last_ms=int(t*1000)
                stationary=False
                last_packet=now
                status='DEMO (NO HARDWARE)'
            else:
                count=min(ser.in_waiting,4096)
                if count:
                    rx.extend(ser.read(count))
                if len(rx)>8192:
                    rx.clear()
                    status='SERIAL BUFFER RESET'
                for _ in range(90):
                    j=rx.find(b'\n')
                    if j<0: break
                    text=bytes(rx[:j]).decode('ascii','replace').strip()
                    del rx[:j+1]
                    if text.startswith('#'):
                        print(text)
                        if 'ROCKET ATTITUDE V2' in text:
                            version_seen=True
                        if 'CALIBRATING' in text:
                            status='CALIBRATING - HOLD STILL'
                        elif 'ORIGIN_ZEROED' in text:
                            status='ORIGIN RESET'
                        elif 'READY' in text:
                            status='RECEIVING ATTITUDE'
                        elif 'ERROR' in text or 'FATAL' in text:
                            status=text[2:]
                        continue
                    pkt=parse_packet(text)
                    if pkt is None:
                        continue
                    last_ms,latest_sensor,accel,gyro,stationary=pkt
                    last_packet=now
                    status=('STREAMING' if version_seen or stationary is not None else
                            'LEGACY FIRMWARE - INSTALL V2 main.cpp')
                if now-last_packet>2.5:
                    status='STALE: NO RECENT SENSOR DATA'
            # Sensor-frame delta -> model-frame delta; never convert to Euler
            # angles for visual animation (no Euler gimbal lock).
            model_q=map_sensor_to_rocket(latest_sensor,align)
            axis,angle=delta_axis_angle(model_q,previous)
            if angle>1e-7:
                rocket.rotate(axis=vec(*axis),angle=angle,origin=vec(0,0,0))
            previous=model_q
            if now-last_screen>0.1:
                x,y,z=euler_degrees(model_q)
                if stationary is True: still_text='YES - ZARU ACTIVE'
                elif stationary is False: still_text='NO - MOVING'
                else: still_text='UNKNOWN'
                scene.caption=(
                    f'STATUS: {status}    t={last_ms/1000:.2f}s     '
                    f'STATIONARY: {still_text}\n'
                    f'ROCKET PITCH (rotation about +X): {x:+7.1f} deg   '
                    f'ROCKET YAW (rotation about +Y): {y:+7.1f} deg   '
                    f'AXIAL ROLL (+Z): {z:+7.1f} deg\n'
                    f'SENSOR ACCEL(g) X={accel[0]:+.2f} Y={accel[1]:+.2f} Z={accel[2]:+.2f}   '
                    f'BIAS-CORRECTED GYRO(deg/s) X={gyro[0]:+.2f} Y={gyro[1]:+.2f} Z={gyro[2]:+.2f}\n'
                    f'MOUNT: nose={args.nose_axis}, turn={args.turn} deg   '
                    'Z=ZERO CURRENT POSE   C=CALIBRATE WHILE STILL\n'
                    'Ghost = origin; orange fin = roll marker. MPU6050 yaw has no absolute heading reference.\n'
                )
                last_screen=now
    except KeyboardInterrupt:
        print('Closing ground station.')
    finally:
        if ser: ser.close()


if __name__=='__main__':
    main()
