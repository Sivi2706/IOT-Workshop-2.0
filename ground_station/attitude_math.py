"""Quaternion helpers for Arduino Nano v2 sensor-frame orientation packets.

Conventions:
    q=(w,x,y,z), active rotations; +Z is the rocket's NOSE axis.
    q_from_Nano = conjugate(q_calibrated) * q_current, expressed in sensor axes.
    align maps sensor reference axes -> model reference axes.
    q_model = align * q_from_Nano * conjugate(align)
"""
import math


def normalize(q):
    n=math.sqrt(sum(x*x for x in q))
    if not math.isfinite(n) or n<1e-10:
        raise ValueError('Invalid quaternion')
    return tuple(x/n for x in q)


def conjugate(q):
    w,x,y,z=q
    return (w,-x,-y,-z)


def multiply(a,b):
    w,x,y,z=a
    v,p,q,r=b
    return (w*v-x*p-y*q-z*r, w*p+x*v+y*r-z*q,
            w*q-x*r+y*v+z*p, w*r+x*q-y*p+z*v)


def q_from_euler(x,y,z):
    cx,sx=math.cos(x/2),math.sin(x/2)
    cy,sy=math.cos(y/2),math.sin(y/2)
    cz,sz=math.cos(z/2),math.sin(z/2)
    return normalize((cx*cy*cz+sx*sy*sz,
                      sx*cy*cz-cx*sy*sz,
                      cx*sy*cz+sx*cy*sz,
                      cx*cy*sz-sx*sy*cz))


def delta_axis_angle(current,previous):
    d=normalize(multiply(current,conjugate(previous)))
    if d[0]<0: d=tuple(-v for v in d)
    w,x,y,z=d
    sn=math.sqrt(x*x+y*y+z*z)
    if sn<1e-8: return ((0.,0.,1.),0.)
    return ((x/sn,y/sn,z/sn),2.0*math.atan2(sn,w))


def euler_degrees(q):
    w,x,y,z=normalize(q)
    roll_x=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch_y=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
    yaw_z=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return tuple(math.degrees(v) for v in (roll_x,pitch_y,yaw_z))


def alignment(nose_axis='+z',turn=0):
    """Fixed mounting change of basis from sensor axes to 3D rocket model.

    Nose axis is the MPU6050 axis pointing towards the model rocket's nose
    when the sensor is at the calibrated zero position. Additional rotation
    `turn` (0,90,180,270 degrees) rotates X/Y alignment around model +Z.
    """
    a=nose_axis.strip().lower()
    if a=='+z': base=q_from_euler(0.,0.,0.)
    elif a=='-z': base=q_from_euler(math.pi,0.,0.)
    elif a=='+x': base=q_from_euler(0.,-math.pi/2,0.)
    elif a=='-x': base=q_from_euler(0.,math.pi/2,0.)
    elif a=='+y': base=q_from_euler(math.pi/2,0.,0.)
    elif a=='-y': base=q_from_euler(-math.pi/2,0.,0.)
    else: raise ValueError('nose axis must be one of +x,-x,+y,-y,+z,-z')
    return normalize(multiply(q_from_euler(0,0,math.radians(turn)),base))


def map_sensor_to_rocket(q_sensor,align):
    return normalize(multiply(multiply(align,q_sensor),conjugate(align)))


def rotate_vector(q,v):
    return multiply(multiply(q,(0.,*v)),conjugate(q))[1:]


def parse_packet(line):
    """Accept v2 13-value packets or legacy 12-value packets; return 5-tuple.

    Legacy packets have different reference-frame semantics. Use v2 Nano
    firmware with this viewer if --nose-axis changes are required.
    """
    f=line.strip().split(',')
    if f[0]!='Q' or len(f) not in (12,13): return None
    try:
        t=int(f[1]); values=[float(x) for x in f[2:]]
        if not all(math.isfinite(v) for v in values): return None
        quat=normalize(tuple(values[0:4]))
        still=None if len(f)==12 else (values[10]>0.5)
        return (t,quat,tuple(values[4:7]),tuple(values[7:10]),still)
    except (ValueError, OverflowError,IndexError):
        return None
