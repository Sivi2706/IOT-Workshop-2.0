
import math

# Quaternion format: (w, x, y, z)

def normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-10 or not math.isfinite(n):
        raise ValueError("Invalid quaternion")
    return tuple(v / n for v in q)


def conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def multiply(a, b):
    w, x, y, z = a
    v, p, q, r = b

    return (
        w*v - x*p - y*q - z*r,
        w*p + x*v + y*r - z*q,
        w*q - x*r + y*v + z*p,
        w*r + x*q - y*p + z*v
    )


def delta_axis_angle(current, previous):
    d = normalize(multiply(current, conjugate(previous)))

    if d[0] < 0:
        d = tuple(-v for v in d)

    w, x, y, z = d
    sinhalf = math.sqrt(x*x + y*y + z*z)

    if sinhalf < 1e-8:
        return (0, 0, 1), 0.0

    return (
        (x/sinhalf, y/sinhalf, z/sinhalf),
        2.0 * math.atan2(sinhalf, w)
    )


def euler_degrees(q):
    w, x, y, z = normalize(q)

    roll = math.atan2(
        2*(w*x + y*z),
        1 - 2*(x*x + y*y)
    )

    pitch = math.asin(
        max(-1.0, min(1.0, 2*(w*y - z*x)))
    )

    yaw = math.atan2(
        2*(w*z + x*y),
        1 - 2*(y*y + z*z)
    )

    return tuple(
        math.degrees(v) for v in (roll, pitch, yaw)
    )


def q_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)

    return normalize((
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ))


def parse_packet(line):
    fields = line.strip().split(',')

    if len(fields) != 12 or fields[0] != 'Q':
        return None

    try:
        t = int(fields[1])
        nums = [float(v) for v in fields[2:]]

        if not all(math.isfinite(v) for v in nums):
            return None

        q = normalize(tuple(nums[:4]))

        return (
            t,
            q,
            tuple(nums[4:7]),
            tuple(nums[7:10])
        )

    except (ValueError, OverflowError):
        return None
