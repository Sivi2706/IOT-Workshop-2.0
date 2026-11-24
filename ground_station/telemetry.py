"""Read OpenRocket CSV exports and interpolate their flight-state measurements.

All values are normalised to seconds, m, m/s, deg, C and mbar.
Absent measurements remain None; no simulated RSSI or attitude is invented.
"""
from __future__ import annotations

import bisect
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

FIELDS = {
    'time': ['time', 'simulationtime', 'times', 'timesec', 'timeelapsed', 'elapsedtime', 'timesinceignition'],
    'altitude': ['altitude', 'altitudeaboveground', 'height', 'heightaboveground', 'baroalt', 'altitudem'],
    'vertical_velocity': ['verticalvelocity', 'verticalspeed', 'vz', 'velocityz', 'upwardvelocity'],
    'speed': ['totalvelocity', 'velocity', 'speed', 'totalspeed'],
    'pressure': ['airpressure', 'ambientpressure', 'atmosphericpressure', 'pressure', 'barometricpressure'],
    'temperature': ['airtemperature', 'ambienttemperature', 'temperature'],
    'pitch': ['pitch', 'pitchangle'],
    'yaw': ['yaw', 'yawangle'],
    'roll': ['rollangle', 'rollorientation', 'roll', 'rollposition'],
    'roll_rate': ['rollrate', 'angularvelocityroll'],
    'zenith': ['verticalorientationzenith', 'verticalorientation', 'zenith', 'zenithangle', 'orientationtheta', 'theta'],
    'azimuth': ['lateralorientationazimuth', 'lateralorientation', 'azimuth', 'azimuthangle', 'orientationphi', 'phi'],
    'east': ['positioneastoflaunch', 'eastposition', 'positioneast', 'east', 'positionx', 'xposition'],
    'north': ['positionnorthoflaunch', 'northposition', 'positionnorth', 'north', 'positiony', 'yposition'],
}

@dataclass
class Measurement:
    time: float
    altitude: Optional[float] = None
    velocity: Optional[float] = None
    pressure_mbar: Optional[float] = None
    temperature_c: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    roll: Optional[float] = None
    zenith: Optional[float] = None
    azimuth: Optional[float] = None
    east: Optional[float] = None
    north: Optional[float] = None


def strip_unit(header: str) -> str:
    return re.sub(r'\([^)]*\)|\[[^]]*\]', '', header).strip()


def norm(header: str) -> str:
    return re.sub(r'[^a-z0-9]', '', strip_unit(header).lower())


def find_columns(headers: list[str]) -> dict[str, int]:
    result = {}
    for field, aliases in FIELDS.items():
        for candidate in aliases:
            for idx, header in enumerate(headers):
                if norm(header) == candidate:
                    result[field] = idx
                    break
            if field in result:
                break
    return result


def unit(header: str) -> str:
    matches = list(re.finditer(r'\(([^)]*)\)|\[([^]]*)\]', header))
    match = matches[-1] if matches else None
    u = (match.group(1) or match.group(2)).strip().lower() if match else ''
    return u.replace('°','deg').replace(' ', '').replace('²','2')


def to_base(v: Optional[float], header: str, kind: str) -> Optional[float]:
    if v is None: return None
    u = unit(header)
    if kind == 'time':
        if u in ('ms','millisecond','milliseconds'): return v/1000
        return v
    if kind in ('altitude', 'east', 'north'):
        if u in ('ft','feet','foot'): return v*0.3048
        if u in ('km',): return v*1000
        return v
    if kind in ('vertical_velocity', 'speed'):
        if 'ft/s' in u or 'fps' in u: return v*0.3048
        if 'km/h' in u or 'kmh' in u: return v/3.6
        return v
    if kind == 'pressure':
        if u in ('kpa',): return v*10
        if u in ('mbar','hpa','mb'): return v
        if u in ('bar',): return v*1000
        if u in ('psi',): return v*68.94757
        return v/100.0  # OpenRocket SI pressure output: Pa
    if kind == 'temperature':
        if u in ('degc','c','celsius'): return v
        if u in ('degf','f','fahrenheit'): return (v-32)*5/9
        return v-273.15  # OpenRocket native SI temperature: kelvin
    if kind in ('pitch','yaw','roll','zenith','azimuth'):
        if u in ('rad','radian','radians') or not u:
            return math.degrees(v)  # OpenRocket native SI orientation uses rad
        return v
    if kind == 'roll_rate':
        if u in ('rad/s','radians/s','radian/s') or not u: return math.degrees(v)
        return v
    return v


def safe_float(value: str) -> Optional[float]:
    try:
        x = float(value.strip())
        return x if math.isfinite(x) else None
    except (ValueError, TypeError, AttributeError):
        return None


def unwrap_deg(values: list[Optional[float]]) -> list[Optional[float]]:
    prev = None
    offset = 0.0
    result = []
    for v in values:
        if v is None:
            result.append(None)
            continue
        if prev is not None:
            diff = v-prev
            if diff > 180: offset -= 360
            elif diff < -180: offset += 360
        result.append(v + offset)
        prev = v
    return result


class Flight:
    def __init__(self, timestamps: list[float], series: dict[str, list[Optional[float]]],
                 title: str, headers: list[str], warnings: list[str]):
        self.times = timestamps
        self.series = series
        self.title = title
        self.headers = headers
        self.warnings = warnings
        self.duration = max(0.0, timestamps[-1]-timestamps[0])
        self.t0 = timestamps[0]
        self.has_attitude = any(series.get(k) and any(x is not None for x in series[k])
                                for k in ('pitch','yaw','roll','zenith'))
        self.has_position = all(series.get(k) and any(x is not None for x in series[k])
                                for k in ('east','north'))
        self.attitude_source = ('direct' if series.get('pitch') or series.get('yaw')
                                else 'zenith' if series.get('zenith') and series.get('azimuth')
                                else 'missing')
        self._ensure_velocity()
        self._ensure_derived_attitude()

    @classmethod
    def from_csv(cls, path: str | Path) -> 'Flight':
        path = Path(path)
        text = path.read_text(encoding='utf-8-sig', errors='replace').splitlines()
        header_i = None
        delim = ','
        headers = []
        for i,line in enumerate(text):
            candidate = line.strip().lstrip('#').strip()
            if not candidate: continue
            for separator in (',','\t',';'):
                row = next(csv.reader([candidate], delimiter=separator, skipinitialspace=True))
                cols = find_columns(row)
                if 'time' in cols and 'altitude' in cols and len(row) >= 2:
                    header_i, delim, headers = i, separator, [v.strip().strip('"') for v in row]
                    break
            if header_i is not None: break
        if header_i is None:
            raise ValueError('Cannot find an OpenRocket header containing Time. In OpenRocket use Plot / Export > Export data and include Time.')
        selected = find_columns(headers)
        if 'altitude' not in selected:
            raise ValueError('Altitude column not found. Export the Altitude flight-data field in OpenRocket.')
        records = []
        for line in text[header_i+1:]:
            if not line.strip() or line.lstrip().startswith('#'): continue
            try: row = next(csv.reader([line],delimiter=delim,skipinitialspace=True))
            except csv.Error: continue
            if len(row) < len(headers): continue
            parsed = {}
            for field, idx in selected.items():
                parsed[field] = to_base(safe_float(row[idx]),headers[idx],field)
            if parsed.get('time') is not None and parsed.get('altitude') is not None:
                records.append(parsed)
        if len(records)<2: raise ValueError('Not enough numeric flight rows (minimum 2).')
        # Sort and collapse duplicate time samples; keep last record.
        samples = {r['time']:r for r in records}
        rows = [samples[k] for k in sorted(samples)]
        times = [r['time'] for r in rows]
        series = {k:[r.get(k) for r in rows] for k in selected if k!='time'}
        warnings = []
        if 'pressure' not in series: warnings.append('No air-pressure field: pressure displays N/A.')
        if 'temperature' not in series: warnings.append('No air-temperature field: temperature displays N/A.')
        if not ('pitch' in series or 'zenith' in series):
            warnings.append('No orientation columns: pitch/yaw/roll are not fabricated.')
        if not ('east' in series and 'north' in series):
            warnings.append('No east/north position columns: downrange trajectory unavailable.')
        return cls(times,series,path.name,headers,warnings)

    def _ensure_velocity(self):
        if self.series.get('vertical_velocity'):
            self.velocity_key = 'vertical_velocity'
            self.velocity_note = 'OpenRocket vertical velocity'
            return
        a = self.series['altitude']; t = self.times
        if len(t)>=2:
            vel=[]
            for i in range(len(t)):
                j,k=max(0,i-1),min(len(t)-1,i+1)
                dt=t[k]-t[j]
                vel.append((a[k]-a[j])/dt if dt>0 and a[j] is not None and a[k] is not None else None)
            self.series['vertical_velocity']=vel
            self.velocity_key = 'vertical_velocity'
            self.velocity_note = 'DERIVED from altitude (central differences)'
            self.warnings.append('Vertical velocity derived from altitude; export Vertical velocity for native values.')
        else:
            self.velocity_key = 'speed'
            self.velocity_note = 'Total speed (unsigned)'

    def _ensure_derived_attitude(self):
        s=self.series
        for k in ('pitch','yaw','roll','zenith','azimuth'):
            if k in s: s[k]=unwrap_deg(s[k])
        if 'pitch' not in s and 'zenith' in s and 'azimuth' in s:
            # Near vertical: apparent tilts about model X/Y; NOT physical Euler angles.
            # More exact orientation is reconstructed from zenith/azimuth in 3D view.
            s['pitch']=[v*math.sin(math.radians(a)) if v is not None and a is not None else None
                        for v,a in zip(s['zenith'],s['azimuth'])]
            s['yaw']=[v*math.cos(math.radians(a)) if v is not None and a is not None else None
                      for v,a in zip(s['zenith'],s['azimuth'])]
            self.warnings.append('Pitch/yaw shown as projections of OpenRocket zenith/azimuth, not Euler angles.')
        if 'roll' not in s and 'roll_rate' in s:
            rates=s['roll_rate']; out=[0.0]
            for i in range(1,len(self.times)):
                a,b=rates[i-1],rates[i]
                out.append(out[-1]+0.5*(a+b)*(self.times[i]-self.times[i-1])
                           if a is not None and b is not None else out[-1])
            s['roll'] = out
            self.warnings.append('Roll angle integrated from roll rate, relative to simulation start.')
        # Zero angle readouts to first valid sample to mirror origin calibration.
        for k in ('pitch','yaw','roll'):
            values=s.get(k)
            if values:
                initial=next((v for v in values if v is not None),0.0)
                s[k]=[v-initial if v is not None else None for v in values]

    def sample(self, seconds: float) -> Measurement:
        t = min(max(float(seconds)+self.t0, self.times[0]), self.times[-1])
        j = bisect.bisect_right(self.times,t)
        i=max(0,min(j-1,len(self.times)-1)); j=min(i+1,len(self.times)-1)
        weight=(t-self.times[i])/(self.times[j]-self.times[i]) if j!=i else 0.0
        def get(name):
            v=self.series.get(name)
            if not v: return None
            a,b=v[i],v[j]
            if a is None:return b
            if b is None:return a
            return a+(b-a)*weight
        return Measurement(time=t-self.t0,altitude=get('altitude'),
            velocity=get(self.velocity_key),pressure_mbar=get('pressure'),
            temperature_c=get('temperature'),pitch=get('pitch'),yaw=get('yaw'),
            roll=get('roll'),zenith=get('zenith'),azimuth=get('azimuth'),
            east=get('east'),north=get('north'))

    def history(self, field: str, seconds: float, limit: int=420):
        stop=min(len(self.times),bisect.bisect_right(self.times,self.t0+seconds))
        if stop<=0 or field not in self.series: return []
        step=max(1,(stop-1)//max(1,limit-1))
        idx=list(range(0,stop,step))
        if idx[-1]!=stop-1: idx.append(stop-1)
        return [(self.times[i]-self.t0,self.series[field][i]) for i in idx
                if self.series[field][i] is not None]


def write_synthetic_demo(path: str | Path):
    """A clearly labeled artificial demonstration; NOT an OpenRocket export."""
    path=Path(path)
    with path.open('w',newline='',encoding='utf-8') as fh:
        fh.write('# SYNTHETIC DEMO DATA - NOT GENERATED BY OPENROCKET\n')
        wr=csv.writer(fh)
        wr.writerow(['Time (s)','Altitude (m)','Vertical velocity (m/s)',
                     'Air pressure (Pa)','Air temperature (K)',
                     'Vertical orientation (zenith) (rad)',
                     'Lateral orientation (azimuth) (rad)','Roll angle (rad)',
                     'Position East of launch (m)','Position North of launch (m)'])
        # Piecewise smooth toy flight, not a rocket safety or flight model.
        duration=90.0; dt=0.1
        for n in range(int(duration/dt)+1):
            t=n*dt
            if t<3:
                h=15*t*t; v=30*t
            elif t<15:
                k=t-3; h=135+90*k-3.75*k*k; v=90-7.5*k
            elif t<30:
                h=675-11*(t-15); v=-11
            elif t<84:
                h=max(0,510-9.444444*(t-30)); v=-9.444444 if h>0 else 0
            else:
                h=0; v=0
            theta=math.radians(2.4+1.7*math.sin(t/6))
            phi=math.radians(10+16*math.sin(t/9))
            roll=math.radians(0.8*t+10*math.sin(t/15))
            east=0.42*t; north=0.1*t+1.4*math.sin(t/10)
            wr.writerow([f'{t:.3f}',f'{h:.4f}',f'{v:.4f}',
                f'{101325*math.exp(-h/8434.0):.3f}',f'{294.15-0.0065*h:.3f}',
                f'{theta:.7f}',f'{phi:.7f}',f'{roll:.7f}',f'{east:.3f}',f'{north:.3f}'])
    return path
