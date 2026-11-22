#!/usr/bin/env python3
"""Generate WD_Description/model.sdf from assembly_poses.json + STL meshes (mm).
Edit DENSITY / MASS_OVERRIDE_KG below and re-run:  python3 gen_sdf.py
"""
import json, struct, math, numpy as np, os
HERE = os.path.dirname(os.path.abspath(__file__))
DENSITY = 1040.0            # kg/m^3 (ABS). Change for your plastic (PLA ~1240, PETG ~1270, nylon ~1150)
MASS_OVERRIDE_KG = {}       # e.g. {"motor": 0.0065}  keys: body3, body4, motor, blade_cw, blade_ccw (per instance)
MESH = {'Body3':'body3','Body4':'body4','Motor_coreless':'motor_coreless',
        'Imported2_CW_Blade':'blade_cw','Imported2_CCW_Blade':'blade_ccw'}
MESH_DIR = os.path.join(HERE, 'WD_Description', 'meshes')

def load_stl(p):
    d = open(p, 'rb').read(); n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84+n*50], dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
    return a['v'].astype(float) * 1e-3          # metres

def mass_props(T, rho):
    a, b, c = T[:,0], T[:,1], T[:,2]
    v = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6.0
    V = v.sum(); cen = ((a+b+c)/4 * v[:,None]).sum(0) / V
    s = a+b+c
    C = sum(vi/20.0*(np.outer(x,x)+np.outer(y,y)+np.outer(z,z)+np.outer(w,w))
            for vi,x,y,z,w in zip(v,a,b,c,s))
    C = C - V*np.outer(cen, cen)
    I = (np.trace(C)*np.eye(3) - C) * rho
    return V*rho, cen, I

def rpy(R):
    sp = -R[2,0]; pitch = math.asin(max(-1,min(1,sp)))
    if abs(sp) < 0.999999:
        roll = math.atan2(R[2,1], R[2,2]); yaw = math.atan2(R[1,0], R[0,0])
    else:
        roll = math.atan2(-R[1,2], R[1,1]); yaw = 0.0
    return roll, pitch, yaw

P = json.load(open(os.path.join(HERE, 'assembly_poses.json')))
props = {}
for part, fn in MESH.items():
    # vectorised-ish: use fewer python loops for big meshes
    T = load_stl(os.path.join(MESH_DIR, fn + '.stl'))
    props[part] = mass_props(T, DENSITY)

def f(x): return f'{x:.9g}'
def link_xml(name, part, kind, collide):
    inst = P[name]; A = np.array(inst['T']); R = A[:3,:3]; t = A[:3,3]*1e-3
    r = rpy(R); off = np.array(inst['mesh_offset_mm'])*1e-3
    m, cen, I = props[part]
    key = {'Body3':'body3','Body4':'body4','Motor_coreless':'motor','Imported2_CW_Blade':'blade_cw','Imported2_CCW_Blade':'blade_ccw'}[part]
    mass = MASS_OVERRIDE_KG.get(key, m)
    I = I * (mass / m)
    c = cen + off
    uri = f'model://WD_Description/meshes/{MESH[part]}.stl'
    s = f'''    <link name="{name}">
      <pose>{f(t[0])} {f(t[1])} {f(t[2])} {f(r[0])} {f(r[1])} {f(r[2])}</pose>
      <inertial>
        <pose>{f(c[0])} {f(c[1])} {f(c[2])} 0 0 0</pose>
        <mass>{f(mass)}</mass>
        <inertia>
          <ixx>{f(I[0,0])}</ixx><ixy>{f(I[0,1])}</ixy><ixz>{f(I[0,2])}</ixz>
          <iyy>{f(I[1,1])}</iyy><iyz>{f(I[1,2])}</iyz><izz>{f(I[2,2])}</izz>
        </inertia>
      </inertial>
      <visual name="{name}_visual">
        <pose>{f(off[0])} {f(off[1])} {f(off[2])} 0 0 0</pose>
        <geometry><mesh><uri>{uri}</uri><scale>0.001 0.001 0.001</scale></mesh></geometry>
        <material><ambient>{kind}</ambient><diffuse>{kind}</diffuse></material>
      </visual>
'''
    if collide:
        s += f'''      <collision name="{name}_collision">
        <pose>{f(off[0])} {f(off[1])} {f(off[2])} 0 0 0</pose>
        <geometry><mesh><uri>{uri}</uri><scale>0.001 0.001 0.001</scale></mesh></geometry>
      </collision>
'''
    return s + '    </link>\n'

GREY='0.75 0.75 0.8 1'; DARK='0.2 0.2 0.22 1'; GREEN='0.3 0.8 0.3 1'
x = ['<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="WD_Description">\n',
     '    <!-- Frame = SolidWorks assembly frame (Y up, metres). Mesh scale 0.001 (STL in mm). -->\n']
x.append(link_xml('body4','Body4',GREY,True))
for n in ('body3_1','body3_2'): x.append(link_xml(n,'Body3',GREY,True))
for i in range(1,5): x.append(link_xml(f'motor_{i}','Motor_coreless',DARK,True))
BL = {1:'Imported2_CW_Blade',2:'Imported2_CCW_Blade',3:'Imported2_CCW_Blade',4:'Imported2_CW_Blade'}
for i in range(1,5): x.append(link_xml(f'blade_{i}',BL[i],GREEN,False))
x.append('\n    <!-- Rigid connections (mates: coincident + concentric with rotation locked) -->\n')
def fixed(n,p,c): return f'    <joint name="{n}" type="fixed"><parent>{p}</parent><child>{c}</child></joint>\n'
x.append(fixed('body4_to_body3_1','body4','body3_1')); x.append(fixed('body4_to_body3_2','body4','body3_2'))
for i,b in ((1,'body3_1'),(2,'body3_1'),(3,'body3_2'),(4,'body3_2')): x.append(fixed(f'{b}_to_motor_{i}',b,f'motor_{i}'))
x.append('\n    <!-- Rotors: concentric mate without rotation lock -> free revolute about the shared axis (child-frame Y) -->\n')
for i in range(1,5):
    x.append(f'''    <joint name="rotor_{i}_joint" type="revolute">
      <parent>motor_{i}</parent><child>blade_{i}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><damping>1e-6</damping></dynamics></axis>
    </joint>
''')
x.append('\n    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>\n')
for i in range(1,5):
    x.append(f'''    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>rotor_{i}_joint</joint_name><topic>/wd/rotor_{i}/cmd_vel</topic><use_force_commands>false</use_force_commands>
    </plugin>
''')
x.append('  </model>\n</sdf>\n')
open(os.path.join(HERE,'WD_Description','model.sdf'),'w').write(''.join(x))
tot = sum(MASS_OVERRIDE_KG.get(k,0) or 0 for k in [])
print({k:(round(v[0]*1000,3),'g') for k,v in props.items()})
