import sys, json, numpy as np, trimesh
from trimesh.transformations import euler_matrix, translation_matrix
M="/home/claude/tesis_yay/src/bumperbot_description/meshes/"
def T(xyz=(0,0,0), rpy=(0,0,0)):
    return translation_matrix(xyz) @ euler_matrix(*rpy, 'sxyz')
base_up = T((0,0,0.033))
def load(n): return trimesh.load(M+n+".STL")
out={}
b=load("base_link"); parts=b.split(only_watertight=False)
groups={"placa_base":[], "placa_medio":[], "placa_tope":[], "resto":[]}
for p in parts:
    z0=p.bounds[0][2]
    if len(p.faces)>20000 and -0.03<z0<-0.005: groups["placa_base"].append(p)
    elif len(p.faces)>20000 and 0.015<z0<0.035: groups["placa_medio"].append(p)
    elif len(p.faces)>20000 and 0.065<z0<0.085: groups["placa_tope"].append(p)
    else: groups["resto"].append(p)
for k,v in groups.items():
    m=trimesh.util.concatenate(v); m.apply_transform(base_up); out[k]=m
def place(name, joint, visual=((0,0,0),(0,0,0))):
    m=load(name); m.apply_transform(base_up @ T(*joint) @ T(*visual)); return m
out["rueda_der"]=place("wheel_right_link", ((0,-0.0701101849418637,0),(0,0,0)), ((0,0,0),(1.57,0,0)))
out["rueda_izq"]=place("wheel_left_link", ((0,0.0701101849418642,0),(0,0,0)), ((0,0,0),(-1.57,0,0)))
out["loca_del"]=place("caster_front_link", ((0.04755,0,-0.0275),(0,0,0)))
out["loca_tras"]=place("caster_rear_link", ((-0.04755,0,-0.0275),(0,0,0)))
out["imu"]=place("imu_link", ((0,0,0.0698986241758014),(0,0,0)))
out["lidar"]=place("laser_link", ((-0.0050526,-0.0023221,0.1208),(0,0,3.14)))
info={}
for k,m in out.items():
    m.export(f"assets/{k}.glb"); info[k]=m.bounds.round(4).tolist()
json.dump(info, open("assets/bounds.json","w"), indent=1)
print(json.dumps(info, indent=1))
