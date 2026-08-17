"""S1 mesh build: feature-preserving decimation per material group.
in : s1d/s1_description  (CAD OBJ + MTL, metres)
out: s1_out/s1_description  (OBJ+MTL for simulators, GLB with PBR materials for the web viewer,
                             convex-hull collision STL, URDFs pointing at the OBJs)"""
import trimesh, fast_simplification, numpy as np, os, glob, shutil, re
from trimesh.visual.material import PBRMaterial
SRC='s1d/s1_description'; OUT='s1_out/s1_description'
shutil.rmtree('s1_out', ignore_errors=True)
for d in ('meshes/visual','meshes/collision','urdf'): os.makedirs(f'{OUT}/{d}')

# target faces per link (visual). Detail kept, files small.
BUDGET={'base_link':45000,'link1':55000,'link2':60000,'link3':60000,'link4':70000,'link5':70000,
        'gripper_base_link':60000,'left_finger_link':40000,'right_finger_link':40000}
# physically based look per CAD material class (colour from MTL Kd unless overridden)
PBR={ 'aluminum_plate':dict(color=(0.86,0.87,0.88),metallic=1.0,roughness=0.32),
      'silver_metal':  dict(color=(0.80,0.80,0.79),metallic=1.0,roughness=0.28),
      'black_oxide':   dict(color=(0.06,0.06,0.065),metallic=0.85,roughness=0.45),
      'motor_dark_gray':dict(color=(0.20,0.205,0.215),metallic=0.75,roughness=0.48),
      'camera_black':  dict(color=(0.03,0.03,0.035),metallic=0.1,roughness=0.25),
      'rubber_black':  dict(color=(0.04,0.04,0.045),metallic=0.0,roughness=0.9),
      'soft_finger_orange':dict(color=(1.0,0.31,0.02),metallic=0.0,roughness=0.75),
      'pla_cf_black':  dict(color=(0.07,0.072,0.078),metallic=0.05,roughness=0.62)}
def pbr_for(name, kd):
    if name in PBR: p=PBR[name]
    elif 'graphite' in name or 'pla' in name: p=dict(PBR['pla_cf_black'])   # printed PLA-CF: black, MTL tints are CAD debug colours
    else: p=dict(color=tuple(kd),metallic=0.0,roughness=0.6)
    return p

# parse MTL Kd
KD={}
for blk in open(f'{SRC}/meshes/visual/s1_materials.mtl').read().split('newmtl')[1:]:
    n=blk.split()[0]; m=re.search(r'Kd\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)',blk); KD[n]=tuple(map(float,m.groups())) if m else (0.5,0.5,0.5)

def parse_obj_groups(path):
    """OBJ -> {material: (V, F)} with per-group faces (positions only)."""
    V=[]; groups={}; cur=None
    for line in open(path):
        if line.startswith('v '): V.append(list(map(float,line.split()[1:4])))
        elif line.startswith('usemtl'): cur=line.split()[1]; groups.setdefault(cur,[])
        elif line.startswith('f '):
            idx=[int(t.split('/')[0])-1 for t in line.split()[1:]]
            for k in range(1,len(idx)-1): groups[cur].append([idx[0],idx[k],idx[k+1]])
    V=np.array(V,dtype=np.float64)
    out={}
    for m,F in groups.items():
        F=np.array(F,dtype=np.int64); used=np.unique(F); remap=-np.ones(len(V),dtype=np.int64); remap[used]=np.arange(len(used))
        out[m]=(V[used].copy(), remap[F])
    return out

def simplify(V,F,target):
    m=trimesh.Trimesh(V,F,process=True); m.merge_vertices(merge_tex=True,merge_norm=True)
    V,F=m.vertices.astype(np.float64),m.faces.astype(np.int64)
    for _ in range(5):
        if len(F)<=target*1.05: break
        V,F=fast_simplification.simplify(V,F,target_reduction=min(0.85,1-target/len(F)),agg=8)
        t=trimesh.Trimesh(V,F,process=True); t.merge_vertices(merge_tex=True,merge_norm=True)
        V,F=t.vertices.astype(np.float64),t.faces.astype(np.int64)
    return trimesh.Trimesh(V,F,process=True)

mtl_out=[]; tot_in=tot_out=0
for f in sorted(glob.glob(f'{SRC}/meshes/visual/*.obj')):
    name=os.path.basename(f)[:-4]; groups=parse_obj_groups(f)
    n_in=sum(len(F) for _,F in groups.values()); tot_in+=n_in
    budget=BUDGET[name]; scene=trimesh.Scene(); obj_lines=[f'mtllib s1_materials.mtl','o '+name]; voff=0; n_out=0
    keep_full = n_in <= 60000          # small links (thin printed shells) collapse under decimation: keep as modelled
    for mname,(V,F) in groups.items():
        if keep_full:
            d=trimesh.Trimesh(V,F,process=False)      # untouched: trimesh's vertex merge shatters thin printed shells
        else:
            share=max(400,int(budget*len(F)/n_in)); d=simplify(V,F,share)
        n_out+=len(d.faces)
        # OBJ (positions only; loaders compute normals)
        obj_lines+= [f'usemtl {mname}'] + [f'v {x:.6f} {y:.6f} {z:.6f}' for x,y,z in d.vertices] + [f'f {a+voff+1} {b+voff+1} {c+voff+1}' for a,b,c in d.faces]
        voff+=len(d.vertices)
        # GLB with PBR
        p=pbr_for(mname,KD.get(mname,(0.5,0.5,0.5)))
        g=d.copy(); g.visual=trimesh.visual.TextureVisuals(material=PBRMaterial(name=mname,baseColorFactor=[int(255*c) for c in p['color']]+[255],metallicFactor=p['metallic'],roughnessFactor=p['roughness']))
        scene.add_geometry(g,node_name=f'{name}:{mname}',geom_name=f'{name}:{mname}')
    open(f'{OUT}/meshes/visual/{name}.obj','w').write('\n'.join(obj_lines)+'\n')
    scene.export(f'{OUT}/meshes/visual/{name}.glb')
    # collision hull from full-res
    full=trimesh.util.concatenate([trimesh.Trimesh(V,F) for V,F in groups.values()])
    h=full.convex_hull; hv,hf=fast_simplification.simplify(h.vertices.astype(np.float64),h.faces.astype(np.int64),target_reduction=max(0,1-400/len(h.faces)),agg=7)
    trimesh.Trimesh(hv,hf,process=True).convex_hull.export(f'{OUT}/meshes/collision/{name}_hull.stl')
    tot_out+=n_out
    print(f'{name:20s} {n_in:>7d} -> {n_out:>6d} faces  {"(kept)" if keep_full else ""} groups={list(groups)}')
print('total',tot_in,'->',tot_out)
shutil.copy(f'{SRC}/meshes/visual/s1_materials.mtl', f'{OUT}/meshes/visual/s1_materials.mtl')
for u in glob.glob(f'{SRC}/urdf/*.urdf'):
    shutil.copy(u, f'{OUT}/urdf/{os.path.basename(u)}')   # unchanged: still ../meshes/visual/*.obj
print('urdf copied')
