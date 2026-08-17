"""Viewer meshes: FULL-resolution CAD tessellation, welded (indexed), per material group,
PBR materials, exported as GLB then Draco-compressed with gltf-pipeline.
in : s1d/s1_description/meshes/visual/*.obj (+mtl)   out: web/<link>.glb"""
import numpy as np, trimesh, os, glob, re, subprocess, shutil, sys
from trimesh.visual.material import PBRMaterial
sys.path.insert(0,'.')
SRC='s1d/s1_description/meshes/visual'; OUT='web'; shutil.rmtree(OUT,ignore_errors=True); os.makedirs(OUT)
PBR={ 'aluminum_plate':dict(color=(0.86,0.87,0.88),metallic=1.0,roughness=0.32),
      'silver_metal':  dict(color=(0.80,0.80,0.79),metallic=1.0,roughness=0.28),
      'black_oxide':   dict(color=(0.06,0.06,0.065),metallic=0.85,roughness=0.45),
      'motor_dark_gray':dict(color=(0.20,0.205,0.215),metallic=0.75,roughness=0.48),
      'camera_black':  dict(color=(0.03,0.03,0.035),metallic=0.1,roughness=0.25),
      'rubber_black':  dict(color=(0.04,0.04,0.045),metallic=0.0,roughness=0.9),
      'soft_finger_orange':dict(color=(1.0,0.31,0.02),metallic=0.0,roughness=0.75),
      'pla_cf_black':  dict(color=(0.07,0.072,0.078),metallic=0.05,roughness=0.62)}
def pbr(name): return PBR.get(name, PBR['pla_cf_black'] if ('graphite' in name or 'pla' in name) else dict(color=(0.5,0.5,0.5),metallic=0,roughness=0.6))
def parse(path):
    V=[]; groups={}; cur=None
    for line in open(path):
        if line.startswith('v '): V.append(line.split()[1:4])
        elif line.startswith('usemtl'): cur=line.split()[1]; groups.setdefault(cur,[])
        elif line.startswith('f '):
            idx=[int(t.split('/')[0])-1 for t in line.split()[1:]]
            for k in range(1,len(idx)-1): groups[cur].append((idx[0],idx[k],idx[k+1]))
    return np.array(V,dtype=np.float64), {m:np.array(F,dtype=np.int64) for m,F in groups.items()}
def creased(Vw,Fw,crease_deg=30.0):
    """Per-corner normals: average of incident face normals within the crease angle of this face
    (what three.js toCreasedNormals does), then split vertices by (position, normal)."""
    fn=np.cross(Vw[Fw[:,1]]-Vw[Fw[:,0]],Vw[Fw[:,2]]-Vw[Fw[:,0]]); l=np.linalg.norm(fn,axis=1); fn=fn/np.where(l>0,l,1)[:,None]
    nf=len(Fw); corner_v=Fw.reshape(-1); corner_f=np.repeat(np.arange(nf),3)
    order=np.argsort(corner_v,kind='stable'); sv=corner_v[order]; sf=corner_f[order]
    starts=np.searchsorted(sv,np.arange(len(Vw))); counts=np.bincount(sv,minlength=len(Vw))
    # pairs (corner i, incident face g of same vertex)
    deg=counts[corner_v]; tot=int(deg.sum())
    rep_c=np.repeat(np.arange(len(corner_v)),deg)
    off=np.arange(tot)-np.repeat(np.cumsum(deg)-deg,deg)
    g=sf[starts[corner_v[rep_c]]+off]
    cosang=np.einsum('ij,ij->i',fn[corner_f[rep_c]],fn[g]); ok=cosang>np.cos(np.radians(crease_deg))
    N=np.zeros((len(corner_v),3)); np.add.at(N,rep_c[ok],fn[g[ok]])
    l=np.linalg.norm(N,axis=1); N=N/np.where(l>0,l,1)[:,None]
    key=np.concatenate([corner_v[:,None],np.round(N*127).astype(np.int64)],axis=1)
    _,first,inv=np.unique(key,axis=0,return_index=True,return_inverse=True); inv=inv.reshape(-1)
    return Vw[corner_v[first]], N[first], inv.reshape(-1,3)

def weld(V,F,tol=1e-6):
    """index-weld by rounded position; keeps every face and its winding — no topology surgery."""
    used=np.unique(F); Vu=V[used]; remap=np.full(len(V),-1); remap[used]=np.arange(len(used)); F=remap[F]
    key=np.round(Vu/tol).astype(np.int64)
    _,first,inv=np.unique(key,axis=0,return_index=True,return_inverse=True)
    Vw=Vu[first]; Fw=inv.reshape(-1)[F] if inv.ndim>1 else inv[F]
    Fw=Fw[(Fw[:,0]!=Fw[:,1])&(Fw[:,1]!=Fw[:,2])&(Fw[:,0]!=Fw[:,2])]   # drop degenerate slivers only
    return Vw,Fw
COVER_LINKS={'link2','link3'}
def split_covers(Vw,Fw):
    """returns (body_faces, cover_faces): connected components that are thin (< 3.5 mm) and large (> 15 cm²) are covers"""
    m=trimesh.Trimesh(Vw,Fw,process=False); comps=trimesh.graph.connected_components(m.face_adjacency,nodes=np.arange(len(Fw)))
    cover=np.zeros(len(Fw),bool)
    for c in comps:
        sub=trimesh.Trimesh(Vw,Fw[c],process=False); ext=np.sort(sub.bounding_box_oriented.extents)
        if ext[0]<0.0035 and sub.area/2>0.0015: cover[c]=True
    return Fw[~cover],Fw[cover]
PBR['cover_clear']=dict(color=(0.92,0.94,0.96),metallic=0.0,roughness=0.15)
tot=0
for f in sorted(glob.glob(f'{SRC}/*.obj')):
    name=os.path.basename(f)[:-4]; V,groups=parse(f); sc=trimesh.Scene(); n=0
    if name in COVER_LINKS:
        body=[m for m in groups if 'graphite' in m][0]; Vw,Fw=weld(V,groups[body]); Fb,Fc=split_covers(Vw,Fw)
        if len(Fc): groups[body]=Fb; groups['cover_clear']=Fc; V=Vw
        print(f'  {name}: cover faces {len(Fc)} of {len(Fw)}')
    for m,F in groups.items():
        Vw,Fw=weld(V,F); n+=len(Fw)
        Vc,Nc,Fc=creased(Vw,Fw)
        g=trimesh.Trimesh(Vc,Fc,vertex_normals=Nc,process=False)
        p=pbr(m); g.visual=trimesh.visual.TextureVisuals(material=PBRMaterial(name=m,baseColorFactor=[int(255*c) for c in p['color']]+[255],metallicFactor=p['metallic'],roughnessFactor=p['roughness']))
        sc.add_geometry(g,geom_name=f'{name}:{m}')
    raw=f'{OUT}/{name}.raw.glb'; sc.export(raw)
    r=subprocess.run(['npx','--yes','gltf-pipeline','-i',raw,'-o',f'{OUT}/{name}.glb','-d','--draco.compressionLevel','7','--draco.quantizePositionBits','14','--draco.quantizeNormalBits','10'],capture_output=True,text=True)
    if r.returncode: print('DRACO FAILED',name,r.stderr[-300:]); shutil.copy(raw,f'{OUT}/{name}.glb')
    os.remove(raw); tot+=n
    print(f'{name:20s} {n:>7d} faces  verts {sum(len(np.unique(F)) for F in groups.values()):>7d}  glb {os.path.getsize(f"{OUT}/{name}.glb")//1024:>5d} KB')
print('total faces',tot,'total KB',sum(os.path.getsize(p) for p in glob.glob(f'{OUT}/*.glb'))//1024)
