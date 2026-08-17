"""Tag clear-cover triangles in the per-link OBJs using named bodies from the STEP assembly.
Writes new OBJs (same folder, in place) with an extra `usemtl cover_clear` block."""
import numpy as np, sys, os, itertools
from scipy.spatial import cKDTree
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

STEP=sys.argv[1]; OBJDIR=sys.argv[2]
doc=TDocStd_Document(TCollection_ExtendedString("doc"))
r=STEPCAFControl_Reader(); r.SetNameMode(True); assert r.ReadFile(STEP)==IFSelect_RetDone; r.Transfer(doc)
shapes=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
def name_of(lab):
    n=TDataStd_Name(); return TCollection_ExtendedString(n.Get()).ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), n) else '?'
def tess(shape, defl=0.3):
    BRepMesh_IncrementalMesh(shape, defl, False, 0.5, True)
    pts=[]; tris=[]; ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        f=TopoDS.Face_s(ex.Current()); loc=TopLoc_Location(); tri=BRep_Tool.Triangulation_s(f,loc)
        if tri is not None:
            trsf=loc.Transformation(); base=len(pts)
            for i in range(1,tri.NbNodes()+1):
                p=tri.Node(i).Transformed(trsf); pts.append((p.X(),p.Y(),p.Z()))
            for i in range(1,tri.NbTriangles()+1):
                a,b,c=tri.Triangle(i).Get(); tris.append((base+a-1,base+b-1,base+c-1))
        ex.Next()
    return np.array(pts)/1000.0, np.array(tris)   # mm → m
# collect leaf shapes with full assembly location
found={}
def walk(lab, loc):
    nm=name_of(lab); ref=TDF_Label()
    isref=XCAFDoc_ShapeTool.IsReference_s(lab) and XCAFDoc_ShapeTool.GetReferredShape_s(lab, ref)
    lab2=ref if isref else lab
    loc2=loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(lab)) if isref else loc
    sub=TDF_LabelSequence(); XCAFDoc_ShapeTool.GetComponents_s(lab2, sub)
    if sub.Length()==0:
        shp=XCAFDoc_ShapeTool.GetShape_s(lab2).Located(loc2); found.setdefault(nm,[]).append(shp)
    else:
        for i in range(1,sub.Length()+1): walk(sub.Value(i),loc2)
labels=TDF_LabelSequence(); shapes.GetFreeShapes(labels)
for i in range(1,labels.Length()+1): walk(labels.Value(i), TopLoc_Location())
def bodies(prefix): return [shp for nm,v in found.items() if nm.split(':')[0]==prefix for shp in v]
print("bodies:", len(found))

def parse_obj(path):
    V=[]; groups={}; cur=None; lines=open(path).read().splitlines()
    for line in lines:
        if line.startswith('v '): V.append(list(map(float,line.split()[1:4])))
        elif line.startswith('usemtl'): cur=line.split()[1]; groups.setdefault(cur,[])
        elif line.startswith('f '):
            idx=[int(t.split('/')[0])-1 for t in line.split()[1:]]
            groups[cur].append(idx)
    return np.array(V), groups, lines
def sample_surface(P,T,n=60000):
    a,b,c=P[T[:,0]],P[T[:,1]],P[T[:,2]]; area=0.5*np.linalg.norm(np.cross(b-a,c-a),axis=1)
    idx=np.random.default_rng(0).choice(len(T),size=n,p=area/area.sum()); r1,r2=np.random.default_rng(1).random((2,n))
    s=np.sqrt(r1); return (1-s)[:,None]*a[idx]+(s*(1-r2))[:,None]*b[idx]+(s*r2)[:,None]*c[idx]
# 24 axis-aligned rotations
ROTS=[]
for perm in itertools.permutations(range(3)):
    for signs in itertools.product([1,-1],repeat=3):
        R=np.zeros((3,3)); R[np.arange(3),perm]=signs
        if np.linalg.det(R)>0: ROTS.append(R)

JOBS={'link2':dict(cover=['link 2_plastic cover v1'],ref=['link2_long_plate','link2_short_plate'],refmat='aluminum_plate',bodymat='link2_blue_graphite'),
      'link3':dict(cover=['link3 plastic cover'],   ref=['link3_long_plate','link3_short_plate'],refmat='aluminum_plate',bodymat='link3_warm_graphite')}
for link,job in JOBS.items():
    V,groups,lines=parse_obj(f'{OBJDIR}/{link}.obj')
    refF=np.array([f[:3] for f in groups[job['refmat']]]); ref_pts=sample_surface(V,refF,80000)
    # STEP reference (plates) and cover, tessellated
    P=[];T=[]
    for nm in job['ref']:
        for shp in bodies(nm):
            p,t=tess(shp); T.append(t+sum(len(x) for x in P)); P.append(p)
    P=np.vstack(P); T=np.vstack(T); step_ref=sample_surface(P,T,80000)
    Pc=[];Tc=[]
    for nm in job['cover']:
        for shp in bodies(nm):
            p,t=tess(shp,0.2); Tc.append(t+sum(len(x) for x in Pc)); Pc.append(p)
    Pc=np.vstack(Pc); Tc=np.vstack(Tc); step_cov=sample_surface(Pc,Tc,120000)
    tree=cKDTree(ref_pts); best=None
    def kabsch(A,B):
        ca,cb=A.mean(0),B.mean(0); H=(A-ca).T@(B-cb); U,S,Vt=np.linalg.svd(H); D=np.diag([1,1,np.sign(np.linalg.det(Vt.T@U.T))])
        R=Vt.T@D@U.T; return R, cb-R@ca
    def pca_axes(X):
        c=X.mean(0); w,v=np.linalg.eigh(np.cov((X-c).T)); return c, v[:,::-1]   # major→minor
    cs,As=pca_axes(step_ref); co,Ao=pca_axes(ref_pts)
    for sx,sy in itertools.product([1,-1],[1,-1]):
        A2=As*np.array([sx,sy,sx*sy])          # keep right-handed
        R=Ao@A2.T; R=R if np.linalg.det(R)>0 else Ao@(A2*np.array([1,1,-1])).T
        t=co-R@cs
        for _ in range(25):                    # ICP
            q=step_ref@R.T+t; d,i=tree.query(q); m=d<np.percentile(d,80)
            R2,t2=kabsch(q[m],ref_pts[i][m]); R=R2@R; t=R2@t+t2
        d,_=tree.query(step_ref@R.T+t); score=np.mean(d)
        if best is None or score<best[0]: best=(score,R,t)
    score,R,t=best
    d,_=tree.query(step_ref@R.T+t); print(f'{link}: frame fit mean {np.mean(d)*1000:.2f} mm, 95% {np.percentile(d,95)*1000:.2f} mm')
    cov=step_cov@R.T+t; ctree=cKDTree(cov)
    body=groups[job['bodymat']]; F=np.array([f[:3] for f in body]); cen=V[F].mean(1)
    dd,_=ctree.query(cen); is_cov=dd<0.0006
    print(f'{link}: cover triangles {is_cov.sum()} of {len(F)}  (cover STEP area {0.5*np.linalg.norm(np.cross(Pc[Tc[:,1]]-Pc[Tc[:,0]],Pc[Tc[:,2]]-Pc[Tc[:,0]]),axis=1).sum()*1e4:.0f} cm², tagged area {0.5*np.linalg.norm(np.cross(V[F[is_cov,1]]-V[F[is_cov,0]],V[F[is_cov,2]]-V[F[is_cov,0]]),axis=1).sum()*1e4:.0f} cm²)')
    # rewrite OBJ: faces of the body group are split into body + cover_clear; everything else passes through
    out=[]; in_body=False; k=0; body_lines=[]; cover_lines=[]
    def flush():
        nonlocal_out=out
        nonlocal_out.extend(body_lines); nonlocal_out.append('usemtl cover_clear'); nonlocal_out.extend(cover_lines)
    for line in lines:
        if line.startswith('usemtl'):
            if in_body: flush(); body_lines=[]; cover_lines=[]
            in_body = (line.split()[1]==job['bodymat'])
            out.append(line); continue
        if in_body and line.startswith('f '):
            (cover_lines if is_cov[k] else body_lines).append(line); k+=1; continue
        out.append(line)
    if in_body: flush()
    open(f'{OBJDIR}/{link}.obj','w').write('\n'.join(out)+'\n')
mtl=f'{OBJDIR}/s1_materials.mtl'
if 'cover_clear' not in open(mtl).read():
    open(mtl,'a').write('\nnewmtl cover_clear\nKd 0.90 0.93 0.95\nKa 0.3 0.3 0.3\nKs 0.5 0.5 0.5\nNs 200\nd 0.45\nillum 2\n')
print('done')
