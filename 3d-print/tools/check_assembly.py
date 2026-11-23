import trimesh, numpy as np
import sys
# 使い方: python3 check_assembly.py half_b.stl stand.stl collar.stl
#   half_b.stl = part="half_b"(塔の後ろ半分を本体座標のまま書き出したもの。印刷用の back とは向きが違う)
#   stand.stl  = part="stand"(25°)  /  collar.stl = part="collar"
# 本体座標:X=3時, Y=12時, Z=画面側, 背面=0。台は tilt=25 の変換で本体座標に戻して、点を内外判定する
hb=trimesh.load(sys.argv[1]); base=trimesh.load(sys.argv[2]); cap=trimesh.load(sys.argv[3])
for n,m in [('hb',hb),('base',base)]: print(n,m.is_watertight,m.bounds.round(2).tolist(),len(m.split()))
cap.apply_transform(np.array([[1,0,0,0],[0,0,-1,-26.0],[0,1,0,8.6],[0,0,0,1]]))
t=np.radians(25); s,c=np.sin(t),np.cos(t); R=26; uz=8.6
tt=-(c*(-67.95+R)+s*(2.4-uz))
T=np.array([[1,0,0,0],[0,s,-c,s*R+c*uz],[0,c,s,tt+c*R-s*uz],[0,0,0,1]])
Ti=np.linalg.inv(T)
toW=lambda p:(T[:3,:3]@p.T).T+T[:3,3]
def cont(m,p):
    out=np.zeros(len(p),bool)
    for i in range(0,len(p),15000): out[i:i+15000]=m.contains(p[i:i+15000])
    return out
# base in watch frame for convenience
bw=base.copy(); bw.apply_transform(Ti)
g=np.mgrid[-9:9:0.3,-66.2:-26.3:0.3,2.3:8.7:0.3].reshape(3,-1).T
inb=g[cont(hb,g)]; print('back pts',len(inb))
for d in [0,2,4,6,8,10]:
    print('slide',d,'back in base',cont(bw,inb+np.array([0,d,0])).sum())
# cable centerline (with radius probes) and boot during slide
pts=[[0,y,uz] for y in np.arange(-41,-48,-0.3)]
for a in np.linspace(0,90,30): r=np.radians(a); pts.append([0,-48.05-5*np.sin(r),uz-5+5*np.cos(r)])
pts=np.array(pts)
boot=np.mgrid[-4.95:4.95:0.4,-40.85:-26.45:0.4,6.45:10.75:0.3].reshape(3,-1).T
for d in [0,3,6,9,10]:
    off=np.array([0,d,0]); n=0
    for dx,dz in [(0,0),(1.9,0),(-1.9,0),(0,1.9),(0,-1.9)]: n+=cont(bw,pts+off+np.array([dx,0,dz])).sum()
    print('slide',d,'cable blocked',n,'boot blocked',cont(bw,boot+off).sum())
pc=np.mgrid[-9:9:0.3,-33.4:-26:0.3,2.4:14.8:0.3].reshape(3,-1).T; pc=pc[cont(cap,pc)]
print('cap pts',len(pc),'in back',cont(hb,pc).sum(),'in base',cont(bw,pc).sum())
g=np.array([[x,y,z] for x in np.arange(-5,5.01,0.5) for y in np.arange(-55.25,-50.84,0.5) for z in np.arange(2.0,8.6,0.5)])
print('exit blocked',cont(hb,g).sum())
th=np.linspace(0,2*np.pi,40); wp=[]
for r in np.arange(0,26,1.0):
    for a in th:
        for z in np.arange(-2 if r<=24.5 else 0, 15.5, 1.0): wp.append([r*np.cos(a), r*np.sin(a), z])
wp=np.array(wp); print('watch in base',cont(bw,wp).sum(),'in back',cont(hb,wp).sum(),'in cap',cont(cap,wp).sum())
