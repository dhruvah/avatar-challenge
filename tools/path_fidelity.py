"""How far does the executed path stray from the shape we asked for?

Uses the live-progress path (already in the shape's 2D frame) and measures every
recorded sample against the target outline, so the answer is in the same
millimetres the user drew in.
"""
import json, subprocess, sys, time, threading
import numpy as np

BASE="http://localhost:8080"

def curl(path, data=None, t=400):
    cmd=["curl","-s","--max-time",str(t),BASE+path]
    if data is not None:
        cmd+=["-X","POST","-H","Content-Type: application/json","-d",json.dumps(data)]
    return subprocess.run(cmd,capture_output=True,text=True).stdout

def target_polyline(verts_mm, closed, n=2000):
    pts=[np.array(v,dtype=float) for v in verts_mm]
    if closed: pts=pts+[pts[0]]
    return pts

def dist_to_outline(p, poly):
    best=1e9
    for a,b in zip(poly,poly[1:]):
        ab=b-a; L=float(ab@ab)
        t=0.0 if L<1e-12 else max(0.0,min(1.0,float((p-a)@ab)/L))
        best=min(best,float(np.linalg.norm(a+t*ab-p)))
    return best

def run(name, verts_mm, verts_m, closed, pose, speed, blend_note):
    samples=[]
    stop=threading.Event()
    def poll():
        while not stop.is_set():
            try:
                j=json.loads(curl("/api/progress",t=5))
                if j.get("path"): samples.append(j["path"])
            except Exception: pass
            time.sleep(0.12)
    th=threading.Thread(target=poll,daemon=True); th.start()
    res=curl("/api/trace",{"shapes":[{"name":name,"vertices":verts_m,"closed":closed,
             "start_pose":pose,"speed":speed}]})
    stop.set(); th.join(timeout=2)
    time.sleep(0.5)
    try: final=json.loads(curl("/api/progress",t=10)).get("path",[])
    except Exception: final=[]
    if not final and samples: final=samples[-1]
    if not final:
        print(f"{name}: no path recorded ({res[:80]})"); return
    P=np.array(final,dtype=float)
    poly=target_polyline(verts_mm, closed)
    d=np.array([dist_to_outline(p,poly) for p in P])
    # ignore the pen-up descent/lift: they project onto the outline anyway,
    # but the very first/last samples can sit at the hover footprint
    print(f"{name:<16} {blend_note:<12} samples={len(P):4d}  "
          f"max={d.max():6.2f}mm  mean={d.mean():5.2f}mm  p95={np.percentile(d,95):5.2f}mm")

SQ_MM=[[0,0],[0,100],[100,100],[100,0]]
SQ_M =[[0,0],[0,0.1],[0.1,0.1],[0.1,0]]
POSE={"position":[0.3,-0.05,0.25],"rpy":[0,0,0.785]}
TRI_MM=[[0,0],[90,0],[45,80]]
TRI_M =[[0,0],[0.09,0],[0.045,0.08]]
run("square",   SQ_MM, SQ_M, True, POSE, 0.3, "blend on")
run("triangle", TRI_MM,TRI_M,True, {"position":[0.34,-0.045,0.28],"rpy":[0,0,0]}, 0.3, "blend on")
