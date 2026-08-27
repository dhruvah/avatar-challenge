"""Record tool height through a two-shape run and check the pen lifts between them."""
import json, subprocess, threading, time
import numpy as np, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

PLANE_Z = 0.26
LIFT    = 0.03

class Rec(Node):
    def __init__(self):
        super().__init__("lift_check")
        self.b=Buffer(); self.l=TransformListener(self.b,self)
        self.rows=[]
        self.create_timer(0.02, self.tick)
    def tick(self):
        try: t=self.b.lookup_transform("world","link_eef",rclpy.time.Time())
        except Exception: return
        tr=t.transform.translation
        self.rows.append((time.time(), tr.x, tr.y, tr.z))

def curl(p, data=None, t=400):
    cmd=["curl","-s","--max-time",str(t),"http://localhost:8080"+p]
    if data is not None:
        cmd+=["-X","POST","-H","Content-Type: application/json","-d",json.dumps(data)]
    return subprocess.run(cmd,capture_output=True,text=True).stdout

PAYLOAD={"shapes":[
 {"name":"rect_left","vertices":[[0,0],[0.08,0],[0.08,0.05],[0,0.05]],"closed":True,
  "start_pose":{"position":[0.30,-0.10,PLANE_Z],"rpy":[0,0,0]},"speed":0.5},
 {"name":"quad_right","vertices":[[0,0],[0.07,0],[0.09,0.06],[0.01,0.05]],"closed":True,
  "start_pose":{"position":[0.30,0.03,PLANE_Z],"rpy":[0,0,0]},"speed":0.5}]}

rclpy.init(); r=Rec()
res={}
def go(): res["out"]=curl("/api/trace", PAYLOAD)
th=threading.Thread(target=go); th.start()
while th.is_alive(): rclpy.spin_once(r, timeout_sec=0.05)
for _ in range(40): rclpy.spin_once(r, timeout_sec=0.05)
print(res.get("out","").strip())

A=np.array([[x,y,z] for _,x,y,z in r.rows])
z=A[:,2]; y=A[:,1]
print(f"\nsamples: {len(A)}")
print(f"tool height: min {z.min():.4f}  max {z.max():.4f}  (plane at {PLANE_Z}, hover {PLANE_Z+LIFT})")

# the transit between shapes is where y crosses from the left shape to the right
on_plane = np.abs(z - PLANE_Z) < 0.002
left  = y < -0.045
right = y > -0.02
crossing = (~left) & (~right)          # the gap between the two shapes
if crossing.any():
    zc = z[crossing]
    print(f"\nwhile crossing the gap between the shapes:")
    print(f"  height min {zc.min():.4f}  max {zc.max():.4f}")
    print(f"  samples drawn at plane height while crossing: {int(on_plane[crossing].sum())}")
    ok = zc.min() > PLANE_Z + 0.005
    print(f"\n{'PASS' if ok else 'FAIL'}: pen {'stayed lifted' if ok else 'DRAGGED'} between shapes")
else:
    print("no crossing samples captured")
rclpy.shutdown()
