#!/usr/bin/env python3
import time, threading, json, math, numpy as np, cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
try:
    from message_filters import Subscriber as MfSub, ApproximateTimeSynchronizer
    HAS_MESSAGE_FILTERS = True
except ImportError:
    HAS_MESSAGE_FILTERS = False

STATE_IDLE='IDLE'; STATE_INIT_ARM='INIT_ARM'; STATE_FOLLOWING='FOLLOWING'; STATE_STOPPED='STOPPED'; STATE_TURNING_23='TURNING_23'; STATE_TURNING_SEARCH='TURNING_SEARCH'
OBSERVE_ARM_CMD='{#0P1550T1000!#1P1800T1000!#2P2000T1000!#3P0800T1000!#4P1500T1000!}'

class SimplePID:
    def __init__(self,kp,ki,kd,il=0.3):
        self.target=0.0; self.last_err=0.0; self.sum_err=0.0
        self.kp=kp; self.ki=ki; self.kd=kd; self.il=il
        self.hist=[]; self.win=3
    def compute(self,actual):
        err=self.target-actual
        self.hist.append(err)
        if len(self.hist)>self.win: self.hist.pop(0)
        se=np.mean(self.hist)
        kp=self.kp*(1.5 if abs(se)>0.05 else 1.0)
        self.sum_err+=se; self.sum_err=max(-self.il,min(self.il,self.sum_err))
        d=se-self.last_err; self.last_err=se
        return self.kp*se+self.ki*self.sum_err+self.kd*d
    def reset(self):
        self.last_err=0.0; self.sum_err=0.0; self.hist.clear()

class LineFollowNode(Node):
    def __init__(self):
        super().__init__('line_follow_node')
        self.state=STATE_IDLE; self.mission_active=False; self.lost_count=0
        self._turn23_start=0.0; self._turn23_dur=2.805; self._turn23_ang=-0.28; self._turn23_last=0.0; self._search_start=0.0; self._search_ang=0.20; self._search_max_dur=5.45
        self.latest_rgb=None; self.latest_depth=None
        self._frame_lock=threading.Lock()
        self.depth_ready=threading.Event(); self.rgb_ready=threading.Event()
        self.bridge=CvBridge()

        # 里程计数据
        self.odom_x=0.0; self.odom_y=0.0; self.odom_yaw=0.0
        self._odom_sub=self.create_subscription(Odometry,'/odom',self._odom_cb,10)
        self.create_timer(1.0,self._log_odom)  # 1Hz打印里程计
        self._declare_params()
        kp=self.get_parameter('pid_kp').value
        ki=self.get_parameter('pid_ki').value
        kd=self.get_parameter('pid_kd').value
        il=self.get_parameter('integral_limit').value
        self.pid=SimplePID(kp,ki,kd,il)
        self._last_steer=0.0  # 上一次转向值(死区用)
        self.pid.target=self.get_parameter('line_target_error').value
        self.is_black=(self.get_parameter('line_mode').value=='black')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST)
        if HAS_MESSAGE_FILTERS:
            rs=MfSub(self,Image,'/aurora/rgb/image_raw',qos)
            ds=MfSub(self,Image,'/aurora/depth/image_raw',qos)
            self._sync=ApproximateTimeSynchronizer([rs,ds],queue_size=5,slop=0.1)
            self._sync.registerCallback(self._synced_cb)
            self.get_logger().info('[LineFollow] message_filters OK')
        else:
            self.create_subscription(Image,'/aurora/rgb/image_raw',self._rgb_cb,qos)
            self.create_subscription(Image,'/aurora/depth/image_raw',self._depth_cb,qos)
        self.create_subscription(String,'/line_follow/control',self._ctrl_cb,10)
        self.cmd_vel_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.arm_cmd_pub=self.create_publisher(String,'/arm_command',10)
        self.status_pub=self.create_publisher(String,'/line_follow/status',10)
        self.debug_pub=self.create_publisher(Image,'/line_follow/debug_image',10)

        self.create_timer(0.1,self._loop)
        self.get_logger().info('[LineFollow] started')
        if self.get_parameter('auto_start').value:
            self.get_logger().info('[LineFollow] auto_start in 3s...')
            threading.Timer(3.0,self.start_mission).start()

    def _declare_params(self):
        for n,v in [('hsv_h_min',0),('hsv_h_max',88),('hsv_s_min',61),('hsv_s_max',230),
                    ('hsv_v_min',200),('hsv_v_max',255),('ground_depth_min_mm',150),('ground_depth_max_mm',600),
                    ('roi_y_start',176),('roi_y_end',400),('roi_x_left',44),('roi_x_right',320),
                    ('morph_kernel_size',5),('morph_open_iter',2),('morph_close_iter',2),
                    ('scan_rows',10),('min_line_pixels',5),('line_target_error',-0.6),
                    ('pid_kp',0.12),('pid_ki',0.01),('pid_kd',0.08),('max_steering',0.50),
                    ('integral_limit',0.3),('move_speed',0.15),('max_lost_frames',30),
                    ('publish_debug_image',True),('auto_start',False),('line_mode','yellow')]:
            self.declare_parameter(n,v)

    def _synced_cb(self,rm,dm):
        try:
            rgb=self.bridge.imgmsg_to_cv2(rm,'bgr8')
            depth=np.frombuffer(dm.data,dtype=np.uint16).reshape(dm.height,dm.width)
            with self._frame_lock:
                self.latest_rgb=rgb; self.latest_depth=depth
                if not self.depth_ready.is_set(): self.depth_ready.set()
                if not self.rgb_ready.is_set(): self.rgb_ready.set()
        except Exception as e: self.get_logger().error(f'sync err: {e}')

    def _rgb_cb(self,m):
        try:
            rgb=self.bridge.imgmsg_to_cv2(m,'bgr8')
            with self._frame_lock: self.latest_rgb=rgb
            if not self.rgb_ready.is_set(): self.rgb_ready.set()
        except: pass

    def _depth_cb(self,m):
        try:
            depth=np.frombuffer(m.data,dtype=np.uint16).reshape(m.height,m.width)
            with self._frame_lock: self.latest_depth=depth
            if not self.depth_ready.is_set(): self.depth_ready.set()
        except: pass

    def _ctrl_cb(self,m):
        c=m.data.strip().lower()
        if c=='start': self.start_mission()
        elif c=='stop': self.stop_mission()
        elif c=='reset': self.reset_state()

    def _odom_cb(self,msg):
        self.odom_x=msg.pose.pose.position.x
        self.odom_y=msg.pose.pose.position.y
        q=msg.pose.pose.orientation
        siny=2.0*(q.w*q.z+q.x*q.y); cosy=1.0-2.0*(q.y*q.y+q.z*q.z)
        self.odom_yaw=math.atan2(siny,cosy)*180.0/math.pi

    def _log_odom(self):
        if self.mission_active:
            self.get_logger().info(f'[Odom] X={self.odom_x:.3f}m, Y={self.odom_y:.3f}m, YAW={self.odom_yaw:.1f}deg')

    def detect_line(self,rgb,depth):
        h,w=rgb.shape[:2]
        hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
        lo=np.array([self.get_parameter('hsv_h_min').value,self.get_parameter('hsv_s_min').value,self.get_parameter('hsv_v_min').value],dtype=np.uint8)
        hi=np.array([self.get_parameter('hsv_h_max').value,self.get_parameter('hsv_s_max').value,self.get_parameter('hsv_v_max').value],dtype=np.uint8)
        mask=cv2.inRange(hsv,lo,hi)
        if depth is not None:
            gm=((depth>=self.get_parameter('ground_depth_min_mm').value)&(depth<=self.get_parameter('ground_depth_max_mm').value)).astype(np.uint8)*255
            mask=cv2.bitwise_and(mask,gm)
        rys=self.get_parameter('roi_y_start').value; rye=self.get_parameter('roi_y_end').value
        rxl=self.get_parameter('roi_x_left').value; rxr=self.get_parameter('roi_x_right').value
        rys=min(rys,h-10); rye=min(rye,h-1); rxl=max(0,min(rxl,w-10)); rxr=max(rxl+10,min(rxr,w))
        mask[:rys,:]=0; mask[rye:,:]=0; mask[:,:rxl]=0; mask[:,rxr:]=0
        ks=self.get_parameter('morph_kernel_size').value
        k=cv2.getStructuringElement(cv2.MORPH_RECT,(ks,ks))
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,k,iterations=self.get_parameter('morph_open_iter').value)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,k,iterations=self.get_parameter('morph_close_iter').value)
        nr=self.get_parameter('scan_rows').value; mp=self.get_parameter('min_line_pixels').value
        pts=[]
        for y in np.linspace(rys,rye,nr,dtype=int):
            px=np.where(mask[y,:]>0)[0]
            if len(px)>=mp: pts.append((int(np.mean(px)),y))
        icx=w/2.0; le=0.0; found=False
        if pts:
            tw=0.; we=0.
            for i,(cx,y) in enumerate(pts):
                wt=i+1; we+=(cx-icx)/icx*wt; tw+=wt
            le=we/tw if tw>0 else 0.; found=True
        lr=np.sum(mask>0)/mask.size if mask.size>0 else 0.
        return {'line_mask':mask,'line_points':pts,'lateral_error':le,'line_found':found,'line_ratio':lr,'roi':(rxl,rxr,rys,rye)}

    def _loop(self):
        if not self.mission_active: return
        with self._frame_lock: rgb=self.latest_rgb; depth=self.latest_depth
        if rgb is None: return
        r=self.detect_line(rgb,depth)
        self._pub_status(r)
        if self.get_parameter('publish_debug_image').value: self._pub_debug(rgb,r)
        if self.state==STATE_INIT_ARM: pass
        elif self.state==STATE_FOLLOWING: self._follow(r)
        elif self.state in (STATE_TURNING_23,STATE_TURNING_SEARCH): self._follow(r)

    def _follow(self,result):
        cmd=Twist()
        ms=self.get_parameter('max_steering').value
        bs=self.get_parameter('move_speed').value
        ml=self.get_parameter('max_lost_frames').value

        if self.state==STATE_TURNING_SEARCH:
            el=time.time()-self._search_start
            if result['line_found']:
                err=result['lateral_error']
                if -0.7<=err<=-0.4:
                    self.get_logger().info(f'[LineFollow] search turn found line err={err:+.3f}, resume')
                    self.state=STATE_FOLLOWING; self.pid.reset()
                    cmd.linear.x=0.0; cmd.angular.z=0.0
                    self.cmd_vel_pub.publish(cmd); return
            if el>=self._search_max_dur:
                self.get_logger().warn('[LineFollow] search turn 25deg limit, stop')
                self.stop_mission(); return
            cmd.linear.x=0.0; cmd.angular.z=self._turn23_ang
            self.cmd_vel_pub.publish(cmd)
            deg=el*self._search_ang*180/3.14159
            self.get_logger().info(f'[LineFollow] searching R... {deg:.0f}/25deg {el:.1f}s')
            return

        if self.state==STATE_TURNING_23:
            el=time.time()-self._turn23_start
            if el<self._turn23_dur:
                cmd.linear.x=0.0; cmd.angular.z=self._turn23_ang
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f'[LineFollow] turning -45deg... {el:.1f}/{self._turn23_dur:.1f}s')
                return
            else:
                self.get_logger().info('[LineFollow] -45deg done, resume')
                self.state=STATE_FOLLOWING; self.pid.reset()

        if result['line_found']:
            self.lost_count=0
            err=result['lateral_error']

            if not self.is_black and err>-0.27 and time.time()-self._turn23_last>5.0:
                self.get_logger().warn(f'[LineFollow] err={err:+.3f}<0.27 fixed -26deg L turn')
                self._turn23_start=time.time(); self._turn23_last=time.time(); self.state=STATE_TURNING_23
                cmd.linear.x=0.0; cmd.angular.z=self._turn23_ang
                self.cmd_vel_pub.publish(cmd); return

            if self.is_black and abs(err) <= 0.007:
                st=self._last_steer  # 死区内保持上次方向
            else:
                st=self.pid.compute(err); st=max(-ms,min(ms,st))
            self._last_steer=st
            cmd.linear.x=bs; cmd.angular.z=st
            if self.is_black:
                if -0.040<=err<=0.010: s='straight'
                elif err>0.010: s='R'
                else: s='L'
            else:
                if -0.65<=err<=-0.55: s='straight'
                elif err>-0.55: s='L'
                else: s='R'
            self.get_logger().info(f'[LineFollow] err={err:+.3f} tgt={self.pid.target:+.3f} st={st:+.3f} spd={bs:.2f} ln={result["line_ratio"]:.1%} [{s}]')
        else:
            self.lost_count+=1
            self.get_logger().warn(f'[LineFollow] line lost ({self.lost_count}/{ml})')
            if self.lost_count>=ml:
                self.get_logger().error('[LineFollow] lost too long, stop')
                self.stop_mission(); return
            cmd.linear.x=0.0; cmd.angular.z=0.0
        self.cmd_vel_pub.publish(cmd)

    def start_mission(self):
        if self.mission_active: return
        self.get_logger().info('[LineFollow] starting mission')
        mode="black" if self.is_black else "yellow"
        self.get_logger().info(
            f'[LineFollow] mode={mode} PID: '
            f'Kp={self.pid.kp:.3f} Ki={self.pid.ki:.3f} Kd={self.pid.kd:.3f}')
        self.mission_active=True; self.state=STATE_INIT_ARM; self.lost_count=0; self.pid.reset()
        self.cmd_vel_pub.publish(Twist())
        def arm():
            time.sleep(0.5); self._set_arm()
            self.get_logger().info('[LineFollow] waiting arm...')
            time.sleep(5.0); self._arm_ready()
        threading.Thread(target=arm,daemon=True).start()

    def _arm_ready(self):
        if not self.mission_active: return
        if not self.depth_ready.wait(timeout=10.):
            self.get_logger().error('[LineFollow] depth not ready'); self.stop_mission(); return
        if not self.rgb_ready.wait(timeout=5.):
            self.get_logger().error('[LineFollow] rgb not ready'); self.stop_mission(); return
        self.get_logger().info('[LineFollow] ready, following!')
        self.state=STATE_FOLLOWING

    def stop_mission(self):
        self.get_logger().info('[LineFollow] stopped')
        self.mission_active=False; self.state=STATE_STOPPED
        self.cmd_vel_pub.publish(Twist())

    def reset_state(self):
        self.cmd_vel_pub.publish(Twist())
        self.state=STATE_IDLE; self.mission_active=False; self.lost_count=0; self.pid.reset()

    def _set_arm(self):
        m=String(); m.data=OBSERVE_ARM_CMD; self.arm_cmd_pub.publish(m)
        self.get_logger().info(f'[LineFollow] arm: {OBSERVE_ARM_CMD}')

    def _pub_status(self,r):
        m=String()
        m.data=json.dumps({'state':self.state,'found':r['line_found'],'err':float(r['lateral_error']),'tgt':float(self.pid.target),'lr':float(r['line_ratio']),'lost':self.lost_count})
        self.status_pub.publish(m)

    def _pub_debug(self,rgb,r):
        try:
            ov=rgb.copy(); h,w=ov.shape[:2]
            yl=np.zeros_like(ov); yl[:,:,1]=255; yl[:,:,2]=255
            mb=r['line_mask']>0; bl=cv2.addWeighted(rgb,0.5,yl,0.5,0); ov[mb]=bl[mb]
            rl,rr,rs,re=r['roi']
            cv2.rectangle(ov,(rl,rs),(rr,re),(255,255,0),1)
            te=self.get_parameter('line_target_error').value
            tx=int((te+1.)*w/2.)
            cv2.line(ov,(tx,0),(tx,h),(0,255,0),2)
            txh=int((-0.5+1.)*w/2.); txl=int((-0.6+1.)*w/2.)
            cv2.line(ov,(txh,0),(txh,h),(100,255,100),1)
            cv2.line(ov,(txl,0),(txl,h),(100,255,100),1)
            for cx,y in r['line_points']: cv2.circle(ov,(cx,y),5,(0,0,255),-1)
            pts=r['line_points']
            if len(pts)>1:
                for i in range(len(pts)-1): cv2.line(ov,pts[i],pts[i+1],(0,0,255),2)
            cv2.putText(ov,f'Err:{r["lateral_error"]:+.3f}',(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
            cv2.putText(ov,f'Ln:{r["line_ratio"]:.1%}',(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(ov,encoding='bgr8'))
        except: pass

def main(args=None):
    rclpy.init(args=args); node=LineFollowNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.stop_mission(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
