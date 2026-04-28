#!/usr/bin/env python3
"""
OAK-D: Depth + AprilTag детекция
"""
import matplotlib
matplotlib.use('TkAgg')

import cv2
import numpy as np
import depthai as dai
from pyapriltags import Detector
import math
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

TAG_SIZE = 0.162
TAG_FAMILY = 'tag36h11'

CAMERA_PARAMS = {
    'fx': 634.37, 'fy': 633.00,
    'cx': 280.28, 'cy': 230.60,
    'baseline': 0.075
}

class MPL3DVisualizer:
    def __init__(self):
        self.tag_positions = {}
        self.lock = threading.Lock()
        
        plt.ion()
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.ax.set_xlabel('X (m) - right/left')
        self.ax.set_ylabel('Y (m) - up/down')
        self.ax.set_zlabel('Z (m) - depth')
        self.ax.set_title('3D TF: Camera -> Tag (Drag mouse to rotate)')
        self.ax.grid(True, alpha=0.3)
        
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_zlim(0, 3)
        
        self.ax.scatter(0, 0, 0, c='white', s=150, marker='o', edgecolors='black', linewidth=2)
        self.ax.quiver(0, 0, 0, 0.5, 0, 0, color='red', arrow_length_ratio=0.2, linewidth=3)
        self.ax.quiver(0, 0, 0, 0, 0.5, 0, color='green', arrow_length_ratio=0.2, linewidth=3)
        self.ax.quiver(0, 0, 0, 0, 0, 0.5, color='blue', arrow_length_ratio=0.2, linewidth=3)
        
        self.ax.text(0.55, 0, 0, 'X', color='red', fontsize=12)
        self.ax.text(0, 0.55, 0, 'Y', color='green', fontsize=12)
        self.ax.text(0, 0, 0.55, 'Z', color='blue', fontsize=12)
        self.ax.text(0.05, 0.05, 0.05, 'CAM', color='white', fontsize=10)
        
        self.text_info = self.ax.text2D(0.02, 0.95, "", transform=self.ax.transAxes, fontsize=10,
                                         bbox=dict(boxstyle="round", facecolor="black", alpha=0.7))
        
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
        
        plt.show(block=False)
    
    def update_tag(self, tag_id, position):
        with self.lock:
            self.tag_positions[tag_id] = position
    
    def _update_loop(self):
        while self.running:
            try:
                with self.lock:
                    for artist in self.ax.collections + self.ax.lines + self.ax.texts:
                        if hasattr(artist, '_gid') and artist._gid == 'tag':
                            try:
                                artist.remove()
                            except:
                                pass
                    
                    for tag_id, pos in self.tag_positions.items():
                        X, Y, Z = pos
                        sc = self.ax.scatter([X], [Y], [Z], c='yellow', s=200, marker='s',
                                              edgecolors='orange', linewidth=2, zorder=10)
                        sc._gid = 'tag'
                        line, = self.ax.plot([0, X], [0, Y], [0, Z], 'cyan', linewidth=2, alpha=0.7)
                        line._gid = 'tag'
                        txt = self.ax.text(X, Y, Z + 0.1, f'Tag {tag_id}', color='yellow', fontsize=10,
                                            ha='center', va='bottom', weight='bold')
                        txt._gid = 'tag'
                        
                        dist = math.sqrt(X**2 + Y**2 + Z**2)
                        self.text_info.set_text(f"Tag {tag_id}: X={X:.3f}m Y={Y:.3f}m Z={Z:.3f}m\nDistance = {dist:.3f}m")
                    
                    if self.tag_positions:
                        max_coord = max(max(abs(p[0]), abs(p[1]), p[2]) for p in self.tag_positions.values())
                        limit = max(1.5, max_coord + 0.5)
                        self.ax.set_xlim(-limit, limit)
                        self.ax.set_ylim(-limit, limit)
                        self.ax.set_zlim(0, limit)
                
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
                time.sleep(0.1)
            except:
                time.sleep(0.1)
    
    def destroy(self):
        self.running = False
        plt.close(self.fig)


class DepthAprilTagDetector:
    def __init__(self):
        self.detector = Detector(families=TAG_FAMILY, nthreads=4)
        self.viz = MPL3DVisualizer()
        self.setup_pipeline()
    
    def setup_pipeline(self):
        self.pipeline = dai.Pipeline()
        
        self.cam_rgb = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        self.left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.stereo.setRectification(True)
        self.stereo.setExtendedDisparity(True)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        self.stereo.setOutputSize(640, 480)
        
        self.rgbOut = self.cam_rgb.requestOutput(size=(640, 480), fps=30)
        self.leftOut = self.left.requestOutput(size=(640, 400), fps=30)
        self.rightOut = self.right.requestOutput(size=(640, 400), fps=30)
        
        self.leftOut.link(self.stereo.left)
        self.rightOut.link(self.stereo.right)
        
        self.rgbQueue = self.rgbOut.createOutputQueue()
        self.depthQueue = self.stereo.depth.createOutputQueue()
        self.disparityQueue = self.stereo.disparity.createOutputQueue()
    
    def calculate_3d_position(self, center, depth_m):
        if depth_m <= 0:
            return None
        u, v = center
        X = (u - CAMERA_PARAMS['cx']) * depth_m / CAMERA_PARAMS['fx']
        Y = (v - CAMERA_PARAMS['cy']) * depth_m / CAMERA_PARAMS['fy']
        Z = depth_m
        return (X, Y, Z)
    
    def run(self):
        print("🔌 Connecting to OAK-D...")
        print("🎮 Controls: Left mouse drag = rotate, Right mouse drag = zoom, 'q' = quit")
        
        with self.pipeline:
            self.pipeline.start()
            print("✅ Camera ready!")
            
            while self.pipeline.isRunning():
                in_rgb = self.rgbQueue.tryGet()
                in_depth = self.depthQueue.tryGet()
                in_disparity = self.disparityQueue.tryGet()
                
                if in_rgb is not None and in_depth is not None:
                    frame = in_rgb.getCvFrame()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    tags = self.detector.detect(gray)
                    
                    depth_frame = in_depth.getFrame()
                    h, w = depth_frame.shape
                    
                    for tag in tags:
                        corners = tag.corners.astype(int)
                        center = tag.center.astype(int)
                        
                        for i in range(4):
                            cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
                        cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"ID: {tag.tag_id}", 
                                   (center[0] - 20, center[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        
                        xd = int(center[0] * w / frame.shape[1])
                        yd = int(center[1] * h / frame.shape[0])
                        
                        depth_m = 0
                        if 0 <= xd < w and 0 <= yd < h:
                            depth_mm = depth_frame[yd, xd]
                            if depth_mm > 0:
                                depth_m = depth_mm / 1000.0
                        
                        if depth_m > 0:
                            pos = self.calculate_3d_position(center, depth_m)
                            if pos:
                                X, Y, Z = pos
                                cv2.putText(frame, f"Z: {Z:.2f}m", (center[0] - 20, center[1] + 20),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                                self.viz.update_tag(tag.tag_id, [X, Y, Z])
                                print(f"🔍 Tag {tag.tag_id}: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} m")
                        else:
                            cv2.putText(frame, "Z: N/A", (center[0] - 20, center[1] + 20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    cv2.putText(frame, f"Tags: {len(tags)}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.imshow("AprilTag Detection", frame)
                
                if in_disparity is not None:
                    disp_frame = in_disparity.getFrame()
                    max_disp = np.max(disp_frame)
                    if max_disp > 0:
                        depth_vis = (disp_frame / max_disp * 255).astype(np.uint8)
                    else:
                        depth_vis = disp_frame.astype(np.uint8)
                    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    cv2.imshow("Depth Map", depth_color)
                
                if cv2.waitKey(1) == ord('q'):
                    self.pipeline.stop()
                    break
            
            cv2.destroyAllWindows()
            self.viz.destroy()
            plt.show(block=True)

if __name__ == "__main__":
    detector = DepthAprilTagDetector()
    detector.run()