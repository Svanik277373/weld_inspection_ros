#!/usr/bin/env python3
"""
spawn_welds_node.py — Full Factory Loop version
================================================
Complete flow:
  SPAWN → Input Belt → Main Belt → Inspection → Diverter
                                              → Good Belt → Good Bin
                                              → Reject Belt → Rework → Return Belt → Input Belt

Robot arm is animated via continuous sinusoidal joint commands.

Geometry (matches weld_inspection_world.sdf):
  Input belt:   X=-6.5→-3.0  Y=0     Z=0.835
  Main belt:    X=-3.0→2.5   Y=0     Z=0.835
  Good belt:    X=2.5→5.2    Y=0     Z=0.835
  Good bin:     X=5.55       Y=0
  Reject belt:  X=2.5        Y=0→-5.5 Z=0.835
  Rework zone:  X=2.5        Y=-6.2
  Return belt:  X=2.5→-6.5   Y=-1.61  Z=0.835
  U-turn:       X=-6.5       Y=-1.61→0
"""

import glob
import json
import math
import os
import random
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

GOOD_LABEL = "Good Welding"

DATASET_DIR = os.path.expanduser(
    "~/Downloads/dataset/Weld quality inspection - Segmentation/train/images"
)

# ── Geometry ──────────────────────────────────────────────────────────────────
BELT_Z            =  0.835
SPAWN_X           = -6.2
SPAWN_Y           =  0.0
INSPECTION_ENTER  = -0.30
INSPECTION_EXIT   =  0.30
DIVERTER_X        =  1.90
GOOD_BIN_X        =  5.45
REJECT_JUNCTION_X =  2.5
REJECT_BELT_END_Y = -5.4   # hand off to rework
REWORK_Y          = -6.2
RETURN_BELT_Y     = -1.61
RETURN_END_X      = -6.3   # hand off to u-turn
UTURN_X           = -6.5
INPUT_REENTRY_Y   =  0.0

# ── Weld piece SDF ────────────────────────────────────────────────────────────
PIECE_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <inertial><mass>0.5</mass>
        <inertia><ixx>0.00042</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00042</iyy><iyz>0</iyz><izz>0.00083</izz></inertia>
      </inertial>
      <collision name="col">
        <geometry><box><size>0.20 0.15 0.03</size></box></geometry>
        <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
      </collision>
      <visual name="base">
        <geometry><box><size>0.20 0.15 0.03</size></box></geometry>
        <material>
          <ambient>0.28 0.28 0.30 1</ambient>
          <diffuse>0.30 0.30 0.32 1</diffuse>
          <specular>0.45 0.45 0.45 1</specular>
        </material>
      </visual>
      <visual name="bead">
        <pose>0 0 0.018 0 0 0</pose>
        <geometry><box><size>0.17 0.030 0.008</size></box></geometry>
        <material>
          <ambient>0.58 0.40 0.12 1</ambient>
          <diffuse>0.65 0.46 0.14 1</diffuse>
          <specular>0.30 0.22 0.05 1</specular>
        </material>
      </visual>
      <visual name="indicator">
        <pose>0 0 0.017 0 0 0</pose>
        <geometry><box><size>0.14 0.10 0.003</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <emissive>{er} {eg} {eb} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def _colour(path):
    p = path.lower()
    if "good"    in p: return (0.08, 0.82, 0.14)
    if "crack"   in p: return (0.88, 0.08, 0.08)
    if "poros"   in p: return (0.82, 0.32, 0.05)
    if "excess"  in p or "reinforc" in p: return (0.82, 0.62, 0.05)
    if "spatter" in p: return (0.55, 0.10, 0.70)
    if "bad"     in p: return (0.85, 0.08, 0.08)
    return (0.32, 0.32, 0.35)


def gz_run(cmd, timeout=5):
    try:
        subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except Exception:
        pass


def gz_spawn(name, x, y, z, image_path):
    r, g, b = _colour(image_path)
    sdf = PIECE_SDF.format(
        name=name, x=round(x,3), y=round(y,3), z=round(z,3),
        r=r, g=g, b=b, er=r*0.25, eg=g*0.25, eb=b*0.25)
    tmp = "/tmp/" + name + ".sdf"
    with open(tmp, "w") as f:
        f.write(sdf)
    cmd = ["gz", "service",
           "-s", "/world/weld_inspection_world/create",
           "--reqtype", "gz.msgs.EntityFactory",
           "--reptype", "gz.msgs.Boolean",
           "--timeout", "4000",
           "--req", 'sdf_filename: "' + tmp + '", name: "' + name + '"']
    try:
        r2 = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
        return "true" in r2.stdout.lower()
    except Exception:
        return False


def gz_set_pose(name, x, y, z):
    req = ('name: "' + name + '", position: {x: ' +
           str(round(x,3)) + ', y: ' + str(round(y,3)) +
           ', z: ' + str(round(z,3)) + '}')
    gz_run(["gz", "service",
            "-s", "/world/weld_inspection_world/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000", "--req", req], timeout=3)


def gz_remove(name):
    gz_run(["gz", "service",
            "-s", "/world/weld_inspection_world/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", 'name: "' + name + '", type: 2'], timeout=4)


class SpawnWeldsNode(Node):

    def __init__(self):
        super().__init__("spawn_welds")

        self.declare_parameter("spawn_interval", 12.0)
        self.declare_parameter("belt_speed",      0.10)

        self._interval = self.get_parameter("spawn_interval").value
        self._speed    = self.get_parameter("belt_speed").value

        self._images = self._load_images()
        self.get_logger().info("Images: " + str(len(self._images)))

        self._count         = 0
        self._pieces        = {}
        self._plock         = threading.Lock()
        self._piece_in_zone = None
        self._zlock         = threading.Lock()

        # Publishers for robot arm joints
        self._waist_pub  = self.create_publisher(Float64, "/robot_arm/waist_cmd",  10)
        self._elbow_pub  = self.create_publisher(Float64, "/robot_arm/elbow_cmd",  10)
        self._wrist_pub  = self.create_publisher(Float64, "/robot_arm/wrist_cmd",  10)

        self.create_subscription(String, "/weld_inspection/result", self._result_cb, 10)
        self.create_timer(self._interval, self._spawn_piece)
        self.create_timer(1.0, self._tick)

        # Robot arm animation timer (10 Hz)
        self._arm_t = 0.0
        self.create_timer(0.1, self._animate_arm)

        self.get_logger().info("SpawnWeldsNode ready — full loop active")
        self.get_logger().info("PASS = '" + GOOD_LABEL + "'")

    def _load_images(self):
        imgs = []
        for pat in ["*.jpg", "*.jpeg", "*.png"]:
            imgs.extend(glob.glob(os.path.join(DATASET_DIR, pat)))
        if not imgs:
            self.get_logger().warn("No images found in " + DATASET_DIR)
        return imgs

    def _pick_image(self):
        return random.choice(self._images) if self._images else ""

    # ── Robot arm animation ────────────────────────────────────────────────
    def _animate_arm(self):
        """Sinusoidal welding motion across rework table."""
        self._arm_t += 0.1

        # Waist: slow sweep left-right over the table (±60°)
        waist = 0.8 * math.sin(self._arm_t * 0.4)

        # Elbow: slight up-down mimicking weld bead motion
        elbow = -0.8 + 0.15 * math.sin(self._arm_t * 1.2)

        # Wrist: torch angle adjustment
        wrist = -0.6 + 0.10 * math.sin(self._arm_t * 0.8 + 1.0)

        w = Float64(); w.data = waist;  self._waist_pub.publish(w)
        e = Float64(); e.data = elbow;  self._elbow_pub.publish(e)
        r = Float64(); r.data = wrist;  self._wrist_pub.publish(r)

    # ── Spawn ──────────────────────────────────────────────────────────────
    def _spawn_piece(self):
        name  = "weld_piece_" + str(self._count)
        image = self._pick_image()
        self._count += 1
        self.get_logger().info("Spawning " + name)

        def _do():
            ok = gz_spawn(name, SPAWN_X, SPAWN_Y, BELT_Z, image)
            if ok:
                with self._plock:
                    self._pieces[name] = {
                        "x":       SPAWN_X,
                        "y":       SPAWN_Y,
                        "z":       BELT_Z,
                        "state":   "on_input",
                        "verdict": None,
                        "votes":   [],
                        "in_zone": False,
                    }
                self.get_logger().info("Spawned " + name)
            else:
                self.get_logger().warn("Spawn failed: " + name)

        threading.Thread(target=_do, daemon=True).start()

    # ── Main tick 1 Hz ─────────────────────────────────────────────────────
    def _tick(self):
        dt = 1.0
        to_remove = []

        with self._plock:
            names = list(self._pieces.keys())

        for name in names:
            with self._plock:
                if name not in self._pieces:
                    continue
                meta = self._pieces[name]

            state = meta["state"]
            x, y = meta["x"], meta["y"]

            # ─────────────────────────────────────────────
            # INPUT BELT
            # ─────────────────────────────────────────────
            if state == "on_input":
                nx = x + self._speed * dt

                gz_set_pose(name, nx, 0.0, BELT_Z)

                meta["x"] = nx

                if nx >= -3.0:
                    meta["state"] = "on_main"

            # ─────────────────────────────────────────────
            # MAIN BELT (WITH STOP-INSPECT LOGIC)
            # ─────────────────────────────────────────────
            elif state == "on_main":

                # 🟡 STOP during inspection
                if meta.get("state") == "inspecting":
                    continue

                nx = x + self._speed * dt
                gz_set_pose(name, nx, 0.0, BELT_Z)
                meta["x"] = nx

                # ENTER inspection → STOP
                if nx >= INSPECTION_ENTER and not meta["in_zone"]:
                    meta["in_zone"] = True
                    meta["votes"] = []
                    meta["state"] = "inspecting"

                    with self._zlock:
                        self._piece_in_zone = name

                    self.get_logger().info(name + " → STOP → INSPECT")

            # ─────────────────────────────────────────────
            # INSPECTING (WAIT FOR VOTES)
            # ─────────────────────────────────────────────
            elif state == "inspecting":

                votes = meta["votes"]

                # wait for enough votes
                if len(votes) < 3:
                    continue

                good = sum(1 for v in votes if v == GOOD_LABEL)
                total = len(votes)

                verdict = "good" if good > total/2 else "defect"

                meta["verdict"] = verdict
                meta["state"] = "decided"
                meta["in_zone"] = False

                with self._zlock:
                    if self._piece_in_zone == name:
                        self._piece_in_zone = None

                self.get_logger().info(
                    f"{name} FINAL: {verdict.upper()} ({good}/{total})"
                )

            # ─────────────────────────────────────────────
            # AFTER DECISION → MOVE AGAIN
            # ─────────────────────────────────────────────
            elif state == "decided":

                nx = x + self._speed * dt
                gz_set_pose(name, nx, 0.0, BELT_Z)
                meta["x"] = nx

                if nx > DIVERTER_X:
                    if meta["verdict"] == "good":
                        meta["state"] = "on_good"
                    else:
                        meta["state"] = "to_junction"

            # ─────────────────────────────────────────────
            # GOOD PATH
            # ─────────────────────────────────────────────
            elif state == "on_good":

                nx = x + self._speed * dt
                gz_set_pose(name, nx, 0.0, BELT_Z)
                meta["x"] = nx

                if nx >= GOOD_BIN_X:
                    self.get_logger().info(name + " → GOOD BIN ✓")
                    meta["state"] = "done"

            # ─────────────────────────────────────────────
            # REJECT PATH (UNCHANGED)
            # ─────────────────────────────────────────────
            elif state == "to_junction":

                nx = min(x + self._speed * dt, REJECT_JUNCTION_X)
                gz_set_pose(name, nx, 0.0, BELT_Z)
                meta["x"] = nx

                if nx >= REJECT_JUNCTION_X:
                    meta["state"] = "on_reject"

            elif state == "on_reject":

                ny = y - self._speed * dt
                gz_set_pose(name, REJECT_JUNCTION_X, ny, BELT_Z)
                meta["y"] = ny

                if ny <= REJECT_BELT_END_Y:
                    meta["state"] = "rework_wait"
                    meta["rework_ticks"] = 0

            elif state == "rework_wait":

                meta["rework_ticks"] += 1
                gz_set_pose(name, REJECT_JUNCTION_X, REWORK_Y, BELT_Z)

                if meta["rework_ticks"] >= 8:
                    meta["state"] = "on_return"
                    meta["x"] = REJECT_JUNCTION_X
                    meta["y"] = RETURN_BELT_Y
                    meta["votes"] = []

            elif state == "on_return":

                nx = x - self._speed * dt
                gz_set_pose(name, nx, RETURN_BELT_Y, BELT_Z)
                meta["x"] = nx

                if nx <= RETURN_END_X:
                    meta["state"] = "uturn"

            elif state == "uturn":

                ny = y + self._speed * dt
                gz_set_pose(name, UTURN_X, ny, BELT_Z)
                meta["y"] = ny

                if ny >= INPUT_REENTRY_Y:
                    meta["state"] = "on_input"

            elif state == "done":
                to_remove.append(name)

        for name in to_remove:
            gz_remove(name)
            with self._plock:
                self._pieces.pop(name, None)
    # ── Result callback ────────────────────────────────────────────────────
    def _result_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        top_label = data.get("top_label", "")
        if not top_label:
            return

        with self._zlock:
            piece = self._piece_in_zone

        if piece is None:
            return

        with self._plock:
            if piece in self._pieces and self._pieces[piece]["state"] == "inspecting":
                self._pieces[piece]["votes"].append(top_label)

                self.get_logger().info(
                    f"[{piece}] vote #{len(self._pieces[piece]['votes'])}: {top_label}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = SpawnWeldsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
