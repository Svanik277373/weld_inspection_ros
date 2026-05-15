#!/usr/bin/env python3
import asyncio
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from conveyorbelt_msgs.srv import ConveyorBeltControl
from gazebo_msgs.srv import SpawnEntity, SetEntityState

REWORK_THRESHOLD = 20

REWORK_ZONE_SDF = (
    '<?xml version="1.0"?><sdf version="1.6">'
    '<model name="rework_zone"><static>true</static>'
    '<link name="link">'
    '<visual name="v"><geometry><box><size>4 4 0.02</size></box></geometry>'
    '<material><ambient>0.8 0.4 0.0 1</ambient><diffuse>1.0 0.5 0.0 1</diffuse></material>'
    '</visual>'
    '<collision name="c"><geometry><box><size>4 4 0.02</size></box></geometry></collision>'
    '</link></model></sdf>'
)

# Home positions of world-static entities (must match the .world file)
FORKLIFT_HOME   = (-2.0,  0.4, 0.0)
REJECT_BIN_HOME = (-0.5,  0.4, 0.05)


class FactoryOrchestratorNode(Node):
    def __init__(self):
        super().__init__("factory_orchestrator")

        self.spawn_cli = self.create_client(SpawnEntity,        '/spawn_entity')
        self.belt_cli  = self.create_client(ConveyorBeltControl, '/CONVEYORPOWER')
        self.state_cli = self.create_client(SetEntityState,     '/gazebo/set_entity_state')

        while not self.belt_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /CONVEYORPOWER service...')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        while not self.state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        self.get_logger().info('All Gazebo services linked. Starting factory line.')

        self.count         = 0
        self.state         = 'IDLE'
        self.spawn_time    = 0.0
        self.current_piece = ""
        self.is_good       = True

        self.rejected_count  = 0
        self.rejected_pieces: list[str] = []

        self._inspection_result = None
        self.create_subscription(String, '/weld_inspection/result', self._inspection_cb, 10)

        self.set_belt_power(50.0)
        self._tick_timer = self.create_timer(0.1, self.tick)

    def _inspection_cb(self, msg: String):
        try:
            self._inspection_result = json.loads(msg.data)
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Fire-and-forget helpers (called from the rclpy spin thread)         #
    # ------------------------------------------------------------------ #

    def set_belt_power(self, power: float):
        req = ConveyorBeltControl.Request()
        req.power = float(power)
        self.belt_cli.call_async(req)

    def _teleport(self, name: str, x: float, y: float, z: float):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.reference_frame = "world"
        self.state_cli.call_async(req)

    def set_indicator(self, color: str):
        green_z = 0.95 if color == 'green' else -1.0
        red_z   = 0.95 if color == 'red'   else -1.0
        self._teleport("movable_indicator_green", 0.385, 0.36, green_z)
        self._teleport("movable_indicator_red",   0.385, 0.44, red_z)

    def set_pusher_state(self, is_extending: bool, progress: float = 0.0):
        x_start, x_ext = 0.25, 0.35
        x_pos = (x_start - progress * x_ext) if is_extending \
                else ((x_start - x_ext) + progress * x_ext)
        self._teleport("pusher_ram", x_pos, 0.4, 0.78)

    def spawn_piece(self):
        self.current_piece = f"weld_piece_{self.count}"
        self.count += 1
        req = SpawnEntity.Request()
        req.name            = self.current_piece
        req.reference_frame = "world"
        req.xml = (
            '<?xml version="1.0"?><sdf version="1.6">'
            '<include><uri>model://weld_piece</uri></include></sdf>'
        )
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = -0.55
        req.initial_pose.position.z = 0.76
        self.spawn_cli.call_async(req)

    def reject_piece(self):
        self._teleport(self.current_piece, -0.4, 0.4, 0.6)

    def finalize_good_sort(self):
        self.get_logger().info(
            f"[{self.current_piece}] Sorting: GOOD Welding. Falling into GREEN bin."
        )

    # ------------------------------------------------------------------ #
    # State-machine tick  (10 Hz)                                         #
    # ------------------------------------------------------------------ #

    def tick(self):
        if self.state == 'REWORK_IN_PROGRESS':
            return  # async rework thread owns control; tick is a no-op

        now     = time.time()
        elapsed = now - self.spawn_time

        if self.state == 'IDLE':
            if self.rejected_count >= REWORK_THRESHOLD:
                self.get_logger().warn(
                    f"Batch of {REWORK_THRESHOLD} rejects reached — initiating rework sequence."
                )
                self.state = 'REWORK_IN_PROGRESS'
                self._tick_timer.cancel()  # pause spawner
                threading.Thread(target=self._run_rework, daemon=True).start()
                return

            self.spawn_piece()
            self.set_indicator('none')
            self.set_pusher_state(False, 1.0)
            self.spawn_time = now
            self.state = 'MOVING_TO_CHAMBER'

        elif self.state == 'MOVING_TO_CHAMBER':
            if elapsed > 1.1:
                res = self._inspection_result
                if res is not None:
                    self.is_good = not res.get('defective', True)
                    top_label = res.get('top_label', 'none')
                    top_conf  = res.get('top_conf', 0.0)
                else:
                    self.is_good = True
                    top_label = 'none'
                    top_conf  = 0.0
                status = 'PASS' if self.is_good else 'FAIL'
                self.get_logger().info(
                    f"[{self.current_piece}] Entered chamber. "
                    f"Inspector: {top_label} ({top_conf:.2f}) → {status}"
                )
                self.set_indicator('green' if self.is_good else 'red')
                self.state = 'MOVING_TO_SORTER'

        elif self.state == 'MOVING_TO_SORTER':
            if elapsed > 1.9:
                if self.is_good:
                    self.finalize_good_sort()
                    self.state = 'FINISHING'
                else:
                    self.get_logger().warn(
                        f"[{self.current_piece}] REJECTED! Activating pneumatic pusher."
                    )
                    self.state = 'SORTER_PUSH'

        elif self.state == 'SORTER_PUSH':
            progress = (elapsed - 1.9) / 0.2
            if progress <= 1.0:
                self.set_pusher_state(True, progress)
            elif progress <= 2.0:
                if progress > 1.5 and self.current_piece:
                    self.reject_piece()
                    self.rejected_pieces.append(self.current_piece)
                    self.rejected_count += 1
                    self.get_logger().info(
                        f"[{self.current_piece}] Rejected. "
                        f"Batch: {self.rejected_count}/{REWORK_THRESHOLD}"
                    )
                    self.current_piece = ""  # prevent double-teleport
                self.set_pusher_state(False, progress - 1.0)
            else:
                self.set_pusher_state(False, 1.0)
                self.state = 'FINISHING'

        elif self.state == 'FINISHING':
            if elapsed > 3.2:
                self.state = 'IDLE'

    # ------------------------------------------------------------------ #
    # Async rework sequence (runs on its own thread + event loop)         #
    # ------------------------------------------------------------------ #

    def _run_rework(self):
        """Entry point for the rework background thread."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._rework_coroutine(loop))
        except Exception as exc:
            self.get_logger().error(f"Rework sequence failed: {exc}. Recovering.")
            self.set_belt_power(50.0)
            self.rejected_pieces.clear()
            self.rejected_count = 0
            self.state = 'IDLE'
            self._tick_timer = self.create_timer(0.1, self.tick)
        finally:
            loop.close()

    async def _await_srv(self, rclpy_future, loop: asyncio.AbstractEventLoop):
        """Bridge a rclpy Future onto this asyncio event loop without blocking."""
        aio_fut = loop.create_future()
        def _cb(f):
            try:
                loop.call_soon_threadsafe(aio_fut.set_result, f.result())
            except Exception as exc:
                loop.call_soon_threadsafe(aio_fut.set_exception, exc)
        rclpy_future.add_done_callback(_cb)
        return await aio_fut

    async def _async_teleport(
        self, name: str, x: float, y: float, z: float,
        loop: asyncio.AbstractEventLoop
    ):
        req = SetEntityState.Request()
        req.state.name            = name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.reference_frame = "world"
        await self._await_srv(self.state_cli.call_async(req), loop)

    async def _rework_coroutine(self, loop: asyncio.AbstractEventLoop):
        self.get_logger().info("=== REWORK SEQUENCE STARTED ===")

        # Step 1 — Stop the belt
        belt_req = ConveyorBeltControl.Request()
        belt_req.power = 0.0
        await self._await_srv(self.belt_cli.call_async(belt_req), loop)
        self.get_logger().info("[Rework] Belt stopped.")

        # Step 2 — Spawn the rework zone flat box at (10, 15)
        spawn_req = SpawnEntity.Request()
        spawn_req.name                    = "rework_zone"
        spawn_req.reference_frame         = "world"
        spawn_req.xml                     = REWORK_ZONE_SDF
        spawn_req.initial_pose.position.x = 10.0
        spawn_req.initial_pose.position.y = 15.0
        spawn_req.initial_pose.position.z = 0.01
        await self._await_srv(self.spawn_cli.call_async(spawn_req), loop)
        self.get_logger().info("[Rework] Rework zone spawned at (10, 15).")

        # Step 3 — Transport forklift + reject_bin to rework zone
        await self._async_teleport("forklift",   10.0, 15.0, 0.0,  loop)
        await self._async_teleport("reject_bin", 10.0, 15.0, 0.05, loop)
        self.get_logger().info("[Rework] Forklift + bin en route to rework zone…")
        await asyncio.sleep(3.0)

        # Step 4 — Drop all rejected pieces onto the rework zone (5-column grid)
        self.get_logger().info(
            f"[Rework] Placing {len(self.rejected_pieces)} pieces on rework zone."
        )
        for i, piece in enumerate(self.rejected_pieces):
            px = 9.0  + (i % 5) * 0.5   # columns: 9.0 → 11.0
            py = 14.25 + (i // 5) * 0.5  # rows:    14.25 → 15.75
            await self._async_teleport(piece, px, py, 0.1, loop)

        # Step 5 — Return forklift + empty reject_bin to home positions
        await self._async_teleport("forklift",   *FORKLIFT_HOME,   loop)
        await self._async_teleport("reject_bin", *REJECT_BIN_HOME, loop)
        self.get_logger().info("[Rework] Forklift + empty bin returning home…")
        await asyncio.sleep(3.0)

        # Step 6 — Resume production
        self.rejected_pieces.clear()
        self.rejected_count = 0

        belt_req.power = 50.0
        await self._await_srv(self.belt_cli.call_async(belt_req), loop)
        self.get_logger().info("[Rework] Belt restarted.")

        self.state = 'IDLE'
        self._tick_timer = self.create_timer(0.1, self.tick)
        self.get_logger().info("=== REWORK SEQUENCE COMPLETE — production resumed ===")


def main(args=None):
    rclpy.init(args=args)
    node = FactoryOrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
