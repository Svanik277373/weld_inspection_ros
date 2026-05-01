#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import random
from conveyorbelt_msgs.srv import ConveyorBeltControl
from gazebo_msgs.srv import SpawnEntity, SetEntityState

class FactoryOrchestratorNode(Node):
    def __init__(self):
        super().__init__("factory_orchestrator")
        
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.belt_cli = self.create_client(ConveyorBeltControl, '/CONVEYORPOWER')
        self.state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # === Wait for Gazebo services to come online before proceeding ===
        while not self.belt_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /CONVEYORPOWER service...')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        while not self.state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        self.get_logger().info('All Gazebo services linked! Starting random factory line.')
        
        self.count = 0
        self.state = 'IDLE'
        self.spawn_time = 0.0
        self.current_piece = ""
        self.is_good = True
        
        # Start main belt at power 50 (roughly 0.5 m/s)
        self.set_belt_power(50.0)
        
        # Fast tick rate for the state machine
        self.create_timer(0.1, self.tick)

    def set_belt_power(self, power):
        req = ConveyorBeltControl.Request()
        req.power = float(power)
        self.belt_cli.call_async(req)

    def set_indicator(self, color):
        # Move indicators slightly in front of the black screen or hide them underground
        green_z = 0.95 if color == 'green' else -1.0
        red_z = 0.95 if color == 'red' else -1.0
        
        for name, pose_data in [
            ("movable_indicator_green", {"color": "green", "x": 0.385, "y": 0.36, "z_show": green_z}),
            ("movable_indicator_red", {"color": "red", "x": 0.385, "y": 0.44, "z_show": red_z})
        ]:
            req = SetEntityState.Request()
            req.state.name = name
            req.state.pose.position.x = pose_data["x"]
            req.state.pose.position.y = pose_data["y"]
            req.state.pose.position.z = pose_data["z_show"]
            self.state_cli.call_async(req)

    def set_pusher_state(self, is_extending, progress=0.0):
        # Teleport the kinematic pusher ram model
        x_start = 0.25
        x_ext = 0.35 # Total distance to extend
        
        if is_extending:
            x_pos = x_start - (progress * x_ext)
        else:
            x_pos = (x_start - x_ext) + (progress * x_ext)

        req = SetEntityState.Request()
        req.state.name = "pusher_ram"
        req.state.pose.position.x = x_pos
        req.state.pose.position.y = 0.4
        req.state.pose.position.z = 0.78
        req.state.reference_frame = "world"
        self.state_cli.call_async(req)

    def spawn_piece(self):
        self.current_piece = f"weld_piece_{self.count}"
        self.count += 1
        
        req = SpawnEntity.Request()
        req.name = self.current_piece
        req.reference_frame = "world"
        req.xml = """<?xml version="1.0"?><sdf version="1.6"><include><uri>model://weld_piece</uri></include></sdf>"""
            
        # Spawn at the beginning of the 1.2m belt
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = -0.55
        req.initial_pose.position.z = 0.76 
        
        self.spawn_cli.call_async(req)
        
    def reject_piece(self):
        # Force teleport the piece into the bad bin to ensure it clears the belt
        req = SetEntityState.Request()
        req.state.name = self.current_piece
        req.state.pose.position.x = -0.4 
        req.state.pose.position.y = 0.4
        req.state.pose.position.z = 0.6 
        self.state_cli.call_async(req)

    def finalize_good_sort(self):
        self.get_logger().info(f"[{self.current_piece}] Sorting: GOOD Welding. Falling into GREEN bin.")
        
    def tick(self):
        now = time.time()
        elapsed = now - self.spawn_time
        
        if self.state == 'IDLE':
            self.spawn_piece()
            # RANDOM LOGIC IS BACK
            self.is_good = random.choice([True, False])
            
            self.set_indicator('none')
            self.set_pusher_state(False, 1.0) # Ensure retracted
            self.spawn_time = now
            self.state = 'MOVING_TO_CHAMBER'
            
        elif self.state == 'MOVING_TO_CHAMBER':
            # Reaches chamber at Y=0.0
            if elapsed > 1.1:
                status = 'PASS' if self.is_good else 'FAIL'
                self.get_logger().info(f"[{self.current_piece}] Entered chamber. Simulated Result: {status}")
                
                color = 'green' if self.is_good else 'red'
                self.set_indicator(color)
                self.state = 'MOVING_TO_SORTER'
                
        elif self.state == 'MOVING_TO_SORTER':
            # Reaches pusher at Y=0.4
            if elapsed > 1.9:
                if self.is_good:
                    self.finalize_good_sort()
                    self.state = 'FINISHING'
                else:
                    self.get_logger().warn(f"[{self.current_piece}] REJECTED! Activating pneumatic pusher.")
                    self.state = 'SORTER_PUSH'
                    
        elif self.state == 'SORTER_PUSH':
            # Pneumatic push action. 0.2s extend, 0.2s retract
            progress = (elapsed - 1.9) / 0.2
            if progress <= 1.0:
                self.set_pusher_state(True, progress)
            elif progress <= 2.0:
                if progress > 1.5 and self.current_piece:
                    self.reject_piece()
                    self.current_piece = "" # Prevent multi-teleport
                self.set_pusher_state(False, progress - 1.0)
            else:
                self.set_pusher_state(False, 1.0)
                self.state = 'FINISHING'
                
        elif self.state == 'FINISHING':
            # Good weld reaches edge at Y=0.6 and falls
            if elapsed > 3.2:
                self.state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = FactoryOrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()