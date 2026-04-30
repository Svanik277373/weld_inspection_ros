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
        
        # Service clients
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.belt_cli = self.create_client(ConveyorBeltControl, '/CONVEYORPOWER')
        self.state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # === THE FIX: Wait for Gazebo services to come online before proceeding ===
        while not self.belt_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /CONVEYORPOWER service...')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        while not self.state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        self.get_logger().info('All Gazebo services linked! Starting factory line.')
        
        self.count = 0
        self.state = 'IDLE'
        self.spawn_time = 0.0
        self.current_piece = ""
        self.is_good = True
        
        # Start main belt at 50 power 
        self.set_belt_power(50.0)
        
        # Fast tick rate for the state machine
        self.create_timer(0.1, self.tick)

    def set_belt_power(self, power):
        req = ConveyorBeltControl.Request()
        req.power = float(power)
        self.belt_cli.call_async(req)

    def set_indicator(self, color):
        # Move green and red indicators up to the panel or down underground
        green_z = 0.85 if color == 'green' else -1.0
        red_z = 0.85 if color == 'red' else -1.0
        
        for name, z in [("indicator_green", green_z), ("indicator_red", red_z)]:
            req = SetEntityState.Request()
            req.state.name = name
            req.state.pose.position.x = 0.38
            req.state.pose.position.y = 0.15
            req.state.pose.position.z = z
            self.state_cli.call_async(req)

    def reject_piece(self):
        # Simulate a pneumatic air blast pushing it into the bad bin
        req = SetEntityState.Request()
        req.state.name = self.current_piece
        req.state.pose.position.x = -0.4
        req.state.pose.position.y = 0.4
        req.state.pose.position.z = 0.76
        self.state_cli.call_async(req)

    def spawn_piece(self):
        self.current_piece = f"weld_piece_{self.count}"
        self.count += 1
        
        req = SpawnEntity.Request()
        req.name = self.current_piece
        req.reference_frame = "world"
        
        # Spawn using the model we created in models/weld_piece/
        req.xml = """<?xml version="1.0"?><sdf version="1.6"><include><uri>model://weld_piece</uri></include></sdf>"""
            
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = -0.5
        req.initial_pose.position.z = 0.76 
        
        self.spawn_cli.call_async(req)

    def tick(self):
        now = time.time()
        elapsed = now - self.spawn_time
        
        # 1. IDLE: Spawn a piece and reset everything
        if self.state == 'IDLE':
            self.spawn_piece()
            self.is_good = random.choice([True, False])
            self.set_indicator('none')
            self.spawn_time = now
            self.state = 'MOVING_TO_CHAMBER'
            
        # 2. CHAMBER: It takes ~2.5s to reach the camera (y=0) at 50 power
        elif self.state == 'MOVING_TO_CHAMBER':
            if elapsed > 2.5:
                status = 'PASS' if self.is_good else 'FAIL'
                self.get_logger().info(f"[{self.current_piece}] Entered chamber. Result: {status}")
                
                color = 'green' if self.is_good else 'red'
                self.set_indicator(color)
                self.state = 'MOVING_TO_SORTER'
                
        # 3. SORTER: It takes ~4.5s to reach the reject bin
        elif self.state == 'MOVING_TO_SORTER':
            if elapsed > 4.5:
                if not self.is_good:
                    self.get_logger().warn(f"[{self.current_piece}] REJECTED! Blasting to bad bin.")
                    self.reject_piece()
                self.state = 'FINISHING'
                
        # 4. FINISHING: It takes ~6.5s to fall off the end into the good bin
        elif self.state == 'FINISHING':
            if elapsed > 7.0:
                self.state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = FactoryOrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()