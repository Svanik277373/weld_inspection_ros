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
        
        while not self.belt_cli.wait_for_service(timeout_sec=1.0):
            pass
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            pass
        while not self.state_cli.wait_for_service(timeout_sec=1.0):
            pass
            
        self.count = 0
        self.state = 'IDLE'
        self.spawn_time = 0.0
        self.current_piece = ""
        self.is_good = True
        
        # Power=20 moves at roughly 0.2 m/s. 
        self.set_belt_power(20.0)
        self.create_timer(0.1, self.tick)

    def set_belt_power(self, power):
        req = ConveyorBeltControl.Request()
        req.power = float(power)
        self.belt_cli.call_async(req)

    def set_indicator(self, color):
        green_z = 0.85 if color == 'green' else -1.0
        red_z = 0.85 if color == 'red' else -1.0
        
        for name, z in [("indicator_green", green_z), ("indicator_red", red_z)]:
            req = SetEntityState.Request()
            req.state.name = name
            req.state.pose.position.x = 0.38
            req.state.pose.position.y = 0.0  # <--- Fixed Y coordinate
            req.state.pose.position.z = z
            self.state_cli.call_async(req)

    def set_pusher(self, x_pos):
        req = SetEntityState.Request()
        req.state.name = "pneumatic_pusher"
        req.state.pose.position.x = float(x_pos)
        req.state.pose.position.y = 0.4  # <--- Fixed Y coordinate
        req.state.pose.position.z = 0.78
        self.state_cli.call_async(req)

    def reject_piece(self):
        # Force the piece into the bad bin to prevent it jamming the edge
        req = SetEntityState.Request()
        req.state.name = self.current_piece
        req.state.pose.position.x = -0.4
        req.state.pose.position.y = 0.4
        req.state.pose.position.z = 0.6 
        self.state_cli.call_async(req)

    def spawn_piece(self):
        self.current_piece = f"weld_piece_{self.count}"
        self.count += 1
        
        req = SpawnEntity.Request()
        req.name = self.current_piece
        req.reference_frame = "world"
        req.xml = """<?xml version="1.0"?><sdf version="1.6"><include><uri>model://weld_piece</uri></include></sdf>"""
            
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = -0.5
        req.initial_pose.position.z = 0.76 
        
        self.spawn_cli.call_async(req)

    def tick(self):
        now = time.time()
        elapsed = now - self.spawn_time
        
        if self.state == 'IDLE':
            self.spawn_piece()
            self.is_good = random.choice([True, False])
            self.set_indicator('none')
            self.spawn_time = now
            self.state = 'MOVING_TO_CHAMBER'
            
        elif self.state == 'MOVING_TO_CHAMBER':
            # Reaches chamber at Y=0.0 around 2.5s
            if elapsed > 2.5:
                color = 'green' if self.is_good else 'red'
                self.set_indicator(color)
                self.state = 'WAITING_FOR_SORTER'
                
        elif self.state == 'WAITING_FOR_SORTER':
            # Reaches sorter at Y=0.4 around 4.5s
            if elapsed > 4.5:
                self.state = 'SORTER_ACTION'
                
        elif self.state == 'SORTER_ACTION':
            if not self.is_good:
                # Animate the pneumatic push
                progress = (elapsed - 4.5) / 0.2
                if progress <= 1.0:
                    self.set_pusher(0.3 - (progress * 0.4))
                elif progress <= 2.0:
                    if progress > 1.5 and self.current_piece:
                        self.reject_piece()
                        self.current_piece = "" # Clear it so we only teleport once
                    self.set_pusher(-0.1 + ((progress - 1.0) * 0.4))
                else:
                    self.set_pusher(0.3)
                    self.state = 'FINISHING'
            else:
                self.state = 'FINISHING'
                
        elif self.state == 'FINISHING':
            # Reaches edge of belt and falls into green bin around 5.5s
            if elapsed > 6.5:
                self.state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = FactoryOrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()