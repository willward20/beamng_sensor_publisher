import rclpy
from rclpy.node import Node

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import *

from beamng_msgs.msg import StateSensor, VehicleControl
from beamng_sensor_publisher.msg import DamageTotal



class BeamNGBridge(Node):
    def __init__(self):
        super().__init__('beamng_bridge')

        # Connect to BeamNG
        self.bng = BeamNGpy(
            'localhost', 64256,
            home='/home/arl/BeamNG',
            user='/home/arl/.local/share/BeamNG.drive'
        )
        self.bng.open()

        # Vehicle + State Sensor
        self.vehicle = Vehicle('ego', model='scintilla')
        self.vehicle.attach_sensor('state_sensor', State())
        self.vehicle.attach_sensor('electrics', Electrics())
        self.vehicle.attach_sensor('damage', Damage())
        self.vehicle.attach_sensor('gforces', GForces())
        self.vehicle.attach_sensor('timer', Timer())

        # Scenario
        scenario = Scenario('grass', 'ros2_bridge')
        scenario.add_vehicle(self.vehicle, pos=(250, 250, 0))
        scenario.make(self.bng)

        self.bng.scenario.load(scenario)
        self.bng.scenario.start()

        # Publisher
        self.state_pub = self.create_publisher(StateSensor, 'beamng/state', 10)
        self.control_pub = self.create_publisher(VehicleControl, 'beamng/vehicle_control', 10)
        self.damage_pub = self.create_publisher(DamageTotal, 'beamng/damage_total', 10)

        # Timer to poll + publish
        self.timer = self.create_timer(0.1, self.publish_state)  # 10 Hz

    def publish_state(self):

        # Poll sensors → updates vehicle.sensors
        self.vehicle.poll_sensors()
        state = self.vehicle.sensors['state_sensor']
        elec = self.vehicle.sensors['electrics']
        damage = self.vehicle.sensors['damage']
        gforces = self.vehicle.sensors['gforces']
        timer_values = self.vehicle.sensors['timer']
        import pdb; pdb.set_trace()

        # Fill State ROS message
        msg = StateSensor()
        msg.position.x, msg.position.y, msg.position.z = state['pos']
        msg.velocity.x, msg.velocity.y, msg.velocity.z = state['vel']

        # Fill control ROS message
        vc = VehicleControl()
        vc.steering = elec['steering']
        vc.throttle = elec['throttle']
        vc.brake = elec['brake']
        vc.parkingbrake = elec['parkingbrake']
        vc.clutch = elec['clutch']

        # Fill the damaga ROS message
        damage_total = DamageTotal()
        damage_total.lowpressure = damage['lowpressure']
        damage_total.damage = damage['damage']
        damage_total.part_damage = damage['part_damage']
        damage_total.damage_ext = damage['damage_ext']

        # Publish
        self.state_pub.publish(msg)
        self.control_pub.publish(vc)
        self.damage_pub.publish(damage_total)
        # self.get_logger().info(f"Published state at pos={msg.position.x:.2f},{msg.position.y:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = BeamNGBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
