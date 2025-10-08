import rclpy
from rclpy.node import Node

from beamngpy import BeamNGpy, Scenario, Vehicle

from beamng_ros2.publishers.sensors import SensorPublisher



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

        # Scenario
        scene = 'east_coast_usa'
        scenario = Scenario(scene, 'ros2_bridge')
        if scene == 'east_coast_usa':
            scenario.add_vehicle(self.vehicle, pos=(-605, -353, 34))
        else:
            scenario.add_vehicle(self.vehicle, pos=(250, 250, 0))
        scenario.make(self.bng)

        self.bng.scenario.load(scenario)
        self.bng.scenario.start()

        # Create the StatePublisher
        self.state_pub = SensorPublisher.create('state', 'state', {})
        self.state_pub.pre_scenario_start(self.vehicle)
        self.state_pub.create_publisher(self)

        # Create the damage publisher
        self.damage_pub = SensorPublisher.create('damage', 'damage', {})
        self.damage_pub.pre_scenario_start(self.vehicle)
        self.damage_pub.create_publisher(self)

        # Create the electrics publisher
        self.electrics_pub = SensorPublisher.create('electrics', 'electrics', {})
        self.electrics_pub.pre_scenario_start(self.vehicle)
        self.electrics_pub.create_publisher(self)

        # Create timer to periodically publish
        self.timer = self.create_timer(0.05, self.publish_data)

        
    def publish_data(self):
        # Poll BeamNG for new sensor data
        self.vehicle.sensors.poll()

        # # Publish using built-in publisher
        # import pdb; pdb.set_trace()
        self.state_pub.publish(self.get_clock().now())
        self.damage_pub.publish(self.get_clock().now())
        self.electrics_pub.publish(self.get_clock().now())


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
