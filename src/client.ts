import * as rclnodejs from 'rclnodejs';

const NODE_NAME = 'ros_lient';
const LOGGER_NAME = 'ros_client_logger';
const TOPIC = 'temperature';

/**
 * A ROS2 rclnodejs client that listens for Temperature messages published 
 * on the topic named 'temperature'
 */

export class RosClientNode {
	private node: rclnodejs.Node;
	private temperatureSubscription: rclnodejs.Subscription;
	private logger: rclnodejs.Logging;

	constructor() {
	}

	/**
	 * Asynchronously create and initialization the ros2 node, the rplidar-driver
	 * and service handler callbacks for start and stop the rplidar device.
	 */
	async run(): Promise<void> {

		await rclnodejs.init();
		this.node = rclnodejs.createNode(NODE_NAME);
		this.logger = (rclnodejs.logging as any).getLogger(LOGGER_NAME);
		this.logger.info('Starting ' + NODE_NAME);

		this.temperatureSubscription = this.node.createSubscription(
			'sensor_msgs/msg/Temperature', TOPIC, undefined, (msg) => {
				this.logger.info( JSON.stringify(msg));
			});

		
		this.logger.info('Waiting for lidar data');

		rclnodejs.spin(this.node);
	}
}

const client = new RosClientNode();
client.run();

