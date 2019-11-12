

import * as rclnodejs from 'rclnodejs';
import { ROSUtils } from './utils';

const NODE_NAME = "TemperatureGenNode";
const INTERVAL_MS = 1000;
const TOPIC = "temperature";
const FRAME_ID = "Temperature";
const LOGGER_NAME = "TEMPGEN";
const TEMP = 100;
const TEMP_STD_DEV = 10;

export class TemperatureNode {

	private node: rclnodejs.Node;
	private publisher: rclnodejs.Publisher;
	private timer: rclnodejs.Timer;
	private clock: rclnodejs.Clock;
	private logger: rclnodejs.Logging;

	constructor() {
	}

	// create node and publisher
	async init(): Promise<void> {
		await rclnodejs.init();
		this.node = rclnodejs.createNode(NODE_NAME);

		this.logger = (rclnodejs.logging as any).getLogger(LOGGER_NAME);
		this.logger.info('Starting ' + NODE_NAME);

		this.publisher = this.node.createPublisher('sensor_msgs/msg/Temperature', TOPIC);
		this.clock = new rclnodejs.ROSClock();
	}

	// generate temperature msgs every second
	start(): void {
		if (this.timer && !this.timer.isCanceled()) {
			this.logger.error(NODE_NAME + ' already started.');
			return;
		} else {
			this.node.destroyTimer(this.timer);
		}

		this.node.createTimer(INTERVAL_MS, this.createAndPublishTemperature);
		rclnodejs.spin(this.node);
	}

	stop(): void {
		if (this.timer.isCanceled()) return;

		this.timer.cancel();
	}

	protected createAndPublishTemperature(): void {
		// create random temp
		const temp = TEMP + TEMP_STD_DEV - 2 * TEMP_STD_DEV * Math.random();

		// create ros2 temperature msg
		const msg = rclnodejs.createMessageObject('sensor_msgs/msg/LaserScan') as any;

		// init msg header
		msg.header.frame_id = FRAME_ID;
		msg.header.stamp = ROSUtils.createMsgHeaderTimestamp(this.clock.now());
		
		msg.temperature = temp;
		msg.variance = 0.0;

		// publish temperature msg
		this.publisher.publish(msg);
	}
}
