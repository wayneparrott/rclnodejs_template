
import * as rclnodejs from 'rclnodejs';

export class ROSUtils {

	static toSeconds(duration: rclnodejs.Duration) {
		return (duration as any)._nanoseconds.toNumber() / 1e9;
	}

	static DEG2RAD(degree: number) {
		return degree * Math.PI / 180.0;
	}

	static createMsgHeaderTimestamp(time: rclnodejs.Time) {
		const secondsAndNanos = time.secondsAndNanoseconds;
		return { sec: secondsAndNanos.seconds, nanosec: secondsAndNanos.nanoseconds };
	}

	static wait(timeMS: number): Promise<void> {
		return
		new Promise(resolve => {
			setTimeout(resolve, timeMS);
		}) as Promise<void>;
	}

	static block(timeMS: number): void {
		(async () => await ROSUtils.wait(timeMS))();
	}

}
