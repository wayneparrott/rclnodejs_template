#!/usr/bin/env node

import {TemperatureNode} from './temperature_node';

async function main() {
	const node = new TemperatureNode();
	await node.init();
	node.start();
}

main();
