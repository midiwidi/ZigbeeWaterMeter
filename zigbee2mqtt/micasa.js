const exposes = require('zigbee-herdsman-converters/lib/exposes');
const m = require('zigbee-herdsman-converters/lib/modernExtend');
const utils = require('zigbee-herdsman-converters/lib/utils');
// const reporting = require('zigbee-herdsman-converters/lib/reporting');
// const fz = require('zigbee-herdsman-converters/converters/fromZigbee');

const e = exposes.presets;
const ea = exposes.access;

const fz_gas_meter_converter = {
	cluster: 'seMetering',
	type: ['attributeReport', 'readResponse'],
	convert: (model, msg, publish, options, meta) => {
			if (utils.hasAlreadyProcessedMessage(msg, model)) return;
			const payload = {};
			const multiplier = msg.endpoint.getClusterAttributeValue('seMetering', 'multiplier');
			const divisor = msg.endpoint.getClusterAttributeValue('seMetering', 'divisor');
			const factor = multiplier && divisor ? multiplier / divisor : null;

			if (msg.data.instantaneousDemand !== undefined) {
					let gas_flow_rate = msg.data['instantaneousDemand'];
					const property = utils.postfixWithEndpointName('gas_flow_rate', msg, model, meta);
					payload[property] = gas_flow_rate * (factor ?? 1);
			}

			if (msg.data.currentSummDelivered !== undefined) {
					const value = msg.data['currentSummDelivered'];
					const property = utils.postfixWithEndpointName('gas_volume', msg, model, meta);
					payload[property] = value * (factor ?? 1);
			}

			return payload;
	}
};

const tz_current_summ_delivered = {
	key: ['gas_volume'],
	convertGet: async (entity, key, meta) => {
			utils.assertEndpoint(entity);
			await entity.read('seMetering', ['currentSummDelivered'])
	},
	convertSet: async (entity, key, value, meta) => {
		utils.assertNumber(value, key);
		await entity.write('seMetering', {currentSummDelivered: Math.round(value * 100)});
		return {state: {gas_volume: value}};
	},
};
const tz_instantaneous_demand = {
	key: ['gas_flow_rate'],
	convertGet: async (entity, key, meta) => {
			utils.assertEndpoint(entity);
			await utils.enforceEndpoint(entity, key, meta).read('seMetering', ['instantaneousDemand']);
	},
};

function getEndpointsWithCluster(device, cluster, type) {
	if (!device.endpoints) {
			throw new Error(device.ieeeAddr + ' ' + device.endpoints);
	}
	const endpoints =
			type === 'input'
					? device.endpoints.filter((ep) => ep.getInputClusters().find((c) => (utils.isNumber(cluster) ? c.ID === cluster : c.name === cluster)))
					: device.endpoints.filter((ep) => ep.getOutputClusters().find((c) => (utils.isNumber(cluster) ? c.ID === cluster : c.name === cluster)));
	if (endpoints.length === 0) {
			throw new Error(`Device ${device.ieeeAddr} has no ${type} cluster ${cluster}`);
	}
	return endpoints;
}

function gasMeter(args) {
	args = {
			configureReporting: true,
			...args,
	};
	const divisors = new Set([
			args.gas_flow_rate?.divisor,
			args.gas_volume?.divisor,
	]);
	const multipliers = new Set([
			args.gas_flow_rate?.multiplier,
			args.gas_volume?.multiplier,
	]);
	if (multipliers.size > 1 || divisors.size > 1) {
			throw new Error(
					`When cluster is metering, gas_flow_rate and gas_volume divisor/multiplier should be equal, got divisors=${Array.from(divisors).join(', ')}, multipliers=${Array.from(multipliers).join(', ')}`,
			);
	}

	// let exposes = [e.battery(), e.battery_voltage()];
	let exposes = [];
	let fromZigbee;
	let toZigbee;

	const configureLookup = {
			seMetering: {
					// Report change with every 5W change
					gas_flow_rate: {
						attribute: 'instantaneousDemand', 
						divisor: 'divisor', 
						multiplier: 'multiplier', 
						forced: args.gas_flow_rate, 
						change: 0.005
					},
					// Report change with every 0.1kWh change
					gas_volume: {
						attribute: 'currentSummDelivered', 
						divisor: 'divisor', 
						multiplier: 'multiplier', 
						forced: args.gas_volume, 
						change: 0.1
					},
			},
	};

	if (args.gas_flow_rate !== false) exposes.push(
		e.numeric('gas_flow_rate', ea.STATE_GET).withUnit('m³/h').withDescription('Instantaneous gas flow in m³/h')
	);
	if (args.gas_volume !== false) exposes.push(
		e.numeric('gas_volume', ea.ALL).withUnit('m³').withDescription('Total gas consumption in m³')
	);
	fromZigbee = [fz_gas_meter_converter/*, fz.battery*/];
	toZigbee = [tz_current_summ_delivered, tz_instantaneous_demand];


	if (args.endpointNames) {
			exposes = flatten(exposes.map((expose) => args.endpointNames.map((endpoint) => expose.clone().withEndpoint(endpoint))));
	}

	const result = {exposes, fromZigbee, toZigbee, isModernExtend: true};

	if (args.configureReporting) {
			result.configure = [
					async (device, coordinatorEndpoint) => {
							for (const [cluster, properties] of Object.entries(configureLookup)) {
									for (const endpoint of getEndpointsWithCluster(device, cluster, 'input')) {
											const items = [];
											for (const property of Object.values(properties)) {
													let change = property.change;
													let min = '10_SECONDS';
													let max = 'MAX';

													// Check if this property has a divisor and multiplier, e.g. AC frequency doesn't.
													if ('divisor' in property) {
															// In case multiplier or divisor was provided, use that instead of reading from device.
															if (property.forced && (property.forced.divisor || property.forced.multiplier)) {
																	endpoint.saveClusterAttributeKeyValue(cluster, {
																			[property.divisor]: property.forced.divisor ?? 1,
																			[property.multiplier]: property.forced.multiplier ?? 1,
																	});
																	endpoint.save();
															} else {
																	await endpoint.read(cluster, [property.divisor, property.multiplier]);
															}

															const divisor = endpoint.getClusterAttributeValue(cluster, property.divisor);
															utils.assertNumber(divisor, property.divisor);
															const multiplier = endpoint.getClusterAttributeValue(cluster, property.multiplier);
															utils.assertNumber(multiplier, property.multiplier);
															change = property.change * (divisor / multiplier);
													}

													if ('forced' in property && property.forced) {
															if ('min' in property.forced) {
																	min = property.forced.min;
															}
															if ('max' in property.forced) {
																	max = property.forced.max;
															}
															if ('change' in property.forced) {
																	change = property.forced.change;
															}
													}

													items.push({attribute: property.attribute, min, max, change});
											}
											if (items.length) {
													await m.setupAttributes(endpoint, coordinatorEndpoint, cluster, items);
											}
									}
							}
					},
					// async (device, coordinatorEndpoint) => {
					// 	const endpoint = device.getEndpoint(1);
					// 	await reporting.bind(endpoint, coordinatorEndpoint, ['genPowerCfg']),
					// 	await reporting.batteryPercentageRemaining(endpoint)
					// }
			];
	}

	return result;
}

const definition = {
    zigbeeModel: ['GasMeter'],
    model: 'GasMeter',
    vendor: 'MICASA',
    description: 'Zigbee Gas meter created by Ignacio Hernández-Ros',
    extend: [gasMeter({"cluster":"metering"})],
    meta: {},
};

module.exports = definition;