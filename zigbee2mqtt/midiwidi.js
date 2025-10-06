const m = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['WaterMeter'],
    model: 'WaterMeter',
    vendor: 'MIDIWIDI',
    description: 'Zigbee Water meter based on a Gas Meter by Ignacio Hernández-Ros',
		ota: true,
    extend: [/*m.identify({"isSleepy":true}),*/ m.gasMeter({"cluster":"metering"}), m.battery({voltage: true, lowStatus: true})],
    meta: {},
};

module.exports = definition;