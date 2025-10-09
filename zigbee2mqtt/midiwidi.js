const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

// ------------------------ FROM ZIGBEE ------------------------
const fzLocal = {
    water_metering: {
        cluster: 'seMetering',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};

            // Total consumption
            if (msg.data.hasOwnProperty('currentSummDelivered')) {
                const data = msg.data.currentSummDelivered;
                result.water_consumption = Array.isArray(data) ? data[1] * 0x100000000 + data[0] : data;
            }

            // Instantaneous flow
            if (msg.data.hasOwnProperty('instantaneousDemand')) {
                result.water_flow = msg.data.instantaneousDemand;
            }

            return result;
        },
    },
};

// ------------------------ TO ZIGBEE ------------------------
const tzLocal = {
    water_metering: {
        key: ['water_consumption', 'water_flow'],

        convertSet: async (entity, key, value, meta) => {
            if (key === 'water_consumption') {
                const intValue = Math.floor(value);
                
                await entity.write('seMetering', {
                    currentSummDelivered: intValue
                });
                
                return {state: {water_consumption: value}};
            }
        },

        convertGet: async (entity, key, meta) => {
            if (key === 'water_consumption') {
                await entity.read('seMetering', ['currentSummDelivered']);
            } else if (key === 'water_flow') {
                await entity.read('seMetering', ['instantaneousDemand']);
            }
        },
    },
};

// ------------------------ DEVICE DEFINITION ------------------------
const definition = {
    zigbeeModel: ['WaterMeter'],
    model: 'WaterMeter',
    vendor: 'MIDIWIDI',
    description: 'Zigbee Water meter based on a Gas Meter by Ignacio Hernández-Ros',
    ota: true,

    fromZigbee: [fzLocal.water_metering, fz.battery],
    toZigbee: [tzLocal.water_metering],

    exposes: [
        e.numeric('water_consumption', ea.ALL)  // Changed to ALL to allow SET
            .withUnit('L')
            .withDescription('Total water consumption (can be set to calibrate meter)'),
        e.numeric('water_flow', ea.STATE_GET)
            .withUnit('L/h')
            .withDescription('Instantaneous water flow'),
        e.battery(),
        e.battery_voltage(),
        e.battery_low(),
    ],

    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);

        try {
            // Bind clusters
            await reporting.bind(endpoint, coordinatorEndpoint, ['seMetering', 'genPowerCfg']);

            // Enable reporting for meter
            await reporting.instantaneousDemand(endpoint);
            await reporting.currentSummDelivered(endpoint);

            // Battery reporting
            await reporting.batteryPercentageRemaining(endpoint);
            
            // Only try battery voltage if supported
            try {
                await reporting.batteryVoltage(endpoint);
            } catch (e) {
                logger.warn('Battery voltage reporting not supported');
            }
        } catch (error) {
            logger.error('Failed to configure device:', error);
        }
    },

    meta: {},
};

module.exports = definition;