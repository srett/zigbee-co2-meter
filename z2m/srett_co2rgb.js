const modern = require("zigbee-herdsman-converters/lib/modernExtend");
const { Zcl } = require('zigbee-herdsman');

const addCustomCluster = () =>
    modern.deviceAddCustomCluster('haCustomCo2', {
        ID: 0xffd0,
        manufacturerCode: 0x1307,
        attributes: {
            statusFlags: {ID: 0x0001, type: Zcl.DataType.UINT16},
            abcHours: {ID: 0x0002, type: Zcl.DataType.UINT16},
            co2Brightness: {ID: 0x0003, type: Zcl.DataType.UINT8, write: true},
            co2Low: {ID: 0x0004, type: Zcl.DataType.UINT16, write: true},
            co2Med: {ID: 0x0005, type: Zcl.DataType.UINT16, write: true},
            co2High: {ID: 0x0006, type: Zcl.DataType.UINT16, write: true},
        },
        commands: {},
        commandsResponse: {},
    });

const definition = {
    fingerprint: [
        {
            modelID: 'KLOJA-CO2-RGB',
            manufacturerName: 'srett',
        },
    ],
    model: 'ESP32 RGB CO2 Monitor',
    vendor: 'srett',
    description: 'Custom CO2 Monitor with RGB LED',
    extend: [
        modern.light({color: true, endpointNames: ['light']}),
        modern.co2({endpointNames: ['co2']}),
        addCustomCluster(),
        // Attributes
        modern.numeric({
            name: 'status_flags',
            cluster: 'haCustomCo2',
            attribute: 'statusFlags',
            access: 'STATE_GET',
            reporting: {
                min: 1,
                max: 3600,
                change: 1,
            },
            precision: 0,
            label: 'Status Flags',
            description: 'Status Flags of SenseAir S8',
            endpointNames: ['co2'],
            entityCategory: "diagnostic",
        }),
        modern.numeric({
            name: 'abc_hours',
            cluster: 'haCustomCo2',
            attribute: 'abcHours',
            access: 'STATE_GET',
            reporting: {
                min: 1800,
                max: 14400,
                change: 1,
            },
            precision: 0,
            unit: 'h',
            label: 'ABC Interval',
            description: 'automatic baseline correction interval',
            endpointNames: ['co2'],
            entityCategory: "diagnostic",
        }),
        modern.numeric({
            name: 'co2_display_brightness',
            cluster: 'haCustomCo2',
            attribute: 'co2Brightness',
            access: 'ALL',
            valueMin: 0,
            valueMax: 255,
            precision: 0,
            label: 'CO2 Display Brightness',
            description: 'configures the brightness of the co2 level display (0-255)',
            endpointNames: ['co2'],
        }),
        modern.numeric({
            name: 'co2_low_level',
            cluster: 'haCustomCo2',
            attribute: 'co2Low',
            access: 'ALL',
            valueMin: 400,
            valueMax: 10000,
            precision: 0,
            unit: 'ppm',
            label: 'CO2 Low Level',
            description: 'level for when the led is green (400-10000)',
            endpointNames: ['co2'],
            entityCategory: "config",
        }),
        modern.numeric({
            name: 'co2_medium_level',
            cluster: 'haCustomCo2',
            attribute: 'co2Med',
            access: 'ALL',
            valueMin: 400,
            valueMax: 10000,
            precision: 0,
            unit: 'ppm',
            label: 'CO2 Medium Level',
            description: 'level for when the led is yellow (400-10000)',
            endpointNames: ['co2'],
            entityCategory: "config",
        }),
        modern.numeric({
            name: 'co2_high_level',
            cluster: 'haCustomCo2',
            attribute: 'co2High',
            access: 'ALL',
            valueMin: 400,
            valueMax: 10000,
            precision: 0,
            unit: 'ppm',
            label: 'CO2 High Level',
            description: 'level for when the led is red (400-10000)',
            endpointNames: ['co2'],
            entityCategory: "config",
        }),
    ],
    fromZigbee: [],
    toZigbee: [],
    exposes: [],
    meta: {
        multiEndpoint: true,
    },
    endpoint: (device) => {
        return {light: 10, co2: 12};
    },
};

module.exports = definition;

