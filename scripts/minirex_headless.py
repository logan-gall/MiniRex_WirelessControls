import pygame
import sys
import serial
import time
from enum import IntEnum
import threading
import serial.tools.list_ports
import configparser
import os

# Initialize pygame for joystick handling
pygame.init()
pygame.joystick.init()

# CRSF Protocol Constants and Functions
CRSF_SYNC_BYTE = 0xC8
CRSF_MAX_PACKET_SIZE = 64

class CRSFPacketType(IntEnum):
    GPS = 0x02
    BATTERY_SENSOR = 0x08
    LINK_STATISTICS = 0x14
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_PING = 0x28
    DEVICE_INFO = 0x29
    REQUEST_SETTINGS = 0x2A
    CHANNELS_INFO = 0x2F
    RC_CHANNELS_PACKED = 0x16

def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 0x80):
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def crsf_validate_frame(frame):
    length = frame[1]
    if length != len(frame) - 2:
        return False
    crc = crc8_dvb_s2(frame[2:-1])
    return crc == frame[-1]

def packCrsfToBytes(channels):
    # channels is in CRSF format! (0-2047)
    if len(channels) != 16:
        raise ValueError('CRSF must have 16 channels')
    result = bytearray()
    bit_buffer = 0
    bits_in_buffer = 0
    for ch in channels:
        bit_buffer |= ch << bits_in_buffer
        bits_in_buffer += 11
        while bits_in_buffer >= 8:
            result.append(bit_buffer & 0xFF)
            bit_buffer >>=8
            bits_in_buffer -=8
    if bits_in_buffer > 0:
        result.append(bit_buffer & 0xFF)
    return result

def channelsCrsfToChannelsPacket(channels):
    payload = bytearray([CRSFPacketType.RC_CHANNELS_PACKED])
    payload += packCrsfToBytes(channels)
    length = len(payload) + 1  # +1 for the type byte
    packet = bytearray([CRSF_SYNC_BYTE, length]) + payload
    crc = crc8_dvb_s2(packet[2:])
    packet.append(crc)
    return packet

def map_to_crsf(value):
    # Map 1000-2000 to 0-2047 for CRSF protocol
    return int((value - 1000) * 2047 / 1000)

def get_channels_from_joystick(joystick, axis_channel_map, button_channel_map, hat_channel_map):
    channels = [1500] * 16  # Initialize all channels to 1500

    if joystick:
        # Read joystick axes
        axis_values = {}
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            axis_values[f'axis_{i}'] = axis_value

        # Map axes to channels with inversion
        for axis_key, mapping in axis_channel_map.items():
            channel_num = mapping['channel']
            invert = mapping.get('invert', False)
            value = int(1500 + axis_values.get(axis_key, 0.0) * 500)
            if invert:
                value = 3000 - value  # Invert around 1500
            channels[channel_num - 1] = value

        # Read buttons
        button_values = {}
        for i in range(joystick.get_numbuttons()):
            button_value = joystick.get_button(i)
            button_values[f'button_{i}'] = button_value

        # Map buttons to channels with inversion
        for button_key, mapping in button_channel_map.items():
            channel_num = mapping['channel']
            invert = mapping.get('invert', False)
            value = 2000 if button_values.get(button_key, 0) else 1000
            if invert:
                value = 3000 - value  # Invert 1000 <-> 2000
            channels[channel_num - 1] = value

        # Read D-pad (hat) and split into x and y
        hat_values = {}
        for i in range(joystick.get_numhats()):
            hat_value = joystick.get_hat(i)
            hat_values[f'hat_{i}_x'] = hat_value[0]  # Left (-1), Neutral (0), Right (1)
            hat_values[f'hat_{i}_y'] = hat_value[1]  # Down (-1), Neutral (0), Up (1)

        # Map D-pad x to channels with inversion
        for hat_key_x, mapping in hat_channel_map['x'].items():
            channel_num = mapping['channel']
            invert = mapping.get('invert', False)
            value = 1500 + hat_values.get(hat_key_x, 0) * 500
            if invert:
                value = 3000 - value  # Invert around 1500
            channels[channel_num - 1] = value

        # Map D-pad y to channels with inversion
        for hat_key_y, mapping in hat_channel_map['y'].items():
            channel_num = mapping['channel']
            invert = mapping.get('invert', False)
            value = 1500 + hat_values.get(hat_key_y, 0) * 500
            if invert:
                value = 3000 - value  # Invert around 1500
            channels[channel_num - 1] = value

    # Ensure channels are within valid range
    for i in range(16):
        channels[i] = max(1000, min(2000, channels[i]))

    # Map channels to CRSF format (0-2047)
    channels = [map_to_crsf(ch) for ch in channels]

    return channels

def initialize_controller(joystick_index=None):
    joystick = None
    if joystick_index is not None and joystick_index >= 0 and joystick_index < pygame.joystick.get_count():
        joystick = pygame.joystick.Joystick(joystick_index)
        joystick.init()
    else:
        joystick = None
    return joystick

def main():
    # Default parameters
    default_joystick_index = None  # None implies no joystick selected
    default_serial_port = "Not Connected"
    default_baud_rate = 921600  # Default baud rate set to 921600
    default_axis_channel_map = {}  # e.g., {'axis_0': {'channel': 1, 'invert': False}, ...}
    default_button_channel_map = {}  # e.g., {'button_0': {'channel': 2, 'invert': False}, ...}
    default_hat_channel_map = {'x': {}, 'y': {}}  # e.g., {'x': {'hat_0_x': {'channel': 3, 'invert': False}}, 'y': {...}}

    # Load configuration from 'controller_map.txt' if it exists
    config = configparser.ConfigParser()
    config_file = 'controller_map.txt'
    if os.path.exists(config_file):
        config.read(config_file)
        # General settings
        if 'General' in config:
            general = config['General']
            joystick_index = general.getint('joystick_index', fallback=default_joystick_index)
            serial_port = general.get('serial_port', fallback=default_serial_port)
            baud_rate = general.getint('baud_rate', fallback=default_baud_rate)
        else:
            joystick_index = default_joystick_index
            serial_port = default_serial_port
            baud_rate = default_baud_rate

        # Axis mappings
        if 'AxisMappings' in config:
            for axis_key in config['AxisMappings']:
                mapping_str = config['AxisMappings'][axis_key]
                parts = [part.strip() for part in mapping_str.split(',')]
                mapping = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        key = key.strip().lower()
                        value = value.strip().lower()
                        if key == 'channel':
                            mapping['channel'] = int(value)
                        elif key == 'invert':
                            mapping['invert'] = value == 'true'
                if 'channel' in mapping:
                    default_axis_channel_map[axis_key] = mapping

        # Button mappings
        if 'ButtonMappings' in config:
            for button_key in config['ButtonMappings']:
                mapping_str = config['ButtonMappings'][button_key]
                parts = [part.strip() for part in mapping_str.split(',')]
                mapping = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        key = key.strip().lower()
                        value = value.strip().lower()
                        if key == 'channel':
                            mapping['channel'] = int(value)
                        elif key == 'invert':
                            mapping['invert'] = value == 'true'
                if 'channel' in mapping:
                    default_button_channel_map[button_key] = mapping

        # Hat mappings
        if 'HatMappings' in config:
            for hat_key in config['HatMappings']:
                mapping_str = config['HatMappings'][hat_key]
                parts = [part.strip() for part in mapping_str.split(',')]
                mapping = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        key = key.strip().lower()
                        value = value.strip().lower()
                        if key == 'channel':
                            mapping['channel'] = int(value)
                        elif key == 'invert':
                            mapping['invert'] = value == 'true'
                if 'channel' in mapping:
                    if hat_key.endswith('_x'):
                        default_hat_channel_map['x'][hat_key] = mapping
                    elif hat_key.endswith('_y'):
                        default_hat_channel_map['y'][hat_key] = mapping
    else:
        joystick_index = default_joystick_index
        serial_port = default_serial_port
        baud_rate = default_baud_rate
        axis_channel_map = default_axis_channel_map
        button_channel_map = default_button_channel_map
        hat_channel_map = default_hat_channel_map

    # Initialize joystick
    joystick = initialize_controller(joystick_index)

    # Find serial ports if not specified
    if serial_port == "Not Connected":
        serial_ports = [port.device for port in serial.tools.list_ports.comports()]
        if serial_ports:
            serial_port = serial_ports[0]
            print(f"No serial port specified. Using first available: {serial_port}")
        else:
            print("No serial ports available.")
            serial_port = None

    # Open serial port
    ser = None
    if serial_port and serial_port != "Not Connected":
        try:
            ser = serial.Serial(serial_port, baud_rate, timeout=0)
            print(f"Opened serial port: {serial_port} at {baud_rate} baud.")
        except Exception as e:
            print(f"Failed to open serial port {serial_port}: {e}")
            ser = None
    else:
        print("Serial port not connected.")

    ser_lock = threading.Lock()
    running = True

    telemetry_data = {}  # Dictionary to hold telemetry data

    # Define the mappings between axes/buttons and channels with inversion flags
    axis_channel_map = default_axis_channel_map.copy()  # e.g., {'axis_0': {'channel': 1, 'invert': False}, ...}
    button_channel_map = default_button_channel_map.copy()  # e.g., {'button_0': {'channel': 2, 'invert': False}, ...}
    hat_channel_map = {'x': default_hat_channel_map['x'].copy(), 'y': default_hat_channel_map['y'].copy()}  # e.g., {'x': {'hat_0_x': {'channel': 3, 'invert': False}}, 'y': {...}}

    def handle_crsf_packet(packet):
        type_byte = packet[2]
        payload = packet[3:-1]
        if type_byte == CRSFPacketType.LINK_STATISTICS:
            # Parse Link Statistics
            if len(payload) >= 10:
                uplink_rssi_1 = payload[0]
                uplink_rssi_2 = payload[1]
                uplink_link_quality = payload[2]
                uplink_snr = payload[3]
                active_antenna = payload[4]
                rf_mode = payload[5]
                uplink_tx_power = payload[6]
                downlink_rssi = payload[7]
                downlink_link_quality = payload[8]
                downlink_snr = payload[9]
                telemetry_data['Uplink RSSI 1'] = uplink_rssi_1
                telemetry_data['Uplink RSSI 2'] = uplink_rssi_2
                telemetry_data['Uplink LQ'] = uplink_link_quality
                telemetry_data['Uplink SNR'] = uplink_snr
                telemetry_data['Active Antenna'] = active_antenna
                telemetry_data['RF Mode'] = rf_mode
                telemetry_data['Uplink TX Power'] = uplink_tx_power
                telemetry_data['Downlink RSSI'] = downlink_rssi
                telemetry_data['Downlink LQ'] = downlink_link_quality
                telemetry_data['Downlink SNR'] = downlink_snr
        elif type_byte == CRSFPacketType.BATTERY_SENSOR:
            # Parse Battery Sensor Data
            if len(payload) >= 8:
                voltage = int.from_bytes(payload[0:2], byteorder='little') / 100.0  # in Volts
                current = int.from_bytes(payload[2:4], byteorder='little') / 100.0  # in Amps
                capacity = int.from_bytes(payload[4:6], byteorder='little')  # in mAh
                remaining = payload[6]  # in %
                telemetry_data['Voltage'] = f"{voltage:.2f} V"
                telemetry_data['Current'] = f"{current:.2f} A"
                telemetry_data['Capacity'] = f"{capacity} mAh"
                telemetry_data['Remaining'] = f"{remaining} %"
        elif type_byte == CRSFPacketType.GPS:
            # Parse GPS Data
            if len(payload) >= 15:
                latitude = int.from_bytes(payload[0:4], byteorder='little', signed=True) / 1e7
                longitude = int.from_bytes(payload[4:8], byteorder='little', signed=True) / 1e7
                ground_speed = int.from_bytes(payload[8:10], byteorder='little')  # in km/h
                heading = int.from_bytes(payload[10:12], byteorder='little') / 100.0  # in degrees
                altitude = int.from_bytes(payload[12:15], byteorder='little', signed=True) / 100.0  # in meters
                telemetry_data['Latitude'] = f"{latitude:.7f}"
                telemetry_data['Longitude'] = f"{longitude:.7f}"
                telemetry_data['Speed'] = f"{ground_speed} km/h"
                telemetry_data['Heading'] = f"{heading:.2f}°"
                telemetry_data['Altitude'] = f"{altitude:.2f} m"
        # Add handling for other telemetry packet types as needed

    def serial_read_thread_func():
        buffer = bytearray()
        while running:
            if ser and ser.is_open:
                try:
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        buffer.extend(data)
                        while len(buffer) >= 4:
                            if buffer[0] != CRSF_SYNC_BYTE:
                                buffer.pop(0)
                                continue
                            length = buffer[1]
                            if length > CRSF_MAX_PACKET_SIZE:
                                buffer.pop(0)
                                continue
                            if len(buffer) < length + 2:
                                break  # Wait for more data
                            packet = buffer[:length + 2]
                            if crsf_validate_frame(packet):
                                handle_crsf_packet(packet)
                                buffer = buffer[length + 2:]
                            else:
                                buffer.pop(0)
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    print(f"Serial read error: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def serial_write_thread_func():
        while running:
            if ser and ser.is_open:
                try:
                    channels = get_channels_from_joystick(joystick, axis_channel_map, button_channel_map, hat_channel_map)
                    packet = channelsCrsfToChannelsPacket(channels)
                    ser.write(packet)
                except Exception as e:
                    print(f"Serial write error: {e}")
                time.sleep(0.02)  # Approximately 50Hz
            else:
                time.sleep(0.1)

    # Start serial read and write threads
    ser_read_thread = threading.Thread(target=serial_read_thread_func, daemon=True)
    ser_read_thread.start()

    ser_write_thread = threading.Thread(target=serial_write_thread_func, daemon=True)
    ser_write_thread.start()

    print("Headless Controller Interface Running. Press Ctrl+C to exit.")
    try:
        while True:
            if telemetry_data:
                print("\n--- Telemetry Data ---")
                for key, value in telemetry_data.items():
                    print(f"{key}: {value}")
                print("----------------------")
                telemetry_data.clear()
            time.sleep(1)  # Update telemetry every second
    except KeyboardInterrupt:
        print("\nExiting program...")
        running = False
        if ser and ser.is_open:
            ser.close()
        pygame.joystick.quit()
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    main()
