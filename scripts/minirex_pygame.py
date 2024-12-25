import pygame
import sys
import serial
import time
from enum import IntEnum
import threading
import serial.tools.list_ports
import configparser
import os

# Initialize pygame
pygame.init()

# Set up colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PINK = (255, 105, 180)
GREEN = (0, 255, 0)
GRAY = (200, 200, 200)
LIGHT_BLUE = (173, 216, 230)
DARK_GRAY = (100, 100, 100)
YELLOW = (255, 255, 0)

# Set up fonts
font = pygame.font.Font(None, 24)

# Set up display
WIDTH, HEIGHT = 1200, 850
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Controller Input Display")

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

def map_axis(value):
    # Map axis value from -1.0..1.0 to 1000..2000
    return int(1500 + value * 500)

def map_button(value):
    # Map button value to 1000 or 2000
    return 2000 if value else 1000

def map_to_crsf(value):
    # Map 1000-2000 to 0-2047 for CRSF protocol
    return int((value - 1000) * 2047 / 1000)

def get_channels_from_joystick(joystick, axis_channel_map, button_channel_map, hat_channel_map):
    # Update joystick events
    pygame.event.pump()

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
            invert = mapping['invert']
            value = map_axis(axis_values.get(axis_key, 0.0))
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
            invert = mapping['invert']
            value = map_button(button_values.get(button_key, 0))
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
            invert = mapping['invert']
            value = 1500 + hat_values.get(hat_key_x, 0) * 500
            if invert:
                value = 3000 - value  # Invert around 1500
            channels[channel_num - 1] = value

        # Map D-pad y to channels with inversion
        for hat_key_y, mapping in hat_channel_map['y'].items():
            channel_num = mapping['channel']
            invert = mapping['invert']
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

# Function to initialize joystick and create input boxes
def initialize_controller(joystick_index=None):
    joystick = None
    if joystick_index is not None and joystick_index >= 0 and joystick_index < pygame.joystick.get_count():
        joystick = pygame.joystick.Joystick(joystick_index)
        joystick.init()
    else:
        joystick = None
    # Define box dimensions
    BOX_WIDTH = 240
    BOX_HEIGHT = 60  # Increased from 50
    BOX_SPACING = 70  # Increased from 60

    # Modify max_boxes_per_column to 8 as per requirement
    max_boxes_per_column = 8  # Changed from 10 to 8

    # Create input boxes dynamically based on controller capabilities
    boxes = {}
    column_width = 250  # Width for each column
    axis_y_offset = 50  # Initial y-offset for axis boxes
    button_y_offset = 50  # Initial y-offset for button boxes
    x_offset = 50  # Initial x-offset for all columns
    current_axis_column = 0
    current_button_column = 0

    if joystick:
        # Create boxes for axes
        for i in range(joystick.get_numaxes()):
            if i % max_boxes_per_column == 0 and i != 0:
                current_axis_column += 1
                axis_y_offset = 50
            # Create a box for each axis input
            boxes[f'axis_{i}'] = pygame.Rect(x_offset + current_axis_column * column_width, axis_y_offset, BOX_WIDTH, BOX_HEIGHT)
            axis_y_offset += BOX_SPACING

        # Create boxes for hat switches, split into x and y
        for i in range(joystick.get_numhats()):
            # Create box for hat X-axis
            if (joystick.get_numaxes() + i * 2) % max_boxes_per_column == 0 and i != 0:
                current_axis_column += 1
                axis_y_offset = 50
            boxes[f'hat_{i}_x'] = pygame.Rect(x_offset + current_axis_column * column_width, axis_y_offset, BOX_WIDTH, BOX_HEIGHT)
            axis_y_offset += BOX_SPACING

            # Create box for hat Y-axis
            if (joystick.get_numaxes() + i * 2 + 1) % max_boxes_per_column == 0 and i != 0:
                current_axis_column += 1
                axis_y_offset = 50
            boxes[f'hat_{i}_y'] = pygame.Rect(x_offset + current_axis_column * column_width, axis_y_offset, BOX_WIDTH, BOX_HEIGHT)
            axis_y_offset += BOX_SPACING

        # Create boxes for buttons (split into multiple columns if needed)
        for i in range(joystick.get_numbuttons()):
            if i % max_boxes_per_column == 0 and i != 0:
                current_button_column += 1
                button_y_offset = 50
            # Create a box for each button input
            boxes[f'button_{i}'] = pygame.Rect(
                x_offset + (current_axis_column + 1 + current_button_column) * column_width,
                button_y_offset,
                BOX_WIDTH,
                BOX_HEIGHT
            )
            button_y_offset += BOX_SPACING

    # Create smaller boxes for channels 1-16, split into 2 rows, at the bottom of the screen
    channel_boxes = {}
    channel_box_width = 120  # Width for each channel box (increased to fit text)
    channel_box_height = 80  # Height for each channel box (increased to fit three lines)
    channel_x_offset = 10  # Initial x-offset for channel boxes
    channel_y_offset_top = HEIGHT - (2 * channel_box_height) - 30  # Position for the first row
    channel_y_offset_bottom = HEIGHT - channel_box_height - 20  # Position for the second row

    for i in range(8):
        # Create the first row of channel boxes (Channel 1-8)
        channel_boxes[f'channel_{i + 1}'] = pygame.Rect(
            channel_x_offset + i * (channel_box_width + 10),
            channel_y_offset_top,
            channel_box_width,
            channel_box_height
        )
    for i in range(8, 16):
        # Create the second row of channel boxes (Channel 9-16)
        channel_boxes[f'channel_{i + 1}'] = pygame.Rect(
            channel_x_offset + (i - 8) * (channel_box_width + 10),
            channel_y_offset_bottom,
            channel_box_width,
            channel_box_height
        )

    return joystick, boxes, channel_boxes

# Function to check if an input is active
def is_input_active(input_key, input_status):
    if input_key.startswith('axis_') or (input_key.startswith('hat_') and ('_x' in input_key or '_y' in input_key)):
        value = input_status.get(input_key, 0.0)
        return abs(value) > 0.1  # Consider axis active if value is beyond a threshold
    elif input_key.startswith('button_'):
        value = input_status.get(input_key, False)
        return value
    return False

# Function to draw the input boxes based on the current input status
def draw_input_boxes(input_status, boxes, axis_channel_map, button_channel_map, hat_channel_map, selected_input=None):
    for key, rect in boxes.items():
        # Highlight the selected input
        if key == selected_input:
            pygame.draw.rect(screen, YELLOW, rect)
        else:
            pygame.draw.rect(screen, WHITE, rect)

        # Calculate positions for "Invert" label and checkbox
        invert_label_pos = (rect.x + rect.width - 80, rect.y + 5)  # Positioned on the right side
        invert_checkbox_pos = (rect.x + rect.width - 80, rect.y + 25)  # Below the label

        # Handle axis inputs
        if key.startswith('axis_'):
            axis_value = input_status.get(key, 0.0)
            fill_width = int((axis_value + 1) / 2 * rect.width)  # Normalize axis value (-1 to 1) to box width (0 to rect.width)
            pygame.draw.rect(screen, PINK, (rect.x, rect.y, fill_width, rect.height))
            # Get the channel mapping
            mapping = axis_channel_map.get(key, {})
            channel_mapped = mapping.get('channel')
            invert = mapping.get('invert', False)
            if channel_mapped:
                label = f"Axis {key.split('_')[-1]}\n(Channel {channel_mapped})\nVALUE: {axis_value:.2f}"
            else:
                label = f"Axis {key.split('_')[-1]}\n(Channel None)\nVALUE: {axis_value:.2f}"
            pygame.draw.rect(screen, BLACK, rect, 2)  # Draw the border of the box
            for idx, line in enumerate(label.split('\n')):
                screen.blit(font.render(line, True, BLACK), (rect.x + 10, rect.y + 5 + (idx * 15)))

            # Draw "Invert" label
            invert_label = font.render("Invert:", True, BLACK)
            screen.blit(invert_label, invert_label_pos)

            # Draw checkbox
            checkbox_size = 20
            checkbox_rect = pygame.Rect(invert_checkbox_pos[0], invert_checkbox_pos[1], checkbox_size, checkbox_size)
            pygame.draw.rect(screen, BLACK, checkbox_rect, 2)  # Border
            if invert:
                pygame.draw.line(screen, BLACK, checkbox_rect.topleft, checkbox_rect.bottomright, 2)
                pygame.draw.line(screen, BLACK, checkbox_rect.topright, checkbox_rect.bottomleft, 2)
            # No need to store checkbox_rects here as event handling is managed elsewhere

        # Handle hat X-axis
        elif key.startswith('hat_') and key.endswith('_x'):
            hat_value = input_status.get(key, 0.0)
            direction = "Right" if hat_value > 0 else ("Left" if hat_value < 0 else "Neutral")
            color = PINK if hat_value != 0.0 else WHITE
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, BLACK, rect, 2)  # Draw the border of the box
            # Get the channel mapping
            mapping = hat_channel_map['x'].get(key, {})
            channel_mapped = mapping.get('channel')
            invert = mapping.get('invert', False)
            if channel_mapped:
                label = f"HAT X\n(Channel {channel_mapped})\nDirection: {direction}"
            else:
                label = f"HAT X\n(Channel None)\nDirection: {direction}"
            for idx, line in enumerate(label.split('\n')):
                screen.blit(font.render(line, True, BLACK), (rect.x + 10, rect.y + 5 + (idx * 15)))

            # Draw "Invert" label
            invert_label = font.render("Invert:", True, BLACK)
            screen.blit(invert_label, invert_label_pos)

            # Draw checkbox
            checkbox_size = 20
            checkbox_rect = pygame.Rect(invert_checkbox_pos[0], invert_checkbox_pos[1], checkbox_size, checkbox_size)
            pygame.draw.rect(screen, BLACK, checkbox_rect, 2)  # Border
            if invert:
                pygame.draw.line(screen, BLACK, checkbox_rect.topleft, checkbox_rect.bottomright, 2)
                pygame.draw.line(screen, BLACK, checkbox_rect.topright, checkbox_rect.bottomleft, 2)

        # Handle hat Y-axis
        elif key.startswith('hat_') and key.endswith('_y'):
            hat_value = input_status.get(key, 0.0)
            direction = "Up" if hat_value > 0 else ("Down" if hat_value < 0 else "Neutral")
            color = PINK if hat_value != 0.0 else WHITE
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, BLACK, rect, 2)  # Draw the border of the box
            # Get the channel mapping
            mapping = hat_channel_map['y'].get(key, {})
            channel_mapped = mapping.get('channel')
            invert = mapping.get('invert', False)
            if channel_mapped:
                label = f"HAT Y\n(Channel {channel_mapped})\nDirection: {direction}"
            else:
                label = f"HAT Y\n(Channel None)\nDirection: {direction}"
            for idx, line in enumerate(label.split('\n')):
                screen.blit(font.render(line, True, BLACK), (rect.x + 10, rect.y + 5 + (idx * 15)))

            # Draw "Invert" label
            invert_label = font.render("Invert:", True, BLACK)
            screen.blit(invert_label, invert_label_pos)

            # Draw checkbox
            checkbox_size = 20
            checkbox_rect = pygame.Rect(invert_checkbox_pos[0], invert_checkbox_pos[1], checkbox_size, checkbox_size)
            pygame.draw.rect(screen, BLACK, checkbox_rect, 2)  # Border
            if invert:
                pygame.draw.line(screen, BLACK, checkbox_rect.topleft, checkbox_rect.bottomright, 2)
                pygame.draw.line(screen, BLACK, checkbox_rect.topright, checkbox_rect.bottomleft, 2)

        # Handle button inputs
        elif key.startswith('button_'):
            # If the input is a button, fill the box if the button is pressed
            mapping = button_channel_map.get(key, {})
            channel_mapped = mapping.get('channel')
            invert = mapping.get('invert', False)
            color = PINK if input_status.get(key, False) else WHITE
            pygame.draw.rect(screen, color, rect)
            # Get the channel mapping
            if channel_mapped:
                label = f"Button {key.split('_')[-1]}\n(Channel {channel_mapped})\nVALUE: {input_status.get(key, False)}"
            else:
                label = f"Button {key.split('_')[-1]}\n(Channel None)\nVALUE: {input_status.get(key, False)}"
            pygame.draw.rect(screen, BLACK, rect, 2)  # Draw the border of the box
            for idx, line in enumerate(label.split('\n')):
                screen.blit(font.render(line, True, BLACK), (rect.x + 10, rect.y + 5 + (idx * 15)))

            # Draw "Invert" label
            invert_label = font.render("Invert:", True, BLACK)
            screen.blit(invert_label, invert_label_pos)

            # Draw checkbox
            checkbox_size = 20
            checkbox_rect = pygame.Rect(invert_checkbox_pos[0], invert_checkbox_pos[1], checkbox_size, checkbox_size)
            pygame.draw.rect(screen, BLACK, checkbox_rect, 2)  # Border
            if invert:
                pygame.draw.line(screen, BLACK, checkbox_rect.topleft, checkbox_rect.bottomright, 2)
                pygame.draw.line(screen, BLACK, checkbox_rect.topright, checkbox_rect.bottomleft, 2)

        else:
            # Other inputs
            pass

# Function to draw the channel boxes based on the current input status and channels
def draw_channel_boxes(channel_boxes, channels, axis_channel_map, button_channel_map, hat_channel_map):
    for key, rect in channel_boxes.items():
        channel_num = int(key.split('_')[-1]) - 1  # Channels are from 1 to 16
        crsf_value = channels[channel_num]  # This is in 0-2047
        us_value = int(crsf_value * 1000 / 2047 + 1000)  # Map back to 1000-2000 us

        # Find if this channel is mapped from an axis, button, or hat
        input_mapped = None
        for axis_key, mapping in axis_channel_map.items():
            if mapping['channel'] - 1 == channel_num:
                input_mapped = axis_key
                break
        if not input_mapped:
            for button_key, mapping in button_channel_map.items():
                if mapping['channel'] - 1 == channel_num:
                    input_mapped = button_key
                    break
        if not input_mapped:
            for axis_type, mapping_dict in hat_channel_map.items():
                for hat_key, mapping in mapping_dict.items():
                    if mapping['channel'] - 1 == channel_num:
                        input_mapped = hat_key
                        break
                if input_mapped:
                    break

        # Prepare the label
        if input_mapped:
            # Convert input_mapped to a friendly name
            if input_mapped.startswith('axis_'):
                input_name = f"Axis {input_mapped.split('_')[-1]}"
            elif input_mapped.startswith('button_'):
                input_name = f"Button {input_mapped.split('_')[-1]}"
            elif input_mapped.startswith('hat_'):
                if input_mapped.endswith('_x'):
                    hat_id = input_mapped.split('_')[1]
                    input_name = f"HAT {hat_id} X"
                elif input_mapped.endswith('_y'):
                    hat_id = input_mapped.split('_')[1]
                    input_name = f"HAT {hat_id} Y"
                else:
                    input_name = input_mapped
            else:
                input_name = input_mapped
            label = f"Channel {channel_num + 1}\n({input_name})\nPWM: {us_value} µs\nCRSF: {crsf_value}"
        else:
            label = f"Channel {channel_num + 1}\nPWM: {us_value} µs\nCRSF: {crsf_value}"

        # Check if the input is active
        # For channel boxes, we don't have access to input_status directly here
        # So, we'll assume channels are active if they deviate from 1500
        is_active = channels[channel_num] != 1500

        # Fill the box proportionally to the us_value
        fill_width = int((us_value - 1000) / 1000 * rect.width)
        fill_width = max(0, min(rect.width, fill_width))  # Ensure within bounds
        pygame.draw.rect(screen, PINK, (rect.x, rect.y, fill_width, rect.height))

        # Draw the border with GREEN if active, else BLACK
        border_color = GREEN if is_active else BLACK
        pygame.draw.rect(screen, border_color, rect, 2)

        # Render the label
        for idx, line in enumerate(label.split('\n')):
            screen.blit(font.render(line, True, BLACK), (rect.x + 5, rect.y + 5 + (idx * 18)))

# Function to draw the dropdown menu with dynamic positioning
def draw_dropdown(rect, options, selected_option):
    option_height = 25
    dropdown_height = len(options) * option_height
    # Determine if dropdown should appear above or below
    if rect.y + rect.height + dropdown_height > HEIGHT:
        # Not enough space below, draw above
        dropdown_y = rect.y - dropdown_height
    else:
        # Draw below
        dropdown_y = rect.y + rect.height
    dropdown_rect = pygame.Rect(rect.x, dropdown_y, rect.width, dropdown_height)
    pygame.draw.rect(screen, WHITE, dropdown_rect)
    pygame.draw.rect(screen, BLACK, dropdown_rect, 2)
    for idx, option in enumerate(options):
        option_rect = pygame.Rect(rect.x, dropdown_y + idx * option_height, rect.width, option_height)
        if option == selected_option:
            pygame.draw.rect(screen, LIGHT_BLUE, option_rect)
        screen.blit(font.render(option, True, BLACK), (option_rect.x + 5, option_rect.y + 5))
    return dropdown_rect

# GUI elements for serial port and joystick selection
def draw_gui_elements(selected_serial_port, selected_baud_rate, selected_joystick_index, serial_ports, baud_rates, joystick_names, telemetry_data):
    # Serial Port Selection
    serial_rect = pygame.Rect(800, 50, 350, 30)
    pygame.draw.rect(screen, LIGHT_BLUE, serial_rect)
    pygame.draw.rect(screen, BLACK, serial_rect, 2)
    serial_text = font.render(f"Serial Port: {selected_serial_port}", True, BLACK)
    screen.blit(serial_text, (serial_rect.x + 5, serial_rect.y + 5))

    # Baud Rate Selection
    baud_rect = pygame.Rect(800, 100, 350, 30)
    pygame.draw.rect(screen, LIGHT_BLUE, baud_rect)
    pygame.draw.rect(screen, BLACK, baud_rect, 2)
    baud_text = font.render(f"Baud Rate: {selected_baud_rate}", True, BLACK)
    screen.blit(baud_text, (baud_rect.x + 5, baud_rect.y + 5))

    # Joystick Selection
    joystick_rect = pygame.Rect(800, 150, 350, 30)
    pygame.draw.rect(screen, LIGHT_BLUE, joystick_rect)
    pygame.draw.rect(screen, BLACK, joystick_rect, 2)
    if selected_joystick_index is not None and selected_joystick_index >=0 and selected_joystick_index < len(joystick_names) -1:
        joystick_name = joystick_names[selected_joystick_index +1]
    else:
        joystick_name = "None"
    joystick_text = font.render(f"Joystick: {joystick_name}", True, BLACK)
    screen.blit(joystick_text, (joystick_rect.x + 5, joystick_rect.y + 5))

    # Telemetry Data Display
    telemetry_rect = pygame.Rect(800, 200, 350, 200)
    pygame.draw.rect(screen, WHITE, telemetry_rect)
    pygame.draw.rect(screen, BLACK, telemetry_rect, 2)
    telemetry_text = "Telemetry Data:"
    y_offset = telemetry_rect.y + 5
    screen.blit(font.render(telemetry_text, True, BLACK), (telemetry_rect.x + 5, y_offset))
    y_offset += 25
    for key, value in telemetry_data.items():
        text_line = f"{key}: {value}"
        screen.blit(font.render(text_line, True, BLACK), (telemetry_rect.x + 5, y_offset))
        y_offset += 20

    return serial_rect, baud_rect, joystick_rect

def main():
    clock = pygame.time.Clock()

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

    # Initial selected options
    selected_serial_port = serial_port
    selected_baud_rate = baud_rate
    baud_rates = [921600, 9600, 19200, 38400, 57600, 115200, 400000, 1870000, 3750000, 5250000]  # Baud rates list starting with 921600
    selected_joystick_index = joystick_index
    serial_ports = [port.device for port in serial.tools.list_ports.comports()]
    joystick_names = ["None"] + [pygame.joystick.Joystick(i).get_name() for i in range(pygame.joystick.get_count())]

    joystick, boxes, channel_boxes = initialize_controller(selected_joystick_index)

    ser = None
    if selected_serial_port != "Not Connected":
        try:
            ser = serial.Serial(selected_serial_port, selected_baud_rate, timeout=0)
        except Exception as e:
            print(f"Failed to open serial port {selected_serial_port}: {e}")
            ser = None

    ser_thread = None
    ser_lock = threading.Lock()
    running = True

    telemetry_data = {}  # Dictionary to hold telemetry data

    # Define the mappings between axes/buttons and channels with inversion flags
    axis_channel_map = default_axis_channel_map.copy()  # e.g., {'axis_0': {'channel': 1, 'invert': False}, ...}
    button_channel_map = default_button_channel_map.copy()  # e.g., {'button_0': {'channel': 2, 'invert': False}, ...}
    hat_channel_map = {'x': default_hat_channel_map['x'].copy(), 'y': default_hat_channel_map['y'].copy()}  # e.g., {'x': {'hat_0_x': {'channel': 3, 'invert': False}}, 'y': {...}}

    selected_input = None
    dropdown_active = False
    dropdown_options = []
    dropdown_rect = None

    def serial_thread_func():
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

    serial_read_thread = threading.Thread(target=serial_thread_func, daemon=True)
    serial_read_thread.start()

    def serial_write_thread_func():
        while running:
            if ser and ser.is_open:
                with ser_lock:
                    channels = get_channels_from_joystick(joystick, axis_channel_map, button_channel_map, hat_channel_map)
                    packet = channelsCrsfToChannelsPacket(channels)
                    ser.write(packet)
                time.sleep(0.02)
            else:
                time.sleep(0.1)

    ser_write_thread = threading.Thread(target=serial_write_thread_func, daemon=True)
    ser_write_thread.start()

    while running:
        screen.fill(WHITE)  # Clear the screen with white before drawing

        # Event handling
        input_status = {}
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                if dropdown_active:
                    # Check if click is inside dropdown
                    if dropdown_rect and dropdown_rect.collidepoint(mouse_pos):
                        # Calculate which option was clicked
                        option_index = (mouse_pos[1] - dropdown_rect.y) // 25
                        if 0 <= option_index < len(dropdown_options):
                            selected_channel_str = dropdown_options[option_index]
                            selected_channel = int(selected_channel_str.split()[1])
                            # Update mappings
                            # Remove previous mapping if the channel is already mapped
                            for mapping in [axis_channel_map, button_channel_map, hat_channel_map['x'], hat_channel_map['y']]:
                                keys_to_remove = [k for k, v in mapping.items() if v['channel'] == selected_channel]
                                for k in keys_to_remove:
                                    del mapping[k]
                            # Add new mapping
                            if selected_input.startswith('axis_'):
                                axis_channel_map[selected_input] = {'channel': selected_channel, 'invert': False}
                            elif selected_input.startswith('button_'):
                                button_channel_map[selected_input] = {'channel': selected_channel, 'invert': False}
                            elif selected_input.startswith('hat_'):
                                # Determine if it's x or y axis
                                if selected_input.endswith('_x'):
                                    hat_channel_map['x'][selected_input] = {'channel': selected_channel, 'invert': False}
                                elif selected_input.endswith('_y'):
                                    hat_channel_map['y'][selected_input] = {'channel': selected_channel, 'invert': False}
                            # Close dropdown
                            dropdown_active = False
                            selected_input = None
                    else:
                        # Clicked outside dropdown, close it
                        dropdown_active = False
                        selected_input = None
                elif serial_rect.collidepoint(mouse_pos):
                    # Toggle serial port selection
                    serial_ports = [port.device for port in serial.tools.list_ports.comports()]
                    if serial_ports:
                        if selected_serial_port in serial_ports:
                            index = serial_ports.index(selected_serial_port)
                            selected_serial_port = serial_ports[(index + 1) % len(serial_ports)]
                        else:
                            selected_serial_port = serial_ports[0]
                    else:
                        selected_serial_port = "Not Connected"
                    # Reconnect serial port
                    with ser_lock:
                        if ser and ser.is_open:
                            ser.close()
                        if selected_serial_port != "Not Connected":
                            try:
                                ser = serial.Serial(selected_serial_port, selected_baud_rate, timeout=0)
                            except Exception as e:
                                print(f"Failed to open serial port {selected_serial_port}: {e}")
                                ser = None
                        else:
                            ser = None
                elif baud_rect.collidepoint(mouse_pos):
                    # Toggle baud rate selection
                    if selected_baud_rate in baud_rates:
                        index = baud_rates.index(selected_baud_rate)
                        selected_baud_rate = baud_rates[(index + 1) % len(baud_rates)]
                    else:
                        selected_baud_rate = baud_rates[0]
                    # Reconnect serial port with new baud rate
                    with ser_lock:
                        if ser and ser.is_open:
                            ser.close()
                        if selected_serial_port != "Not Connected":
                            try:
                                ser = serial.Serial(selected_serial_port, selected_baud_rate, timeout=0)
                            except Exception as e:
                                print(f"Failed to open serial port {selected_serial_port}: {e}")
                                ser = None
                        else:
                            ser = None
                elif joystick_rect.collidepoint(mouse_pos):
                    # Toggle joystick selection
                    joystick_names = ["None"] + [pygame.joystick.Joystick(i).get_name() for i in range(pygame.joystick.get_count())]
                    if len(joystick_names) > 1:
                        index = selected_joystick_index if selected_joystick_index is not None else -1
                        index = (index + 1) % (len(joystick_names) - 1)  # Adjust for "None" option
                        selected_joystick_index = index if index >= 0 else None
                    else:
                        selected_joystick_index = None
                    if joystick:
                        joystick.quit()
                    if selected_joystick_index is not None and selected_joystick_index >= 0:
                        joystick = pygame.joystick.Joystick(selected_joystick_index)
                        joystick.init()
                    else:
                        joystick = None
                    joystick, boxes, channel_boxes = initialize_controller(selected_joystick_index)
                else:
                    # Check if clicked on any input box's checkbox
                    checkbox_clicked = False
                    for key, rect in boxes.items():
                        # Calculate positions for "Invert" label and checkbox
                        invert_checkbox_pos = (rect.x + rect.width - 80, rect.y + 25)  # Below the label
                        checkbox_size = 20
                        checkbox_rect = pygame.Rect(invert_checkbox_pos[0], invert_checkbox_pos[1], checkbox_size, checkbox_size)
                        if checkbox_rect.collidepoint(mouse_pos):
                            # Toggle the 'invert' flag for this mapping
                            if key.startswith('axis_'):
                                if key in axis_channel_map:
                                    axis_channel_map[key]['invert'] = not axis_channel_map[key]['invert']
                            elif key.startswith('hat_'):
                                if key.endswith('_x') and key in hat_channel_map['x']:
                                    hat_channel_map['x'][key]['invert'] = not hat_channel_map['x'][key]['invert']
                                elif key.endswith('_y') and key in hat_channel_map['y']:
                                    hat_channel_map['y'][key]['invert'] = not hat_channel_map['y'][key]['invert']
                            elif key.startswith('button_'):
                                if key in button_channel_map:
                                    button_channel_map[key]['invert'] = not button_channel_map[key]['invert']
                            checkbox_clicked = True
                            break  # Only one checkbox can be clicked at a time

                    if not checkbox_clicked:
                        # Check if clicked on any input box to select for mapping
                        for key, rect in boxes.items():
                            if rect.collidepoint(mouse_pos):
                                selected_input = key
                                dropdown_active = True
                                # Prepare dropdown options
                                mapped_channels = set()
                                for mapping in axis_channel_map.values():
                                    mapped_channels.add(mapping['channel'])
                                for mapping in button_channel_map.values():
                                    mapped_channels.add(mapping['channel'])
                                for mapping in hat_channel_map['x'].values():
                                    mapped_channels.add(mapping['channel'])
                                for mapping in hat_channel_map['y'].values():
                                    mapped_channels.add(mapping['channel'])
                                dropdown_options = []
                                for ch in range(1, 17):
                                    option = f"Channel {ch}"
                                    if ch in mapped_channels:
                                        option += " (mapped)"
                                    dropdown_options.append(option)
                                dropdown_rect = pygame.Rect(rect.x, rect.y + rect.height, rect.width, len(dropdown_options) * 25)
                                break
            elif event.type == pygame.JOYDEVICEADDED or event.type == pygame.JOYDEVICEREMOVED:
                # Update joystick list
                joystick_names = ["None"] + [pygame.joystick.Joystick(i).get_name() for i in range(pygame.joystick.get_count())]

        if joystick:
            # Check joystick axes and store their current values
            for i in range(joystick.get_numaxes()):
                axis_value = joystick.get_axis(i)
                input_status[f'axis_{i}'] = axis_value

            # Check joystick buttons and store their current values (pressed or not)
            for i in range(joystick.get_numbuttons()):
                button_value = joystick.get_button(i)
                input_status[f'button_{i}'] = button_value == 1

            # Check hat switches (D-pad) and store their current values
            for i in range(joystick.get_numhats()):
                hat_x = joystick.get_hat(i)[0]  # Left (-1), Neutral (0), Right (1)
                hat_y = joystick.get_hat(i)[1]  # Down (-1), Neutral (0), Up (1)
                input_status[f'hat_{i}_x'] = hat_x
                input_status[f'hat_{i}_y'] = hat_y
        else:
            input_status = {}

        # Map the inputs to channels
        channels = get_channels_from_joystick(joystick, axis_channel_map, button_channel_map, hat_channel_map)

        # Draw the input boxes based on the current input status and channels
        draw_input_boxes(input_status, boxes, axis_channel_map, button_channel_map, hat_channel_map, selected_input)

        # Draw channel maps
        draw_channel_boxes(channel_boxes, channels, axis_channel_map, button_channel_map, hat_channel_map)

        # Draw dropdown if active
        if dropdown_active and selected_input:
            rect = boxes[selected_input]
            dropdown_rect = draw_dropdown(rect, dropdown_options, None)

        # Draw GUI elements
        serial_rect, baud_rect, joystick_rect = draw_gui_elements(
            selected_serial_port,
            selected_baud_rate,
            selected_joystick_index,
            serial_ports,
            baud_rates,
            joystick_names,
            telemetry_data
        )

        # Update display
        pygame.display.flip()  # Update the full display surface to the screen
        clock.tick(50)  # Limit the loop to 50 frames per second

if __name__ == "__main__":
    main()
