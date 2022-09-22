import serial
from serial.tools import list_ports
import crcmod    # if you encounter an error here, pip install crcmod

'''
Low-level driver functions for the Coretronics Venus3 stereo DLP driver PCBA.
Connect the PCBA to a USB port to power the PCBA. After successful enumeration,
a virtual COM-port is established via which the TI3433 DLP controller can be 
accessed.
'''

class coretronics_venus3:
    
    #####################################################################
    # Private members of the class, setting up communication mechanisms #
    #####################################################################

    def __init__(self):
        self.ser = 0
        self.PCBA_NAME = "Coretronics Venus3"
        self.isOpen = self._open_serial()

        # CRC-16 CCITT check sum of whole package from HEAD to PAYLOAD including commas
        # http://srecord.sourceforge.net/crc16-ccitt.html
        # CRC “polynomial” is 0x1021 and initial value is 0 and “final XOR value” is 0.
        self.crc_fun = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0x0000, xorOut=0x0000)

    def __del__(self):
        """RAII: release the virtual COM port when the coretronics instance goes out of scope."""
        if self.ser.isOpen():
            self.ser.close()

    def _open_serial(self):
        """ Opens the serial port to the Coretronics device
        """
        for port in list_ports.comports():
            if port.description.find('EVK1XXX Virtual Com Port') == 0:
                self.ser = serial.Serial(
                    port=port.device,
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout = 0.5)
                return True
        assert "Coretronic Device Not Found!"
        return False

    def _to_hex(self, integer, num_result_hex_digits=2):
        """Convert a byte to a two-digit hex string"""
        assert isinstance(integer, int) 
        assert integer == integer & 0xFF
        padding = 1 << (4*num_result_hex_digits)
        return hex(padding + integer)[3:]

    def _get_crc_str(self, cmd):
        crc16 = self.crc_fun(bytearray(cmd, 'ascii'))
        return self._to_hex(crc16 & 0xFF).upper()

    def _send_command( self, cmd_idx, payload, wg):
        """ Sends a command to via the serial interface.
            Parameters:
                cmd_idx : command index [byte]
                payload : command operands [bytearray]
                wg      : side selector ( LEFT, RIGHT, or BOTH)
        """

        # General command packet structure: [HEAD][CMD],[WG],[LENGTH],[PAYLOAD],[CRC][\0], where
        # [HEAD]      : '#'
        # [CMD]       : two hex characters, from 00 to FF
        # [WG]        : '0': left, '1': right, '2': both
        # [Length]    : payload length in string format (N)
        # [Payload]   : N bytes of data, Data0 .. Data N-1
        # [CRC]       : The lower byte of the CRC-16.

        payload_str = ""
        for d in payload:
            payload_str += ',' + self._to_hex(d)
        cmd = "#{0},{1},{2}{3},".format(self._to_hex(cmd_idx), wg, str(len(payload)), payload_str).upper()
        cmd += self._get_crc_str(cmd) + '\0'
        self.ser.write(bytes(cmd, 'ascii'))
    
    def _read_line(self, eol):
        ret = b''
        while 1:
            c = self.ser.read()
            assert len(c)>0, "Response timeout!" 
            if c==eol: return ret
            ret += c

    def _read_response(self, cmd_id, channel, expect_only_ACK=False):       
        response = ""
        # Considering serial responses indicate command_id and side, 
        # the Venus may, in theory respond to queries out-of-order.
        # However, ACK after response indicates completion of command,
        # suggesting a simple response/reply (pull model) protocol.
        while 1:
            c = self._read_line(eol=b'\0')
            response = c.decode()
            assert response[0] == '#', "First character of response [HEAD] expected to be '#'!" 
            assert response[1] != 'N', "Command NACK received!"
            if response[1]=='A':                     
                if expect_only_ACK: return
                return byte_list
            else: 
                cmd, wg, byte_list = self._parse_response(response)
                assert cmd_id==int(cmd,16), "Mismatch between queried command ID and response ID!"
                assert int(wg)==int(channel), "Mismatch between command and response channel IDs!"
     
    def _parse_response(self, response):
        assert response[0] == '#', "First character of response [HEAD] expected to be '#'!" 
        try:
            last_comma_pos = response.rfind(',')
            crc_read = int(response[last_comma_pos+1:], 16)
            crc_calc = self.crc_fun(bytes(response[0:last_comma_pos+1], 'ascii')) & 0xFF
            chunks = response[1:].split(',')
            cmd = chunks[0]
            wg  = chunks[1]
            length = int(chunks[2], 16)
            payload = [int(i,16) for i in chunks[3:3+length]]
            assert  crc_read == crc_calc, "CRC mismatch!" 
            return cmd, wg, payload
        except:
            assert ValueError("Error parsing read response from {}!".format(self.PCBA_NAME))
        return

    def _get_rgb(self, byte_list):
        R = (byte_list[1]*256 + byte_list[0])
        G = (byte_list[3]*256 + byte_list[2])
        B = (byte_list[5]*256 + byte_list[4])
        return R,G,B

    def _set_rgb(self, R,G,B):
        return bytes([R & 0xFF, R >> 8, G & 0xFF, G >> 8, B & 0xFF, B >> 8])        
    # --------------------------------------
    # Implementation of public API functions
    # --------------------------------------

    # Venus3 specific enums for specifying DLP instances:
    LEFT                    = '0'
    RIGHT                   = '1'
    BOTH                    = '2'

    # Video source specific constants:
    EXTERNAL_VIDEO_PORT     = 0
    TEST_PATTERN_GENERATOR  = 1
    SPLASH_SCREEN           = 2

    def set_input_source(self, channel, source):
        """ Selects source for DLP(s)
            parameters:
                channel: LEFT, RIGHT or BOTH
                source:  EXTERNAL_VIDEO_PORT, TEST_PATTERN_GENERATOR, or SPLASH_SCREEN
            returns:
                None
        """
        assert channel == self.LEFT or channel == self.RIGHT or channel == self.BOTH
        assert (source == self.EXTERNAL_VIDEO_PORT or  
                source == self.TEST_PATTERN_GENERATOR or 
                source == self.SPLASH_SCREEN)
        cmd_id = 0x05
        self._send_command(cmd_id, bytes([source]), channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void

    def get_input_source(self, channel):
        """ Reads input source setting from the DLP
            parameters:
                channel: LEFT, RIGHT 
            returns:
                source:  EXTERNAL_VIDEO_PORT, TEST_PATTERN_GENERATOR, or SPLASH_SCREEN
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x06
        self._send_command(cmd_id, bytearray(), channel)
        source = self._read_response(cmd_id, channel)[0]      
        assert (source == self.EXTERNAL_VIDEO_PORT or  
                source == self.TEST_PATTERN_GENERATOR or 
                source == self.SPLASH_SCREEN)
        return source

    COLOR_BARS              = 0
    SOLID_FIELD_WHITE       = 1
    SOLID_FIELD_RED         = 2
    SOLID_FIELD_GREEN       = 3
    SOLID_FIELD_BLUE        = 4
    HORIZONTAL_RAMP         = 5
    GRID                    = 6
    CHECKERBOARD            = 7
    SOLID_FIELD_BLACK       = 8
    SOLID_FIELD_CYAN        = 9
    SOLID_FIELD_YELLOW      = 10
    VERTICAL_RAMP           = 11
    HORIZONTAL_LINES        = 12
    VERTICAL_LINES          = 13
    DIAGONAL_LINES          = 14

    def set_test_pattern(self, channel, pattern):
        """ Selects test pattern for DLP(s)
            parameters:
                channel: LEFT, RIGHT or BOTH
                pattern: byte in the range [0..14]. 
                         For a list of enums see "Venus3 specific enums" above 
            returns:
                None
        """
        assert channel == self.LEFT or channel == self.RIGHT or channel == self.BOTH
        assert isinstance(pattern, int), ValueError(pattern)
        assert (0 <= pattern <= 14), ValueError(pattern)
        cmd_id = 0x0B
        self._send_command(cmd_id, bytes([pattern]), channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void

    def set_image_orientation(self, channel, flip_x, flip_y, rot_90):
        """ Sets image orientation for the DLP(s)
            parameters:
                channel: LEFT, RIGHT or BOTH
                flip_x: True or 1 indicates that the image needs to be flipped along the long axis.
                flip_y: True or 1indicates that the image needs to be flipped along the short axis.
                rot_90: True or 1indicates that the image needs to rotated 90 degrees counterclockwise
            returns:
                None
        """
        assert channel == self.LEFT or channel == self.RIGHT or channel == self.BOTH
        byte = 0
        if flip_y: byte += 4
        if flip_x: byte += 2
        if rot_90: byte += 1
        cmd_id = 0x14
        self._send_command(cmd_id, bytes([byte]), channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void
    
    def get_image_orientation(self, channel):
        """ Reads current image orientation of a DLP
            parameters:
                channel: LEFT, RIGHT
            returns:
                flip_x: True or 1 indicates that the image needs to be flipped along the long axis.
                flip_y: True or 1indicates that the image needs to be flipped along the short axis.
                rot_90: True or 1indicates that the image needs to rotated 90 degrees counterclockwise
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x15
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        byte = byte_list[0]
        flip_y = (byte & 4) > 0
        flip_x = (byte & 2) > 0
        rot_90 = (byte & 1) > 0
        return flip_x, flip_y, rot_90
    
    def set_image_freeze(self, channel, freeze_enabled):
        """ Freezes / unfreezes image on the DLP(s)
            parameters:
                channel: LEFT, RIGHT or BOTH
                freeze_enabled: True or 1 indicates that display image freezing is enabled
            returns:
                None
        """
        assert channel == self.LEFT or channel == self.RIGHT or channel == self.BOTH
        byte = 0
        if freeze_enabled: byte += 1
        cmd_id = 0x1A
        self._send_command(cmd_id, bytes([byte]), channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void
    
    def get_image_freeze(self, channel):
        """ Reads the image freezing setting of a DLP.
            parameters:
                channel: LEFT, RIGHT
            returns:
                freeze_enabled: True or 1 indicates that display image freezing is enabled
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x1B
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        byte = byte_list[0]
        freeze_enabled = (byte & 1) > 0
        return freeze_enabled

    def set_look(self, channel, look_id):
        """ Sets proprietary display tonality preset
            parameters:
                look_id: byte
            returns:
                None
        """
        assert channel == self.LEFT or channel == self.RIGHT or channel == self.BOTH
        assert isinstance(look_id, int), ValueError("look_id should be integer!")
        assert 0 <= look_id <= 255, ValueError("look_id should be in the range of [0..255]!")
        cmd_id = 0x22
        self._send_command(cmd_id, bytes([look_id]), channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void
    
    def get_look(self, channel):
        """ Reads the image freezing setting of a DLP.
            parameters:
                channel: LEFT, RIGHT
            returns:
                look_id: byte, proprietary display tonality enum
                sequence_id: byte
                frame_rate: unigned int
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x23
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        look_id = byte_list[0]
        sequence_id = byte_list[1]
        frame_rate_int = byte_list[2] + (byte_list[3] << 8) +(byte_list[4] << 16) + (byte_list[5] << 24)
        return look_id, sequence_id, frame_rate_int

    def set_RGB_duty_cycle(self, channel, R, G, B):
        """ Writes R,G,B LED duty cycles of attached DLPs.
            !!!! This is an undocumented feature !!!!
            parameters:
                channel: LEFT, RIGHT, or BOTH
                R,G,B: uint16s, proportional to LED duty cycles.
        """
        for c in [R,G,B]:
            assert isinstance(c, int), ValueError("R,G,B should be integers!")
            
        cmd_id = 0x25
        payload = self._set_rgb(R,G,B)
        self._send_command(cmd_id, payload, channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void

    def get_RGB_duty_cycle(self, channel):
        """ Reads R,G,B LED duty cycles from attached DLP.
            parameters:
                channel: LEFT or RIGHT
            returns
                [R,G,B] list of uint16s, proportional to LED duty cycle
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x26
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        return self._get_rgb(byte_list)

    def set_CAIC_enable(self, channel, CAIC_enabled):
        """ Enables or disables the Content Adaptive Illumination Control (CAIC) in the TI DLP controller.
            More information: https://www.ti.com/lit/an/dlpa058/dlpa058.pdf 
            parameters:
                channel: LEFT, RIGHT or BOTH
                CAIC_enabled: True or 1 indicates that display image freezing is enabled
            returns:
                None
        """
        assert channel == self.LEFT or channel == self.RIGHT or channel == self.BOTH
        byte = 0
        if CAIC_enabled: byte += 1
        cmd_id = 0x50
        self._send_command(cmd_id, bytes([byte]), channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void
    
    def get_CAIC_enable(self, channel):
        """ Reads Content Adaptive Illumination Control (CAIC) enablement status.
            parameters:
                channel: LEFT, RIGHT
            returns:
                CAIC_enabled: True or 1 indicates that display image freezing is enabled
                More information: https://www.ti.com/lit/an/dlpa058/dlpa058.pdf 
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x51
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        return (byte_list[0] & 1) > 0

    def set_RGB_enable(self, channel, red_enabled, green_enabled, blue_enabled):
        """ Enables or disables the individual R,G,B LEDs in the DLP light engine.
            parameters:
                channel: LEFT, RIGHT or BOTH
                red_enabled:   True or 1 indicates that Red LED should be enabled
                green_enabled: True or 1 indicates that Green LED should be enabled
                blue_enabled:  True or 1 indicates that Blue LED should be enabled
            returns:
                None
        """
        assert channel == self.LEFT or channel == self.RIGHT or channel == self.BOTH
        byte = 0
        if red_enabled: byte += 1
        if green_enabled: byte += 2
        if blue_enabled: byte += 4
        cmd_id = 0x52
        self._send_command(cmd_id, bytes([byte]), channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void
    
    def get_RGB_enable(self, channel):
        """ Reads enablement status of the red, green, and blue LEDs in the DLP selected.
            parameters:
                channel: LEFT, RIGHT
            returns:
                red_enabled:   True when the Red LED is enabled
                green_enabled: True when the Green LED is enabled
                blue_enabled:  True when the Blue LED is enabled
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x53
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        byte = byte_list[0]
        red_enabled =   (byte & 0x01) > 0 
        green_enabled = (byte & 0x02) > 0 
        blue_enabled =  (byte & 0x04) > 0 
        return red_enabled, green_enabled, blue_enabled

    def set_RGB_currents(self, channel, R, G, B):
        """ Writes R,G,B LED currents for attached DLPs.
            parameters:
                channel: LEFT, RIGHT, or BOTH
                R,G,B: uint16s, proportional to LED currents.
        """
        for c in [R,G,B]:
            assert isinstance(c, int), ValueError("R,G,B should be integers!")
            assert (c>=12) and (c<=350), ValueError("R,G,B should be in the range of [12..350]!")

        cmd_id = 0x54
        payload = self._set_rgb(R,G,B)
        self._send_command(cmd_id, payload, channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void

    def get_RGB_currents(self, channel):
        """ Reads R,G,B LED currents from attached DLPs.
            parameters:
                channel: LEFT or RIGHT
            returns
                [R,G,B] list of uint16s, proportional to LED currents.
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x55
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        return self._get_rgb(byte_list)

    def set_brightness_boost(self, channel, sharpness, LABB_control, LABB_manual_setting):
        """ Write control parameters for the TI DLP Local Area Brightness Boost (LABB) algorithm.
            More information: https://www.ti.com/lit/an/dlpa058/dlpa058.pdf
            parameters:
                channel: LEFT, RIGHT, or BOTH
                sharpness: sharpness strength (integer, 0..15)
                LABB_control: 0: disabled, 1: manual, 2: automatic, using light sensor
                LABB_manual_setting: byte
        """
        for c in [sharpness, LABB_control, LABB_manual_setting]:
            assert isinstance(c, int), ValueError("Parameters should be integers!")
        assert (0 <= sharpness <= 15), ValueError("Parameter sharpness out of range!")
        assert (0 <= LABB_control <= 2), ValueError("Parameter LABB_control out of range!")
        assert (0 <= LABB_manual_setting <= 255), ValueError("Parameter LABB_manual_setting out of range!")

        cmd_id = 0x80
        payload = bytes([ (sharpness << 4) + LABB_control, LABB_manual_setting])
        self._send_command(cmd_id, payload, channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void

    def get_brightness_boost(self, channel):
        """ Read control parameters for the TI DLP Local Area Brightness Boost (LABB) algorithm.
            More information: https://www.ti.com/lit/an/dlpa058/dlpa058.pdf
            parameters:
                channel: LEFT or RIGHT
            returns
                sharpness: sharpness strength (integer, 0..15)
                LABB_control: 0: disabled, 1: manual, 2: automatic, using light sensor
                LABB_manual_setting: byte
                LABB_gain: byte
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x81
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        byte_1 = byte_list[0]
        sharpness = byte_1 >> 4
        LABB_control = byte_1 & 0x03
        LABB_manual_setting = byte_list[1] 
        LABB_gain = byte_list[2]
        return [sharpness, LABB_control, LABB_manual_setting, LABB_gain]

    def get_ASIC_device_ID(self, channel):
        """ Reads DLP driver device ID.
            parameters:
                channel: LEFT or RIGHT
            returns
                device_ID (uint16):
                0 : DLP3430
                1 : DLP3433
                4 : DLP3435
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0xD4
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        device_id = byte_list[0]
        if (device_id == 0): return 'DLP3430'
        if (device_id == 1): return 'DLP3433'
        if (device_id == 4): return 'DLP3435'
        assert "Unknown device ID!"

    # Predefined DMD device types
    DMD_WVGA = "64000D60"
    DMD_720p = "68000D60"
    DMD_ours = "72000D60"
    def get_DMD_device_ID(self, channel):
        """ Reads DMD device ID and type
            parameters:
                channel: LEFT or RIGHT
            returns
                DMD_device_ID: byte
                type_code = hex string, DMD_WVGA or DMD_720p
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0xD5
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        device_id = byte_list[0] & 0x07
        type = (byte_list[3] << 24) + (byte_list[2] << 16) + (byte_list[1] << 8) + byte_list[0]
        return device_id, hex(type)[2:].upper()

    def get_DLP_flash_version(self, channel):
        """ Reads DMD device ID and type
            parameters:
                channel: LEFT or RIGHT
            returns
                DMD_device_ID: byte
                version: hex string (major, minor, patch_MSB, patch_version_LSB)
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0xD9
        self._send_command(cmd_id, bytes(), channel)
        byte_list = self._read_response(cmd_id, channel)
        version = "{0}.{1}.{2}".format( byte_list[3], byte_list[2], hex((byte_list[1] << 8) + byte_list[0])[2:].upper())
        return version

# ---------------------------------------------------
# Demonstrate / test the coretronics_venus3 class
# ---------------------------------------------------
if __name__ == "__main__":
    dlp = coretronics_venus3()
    if dlp.isOpen:
        DLP_driver_device_ID = dlp.get_ASIC_device_ID(dlp.LEFT)
        DMD_driver_device_ID, type = dlp.get_DMD_device_ID(dlp.LEFT)
        DLP_Flash_Version = dlp.get_DLP_flash_version(dlp.LEFT)
        dlp.set_image_orientation( dlp.LEFT, flip_x=True, flip_y=False, rot_90=False )
        look_id, sequence_id, frame_rate_int = dlp.get_look(dlp.LEFT)
        dlp.set_look(dlp.LEFT, 1)
        CAIC_enabled = dlp.get_CAIC_enable(dlp.LEFT)
        dlp.set_CAIC_enable(dlp.LEFT, True)
        flip_x, flip_y, rot_90 = dlp.get_image_orientation(dlp.LEFT)
        dlp.set_input_source(dlp.LEFT, dlp.TEST_PATTERN_GENERATOR)
        print(dlp.get_input_source(dlp.LEFT))
        dlp.set_test_pattern(dlp.LEFT, dlp.COLOR_BARS)
        dlp.set_RGB_currents(dlp.LEFT, 200, 200, 200)
        print( dlp.get_RGB_currents(dlp.LEFT) )
        dlp.set_input_source(dlp.LEFT, dlp.EXTERNAL_VIDEO_PORT)
        frozen = dlp.get_image_freeze(dlp.LEFT)
        dlp.set_image_freeze(dlp.LEFT, freeze_enabled=True)
        frozen = dlp.get_image_freeze(dlp.LEFT)
        dlp.set_image_freeze(dlp.LEFT, freeze_enabled=False)
