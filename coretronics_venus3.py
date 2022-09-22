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

        # Considering serial responses indicate command_id and side, 
        # the Venus may, in theory respond to queries out-of-order.
        # However, ACK after response indicates completion of command,
        # suggesting a simple response/reply (pull model) protocol
        # Setting up a producer/consumer model here, where 
        # responses are stored in a dictionary
        # key = [CMD_IDX], value =  [WG, PAYLOAD], where 
        #   CMD_IDX is a 2 digit hex string, 
        #   WG is a one digit ascii character '0', '1' or '2'
        #   PAYLOAD is a bytearray
        # !!! Asynchronous consumption is not implemented !!!
        self.responses = dict()

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
        while 1:
            c = self._read_line(eol=b'\0')
            response = c.decode()
            assert response[0] == '#', "First character of response [HEAD] expected to be '#'!" 
            assert response[1] != 'N', "Command NACK received!"
            if response[1]=='A': 
                if self._to_hex(cmd_id) in self.responses:
                    [wg, byte_list] = self.responses[self._to_hex(cmd_id)]
                    # May want to reconsider if returning a value for the wrong side is tolerable:
                    assert int(channel) == int(wg), ValueError("Channel query mismatch!")
                    self.responses.pop(self._to_hex(cmd_id))
                    return byte_list
                if expect_only_ACK: return
            else: 
                self._parse_response(response)
     
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
            self.responses[cmd] = [wg, payload]
        except:
            assert ValueError("Error parsing read response from {}!".format(self.PCBA_NAME))
        return

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
        """ Selects source for DLP(s)
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

    def get_RGB_currents(self, channel):
        """ Reads R,G,B LED currents from attached DLPs.
            parameters:
                channel: LEFT or RIGHT
            returns
                [R,G,B] list of uint16s, proportional to LED currents.
        """
        assert channel == self.LEFT or channel == self.RIGHT
        cmd_id = 0x55
        self._send_command(cmd_id, bytearray(), channel)
        byte_list = self._read_response(cmd_id, channel)
        R = (byte_list[1]*256 + byte_list[0])
        G = (byte_list[3]*256 + byte_list[2])
        B = (byte_list[5]*256 + byte_list[4])
        return [R,G,B]

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
        payload = bytes([R & 0xFF, R >> 8, G & 0xFF, G >> 8, B & 0xFF, B >> 8])
        self._send_command(cmd_id, payload, channel)
        void = self._read_response(cmd_id, channel, expect_only_ACK=True)        
        return void

# ---------------------------------------------------
# Demonstrate / test the coretronics_venus3 class
# ---------------------------------------------------
if __name__ == "__main__":
    dlp = coretronics_venus3()
    if dlp.isOpen:
        dlp.set_input_source(dlp.LEFT, dlp.TEST_PATTERN_GENERATOR)
        print(dlp.get_input_source(dlp.LEFT))
        dlp.set_test_pattern(dlp.LEFT, dlp.COLOR_BARS)
        dlp.set_RGB_currents(dlp.LEFT, 200, 200, 200)
        print( dlp.get_RGB_currents(dlp.LEFT) )
