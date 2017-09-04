#! /usr/bin/python
"""
test_ieee.py

By Paul Malmsten, 2010
pmalmsten@gmail.com

Tests the XBee (IEEE 802.15.4) implementation class for XBee API compliance
"""
import unittest
from xbee.tests.Fake import FakeDevice, FakeReadDevice
from xbee.ieee import XBee

class InitXBee(unittest.TestCase):
    """
    Base initalization class
    """
    def setUp(self):
        """
        Initialize XBee object
        """
        self.xbee = XBee(None)

class TestBuildCommand(InitXBee):
    """
    _build_command should properly build a command packet
    """
    
    def test_build_at_data_mismatch(self):
        """
        if not enough or incorrect data is provided, an exception should
        be raised.
        """
        try:
            self.xbee._build_command("at")
        except KeyError:
            # Test passes
            return
    
        # No exception? Fail.
        self.fail(
            "An exception was not raised with improper data supplied"
        )
        
    def test_build_at_data_len_mismatch(self):
        """
        if data of incorrect length is provided, an exception should be 
        raised
        """
        try:
            self.xbee._build_command("at", frame_id="AB", command="MY")
        except ValueError:
            # Test passes
            return
    
        # No exception? Fail.
        self.fail(
            "An exception was not raised with improper data length"
        )
        
    def test_build_at(self):
        """
        _build_command should build a valid at command packet which has
        no parameter data to be saved
        """
        
        at_command = "MY"
        frame = chr(43)
        data = self.xbee._build_command(
            "at", 
            frame_id=frame, 
            command=at_command
        ) 

        expected_data = '\x08+MY'
        self.assertEqual(data, expected_data)
        
    def test_build_at_with_default(self):
        """
        _build_command should build a valid at command packet which has
        no parameter data to be saved and no frame specified (the 
        default value of \x00 should be used)
        """
        
        at_command = "MY"
        data = self.xbee._build_command("at", command=at_command) 

        expected_data = '\x08\x00MY'
        self.assertEqual(data, expected_data)
        
class TestSplitResponse(InitXBee):
    """
    _split_response should properly split a response packet
    """
    
    def test_unrecognized_response(self):
        """
        if a response begins with an unrecognized id byte, 
        _split_response should raise an exception
        """
        data = '\x23\x00\x00\x00'
        
        try:
            self.xbee._split_response(data)
        except KeyError:
            # Passes
            return
            
        # Test Fails
        self.fail()
    
    def test_bad_data_long(self):
        """
        if a response doesn't match the specification's layout, 
        _split_response should raise an exception
        """
        # Over length
        data = '\x8a\x00\x00\x00'
        self.assertRaises(ValueError, self.xbee._split_response, data)
        
    def test_bad_data_short(self):
        """
        if a response doesn't match the specification's layout, 
        _split_response should raise an exception
        """
        # Under length
        data = '\x8a'
        self.assertRaises(ValueError, self.xbee._split_response, data)
    
    def test_split_status_response(self):
        """
        _split_response should properly split a status response packet
        """
        data = '\x8a\x01'
        
        info = self.xbee._split_response(data)
        expected_info = {'id':'status',
                         'status':'\x01'}
        
        self.assertEqual(info, expected_info)
        
    def test_split_short_at_response(self):
        """
        _split_response should properly split an at_response packet which
        has no parameter data
        """
        
        data = '\x88DMY\x01'
        info = self.xbee._split_response(data)
        expected_info = {'id':'at_response',
                         'frame_id':'D',
                         'command':'MY',
                         'status':'\x01'}
        self.assertEqual(info, expected_info)
        
    def test_split_at_resp_with_param(self):
        """
        _split_response should properly split an at_response packet which
        has parameter data
        """
        
        data = '\x88DMY\x01ABCDEF'
        info = self.xbee._split_response(data)
        expected_info = {'id':'at_response',
                         'frame_id':'D',
                         'command':'MY',
                         'status':'\x01',
                         'parameter':'ABCDEF'}
        self.assertEqual(info, expected_info)
        
        
class TestParseIOData(InitXBee):
    """
    XBee class should properly parse IO data received from an XBee 
    device
    """
    
    def test_parse_single_dio(self):
        """
        _parse_samples should properly parse a packet containing a single 
        sample of only digital io data
        """
        # One sample, ADC disabled and DIO8 enabled, DIO 0-7 enabled
        header = '\x01\x01\xFF'
        
        # First 7 bits ignored, DIO8 high, DIO 0-7 high
        sample = '\x01\xFF'
        data = header + sample
        
        expected_results = [{'dio-0':True,
                             'dio-1':True,
                             'dio-2':True,
                             'dio-3':True,
                             'dio-4':True,
                             'dio-5':True,
                             'dio-6':True,
                             'dio-7':True,
                             'dio-8':True}]
                             
        results = self.xbee._parse_samples(data)
        
        self.assertEqual(results, expected_results)
        
    def test_parse_single_dio_again(self):
        """
        _parse_samples should properly parse a packet containing a single 
        sample of only digital io data, which alternates between on and 
        off
        """
        # One sample, ADC disabled and DIO8 enabled, DIO 0-7 enabled
        header = '\x01\x01\xFF'
        
        # First 7 bits ignored, DIO8 low, DIO 0-7 alternating
        sample = '\x00\xAA'
        data = header + sample
        
        expected_results = [{'dio-0':False,
                             'dio-1':True,
                             'dio-2':False,
                             'dio-3':True,
                             'dio-4':False,
                             'dio-5':True,
                             'dio-6':False,
                             'dio-7':True,
                             'dio-8':False}]
                             
        results = self.xbee._parse_samples(data)
        
        self.assertEqual(results, expected_results)
        
    def test_parse_single_dio_subset(self):
        """
        _parse_samples should properly parse a packet containing a single 
        sample of only digital io data for only a subset of the 
        available pins
        """
        # One sample, ADC disabled
        # DIO 1,3,5,7 enabled
        header = '\x01\x00\xAA'
        
        # First 7 bits ignored, DIO8 low, DIO 0-7 alternating
        sample = '\x00\xAA'
        data = header + sample
        
        expected_results = [{'dio-1':True,
                             'dio-3':True,
                             'dio-5':True,
                             'dio-7':True}]
                             
        results = self.xbee._parse_samples(data)
        
        self.assertEqual(results, expected_results)
        
    def test_parse_single_dio_subset_again(self):
        """
        _parse_samples should properly parse a packet containing a single 
        sample of only digital io data for only a subset of the 
        available pins
        """
        # One sample, ADC disabled
        # DIO 0 enabled
        header = '\x01\x00\x01'
        
        # First 7 bits ignored, DIO8 low, DIO 0-7 alternating
        sample = '\x00\xAA'
        data = header + sample
        
        expected_results = [{'dio-0':False}]
                             
        results = self.xbee._parse_samples(data)
        
        self.assertEqual(results, expected_results)
        
    def test_parse_multiple_dio_subset(self):
        """
        _parse_samples should properly parse a packet containing two 
        samples of only digital io data for one dio line
        """
        # Two samples, ADC disabled
        # DIO 0 enabled
        header = '\x02\x00\x01'
        
        # First 7 bits ignored, DIO8 low, DIO 0-7 alternating
        sample = '\x00\xAA' + '\x00\x01'
        data = header + sample
        
        expected_results = [{'dio-0':False},
                            {'dio-0':True}]
                             
        results = self.xbee._parse_samples(data)
        
        self.assertEqual(results, expected_results)
        
    def test_parse_multiple_dio(self):
        """
        _parse_samples should properly parse a packet containing three 
        samples of only digital io data
        """
        # Three samples, ADC disabled and DIO8 enabled, DIO 0-7 enabled
        header = '\x03\x01\xFF'
        
        # First 7 bits ignored
        # First sample: all bits on
        # Second sample: alternating bits on
        # Third sample: all bits off
        sample = '\x01\xFF' + '\x00\xAA' + '\x00\x00'
        data = header + sample
        
        expected_results = [{'dio-0':True,
                             'dio-1':True,
                             'dio-2':True,
                             'dio-3':True,
                             'dio-4':True,
                             'dio-5':True,
                             'dio-6':True,
                             'dio-7':True,
                             'dio-8':True},
                             {'dio-0':False,
                             'dio-1':True,
                             'dio-2':False,
                             'dio-3':True,
                             'dio-4':False,
                             'dio-5':True,
                             'dio-6':False,
                             'dio-7':True,
                             'dio-8':False},
                             {'dio-0':False,
                             'dio-1':False,
                             'dio-2':False,
                             'dio-3':False,
                             'dio-4':False,
                             'dio-5':False,
                             'dio-6':False,
                             'dio-7':False,
                             'dio-8':False}]
                             
        results = self.xbee._parse_samples(data)
        
        self.assertEqual(results, expected_results)
        
    def test_parse_multiple_adc_subset(self):
        """
        _parse_samples should parse a data packet containing multiple
        samples of adc data from multiple pins in the proper order
        """
        # One sample, ADC 0,1 enabled
        # DIO disabled
        header = '\x02\x06\x00'
        
        # No dio data
        # ADC0 value of 0
        # ADC1 value of 255
        # ADC0 value of 5
        # ADC1 value of 7
        sample = '\x00\x00' + '\x00\xFF' + '\x00\x05' + '\x00\x07'
        data = header + sample
        
        expected_results = [{'adc-0':0,
                             'adc-1':255},
                            {'adc-0':5,
                             'adc-1':7}]
        
        results = self.xbee._parse_samples(data)
        
        self.assertEqual(results, expected_results)
        
    def test_parse_single_dio_adc_subset(self):
        """
        _parse_samples should properly parse a packet containing a single 
        sample of digital and analog io data for only a subset of the 
        available pins
        """
        # One sample, ADC 0 enabled
        # DIO 1,3,5,7 enabled
        header = '\x01\x02\xAA'
        
        # First 7 bits ignored, DIO8 low, DIO 0-7 alternating
        # ADC0 value of 255
        sample = '\x00\xAA\x00\xFF'
        data = header + sample
        
        expected_results = [{'dio-1':True,
                             'dio-3':True,
                             'dio-5':True,
                             'dio-7':True,
                             'adc-0':255}]
                             
        results = self.xbee._parse_samples(data)
        
        self.assertEqual(results, expected_results)

class TestWriteToDevice(unittest.TestCase):
    """
    XBee class should properly write binary data in a valid API
    frame to a given serial device, including a valid command packet.
    """
    
    def test_send_at_command(self):
        """
        calling send should write a full API frame containing the
        API AT command packet to the serial device.
        """
        
        serial_port = FakeDevice()
        xbee = XBee(serial_port)
        
        # Send an AT command
        xbee.send('at', frame_id='A', command='MY')
        
        # Expect a full packet to be written to the device
        expected_data = '\x7E\x00\x04\x08AMY\x10'
        self.assertEqual(serial_port.data, expected_data)
        
        
    def test_send_at_command_with_param(self):
        """
        calling send should write a full API frame containing the
        API AT command packet to the serial device.
        """
        
        serial_port = FakeDevice()
        xbee = XBee(serial_port)
        
        # Send an AT command
        xbee.send(
            'at', 
            frame_id='A', 
            command='MY', 
            parameter='\x00\x00'
        )
        
        # Expect a full packet to be written to the device
        expected_data = '\x7E\x00\x06\x08AMY\x00\x00\x10'
        self.assertEqual(serial_port.data, expected_data)
        
class TestSendShorthand(unittest.TestCase):
    """
    Tests shorthand for sending commands to an XBee provided by
    XBee.__getattr__
    """
    
    def setUp(self):
        """
        Prepare a fake device to read from
        """
        self.ser = FakeDevice()
        self.xbee = XBee(self.ser)
    
    def test_send_at_command(self):
        """
        Send an AT command with a shorthand call
        """
        # Send an AT command
        self.xbee.at(frame_id='A', command='MY')
        
        # Expect a full packet to be written to the device
        expected_data = '\x7E\x00\x04\x08AMY\x10'
        self.assertEqual(self.ser.data, expected_data)
        
    def test_send_at_command_with_param(self):
        """
        calling send should write a full API frame containing the
        API AT command packet to the serial device.
        """
        
        # Send an AT command
        self.xbee.at(frame_id='A', command='MY', parameter='\x00\x00')
        
        # Expect a full packet to be written to the device
        expected_data = '\x7E\x00\x06\x08AMY\x00\x00\x10'
        self.assertEqual(self.ser.data, expected_data)
        
    def test_shorthand_disabled(self):
        """
        When shorthand is disabled, any attempt at calling a 
        non-existant attribute should raise AttributeError
        """
        self.xbee = XBee(self.ser, shorthand=False)
        
        try:
            self.xbee.at
        except AttributeError:
            pass
        else:
            self.fail("Specified shorthand command should not exist")

class TestReadFromDevice(unittest.TestCase):
    """
    XBee class should properly read and parse binary data from a serial 
    port device.
    """
    def test_read_at(self):
        """
        read and parse a parameterless AT command
        """
        device = FakeReadDevice('\x7E\x00\x05\x88DMY\x01\x8c')
        xbee = XBee(device)
        
        info = xbee.wait_read_frame()
        expected_info = {'id':'at_response',
                         'frame_id':'D',
                         'command':'MY',
                         'status':'\x01'}
        self.assertEqual(info, expected_info)
        
    def test_read_at_params(self):
        """
        read and parse an AT command with a parameter
        """
        device = FakeReadDevice(
            '\x7E\x00\x08\x88DMY\x01\x00\x00\x00\x8c'
        )
        xbee = XBee(device)
        
        info = xbee.wait_read_frame()
        expected_info = {'id':'at_response',
                         'frame_id':'D',
                         'command':'MY',
                         'status':'\x01',
                         'parameter':'\x00\x00\x00'}
        self.assertEqual(info, expected_info)
        
    def test_read_io_data(self):
        """
        XBee class should properly read and parse incoming IO data
        """
        ## Build IO data
        # One sample, ADC 0 enabled
        # DIO 1,3,5,7 enabled
        header = '\x01\x02\xAA'
        
        # First 7 bits ignored, DIO8 low, DIO 0-7 alternating
        # ADC0 value of 255
        sample = '\x00\xAA\x00\xFF'
        data = header + sample
        
        ## Wrap data in frame
        # RX frame data
        rx_io_resp = '\x83\x00\x01\x28\x00'
    
        device = FakeReadDevice(
            '\x7E\x00\x0C'+ rx_io_resp + data + '\xfd'
        )
        xbee = XBee(device)
        
        info = xbee.wait_read_frame()
        expected_info = {'id':'rx_io_data',
                         'source_addr':'\x00\x01',
                         'rssi':'\x28',
                         'options':'\x00',
                         'samples': [{'dio-1':True,
                                      'dio-3':True,
                                      'dio-5':True,
                                      'dio-7':True,
                                      'adc-0':255}]
                        }
        self.assertEqual(info, expected_info)
                    

if __name__ == '__main__':
    unittest.main()
