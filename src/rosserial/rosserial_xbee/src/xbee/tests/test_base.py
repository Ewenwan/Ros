#! /usr/bin/python
"""
test_xbee.py

By Paul Malmsten, 2010
pmalmsten@gmail.com

Tests the XBeeBase superclass module for XBee API conformance.
"""
import unittest
from xbee.base import XBeeBase
from xbee.tests.Fake import FakeDevice, FakeReadDevice

class TestWriteToDevice(unittest.TestCase):
    """
    XBeeBase class should properly._write binary data in a valid API
    frame to a given serial device.
    """
    
    def test_write(self):
        """
        _write method should write the expected data to the serial
        device
        """
        device = FakeDevice()
        
        xbee = XBeeBase(device)
        xbee._write('\x00')
        
        # Check resuting state of fake device
        expected_frame = '\x7E\x00\x01\x00\xFF'
        self.assertEqual(device.data, expected_frame)
        
    def test_write_again(self):
        """
        _write method should write the expected data to the serial
        device
        """
        device = FakeDevice()
        
        xbee = XBeeBase(device)
        xbee._write('\x00\x01\x02')
        
        # Check resuting state of fake device
        expected_frame = '\x7E\x00\x03\x00\x01\x02\xFC'
        self.assertEqual(device.data, expected_frame)

    def test_write_escaped(self):
        """
        _write method should write the expected data to the serial
        device
        """
        device = FakeDevice()
        
        xbee = XBeeBase(device,escaped=True)
        xbee._write('\x7E\x01\x7D\x11\x13')
        
        # Check resuting state of fake device
        expected_frame = '\x7E\x00\x05\x7D\x5E\x01\x7D\x5D\x7D\x31\x7D\x33\xDF'
        self.assertEqual(device.data, expected_frame)
        
class TestReadFromDevice(unittest.TestCase):
    """
    XBeeBase class should properly read and extract data from a valid
    API frame
    """
    def test_read(self):
        """
        _wait_for_frame should properly read a frame of data
        """
        device = FakeReadDevice('\x7E\x00\x01\x00\xFF')
        xbee = XBeeBase(device)
        
        frame = xbee._wait_for_frame()
        self.assertEqual(frame.data, '\x00')
        
    def test_read_invalid_followed_by_valid(self):
        """
        _wait_for_frame should skip invalid data
        """
        device = FakeReadDevice(
            '\x7E\x00\x01\x00\xFA' + '\x7E\x00\x01\x05\xFA')
        xbee = XBeeBase(device)
        
        frame = xbee._wait_for_frame()
        self.assertEqual(frame.data, '\x05')

    def test_read_escaped(self):
        """
        _wait_for_frame should properly read a frame of data
        Verify that API mode 2 escaped bytes are read correctly
        """
        device = FakeReadDevice('\x7E\x00\x04\x7D\x5E\x7D\x5D\x7D\x31\x7D\x33\xE0')

        xbee = XBeeBase(device,escaped=True)
        
        frame = xbee._wait_for_frame()
        self.assertEqual(frame.data, '\x7E\x7D\x11\x13')
        
class TestNotImplementedFeatures(unittest.TestCase):
    """
    In order to properly use the XBeeBase class for most situations,
    it must be subclassed with the proper attributes definined. If
    this is not the case, then a NotImplemented exception should be
    raised as appropriate.
    """
    
    def setUp(self):
        """
        Set up a base class XBeeBase object which does not have 
        api_commands or api_responses defined
        """
        self.xbee = XBeeBase(None)
    
    def test_build_command(self):
        """
        _build_command should raise NotImplemented
        """
        self.assertRaises(NotImplementedError, self.xbee._build_command, "at")
        
    def test_split_response(self):
        """
        split_command should raise NotImplemented
        """
        self.assertRaises(NotImplementedError, self.xbee._split_response, "\00")
        
    def test_shorthand(self):
        """
        Shorthand calls should raise NotImplementedError
        """
        try:
            self.xbee.at
        except NotImplementedError:
            pass
        else:
            self.fail("Shorthand call on XBeeBase base class should raise NotImplementedError")
            
class TestAsyncCallback(unittest.TestCase):
    """
    XBeeBase constructor should accept an optional callback function 
    argument. When provided, this will put the module into a threaded
    mode, in which it will call the provided function with any API
    frame data received.
    
    As it would be very difficult to sanely test an asynchonous callback
    routine with a synchronous test process, proper callback behavior
    is not tested automatically at this time. Theoretically, the
    callback implementation logic is simple, but use it at your own risk.
    """
    
    def setUp(self):
        self.xbee = None
        self.serial = FakeReadDevice([], silent_on_empty=True)
        self.callback = lambda data: None

    def tearDown(self):
        # Ensure proper thread shutdown before continuing
        self.xbee.halt()
    
    def test_provide_callback(self):
        """
        XBeeBase constructor should accept a callback function
        """
        self.xbee = XBeeBase(self.serial, callback=self.callback)
        
class TestInitialization(unittest.TestCase):
    """
    Ensures that XBeeBase objects are properly constructed
    """

    def setUp(self):
        self.base = XBeeBase(None)

    def test_thread_always_initialized(self):
        """
        Even when a callback method is not supplied to the XBeeBase
        constructor, it must be properly initalized as a
        threading.Thread object
        """
        self.assertFalse(self.base.is_alive())

if __name__ == '__main__':
    unittest.main()
