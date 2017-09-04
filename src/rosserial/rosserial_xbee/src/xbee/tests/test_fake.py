#! /usr/bin/python
"""
test_xbee.py

By Paul Malmsten, 2010
pmalmsten@gmail.com

Tests fake device objects for proper functionality.
"""
import unittest
from xbee.tests.Fake import FakeReadDevice

class TestFakeReadDevice(unittest.TestCase):
    """
    FakeReadDevice class should work as intended to emluate a serial 
    port
    """
    def setUp(self):
        """
        Create a fake read device for each test
        """
        self.device = FakeReadDevice("test")
    
    def test_read_single_byte(self):
        """
        reading one byte at a time should work as expected
        """
        self.assertEqual(self.device.read(), 't')
        self.assertEqual(self.device.read(), 'e')
        self.assertEqual(self.device.read(), 's')
        self.assertEqual(self.device.read(), 't')
        
    def test_read_multiple_bytes(self):
        """
        reading multiple bytes at a time should work as expected
        """
        self.assertEqual(self.device.read(3), 'tes')
        self.assertEqual(self.device.read(), 't')
        
    def test_read_too_many(self):
        """
        attempting to read too many bytes should raise an exception
        """
        self.assertRaises(ValueError, self.device.read, 5)
