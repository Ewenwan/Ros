#! /usr/bin/python
"""
test_frame.py

Paul Malmsten, 2010
pmalmsten@gmail.com

Tests frame module for proper behavior
"""
import unittest
from xbee.frame import APIFrame

class TestAPIFrameGeneration(unittest.TestCase):
    """
    XBee class must be able to create a valid API frame given binary
    data, in byte string form.
    """
    def test_single_byte(self):
        """
        create a frame containing a single byte
        """
        data = '\x00'
        # start byte, two length bytes, data byte, checksum
        expected_frame = '\x7E\x00\x01\x00\xFF'
        
        frame = APIFrame(data).output()
        self.assertEqual(frame, expected_frame)
        
class TestAPIFrameParsing(unittest.TestCase):
    """
    XBee class must be able to read and validate the data contained
    by a valid API frame.
    """

    def test_remaining_bytes(self):
        """
        remaining_bytes() should provide accurate indication
        of remaining bytes required before parsing a packet
        """
        api_frame = APIFrame()

        frame = '\x7E\x00\x04\x00\x00\x00\x00\xFF'
        self.assertEqual(api_frame.remaining_bytes(), 3)
        api_frame.fill(frame[0])
        self.assertEqual(api_frame.remaining_bytes(), 2)
        api_frame.fill(frame[1])
        self.assertEqual(api_frame.remaining_bytes(), 1)
        api_frame.fill(frame[2])
        self.assertEqual(api_frame.remaining_bytes(), 5)
        api_frame.fill(frame[3])
        self.assertEqual(api_frame.remaining_bytes(), 4)
    
    def test_single_byte(self):
        """
        read a frame containing a single byte
        """
        api_frame = APIFrame()

        frame = '\x7E\x00\x01\x00\xFF'
        expected_data = '\x00'
        
        for byte in frame:
            api_frame.fill(byte)
        api_frame.parse()

        self.assertEqual(api_frame.data, expected_data)
        
    def test_invalid_checksum(self):
        """
        when an invalid frame is read, an exception must be raised
        """
        api_frame = APIFrame()
        frame = '\x7E\x00\x01\x00\xF6'
        
        for byte in frame:
            api_frame.fill(byte)

        self.assertRaises(ValueError, api_frame.parse)

class TestEscapedOutput(unittest.TestCase):
    """
    APIFrame class must properly escape data
    """

    def test_escape_method(self):
        """
        APIFrame.escape() must work as expected
        """
        test_data = APIFrame.START_BYTE
        new_data = APIFrame.escape(test_data)
        self.assertEqual(new_data, APIFrame.ESCAPE_BYTE + '\x5e')
