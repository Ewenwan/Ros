"""
test_zigbee.py

By Paul Malmsten, 2010
pmalmsten@gmail.com

Tests the XBee ZB (ZigBee) implementation class for API compliance
"""
import unittest
from xbee.zigbee import ZigBee

class TestZigBee(unittest.TestCase):
    """
    Tests ZigBee-specific features
    """

    def setUp(self):
        self.zigbee = ZigBee(None)

    def test_null_terminated_field(self):
        """
        Packets with null-terminated fields
        should be properly parsed
        """
        expected_data = '\x01\x02\x03\x04'
        terminator = '\x00'
        node_identifier = '\x95' + '\x00' * 21 + expected_data + terminator + '\x00' * 8

        data = self.zigbee._split_response(node_identifier)

        self.assertEqual(data['node_id'], expected_data)

    def test_split_node_identification_identifier(self):
            data = '\x95\x00\x13\xa2\x00\x40\x52\x2b\xaa\x7d\x84\x02\x7d\x84\x00\x13\xa2\x00\x40\x52\x2b\xaa\x20\x00\xff\xfe\x01\x01\xc1\x05\x10\x1e'
            info = self.zigbee._split_response(data)
            expected_info = {
                'id': 'node_id_indicator',
                'sender_addr_long': '\x00\x13\xa2\x00\x40\x52\x2b\xaa',
                'sender_addr': '\x7d\x84',
                'options': '\x02',
                'source_addr': '\x7d\x84',
                'source_addr_long': '\x00\x13\xa2\x00\x40\x52\x2b\xaa',
                'node_id': ' ',
                'parent_source_addr': '\xff\xfe',
                'device_type': '\x01',
                'source_event': '\x01',
                'digi_profile_id': '\xc1\x05',
                'manufacturer_id': '\x10\x1e',
            }
            
            self.assertEqual(info, expected_info)
            
    def test_split_node_identification_identifier2(self):
            data = '\x95\x00\x13\xa2\x00\x40\x52\x2b\xaa\x7d\x84\x02\x7d\x84\x00\x13\xa2\x00\x40\x52\x2b\xaaCoordinator\x00\xff\xfe\x01\x01\xc1\x05\x10\x1e'
            info = self.zigbee._split_response(data)
            expected_info = {
                'id': 'node_id_indicator',
                'sender_addr_long': '\x00\x13\xa2\x00\x40\x52\x2b\xaa',
                'sender_addr': '\x7d\x84',
                'options': '\x02',
                'source_addr': '\x7d\x84',
                'source_addr_long': '\x00\x13\xa2\x00\x40\x52\x2b\xaa',
                'node_id': 'Coordinator',
                'parent_source_addr': '\xff\xfe',
                'device_type': '\x01',
                'source_event': '\x01',
                'digi_profile_id': '\xc1\x05',
                'manufacturer_id': '\x10\x1e',
            }
            
            self.assertEqual(info, expected_info)

class TestParseZigBeeIOData(unittest.TestCase):
    """
    Test parsing ZigBee specific IO data
    """

    def setUp(self):
        self.zigbee = ZigBee(None)

    def test_parse_dio_adc(self):
            data = '\x01\x08\x00\x0e\x08\x00\x00\x00\x02P\x02\x06'
            expected_results = [{'dio-11': True,
                                 'adc-1': 0,
                                 'adc-2': 592,
                                 'adc-3': 518}]
            results = self.zigbee._parse_samples(data)
            self.assertEqual(results, expected_results)
