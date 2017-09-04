"""
fake.py

By Paul Malmsten, 2010
pmalmsten@gmail.com

Provides fake objects for testing the dispatch package.
"""

class FakeXBee(object):
    """
    Represents an XBee device from which data can be read.
    """
    def __init__(self, data):
        self.data = data
        
    def wait_read_frame(self):
        return self.data
