"""
dispatch.py

By Paul Malmsten, 2010
pmalmsten@gmail.com

Provides the Dispatch class, which allows one to filter incoming data
packets from an XBee device and call an appropriate method when
one arrives.
"""

from xbee import XBee

class Dispatch(object):
    def __init__(self, ser=None, xbee=None):
        self.xbee = None
        if xbee:
            self.xbee = xbee
        elif ser:
            self.xbee = XBee(ser)
            
        self.handlers = []
        self.names = set()
            
    def register(self, name, callback, filter):
        """
        register: string, function: string, data -> None, function: data -> boolean -> None
        
        Register will save the given name, callback, and filter function
        for use when a packet arrives. When one arrives, the filter
        function will be called to determine whether to call its associated
        callback function. If the filter method returns true, the callback
        method will be called with its associated name string and the packet
        which triggered the call.
        """
        if name in self.names:
            raise ValueError("A callback has already been registered with the name '%s'" % name)
        
        self.handlers.append(
            {'name':name,
             'callback':callback,
             'filter':filter}
        )
        
        self.names.add(name)
        
    def run(self, oneshot=False):
        """
        run: boolean -> None
        
        run will read and dispatch any packet which arrives from the 
        XBee device
        """
        if not self.xbee:
            raise ValueError("Either a serial port or an XBee must be provided to __init__ to execute run()")
        
        while True:
            self.dispatch(self.xbee.wait_read_frame())
                    
            if oneshot:
                break
    
    def dispatch(self, packet):
        """
        dispatch: XBee data dict -> None
        
        When called, dispatch checks the given packet against each 
        registered callback method and calls each callback whose filter 
        function returns true.
        """
        for handler in self.handlers:
            if handler['filter'](packet):
                # Call the handler method with its associated
                # name and the packet which passed its filter check
                handler['callback'](handler['name'], packet)
