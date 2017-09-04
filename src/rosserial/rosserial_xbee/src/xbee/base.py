"""
xbee.py

By Paul Malmsten, 2010
Inspired by code written by Amit Synderman and Marco Sangalli
pmalmsten@gmail.com

   _wait_for_frame modified by Adam Stambler to allow for non
   blocking io  
   Adam Stambler, 2011

XBee superclass module


This class defines data and methods common to all XBee modules. 
This class should be subclassed in order to provide
series-specific functionality.
"""
import struct, threading, time
from xbee.frame import APIFrame

class ThreadQuitException(Exception):
    pass

class XBeeBase(threading.Thread):
    """
    Abstract base class providing command generation and response
    parsing methods for XBee modules.
    
    Constructor arguments:
        ser:    The file-like serial port to use.


        shorthand: boolean flag which determines whether shorthand command 
                   calls (i.e. xbee.at(...) instead of xbee.send("at",...) 
                   are allowed.

        callback: function which should be called with frame data
                  whenever a frame arrives from the serial port.
                  When this is not None, a background thread to monitor
                  the port and call the given function is automatically
                  started.

        escaped: boolean flag which determines whether the library should
                 operate in escaped mode. In this mode, certain data bytes
                 in the output and input streams will be escaped and unescaped
                 in accordance with the XBee API. This setting must match
                 the appropriate api_mode setting of an XBee device; see your
                 XBee device's documentation for more information.
    """
                       
    def __init__(self, ser, shorthand=True, callback=None, escaped=False):
        super(XBeeBase, self).__init__()
        self.serial = ser
        self.shorthand = shorthand
        self._callback = None
        self._thread_continue = False
        self._escaped = escaped  
        
        if callback:
            self._callback = callback
            self._thread_continue = True
            self._thread_quit = threading.Event()
            self.start()

    def halt(self):
        """
        halt: None -> None

        If this instance has a separate thread running, it will be
        halted. This method will wait until the thread has cleaned
        up before returning.
        """
        if self._callback:
            self._thread_continue = False
            self._thread_quit.wait()
        
    def _write(self, data):
        """
        _write: binary data -> None
        
        Packages the given binary data in an API frame and writes the 
        result to the serial port
        """
        frame = APIFrame(data, self._escaped).output()
        self.serial.write(frame)
        
    def run(self):
        """
        run: None -> None

        This method overrides threading.Thread.run() and is automatically
        called when an instance is created with threading enabled.
        """
        while True:
            try:
                self._callback(self.wait_read_frame())
            except ThreadQuitException:
                break
        self._thread_quit.set()
    
    def _wait_for_frame(self):
        """
        _wait_for_frame: None -> binary data
        
        _wait_for_frame will read from the serial port until a valid
        API frame arrives. It will then return the binary data
        contained within the frame.

        If this method is called as a separate thread
        and self.thread_continue is set to False, the thread will
        exit by raising a ThreadQuitException.
        """
        frame = APIFrame(escaped=self._escaped)
        mode = 0
        while True:
                if self._callback and not self._thread_continue:
                    raise ThreadQuitException
                
                while (  self.serial.inWaiting() <1):
                    time.sleep(0.01)
                byte = self.serial.read()
                if byte =='':
                    continue 

                if (mode ==0):
                    if byte == APIFrame.START_BYTE:
                        mode=1
                    else:
                        continue

                frame.fill(byte)
                
                if ( (mode==1) and (frame.remaining_bytes() <=0) ) :
                    try:
                        # Try to parse and return result
                        frame.parse()
                        mode =0
                        return frame
                    except ValueError:
                        # Bad frame, so restart
                        mode=0
                        frame = APIFrame(escaped=self._escaped)
                        
    def _build_command(self, cmd, **kwargs):
        """
        _build_command: string (binary data) ... -> binary data
        
        _build_command will construct a command packet according to the
        specified command's specification in api_commands. It will expect
        named arguments for all fields other than those with a default 
        value or a length of 'None'.
        
        Each field will be written out in the order they are defined
        in the command definition.
        """
        try:
            cmd_spec = self.api_commands[cmd]
        except AttributeError:
            raise NotImplementedError("API command specifications could not be found; use a derived class which defines 'api_commands'.")
            
        packet = ''
        
        for field in cmd_spec:
            try:
                # Read this field's name from the function arguments dict
                data = kwargs[field['name']]
            except KeyError:
                # Data wasn't given
                # Only a problem if the field has a specific length
                if field['len'] is not None:
                    # Was a default value specified?
                    default_value = field['default']
                    if default_value:
                        # If so, use it
                        data = default_value
                    else:
                        # Otherwise, fail
                        raise KeyError(
                            "The expected field %s of length %d was not provided" 
                            % (field['name'], field['len']))
                else:
                    # No specific length, ignore it
                    data = None
            
            # Ensure that the proper number of elements will be written
            if field['len'] and len(data) != field['len']:
                raise ValueError(
                    "The data provided for '%s' was not %d bytes long"\
                    % (field['name'], field['len']))
        
            # Add the data to the packet, if it has been specified
            # Otherwise, the parameter was of variable length, and not 
            #  given
            if data:
                packet += data
                
        return packet
    
    def _split_response(self, data):
        """
        _split_response: binary data -> {'id':str,
                                         'param':binary data,
                                         ...}
                                        
        _split_response takes a data packet received from an XBee device
        and converts it into a dictionary. This dictionary provides
        names for each segment of binary data as specified in the 
        api_responses spec.
        """
        # Fetch the first byte, identify the packet
        # If the spec doesn't exist, raise exception
        packet_id = data[0]
        try:
            packet = self.api_responses[packet_id]
        except AttributeError:
            raise NotImplementedError("API response specifications could not be found; use a derived class which defines 'api_responses'.")
        except KeyError:
            raise KeyError(
                "Unrecognized response packet with id byte %s"
                % data[0])
        
        # Current byte index in the data stream
        index = 1
        
        # Result info
        info = {'id':packet['name']}
        packet_spec = packet['structure']
        
        # Parse the packet in the order specified
        for field in packet_spec:
            if field['len'] == 'null_terminated':
                field_data = ''
                
                while data[index] != '\x00':
                    field_data += data[index]
                    index += 1
                
                index += 1
                info[field['name']] = field_data
            elif field['len'] is not None:
                # Store the number of bytes specified

                # Are we trying to read beyond the last data element?
                if index + field['len'] > len(data):
                    raise ValueError(
                        "Response packet was shorter than expected")
                
                field_data = data[index:index + field['len']]
                info[field['name']] = field_data
                               
                index += field['len']
            # If the data field has no length specified, store any
            #  leftover bytes and quit
            else:
                field_data = data[index:]
                
                # Were there any remaining bytes?
                if field_data:
                    # If so, store them
                    info[field['name']] = field_data
                    index += len(field_data)
                break
            
        # If there are more bytes than expected, raise an exception
        if index < len(data):
            raise ValueError(
                "Response packet was longer than expected; expected: %d, got: %d bytes" % (index, 
                                                                                           len(data)))
                
        # Check if this packet was an IO sample
        # If so, process the sample data
        if 'parse_as_io_samples' in packet:
            field_to_process = packet['parse_as_io_samples']
            info[field_to_process] = self._parse_samples(
                                        info[field_to_process])
            
        return info
        
    def _parse_samples_header(self, io_bytes):
        """
        _parse_samples_header: binary data in XBee IO data format ->
                        (int, [int ...], [int ...], int, int)
                        
        _parse_samples_header will read the first three bytes of the 
        binary data given and will return the number of samples which
        follow, a list of enabled digital inputs, a list of enabled
        analog inputs, the dio_mask, and the size of the header in bytes
        """
        header_size = 3
        
        # number of samples (always 1?) is the first byte
        sample_count = ord(io_bytes[0])
        
        # part of byte 1 and byte 2 are the DIO mask ( 9 bits )
        dio_mask = (ord(io_bytes[1]) << 8 | ord(io_bytes[2])) & 0x01FF
        
        # upper 7 bits of byte 1 is the AIO mask
        aio_mask = (ord(io_bytes[1]) & 0xFE) >> 1
        
        # sorted lists of enabled channels; value is position of bit in mask
        dio_chans = []
        aio_chans = []
        
        for i in range(0,9):
            if dio_mask & (1 << i):
                dio_chans.append(i)
        
        dio_chans.sort()
        
        for i in range(0,7):
            if aio_mask & (1 << i):
                aio_chans.append(i)
        
        aio_chans.sort()
            
        return (sample_count, dio_chans, aio_chans, dio_mask, header_size)
        
    def _parse_samples(self, io_bytes):
        """
        _parse_samples: binary data in XBee IO data format ->
                        [ {"dio-0":True,
                           "dio-1":False,
                           "adc-0":100"}, ...]
                           
        _parse_samples reads binary data from an XBee device in the IO
        data format specified by the API. It will then return a 
        dictionary indicating the status of each enabled IO port.
        """

        sample_count, dio_chans, aio_chans, dio_mask, header_size = \
            self._parse_samples_header(io_bytes)
        
        samples = []
        
        # split the sample data into a list, so it can be pop()'d
        sample_bytes = [ord(c) for c in io_bytes[header_size:]]
        
        # repeat for every sample provided
        for sample_ind in range(0, sample_count):
            tmp_samples = {}
            
            if dio_chans:
                # we have digital data
                digital_data_set = (sample_bytes.pop(0) << 8 | sample_bytes.pop(0))
                digital_values = dio_mask & digital_data_set
                
                for i in dio_chans:
                    tmp_samples['dio-%d' % i] = True if (digital_values >> i) & 1 else False
                        
            for i in aio_chans:
                # only first 10 bits are significant
                analog_sample = (sample_bytes.pop(0) << 8 | sample_bytes.pop(0)) & 0x03FF
                tmp_samples['adc-%d' % i] = analog_sample
            
            samples.append(tmp_samples)
        
        return samples
        
    def send(self, cmd, **kwargs):
        """
        send: string param=binary data ... -> None
        
        When send is called with the proper arguments, an API command
        will be written to the serial port for this XBee device
        containing the proper instructions and data.
        
        This method must be called with named arguments in accordance
        with the api_command specification. Arguments matching all 
        field names other than those in reserved_names (like 'id' and
        'order') should be given, unless they are of variable length 
        (of 'None' in the specification. Those are optional).
        """
        # Pass through the keyword arguments
        self._write(self._build_command(cmd, **kwargs))
        
        
    def wait_read_frame(self):
        """
        wait_read_frame: None -> frame info dictionary
        
        wait_read_frame calls XBee._wait_for_frame() and waits until a
        valid frame appears on the serial port. Once it receives a frame,
        wait_read_frame attempts to parse the data contained within it
        and returns the resulting dictionary
        """
        
        frame = self._wait_for_frame()
        return self._split_response(frame.data)
        
    def __getattr__(self, name):
        """
        If a method by the name of a valid api command is called,
        the arguments will be automatically sent to an appropriate
        send() call
        """

        # If api_commands is not defined, raise NotImplementedError\
        #  If its not defined, _getattr__ will be called with its name
        if name == 'api_commands':
            raise NotImplementedError("API command specifications could not be found; use a derived class which defines 'api_commands'.")
        
        # Is shorthand enabled, and is the called name a command?
        if self.shorthand and name in self.api_commands:
            # If so, simply return a function which passes its arguments
            # to an appropriate send() call
            return lambda **kwargs: self.send(name, **kwargs)
        else:
            raise AttributeError("XBee has no attribute '%s'" % name)
