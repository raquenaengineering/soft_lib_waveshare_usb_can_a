
import time

import logging
logging.basicConfig(level = logging.DEBUG)

import serial

import threading


from .config import config


class usb_can_a():

    serial_port = None
    serial_port_name = None

    serial_buffer = []                          # here we store all read data received from serial port for further processing.
    frame_buffer = []                           # frames will be stored in this buffer, and popped out of it when requested by the right ID

    # HEADER = 0xAA
    HEADER = b'\xaa'
    # FOOTER = 0x55
    FOOTER = b'U'

    # def receive_message(self):
    #     print(self.receive_message.__name__)

    # def send_message(self):
    #     print(self.receive_message.__name__)



    def __init__(self, serial_port_name = None):
        """
        Constructor, on creation, config_serial_port() method is called.
        """
        self.serial_port_name = serial_port_name
        self.config_serial_port(self.serial_port_name)
        self.start_reception_thread()


    def config_serial_port(self, serial_port_name=None):
        """
        Configures, initializes and opens the serial port of the serial to CAN converter.
        :param serial_port_name: the name of the serial port to be used ("COMxx" or "dev/ttyxx"), if no name is given, it defaults to config.py file.
        :return: Always True
        """

        if serial_port_name == None:
            self.serial_port_name = config.serial_port_name

        try:
            self.serial_port = serial.Serial(self.serial_port_name)
            self.serial_port.timeout = config.serial_timeout
        except:
            logging.warning("Tried to open serial port, but was already open")
        else:
            self.serial_port.baudrate = config.serial_speed
            # self.serial_port.open()
            if self.serial_port.isOpen() == False:
                self.serial_port.open()


        return(True)

    def config_can_bus(self, can_speed=None):
        """
        Configures the CAN bus, sending a specially formatted dataframe via serial: Besides the can speed, the rest
        of the parameters (packet size, standard or extended...)are kept as default. Edit this method if needed enhanced functionality.

        Note: the 0xAA and the 0x55 should be replaced by the HEADER and FOOTER definitions, but care needs to be taken (bytes to int conversion )
        :return:
        :param can_speed: Speed to be used for the CAN communication, not to be confused with the serial_speed! please refer to the documentation for the available values.
        :return:
        """
        can_config_frame = [0xAA,  # 0xaa, 0.header
                            0x55,  # 0x55, 1.means config message?
                            0x02,  # 0x02, 2.fixed packet size
                            0x07,  # 0x05, 3.CAN bus speed --> channge to 0x09 and see if sth happens.
                            0x01,  # 0x01, 4.standard CAN frame
                            0x00,  # 5. filter ID
                            0x00,  # 6. filter ID
                            0x00,  # 7. filter ID
                            0x00,  # 8. filter ID
                            0x00,  # 9. mask ID
                            0x00,  # 10. mask ID
                            0x00,  # 11. mask ID
                            0x00,  # 12. mask ID
                            0x00,  # 13. 0x00, CAN mode normal (whatever that means)
                            0x01,  # 14. 0x01, automatic retransmission
                            0x00,  # 15. spare
                            0x00,  # 16. spare
                            0x00,  # 17. spare
                            0x00]  # 18. spare
        checksum = self.calculate_config_frame_checksum(can_config_frame[2:19])
        can_config_frame.append(checksum)  # checksum of previous data
        can_config_frame.append(0x55)  # footer

        # Send new hex data
        can_config_frame = bytes(can_config_frame)
        logging.debug(can_config_frame)
        self.serial_port.write(can_config_frame)

        logging.debug("Config Finished")
        # time.sleep(1)

    def send_can_frame(self, dev_id, data, type):
        """
        :param dev_id: ID of the device where the dataframe will be sent
        :param data: Data to be sent to the device with the given ID
        :param type: "standard" or "extended" frame, check CAN documentation for further details.
        :return: Always True
        """


        byte_type = 0b11000000                              # highest two bits seem to need to be high
        if(type == "extended"):
            byte_type = byte_type + 1 << 5                  # bit 5 of the byte configuring type is the one controlling if the frame is extended or standard.

        datalen = len(data)                                 # variable length messages need to add the length of the data in the byte_type
        byte_type = byte_type + datalen
        logging.debug("lenght of the data to be used: " + str(datalen))

        id_high = dev_id >> 8
        id_low = dev_id & 0xff00 >> 8

        can_frame = bytearray()
        can_frame.append(0xAA)                      # header
        can_frame.append(byte_type)                 # determines lenght of frame and frame type
        can_frame.append(id_low)                    # low byte of the id
        can_frame.append(id_high)                   # high byte of the id

        for byte in data:
            can_frame.append(byte)
            print(byte)

        can_frame.append(0x55)

        can_frame = bytes(can_frame)

        # print("SEND CAN FRAME WITH SERIAL HEADER AND FOOTER: ")
        # print(can_frame)


        self.serial_port.write(bytes(can_frame))

        return(True)

    def start_reception_thread(self):
        """
        Starts a thread which will continously read the serial port,
        and store whatever comes to a buffer.
        :return: True
        """
        print("start_reception_thread() method called")
        self.serial_reception_thread =threading.Thread(target=self.read_serial_data_in_loop)                                            # I would prefer to recycle the same thread, but dunno how to.
        self.stop_reception_thread_flag = False
        self.serial_reception_thread.start()

    def stop_reception_thread(self):
        print("stop_reception_thread() method called")
        self.stop_reception_thread_flag = True
        # print("self.stop_reception_thread_flag")
        # print(self.stop_reception_thread_flag)

    def read_serial_data_in_loop(self):
        """
        This method will read incoming data from serial and store it in the incoming serial data buffer
        :return: True
        """
        print("read_serial_data_in_loop() method called")

        while(self.stop_reception_thread_flag == False):
            # print("self.stop_reception_thread_flag")
            # print(self.stop_reception_thread_flag)
            # print("reading serial data")
            b = self.serial_port.read(1)  # read one single byte
            self.serial_buffer.append(b)

        return(True)

    def get_complete_can_frame_from_serial_buffer(self):
        """
        Checks if there is a complete CAN frame at the bytes contained at the serial buffer,
        if there is a complete CAN frame, it stores it in the frame_buffer
        :return:    False if no complete complete can frame was found on serial buffer
                    True if complete frame was found
        """

        print("get_complete_can_frame_from_serial_buffer() method called")

        # first make sure we have a complete message #
        has_header = False
        has_footer = False
        can_frame = []

        for i in range(len(self.serial_buffer)):
            if(self.serial_buffer[i] == self.HEADER):
                # print("found header")
                has_header = True
                header_pos = i
            if(has_header):
                if(self.serial_buffer[i] == self.FOOTER):
                    # print("found footer")
                    has_footer = True
                    footer_pos = i

        if(has_header and has_footer):

            frame = self.serial_buffer[header_pos+1:footer_pos]                                                         # this already removes the header and the footer bytes from the frame
            self.serial_buffer = self.serial_buffer[footer_pos+1:]                                                      # remove already read frame from serial buffer

            # print("Frame:")
            # print(frame)
            # print("self.serial_buffer after removing frame")
            # print(self.serial_buffer)

            self.frame_buffer.append(frame)

        return(False)

    def receive_can_frame(self):
        """
        Receives a CAN frame FROM ANY SLAVE IN THE BUS, of a maximum length of 20bytes,
        :return: The received CAN frame, without the header(0xAA) and the footer byte (0x55)
        used by the Serial to CAN adapter to determine start and end of CAN frame.
        Note: deterimining which slave sent the frame may need to be implemented in a higher level method.
        !!! --> header and footer seem to still be there, REMOVE AND ADAPT CODE WHICH USES THIS METHOD !!!
        """
        frame_len_max = 20

        print("receive_can_frame() method called")

        self.get_complete_can_frame_from_serial_buffer()
        if(self.frame_buffer):
            return(self.frame_buffer.pop(0))
        else:
            return(None)

    def receive_can_frame_from(self, id):
        """
        Receives a frame from a slave with a specific ID
        :param id: the specific ID to receive the frame from.
        :return: Returns the oldest frame saved at the CAN frame buffer for the given id
        """

        frame = None

        # check if there is something in the frame buffer #
        if(len(self.frame_buffer) != 0):
            print("checking what it's already in the frame buffer")
        else:


            # get a frame #
            frame = self.receive_can_frame()


            # check if the frame size is correct #
            pass

            # divide the frame between data and frame_id #
            print("Received full frame including ID")
            print(frame)
            msb = int.from_bytes(frame[2], byteorder="little") << 8                                                         # this can be done with bytearray, but didn't manage to do it properly
            lsb = int.from_bytes(frame[1], byteorder = "little")
            print("msb: " + str(msb) + " lsb: " + str(lsb))
            received_id = msb + lsb
            print("received_id")
            print(hex(received_id))

            if(received_id == id):
                print("received frame id matches with requested id")

        return(frame)


    def calculate_config_frame_checksum(self, data):
        """
        Required to calculate the checksum OF THE CONFIGURATION MESSAGE for the CAN adapter.
        (the CAN adapter calculates the checksums for the CAN frames to be sent automatically)
        :return:
        """
        checksum = sum(data) & 0xFF
        return checksum

    def init_bus(self):
        """
        Sends special CAN command to initialize the bus, so slaves get ready for listening.
        :return: Always True
        """
        self.send_can_frame(dev_id = 0x00, data = [0x01, 0x00], type = "standard")                               # as referred in Schnier documentation
        return (True)


if __name__ == "__main__":

    canbus = usb_can_a()


    init_bus_message = [0x01, 0x00]
    for i in range(100):
        canbus.init_bus()

    time.sleep(3)

    canbus.stop_reception_thread()