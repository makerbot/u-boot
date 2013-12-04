
import os
from optparse import OptionParser
import struct
import math

EntryPoint = 0xC1080000
LoadAddress = EntryPoint

#memtype='NAND'
memtype='SPI'

#NAND stuff
data_page_length = 4096
oob_page_length = 224
FirstDataPage = 1
BlockNum = 6

#SPI stuff
spi_addr = 0x80000

if memtype == 'NAND':
    MagicNum = 0xA1ACED11
elif memtype == 'SPI':
    MagicNum = 0xA1ACED00

class UBootHeader:
    def __init__(self, in_file, out_file):
        self.file_size = os.path.getsize(in_file)
        self.num_pages = int(math.ceil(self.file_size / float(data_page_length)))
        self.in_file = open(in_file, 'rb')
        self.out_file = open(out_file, 'wb')

    def close(self):
        self.out_file.close()
        self.in_file.close()    

    def add_header(self):
        if memtype == 'NAND':
            header = struct.pack('<6I', MagicNum,
                                        EntryPoint,
                                        self.num_pages,
                                        BlockNum,
                                        FirstDataPage,
                                        LoadAddress
                                        )
        elif memtype == 'SPI':
            header = struct.pack('<5I', MagicNum,
                                        EntryPoint,
                                        self.file_size,
                                        spi_addr + 20, #Size of this header
                                        LoadAddress
                                        )
        for v in header:
            self.out_file.write(v)

        if memtype == 'NAND':
            ff = struct.pack('>B', 0xFF)
            for i in range(data_page_length - 6*4):
                self.out_file.write(ff[0])

        in_byte = 1
        while in_byte != "" :
            in_byte = self.in_file.read(1)
            self.out_file.write(in_byte)

    def print_output(self):
        """ Wow this is broken """

        f = self.out_file
        
        in_byte = 1
        while in_byte != "" :
            for i in range(data_page_length / 16):
                in_byte = f.read(8)
                print " ".join('{0:2x}'.format(ord(c)) for c in in_byte),
                print "  ",
                in_byte = f.read(8)
                print " ".join('{0:2x}'.format(ord(c)) for c in in_byte)
            print "page break"            


if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-i", "--input", dest="in_file",
                      help="file to test")
    parser.add_option("-o", "--output", dest="output",
                      help="output file")
    (options, args) = parser.parse_args()
    
    header = UBootHeader(options.in_file, options.output)

    header.add_header()

    #header.print_output()



