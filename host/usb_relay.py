

import usb.core
import usb.util
import usb
import sys
import time

COMMAND_DEVICE_RESTART = 0x01
COMMAND_I2C_START_SCAN = 0x30
COMMAND_I2C_GET_DEVICE_MAP = 0x31
COMMAND_BIT_WRITE      = 0x10
COMMAND_VALUE_WRITE      = 0x20
COMMAND_DEVICE_WRITE_DEFAULT_VALUE = 0x40

COMMAND_DEVICE_WRITE_TAG = 0x42
COMMAND_DEVICE_READ_TAG = 0x43

class UsbRelay:
    def __init__(self, usb_device):
        self.usb_device=usb_device
        pass

    def get_device_info(self):
        return {
            "SerialNumber" : self.usb_device.util.get_string(dev,dev.iSerialNumber),
            "Product" : self.usb_device.util.get_string(dev,dev.iProduct),
            "Manufacturer" : self.usb_device.util.get_string(dev,dev.iManufacturer),
            # print(usb.util.get_string(dev,dev.iManufacturer))
        }

    def restart_device(self):
        try:
            self.usb_device.ctrl_transfer(0x40, COMMAND_DEVICE_RESTART, 0, 0, 0)
        except:
            pass

    def i2c_scan(self):
        self.usb_device.ctrl_transfer(0x40, COMMAND_I2C_START_SCAN, 0, 0, 0)
        time.sleep(0.1)
        response = self.usb_device.ctrl_transfer(0xC0, COMMAND_I2C_GET_DEVICE_MAP, 0, 0, 16)
        r=[]
        for i in range(len(response)):
            if response[i] > 0:
                for j in range(0,8):
                    if response[i] & (1 << j):
                        address = (i << 3) + j
                        r.append(address)
        return r

    def write_bit(self, bit, value):
        self.usb_device.ctrl_transfer(0x40, COMMAND_BIT_WRITE, 1 if value else 0, bit, 0)

    def write_value(self, value):
        self.usb_device.ctrl_transfer(0x40, COMMAND_VALUE_WRITE, value, 0, 0)

    def write_default_value(self, value):
        self.usb_device.ctrl_transfer(0x40, COMMAND_DEVICE_WRITE_DEFAULT_VALUE, value, 0, 0)

    def get_tag(self):
        return self.usb_device.ctrl_transfer(0xC0, COMMAND_DEVICE_READ_TAG, 0, 0, 4)

    def set_tag(self, tag):
        self.usb_device.ctrl_transfer(0x40, COMMAND_DEVICE_WRITE_TAG, (tag >> 16) & 0xFFFF, (tag & 0xFFFF), 0)


def find_single_device():
    # find our device
    dev = usb.core.find(idVendor=0x4242, idProduct=0xe131)

    # was it found?
    if dev is None:
        raise ValueError('Device not found')

    # set the active configuration. With no arguments, the first
    # configuration will be the active one
    dev.set_configuration()

    return dev

# for cfg in dev:
#     print("bConfigurationValue=" + str(cfg.bConfigurationValue))
#     for intf in cfg:
#         print('\tInterfaceNumber=' + \
#                          str(intf.bInterfaceNumber) + \
#                          ',AlternateSetting=' + \
#                          str(intf.bAlternateSetting))
#         for ep in intf:
#             print('\t\tEndpointAddress=' + \
#                              str(ep.bEndpointAddress))

def main():
    def auto_int(x):
        return int(x, 0)

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=["restart", "bit", "value", "i2c-scan", "write-default", "get-tag", "set-tag"], help='Command to execute')
    parser.add_argument('args', nargs="*", type=auto_int)
    args = parser.parse_args()
    # print(args)

    relay = UsbRelay(find_single_device())
    if args.command == "restart":
        relay.restart_device()
        return
    if args.command == "bit":
        relay.write_bit(*args.args)
        return
    if args.command == "value":
        relay.write_value(*args.args)
        return
    if args.command == "write-default":
        relay.write_default_value(*args.args)
        return
    if args.command == "i2c-scan":
        r = relay.i2c_scan()
        print(f"({len(r)}): " + " ".join([f"0x{x:02x}" for x in r ]))
        return
    if args.command == "get-tag":
        result = relay.get_tag()
        result.reverse()
        print("0x" + ("".join([f"{x:02x}" for x in result ])))
        return
    if args.command == "set-tag":
        relay.set_tag(*args.args)
        return

if __name__ == '__main__':
    main()
