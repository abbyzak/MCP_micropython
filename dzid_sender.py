"""
BO_ 558 STEERING_COMMAND: 5 EON
 SG_ STEER_TORQUE : 32|8@1- (0.125,0) [-16|15.875] "Nm"  EPAS
 SG_ STEER_ANGLE : 16|16@1- (0.125,0) [-4096|4095.875] "deg"  EPAS
 SG_ STEER_MODE : 12|2@1+ (1,0) [0|3] ""  EPAS
 SG_ COUNTER : 8|4@1+ (1,0) [0|15] ""  EPAS
 SG_ CHECKSUM : 0|8@1+ (1,0) [0|255] ""  EPAS
 """
import math
import logging

class Signal:
    def __init__(self, name, lsb, size, is_little_endian, factor, offset):
        self.name = name
        self.lsb = lsb
        self.size = size
        self.is_little_endian = is_little_endian
        self.factor = factor
        self.offset = offset
        self.calc_checksum = None

class SignalPackValue:
    def __init__(self, name, value):
        self.name = name
        self.value = value

class CANPacker:
    def __init__(self, dbc_name):
        self.dbc = self.dbc_lookup(dbc_name)
        assert self.dbc

        self.signal_lookup = {}
        for msg in self.dbc.msgs:
            for sig in msg.sigs:
                self.signal_lookup[(msg.address, sig.name)] = sig

        self.counters = {}

    def dbc_lookup(self, dbc_name):
        # This is a placeholder for the dbc_lookup function
        # You'll need to implement this function to load the DBC file
        class DBC:
            def __init__(self):
                self.msgs = [
                    type('Message', (), {'address': 0x558, 'size': 5, 'sigs': [
                        type('Signal', (), {'name': 'STEER_TORQUE', 'lsb': 32, 'size': 8, 'is_little_endian': True, 'factor': 0.125, 'offset': 0}),
                        type('Signal', (), {'name': 'STEER_ANGLE', 'lsb': 16, 'size': 16, 'is_little_endian': True, 'factor': 0.125, 'offset': 0}),
                        type('Signal', (), {'name': 'STEER_MODE', 'lsb': 12, 'size': 2, 'is_little_endian': True, 'factor': 1, 'offset': 0}),
                        type('Signal', (), {'name': 'COUNTER', 'lsb': 8, 'size': 4, 'is_little_endian': True, 'factor': 1, 'offset': 0}),
                        type('Signal', (), {'name': 'CHECKSUM', 'lsb': 0, 'size': 8, 'is_little_endian': True, 'factor': 1, 'offset': 0}),
                    ]}),
                ]

        return DBC()

    def set_value(self, msg, sig, ival):
        i = sig.lsb // 8
        bits = sig.size
        if sig.size < 64:
            ival &= ((1 << sig.size) - 1)

        while i >= 0 and i < len(msg) and bits > 0:
            shift = (sig.lsb % 8) if (sig.lsb // 8) == i else 0
            size = min(bits, 8 - shift)

            # Clear the correct bits for the MSB
            if i == 0 and shift == 0:
                msg[i] = 0
            else:
                msg[i] &= ~(((1 << size) - 1) << shift)

            msg[i] |= (ival & ((1 << size) - 1)) << shift

            bits -= size
            ival >>= size
            i += 1 if sig.is_little_endian else -1

    def pack(self, address, signals):
        msg_it = next((msg for msg in self.dbc.msgs if msg.address == address), None)
        if msg_it is None:
            logging.error(f"undefined address {address}")
            return []

        ret = [0] * msg_it.size

        counter_set = False
        for sigval in signals[:-1]:  # Exclude the checksum signal
            sig_it = self.signal_lookup.get((address, sigval.name))
            if sig_it is None:
                logging.error(f"undefined signal {sigval.name} - {address}")
                continue

            ival = round((sigval.value - sig_it.offset) / sig_it.factor)
            if ival < 0:
                ival = (1 << sig_it.size) + ival
            self.set_value(ret, sig_it, ival)

            if sigval.name == "COUNTER":
                self.counters[address] = sigval.value
                counter_set = True

        sig_it_counter = self.signal_lookup.get((address, "COUNTER"))
        if sig_it_counter and not counter_set:
            if address not in self.counters:
                self.counters[address] = 0
            self.set_value(ret, sig_it_counter, self.counters[address])
            self.counters[address] = (self.counters[address] + 1) % (1 << sig_it_counter.size)
        checksum_index = self.signal_lookup.get((address, "CHECKSUM")).lsb // 8
        ret[checksum_index] = signals[-1].value
        packed_bytes = bytes(ret)
        checksum = 558
        for i in range(1,5):
            checksum += packed_bytes[i]
        checksum = (checksum & 0xFF) + (checksum >> 8)
        checksum &= 0xFF
        ret[0] = checksum
        return ret


def main():
    # Define the DBC file
    dbc_name = ""

    # Create a CANPacker object
    packer = CANPacker(dbc_name)

    # Define the address and signals to pack
    address = 0x558  # STEERING_COMMAND message
    
    signals = [
        SignalPackValue("STEER_TORQUE", 15),  # 10 Nm
        SignalPackValue("STEER_ANGLE", 70),  # 30 deg
        SignalPackValue("STEER_MODE", 2),    # 2
        SignalPackValue("COUNTER", 1),       # 5
        SignalPackValue("CHECKSUM", 0),       # 5
    ]

    # Pack the data
    ret = packer.pack(address,signals)
    packed_bytes = bytes(ret)
    # Print the packed data
    print("Packed Data:", packed_bytes.hex())

if __name__ == "__main__":
    main()
