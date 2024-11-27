import time
from machine import Pin, SPI
from constants import *

class MCP2515:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self.cs.init(self.cs.OUT, value=1)

    def start_spi(self):
        self.cs.off()

    def end_spi(self):
        self.cs.on()

    def set_register(self, reg, value):
        self.start_spi()
        self.spi.write(bytearray([INSTRUCTION.INSTRUCTION_WRITE, reg, value]))
        self.end_spi()
        
    def set_registerx(self, reg, values, n):
        self.start_spi()
        self.spi.write(bytearray([INSTRUCTION.INSTRUCTION_WRITE]))  # Send the instruction byte
        self.spi.write(bytearray([reg]))  # Send the register address

        for i in range(n):
            self.spi.write(bytearray([values[i]]))  # Send each value, up to `n` values
        self.end_spi()


    def set_registers(self, reg, values):
        self.start_spi()
        self.spi.write(bytearray([INSTRUCTION.INSTRUCTION_WRITE, reg]) + bytearray(values))
        self.end_spi()

    def modify_register(self, reg, mask, value):
        self.start_spi()
        self.spi.write(bytearray([INSTRUCTION.INSTRUCTION_BITMOD, reg, mask, value]))
        self.end_spi()
        
    def readRegister(self, reg):
        # Read a single register
        self.start_spi()
        self.spi.write(bytearray([INSTRUCTION.INSTRUCTION_READ, reg]))  # Write READ instruction + register
        ret = self.spi.read(1)  # Read 1 byte from the register
        self.end_spi()
        return ret[0]  # Return the first byte as the register value
    
    def readRegisters(self, reg, n):
        # Read multiple consecutive registers
        self.start_spi()
        self.spi.write(bytearray([INSTRUCTION.INSTRUCTION_READ, reg]))  # Write READ instruction + register
        values = []
        for _ in range(n):
            values.append(self.spi.read(1)[0])  # Append each byte to values list
        self.end_spi()
        return values
    
    def reset(self):
        self.start_spi()
        self.spi.write(bytearray([INSTRUCTION.INSTRUCTION_RESET]))
        self.end_spi()

        time.sleep(0.01)

        zeros = [0] * 14
        self.set_registers(REGISTER.MCP_TXB0CTRL, zeros)
        self.set_registers(REGISTER.MCP_TXB1CTRL, zeros)
        self.set_registers(REGISTER.MCP_TXB2CTRL, zeros)

        self.set_register(REGISTER.MCP_RXB0CTRL, 0)
        self.set_register(REGISTER.MCP_RXB1CTRL, 0)

        self.set_register(REGISTER.MCP_CANINTE, CANINTF.CANINTF_RX0IF | CANINTF.CANINTF_RX1IF |
                          CANINTF.CANINTF_ERRIF | CANINTF.CANINTF_MERRF)

        self.modify_register(REGISTER.MCP_RXB0CTRL,
                             MCP2515Constants.RXBnCTRL_RXM_MASK | MCP2515Constants.RXB0CTRL_BUKT | MCP2515Constants.RXB0CTRL_FILHIT_MASK,
                             MCP2515Constants.RXBnCTRL_RXM_STDEXT | MCP2515Constants.RXB0CTRL_BUKT | MCP2515Constants.RXB0CTRL_FILHIT)

        self.modify_register(REGISTER.MCP_RXB1CTRL,
                             MCP2515Constants.RXBnCTRL_RXM_MASK | MCP2515Constants.RXB1CTRL_FILHIT_MASK,
                             MCP2515Constants.RXBnCTRL_RXM_STDEXT | MCP2515Constants.RXB1CTRL_FILHIT)

        filters = [0, 1, 2, 3, 4, 5]  # RXF0 to RXF5
        """for filter_id in filters:
            ext = (filter_id == 1)
            result = self.set_filter(filter_id, ext, 0)
            if result != 0:
                return result

        masks = [0, 1]  # MASK0, MASK1
        for mask_id in masks:
            result = self.set_filter_mask(mask_id, True, 0)
            if result != 0:
                return result"""

        return 0

    def set_mode(self, mode):
        # Modify the CANCTRL register to set the requested mode
        self.modify_register(REGISTER.MCP_CANCTRL, MCP2515Constants.CANCTRL_REQOP, mode)

        # Wait for the mode to be set, check within 10 milliseconds
        end_time = time.ticks_add(time.ticks_ms(), 10)  # Current time + 10 ms
        mode_match = False

        while time.ticks_diff(end_time, time.ticks_ms()) > 0:
            # Read the MCP_CANSTAT register to check the current mode
            new_mode = self.readRegister(REGISTER.MCP_CANSTAT)
            new_mode &= MCP2515Constants.CANSTAT_OPMOD

            mode_match = (new_mode == mode)

            if mode_match:
                break

        # Return status based on whether the mode was successfully set
        return ERROR.ERROR_OK if mode_match else ERROR.ERROR_FAIL

    def prepare_id(self, buffer, ext, idx):
            canid = idx & 0x0FFFF
            if ext:
                buffer[MCP2515Constants.MCP_EID0] = canid & 0xFF
                buffer[MCP2515Constants.MCP_EID8] = canid >> 8
                canid = idx >> 16
                buffer[MCP2515Constants.MCP_SIDL] = canid & 0x03
                buffer[MCP2515Constants.MCP_SIDL] += ((canid & 0x1C) << 3)
                buffer[MCP2515Constants.MCP_SIDL] |= MCP2515Constants.TXB_EXIDE_MASK
                buffer[MCP2515Constants.MCP_SIDH] = canid >> 5
            else:  # Standard ID
                buffer[MCP2515Constants.MCP_SIDH] = canid >> 3
                buffer[MCP2515Constants.MCP_SIDL] = (canid & 0x07) << 5
                buffer[MCP2515Constants.MCP_EID0] = 0
                buffer[MCP2515Constants.MCP_EID8] = 0

    def set_bitrate(self,can_speed, can_clock):
        cfg_map = {
            "MCP_8MHZ": {
                "CAN_1000KBPS": (0x00, 0x80, 0x80),
                "CAN_500KBPS": (0x00, 0x90, 0x82),
                "CAN_250KBPS": (0x00, 0xB1, 0x85),
                "CAN_200KBPS": (0x00, 0xB4, 0x86),
                "CAN_125KBPS": (0x01, 0xB1, 0x85),
                "CAN_100KBPS": (0x01, 0xB4, 0x86),
                "CAN_80KBPS": (0x01, 0xBF, 0x87),
                "CAN_50KBPS": (0x03, 0xB4, 0x86),
                "CAN_40KBPS": (0x03, 0xBF, 0x87),
                "CAN_33K3BPS": (0x47, 0xE2, 0x85),
                "CAN_31K25BPS": (0x07, 0xA4, 0x84),
                "CAN_20KBPS": (0x07, 0xBF, 0x87),
                "CAN_10KBPS": (0x0F, 0xBF, 0x87),
                "CAN_5KBPS": (0x1F, 0xBF, 0x87),
            },
            "MCP_16MHZ": {
                "CAN_1000KBPS": (0x00, 0xD0, 0x82),
                "CAN_500KBPS": (0x00, 0xF0, 0x86),
                "CAN_250KBPS": (0x41, 0xF1, 0x85),
                "CAN_200KBPS": (0x01, 0xFA, 0x87),
                "CAN_125KBPS": (0x03, 0xF0, 0x86),
                "CAN_100KBPS": (0x03, 0xFA, 0x87),
                "CAN_95KBPS": (0x03, 0xAD, 0x07),
                "CAN_83K3BPS": (0x03, 0xBE, 0x07),
                "CAN_80KBPS": (0x03, 0xFF, 0x87),
                "CAN_50KBPS": (0x07, 0xFA, 0x87),
                "CAN_40KBPS": (0x07, 0xFF, 0x87),
                "CAN_33K3BPS": (0x4E, 0xF1, 0x85),
                "CAN_20KBPS": (0x0F, 0xFF, 0x87),
                "CAN_10KBPS": (0x1F, 0xFF, 0x87),
                "CAN_5KBPS": (0x3F, 0xFF, 0x87),
            },
            "MCP_20MHZ": {
                "CAN_1000KBPS": (0x00, 0xD9, 0x82),
                "CAN_500KBPS": (0x00, 0xFA, 0x87),
                "CAN_250KBPS": (0x41, 0xFB, 0x86),
                "CAN_200KBPS": (0x01, 0xFF, 0x87),
                "CAN_125KBPS": (0x03, 0xFA, 0x87),
                "CAN_100KBPS": (0x04, 0xFA, 0x87),
                "CAN_83K3BPS": (0x04, 0xFE, 0x87),
                "CAN_80KBPS": (0x04, 0xFF, 0x87),
                "CAN_50KBPS": (0x09, 0xFA, 0x87),
                "CAN_40KBPS": (0x09, 0xFF, 0x87),
                "CAN_33K3BPS": (0x0B, 0xFF, 0x87),
            },
        }
        if can_clock not in cfg_map or can_speed not in cfg_map[can_clock]:
            return 1  # ERROR_FAIL

        cfg1, cfg2, cfg3 = cfg_map[can_clock][can_speed]

        self.set_register(REGISTER.MCP_CNF1, cfg1)
        self.set_register(REGISTER.MCP_CNF2, cfg2)
        self.set_register(REGISTER.MCP_CNF3, cfg3)
        return 0


    TXB0_CTRL, TXB0_SIDH, TXB0_DATA = 0x30, 0x31, 0x32
    TXB1_CTRL, TXB1_SIDH, TXB1_DATA = 0x40, 0x41, 0x42
    TXB2_CTRL, TXB2_SIDH, TXB2_DATA = 0x50, 0x51, 0x52

    # Receive Buffers
    RXB0_CTRL, RXB0_SIDH, RXB0_DATA, RXB0_CANINTF = 0x60, 0x61, 0x62, 0x2C
    RXB1_CTRL, RXB1_SIDH, RXB1_DATA, RXB1_CANINTF = 0x70, 0x71, 0x72, 0x2D

    @classmethod
    def access_txb_registers(cls, buffer_index):
        if buffer_index == 0:
            ctrl, sidh, data = cls.TXB0_CTRL, cls.TXB0_SIDH, cls.TXB0_DATA
        elif buffer_index == 1:
            ctrl, sidh, data = cls.TXB1_CTRL, cls.TXB1_SIDH, cls.TXB1_DATA
        elif buffer_index == 2:
            ctrl, sidh, data = cls.TXB2_CTRL, cls.TXB2_SIDH, cls.TXB2_DATA
        return ctrl, sidh, data

    @classmethod
    def access_rxb_registers(cls, buffer_index):
        if buffer_index == 0:
            ctrl, sidh, data, canintf = cls.RXB0_CTRL, cls.RXB0_SIDH, cls.RXB0_DATA, cls.RXB0_CANINTF
        elif buffer_index == 1:
            ctrl, sidh, data, canintf = cls.RXB1_CTRL, cls.RXB1_SIDH, cls.RXB1_DATA, cls.RXB1_CANINTF
        return ctrl, sidh, data, canintf

    def send_message(self, frame):
        if len(frame['data']) > 10:
            return "ERROR_FAILTX"

        tx_buffers = [0, 1, 2]

        # Check the transmit buffers
        for tx_buffer in tx_buffers:
            ctrlval = self.readRegister(tx_buffer)  # Read the control register for the buffer
            if (ctrlval & TXBnCTRL.TXB_TXREQ) == 0:
                self.send_message_to_buffer(tx_buffer, frame)
                return "SUCCESS"

        return "ERROR_ALLTXBUSY"
    
    def send_message_to_buffer(self, txbn, frame):
        if frame["can_dlc"] > 10:
            return ERROR.ERROR_FAILTX

        txbuf = self.access_txb_registers(txbn)
        data = bytearray(13)

        ext = frame["can_id"] & CAN_EFF_FLAG
        rtr = frame["can_id"] & CAN_RTR_FLAG
        can_id = frame["can_id"] & (CAN_EFF_MASK if ext else CAN_SFF_MASK)

        self.prepare_id(data, ext, can_id)

        # Set DLC and data
        data[MCP2515Constants.MCP_DLC] = frame["can_dlc"] | (RTR_MASK.RTR_MASK if rtr else frame["can_dlc"])
        data[MCP2515Constants.MCP_DATA:MCP2515Constants.MCP_DATA + frame["can_dlc"]] = bytearray(frame["data"])  # Convert list to bytearray
        txbuf = self.access_txb_registers(txbn)  # Tuple of (ctrl, sidh, data)
        self.set_registerx(txbuf[1], data,5+frame['can_dlc'])
        self.modify_register(txbuf[0], TXBnCTRL.TXB_TXREQ, TXBnCTRL.TXB_TXREQ)

        ctrl = self.readRegister(txbuf[0])
        if ctrl & (TXBnCTRL.TXB_ABTF | TXBnCTRL.TXB_MLOA | TXBnCTRL.TXB_TXERR):
            return ERROR.ERROR_FAILTX

        return ERROR.ERROR_OK


        ctrl = self.read_register(txbuf["CTRL"])
        if ctrl & (TXBnCTRL.TXB_ABTF | TXBnCTRL.TXB_MLOA | TXBnCTRL.TXB_TXERR):
            return ERROR.ERROR_FAILTX

        return self.ERROR_OK
    
    
    

    ############################### Untested functions are below ####################################
    def check_status(self):
        self.start_spi()
        self.spi.write(bytearray([INSTRUCTION.INSTRUCTION_READ_STATUS]))  # Read Status instruction
        status = self.spi.read(1)[0]  # Read 1 byte from the status register
        self.end_spi()
        return status

    def read_message(self, rxbn, frame):
        rxb = [REGISTER.MCP_RXB0CTRL, REGISTER.MCP_RXB0SIDH, REGISTER.MCP_RXB0DATA, CANINTF.CANINTF_RX0IF] if rxbn ==0 else [REGISTER.MCP_RXB1CTRL, REGISTER.MCP_RXB1SIDH, REGISTER.MCP_RXB1DATA, CANINTF.CANINTF_RX1IF]
        tbufdata = self.readRegisters(rxb[1], 5)

        can_id = (tbufdata[0] << 3) | (tbufdata[1] >> 5)

        if tbufdata[1] & 0x08:
            can_id = ((can_id << 2) | (tbufdata[1] & 0x03)) << 8 | tbufdata[2]
            can_id = (can_id << 8) | tbufdata[3]
            can_id |= 0x80000000

        dlc = tbufdata[4] & 0x0F
        if dlc > 8:
            return ERROR.ERROR_FAIL

        ctrl = self.readRegister(rxb[0])
        if ctrl & 0x08:
            can_id |= 0x40000000

        frame['can_id'] = can_id
        frame['can_dlc'] = dlc
        frame['data'] = self.readRegisters(rxb[2], dlc)

        self.modify_register(REGISTER.MCP_CANINTF, rxb[3], 0)

        return ERROR.ERROR_OK

    def read_message_any(self, frame):
        stat = self.check_status()

        if stat & (1<<0):
            return self.read_message(0, frame)
        elif stat & (1<<1):
            return self.read_message(1, frame)
        else:
            return ERROR.ERROR_NOMSG


   



spi = SPI(2, baudrate=1000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
cs_pin = Pin(5, Pin.OUT)  # Example GPIO for CS pin
mcp = MCP2515(spi, cs_pin)
mcp.reset()
print(mcp.set_mode(CANCTRL_REQOP_MODE.CANCTRL_REQOP_CONFIG))
print(mcp.set_bitrate("CAN_500KBPS","MCP_8MHZ"))
## Now lets get ready for our first message passing thing
print(mcp.set_mode(CANCTRL_REQOP_MODE.CANCTRL_REQOP_NORMAL))#reccieve mode set now lets finish the code fr the send message
# CAN frame data
frame = {
    "can_id": 0x123,  # Example CAN ID
    "can_dlc": 8,     # Data length
    "data": [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
}

# Send message
result = mcp.send_message(frame)


register_value = mcp.readRegister(REGISTER.MCP_CANCTRL)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_CANSTAT)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_TXB0CTRL)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_RXB0CTRL)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_CNF1)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_CNF2)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_CNF3)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_CANINTE)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_CANINTF)
print(f"Register value: {hex(register_value)}")

register_value = mcp.readRegister(REGISTER.MCP_EFLG)
print(f"Register value: {hex(register_value)}")

# Example usage:
status = mcp.check_status()
print(f"Status register: {hex(status)}")

frame = {
    "can_id": 0,  # Example CAN ID
    "can_dlc": 0,     # Data length
    "data": 0,
}
# Check for messages in RXB0
message = mcp.read_message_any(frame)
if message:
    print(f"Message received in RXB0: {message}")
else:
    print("No message in RXB0.")

    
while(True):
    if mcp.check_status()==11:
        message = mcp.read_message_any(frame)
        print(f"CAN ID: {hex(frame['can_id'])}")
        print(f"CAN DLC: {frame['can_dlc']}")
        print(f"Data: {[hex(byte) for byte in frame['data']]}")

            
    time.sleep(1)
