from pymodbus.client.tcp import ModbusTcpClient


class OnrobotSGDriver:

    def __init__(self, ip, port, modelID):
        # open connection to device
        self.client = ModbusTcpClient(ip, port=port, timeout=1)
        self.open_connection()

        # initialize
        self.set_model_id(modelID)
        self.set_init()

        self.gripperTypes = {
            1: "None",
            2: "SG-a-H (Flower-Like w/o White Tip)",
            3: "SG-a-S (Flower-Like w/ White Tip)",
            4: "SG-b-H (Claw-Like)",
        }
        self.gripperChosen = self.gripperTypes[modelID]
        self.maxWidth, self.minWidth = self.get_maxmin_width()

    def limit(self, realmm):
        if realmm < self.minWidth:
            return self.minWidth
        elif realmm > self.maxWidth:
            return self.maxWidth
        return realmm

    def cnvt_real_to_signal(self, real):
        return int(real * 10)

    def cnvt_signal_to_real(self, signal):
        return signal / 10.0

    def open_connection(self):
        self.client.connect()

    def close_connection(self):
        self.client.close()

    def set_target(self, realmm):
        realmm = self.limit(realmm)
        self.client.write_register(address=0x0000, value=self.cnvt_real_to_signal(realmm), unit=1)

    def set_command(self, command):
        self.client.write_register(address=0x0001, value=command, unit=1)

    def set_init(self):
        self.set_command(0x3)

    def set_move(self):
        self.set_command(0x1)

    def set_stop(self):
        self.set_command(0x2)

    def set_gentle(self, command: bool):
        """
        Gripping speed is reduced at 12.5mm before the specified target width

        """
        self.client.write_register(address=0x0002, value=command, unit=1)

    def set_model_id(self, modelID):
        self.client.write_register(address=0x0003, value=modelID, unit=1)

    def get_width(self):
        response = self.client.read_holding_registers(address=0x0100, count=1, slave=65)
        width = response.registers[0]
        return self.cnvt_signal_to_real(width)

    def get_status(self):
        response = self.client.read_holding_registers(address=0x0103, count=1, slave=65)
        status = response.registers[0]
        return status

    def get_maxmin_width(self):
        response = self.client.read_holding_registers(address=0x0105, count=1, slave=65)
        maxwidth = response.registers[0]

        response = self.client.read_holding_registers(address=0x0106, count=1, slave=65)
        minwidth = response.registers[0]

        return self.cnvt_signal_to_real(maxwidth), self.cnvt_signal_to_real(minwidth)