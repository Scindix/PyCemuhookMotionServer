import random
import socket
import string
import zlib
import struct
import threading


class Controller:
    def __init__(self, state, model, connType, macAddress, battery):
        self.state = state
        self.model = model
        self.connectionType = connType
        self.macAddress = macAddress
        self.battery = battery
        self.gyroX = 0
        self.gyroY = 0
        self.gyroZ = 0
        self.accelX = 0
        self.accelY = 0
        self.accelZ = 0
        self.isActive = True


class MotionServer(threading.Thread):
    threadLock = threading.Lock()

    def __init__(self, port, verbose=False):
        threading.Thread.__init__(self)
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', port))
        self.active = True
        self.controllerList = []
        self.verbose = verbose

    def run(self):
        self.active = True
        while self.active:
            try:
                message, address = self.socket.recvfrom(1024)
                # hex_dump(message)
                receivedPacket = Packet(bytearray(message))
                if(self.verbose):
                    receivedPacket.print()
                with self as controllerList:
                    for packet in receivedPacket.answer(controllerList):
                        if(self.verbose):
                            hex_dump(packet)
                        self.socket.sendto(packet, address)
                # pprint(vars(receivedPacket))
                self.socket.sendto(message, address)
            except OSError:
                self.active = False

    def stop(self):
        self.active = False
        self.socket.close()

    def __enter__(self):
        MotionServer.threadLock.acquire()
        return self.controllerList

    def __exit__(self, type, value, traceback):
        MotionServer.threadLock.release()


class Packet:
    DSUC_Invalid = 0x000000
    DSUC_VersionReq = 0x100000
    DSUS_VersionRsp = 0x100000
    DSUC_ListPorts = 0x100001
    DSUS_PortInfo = 0x100001
    DSUC_PadDataReq = 0x100002
    DSUS_PadDataRsp = 0x100002
    protocolVersion = 1001
    serverId = random.randint(0, 99999999)
    counter = 0

    def __init__(self, msg):
        if(len(msg) < 16 or not msg.startswith(b'DSUC')):
            self.type = Packet.DSUC_Invalid
            return
        index = 4
        self.protocolVersion = int.from_bytes(msg[index:index+2],
                                              byteorder='little', signed=False)
        index = index+2
        self.size = int.from_bytes(msg[index:index+2],
                                   byteorder='little', signed=False)
        index = index+2
        self.crc = int.from_bytes(msg[index:index+4],
                                  byteorder='little', signed=False)
        for i in range(4):
            msg[index+i] = 0
        self.computedCrc = zlib.crc32(msg)
        if(self.crc != self.computedCrc):
            self.type = Packet.DSUC_Invalid
            return
        index = index+4
        self.clientId = int.from_bytes(msg[index:index+4],
                                       byteorder='little', signed=False)
        index = index+4
        self.type = int.from_bytes(msg[index:index+4],
                                   byteorder='little', signed=False)
        index = index+4
        if(self.type == Packet.DSUC_ListPorts):
            numPadRequests = int.from_bytes(msg[index:index+4],
                                            byteorder='little', signed=False)
            index = index+4
            self.padReqs = []
            for i in range(numPadRequests):
                self.padReqs.append(int.from_bytes(msg[index:index+1],
                                                   byteorder='little', signed=False))
                index = index+1
        elif(self.type == Packet.DSUC_PadDataReq):
            self.registrationFlags = int.from_bytes(msg[index:index+1], byteorder='little', signed=False)
            index = index+1
            self.registrationId = int.from_bytes(msg[index:index+1], byteorder='little', signed=False)
            index = index+1
            self.registrationMac = msg[index:index+6]

    def print(self):
        out = "["
        if(self.type == Packet.DSUC_Invalid):
            print("[Invalid Packet]")
            return
        else:
            out = out + "id: " + str(self.clientId) + "; version: " + str(self.protocolVersion) + "; size: " + str(self.size)
        if(self.type == Packet.DSUC_VersionReq):
            out = "Version Request: " + out
        elif(self.type == Packet.DSUC_ListPorts):
            out = "List Ports: " + out + "; padReqs: " + str(self.padReqs)
        elif(self.type == Packet.DSUC_PadDataReq):
            out = "Pad Data Request: " + out + "; flags: {:0>8b}".format(self.registrationFlags) + "; rid: " + str(self.registrationId) + "; " + ":".join(["%02x"%c for c in self.registrationMac])
        else:
            out = "Unknown Type: " + out
        print(out + "]")

    def answer(self, cList):
        if(self.type == Packet.DSUC_VersionReq):
            out = bytearray(8)
            for i, b in enumerate(Packet.DSUS_VersionRsp.to_bytes(4, byteorder='little', signed=False)):
                out[i] = b
            for i, b in enumerate(Packet.protocolVersion.to_bytes(4, byteorder='little', signed=False)):
                out[4+i] = b
            return [Packet.generate(out)]
        elif(self.type == Packet.DSUC_ListPorts):
            answerPackets = []
            for x in self.padReqs:
                if(x < len(cList)):
                    out = bytearray(16)
                    for i, b in enumerate(Packet.DSUS_PortInfo.to_bytes(4, byteorder='little', signed=False)):
                        out[i] = b
                    out[4] = x+1
                    out[5] = cList[x].state
                    out[6] = cList[x].model
                    out[7] = cList[x].connectionType
                    for i, b in enumerate(cList[x].macAddress):
                        out[8+i] = b
                    out[14] = cList[x].battery
                    out[15] = 0
                    answerPackets.append(Packet.generate(out))
            return answerPackets
        elif(self.type == Packet.DSUC_PadDataReq):
            if(self.registrationId >= len(cList)):
                return []
            out = bytearray(84)
            for i, b in enumerate(Packet.DSUS_PadDataRsp.to_bytes(4, byteorder='little', signed=False)):
                out[i] = b
            out[4] = self.registrationId
            out[5] = cList[self.registrationId].state
            out[6] = cList[self.registrationId].model
            out[7] = cList[self.registrationId].connectionType
            for i, b in enumerate(cList[self.registrationId].macAddress):
                out[8+i] = b
            out[14] = cList[self.registrationId].battery
            out[15] = 0x1 if cList[self.registrationId].isActive else 0x0
            Packet.counter = Packet.counter + 1
            for i, b in enumerate(Packet.counter.to_bytes(4, byteorder='little', signed=False)):
                out[16+i] = b
            out[20] = 0x0  # DPAD left, down, right, up, options, share, R3, L3
            out[21] = 0x0  # X, A, B, Y, R1, L1, R2, L2
            out[22] = 0x0  # button.PS
            out[23] = 0x0  # button.touch
            out[24] = 0x0  # position.left.x
            out[25] = 0x0  # position.left.y
            out[26] = 0x0  # position.right.x
            out[27] = 0x0  # position.right.y
            out[28] = 0x0  # DPAD left
            out[29] = 0x0  # DPAD down
            out[30] = 0x0  # DPAD right
            out[31] = 0x0  # DPAD up
            out[32] = 0x0  # X
            out[33] = 0x0  # A
            out[34] = 0x0  # B
            out[35] = 0x0  # Y
            out[36] = 0x0  # R1
            out[37] = 0x0  # L1
            out[38] = 0x0  # R2
            out[39] = 0x0  # L2
            out[40] = 0x0  # trackpad 1 is active
            out[41] = 0x0  # trackpad 1 id
            for i in range(2):  # trackpad 1 x
                out[42+i] = 0x0
            for i in range(2):  # trackpad 1 y
                out[44+i] = 0x0
            out[46] = 0x0       # trackpad 2 is active
            out[47] = 0x0       # trackpad 2 id
            for i in range(2):  # trackpad 2 x
                out[48+i] = 0x0
            for i in range(2):  # trackpad 2 y
                out[50+i] = 0x0
            for i in range(4):  # motion timestamp low
                out[52+i] = 0x0
            for i in range(4):  # motion timestamp high
                out[56+i] = 0x0

            # acceleration x
            struct.pack_into('<f', out, 60, cList[self.registrationId].accelX)
            # acceleration y
            struct.pack_into('<f', out, 64, cList[self.registrationId].accelY)
            # acceleration z
            struct.pack_into('<f', out, 68, cList[self.registrationId].accelZ)

            # gyro x
            struct.pack_into('<f', out, 72, cList[self.registrationId].gyroX)
            # gyro y
            struct.pack_into('<f', out, 76, cList[self.registrationId].gyroY)
            # gyro z
            struct.pack_into('<f', out, 80, cList[self.registrationId].gyroZ)

            return [Packet.generate(out)]
        return []

    @staticmethod
    def generate(data):
        buffer = bytearray(len(data)+16)
        for i, c in enumerate("DSUS"):
            buffer[i] = ord(c)
        for i, b in enumerate(Packet.protocolVersion.to_bytes(2, byteorder='little', signed=False)):
            buffer[4+i] = b
        for i, b in enumerate(len(data).to_bytes(2, byteorder='little', signed=False)):
            buffer[6+i] = b
        for i, b in enumerate(Packet.serverId.to_bytes(4, byteorder='little', signed=False)):
            buffer[12+i] = b
        for i, b in enumerate(data):
            buffer[16+i] = b
        crc = zlib.crc32(buffer)
        for i, b in enumerate(crc.to_bytes(4, byteorder='little', signed=False)):
            buffer[8+i] = b
        return buffer


def hex_dump(message):
    print("-- START --")
    clear = ""
    out = ""
    for c in message:
        if(chr(c) in string.digits+string.ascii_letters+string.punctuation):
            clear = clear+chr(c)
        else:
            clear = clear+'.'
        out = out+(" %02x" % c)
        if(len(clear) == 8):
            print('%s %s' % (out, clear))
            out = ""
            clear = ""
    print("-- END --")







