import serial
import struct
import threading
import time

# -------------------- PROTOCOL CONSTANTS --------------------
SOF = 0xAA
TYPE_TELEM = 1
TYPE_CMD = 2
TYPE_MSG = 3 # Raw string message

# -------------------- COMMAND CODES --------------------
CMD_ARM          = 1
CMD_DISARM       = 2
CMD_TAKEOFF      = 3
CMD_LAND         = 4
CMD_MOVE_FORWARD = 5

# -------------------- VEHICLE MODES (for Status) --------------------
# used in 'estop' field of telemetry to represent current mode
MODE_INITIAL  = 0
MODE_GUIDED   = 1
MODE_AUTO     = 2
MODE_LAND     = 3
MODE_DISARMED = 4 # Special case status

TELEM_FMT = "<IIffBB"  # 18 bytes
CMD_FMT   = "<IBB"     # 6 bytes

class V2VBridge:
    """
    A robust communication library for UAV/UGV bridge systems.
    Handles Serial framing, XOR checksums, and background reading 
    so you can focus on your mission logic.
    """
    def __init__(self, port, baud=115200, name="Bridge"):
        self.port = port
        self.baud = baud
        self.name = name
        self.ser = None
        
        # State
        self.latest_telemetry = None
        self.latest_command = None
        self.latest_msg = None
        self._running = False
        self._lock = threading.Lock()
        self._thread = None

    def connect(self):
        """Opens the serial connection."""
        print(f"[{self.name}] Connecting to {self.port} at {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"[{self.name}] Bridge Thread Started.")

    def stop(self):
        """Stops the bridge."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()

    def _chk_xor(self, type_b, len_b, payload: bytes) -> int:
        c = (type_b ^ len_b) & 0xFF
        for b in payload:
            c ^= b
        return c & 0xFF

    def _read_loop(self):
        """Background thread: scans for SOF and parses frames."""
        while self._running:
            if self.ser.in_waiting == 0:
                time.sleep(0.001)
                continue
                
            # 1. Scan for SOF
            b = self.ser.read(1)
            if not b or b[0] != SOF:
                continue
            
            # 2. Read Type and Length
            hdr = self.ser.read(2)
            if len(hdr) < 2:
                continue
            f_type, f_len = hdr[0], hdr[1]

            # 3. Read Payload
            payload = self.ser.read(f_len)
            if len(payload) < f_len:
                continue

            # 4. Read Checksum
            chk_byte = self.ser.read(1)
            if not chk_byte:
                continue
            
            # 5. Verify Checksum
            if chk_byte[0] == self._chk_xor(f_type, f_len, payload):
                with self._lock:
                    if f_type == TYPE_TELEM and f_len == struct.calcsize(TELEM_FMT):
                        self.latest_telemetry = struct.unpack(TELEM_FMT, payload)
                    elif f_type == TYPE_CMD and f_len == struct.calcsize(CMD_FMT):
                        self.latest_command = struct.unpack(CMD_FMT, payload)
                    elif f_type == TYPE_MSG:
                        try:
                            self.latest_msg = payload.decode('ascii', errors='ignore')
                        except:
                            pass

    def get_telemetry(self):
        """Returns (seq, t_ms, vx, vy, marker, estop) or None."""
        with self._lock:
            return self.latest_telemetry

    def get_command(self, consume=True):
        """Returns (cmdSeq, cmd, estop) or None. Consumes by default."""
        with self._lock:
            val = self.latest_command
            if consume:
                self.latest_command = None
            return val

    def get_message(self, consume=True):
        """Returns the latest string message or None."""
        with self._lock:
            val = self.latest_msg
            if consume:
                self.latest_msg = None
            return val

    def send_telemetry(self, seq, t_ms, vx, vy, marker, estop):
        """Packs and sends a telemetry frame."""
        payload = struct.pack(TELEM_FMT, seq, t_ms, vx, vy, marker, estop)
        chk = self._chk_xor(TYPE_TELEM, len(payload), payload)
        self.ser.write(bytes([SOF, TYPE_TELEM, len(payload)]) + payload + bytes([chk]))
        self.ser.flush()

    def send_command(self, cmdSeq, cmd, estop):
        """Packs and sends a command frame."""
        payload = struct.pack(CMD_FMT, cmdSeq, cmd, estop)
        chk = self._chk_xor(TYPE_CMD, len(payload), payload)
        self.ser.write(bytes([SOF, TYPE_CMD, len(payload)]) + payload + bytes([chk]))
        self.ser.flush()

    def send_message(self, text: str):
        """Sends a raw string message (max 60 chars)."""
        payload = text[:60].encode('ascii', errors='ignore')
        chk = self._chk_xor(TYPE_MSG, len(payload), payload)
        self.ser.write(bytes([SOF, TYPE_MSG, len(payload)]) + payload + bytes([chk]))
        self.ser.flush()
