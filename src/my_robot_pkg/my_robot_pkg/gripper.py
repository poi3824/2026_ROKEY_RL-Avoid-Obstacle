"""OnRobot RG2 그리퍼 제어 — raw socket 기반 Modbus TCP (pymodbus 의존성 없음).

Register map (OnRobot RG protocol):
  0: target force (1/10 N)
  1: target width (1/10 mm)
  2: control        1(0x0001)=grip, 8(0x0008)=stop, 16(0x0010)=grip_w_offset
  268: status       bit 0 = busy          — 1 while motion ongoing, 0 when done
                    bit 1 = grip_detected — 1 when object gripped
"""
import socket
import struct
import time

_MODBUS_UNIT = 65
_STATUS_REG = 268
_CONTROL_ADDR = 0


class RG2Gripper:
    def __init__(self, ip, port=502, max_width=1100, max_force=400):
        self.ip = ip
        self.port = int(port)
        self.max_width = max_width
        self.max_force = max_force
        self._sock = None
        self._txid = 0
        self._connect()

    def _connect(self):
        try:
            sock = socket.create_connection((self.ip, self.port), timeout=2.0)
            sock.settimeout(1.0)
            self._sock = sock
        except Exception as e:
            self._sock = None
            print(f"[RG2Gripper] Modbus TCP connect failed: {e} — will retry on next call")

    def _next_txid(self):
        self._txid = (self._txid + 1) % 0x10000
        return self._txid

    def _read_holding_registers(self, address, count=1):
        for _ in range(2):
            if self._sock is None:
                self._connect()
            if self._sock is None:
                return None
            try:
                txid = self._next_txid()
                req = struct.pack('>HHHBBHH', txid, 0, 6, _MODBUS_UNIT, 3, address, count)
                self._sock.sendall(req)
                resp = self._sock.recv(256)
                if len(resp) >= 9 + 2 * count:
                    return struct.unpack(f'>{count}H', resp[9:9 + 2 * count])
            except Exception:
                self._sock = None
        return None

    def _write_multiple_registers(self, address, values):
        for _ in range(2):
            if self._sock is None:
                self._connect()
            if self._sock is None:
                return False
            try:
                txid = self._next_txid()
                count = len(values)
                pdu = struct.pack('>BHHB', 16, address, count, count * 2) + struct.pack(f'>{count}H', *values)
                header = struct.pack('>HHHB', txid, 0, len(pdu) + 1, _MODBUS_UNIT)
                self._sock.sendall(header + pdu)
                resp = self._sock.recv(256)
                return len(resp) >= 12
            except Exception:
                self._sock = None
        return False

    def get_status(self):
        """레지스터 268을 읽어 (busy, grip_detected)를 반환한다. 읽기 실패 시 (None, None)."""
        regs = self._read_holding_registers(_STATUS_REG, 1)
        if regs is None:
            return None, None
        status = regs[0]
        return bool(status & 0x01), bool(status & 0x02)

    def _command(self, force_val, width_val):
        return self._write_multiple_registers(_CONTROL_ADDR, [force_val, width_val, 16])

    def close_gripper(self, force_val=None):
        return self._command(force_val if force_val is not None else self.max_force, 0)

    def open_gripper(self, force_val=None):
        return self._command(force_val if force_val is not None else self.max_force, self.max_width)

    def wait_grip_done(self, timeout_sec=3.0, poll_interval=0.05):
        """모션이 끝날 때(busy=0)까지 대기하고, 그 시점의 grip_detected를 반환한다.

        Returns:
            (motion_done, grip_detected): motion_done=False면 timeout까지 busy가
            안 풀린 것이므로 grip_detected도 신뢰할 수 없어 False로 반환한다.
        """
        start = time.time()
        while time.time() - start < timeout_sec:
            busy, grip_detected = self.get_status()
            if busy is not None and not busy:
                return True, grip_detected
            time.sleep(poll_interval)
        return False, False

    def close_connection(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
