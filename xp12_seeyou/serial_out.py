"""Output writers: COM port (blocking or threaded) and console."""

import queue
import threading
import time
from typing import Callable, List, Optional, Tuple

from .core import DEFAULT_BAUDRATE, DEFAULT_COM_PORT

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    serial = None
    HAS_PYSERIAL = False

# Naviter dongle = Silicon Labs CP210x USB to UART bridge
NAVITER_DONGLE_VID = 0x10C4
NAVITER_DONGLE_PID = 0xEA60

Logger = Callable[[str], None]


def list_ports_detailed() -> List[Tuple[str, str, Optional[int], Optional[int]]]:
    """(device, description, vid, pid) for each COM port, sorted by name."""
    if not HAS_PYSERIAL:
        return []
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    return [(p.device, p.description, p.vid, p.pid) for p in ports]


def is_naviter_dongle(description: str, vid: Optional[int] = None, pid: Optional[int] = None) -> bool:
    return (vid == NAVITER_DONGLE_VID and pid == NAVITER_DONGLE_PID) or "CP210" in description


def find_naviter_dongle() -> Optional[str]:
    """COM port of the first Silicon Labs CP210x device, which is what the dongle uses."""
    for device, description, vid, pid in list_ports_detailed():
        if is_naviter_dongle(description, vid, pid):
            return device
    return None


class SerialWriter:
    """Writes NMEA sentences to the COM port of the Naviter dongle (blocking calls)."""

    RECONNECT_INTERVAL = 3.0

    def __init__(self, port: str = DEFAULT_COM_PORT, baudrate: int = DEFAULT_BAUDRATE,
                 log: Logger = print):
        self.port = port
        self.baudrate = baudrate
        self.log = log
        self.last_error: Optional[str] = None
        self._serial = None
        self._lock = threading.Lock()

    def start(self):
        self._connect()

    def _connect(self) -> bool:
        if not HAS_PYSERIAL:
            self._set_error("pyserial is not installed")
            return False
        try:
            self._serial = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=1, write_timeout=1,
            )
            self.last_error = None
            self.log(f"Serial port {self.port} opened at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            self._serial = None
            self._set_error(f"could not open {self.port}: {e}")
            return False

    def _set_error(self, message: str):
        # Log only when the error changes, so reconnect attempts don't flood the log
        if message != self.last_error:
            self.log(message)
        self.last_error = message

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def send_batch(self, sentences: List[str]) -> bool:
        """Write all sentences of one cycle in a single call, each ended with CR+LF."""
        if not self.is_connected() or not sentences:
            return False
        payload = "".join(s + "\r\n" for s in sentences).encode("ascii")
        with self._lock:
            try:
                self._serial.write(payload)
                return True
            except serial.SerialException as e:
                self._set_error(f"write error on {self.port}: {e}")
                self._close()
                return False

    def reconnect(self):
        with self._lock:
            self._close()
        self._connect()

    def _close(self):
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def stop(self):
        with self._lock:
            self._close()


class ThreadedSerialWriter:
    """
    SerialWriter on its own thread: submit() never blocks.

    Used inside X-Plane, where opening a port or writing at 4800 baud must not
    stall the simulator's frame. Only the newest cycles are kept if the port
    falls behind.
    """

    RECONNECT_INTERVAL = 3.0

    def __init__(self, port: str = DEFAULT_COM_PORT, baudrate: int = DEFAULT_BAUDRATE,
                 log: Logger = print, writer: Optional[SerialWriter] = None):
        self._writer = writer or SerialWriter(port, baudrate, log)
        self._queue: "queue.Queue[Optional[List[str]]]" = queue.Queue(maxsize=3)
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self.sentences_sent = 0

    @property
    def port(self) -> str:
        return self._writer.port

    @property
    def baudrate(self) -> int:
        return self._writer.baudrate

    @property
    def last_error(self) -> Optional[str]:
        return self._writer.last_error

    def is_connected(self) -> bool:
        return self._writer.is_connected()

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="seeyou-serial", daemon=True)
        self._thread.start()

    def submit(self, sentences: List[str]):
        if not self._running or not sentences:
            return
        try:
            self._queue.put_nowait(sentences)
        except queue.Full:
            try:
                self._queue.get_nowait()   # drop the oldest cycle
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(sentences)
            except queue.Full:
                pass

    def stop(self, timeout: float = 2.0):
        self._running = False
        try:
            self._queue.put_nowait(None)   # wake the thread
        except queue.Full:
            pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout)
        self._writer.stop()

    def _loop(self):
        last_attempt = 0.0
        while self._running:
            if not self._writer.is_connected():
                now = time.monotonic()
                if now - last_attempt >= self.RECONNECT_INTERVAL:
                    last_attempt = now
                    self._writer.reconnect()
            try:
                sentences = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if sentences is None:
                break
            if self._writer.send_batch(sentences):
                self.sentences_sent += len(sentences)


class ConsoleWriter:
    """Prints the sentences instead of writing to a COM port (--dry-run)."""

    RECONNECT_INTERVAL = 3.0

    def __init__(self):
        self.port = "console"
        self.baudrate = 0

    def start(self):
        print("Dry run: NMEA sentences are printed, no COM port is opened.")

    def is_connected(self) -> bool:
        return True

    def send_batch(self, sentences: List[str]) -> bool:
        for s in sentences:
            print(s)
        return bool(sentences)

    def reconnect(self):
        pass

    def stop(self):
        pass
