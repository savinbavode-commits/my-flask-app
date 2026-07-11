#!/usr/bin/env python3
"""
Modbus TCP Alarm Monitor  (ModScan-style)
=============================================
An open-source Python + Tkinter alternative to WinTech ModScan32, built on
the `pymodbus` library. Supports both Modbus TCP and Modbus RTU over a
serial/COM port.

Requirements
------------
    pip install "pymodbus>=3.12.1"        # 3.12.1+ has full Python 3.14 support
    pip install "pymodbus[serial]"        # adds pyserial, needed for COM-port mode
    pip install reportlab                 # only needed for "Save as PDF" in the Data Report window
    Tkinter ships with standard Python installers on Windows/macOS.
    On Debian/Ubuntu Linux: sudo apt-get install python3-tk

Run
---
    python modbus_master_alarm_monitor.py

Layout
------
- Menu bar: Connection (Connect.../Disconnect) | Settings (Protocol
  Selection...) | Data Format (Binary/Decimal/Integer/Hex, Alarm Monitor
  only) | Data Report (opens the Data Report window, see below) | Help
  (About).
- Data Report window: a ModScan-style logging report. Shows the live
  connection / Device Id / Modbus Type / Conversion (always "as per the
  ModScan panel's current selection"). Configure a Time Interval as a
  number plus a unit (Second(s)/Minute(s)/Hour(s)), a Max Rows limit
  (editable dropdown, default 200 - pick higher if you need longer
  logging runs), and up to 9 addresses (each with its own editable
  column label, defaulting to 40001-40009, plus a Decimal Places (0-6)
  spinbox for values whose raw register has no decimal point, e.g. 1234
  with 1 decimal place displays as 123.4). A free-text "Report Notes /
  Format" box lets you type anything you want to appear on the first
  page of the exported report. Press Start and one new row (Sl No.,
  Date, Time, then the 9 address values) is appended every interval,
  up to the configured Max Rows, in a scrollable table. While running,
  the address/decimal boxes and Max Rows are locked (Stop to unlock and
  edit them again). A connection LED plus "Interval set: ..." and "Next
  log in: MM:SS" (live countdown) show the configured interval and when
  the next row will be logged. The window can be minimized like a normal
  window. "Save as PDF" exports everything currently shown - connection/
  point-type details, interval, the Report Notes text, every configured
  address/label/decimal, and every logged row - to a PDF report.
  Stop/Clear Table are also provided. The 9 addresses are read
  continuously in the background (like the Alarm Monitor rows) so each
  logged row reflects a fresh value, not a stale one.
- Left panel ("ModScan" box): Start Address, Length, Device Id, Modbus
  Type, Conversion, No of Polls / Valid Slave Response + Reset Counter,
  and a live data table below.
- Middle panel: 20-row Alarm Monitor (Address / Value / Low Setpoint /
  High Setpoint / Alarm Count), its own Point Type selector, and its own
  No of Polls / Valid Slave Response box on the right of its header.
- Right panel: Event Log with "Save to TXT" and "Save as PDF" (a
  standalone alarm PDF report: connection summary, total/high/low alarm
  counts, the full 20-row Alarm Monitor table with each row's alarm count,
  and the complete Event Log).

Addressing (Modicon / ModScan-style)
--------------------------------------
Addresses you type are in the classic Modicon numbering ModScan itself
uses, based on the selected point type:
    Coil               00001 - 09999   (protocol address = typed - 1)
    Discrete Input     10001 - 19999   (protocol address = typed - 10001)
    Input Register     30001 - 39999   (protocol address = typed - 30001)
    Holding Register   40001 - 49999   (protocol address = typed - 40001)
Changing the left panel's Modbus Type automatically resets its Start
Address to that type's base (e.g. picking Holding Register sets Start
Address to 40001). The Alarm Monitor's per-row addresses are NOT
auto-reset when you change its Point Type, since each row is an
independent point you configure yourself - just make sure the address
you type matches whichever point type you have selected there.

Design notes / assumptions
---------------------------
- CONNECTED and DISCONNECTED events are now written to alarm_log.txt (and
  your chosen export file) with full date/time, same as alarm events.
- "About" now also shows where the Event Log is currently being saved to
  (or "not set" if you haven't used Save to TXT yet).
- The connection LED blinks while connected (alternates between two
  shades) and is solid red while disconnected. The app title also gets a
  light-green highlight while connected.
- Alarm Monitor's own No of Polls / Valid Slave Response count its
  individual per-row read attempts/successes (it polls on its own
  schedule, independently of the left panel's block scan).
- Menu label auto-updates (e.g. "Data Format: Hex") now look up their own
  index via `menubar.index(tk.END)` right after being added, instead of a
  hard-coded number - the earlier version's hard-coded index was the bug
  that caused "Settings" to get overwritten in your screenshot.
- UI clarity pass: base fonts were bumped up a size across the whole app,
  every table (Treeview) now uses an explicit, larger font and taller row
  height, and Windows DPI awareness is requested at startup so text stays
  crisp instead of blurry-upscaled on large/high-resolution monitors.
- The window now opens maximized to the full screen on launch (falls back
  to an exact screen-sized window if the platform has no "zoomed" state).
- Hover feedback: buttons brighten distinctly on hover/press, Treeview rows
  (both the ModScan live data table and the Data Report table) highlight
  under the mouse, the Alarm Monitor's Address/Value cells highlight on
  hover, and menu items keep their built-in highlight-on-hover behavior.
- The background GUI-queue poll interval was shortened from 150ms to 80ms
  so new readings/alarms land on screen sooner.

Testing note
------------
Written and syntax-checked in a sandboxed environment without network,
serial, or GUI access, so it could not be run end-to-end here. Please run
it locally and let me know if you hit any errors.
"""

import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ConnectionException

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    ModbusSerialClient = None

try:
    from pymodbus.framer import FramerType
    FRAMER_RTU = FramerType.RTU
    FRAMER_ASCII = FramerType.ASCII
except ImportError:
    # Older pymodbus releases used plain strings instead of the FramerType enum.
    FRAMER_RTU = "rtu"
    FRAMER_ASCII = "ascii"

# --------------------------------------------------------------------------
# Constants / theme
# --------------------------------------------------------------------------
NUM_ROWS = 20
ALARM_LOG_FILE = "alarm_log.txt"
REPORT_ADDR_COLS = 9
REPORT_MAX_ROWS_DEFAULT = 200
REPORT_MAX_ROWS_WARN_THRESHOLD = 20000
REPORT_DEFAULT_BASE_ADDR = 40001

POINT_TYPES = {
    "01: COIL": "co",
    "02: DISCRETE INPUT": "di",
    "03: HOLDING REG": "hr",
    "04: INPUT REG": "ir",
}
MODICON_BASE = {"co": 1, "di": 10001, "hr": 40001, "ir": 30001}
BIT_TYPES = ("co", "di")
DATA_FORMATS = ["Decimal", "Integer", "Hex", "Binary"]
TRANSMISSION_MODES = ["RTU", "ASCII"]
try:
    import winsound
    HAVE_WINSOUND = True
except ImportError:
    winsound = None
    HAVE_WINSOUND = False

COM_PORTS = ["COM1", "COM2", "COM3", "COM4", "COM5", "COM6"]
BAUD_RATES = ["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"]
WORD_LENGTHS = ["7", "8"]
PARITIES = {"None": "N", "Even": "E", "Odd": "O"}
STOP_BITS = ["1", "2"]

COLORS = {
    "bg": "#0d1117",
    "panel": "#141a22",
    "panel_alt": "#1c2530",
    "panel_alt2": "#212b38",
    "panel_raised": "#182029",
    "border": "#2d3844",
    "text": "#e6edf3",
    "text_dim": "#7d8b9a",
    "accent": "#2dd4bf",
    "accent2": "#f59e0b",
    "accent2_fg": "#241a05",
    "accent_dim": "#0f5c50",
    "accent_soft": "#123b37",
    "gold": "#f59e0b",
    "red": "#f85149",
    "red_dim": "#4c1319",
    "yellow": "#ffd33d",
    "yellow_dim": "#4a3c07",
    "warn_red": "#ff6b6b",
    "conn_green_bg": "#04261a",
    "conn_green_fg": "#3fb950",
    "log_bg": "#fbfbfa",
    "log_text": "#1b1f27",
    "log_alarm_high": "#c0263c",
    "log_alarm_low": "#8a6d00",
    "log_clear": "#1f8a4c",
    "log_info": "#555f6e",
    "btn_bg": "#0d9488",
    "btn_hover": "#20e8cf",
    "btn_accent_hover": "#ffd75e",
    "table_bg": "#10151c",
    "row_hover": "#233041",
    "row_hover_alt": "#2a3849",
    "menu_hover_fg": "#0d1117",
}

# Slightly larger, higher-contrast fonts than the original build so the UI
# stays crisp and readable on large / high-resolution monitors, not just
# on the small laptop screen it was first designed on.
FONT_UI = ("Segoe UI", 11)
FONT_UI_BOLD = ("Segoe UI", 11, "bold")
FONT_HEADER = ("Segoe UI", 17, "bold")
FONT_MONO = ("Consolas", 10)
MENU_FONT = ("Segoe UI", 12)
FONT_SMALL = ("Segoe UI", 10)
FONT_SMALL_BOLD = ("Segoe UI", 10, "bold")


def timestamp():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def enable_row_hover(tree, hover_bg=None):
    """Highlight whichever Treeview row is under the mouse, ModScan-style
    tables don't do this by default (only the selected row stands out),
    which makes it hard to tell what you're pointing at on a big monitor."""
    hover_bg = hover_bg or COLORS["row_hover"]
    tree.tag_configure("__hover__", background=hover_bg)
    state = {"row": None}

    def _clear():
        if state["row"] is not None:
            try:
                tags = [t for t in tree.item(state["row"], "tags") if t != "__hover__"]
                tree.item(state["row"], tags=tags)
            except tk.TclError:
                pass
            state["row"] = None

    def _on_motion(event):
        row = tree.identify_row(event.y)
        if row == state["row"]:
            return
        _clear()
        if row:
            tags = list(tree.item(row, "tags"))
            if "__hover__" not in tags:
                tags.append("__hover__")
                tree.item(row, tags=tags)
            state["row"] = row

    def _on_leave(_event):
        _clear()

    tree.bind("<Motion>", _on_motion, add="+")
    tree.bind("<Leave>", _on_leave, add="+")


def enable_widget_hover(widget, normal_bg, hover_bg):
    """Simple background swap for plain Tk widgets (Entry grids, etc.) that
    don't get a hover state for free the way ttk widgets do."""
    def _on_enter(_event):
        try:
            widget.configure(bg=hover_bg)
        except tk.TclError:
            pass

    def _on_leave(_event):
        try:
            widget.configure(bg=normal_bg)
        except tk.TclError:
            pass

    widget.bind("<Enter>", _on_enter, add="+")
    widget.bind("<Leave>", _on_leave, add="+")


def to_protocol_address(display_address, point_type):
    """Convert a Modicon-style address (e.g. 40007) into the 0-based
    address the Modbus protocol actually uses on the wire."""
    base = MODICON_BASE.get(point_type, 1)
    return max(0, display_address - base)


def format_value(raw_value, point_type, data_format):
    if point_type in BIT_TYPES:
        return "ON" if raw_value else "OFF"
    raw_value = int(raw_value) & 0xFFFF
    if data_format == "Decimal":
        return str(raw_value)
    if data_format == "Integer":
        signed = raw_value - 0x10000 if raw_value >= 0x8000 else raw_value
        return str(signed)
    if data_format == "Hex":
        return f"0x{raw_value:04X}"
    if data_format == "Binary":
        return format(raw_value, "016b")
    return str(raw_value)


# --------------------------------------------------------------------------
# Background polling thread
# --------------------------------------------------------------------------
class PollerThread(threading.Thread):
    def __init__(self, conn_params, timeout_ms, get_scan_params, get_alarm_params, result_queue,
                 transmission_mode="RTU", get_report_params=None):
        super().__init__(daemon=True)
        self.conn_params = conn_params
        self.timeout_ms = timeout_ms
        self.transmission_mode = transmission_mode
        # callable -> (protocol_address, length, device_id, point_type, delay_ms, display_address)
        self.get_scan_params = get_scan_params
        # callable -> (point_type, {row_idx: (protocol_addr, display_addr)})
        self.get_alarm_params = get_alarm_params
        # optional callable -> (point_type, {col_idx: (protocol_addr, display_addr)}) used by the
        # Data Report window; None (or an empty dict) means "nothing to read for the report".
        self.get_report_params = get_report_params
        self.result_queue = result_queue
        self._stop_event = threading.Event()
        self.client = None

    def stop(self):
        self._stop_event.set()

    def run(self):
        timeout_s = max(0.2, self.timeout_ms / 1000.0)
        try:
            if self.conn_params["mode"] == "tcp":
                self.client = ModbusTcpClient(self.conn_params["host"], port=self.conn_params["port"],
                                               timeout=timeout_s)
            else:
                if ModbusSerialClient is None:
                    raise RuntimeError('Serial support not installed - run: pip install "pymodbus[serial]"')
                framer = FRAMER_ASCII if self.transmission_mode == "ASCII" else FRAMER_RTU
                try:
                    self.client = ModbusSerialClient(
                        port=self.conn_params["port"], framer=framer, baudrate=self.conn_params["baudrate"],
                        bytesize=self.conn_params["bytesize"], parity=self.conn_params["parity"],
                        stopbits=self.conn_params["stopbits"], timeout=timeout_s,
                    )
                except TypeError:
                    # Fallback for pymodbus versions that still expect method= instead of framer=
                    method = "ascii" if self.transmission_mode == "ASCII" else "rtu"
                    self.client = ModbusSerialClient(
                        port=self.conn_params["port"], method=method, baudrate=self.conn_params["baudrate"],
                        bytesize=self.conn_params["bytesize"], parity=self.conn_params["parity"],
                        stopbits=self.conn_params["stopbits"], timeout=timeout_s,
                    )
        except Exception as exc:
            self.result_queue.put({"type": "disconnected", "error": f"Could not create client: {exc}"})
            return

        if not self.client.connect():
            self.result_queue.put({"type": "disconnected", "error": f"Could not connect to {self._conn_desc()}"})
            return
        self.result_queue.put({"type": "connected"})

        consecutive_errors = 0
        MAX_SLOW_ERRORS = 3  # tolerate a couple of odd timeouts before declaring dead
        while not self._stop_event.is_set():
            protocol_address, length, device_id, scan_point_type, delay_ms, display_address = self.get_scan_params()

            try:
                values = self._read_block(scan_point_type, protocol_address, length, device_id)
                consecutive_errors = 0
                self.result_queue.put({"type": "block", "ok": True, "point_type": scan_point_type,
                                        "display_start": display_address, "values": values})
            except Exception as exc:
                self.result_queue.put({"type": "block", "ok": False, "error": str(exc)})

                # ModScan-style instant detection: if the transport itself
                # is gone (peer closed the socket / serial port yanked /
                # pymodbus raised its own ConnectionException, or the
                # client's own socket-open check now says False), there is
                # no point waiting for repeated timeouts - declare
                # disconnected immediately, same poll cycle it happens on.
                transport_dead = isinstance(exc, (ConnectionException, OSError))
                try:
                    if not transport_dead and not self.client.is_socket_open():
                        transport_dead = True
                except Exception:
                    transport_dead = True

                if transport_dead:
                    self.result_queue.put({"type": "disconnected", "error": f"Lost connection: {exc}"})
                    self._cleanup()
                    return

                consecutive_errors += 1
                if consecutive_errors >= MAX_SLOW_ERRORS:
                    self.result_queue.put({"type": "disconnected", "error": f"Lost connection: {exc}"})
                    self._cleanup()
                    return

            alarm_point_type, alarm_addrs = self.get_alarm_params()
            for row_idx, (protocol_addr, display_addr) in alarm_addrs.items():
                if self._stop_event.is_set():
                    break
                try:
                    value = self._read_one(alarm_point_type, protocol_addr, device_id)
                    self.result_queue.put({"type": "value", "row": row_idx, "address": display_addr,
                                            "value": value, "ok": True, "point_type": alarm_point_type})
                except Exception as exc:
                    self.result_queue.put({"type": "value", "row": row_idx, "address": display_addr,
                                            "ok": False, "error": str(exc), "point_type": alarm_point_type})

            # Data Report window: keeps its columns' latest live values fresh,
            # exactly like the Alarm Monitor rows above, using whichever
            # addresses/point type are currently configured in that window's
            # top address boxes. Report row *generation* is timed separately
            # (in the GUI thread, based on the report's own minute interval);
            # this just guarantees a fresh value is ready whenever a row fires.
            if self.get_report_params is not None:
                try:
                    report_point_type, report_addrs = self.get_report_params()
                except Exception:
                    report_point_type, report_addrs = None, {}
                for col_idx, (protocol_addr, display_addr) in report_addrs.items():
                    if self._stop_event.is_set():
                        break
                    try:
                        value = self._read_one(report_point_type, protocol_addr, device_id)
                        self.result_queue.put({"type": "report_value", "col": col_idx, "address": display_addr,
                                                "value": value, "ok": True, "point_type": report_point_type})
                    except Exception as exc:
                        self.result_queue.put({"type": "report_value", "col": col_idx, "address": display_addr,
                                                "ok": False, "error": str(exc), "point_type": report_point_type})

            slept, interval_s = 0.0, max(0.02, delay_ms / 1000.0)
            while slept < interval_s and not self._stop_event.is_set():
                time.sleep(0.02)
                slept += 0.02

        self._cleanup()

    def _conn_desc(self):
        if self.conn_params["mode"] == "tcp":
            return f"{self.conn_params['host']}:{self.conn_params['port']}"
        return f"{self.conn_params['port']} @ {self.conn_params['baudrate']} baud"

    def _read_block(self, point_type, address, length, device_id):
        if point_type == "hr":
            rr = self.client.read_holding_registers(address, count=length, slave=device_id)
        elif point_type == "ir":
            rr = self.client.read_input_registers(address, count=length, slave=device_id)
        elif point_type == "co":
            rr = self.client.read_coils(address, count=length, slave=device_id)
        else:
            rr = self.client.read_discrete_inputs(address, count=length, slave=device_id)
        if rr.isError():
            raise RuntimeError(str(rr))
        return list(rr.registers) if point_type in ("hr", "ir") else list(rr.bits)[:length]

    def _read_one(self, point_type, address, device_id):
        if point_type == "hr":
            rr = self.client.read_holding_registers(address, count=1, slave=device_id)
        elif point_type == "ir":
            rr = self.client.read_input_registers(address, count=1, slave=device_id)
        elif point_type == "co":
            rr = self.client.read_coils(address, count=1, slave=device_id)
        else:
            rr = self.client.read_discrete_inputs(address, count=1, slave=device_id)
        if rr.isError():
            raise RuntimeError(str(rr))
        return rr.registers[0] if point_type in ("hr", "ir") else rr.bits[0]

    def _cleanup(self):
        try:
            if self.client:
                self.client.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# Popup dialogs
# --------------------------------------------------------------------------
class ConnectionDialog(tk.Toplevel):
    def __init__(self, parent, defaults, on_connect):
        super().__init__(parent)
        self.title("Connection")
        self.configure(bg=COLORS["panel"])
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()

        pad = {"padx": 10, "pady": 6}
        LBL = {"bg": COLORS["panel"], "fg": COLORS["text"], "font": FONT_UI}

        tk.Label(self, text="Connect Using:", **LBL).grid(row=0, column=0, sticky="w", **pad)
        self.mode_var = tk.StringVar(value=defaults.get("mode_label", "TCP/IP"))
        mode_combo = ttk.Combobox(self, textvariable=self.mode_var, values=["TCP/IP", "Serial (COM Port)"],
                                   width=20, state="readonly")
        mode_combo.grid(row=0, column=1, **pad)
        mode_combo.bind("<<ComboboxSelected>>", lambda e: self._toggle_mode())

        self.tcp_frame = tk.Frame(self, bg=COLORS["panel"])
        self.host_var = tk.StringVar(value=defaults.get("host", "127.0.0.1"))
        self.port_var = tk.StringVar(value=str(defaults.get("port", 502)))
        tk.Label(self.tcp_frame, text="IP Address:", **LBL).grid(row=0, column=0, sticky="w", **pad)
        tk.Entry(self.tcp_frame, textvariable=self.host_var, width=18).grid(row=0, column=1, **pad)
        tk.Label(self.tcp_frame, text="Port Number:", **LBL).grid(row=1, column=0, sticky="w", **pad)
        tk.Entry(self.tcp_frame, textvariable=self.port_var, width=18).grid(row=1, column=1, **pad)

        self.serial_frame = tk.Frame(self, bg=COLORS["panel"])
        self.com_var = tk.StringVar(value=defaults.get("com_port", "COM1"))
        self.baud_var = tk.StringVar(value=defaults.get("baudrate", "9600"))
        self.wordlen_var = tk.StringVar(value=defaults.get("bytesize", "8"))
        self.parity_var = tk.StringVar(value=defaults.get("parity_label", "None"))
        self.stopbits_var = tk.StringVar(value=defaults.get("stopbits", "1"))

        tk.Label(self.serial_frame, text="COM Port:", **LBL).grid(row=0, column=0, sticky="w", **pad)
        ttk.Combobox(self.serial_frame, textvariable=self.com_var, values=COM_PORTS, width=16).grid(
            row=0, column=1, **pad)
        tk.Label(self.serial_frame, text="Baud Rate:", **LBL).grid(row=1, column=0, sticky="w", **pad)
        ttk.Combobox(self.serial_frame, textvariable=self.baud_var, values=BAUD_RATES, width=16,
                     state="readonly").grid(row=1, column=1, **pad)
        tk.Label(self.serial_frame, text="Word Length:", **LBL).grid(row=2, column=0, sticky="w", **pad)
        ttk.Combobox(self.serial_frame, textvariable=self.wordlen_var, values=WORD_LENGTHS, width=16,
                     state="readonly").grid(row=2, column=1, **pad)
        tk.Label(self.serial_frame, text="Parity:", **LBL).grid(row=3, column=0, sticky="w", **pad)
        ttk.Combobox(self.serial_frame, textvariable=self.parity_var, values=list(PARITIES.keys()), width=16,
                     state="readonly").grid(row=3, column=1, **pad)
        tk.Label(self.serial_frame, text="Stop Bits:", **LBL).grid(row=4, column=0, sticky="w", **pad)
        ttk.Combobox(self.serial_frame, textvariable=self.stopbits_var, values=STOP_BITS, width=16,
                     state="readonly").grid(row=4, column=1, **pad)

        self.tcp_frame.grid(row=1, column=0, columnspan=2)
        self._toggle_mode()

        btns = tk.Frame(self, bg=COLORS["panel"])
        btns.grid(row=2, column=0, columnspan=2, pady=(4, 10))
        ttk.Button(btns, text="OK", style="Accent.TButton",
                   command=lambda: self._ok(on_connect)).pack(side=tk.LEFT, padx=6)
        ttk.Button(btns, text="Cancel", command=self.destroy).pack(side=tk.LEFT, padx=6)

    def _toggle_mode(self):
        if self.mode_var.get() == "TCP/IP":
            self.serial_frame.grid_forget()
            self.tcp_frame.grid(row=1, column=0, columnspan=2)
        else:
            self.tcp_frame.grid_forget()
            self.serial_frame.grid(row=1, column=0, columnspan=2)

    def _ok(self, on_connect):
        if self.mode_var.get() == "TCP/IP":
            try:
                port = int(self.port_var.get().strip())
            except ValueError:
                messagebox.showerror("Invalid port", "Port must be an integer.", parent=self)
                return
            params = {"mode": "tcp", "mode_label": "TCP/IP", "host": self.host_var.get().strip(), "port": port}
        else:
            if ModbusSerialClient is None:
                messagebox.showerror(
                    "Serial support missing",
                    'Serial mode needs pyserial. Run: pip install "pymodbus[serial]"', parent=self)
                return
            try:
                baud = int(self.baud_var.get())
                wordlen = int(self.wordlen_var.get())
                stopbits = int(self.stopbits_var.get())
            except ValueError:
                messagebox.showerror("Invalid input", "Baud/Word Length/Stop Bits must be numeric.", parent=self)
                return
            params = {
                "mode": "serial", "mode_label": "Serial (COM Port)",
                "com_port": self.com_var.get().strip(), "port": self.com_var.get().strip(),
                "baudrate": baud, "bytesize": wordlen,
                "parity_label": self.parity_var.get(), "parity": PARITIES[self.parity_var.get()],
                "stopbits": stopbits,
            }
        self.destroy()
        on_connect(params)


class ProtocolSelectionDialog(tk.Toplevel):
    def __init__(self, parent, timeout_ms, delay_ms, transmission_mode, on_apply):
        super().__init__(parent)
        self.title("Protocol Selection")
        self.configure(bg=COLORS["panel"])
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()

        pad = {"padx": 10, "pady": 6}
        LBL = {"bg": COLORS["panel"], "fg": COLORS["text"], "font": FONT_UI}
        self.timeout_var = tk.StringVar(value=str(timeout_ms))
        self.delay_var = tk.StringVar(value=str(delay_ms))
        self.transmission_mode_var = tk.StringVar(value=transmission_mode)

        tk.Label(self, text="Slave Response Timeout (ms):", **LBL).grid(row=0, column=0, sticky="w", **pad)
        tk.Entry(self, textvariable=self.timeout_var, width=10).grid(row=0, column=1, **pad)
        tk.Label(self, text="Delay Between Polls (ms):", **LBL).grid(row=1, column=0, sticky="w", **pad)
        tk.Entry(self, textvariable=self.delay_var, width=10).grid(row=1, column=1, **pad)

        tk.Label(self, text="Transmission Mode:", **LBL).grid(row=2, column=0, sticky="w", **pad)
        mode_combo = ttk.Combobox(self, textvariable=self.transmission_mode_var, values=TRANSMISSION_MODES,
                                   width=8, state="readonly")
        mode_combo.grid(row=2, column=1, sticky="w", **pad)

        tk.Label(self, text="Timeout applies next time you Connect.\nDelay applies immediately.\n"
                             "Transmission Mode (RTU/ASCII) applies to Serial connections\n"
                             "and takes effect next time you Connect.",
                 bg=COLORS["panel"], fg=COLORS["text_dim"], font=("Segoe UI", 10, "italic"), justify="left").grid(
            row=3, column=0, columnspan=2, sticky="w", padx=10, pady=(0, 6))

        btns = tk.Frame(self, bg=COLORS["panel"])
        btns.grid(row=4, column=0, columnspan=2, pady=(4, 10))
        ttk.Button(btns, text="OK", style="Accent.TButton",
                   command=lambda: self._ok(on_apply)).pack(side=tk.LEFT, padx=6)
        ttk.Button(btns, text="Cancel", command=self.destroy).pack(side=tk.LEFT, padx=6)

    def _ok(self, on_apply):
        try:
            timeout_ms = max(50, int(self.timeout_var.get().strip()))
            delay_ms = max(20, int(self.delay_var.get().strip()))
        except ValueError:
            messagebox.showerror("Invalid input", "Both values must be integers.", parent=self)
            return
        transmission_mode = self.transmission_mode_var.get()
        self.destroy()
        on_apply(timeout_ms, delay_ms, transmission_mode)


class DataReportWindow(tk.Toplevel):
    """ModScan-style 'Data Report' window.

    - Top info bar shows the live connection / Device Id / Modbus Type /
      Conversion, always taken from the main ModScan panel (so report
      values are always read using 'whatever the ModScan screen has
      selected', per spec).
    - A Time Interval (minutes) box plus Start/Stop controls.
    - Nine editable address boxes, each with an editable report-label box
      right below it (this label doubles as that column's table heading).
      Defaults to 40001-40009. Locked while logging is running (Stop to
      unlock and edit them again).
    - A scrollable table: Sl No. | Date | Time | <9 address columns>,
      capped at a user-selectable Max Rows (default REPORT_MAX_ROWS_DEFAULT,
      up to REPORT_MAX_ROWS_WARN_THRESHOLD before a "may be slow" warning) -
      logging auto-stops once that many rows are logged.
    - "Save as PDF" exports the connection/point-type details, interval,
      every configured address/label, and every logged row to a PDF.

    Actual register reads happen continuously in the background
    PollerThread (see ModbusMasterApp._get_report_params), the same way
    the Alarm Monitor rows are read; this window just samples the latest
    cached value for each configured column every time its own interval
    timer fires, and turns that into one more table row.
    """

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.title("Modscan Data Report")
        self.configure(bg=COLORS["bg"])
        self.geometry("1360x600")
        self.minsize(1100, 480)
        # NOTE: deliberately NOT calling self.transient(parent) - transient
        # child windows lose their minimize button on Windows, and this
        # window needs one so it can be minimized while logging runs.
        # grab_set() is also never used, so the main window stays usable.

        self.running = False
        self.timer_job = None
        self.info_job = None
        self.countdown_job = None
        self.interval_ms = 60000
        self.interval_seconds = 60
        self.next_log_ts = None
        self.max_rows = REPORT_MAX_ROWS_DEFAULT
        self.serial_no = 0

        self._build_ui()
        self._refresh_conn_info()
        self._info_refresh_loop()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------
    def _build_ui(self):
        tk.Label(self, text="MODSCAN DATA REPORT", bg=COLORS["bg"], fg=COLORS["accent2"],
                 font=FONT_HEADER).pack(side=tk.TOP, pady=(10, 6))

        # ---- connection / point-type info bar (read-only, live) ----
        info_box = tk.Frame(self, bg=COLORS["panel"], highlightbackground=COLORS["border"],
                             highlightthickness=1)
        info_box.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(0, 8))
        self.info_var = tk.StringVar(value="")
        tk.Label(info_box, textvariable=self.info_var, bg=COLORS["panel"], fg=COLORS["text"],
                 font=FONT_UI, justify="left", padx=10, pady=6).pack(side=tk.LEFT)

        # ---- interval + start/stop/clear controls ----
        ctrl = tk.Frame(self, bg=COLORS["bg"])
        ctrl.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(0, 8))
        tk.Label(ctrl, text="Time Interval:", bg=COLORS["bg"], fg=COLORS["text"],
                 font=FONT_UI).pack(side=tk.LEFT)
        self.interval_var = tk.StringVar(value="1")
        tk.Entry(ctrl, textvariable=self.interval_var, width=8, justify="center",
                 bg=COLORS["panel_alt"], fg=COLORS["text"], insertbackground=COLORS["text"],
                 relief="flat", highlightthickness=1, highlightbackground=COLORS["border"],
                 highlightcolor=COLORS["accent"]).pack(side=tk.LEFT, padx=(6, 4))
        self.interval_unit_var = tk.StringVar(value="Minute(s)")
        ttk.Combobox(ctrl, textvariable=self.interval_unit_var, state="readonly", width=10,
                     values=["Second(s)", "Minute(s)", "Hour(s)"]).pack(side=tk.LEFT, padx=(0, 18))

        tk.Label(ctrl, text="Max Rows:", bg=COLORS["bg"], fg=COLORS["text"],
                 font=FONT_UI).pack(side=tk.LEFT)
        self.max_rows_var = tk.StringVar(value=str(REPORT_MAX_ROWS_DEFAULT))
        self.max_rows_combo = ttk.Combobox(
            ctrl, textvariable=self.max_rows_var, state="normal", width=8,
            values=["50", "100", "200", "500", "1000", "2000", "5000", "10000", "20000"])
        self.max_rows_combo.pack(side=tk.LEFT, padx=(6, 18))

        self.start_btn = ttk.Button(ctrl, text="Start", style="Accent.TButton", command=self._start)
        self.start_btn.pack(side=tk.LEFT, padx=4)
        self.stop_btn = ttk.Button(ctrl, text="Stop", command=self._stop, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="Clear Table", command=self._clear_table).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="Save as PDF", command=self._save_pdf).pack(side=tk.LEFT, padx=4)

        self.status_var = tk.StringVar(value="Stopped.")
        tk.Label(ctrl, textvariable=self.status_var, bg=COLORS["bg"], fg=COLORS["text_dim"],
                 font=("Segoe UI", 11, "italic")).pack(side=tk.LEFT, padx=16)

        # ---- second status row: connection LED + configured interval + live countdown ----
        status_row = tk.Frame(self, bg=COLORS["panel"], highlightbackground=COLORS["border"],
                               highlightthickness=1)
        status_row.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(0, 8))

        led_box = tk.Frame(status_row, bg=COLORS["panel"])
        led_box.pack(side=tk.LEFT, padx=(10, 4), pady=6)
        self.rep_led_canvas = tk.Canvas(led_box, width=18, height=18, bg=COLORS["panel"],
                                         highlightthickness=0)
        self.rep_led_canvas.pack(side=tk.LEFT)
        self.rep_led_ring = self.rep_led_canvas.create_oval(2, 2, 16, 16, outline=COLORS["red"], width=2, fill="")
        self.rep_led_dot = self.rep_led_canvas.create_oval(6, 6, 12, 12, fill=COLORS["red"], outline="")
        self.rep_conn_text_var = tk.StringVar(value="DISCONNECTED")
        self.rep_conn_label = tk.Label(led_box, textvariable=self.rep_conn_text_var, bg=COLORS["panel"],
                                        fg=COLORS["red"], font=FONT_UI_BOLD, padx=6)
        self.rep_conn_label.pack(side=tk.LEFT)

        self.interval_info_var = tk.StringVar(value="Interval set: -")
        tk.Label(status_row, textvariable=self.interval_info_var, bg=COLORS["panel"],
                 fg=COLORS["accent"], font=FONT_UI_BOLD, padx=10).pack(side=tk.LEFT)

        self.countdown_var = tk.StringVar(value="Next log in: -")
        tk.Label(status_row, textvariable=self.countdown_var, bg=COLORS["panel"],
                 fg=COLORS["accent2"], font=FONT_UI_BOLD, padx=10).pack(side=tk.LEFT)

        # ---- address configuration header: 9 columns, each with an
        # editable address box on top and an editable report-label box
        # below it (this mirrors the layout in the reference screenshot) ----
        addr_frame = tk.Frame(self, bg=COLORS["panel"], highlightbackground=COLORS["border"],
                               highlightthickness=1)
        addr_frame.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(0, 4))

        fixed_kw = dict(bg=COLORS["panel_raised"], fg=COLORS["accent2"], font=FONT_UI_BOLD,
                         justify="center")
        tk.Label(addr_frame, text="Sl No.", width=8, **fixed_kw).grid(
            row=0, column=0, rowspan=3, sticky="nsew", padx=1, pady=1)
        tk.Label(addr_frame, text="Date", width=12, **fixed_kw).grid(
            row=0, column=1, rowspan=3, sticky="nsew", padx=1, pady=1)
        tk.Label(addr_frame, text="Time", width=10, **fixed_kw).grid(
            row=0, column=2, rowspan=3, sticky="nsew", padx=1, pady=1)

        self.address_vars = []
        self.label_vars = []
        self.decimal_vars = []
        self.address_entries = []
        self.decimal_entries = []
        for c in range(REPORT_ADDR_COLS):
            addr_var = tk.StringVar(value=str(REPORT_DEFAULT_BASE_ADDR + c))
            label_var = tk.StringVar(value=f"Addr {c + 1}")
            decimal_var = tk.StringVar(value="0")

            addr_entry = tk.Entry(addr_frame, textvariable=addr_var, width=10, justify="center",
                                   bg=COLORS["panel_alt"], fg=COLORS["accent"],
                                   insertbackground=COLORS["text"], relief="flat")
            addr_entry.grid(row=0, column=3 + c, sticky="nsew", padx=1, pady=1)

            label_entry = tk.Entry(addr_frame, textvariable=label_var, width=10, justify="center",
                                    bg=COLORS["panel_alt2"], fg=COLORS["text"],
                                    insertbackground=COLORS["text"], relief="flat")
            label_entry.grid(row=1, column=3 + c, sticky="nsew", padx=1, pady=1)
            label_var.trace_add("write", lambda *_a, idx=c: self._on_label_change(idx))

            decimal_entry = ttk.Spinbox(addr_frame, textvariable=decimal_var, from_=0, to=6, width=8,
                                         justify="center")
            decimal_entry.grid(row=2, column=3 + c, sticky="nsew", padx=1, pady=1)

            self.address_vars.append(addr_var)
            self.label_vars.append(label_var)
            self.decimal_vars.append(decimal_var)
            self.address_entries.append(addr_entry)
            self.decimal_entries.append(decimal_entry)

        tk.Label(addr_frame,
                 text="\u2191 Address (editable, Modicon-style e.g. 40001)      "
                      "Report column label (editable)      "
                      "\u2193 Decimal places (0-6, for values with no decimal point)",
                 bg=COLORS["panel"], fg=COLORS["text_dim"], font=("Segoe UI", 10, "italic")).grid(
            row=3, column=0, columnspan=3 + REPORT_ADDR_COLS, sticky="w", padx=6, pady=(2, 4))

        # ---- free-text report notes/format box - shown on the PDF's first page ----
        notes_frame = tk.Frame(self, bg=COLORS["panel"], highlightbackground=COLORS["border"],
                                highlightthickness=1)
        notes_frame.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(0, 8))
        tk.Label(notes_frame, text="Report Notes / Format (appears on the report's first page):",
                 bg=COLORS["panel"], fg=COLORS["accent2"], font=FONT_UI_BOLD).pack(
            side=tk.TOP, anchor="w", padx=8, pady=(6, 2))
        self.notes_text = tk.Text(notes_frame, height=4, wrap="word", bg=COLORS["panel_alt"],
                                   fg=COLORS["text"], insertbackground=COLORS["text"], relief="flat",
                                   font=FONT_UI, padx=8, pady=6)
        self.notes_text.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 8))

        # ---- scrollable data table ----
        table_frame = tk.Frame(self, bg=COLORS["panel"])
        table_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=14, pady=(0, 12))

        columns = ["sl", "date", "time"] + [f"c{c}" for c in range(REPORT_ADDR_COLS)]
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=18)
        self.tree.heading("sl", text="Sl No.")
        self.tree.column("sl", width=55, anchor=tk.CENTER)
        self.tree.heading("date", text="Date")
        self.tree.column("date", width=95, anchor=tk.CENTER)
        self.tree.heading("time", text="Time")
        self.tree.column("time", width=85, anchor=tk.CENTER)
        for c in range(REPORT_ADDR_COLS):
            self.tree.heading(f"c{c}", text=self.label_vars[c].get())
            self.tree.column(f"c{c}", width=90, anchor=tk.CENTER)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        enable_row_hover(self.tree)

        vscroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.tree.yview)
        vscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.configure(yscrollcommand=vscroll.set)

        self.hint_var = tk.StringVar(
            value=f"Logs up to {self.max_rows} rows per run (scroll to see earlier rows); "
                  f"click Clear Table to start a fresh batch.")
        tk.Label(self, textvariable=self.hint_var, bg=COLORS["bg"], fg=COLORS["text_dim"],
                 font=("Segoe UI", 10, "italic")).pack(side=tk.TOP, pady=(0, 8))

    # ---------------------------------------------------------------
    def _on_label_change(self, idx):
        try:
            self.tree.heading(f"c{idx}", text=self.label_vars[idx].get().strip() or f"Addr {idx + 1}")
        except tk.TclError:
            pass

    def _refresh_conn_info(self):
        params = self.app.conn_params
        summary = self.app._connection_summary_text(params) if params else "-"
        device_id = self.app.device_id_var.get().strip() or "?"
        modbus_type = self.app.point_type_var.get()
        conversion = self.app.conversion_var.get()
        conn_state = "CONNECTED" if self.app.connected else "DISCONNECTED"
        self.info_var.set(
            f"IP/Connection: {summary}   [{conn_state}]        Device Id (Slave Id): {device_id}\n"
            f"Modbus Type: {modbus_type}   (as per ModScan selection)        "
            f"Conversion: {conversion}   (as per ModScan selection)"
        )
        led_color = COLORS["accent2"] if self.app.connected else COLORS["red"]
        self.rep_led_canvas.itemconfig(self.rep_led_ring, outline=led_color)
        self.rep_led_canvas.itemconfig(self.rep_led_dot, fill=led_color)
        self.rep_conn_text_var.set(conn_state)
        self.rep_conn_label.configure(fg=led_color)

    def _info_refresh_loop(self):
        if not self.winfo_exists():
            return
        self._refresh_conn_info()
        self.info_job = self.after(1000, self._info_refresh_loop)

    # ---------------------------------------------------------------
    def _start(self):
        if self.running:
            return
        raw_interval = self.interval_var.get().strip()
        try:
            value = float(raw_interval)
            if value <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid interval", "Time Interval must be a positive number.",
                                  parent=self)
            return
        unit = self.interval_unit_var.get()
        if unit.startswith("Second"):
            seconds = value
        elif unit.startswith("Hour"):
            seconds = value * 3600
        else:
            seconds = value * 60
        if not any(v.get().strip() for v in self.address_vars):
            messagebox.showerror("No addresses", "Please enter at least one address in the boxes above "
                                                   "before starting.", parent=self)
            return

        raw_rows = self.max_rows_var.get().strip()
        try:
            max_rows = int(raw_rows)
            if max_rows <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid Max Rows", "Max Rows must be a positive whole number.",
                                  parent=self)
            return
        if max_rows > REPORT_MAX_ROWS_WARN_THRESHOLD:
            if not messagebox.askyesno(
                    "Large row count",
                    f"{max_rows} rows may make the table and 'Save as PDF' feel sluggish on some "
                    f"PCs.\nContinue anyway?", parent=self):
                return

        for c, dec_var in enumerate(self.decimal_vars):
            try:
                dec = int(dec_var.get().strip())
                if dec < 0 or dec > 6:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Invalid Decimal Places",
                                      f"Decimal places for column {c + 1} must be a whole number 0-6.",
                                      parent=self)
                return

        if not self.app.connected:
            if not messagebox.askyesno(
                    "Not connected",
                    "The device is not currently connected.\nStart logging anyway? "
                    "(Rows will show ERR until a connection is established.)", parent=self):
                return

        self.max_rows = max_rows
        self.hint_var.set(f"Logs up to {self.max_rows} rows per run (scroll to see earlier rows); "
                           f"click Clear Table to start a fresh batch.")
        self.interval_seconds = seconds
        self.interval_ms = max(200, int(seconds * 1000))
        self.running = True
        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)
        self.max_rows_combo.configure(state=tk.DISABLED)
        for entry in self.address_entries:
            entry.configure(state=tk.DISABLED)
        for entry in self.decimal_entries:
            entry.configure(state=tk.DISABLED)
        self.interval_info_var.set(f"Interval set: {value:g} {unit}")
        self.status_var.set(f"Running - logging every {value:g} {unit}.")
        self._log_row()
        self._schedule_next()

    def _schedule_next(self):
        if not self.running:
            return
        self.next_log_ts = time.time() + self.interval_seconds
        self.timer_job = self.after(self.interval_ms, self._tick)
        self._update_countdown()

    def _update_countdown(self):
        if not self.running or self.next_log_ts is None:
            self.countdown_var.set("Next log in: -")
            return
        remaining = max(0.0, self.next_log_ts - time.time())
        self.countdown_var.set(f"Next log in: {self._format_duration(remaining)}")
        self.countdown_job = self.after(1000, self._update_countdown)

    @staticmethod
    def _format_duration(seconds):
        total = int(round(seconds))
        h, rem = divmod(total, 3600)
        m, s = divmod(rem, 60)
        if h > 0:
            return f"{h:02d}:{m:02d}:{s:02d}"
        return f"{m:02d}:{s:02d}"

    def _tick(self):
        if not self.running:
            return
        self._log_row()
        if len(self.tree.get_children()) >= self.max_rows:
            self._stop(reason=f"Reached {self.max_rows} rows - logging stopped automatically.")
            return
        self._schedule_next()

    def _column_decimals(self, col_idx):
        try:
            dec = int(self.decimal_vars[col_idx].get().strip())
            return max(0, min(6, dec))
        except (ValueError, IndexError):
            return 0

    @staticmethod
    def _apply_decimals(value_text, decimals):
        """Takes the plain formatted value (as produced by the shared
        format_value() used everywhere else in the app) and, only for
        decimals > 0, re-renders it as a scaled decimal number - e.g. a raw
        register value of 1234 with 1 decimal place becomes 123.4. Values
        that aren't plain numbers (ON/OFF, hex, binary) are left untouched."""
        if decimals <= 0:
            return value_text
        try:
            num = float(value_text)
        except (TypeError, ValueError):
            return value_text
        scaled = num / (10 ** decimals)
        return f"{scaled:.{decimals}f}"

    def _log_row(self):
        self.serial_no += 1
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H:%M:%S")

        fallback_point_type = POINT_TYPES[self.app.point_type_var.get()]
        data_format = self.app.conversion_var.get()

        values = []
        for c, addr_var in enumerate(self.address_vars):
            if addr_var.get().strip() == "":
                values.append("")
                continue
            entry = self.app.report_current.get(c)
            if entry is None:
                values.append("...")
            elif not entry.get("ok"):
                values.append("ERR")
            else:
                pt = entry.get("point_type") or fallback_point_type
                raw_text = format_value(entry["value"], pt, data_format)
                decimals = self._column_decimals(c)
                values.append(self._apply_decimals(raw_text, decimals))

        self.tree.insert("", tk.END, values=(self.serial_no, date_str, time_str, *values))
        children = self.tree.get_children()
        if children:
            self.tree.see(children[-1])

        if len(children) >= self.max_rows:
            self.status_var.set(f"Reached {self.max_rows} rows - logging stopped automatically.")

    def _stop(self, reason=None):
        self.running = False
        if self.timer_job is not None:
            try:
                self.after_cancel(self.timer_job)
            except Exception:
                pass
            self.timer_job = None
        if self.countdown_job is not None:
            try:
                self.after_cancel(self.countdown_job)
            except Exception:
                pass
            self.countdown_job = None
        self.next_log_ts = None
        self.countdown_var.set("Next log in: -")
        self.start_btn.configure(state=tk.NORMAL)
        self.stop_btn.configure(state=tk.DISABLED)
        self.max_rows_combo.configure(state=tk.NORMAL)
        for entry in self.address_entries:
            entry.configure(state=tk.NORMAL)
        for entry in self.decimal_entries:
            entry.configure(state=tk.NORMAL)
        self.status_var.set(reason if reason else "Stopped.")

    def _clear_table(self):
        self.tree.delete(*self.tree.get_children())
        self.serial_no = 0
        if not self.running:
            self.status_var.set("Stopped.")

    # ---------------------------------------------------------------
    def _save_pdf(self):
        """Exports everything currently shown in this window - connection
        details, Device Id, Modbus Type, Conversion, the interval, every
        configured address/label, and every logged row - to a PDF file."""
        try:
            from reportlab.lib import colors as rl_colors
            from reportlab.lib.pagesizes import A4, landscape
            from reportlab.lib.styles import getSampleStyleSheet
            from reportlab.lib.units import mm
            from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer
            from xml.sax.saxutils import escape as xml_escape
        except ImportError:
            messagebox.showerror(
                "reportlab not installed",
                'Saving as PDF needs the "reportlab" package.\n\nRun:\n    pip install reportlab\n\n'
                'then try Save as PDF again.', parent=self)
            return

        rows = self.tree.get_children()
        if not rows:
            messagebox.showinfo("Nothing to save", "The table is empty - there is no data logged yet.",
                                 parent=self)
            return

        default_name = f"modscan_data_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pdf"
        path = filedialog.asksaveasfilename(
            title="Save Data Report As", defaultextension=".pdf",
            filetypes=[("PDF file", "*.pdf"), ("All files", "*.*")],
            initialfile=default_name, parent=self)
        if not path:
            return

        params = self.app.conn_params
        summary = self.app._connection_summary_text(params) if params else "-"
        device_id = self.app.device_id_var.get().strip() or "?"
        modbus_type = self.app.point_type_var.get()
        conversion = self.app.conversion_var.get()
        conn_state = "CONNECTED" if self.app.connected else "DISCONNECTED"
        interval_text = f"{self.interval_var.get().strip() or '-'} {self.interval_unit_var.get()}"

        styles = getSampleStyleSheet()
        story = [
            Paragraph("MODSCAN DATA REPORT", styles["Title"]),
            Spacer(1, 6 * mm),
            Paragraph(
                f"<b>Connection:</b> {summary} &nbsp;&nbsp; [{conn_state}] &nbsp;&nbsp; "
                f"<b>Device Id (Slave Id):</b> {device_id}", styles["Normal"]),
            Paragraph(
                f"<b>Modbus Type:</b> {modbus_type} &nbsp;&nbsp; <b>Conversion:</b> {conversion} "
                f"&nbsp;&nbsp; <b>Time Interval:</b> {interval_text}", styles["Normal"]),
            Paragraph(
                f"<b>Report generated:</b> {timestamp()} &nbsp;&nbsp; "
                f"<b>Rows logged:</b> {len(rows)} of max {self.max_rows}", styles["Normal"]),
            Spacer(1, 6 * mm),
        ]

        notes = self.notes_text.get("1.0", tk.END).strip()
        if notes:
            notes_html = xml_escape(notes).replace("\n", "<br/>")
            story.append(Paragraph("<b>Report Notes / Format:</b>", styles["Heading3"]))
            story.append(Paragraph(notes_html, styles["Normal"]))
            story.append(Spacer(1, 6 * mm))

        # Address / label / decimal-places configuration summary
        addr_lines = ", ".join(
            f"{self.label_vars[c].get().strip() or f'Addr {c+1}'} = "
            f"{self.address_vars[c].get().strip() or '-'} "
            f"(decimals: {self._column_decimals(c)})"
            for c in range(REPORT_ADDR_COLS)
        )
        story.append(Paragraph(f"<b>Configured addresses:</b> {addr_lines}", styles["Normal"]))
        story.append(Spacer(1, 6 * mm))

        header = ["Sl No.", "Date", "Time"] + [
            f"{self.label_vars[c].get().strip() or f'Addr {c+1}'}\n({self.address_vars[c].get().strip() or '-'})"
            for c in range(REPORT_ADDR_COLS)
        ]
        table_data = [header]
        for iid in rows:
            table_data.append([str(v) for v in self.tree.item(iid, "values")])

        col_widths = [16 * mm, 20 * mm, 16 * mm] + [(230 * mm) / REPORT_ADDR_COLS] * REPORT_ADDR_COLS
        table = Table(table_data, colWidths=col_widths, repeatRows=1)
        table.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, 0), rl_colors.HexColor("#212b38")),
            ("TEXTCOLOR", (0, 0), (-1, 0), rl_colors.white),
            ("FONTSIZE", (0, 0), (-1, -1), 7),
            ("ALIGN", (0, 0), (-1, -1), "CENTER"),
            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ("GRID", (0, 0), (-1, -1), 0.4, rl_colors.grey),
            ("ROWBACKGROUNDS", (0, 1), (-1, -1), [rl_colors.white, rl_colors.HexColor("#f0f2f5")]),
        ]))
        story.append(table)

        try:
            doc = SimpleDocTemplate(path, pagesize=landscape(A4),
                                     leftMargin=14 * mm, rightMargin=14 * mm,
                                     topMargin=12 * mm, bottomMargin=12 * mm)
            doc.build(story)
        except Exception as exc:
            messagebox.showerror("Save failed", f"Could not save PDF:\n{exc}", parent=self)
            return

        self.status_var.set(f"Saved PDF report to: {path}")
        messagebox.showinfo("Saved", f"Data report saved to:\n{path}", parent=self)

    def _on_close(self):
        self._stop()
        if self.info_job is not None:
            try:
                self.after_cancel(self.info_job)
            except Exception:
                pass
        self.app.report_window = None
        self.app.report_current = {}
        self.destroy()


# --------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------
class ModbusMasterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Modbus TCP Alarm Monitor")
        self.root.configure(bg=COLORS["bg"])
        self.root.minsize(1280, 760)
        self._launch_full_size()

        self.poller: PollerThread | None = None
        self.connected = False
        self.result_queue: "queue.Queue[dict]" = queue.Queue()
        self.rows = []
        self.alarm_state = {}
        self.alarm_counts = {}
        self.blink_on = False
        self.log_lines = []
        self.export_log_path = None
        self.num_polls = 0
        self.valid_responses = 0
        self.alarm_num_polls = 0
        self.alarm_valid_responses = 0

        self.report_window = None   # the Data Report Toplevel, when open
        self.report_current = {}    # col_idx -> latest {"value", "ok", "point_type", "address"}

        self.conn_params = {"mode": "tcp", "mode_label": "TCP/IP", "host": "127.0.0.1", "port": 502}
        self.timeout_ms = 1000
        self.delay_ms = 1000
        self.transmission_mode = "RTU"

        self.data_format_var = tk.StringVar(value="Decimal")              # Alarm Monitor ONLY
        self.conversion_var = tk.StringVar(value="Decimal")               # ModScan panel ONLY
        self.point_type_var = tk.StringVar(value="01: COIL")              # ModScan panel scan
        self.alarm_point_type_var = tk.StringVar(value="03: HOLDING REG") # Alarm Monitor scan
        self.alarm_point_type_prev = "03: HOLDING REG"

        self._setup_style()
        self._build_menu_bar()
        self._build_header()
        self._build_body()

        self._poll_gui_queue()
        self._blink_tick()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------
    def _launch_full_size(self):
        """Open maximized so the whole layout benefits from the extra
        space on large monitors, instead of a small fixed-size window
        that then looks cramped/blurry when stretched by hand."""
        try:
            self.root.state("zoomed")  # Windows / most Linux window managers
            return
        except tk.TclError:
            pass
        try:
            self.root.attributes("-zoomed", True)  # some Linux WMs
            return
        except tk.TclError:
            pass
        try:
            sw = self.root.winfo_screenwidth()
            sh = self.root.winfo_screenheight()
            self.root.geometry(f"{sw}x{sh}+0+0")
        except tk.TclError:
            self.root.geometry("1400x830")

    # ---------------------------------------------------------------
    def _setup_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=COLORS["panel"])
        style.configure("TLabel", background=COLORS["panel"], foreground=COLORS["text"], font=FONT_UI)
        style.configure("Hint.TLabel", background=COLORS["panel"], foreground=COLORS["text_dim"],
                         font=("Segoe UI", 11, "italic"))
        style.configure("Col.TLabel", background=COLORS["panel_raised"], foreground=COLORS["accent2"],
                         font=FONT_UI_BOLD)
        # Buttons: a clearly brighter background on hover ("active") so it's
        # obvious the cursor is over something clickable, plus a "pressed"
        # shade so a click still reads distinctly from a hover.
        style.configure("TButton", font=FONT_UI_BOLD, padding=(14, 8), background=COLORS["btn_bg"],
                         foreground=COLORS["text"], borderwidth=0, relief="flat")
        style.map("TButton",
                  background=[("disabled", COLORS["border"]),
                              ("pressed", COLORS["accent_dim"]),
                              ("active", COLORS["btn_hover"])],
                  foreground=[("active", COLORS["menu_hover_fg"])])
        style.configure("Accent.TButton", background=COLORS["accent2"], foreground=COLORS["accent2_fg"],
                         font=FONT_UI_BOLD, padding=(14, 8), borderwidth=0, relief="flat")
        style.map("Accent.TButton",
                  background=[("disabled", COLORS["border"]),
                              ("pressed", COLORS["gold"]),
                              ("active", COLORS["btn_accent_hover"])])
        style.configure("TEntry", fieldbackground=COLORS["panel_alt"], foreground=COLORS["text"],
                         insertcolor=COLORS["text"], bordercolor=COLORS["border"], borderwidth=1,
                         font=FONT_UI)
        style.configure("TCombobox", fieldbackground=COLORS["panel_alt"], background=COLORS["panel_alt"],
                         foreground=COLORS["text"], bordercolor=COLORS["border"], arrowcolor=COLORS["text"],
                         font=FONT_UI)
        style.map("TCombobox", fieldbackground=[("readonly", COLORS["panel_alt"])],
                  foreground=[("readonly", COLORS["text"])])
        # Bigger row height + explicit font (was left at the tiny Tk default
        # before) is most of what makes tables read clearly on a big screen.
        style.configure("Treeview", background=COLORS["table_bg"], fieldbackground=COLORS["table_bg"],
                         foreground=COLORS["text"], rowheight=28, borderwidth=0, font=FONT_UI)
        style.map("Treeview", background=[("selected", COLORS["accent_soft"])])
        style.configure("Treeview.Heading", background=COLORS["panel_raised"], foreground=COLORS["accent2"],
                         font=FONT_UI_BOLD, relief="flat", padding=(4, 6))
        style.map("Treeview.Heading", background=[("active", COLORS["btn_hover"])])

        style.configure("Vertical.TScrollbar", background=COLORS["panel_alt"], troughcolor=COLORS["panel"],
                         bordercolor=COLORS["panel"], arrowcolor=COLORS["text_dim"], relief="flat")
        style.map("Vertical.TScrollbar", background=[("active", COLORS["accent"])])

    # ---------------------------------------------------------------
    # Menu bar
    # ---------------------------------------------------------------
    def _build_menu_bar(self):
        menu_kw = dict(bg=COLORS["panel_raised"], fg=COLORS["text"],
                        activebackground=COLORS["accent"], activeforeground=COLORS["menu_hover_fg"])
        menubar = tk.Menu(self.root, font=MENU_FONT, tearoff=0, **menu_kw)
        self.root.config(menu=menubar)
        self.menubar = menubar

        connection_menu = tk.Menu(menubar, tearoff=0, font=MENU_FONT, **menu_kw)
        connection_menu.add_command(label="Connect...", command=self._open_connect_dialog)
        connection_menu.add_command(label="Disconnect", command=self.disconnect, state=tk.DISABLED)
        menubar.add_cascade(label="Connection", menu=connection_menu)
        self.connection_menu = connection_menu

        settings_menu = tk.Menu(menubar, tearoff=0, font=MENU_FONT, **menu_kw)
        settings_menu.add_command(label="Protocol Selection...", command=self._open_protocol_dialog)
        menubar.add_cascade(label="Settings", menu=settings_menu)

        data_format_menu = tk.Menu(menubar, tearoff=0, font=MENU_FONT, **menu_kw)
        for fmt in DATA_FORMATS:
            data_format_menu.add_radiobutton(label=fmt, value=fmt, variable=self.data_format_var,
                                              command=self._on_data_format_change)
        menubar.add_cascade(label="Data Format: Decimal (Alarm Monitor)", menu=data_format_menu)
        # Robust index lookup (fixes the earlier bug where a hard-coded
        # index accidentally overwrote the "Settings" label instead).
        self.data_format_menu_index = menubar.index(tk.END)

        menubar.add_command(label="Data Report", command=self._open_data_report_window)

        help_menu = tk.Menu(menubar, tearoff=0, font=MENU_FONT, **menu_kw)
        help_menu.add_command(label="About", command=self._show_about)
        menubar.add_cascade(label="Help", menu=help_menu)

    def _on_data_format_change(self):
        fmt = self.data_format_var.get()
        self.menubar.entryconfig(self.data_format_menu_index, label=f"Data Format: {fmt} (Alarm Monitor)")

    def _show_about(self):
        saving_path = self.export_log_path if self.export_log_path else "Not set (use 'Save to TXT' in the Event Log)"
        messagebox.showinfo(
            "About",
            "Modbus TCP Alarm Monitor\n"
            "An open-source ModScan-style tool built with Python, "
            "Tkinter and pymodbus.\n\n"
            f"Saving Path: {saving_path}",
        )

    # ---------------------------------------------------------------
    # Header: title (highlights light green when connected) + blinking LED
    # ---------------------------------------------------------------
    def _build_header(self):
        outer = tk.Frame(self.root, bg=COLORS["bg"])
        outer.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(10, 4))

        self.title_label = tk.Label(outer, text="\u26A1 MODBUS TCP ALARM MONITOR", bg=COLORS["bg"],
                                     fg=COLORS["accent"], font=FONT_HEADER, padx=8, pady=3)
        self.title_label.pack(side=tk.LEFT)

        # Two-tone divider (teal -> amber) instead of a flat line, for a
        # bit of visual flair under the title bar.
        divider = tk.Frame(self.root, bg=COLORS["bg"], height=3)
        divider.pack(side=tk.TOP, fill=tk.X, padx=14)
        tk.Frame(divider, bg=COLORS["accent"], height=3).pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Frame(divider, bg=COLORS["accent2"], height=3).pack(side=tk.LEFT, fill=tk.X, expand=True)

        led_frame = tk.Frame(outer, bg=COLORS["bg"])
        led_frame.pack(side=tk.RIGHT)
        self.led_canvas = tk.Canvas(led_frame, width=22, height=22, bg=COLORS["bg"], highlightthickness=0)
        self.led_canvas.pack(side=tk.LEFT, padx=(0, 8))
        self.led_ring = self.led_canvas.create_oval(2, 2, 20, 20, outline=COLORS["red"], width=2, fill="")
        self.led_dot = self.led_canvas.create_oval(7, 7, 15, 15, fill=COLORS["red"], outline="")
        self.conn_status_var = tk.StringVar(value="Disconnected")
        self.conn_status_label = tk.Label(led_frame, textvariable=self.conn_status_var, bg=COLORS["bg"],
                                           fg=COLORS["red"], font=FONT_UI_BOLD, padx=4)
        self.conn_status_label.pack(side=tk.LEFT)

        self.conn_summary_var = tk.StringVar(value="")
        tk.Label(outer, textvariable=self.conn_summary_var, bg=COLORS["bg"], fg=COLORS["text_dim"],
                 font=("Segoe UI", 11)).pack(side=tk.RIGHT, padx=(0, 16))

        # ---- Global poll-statistics box (moved here from the ModScan /
        # Alarm Monitor panels per request - one place, top of window) ----
        stats_box = tk.Frame(outer, bg=COLORS["panel"], highlightbackground=COLORS["border"],
                              highlightthickness=1)
        stats_box.pack(side=tk.RIGHT, padx=(0, 16))

        scan_stats = tk.Frame(stats_box, bg=COLORS["panel"])
        scan_stats.pack(side=tk.LEFT, padx=(10, 14), pady=4)
        tk.Label(scan_stats, text="ModScan:", bg=COLORS["panel"], fg=COLORS["text_dim"],
                 font=("Segoe UI", 10)).pack(anchor="w")
        self.stat_num_polls_var = tk.StringVar(value="No of Polls: 0")
        self.stat_valid_resp_var = tk.StringVar(value="Valid Slave Response: 0")
        tk.Label(scan_stats, textvariable=self.stat_num_polls_var, bg=COLORS["panel"], fg=COLORS["accent"],
                 font=("Segoe UI", 11, "bold")).pack(anchor="w")
        tk.Label(scan_stats, textvariable=self.stat_valid_resp_var, bg=COLORS["panel"], fg=COLORS["accent"],
                 font=("Segoe UI", 11, "bold")).pack(anchor="w")

        alarm_stats = tk.Frame(stats_box, bg=COLORS["panel"])
        alarm_stats.pack(side=tk.LEFT, padx=(0, 10), pady=4)
        tk.Label(alarm_stats, text="Alarm Monitor:", bg=COLORS["panel"], fg=COLORS["text_dim"],
                 font=("Segoe UI", 10)).pack(anchor="w")
        self.alarm_num_polls_var = tk.StringVar(value="No of Polls: 0")
        self.alarm_valid_resp_var = tk.StringVar(value="Valid Slave Response: 0")
        tk.Label(alarm_stats, textvariable=self.alarm_num_polls_var, bg=COLORS["panel"], fg=COLORS["accent"],
                 font=("Segoe UI", 11, "bold")).pack(anchor="w")
        tk.Label(alarm_stats, textvariable=self.alarm_valid_resp_var, bg=COLORS["panel"], fg=COLORS["accent"],
                 font=("Segoe UI", 11, "bold")).pack(anchor="w")

        ttk.Button(stats_box, text="Reset\nCounters", command=self._reset_ctrs).pack(
            side=tk.LEFT, padx=(0, 8), pady=4)

    def _set_led(self, connected):
        self.connected = connected
        if connected:
            self.title_label.configure(bg=COLORS["conn_green_bg"], fg=COLORS["conn_green_fg"])
            self.conn_status_label.configure(fg=COLORS["accent2"])
            self.led_canvas.itemconfig(self.led_ring, outline=COLORS["accent2"])
        else:
            self.title_label.configure(bg=COLORS["bg"], fg=COLORS["accent"])
            self.led_canvas.itemconfig(self.led_dot, fill=COLORS["red"])
            self.led_canvas.itemconfig(self.led_ring, outline=COLORS["red"])
            self.conn_status_label.configure(fg=COLORS["red"])
        self.conn_status_var.set("Connected" if connected else "DISCONNECTED")

    def _connection_summary_text(self, params):
        if params["mode"] == "tcp":
            return f"{params['host']}:{params['port']}"
        return (f"{params['port']}  {params['baudrate']} baud, "
                f"{params['bytesize']}{params['parity']}{params['stopbits']}")

    def _play_alert_sound(self, kind):
        """Short audible cue on Connect / Disconnect, ModScan-style.
        Runs on a background thread since winsound.Beep() blocks for its
        whole duration and we never want that to stall the GUI thread."""
        def _do():
            try:
                if HAVE_WINSOUND:
                    if kind == "connect":
                        winsound.Beep(1400, 150)
                        winsound.Beep(1800, 150)
                    else:
                        winsound.Beep(700, 200)
                        winsound.Beep(400, 300)
                else:
                    self.root.bell()
            except Exception:
                try:
                    self.root.bell()
                except Exception:
                    pass
        threading.Thread(target=_do, daemon=True).start()

    def _open_connect_dialog(self):
        if self.poller is not None:
            return
        ConnectionDialog(self.root, self.conn_params, self._do_connect)

    def _open_protocol_dialog(self):
        ProtocolSelectionDialog(self.root, self.timeout_ms, self.delay_ms, self.transmission_mode,
                                 self._apply_protocol)

    def _apply_protocol(self, timeout_ms, delay_ms, transmission_mode):
        self.timeout_ms = timeout_ms
        self.delay_ms = delay_ms
        self.transmission_mode = transmission_mode
        self._log(f"Protocol Selection updated: Timeout={timeout_ms}ms, Delay={delay_ms}ms, "
                   f"Transmission Mode={transmission_mode}", "info")

    # ---------------------------------------------------------------
    # Body
    # ---------------------------------------------------------------
    def _build_body(self):
        body = tk.Frame(self.root, bg=COLORS["bg"])
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=14, pady=(6, 14))

        self._build_left_panel(body)
        self._build_alarm_panel(body)
        self._build_log_panel(body)

    # ---------------- LEFT: ModScan panel + live data table ----------------
    def _build_left_panel(self, parent):
        left = tk.Frame(parent, bg=COLORS["panel"], highlightbackground=COLORS["border"],
                         highlightthickness=1, width=430)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left.pack_propagate(False)

        title_bar = tk.Frame(left, bg=COLORS["panel_raised"], height=30)
        title_bar.pack(side=tk.TOP, fill=tk.X)
        tk.Frame(title_bar, bg=COLORS["accent2"], width=4).pack(side=tk.LEFT, fill=tk.Y)
        tk.Label(title_bar, text="\U0001F4E1 MODSCAN", bg=COLORS["panel_raised"], fg=COLORS["accent2"],
                 font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=10, pady=5)
        self.left_conn_summary_var = tk.StringVar(value="")
        tk.Label(title_bar, textvariable=self.left_conn_summary_var, bg=COLORS["panel_raised"],
                 fg=COLORS["text_dim"], font=("Segoe UI", 10)).pack(side=tk.RIGHT, padx=8)
        tk.Frame(left, bg=COLORS["border"], height=1).pack(side=tk.TOP, fill=tk.X)

        panel = tk.Frame(left, bg=COLORS["panel"])
        panel.pack(side=tk.TOP, fill=tk.X, padx=12, pady=12)

        LBL = {"bg": COLORS["panel"], "fg": COLORS["text_dim"], "font": ("Segoe UI", 11)}

        self.address_var = tk.StringVar(value="1")
        self.length_var = tk.StringVar(value="100")
        self.device_id_var = tk.StringVar(value="1")

        def dark_entry(var, width=8):
            return tk.Entry(panel, textvariable=var, width=width, relief="flat",
                             bg=COLORS["panel_alt"], fg=COLORS["text"], insertbackground=COLORS["text"],
                             highlightthickness=1, highlightbackground=COLORS["border"],
                             highlightcolor=COLORS["accent"])

        # column 0-1: Start Address / Length (narrower, per request)
        tk.Label(panel, text="Start Address:", **LBL).grid(row=0, column=0, sticky="w", pady=5)
        dark_entry(self.address_var, width=8).grid(row=0, column=1, sticky="w", padx=(4, 20), pady=5)
        tk.Label(panel, text="Length:", **LBL).grid(row=1, column=0, sticky="w", pady=5)
        dark_entry(self.length_var, width=8).grid(row=1, column=1, sticky="w", padx=(4, 20), pady=5)

        # column 2-3: Device Id / Modbus Type / Conversion
        tk.Label(panel, text="Device Id:", **LBL).grid(row=0, column=2, sticky="w", pady=5)
        dark_entry(self.device_id_var, width=6).grid(row=0, column=3, sticky="w", pady=5)
        tk.Label(panel, text="Modbus Type:", **LBL).grid(row=1, column=2, sticky="w", pady=5)
        self.point_type_combo = ttk.Combobox(panel, textvariable=self.point_type_var,
                                              values=list(POINT_TYPES.keys()), width=15, state="readonly")
        self.point_type_combo.grid(row=1, column=3, sticky="w", pady=5)
        self.point_type_combo.bind("<<ComboboxSelected>>", self._on_point_type_change)

        tk.Label(panel, text="Conversion:", **LBL).grid(row=2, column=2, sticky="w", pady=5)
        ttk.Combobox(panel, textvariable=self.conversion_var, values=DATA_FORMATS,
                     width=15, state="readonly").grid(row=2, column=3, sticky="w", pady=5)

        self.data_status_var = tk.StringVar(value="** Data Uninitialized **")
        self.data_status_label = tk.Label(left, textvariable=self.data_status_var, bg=COLORS["panel"],
                                           fg=COLORS["warn_red"], font=("Consolas", 9, "bold"))
        self.data_status_label.pack(side=tk.TOP, anchor="w", padx=12, pady=(4, 6))

        table_frame = tk.Frame(left, bg=COLORS["panel"])
        table_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        self.data_tree = ttk.Treeview(table_frame, columns=("address", "value"), show="headings")
        self.data_tree.heading("address", text="Address")
        self.data_tree.heading("value", text="Value")
        self.data_tree.column("address", width=110, anchor=tk.CENTER)
        self.data_tree.column("value", width=150, anchor=tk.CENTER)
        self.data_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        enable_row_hover(self.data_tree)

        vscroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.data_tree.yview)
        vscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.data_tree.configure(yscrollcommand=vscroll.set)

    def _on_point_type_change(self, event=None):
        """Reset Start Address to the new type's Modicon base (e.g. 40001
        for Holding Register), matching how ModScan itself behaves."""
        point_type = POINT_TYPES[self.point_type_var.get()]
        self.address_var.set(str(MODICON_BASE[point_type]))

    def _reset_ctrs(self):
        self.num_polls = 0
        self.valid_responses = 0
        self.alarm_num_polls = 0
        self.alarm_valid_responses = 0
        self.stat_num_polls_var.set("No of Polls: 0")
        self.stat_valid_resp_var.set("Valid Slave Response: 0")
        self.alarm_num_polls_var.set("No of Polls: 0")
        self.alarm_valid_resp_var.set("Valid Slave Response: 0")

    # ---------------- MIDDLE: Alarm Monitor ----------------
    def _build_alarm_panel(self, parent):
        mid = tk.Frame(parent, bg=COLORS["panel"], highlightbackground=COLORS["border"], highlightthickness=1)
        mid.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        header = tk.Frame(mid, bg=COLORS["panel_raised"])
        header.pack(side=tk.TOP, fill=tk.X)
        tk.Frame(header, bg=COLORS["accent2"], width=4).pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(header, text="\U0001F514 ALARM MONITOR", style="Col.TLabel", padding=6).pack(side=tk.LEFT)
        tk.Frame(mid, bg=COLORS["border"], height=1).pack(side=tk.TOP, fill=tk.X)

        ptype_frame = tk.Frame(mid, bg=COLORS["panel"])
        ptype_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(6, 4))
        tk.Label(ptype_frame, text="Point Type:", bg=COLORS["panel"], fg=COLORS["text_dim"],
                 font=("Segoe UI", 11)).pack(side=tk.LEFT)
        self.alarm_point_type_combo = ttk.Combobox(ptype_frame, textvariable=self.alarm_point_type_var,
                                                    values=list(POINT_TYPES.keys()), width=15, state="readonly")
        self.alarm_point_type_combo.pack(side=tk.LEFT, padx=(4, 10))
        self.alarm_point_type_combo.bind("<<ComboboxSelected>>", self._on_alarm_point_type_change)
        ttk.Button(ptype_frame, text="Apply", command=self.apply_addresses).pack(side=tk.RIGHT)

        # The 20-row grid can be taller than the available window height
        # (especially now that fonts/rows are larger for readability), which
        # was clipping off the bottom rows (e.g. address 40020) with no way
        # to reach them. Wrapping it in a Canvas + Scrollbar - plus mouse
        # wheel support - guarantees every row stays reachable no matter the
        # screen size.
        grid_container = tk.Frame(mid, bg=COLORS["panel"])
        grid_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        grid_canvas = tk.Canvas(grid_container, bg=COLORS["panel"], highlightthickness=0)
        grid_scroll = ttk.Scrollbar(grid_container, orient="vertical", command=grid_canvas.yview)
        grid_canvas.configure(yscrollcommand=grid_scroll.set)
        grid_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        grid_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        grid_frame = tk.Frame(grid_canvas, bg=COLORS["panel"])
        grid_frame_id = grid_canvas.create_window((0, 0), window=grid_frame, anchor="nw")

        def _sync_scrollregion(_event=None):
            grid_canvas.configure(scrollregion=grid_canvas.bbox("all"))
        grid_frame.bind("<Configure>", _sync_scrollregion)

        def _sync_inner_width(event):
            grid_canvas.itemconfig(grid_frame_id, width=event.width)
        grid_canvas.bind("<Configure>", _sync_inner_width)

        def _on_mousewheel(event):
            if event.num == 4:          # Linux scroll up
                grid_canvas.yview_scroll(-1, "units")
            elif event.num == 5:        # Linux scroll down
                grid_canvas.yview_scroll(1, "units")
            else:                       # Windows / macOS
                grid_canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

        def _bind_wheel(_event=None):
            grid_canvas.bind_all("<MouseWheel>", _on_mousewheel)
            grid_canvas.bind_all("<Button-4>", _on_mousewheel)
            grid_canvas.bind_all("<Button-5>", _on_mousewheel)

        def _unbind_wheel(_event=None):
            grid_canvas.unbind_all("<MouseWheel>")
            grid_canvas.unbind_all("<Button-4>")
            grid_canvas.unbind_all("<Button-5>")

        grid_canvas.bind("<Enter>", _bind_wheel)
        grid_canvas.bind("<Leave>", _unbind_wheel)

        col_w = 8
        headers = ["Address", "Value", "Low SP", "High SP", "Alarm\nCount"]
        for c, text in enumerate(headers):
            tk.Label(grid_frame, text=text, bg=COLORS["panel_raised"], fg=COLORS["accent"],
                     font=("Segoe UI", 11, "bold"), width=col_w, pady=4, justify="center").grid(
                row=0, column=c, padx=1, pady=1, sticky="nsew")

        hr_base = MODICON_BASE["hr"]
        for r in range(NUM_ROWS):
            row_bg = COLORS["panel_alt"] if r % 2 == 0 else COLORS["panel_alt2"]
            addr_var = tk.StringVar(value=str(hr_base + r))  # Modicon-style default (Holding Register)
            low_var = tk.StringVar(value="")
            high_var = tk.StringVar(value="")
            count_var = tk.StringVar(value="0")

            addr_entry = tk.Entry(grid_frame, textvariable=addr_var, width=col_w, justify="center",
                                   bg=row_bg, fg=COLORS["text"],
                                   insertbackground=COLORS["text"], relief="flat")
            addr_entry.grid(row=r + 1, column=0, padx=1, pady=1)
            enable_widget_hover(addr_entry, row_bg, COLORS["row_hover"])

            value_var = tk.StringVar(value="-")
            value_entry = tk.Entry(grid_frame, textvariable=value_var, width=col_w, justify="center",
                                    bg=row_bg, fg=COLORS["text"], relief="flat",
                                    state="readonly", readonlybackground=row_bg)
            value_entry.grid(row=r + 1, column=1, padx=1, pady=1)
            value_entry.bind("<Enter>", lambda e, w=value_entry: w.configure(readonlybackground=COLORS["row_hover"]))
            value_entry.bind("<Leave>", lambda e, w=value_entry, b=row_bg: w.configure(readonlybackground=b))

            low_entry = tk.Entry(grid_frame, textvariable=low_var, width=col_w, justify="center",
                                  bg=COLORS["panel_alt"], fg=COLORS["text"],
                                  insertbackground=COLORS["text"], relief="flat")
            low_entry.grid(row=r + 1, column=2, padx=1, pady=1)

            high_entry = tk.Entry(grid_frame, textvariable=high_var, width=col_w, justify="center",
                                   bg=COLORS["panel_alt"], fg=COLORS["text"],
                                   insertbackground=COLORS["text"], relief="flat")
            high_entry.grid(row=r + 1, column=3, padx=1, pady=1)

            count_entry = tk.Entry(grid_frame, textvariable=count_var, width=col_w, justify="center",
                                    bg=row_bg, fg=COLORS["gold"], relief="flat",
                                    state="readonly", readonlybackground=row_bg)
            count_entry.grid(row=r + 1, column=4, padx=1, pady=1)

            self.rows.append({
                "addr_var": addr_var, "addr_entry": addr_entry,
                "value_var": value_var, "value_entry": value_entry,
                "low_var": low_var, "low_entry": low_entry,
                "high_var": high_var, "high_entry": high_entry,
                "count_var": count_var,
                "raw_value": None,
            })
            self.alarm_state[r] = "normal"
            self.alarm_counts[r] = 0

        ttk.Label(mid, text=("Addresses are Modicon-style (e.g. 40001=Holding Reg #1, 00001=Coil #1) - "
                              "make sure the address matches the Point Type selected above."),
                  style="Hint.TLabel", justify="left", padding=6, wraplength=420).pack(side=tk.TOP, fill=tk.X)

    # ---------------- RIGHT: Event Log ----------------
    def _build_log_panel(self, parent):
        right = tk.Frame(parent, bg=COLORS["panel"], highlightbackground=COLORS["border"], highlightthickness=1)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        header = tk.Frame(right, bg=COLORS["panel_raised"])
        header.pack(side=tk.TOP, fill=tk.X)
        tk.Frame(header, bg=COLORS["accent2"], width=4).pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(header, text="\U0001F4DD EVENT LOG", style="Col.TLabel", padding=6).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(header, text="\U0001F4C4 Save as PDF", style="Accent.TButton",
                   command=self._save_alarm_report_pdf).pack(side=tk.RIGHT, padx=(0, 6), pady=4)
        ttk.Button(header, text="Save to TXT", style="Accent.TButton",
                   command=self.choose_export_file).pack(side=tk.RIGHT, padx=6, pady=4)
        tk.Frame(right, bg=COLORS["border"], height=1).pack(side=tk.TOP, fill=tk.X)

        self.export_status_var = tk.StringVar(value="Not saving to file")
        tk.Label(right, textvariable=self.export_status_var, bg=COLORS["panel"], fg=COLORS["text_dim"],
                 font=FONT_SMALL, wraplength=300, justify="left").pack(
            side=tk.TOP, fill=tk.X, padx=8, pady=(4, 0), anchor="w")

        text_frame = tk.Frame(right, bg=COLORS["panel"])
        text_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(6, 10))

        self.log_text = tk.Text(text_frame, bg=COLORS["log_bg"], fg=COLORS["log_text"],
                                 insertbackground=COLORS["log_text"], font=FONT_MONO, wrap="word",
                                 state=tk.DISABLED, borderwidth=0)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll = ttk.Scrollbar(text_frame, command=self.log_text.yview)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=scroll.set)

        self.log_text.tag_configure("alarm_high", foreground=COLORS["log_alarm_high"])
        self.log_text.tag_configure("alarm_low", foreground=COLORS["log_alarm_low"])
        self.log_text.tag_configure("clear", foreground=COLORS["log_clear"])
        self.log_text.tag_configure("info", foreground=COLORS["log_info"])

    def choose_export_file(self):
        path = filedialog.asksaveasfilename(
            title="Save Event Log As",
            defaultextension=".txt",
            filetypes=[("Text file", "*.txt"), ("All files", "*.*")],
            initialfile=f"alarm_event_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(self.log_lines) + ("\n" if self.log_lines else ""))
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))
            return
        self.export_log_path = path
        self.export_status_var.set(f"Saving alarms to: {path}")
        self._log(f"Event log saved to: {path}. New events will keep being appended here.", "info")
        messagebox.showinfo("Saved", f"Log exported to:\n{path}\n\n"
                                      f"New events will continue to be appended to this file "
                                      f"automatically while the app runs.")

    # ---------------------------------------------------------------
    def _save_alarm_report_pdf(self):
        """Generates a professional, standalone alarm PDF report: connection
        summary, an Alarm Monitor table with each row's live value/setpoints
        and its alarm count, a total-alarm-count summary line, and the full
        Event Log underneath - everything needed to hand to someone who
        wasn't watching the screen live."""
        try:
            from reportlab.lib import colors as rl_colors
            from reportlab.lib.pagesizes import A4, landscape
            from reportlab.lib.styles import getSampleStyleSheet
            from reportlab.lib.units import mm
            from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer, PageBreak
            from xml.sax.saxutils import escape as xml_escape
        except ImportError:
            messagebox.showerror(
                "reportlab not installed",
                'Saving as PDF needs the "reportlab" package.\n\nRun:\n    pip install reportlab\n\n'
                'then try Save as PDF again.')
            return

        default_name = f"alarm_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pdf"
        path = filedialog.asksaveasfilename(
            title="Save Alarm Report As", defaultextension=".pdf",
            filetypes=[("PDF file", "*.pdf"), ("All files", "*.*")],
            initialfile=default_name)
        if not path:
            return

        summary = self._connection_summary_text(self.conn_params) if self.conn_params else "-"
        device_id = self.device_id_var.get().strip() or "?"
        conn_state = "CONNECTED" if self.connected else "DISCONNECTED"
        alarm_point_type = self.alarm_point_type_var.get()
        total_alarm_count = sum(self.alarm_counts.values())
        active_high = sum(1 for s in self.alarm_state.values() if s == "high")
        active_low = sum(1 for s in self.alarm_state.values() if s == "low")

        styles = getSampleStyleSheet()
        story = [
            Paragraph("ALARM EVENT REPORT", styles["Title"]),
            Spacer(1, 5 * mm),
            Paragraph(
                f"<b>Connection:</b> {summary} &nbsp;&nbsp; [{conn_state}] &nbsp;&nbsp; "
                f"<b>Device Id (Slave Id):</b> {device_id}", styles["Normal"]),
            Paragraph(
                f"<b>Alarm Monitor Point Type:</b> {alarm_point_type} &nbsp;&nbsp; "
                f"<b>Report generated:</b> {timestamp()}", styles["Normal"]),
            Spacer(1, 4 * mm),
        ]

        # Headline summary stats - the numbers a supervisor wants at a glance.
        stat_data = [
            ["Total Alarm Count", "Currently HIGH", "Currently LOW", "Event Log Entries"],
            [str(total_alarm_count), str(active_high), str(active_low), str(len(self.log_lines))],
        ]
        stat_table = Table(stat_data, colWidths=[57.5 * mm] * 4)
        stat_table.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, 0), rl_colors.HexColor("#212b38")),
            ("TEXTCOLOR", (0, 0), (-1, 0), rl_colors.white),
            ("BACKGROUND", (0, 1), (-1, 1), rl_colors.HexColor("#f0f2f5")),
            ("FONTSIZE", (0, 0), (-1, -1), 10),
            ("FONTNAME", (0, 1), (-1, 1), "Helvetica-Bold"),
            ("ALIGN", (0, 0), (-1, -1), "CENTER"),
            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ("GRID", (0, 0), (-1, -1), 0.4, rl_colors.grey),
            ("TOPPADDING", (0, 0), (-1, -1), 6),
            ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
        ]))
        story.append(stat_table)
        story.append(Spacer(1, 6 * mm))

        # Alarm Monitor table - every configured row with its live reading,
        # setpoints, and how many times it has alarmed.
        story.append(Paragraph("Alarm Monitor", styles["Heading2"]))
        am_header = ["Row", "Address", "Value", "Low SP", "High SP", "State", "Alarm Count"]
        am_data = [am_header]
        for idx, row in enumerate(self.rows):
            state = self.alarm_state.get(idx, "normal").upper()
            am_data.append([
                str(idx + 1),
                row["addr_var"].get().strip() or "-",
                row["value_var"].get(),
                row["low_var"].get().strip() or "-",
                row["high_var"].get().strip() or "-",
                state,
                str(self.alarm_counts.get(idx, 0)),
            ])
        am_table = Table(am_data, colWidths=[15 * mm, 28 * mm, 28 * mm, 28 * mm, 28 * mm, 25 * mm, 28 * mm],
                          repeatRows=1)
        row_styles = [
            ("BACKGROUND", (0, 0), (-1, 0), rl_colors.HexColor("#212b38")),
            ("TEXTCOLOR", (0, 0), (-1, 0), rl_colors.white),
            ("FONTSIZE", (0, 0), (-1, -1), 8),
            ("ALIGN", (0, 0), (-1, -1), "CENTER"),
            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ("GRID", (0, 0), (-1, -1), 0.4, rl_colors.grey),
            ("ROWBACKGROUNDS", (0, 1), (-1, -1), [rl_colors.white, rl_colors.HexColor("#f0f2f5")]),
        ]
        # Highlight any row that is currently alarming, or has alarmed at
        # least once, so it stands out from the sea of "normal" rows.
        for idx, row_data in enumerate(am_data[1:], start=1):
            state = row_data[5]
            if state == "HIGH":
                row_styles.append(("BACKGROUND", (0, idx), (-1, idx), rl_colors.HexColor("#f8c9ce")))
            elif state == "LOW":
                row_styles.append(("BACKGROUND", (0, idx), (-1, idx), rl_colors.HexColor("#fdeeb0")))
        am_table.setStyle(TableStyle(row_styles))
        story.append(am_table)
        story.append(Spacer(1, 6 * mm))

        # Full Event Log underneath, on its own page so the alarm summary
        # above always prints cleanly on page 1.
        story.append(PageBreak())
        story.append(Paragraph("Event Log", styles["Heading2"]))
        story.append(Paragraph(f"Total entries: {len(self.log_lines)}", styles["Normal"]))
        story.append(Spacer(1, 3 * mm))
        if self.log_lines:
            log_style = styles["Code"] if "Code" in styles else styles["Normal"]
            for line in self.log_lines:
                story.append(Paragraph(xml_escape(line), log_style))
        else:
            story.append(Paragraph("No events logged yet.", styles["Normal"]))

        try:
            doc = SimpleDocTemplate(path, pagesize=landscape(A4),
                                     leftMargin=14 * mm, rightMargin=14 * mm,
                                     topMargin=12 * mm, bottomMargin=12 * mm)
            doc.build(story)
        except Exception as exc:
            messagebox.showerror("Save failed", f"Could not save PDF:\n{exc}")
            return

        self._log(f"Alarm PDF report saved to: {path}", "info")
        messagebox.showinfo("Saved", f"Alarm report saved to:\n{path}")

    def _append_to_export_file(self, line):
        if not self.export_log_path:
            return
        try:
            with open(self.export_log_path, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except OSError:
            pass

    def _write_persistent_line(self, line):
        """Writes to the fixed background file AND the user-chosen export
        file (if any). Used for alarm onset/clear AND connect/disconnect
        events, all with full date/time."""
        try:
            with open(ALARM_LOG_FILE, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except OSError:
            pass
        self._append_to_export_file(line)

    # ---------------------------------------------------------------
    # Alarm Monitor address application (Modicon-style addressing)
    # ---------------------------------------------------------------
    def _current_addresses(self, point_type):
        """Returns {row_idx: (protocol_address, display_address)} using
        the given point_type's Modicon base for the conversion."""
        addrs = {}
        for idx, row in enumerate(self.rows):
            raw = row["addr_var"].get().strip()
            if raw == "":
                continue
            try:
                display_addr = int(raw)
            except ValueError:
                continue
            addrs[idx] = (to_protocol_address(display_addr, point_type), display_addr)
        return addrs

    def _get_alarm_params(self):
        point_type = POINT_TYPES[self.alarm_point_type_var.get()]
        return point_type, self._current_addresses(point_type)

    def apply_addresses(self):
        point_type = POINT_TYPES[self.alarm_point_type_var.get()]
        n = len(self._current_addresses(point_type))
        self._log(f"Alarm Monitor address list applied - {n} row(s) active.", "info")

    # ---------------------------------------------------------------
    # Data Report window
    # ---------------------------------------------------------------
    def _get_report_params(self):
        """Called by PollerThread (background thread) once per poll cycle.
        Mirrors _get_alarm_params, but the address list comes from the
        Data Report window's 6 editable address boxes (if that window is
        open) and always uses the ModScan panel's current Modbus Type -
        i.e. 'as per the ModScan screen selection', per spec."""
        if self.report_window is None or not self.report_window.winfo_exists():
            return None, {}
        point_type = POINT_TYPES[self.point_type_var.get()]
        addrs = {}
        for idx, addr_var in enumerate(self.report_window.address_vars):
            raw = addr_var.get().strip()
            if raw == "":
                continue
            try:
                display_addr = int(raw)
            except ValueError:
                continue
            addrs[idx] = (to_protocol_address(display_addr, point_type), display_addr)
        return point_type, addrs

    def _open_data_report_window(self):
        if self.report_window is not None and self.report_window.winfo_exists():
            self.report_window.lift()
            self.report_window.focus_force()
            return
        self.report_window = DataReportWindow(self.root, self)

    def _on_alarm_point_type_change(self, event=None):
        """When the Alarm Monitor's Point Type changes, the typed Modicon
        addresses (e.g. 40007) still refer to the OLD type's numbering.
        Re-express each row's address under the NEW type's Modicon base,
        keeping the same underlying protocol offset - e.g. 40007 (Holding
        Register, offset 6) becomes 30007 when switching to Input Register.
        This is what was making values look 'stuck' after a Point Type
        change: the old address numbers no longer matched the new type,
        so every read failed silently against the wrong register.

        Also immediately clears displayed values/formatting so nothing
        stale lingers on screen while the next poll (using the new type)
        comes in."""
        old_point_type = POINT_TYPES.get(self.alarm_point_type_prev, "hr")
        new_point_type = POINT_TYPES[self.alarm_point_type_var.get()]

        for row in self.rows:
            raw = row["addr_var"].get().strip()
            if raw != "":
                try:
                    old_display = int(raw)
                    protocol_offset = to_protocol_address(old_display, old_point_type)
                    new_base = MODICON_BASE[new_point_type]
                    row["addr_var"].set(str(new_base + protocol_offset))
                except ValueError:
                    pass
            row["raw_value"] = None
            row["value_var"].set("-")
            row["value_entry"].configure(fg=COLORS["text"])

        self.alarm_point_type_prev = self.alarm_point_type_var.get()
        n = len(self._current_addresses(new_point_type))
        self._log(f"Alarm Monitor Point Type changed to {self.alarm_point_type_var.get()} - "
                   f"addresses remapped, {n} row(s) active.", "info")

    # ---------------------------------------------------------------
    # Connect / disconnect
    # ---------------------------------------------------------------
    def _get_scan_params(self):
        point_type = POINT_TYPES[self.point_type_var.get()]
        try:
            display_address = int(self.address_var.get().strip())
            length = max(1, int(self.length_var.get().strip()))
            device_id = int(self.device_id_var.get().strip())
        except ValueError:
            display_address, length, device_id = MODICON_BASE[point_type], 20, 1
        protocol_address = to_protocol_address(display_address, point_type)
        return protocol_address, length, device_id, point_type, self.delay_ms, display_address

    def _do_connect(self, params):
        self.conn_params = params
        summary = self._connection_summary_text(params)
        self.conn_summary_var.set(summary)
        self.left_conn_summary_var.set(summary)

        self.connection_menu.entryconfig("Connect...", state=tk.DISABLED)
        self._log(f"Connecting via {params['mode_label']} ({summary})...", "info")

        self.num_polls = 0
        self.valid_responses = 0
        self.alarm_num_polls = 0
        self.alarm_valid_responses = 0
        self.stat_num_polls_var.set("No of Polls: 0")
        self.stat_valid_resp_var.set("Valid Slave Response: 0")
        self.alarm_num_polls_var.set("No of Polls: 0")
        self.alarm_valid_resp_var.set("Valid Slave Response: 0")
        self.data_status_var.set("** Data Uninitialized **")
        self.data_status_label.configure(fg=COLORS["warn_red"])
        self.data_tree.delete(*self.data_tree.get_children())
        for idx in self.alarm_counts:
            self.alarm_counts[idx] = 0
            self.rows[idx]["count_var"].set("0")
        self.report_current = {}

        self.poller = PollerThread(
            conn_params=params, timeout_ms=self.timeout_ms,
            get_scan_params=self._get_scan_params,
            get_alarm_params=self._get_alarm_params,
            result_queue=self.result_queue,
            transmission_mode=self.transmission_mode,
            get_report_params=self._get_report_params,
        )
        self.poller.start()

    def disconnect(self):
        if self.poller is None and not getattr(self, "connected", False):
            return
        if self.poller is not None:
            self.poller.stop()
            self.poller = None
        self._set_led(False)
        self.connection_menu.entryconfig("Connect...", state=tk.NORMAL)
        self.connection_menu.entryconfig("Disconnect", state=tk.DISABLED)
        slave_id = self.device_id_var.get().strip() or "?"
        self._log("Disconnected.", "info")
        self._write_persistent_line(f"{timestamp()} | DISCONNECTED (user-initiated) | Slave ID: {slave_id}")
        self._clear_live_data()
        self._play_alert_sound("disconnect")

    def _on_close(self):
        self.disconnect()
        self.root.destroy()

    # ---------------------------------------------------------------
    # Queue draining
    # ---------------------------------------------------------------
    def _poll_gui_queue(self):
        # IMPORTANT: this whole body is guarded so that no single bad
        # message can ever throw an exception that escapes and stops the
        # `after()` reschedule below - that used to be exactly what froze
        # the UI (values/last-update stuck) whenever the Treeview iids and
        # the freshly-typed address range fell out of sync.
        try:
            while True:
                try:
                    msg = self.result_queue.get_nowait()
                except queue.Empty:
                    break

                try:
                    self._handle_gui_message(msg)
                except Exception as exc:
                    self._log(f"Internal UI update error (ignored): {exc}", "alarm_high")
        finally:
            # Faster than the original 150ms so new values/alarms land on
            # screen sooner - still light enough not to burden the CPU.
            self.root.after(80, self._poll_gui_queue)

    def _handle_gui_message(self, msg):
        if msg["type"] == "connected":
            self._set_led(True)
            self.connection_menu.entryconfig("Disconnect", state=tk.NORMAL)
            self.connection_menu.entryconfig("Connect...", state=tk.DISABLED)
            self._log("Connected.", "info")
            summary = self._connection_summary_text(self.conn_params)
            slave_id = self.device_id_var.get().strip() or "?"
            self._write_persistent_line(
                f"{timestamp()} | CONNECTED | {self.conn_params['mode_label']} ({summary}) | Slave ID: {slave_id}")
            self._play_alert_sound("connect")

        elif msg["type"] == "disconnected":
            self._set_led(False)
            self.connection_menu.entryconfig("Connect...", state=tk.NORMAL)
            self.connection_menu.entryconfig("Disconnect", state=tk.DISABLED)
            error = msg.get('error', 'unknown error')
            slave_id = self.device_id_var.get().strip() or "?"
            self._log(f"Connection lost: {error}", "alarm_high")
            self._write_persistent_line(
                f"{timestamp()} | DISCONNECTED (connection lost) | {error} | Slave ID: {slave_id}")
            self.poller = None
            self._clear_live_data()
            self._play_alert_sound("disconnect")

        elif msg["type"] == "block":
            self.num_polls += 1
            self.stat_num_polls_var.set(f"No of Polls: {self.num_polls}")
            if msg["ok"]:
                self.valid_responses += 1
                self.stat_valid_resp_var.set(f"Valid Slave Response: {self.valid_responses}")
                self.data_status_var.set(f"Last update: {datetime.now().strftime('%H:%M:%S')}")
                self.data_status_label.configure(fg=COLORS["accent"])
                self._refresh_data_table(msg["display_start"], msg["values"], msg.get("point_type"))

        elif msg["type"] == "value":
            self.alarm_num_polls += 1
            self.alarm_num_polls_var.set(f"No of Polls: {self.alarm_num_polls}")
            if msg.get("ok"):
                self.alarm_valid_responses += 1
                self.alarm_valid_resp_var.set(f"Valid Slave Response: {self.alarm_valid_responses}")
                self._handle_row_value(msg["row"], msg["address"], msg["value"], msg.get("point_type"))
            else:
                self._handle_row_error(msg["row"])

        elif msg["type"] == "report_value":
            # Just cache the latest value per Data Report column; the report
            # window itself decides when to actually turn this into a row,
            # based on its own minute-interval timer.
            self.report_current[msg["col"]] = msg

    def _clear_live_data(self):
        """Called whenever we go to DISCONNECTED (manual or automatic).
        ModScan-style behavior: once the link is down, stale values are
        misleading, so the live tables are wiped and clearly marked -
        exactly like ModScan32 blanking its grid on a lost connection."""
        self.data_status_var.set("** DISCONNECTED - No live data **")
        self.data_status_label.configure(fg=COLORS["red"])
        self.data_tree.delete(*self.data_tree.get_children())
        self.report_current = {}

        for idx, row in enumerate(self.rows):
            row["raw_value"] = None
            row["value_var"].set("-")
            row["value_entry"].configure(fg=COLORS["text"])
            if self.alarm_state.get(idx, "normal") != "normal":
                self.alarm_state[idx] = "normal"
            row["low_entry"].configure(bg=COLORS["panel_alt"])
            row["high_entry"].configure(bg=COLORS["panel_alt"])

    def _refresh_data_table(self, display_start, values, point_type=None):
        if point_type is None:
            point_type = POINT_TYPES[self.point_type_var.get()]
        data_format = self.conversion_var.get()

        expected_iids = [str(display_start + i) for i in range(len(values))]
        children = self.data_tree.get_children()

        # Rebuild whenever the row COUNT or the actual ADDRESSES differ from
        # what's currently shown (e.g. user changed Start Address / Length /
        # Modbus Type while connected). Only patching values in place when
        # the iids truly match avoids a Tk exception from trying to update
        # a row id that no longer exists - that exception used to escape
        # the queue-draining loop and silently freeze the whole UI.
        if list(children) != expected_iids:
            self.data_tree.delete(*children)
            for addr, val in zip(expected_iids, values):
                self.data_tree.insert("", tk.END, iid=addr,
                                       values=(addr, format_value(val, point_type, data_format)))
        else:
            for addr, val in zip(expected_iids, values):
                self.data_tree.item(addr, values=(addr, format_value(val, point_type, data_format)))

    def _handle_row_value(self, row_idx, address, raw_value, point_type=None):
        row = self.rows[row_idx]
        if point_type is None:
            point_type = POINT_TYPES[self.alarm_point_type_var.get()]
        row["raw_value"] = raw_value
        row["value_var"].set(format_value(raw_value, point_type, self.data_format_var.get()))
        self._evaluate_alarm(row_idx, address, raw_value)

    def _handle_row_error(self, row_idx):
        """A single alarm-row poll failed (bad address, slave exception,
        etc.) while the link overall is still up. Show it plainly instead
        of silently leaving the previous value on screen forever - that
        silent staleness is what looked like 'not updating' when switching
        Point Type to an address that doesn't make sense for the new type."""
        row = self.rows[row_idx]
        row["raw_value"] = None
        row["value_var"].set("ERR")
        row["value_entry"].configure(fg=COLORS["text_dim"])

    # ---------------------------------------------------------------
    def _evaluate_alarm(self, row_idx, address, raw_value):
        row = self.rows[row_idx]
        low_raw = row["low_var"].get().strip()
        high_raw = row["high_var"].get().strip()
        numeric_value = float(raw_value)

        new_state = "normal"
        if low_raw != "":
            try:
                if numeric_value < float(low_raw):
                    new_state = "low"
            except ValueError:
                pass
        if high_raw != "":
            try:
                if numeric_value > float(high_raw):
                    new_state = "high"
            except ValueError:
                pass

        old_state = self.alarm_state.get(row_idx, "normal")
        if new_state != old_state:
            self.alarm_state[row_idx] = new_state
            if new_state == "low":
                self.alarm_counts[row_idx] += 1
                row["count_var"].set(str(self.alarm_counts[row_idx]))
                line = f"{timestamp()} | Address={address} | Value={raw_value} | LOW ALARM | Setpoint={low_raw}"
                self._log(line, "alarm_low")
                self._write_persistent_line(line)
            elif new_state == "high":
                self.alarm_counts[row_idx] += 1
                row["count_var"].set(str(self.alarm_counts[row_idx]))
                line = f"{timestamp()} | Address={address} | Value={raw_value} | HIGH ALARM | Setpoint={high_raw}"
                self._log(line, "alarm_high")
                self._write_persistent_line(line)
            elif old_state != "normal":
                setpoint = low_raw if old_state == "low" else high_raw
                line = f"{timestamp()} | Address={address} | Value={raw_value} | CLEARED | Setpoint={setpoint}"
                self._log(line, "clear")
                self._write_persistent_line(line)

        row["value_entry"].configure(
            fg=COLORS["red"] if new_state == "high" else
               (COLORS["yellow"] if new_state == "low" else COLORS["text"])
        )
        if new_state != "low":
            row["low_entry"].configure(bg=COLORS["panel_alt"])
        if new_state != "high":
            row["high_entry"].configure(bg=COLORS["panel_alt"])

    # ---------------------------------------------------------------
    # Blink/glow: HIGH alarms pulse red, LOW alarms pulse yellow,
    # connection LED pulses teal while connected
    # ---------------------------------------------------------------
    def _blink_tick(self):
        self.blink_on = not self.blink_on

        if self.connected:
            dot_color = COLORS["accent2"] if self.blink_on else COLORS["accent_dim"]
            self.led_canvas.itemconfig(self.led_dot, fill=dot_color)
            self.led_canvas.itemconfig(self.led_ring, outline=dot_color)

        red = COLORS["red"] if self.blink_on else COLORS["red_dim"]
        yellow = COLORS["yellow"] if self.blink_on else COLORS["yellow_dim"]
        for idx, state in self.alarm_state.items():
            row = self.rows[idx]
            if state == "high":
                row["high_entry"].configure(bg=red)
            elif state == "low":
                row["low_entry"].configure(bg=yellow)
        self.root.after(500, self._blink_tick)

    # ---------------------------------------------------------------
    def _log(self, text, tag="info"):
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"{ts}  {text}"
        self.log_lines.append(line)
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n", tag)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)


def main():
    # Ask Windows to render this app at true pixel resolution instead of
    # letting the OS upscale/blur it - this is what usually makes a Tkinter
    # app look "fuzzy" on a large/high-DPI monitor even though the exact
    # same code looks fine on a small laptop screen.
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

    root = tk.Tk()
    app = ModbusMasterApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
