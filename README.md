# ModScan Monitor

An open-source **Modbus TCP/RTU master and alarm monitor**, built with Python and Tkinter — a free alternative to WinTech **ModScan32**, powered by the `pymodbus` library.

Connects to Modbus devices over **TCP/IP** or a **serial/COM port**, polls registers/coils continuously, and gives you a live data table, a 20-row alarm monitor with configurable high/low setpoints, an event log, and scheduled data logging with PDF export — all in one desktop app.

## Features

- **Live data table** — configurable Start Address, Length, Device ID, Modbus point type (Coil / Discrete Input / Holding Register / Input Register), and value conversion (Decimal / Integer / Hex / Binary)
- **20-row Alarm Monitor** — independent point type per row, high/low setpoints, alarm counts, and visual alerts (rows flash red/yellow on alarm)
- **Event Log** — timestamped connection, alarm, and clear events; exportable to TXT or PDF
- **Data Report window** — scheduled logging (by second/minute/hour) of up to 9 addresses with custom labels and decimal-place formatting, exportable as a full PDF report
- **Modbus TCP and Modbus RTU/ASCII (serial)** support
- Classic **Modicon addressing** (e.g. 40001+ for Holding Registers), matching ModScan32 conventions
- Dark, high-contrast UI with a connection status LED and hover feedback throughout

## Requirements

- Windows or macOS (Tkinter ships with standard Python installers); on Linux: `sudo apt-get install python3-tk`
- Python 3.10+
- `pip install "pymodbus>=3.12.1" "pymodbus[serial]" reportlab`

## Running from source

```bash
python modscan_final.py
```

## Download

Prebuilt Windows executable available under [Releases](../../releases) — no Python installation required.

> **Note:** This tool connects to Modbus devices on your local network or a physical serial port. It must be run on a machine that has direct network/serial access to your equipment — it cannot be hosted as a remote web service.

## Addressing reference

| Point Type | Address Range | Protocol Offset |
|---|---|---|
| Coil | 00001–09999 | typed − 1 |
| Discrete Input | 10001–19999 | typed − 10001 |
| Input Register | 30001–39999 | typed − 30001 |
| Holding Register | 40001–49999 | typed − 40001 |

## License

*(Add your preferred license here — e.g. MIT, GPL, or "All rights reserved" if proprietary.)*
