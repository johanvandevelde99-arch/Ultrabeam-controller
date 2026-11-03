#!/usr/bin/env python3
"""UltraBeam Controller - standalone desktop app (Tkinter).

Run:      python ultrabeam_controller.py
Needs:    Python 3.8+ (Tkinter ships with it). Optional: pip install pyserial
CAT:      Radio CAT offers two sources: Hamlib rigctld over TCP (default 127.0.0.1:4532,
          start it first, e.g. rigctld -m <model> -r COM3 -s 38400) or a radio COM port that
          answers the Kenwood/Yaesu/Elecraft "FA;" command. Use Hamlib for Icom and others.
Startup:  settings (controller port, CAT source and address) are saved in %APPDATA%\\UltraBeamController;
          the app reconnects to whatever was connected last time unless you untick
          "Connect automatically at startup" under Connection.

RCU-06:   speaks the controller's native USB protocol (19200 8N1, framed packets with
          checksum) as described in Ultrabeam's "RCU-06 USB port description". Choose
          the controller's COM port under Connection. Until then the app is in demo mode.
          Commands used: 1 status, 2 retract, 3 frequency+direction, 4 calibrate (fw 4.41+),
          9 read element lengths, 10 progress, 12 modify element (fw 4.42+). The factory-only
          commands the document warns about are never sent.
"""
import json
import os
import queue
import socket
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # serial is optional
    serial = None

BANDS = [("40", 7.000, 7.300), ("30", 10.100, 10.150), ("20", 14.000, 14.350),
         ("17", 18.068, 18.168), ("15", 21.000, 21.450), ("12", 24.890, 24.990),
         ("10", 28.000, 29.700), ("6", 50.000, 54.000)]
FONT, BFONT = ("Tahoma", 8), ("Tahoma", 8, "bold")
MFONT, MBFONT = ("Tahoma", 6), ("Tahoma", 6, "bold")  # button matrix
GREEN, OK, WARN = "#167a3a", "#17a34a", "#e0a000"


CAT_SRC = ("Hamlib rigctld", "COM port")
SETTINGS = Path(os.environ.get("APPDATA") or Path.home()) / "UltraBeamController" / "settings.json"


def load_settings():
    try:
        return json.loads(SETTINGS.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_settings(d):
    try:
        SETTINGS.parent.mkdir(parents=True, exist_ok=True)
        SETTINGS.write_text(json.dumps(d, indent=2), encoding="utf-8")
    except OSError:
        pass  # settings are a convenience; never block the app on them


def band_of(f):
    return next((b for b in BANDS if b[1] - 1e-6 <= f <= b[2] + 1e-6), None)


DIRS = ["Normal", "180\u00b0", "Bi-dir"]
STX, ETX, DLE = 0xF5, 0xFA, 0xF6
REPLY = {0: "OK", 1: "BAD (invalid command)", 2: "PAR (bad parameter, e.g. frequency not reachable)",
         3: "ERR (error while executing)"}


def _quote(b):
    return bytes([DLE, b & 0x7F]) if b in (STX, ETX, DLE) else bytes([b])


def build_frame(seq, com, data=b""):
    """STX, quoted SEQ/COM/DATA/CHK, ETX. CHK starts at 0x55: chk = (chk ^ byte) + 1."""
    chk, out = 0x55, bytearray([STX])
    for b in (seq, com, *data):
        chk = ((chk ^ b) + 1) & 0xFF
        out += _quote(b)
    out += _quote(chk)
    out.append(ETX)
    return bytes(out)


def read_frame(ser, timeout=1.0):
    """Return (seq, com, data) for the next valid packet, or None on timeout."""
    end, buf, dle, chk = time.time() + timeout, None, False, 0x55
    while time.time() < end:
        b = ser.read(1)
        if not b:
            continue
        b = b[0]
        if b == STX:
            buf, dle, chk = [], False, 0x55
        elif b == ETX:
            if buf is not None and chk == 1 and len(buf) >= 3:
                return buf[0], buf[1], bytes(buf[2:-1])
            buf = None
        elif buf is not None:
            if b == DLE:
                dle = True
                continue
            if dle:
                b |= 0x80
                dle = False
            chk = ((chk ^ b) + 1) & 0xFF
            buf.append(b)
    return None


class App:
    def __init__(self, root):
        self.root = root
        root.title("UltraBeam Controller")
        root.resizable(False, False)
        self.f, self.band, self.dir, self.retracted = 14.2, BANDS[2], "Normal", False
        self.last = {b[0]: round(b[1] + min(b[2] - b[1], 0.2), 3) for b in BANDS}
        self.ser = self.sock = self.radio = self.job = None
        self.q = queue.Queue()
        self.cmdq = queue.Queue()
        self.cmd_t, self.fw_logged, self.missed, self.fw = 0.0, False, False, None
        self._upd = False
        self.fvar = tk.StringVar()
        self.offset = tk.StringVar(value="0")
        self.thr = tk.StringVar(value="10")
        self.follow = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="Ready")
        self.mode = tk.StringVar(value="Demo mode")
        self.popups = {}
        self.settings = load_settings()
        self.cat_busy = False
        self.autocon = tk.BooleanVar(value=self.settings.get("autoconnect", True))
        self.autocon.trace_add("write", lambda *_: self.remember("autoconnect", self.autocon.get()))
        self.build_ui()
        self.build_cat()
        self.build_conn()
        self.build_elem()
        for v in (self.offset, self.thr):
            v.trace_add("write", lambda *_: self.apply_radio())
        self.follow.trace_add("write", lambda *_: self.apply_radio())
        self.set_freq(14.2)
        self.log("Demo mode ready. Pick a band to retune.")
        root.after(100, self.poll)
        root.after(600, self.autoconnect)

    # ---------------------------------------------------------------- UI
    def build_ui(self):
        r = self.root
        hdr = tk.Frame(r)
        hdr.pack(fill="x", padx=2, pady=(2, 0))
        self.catb = tk.Button(hdr, text="Radio CAT \u25be", font=FONT, relief="flat",
                              command=lambda: self.toggle("cat", self.catb))
        self.conb = tk.Button(hdr, text="Connection \u25be", font=FONT, relief="flat",
                              command=lambda: self.toggle("conn", self.conb))
        self.elb = tk.Button(hdr, text="Elements \u25be", font=FONT, relief="flat",
                             command=lambda: self.toggle("elem", self.elb))
        self.catb.pack(side="left")
        self.conb.pack(side="left")
        self.elb.pack(side="left")
        tk.Label(hdr, textvariable=self.mode, font=FONT, fg="#404040").pack(side="right", padx=4)
        tk.Label(hdr, textvariable=self.status, font=FONT).pack(side="right")
        self.led = tk.Canvas(hdr, width=11, height=11, highlightthickness=0, bg=hdr["bg"])
        self.ledid = self.led.create_oval(1, 1, 10, 10, fill=OK)
        self.led.pack(side="right", padx=(0, 4))

        top = tk.Frame(r, bd=1, relief="groove", padx=8, pady=8)
        top.pack(fill="x", padx=8, pady=(8, 4))
        self.lcd = tk.Label(top, font=("Consolas", 18), bg="black", fg="#46ff6e",
                            bd=2, relief="sunken", padx=8, width=10)
        self.lcd.grid(row=0, column=0, sticky="nw", padx=(0, 10))
        of = tk.Frame(top)
        of.grid(row=1, column=0, sticky="w", pady=(8, 0))
        e = tk.Entry(of, textvariable=self.fvar, width=8, font=FONT, justify="right")
        e.pack(side="left")
        e.bind("<Return>", lambda _: self.set_freq(self.fvar.get()))
        tk.Button(of, text="MHz", font=FONT, pady=0,
                  command=lambda: self.set_freq(self.fvar.get())).pack(side="left", padx=(4, 10))
        tk.Label(of, text="Offset", font=FONT).pack(side="left")
        tk.Entry(of, textvariable=self.offset, width=4, font=FONT,
                 justify="right").pack(side="left", padx=4)
        m = tk.Frame(top)
        m.grid(row=0, column=1, rowspan=2, sticky="n")

        def btn(txt, cmd, row, col, bold=False, fg="black"):
            b = tk.Button(m, text=txt, command=cmd, font=MFONT,  # same font everywhere so every button is the same size
                          width=8, pady=0, fg=fg)
            b.grid(row=row, column=col, padx=1, pady=1)
            return b

        for c, col in enumerate([[("<<", -100), ("<", -10)], [(">>", 100), (">", 10)]]):
            for rr, (t, d) in enumerate(col):
                btn(t, lambda d=d: self.step(d), rr, c, True)
        btn("Retract", self.retract, 2, 0, fg="#b00020")
        btn("Cal", self.calibrate, 2, 1)
        self.dirb = {d: btn(d, lambda d=d: self.set_dir(d), rr, 2)
                     for rr, d in enumerate(["Normal", "180\u00b0", "Bi-dir"])}

        bf = tk.Frame(r, bd=1, relief="groove", padx=6, pady=6)
        bf.pack(fill="x", padx=8, pady=4)
        self.bb = {}
        for b in BANDS:
            x = tk.Button(bf, text=b[0], font=("Tahoma", 7, "bold"), width=3, pady=0,
                          command=lambda b=b: self.set_freq(self.last[b[0]]))
            x.pack(side="left", padx=1)
            self.bb[b[0]] = x

        sf = tk.Frame(r, bd=1, relief="groove", padx=6, pady=2)
        sf.pack(fill="x", padx=8, pady=(4, 8))
        self.sl = tk.Scale(sf, from_=0, to=1, orient="horizontal", showvalue=False,
                           width=6, bd=1, highlightthickness=0, command=self.on_slide)
        self.sl.pack(fill="x")


    def popup(self, key, title):
        t = tk.Toplevel(self.root)
        t.title(title)
        t.withdraw()
        t.resizable(False, False)
        t.transient(self.root)
        t.protocol("WM_DELETE_WINDOW", t.withdraw)
        t.bind("<Escape>", lambda e: t.withdraw())
        self.popups[key] = t
        return t

    def toggle(self, key, btn):
        t = self.popups[key]
        if t.state() != "withdrawn":
            t.withdraw()
            return
        for o in self.popups.values():
            o.withdraw()
        self.root.update_idletasks()
        t.geometry(f"+{btn.winfo_rootx()}+{btn.winfo_rooty() + btn.winfo_height()}")
        t.deiconify()
        t.lift()
        if key == "elem":
            self.read_elements()

    def build_cat(self):
        t = self.popup("cat", "Radio CAT")
        pad = dict(padx=8, pady=3)
        self.cinfo = tk.StringVar(value="Radio   -\nOffset  -\nAntenna -")
        tk.Label(t, textvariable=self.cinfo, font=("Consolas", 10), bg="black", fg="#46ff6e",
                 justify="left", anchor="w", padx=8, pady=4).pack(fill="x", **pad)
        row = tk.Frame(t)
        row.pack(fill="x", **pad)
        tk.Label(row, text="Radio via", font=FONT).pack(side="left")
        self.csrc = tk.StringVar(value=self.settings.get("cat_src", CAT_SRC[0]))
        cb = ttk.Combobox(row, textvariable=self.csrc, values=CAT_SRC, width=16, state="readonly")
        cb.pack(side="left", padx=4)
        cb.bind("<<ComboboxSelected>>", lambda _: self.cat_src_changed())
        self.cholder = tk.Frame(t)
        self.cholder.pack(fill="x", **pad)
        self.fh = tk.Frame(self.cholder)  # Hamlib rigctld address
        self.host = tk.StringVar(value=self.settings.get("cat_host", "127.0.0.1"))
        self.port = tk.StringVar(value=self.settings.get("cat_tcp", "4532"))
        tk.Entry(self.fh, textvariable=self.host, width=15, font=FONT).pack(side="left")
        tk.Entry(self.fh, textvariable=self.port, width=6, font=FONT).pack(side="left", padx=4)
        self.fc = tk.Frame(self.cholder)  # radio COM port
        r1 = tk.Frame(self.fc)
        r1.pack(anchor="w")
        self.ccb = ttk.Combobox(r1, width=10, values=self.ports())
        self.ccb.set(self.settings.get("cat_com", ""))
        self.ccb.pack(side="left")
        tk.Button(r1, text="\u21bb", font=FONT, pady=0,
                  command=lambda: self.ccb.config(values=self.ports())).pack(side="left", padx=2)
        self.baud = tk.StringVar(value=self.settings.get("cat_baud", "9600"))
        ttk.Combobox(r1, textvariable=self.baud, width=7, values=("4800", "9600", "19200", "38400",
                     "57600", "115200")).pack(side="left", padx=4)
        tk.Label(r1, text="baud", font=FONT).pack(side="left")
        tk.Label(self.fc, font=FONT, fg="#404040", justify="left",
                 text="Reads the frequency with the Kenwood/Yaesu/Elecraft 'FA;'\n"
                      "command. For Icom and other radios choose Hamlib.").pack(anchor="w")
        self.show_cat_src()
        row = tk.Frame(t)
        row.pack(fill="x", **pad)
        tk.Button(row, text="Connect CAT", font=FONT, pady=0,
                  command=self.cat_connect).pack(side="left")
        tk.Button(row, text="Disconnect", font=FONT, pady=0,
                  command=self.cat_disconnect).pack(side="left", padx=4)
        row = tk.Frame(t)
        row.pack(fill="x", **pad)
        tk.Label(row, text="Only retune when radio moves more than", font=FONT).pack(side="left")
        ttk.Combobox(row, textvariable=self.thr, width=4, values=("1", "5", "10", "25", "50")
                     ).pack(side="left", padx=4)
        tk.Label(row, text="kHz", font=FONT).pack(side="left")
        tk.Checkbutton(t, text="Follow radio automatically", variable=self.follow,
                       font=FONT).pack(anchor="w", **pad)
        row = tk.Frame(t)
        row.pack(fill="x", **pad)
        self.tf = tk.StringVar(value="14.205")
        tk.Label(row, text="Test without a radio (MHz)", font=FONT).pack(side="left")
        tk.Entry(row, textvariable=self.tf, width=9, font=FONT).pack(side="left", padx=4)
        tk.Button(row, text="Simulate radio", font=FONT, pady=0,
                  command=self.simulate).pack(side="left")
        self.cstat = tk.StringVar(value="Not connected")
        tk.Label(t, textvariable=self.cstat, font=FONT, fg="#404040").pack(anchor="w", **pad)

    def build_elem(self):
        t = self.popup("elem", "Element lengths")
        pad = dict(padx=8, pady=2)
        self.evars = [tk.StringVar(value="-") for _ in range(6)]
        for i, v in enumerate(self.evars):
            row = tk.Frame(t)
            row.pack(fill="x", **pad)
            tk.Label(row, text=f"Element {i + 1}", width=10, anchor="w", font=FONT).pack(side="left")
            tk.Entry(row, textvariable=v, width=7, font=FONT, justify="right").pack(side="left")
            tk.Label(row, text="mm", font=FONT).pack(side="left", padx=(2, 6))
            tk.Button(row, text="Set", font=FONT, pady=0,
                      command=lambda i=i: self.set_element(i)).pack(side="left")
        tk.Button(t, text="Read from controller", font=FONT, pady=0,
                  command=self.read_elements).pack(**pad)
        tk.Label(t, font=FONT, fg="#404040", justify="left",
                 text="0 mm = element not used at this frequency/direction.\n"
                      "Set needs firmware 4.42+. The controller stores changes\n"
                      "permanently 12 s after the last edit - don't power it off before then."
                 ).pack(anchor="w", **pad)

    def build_conn(self):
        t = self.popup("conn", "Connection")
        pad = dict(padx=8, pady=3)
        row = tk.Frame(t)
        row.pack(fill="x", **pad)
        self.pcb = ttk.Combobox(row, width=16, values=self.ports())
        self.pcb.pack(side="left")
        ports = self.ports()
        self.pcb.set(self.settings.get("port") or (ports[0] if len(ports) == 1 else ""))
        self.pcb.bind("<<ComboboxSelected>>", lambda _: self.remember_port())
        tk.Button(row, text="\u21bb", font=FONT, pady=0,
                  command=lambda: self.pcb.config(values=self.ports())).pack(side="left", padx=2)
        tk.Button(row, text="Connect", font=FONT, pady=0,
                  command=self.ser_connect).pack(side="left", padx=4)
        tk.Button(row, text="Disconnect", font=FONT, pady=0,
                  command=self.ser_disconnect).pack(side="left", padx=4)
        tk.Checkbutton(t, text="Connect automatically at startup", variable=self.autocon,
                       font=FONT).pack(anchor="w", **pad)
        if serial is None:
            tk.Label(t, text="pyserial not installed (pip install pyserial)", font=FONT,
                     fg="#b00020").pack(anchor="w", **pad)
        self.logw = tk.Text(t, width=64, height=8, font=("Consolas", 9), state="disabled")
        self.logw.pack(**pad)

    # ------------------------------------------------------------ logic
    def log(self, msg):
        self.logw.config(state="normal")
        self.logw.insert("end", time.strftime("%H:%M:%S  ") + msg + "\n")
        self.logw.see("end")
        self.logw.config(state="disabled")

    def tx(self, com, data, desc):
        """Log a command and, when the RCU-06 is connected, queue it for the serial thread."""
        self.log(f"\u2192 {desc}")
        self.cmd_t = time.time()
        if self.ser:
            self.cmdq.put((com, data))

    def freq_bytes(self, f):
        khz = round(f * 1000)
        return bytes([khz & 255, khz >> 8, DIRS.index(self.dir)])

    def moving(self, text, secs, then=None):
        if self.job:
            self.root.after_cancel(self.job)
        self.status.set(text)
        self.led.itemconfig(self.ledid, fill=WARN)
        if self.ser:  # real hardware: status polling reports when the motors stop
            return

        def done():
            self.job = None
            self.status.set("Retracted" if self.retracted else "Ready")
            self.led.itemconfig(self.ledid, fill=OK)
            if then:
                then()
        self.job = self.root.after(int(secs * 1000), done)

    def set_freq(self, f):
        try:
            f = round(float(f), 3)
        except ValueError:
            return
        b = band_of(f)
        if not b:
            self.log(f"{f:.3f} MHz is outside the supported bands.")
            self.status.set("Out of band")
            return
        self.f, self.band, self.retracted = f, b, False
        self.last[b[0]] = f
        self.tx(3, self.freq_bytes(f), f"FREQ {round(f * 1000)} kHz ({self.dir})")
        self.refresh()
        self.moving("Moving elements...", 0.8)

    def refresh(self):
        self.lcd.config(text=f"{self.f:6.3f} MHz")
        self.fvar.set(f"{self.f:.3f}")
        face = self.root.cget("bg")
        for n, x in self.bb.items():
            on = n == self.band[0]
            x.config(relief="sunken" if on else "raised", bg=GREEN if on else face,
                     fg="white" if on else "black", activebackground=GREEN if on else face)
        for d, x in self.dirb.items():
            on = d == self.dir
            x.config(relief="sunken" if on else "raised", font=MFONT,
                     bg=GREEN if on else self.root.cget("bg"), fg="white" if on else "black",
                     activebackground=GREEN if on else self.root.cget("bg"))
        self._upd = True
        self.sl.config(from_=round(self.band[1] * 1000), to=round(self.band[2] * 1000))
        self.sl.set(round(self.f * 1000))
        self._upd = False

    def on_slide(self, v):
        if not self._upd:
            self.set_freq(float(v) / 1000)

    def step(self, khz):
        lo, hi = self.band[1], self.band[2]
        self.set_freq(min(hi, max(lo, round(self.f + khz / 1000, 3))))

    def set_dir(self, d):
        self.dir = d
        self.refresh()
        if self.ser and self.retracted:
            self.log(f"Direction {d} will apply with the next frequency change.")
            return
        self.tx(3, self.freq_bytes(self.f), f"DIR {d}")
        self.moving("Moving elements...", 0.8)

    def retract(self):
        self.retracted = True
        self.tx(2, b"", "RETRACT")
        self.moving("Retracting...", 1.5)

    def calibrate(self):
        if self.ser:
            if self.fw and self.fw < (4, 41):
                self.log("Calibrate needs RCU-06 firmware 4.41 or newer.")
                return
            if not messagebox.askyesno("Calibrate", "Calibrate drives every element in as if it were fully "
                                       "extended. This stresses the antenna mechanics, so only do it if "
                                       "the controller has lost steps.\n\nContinue?"):
                return
            self.retracted = True  # elements end up fully in; pick a band to extend again
            self.tx(4, b"", "CALIBRATE")
            self.moving("Calibrating...", 0)
            return
        self.log("\u2192 CALIBRATE (demo)")
        self.moving("Calibrating...", 2.5, then=lambda: self.set_freq(self.f))

    def read_elements(self):
        if self.ser:
            self.tx(9, b"", "READ ELEMENTS")
        else:  # demo: simulated lengths
            self.show_elems([round(71250 / self.f * k) for k in (1.05, 1.0, 0.96, 0.94)] + [0, 0])

    def show_elems(self, mm):
        for v, x in zip(self.evars, mm):
            v.set(str(x))

    def set_element(self, i):
        try:
            mm = int(float(self.evars[i].get()))
        except ValueError:
            return
        if not 0 < mm < 65536:
            return
        if not self.ser:
            self.log("Connect to the RCU-06 to change element lengths.")
            return
        if self.fw and self.fw < (4, 42):
            self.log("Changing element lengths needs RCU-06 firmware 4.42 or newer.")
            return
        if not messagebox.askyesno("Set element", f"Set element {i + 1} to {mm} mm?\n\n"
                                   "The element moves now and the controller stores the change "
                                   "permanently 12 s after your last edit."):
            return
        self.tx(12, bytes([i, 0, mm & 255, mm >> 8]), f"ELEMENT {i + 1} -> {mm} mm")
        self.root.after(2500, self.read_elements)

    # -------------------------------------------------------------- CAT
    def simulate(self):
        try:
            self.radio = float(self.tf.get())
        except ValueError:
            return
        self.log(f"Simulated radio at {self.radio:.3f} MHz")
        self.apply_radio()

    def apply_radio(self):
        if self.radio is None:
            return
        try:
            off = float(self.offset.get())
            thr = float(self.thr.get())
        except ValueError:
            return
        target = round(self.radio + off / 1000, 3)
        self.cinfo.set(f"Radio   {self.radio:.3f} MHz\nOffset  {off:+.1f} kHz\n"
                       f"Antenna {target:.3f} MHz")
        if not self.follow.get():
            return
        b = band_of(target)
        if not b:
            self.cstat.set("Antenna freq out of band")
        elif abs(target - self.f) * 1000 < thr and b is self.band:
            self.cstat.set("Tracking")
        else:
            self.cstat.set("Retuning")
            self.set_freq(target)

    def show_cat_src(self):
        self.fh.pack_forget()
        self.fc.pack_forget()
        (self.fh if self.csrc.get() == CAT_SRC[0] else self.fc).pack(anchor="w")

    def cat_src_changed(self):
        self.show_cat_src()
        self.remember_cat()

    def cat_connect(self):
        if self.sock or self.cat_busy:
            return
        src = self.csrc.get()
        if src != CAT_SRC[0] and serial is None:
            self.cstat.set("pyserial not installed")
            self.log("A COM port radio needs pyserial (pip install pyserial).")
            return
        self.cat_busy = True
        self.cstat.set("Connecting...")
        threading.Thread(target=self.cat_open, daemon=True, args=(
            src, self.host.get(), self.port.get(), self.ccb.get(), self.baud.get())).start()

    def cat_open(self, src, host, tcp, com, baud):
        try:  # runs in a thread so a slow or missing server can't freeze the window
            if src == CAT_SRC[0]:
                s = socket.create_connection((host, int(tcp)), timeout=3)
                s.settimeout(2)
                where = f"{host}:{tcp}"
            else:
                s = serial.Serial(com, int(baud), timeout=0.5, write_timeout=1)
                where = f"{com} at {baud} baud"
        except Exception as e:
            self.q.put(("catfail", str(e)))
            return
        self.sock = s
        self.q.put(("catok", (src, where)))
        self.cat_loop(s, src)

    def cat_loop(self, s, src):
        quiet = 0
        try:
            while self.sock is s:
                if src == CAT_SRC[0]:
                    s.sendall(b"f\n")
                    data = s.recv(256).decode(errors="ignore").strip()
                    if not data:
                        break
                    line = data.splitlines()[0]
                    if line.startswith("RPRT"):
                        self.q.put(("log", "rigctld: " + line))
                    else:
                        try:
                            self.q.put(("freq", float(line) / 1e6))
                        except ValueError:
                            pass
                else:
                    s.reset_input_buffer()
                    s.write(b"FA;")
                    txt = s.read_until(b";").decode(errors="ignore").strip()
                    if txt.startswith("FA") and txt.endswith(";") and txt[2:-1].isdigit():
                        self.q.put(("freq", int(txt[2:-1]) / 1e6))
                        quiet = 0
                    else:
                        quiet += 1
                        if quiet == 6:
                            self.q.put(("log", "No CAT reply to 'FA;'. Check port, baud and the "
                                               "radio's CAT settings, or use Hamlib."))
                time.sleep(0.5)
        except Exception as e:  # socket and serial errors vary by platform
            if self.sock is s:
                self.q.put(("log", f"CAT error: {e}"))
        finally:
            self.q.put(("closed", s))

    def cat_disconnect(self):
        s, self.sock = self.sock, None
        self.remember_cat(on=False)
        if s:
            try:
                s.close()
            except Exception:
                pass
            self.cstat.set("Not connected")
            self.log("CAT disconnected.")

    def remember(self, key, value):
        self.settings[key] = value
        save_settings(self.settings)

    def remember_cat(self, on=None):
        self.settings.update(cat_src=self.csrc.get(), cat_host=self.host.get(), cat_tcp=self.port.get(),
                             cat_com=self.ccb.get(), cat_baud=self.baud.get())
        if on is not None:
            self.settings["cat_on"] = on
        save_settings(self.settings)

    def autoconnect(self):
        if not self.autocon.get():
            return
        if self.settings.get("ser_on") and self.pcb.get():
            self.log("Auto-connecting to the RCU-06...")
            self.ser_connect()
        if self.settings.get("cat_on"):
            self.log("Auto-connecting CAT...")
            self.cat_connect()

    def poll(self):
        try:
            while True:
                kind, val = self.q.get_nowait()
                if kind == "freq":
                    self.radio = val
                    self.apply_radio()
                elif kind == "log":
                    self.log(val)
                elif kind == "elems":
                    self.show_elems(val)
                elif kind == "progress":
                    dist, done = val
                    if dist:
                        self.status.set(f"Moving elements... {min(100, round(done * 100 / 60))}%")
                elif kind == "status":
                    self.on_status(*val)
                elif kind == "serclosed":
                    if self.ser is val:
                        self.ser = None
                    if not self.ser:
                        self.mode.set("Demo mode")
                elif kind == "catok":
                    self.cat_busy = False
                    self.cstat.set("Connected")
                    self.log(f"CAT connected: {val[0]} ({val[1]})")
                    self.remember_cat(on=True)
                elif kind == "catfail":
                    self.cat_busy = False
                    self.cstat.set("Connection failed")
                    self.log(f"CAT connection failed: {val}")
                elif kind == "closed":
                    try:
                        val.close()
                    except Exception:
                        pass
                    if self.sock is val:
                        self.sock = None
                        self.cstat.set("Not connected")
                        self.log("CAT connection closed.")
        except queue.Empty:
            pass
        self.root.after(100, self.poll)

    # ----------------------------------------------------------- serial
    def ports(self):
        return [p.device for p in list_ports.comports()] if serial else []

    def remember_port(self, on=None):
        self.settings["port"] = self.pcb.get()
        if on is not None:
            self.settings["ser_on"] = on
        save_settings(self.settings)

    def ser_connect(self):
        if serial is None or self.ser:
            return
        try:
            ser = serial.Serial(self.pcb.get(), 19200, timeout=0.2, write_timeout=1)
        except (OSError, ValueError) as e:
            self.log(f"Could not open port: {e}")
            return
        self.remember_port(on=True)
        self.ser, self.fw_logged, self.missed = ser, False, False
        while not self.cmdq.empty():
            self.cmdq.get_nowait()
        self.mode.set("Connecting...")
        self.log(f"Serial port {self.pcb.get()} opened (19200 8N1).")
        threading.Thread(target=self.ser_loop, args=(ser,), daemon=True).start()

    def ser_disconnect(self):
        if self.ser:
            self.ser = None  # the serial thread notices, closes the port and exits
            self.remember_port(on=False)
            self.mode.set("Demo mode")
            self.log("Serial port closed.")

    def xfer(self, ser, seq, com, data):
        """Send one request and return the reply (seq, com, data) or None."""
        for wait in ((3.0,) if com == 4 else (1.0, 1.0, 3.0)):  # never retry a calibrate
            ser.reset_input_buffer()
            ser.write(build_frame(seq, com, data))
            reply = read_frame(ser, wait)
            if reply or self.ser is not ser:
                return reply
        return None

    def handle_reply(self, com, rcom, d):
        if rcom != 0:
            if com != 1:
                self.q.put(("log", f"RCU-06 replied {REPLY.get(rcom, rcom)} to command {com}"))
        elif com == 1 and len(d) >= 10:
            self.q.put(("status", (f"{d[1]}.{d[0]:02d}", d[3] | d[4] << 8, d[6] & 0x0F, d[9] != 0)))
        elif com == 9 and len(d) >= 12:
            self.q.put(("elems", [d[2 * i] | d[2 * i + 1] << 8 for i in range(6)]))
        elif com == 10 and len(d) >= 4:
            self.q.put(("progress", (d[0] | d[1] << 8, d[2] | d[3] << 8)))
        elif com not in (1, 9, 10):
            self.q.put(("log", "RCU-06 replied OK"))

    def ser_loop(self, ser):
        seq, last = 0, 0.0
        try:
            while self.ser is ser:
                try:
                    com, data = self.cmdq.get(timeout=0.25)
                except queue.Empty:
                    if time.time() - last < 1.0:
                        continue
                    com, data, last = 1, b"", time.time()  # status poll
                reply = self.xfer(ser, seq, com, data)
                seq = (seq + 1) & 0x7F
                if reply is None:
                    if not self.missed:
                        self.missed = True
                        self.q.put(("log", "No reply from the RCU-06. Check the USB cable and port."))
                    continue
                self.missed = False
                self.handle_reply(com, reply[1], reply[2])
                if com == 1 and reply[1] == 0 and len(reply[2]) >= 10 and reply[2][9]:
                    prog = self.xfer(ser, seq, 10, b"")  # motors moving: ask for progress
                    seq = (seq + 1) & 0x7F
                    if prog:
                        self.handle_reply(10, prog[1], prog[2])
        except Exception as e:  # serial errors vary by platform
            if self.ser is ser:
                self.q.put(("log", f"Serial error: {e}"))
        finally:
            try:
                ser.close()
            except Exception:
                pass
            self.q.put(("serclosed", ser))

    def on_status(self, fw, freq, dr, moving):
        self.fw = tuple(int(x) for x in fw.split("."))
        if not self.fw_logged:
            self.fw_logged = True
            self.mode.set(f"RCU-06 v{fw}")
            self.log(f"RCU-06 firmware v{fw}, currently {freq} kHz")
        recent = time.time() - self.cmd_t < 2.5
        if moving:
            self.status.set("Moving elements...")
            self.led.itemconfig(self.ledid, fill=WARN)
        elif not recent and self.job is None:
            self.status.set("Retracted" if self.retracted else "Ready")
            self.led.itemconfig(self.ledid, fill=OK)
        if recent:
            return
        f, b, changed = freq / 1000, band_of(freq / 1000), False
        if b and abs(f - self.f) > 0.0005:  # keep in step with the controller's front panel
            self.f, self.band, self.retracted, changed = f, b, False, True
            self.last[b[0]] = f
        if dr < 3 and DIRS[dr] != self.dir:
            self.dir, changed = DIRS[dr], True
        if changed:
            self.refresh()


if __name__ == "__main__":
    tk_root = tk.Tk()
    App(tk_root)
    tk_root.mainloop()
