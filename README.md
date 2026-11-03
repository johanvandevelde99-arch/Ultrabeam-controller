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
