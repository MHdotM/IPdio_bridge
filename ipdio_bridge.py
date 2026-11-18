#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dnt IPdio mini Frontpanel & S/PDIF Bridge für AbracaDABra auf Raspberry Pi 4
- Kein Software-Lautstärkeregler: Audioausgabe via S/PDIF immer bit-perfekt 100%
- 2x16 LCD (HD44780 4-Bit): Kontrast (Pin 3 V0) fest über R2 an GND;
  Backlight-Helligkeit (Pin 15 A) über Q19 (S8050 NPN) -> Q18 (S8550 PNP) an GPIO 12 (PWM0)
- Dreh-Drück-Rad: Drehen = Senderliste durchblättern / Menüwert ändern, Druck = Auswählen
- Favoriten-Tasten I, II, III (|<<, >||, >>|):
    * Kurzer Druck: Gespeicherten Favoriten abrufen (in WLAN-Key-Eingabe: Zeichen-Sprung)
    * Langer Druck (>= 1.2s): Aktuell gehörten Sender auf Taste speichern
- Reply-Taste: Öffnet Display-Menü (1. Senderinfo, 2. Sendersuchlauf, 3. Helligkeit, 4. WLAN / WiFi)
- Back-Taste: Zurück / Menü schließen (in WLAN-Key-Eingabe: Zeichen löschen)
- Persistente Speicherung von Favoriten & Helligkeit in /opt/dab-radio/radio_state.json
"""

import os
import time
import json
import logging
import subprocess
import threading
import unicodedata
import urllib.request
import urllib.error
import RPi.GPIO as GPIO

# ==========================================
# GPIO PIN BELEGUNG (BCM NUMMERIERUNG)
# ==========================================
# LCD HD44780 (4-Bit Modus)
LCD_RS = 25  # Pin 22
LCD_E  = 24  # Pin 18
LCD_D4 = 23  # Pin 16
LCD_D5 = 27  # Pin 13
LCD_D6 = 22  # Pin 15
LCD_D7 = 10  # Pin 19

# Display Backlight Treiberstufe Q19 (S8050) -> Q18 (S8550) via Hardware PWM0
# PCB Pin 11 geht an R4 (15kOhm an +5V ausloeten! 2.2k-3.3k Vorwiderstand + 47k Pull-Down)
PWM_PIN_BRIGHTNESS = 12  # Pin 32 -> PCB Pin 11 -> Basis Q19 (Hardware PWM0)

# Dreh-Drück-Rad (Encoder Phasen mit 10nF "103" gegen GND auf PCB entprellt)
ENC_A   = 5   # Pin 29 <- PCB Pin 15
ENC_B   = 6   # Pin 31 <- PCB Pin 16

# 3x3 Tastenmatrix vom 20-Pin PCB-Stecker (ohne Leiterbahnen aufzutrennen!)
# Zeilen (Rows): werden beim Scan reihum kurz auf LOW gezogen (sonst Hi-Z)
ROW_P4  = 26  # Pin 37 <- PCB Pin 4  (Zeile 1: Power, Back, Fav I)
ROW_P3  = 8   # Pin 24 <- PCB Pin 3  (Zeile 2: Drehrad-Taster, Fav II)
ROW_P2  = 15  # Pin 10 <- PCB Pin 2  (Zeile 3: Reply, Fav III)
MATRIX_ROWS = [ROW_P4, ROW_P3, ROW_P2]

# Spalten (Cols): Eingänge mit internem Pull-Up
COL_P14 = 14  # Pin 8  <- PCB Pin 14 (Spalte 1: Power)
COL_P13 = 4   # Pin 7  <- PCB Pin 13 (Spalte 2: Back, Drehrad-Taster, Reply)
COL_P12 = 7   # Pin 26 <- PCB Pin 12 (Spalte 3: Fav I, Fav II, Fav III)
MATRIX_COLS = [COL_P14, COL_P13, COL_P12]

# Verifizierte Zuordnung (Row-GPIO, Col-GPIO) -> Taste
MATRIX_KEYMAP = {
    (ROW_P4, COL_P14): "POWER",    # PCB Pin 4 <-> Pin 14
    (ROW_P4, COL_P13): "BACK",     # PCB Pin 4 <-> Pin 13
    (ROW_P4, COL_P12): "FAV1",     # PCB Pin 4 <-> Pin 12
    (ROW_P3, COL_P13): "ENC_BTN",  # PCB Pin 3 <-> Pin 13
    (ROW_P3, COL_P12): "FAV2",     # PCB Pin 3 <-> Pin 12
    (ROW_P2, COL_P13): "REPLY",    # PCB Pin 2 <-> Pin 13
    (ROW_P2, COL_P12): "FAV3",     # PCB Pin 2 <-> Pin 12
}

ALL_LOGICAL_KEYS = ["POWER", "BACK", "REPLY", "FAV1", "FAV2", "FAV3", "ENC_BTN"]

ABRACADABRA_API_URL = "http://127.0.0.1:8080/api"
ABRACADABRA_SERVICE = "abracadabra.service"
STATE_FILE = "/opt/dab-radio/radio_state.json"
LONG_PRESS_SEC = 1.2
WIFI_IFACE = "wlan0"
ENC_STEPS_PER_DETENT = 4  # 4 Quadratur-Flanken = 1 Raste (Standard dnt IPdio mini Frontpanel)
ENC_IDLE_RESET_SEC = 0.5  # Teil-Schritte nach Drehpause zuruecksetzen

WIFI_KEY_TOKENS = (
    ["[OK]", "[<-]", "[CL]"]
    + list("abcdefghijklmnopqrstuvwxyz")
    + list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")
    + list("0123456789")
    + [" "]
    + list("!@#$%&*+-_.,:;?/=()")
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


# ==========================================
# HARDWARE / SOFTWARE PWM TREIBER (Q19 -> Q18)
# ==========================================
class DisplayBacklightPWM:
    """
    Nutzt primär das Linux Hardware-PWM-Interface (/sys/class/pwm/pwmchip0)
    für absolut flimmerfreies 20-kHz-Signal an der Basis von Q19 (S8050),
    welcher über R5 den High-Side-PNP Q18 (S8550) für LCD Pin 15 (A) schaltet.
    Fällt automatisch auf RPi.GPIO PWM zurück, falls sysfs nicht aktiviert ist.
    """
    PWM_CHIP = "/sys/class/pwm/pwmchip0"
    PERIOD_NS = 50000  # 20 kHz

    def __init__(self, bcm_pin, hw_channel=0):
        self.bcm_pin = bcm_pin
        self.hw_channel = hw_channel
        self.use_sysfs = False
        self.sw_pwm = None
        self.duty_percent = 80

        hw_path = f"{self.PWM_CHIP}/pwm{self.hw_channel}"
        try:
            if os.path.isdir(self.PWM_CHIP):
                if not os.path.isdir(hw_path):
                    with open(f"{self.PWM_CHIP}/export", "w") as f:
                        f.write(str(self.hw_channel))
                    time.sleep(0.05)
                with open(f"{hw_path}/period", "w") as f:
                    f.write(str(self.PERIOD_NS))
                with open(f"{hw_path}/enable", "w") as f:
                    f.write("1")
                self.use_sysfs = True
                logging.info(f"Hardware PWM{hw_channel} (GPIO {bcm_pin}) aktiv (20 kHz)")
        except Exception as e:
            logging.debug(f"Sysfs PWM{hw_channel} nicht verfügbar ({e}), nutze Software-PWM")

        if not self.use_sysfs:
            GPIO.setup(self.bcm_pin, GPIO.OUT)
            self.sw_pwm = GPIO.PWM(self.bcm_pin, 2000)
            self.sw_pwm.start(self.duty_percent)

    def set_percent(self, percent):
        self.duty_percent = max(0, min(100, int(percent)))
        if self.use_sysfs:
            duty_ns = int((self.duty_percent / 100.0) * self.PERIOD_NS)
            try:
                with open(f"{self.PWM_CHIP}/pwm{self.hw_channel}/duty_cycle", "w") as f:
                    f.write(str(duty_ns))
            except Exception as e:
                logging.warning(f"PWM write error: {e}")
        elif self.sw_pwm:
            self.sw_pwm.ChangeDutyCycle(self.duty_percent)

    def stop(self):
        if self.use_sysfs:
            try:
                with open(f"{self.PWM_CHIP}/pwm{self.hw_channel}/enable", "w") as f:
                    f.write("0")
            except Exception:
                pass
        elif self.sw_pwm:
            self.sw_pwm.stop()


# ==========================================
# HD44780 LCD TREIBER (4-BIT)
# ==========================================
class HD44780LCD:
    def __init__(self, rs, e, d4, d5, d6, d7):
        self.rs = rs
        self.e = e
        self.d4 = d4
        self.d5 = d5
        self.d6 = d6
        self.d7 = d7
        self._row_cache = ["", ""]
        self._signal_level = None

        GPIO.setup([self.rs, self.e, self.d4, self.d5, self.d6, self.d7], GPIO.OUT)
        self._init_lcd()
        self._load_custom_characters()

    def _pulse_enable(self):
        GPIO.output(self.e, False)
        time.sleep(0.000001)
        GPIO.output(self.e, True)
        time.sleep(0.000001)
        GPIO.output(self.e, False)
        time.sleep(0.00005)

    def _write_4bits(self, value):
        GPIO.output(self.d4, bool(value & 0x01))
        GPIO.output(self.d5, bool(value & 0x02))
        GPIO.output(self.d6, bool(value & 0x04))
        GPIO.output(self.d7, bool(value & 0x08))
        self._pulse_enable()

    def send_byte(self, value, is_data=False):
        GPIO.output(self.rs, is_data)
        self._write_4bits(value >> 4)
        self._write_4bits(value & 0x0F)

    def _init_lcd(self):
        time.sleep(0.05)
        GPIO.output(self.rs, False)
        self._write_4bits(0x03)
        time.sleep(0.005)
        self._write_4bits(0x03)
        time.sleep(0.005)
        self._write_4bits(0x03)
        time.sleep(0.001)
        self._write_4bits(0x02)

        self.send_byte(0x28)  # 2 Zeilen, 5x8 Font
        self.send_byte(0x0C)  # Display Ein, Cursor Aus
        self.send_byte(0x06)  # Auto-Increment
        self.clear()

    SIGNAL_GLYPHS = {
        1: [0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00],  # Stufe 1 (1 Balken links)
        2: [0x00, 0x00, 0x00, 0x00, 0x08, 0x18, 0x18, 0x00],  # Stufe 2 (2 Balken)
        3: [0x00, 0x00, 0x00, 0x04, 0x0C, 0x1C, 0x1C, 0x00],  # Stufe 3 (3 Balken)
        4: [0x00, 0x00, 0x02, 0x06, 0x0E, 0x1E, 0x1E, 0x00],  # Stufe 4 (4 Balken)
        5: [0x01, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x1F, 0x00],  # Stufe 5 (5 Balken voll rechts hoch)
    }

    def _load_custom_characters(self):
        custom_chars = [
            [0x0A, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F, 0x00],  # 0: ä
            self.SIGNAL_GLYPHS[5],                             # 1: Signal (dynamisch 1..5)
            [0x0A, 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x00],  # 2: Ä
            [0x0A, 0x0E, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x00],  # 3: Ö
            [0x0A, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x00],  # 4: Ü
            [0x0A, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00],  # 5: ö
            [0x0A, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0D, 0x00],  # 6: ü
            [0x0E, 0x11, 0x11, 0x16, 0x11, 0x11, 0x16, 0x10],  # 7: ß
        ]
        self.send_byte(0x40)
        for char in custom_chars:
            for byte in char:
                self.send_byte(byte, is_data=True)
        self._signal_level = 5
        self.set_cursor(0, 0)

    def set_signal_level(self, level):
        lvl = max(1, min(5, int(level)))
        if lvl == self._signal_level:
            return
        self.send_byte(0x48)  # CGRAM Slot 1 (0x40 + 1*8)
        for byte in self.SIGNAL_GLYPHS[lvl]:
            self.send_byte(byte, is_data=True)
        self._signal_level = lvl
        self.set_cursor(0, 0)

    def clear(self):
        self._row_cache = ["", ""]
        self.send_byte(0x01)
        time.sleep(0.002)

    def set_cursor(self, row, col):
        offsets = [0x00, 0x40]
        self.send_byte(0x80 | (offsets[row] + col))

    def _to_hd44780_char(self, ch):
        rom_map = {
            "ä": 0x00,
            "Ä": 0x02,
            "Ö": 0x03,
            "Ü": 0x04,
            "ö": 0x05,
            "ü": 0x06,
            "ß": 0x07,
            "·": ord("-"),
            "•": ord("-"),
            "–": ord("-"),
            "—": ord("-"),
            "−": ord("-"),
            "„": ord('"'),
            "“": ord('"'),
            "”": ord('"'),
            "«": ord('"'),
            "»": ord('"'),
            "‚": ord("'"),
            "‘": ord("'"),
            "’": ord("'"),
            "‹": ord("'"),
            "›": ord("'"),
            "´": ord("'"),
            "…": ord("."),
            "€": ord("E"),
            "£": ord("L"),
            "§": ord("S"),
            "×": ord("x"),
            "÷": ord("/"),
            "ø": ord("o"),
            "Ø": ord("O"),
            "ł": ord("l"),
            "Ł": ord("L"),
            "đ": ord("d"),
            "Đ": ord("D"),
            "°": ord("*"),
            " ": ord(" "),
        }
        if ch in rom_map:
            return rom_map[ch]
        code = ord(ch)
        if code < 128:
            return code
        for norm_ch in unicodedata.normalize("NFKD", ch):
            if unicodedata.category(norm_ch) != "Mn" and 32 <= ord(norm_ch) < 128:
                return ord(norm_ch)
        return ord("?")

    def write_line(self, text, row=0):
        norm = unicodedata.normalize("NFC", text).replace("…", "...")
        clean = (norm[:16]).ljust(16)
        if clean == self._row_cache[row]:
            return
        self._row_cache[row] = clean
        self.set_cursor(row, 0)
        for char in clean:
            self.send_byte(self._to_hd44780_char(char) if isinstance(char, str) else char, is_data=True)


# ==========================================
# DNT IPDIO MINI CONTROLLER DAEMON
# ==========================================
class IpdioMiniController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.lcd = HD44780LCD(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7)
        self.pwm_brightness = DisplayBacklightPWM(PWM_PIN_BRIGHTNESS, hw_channel=0)  # Q19 -> Q18

        # Encoder Pins (PCB hat 10nF "103" Hardware-Entprellung nach GND)
        GPIO.setup([ENC_A, ENC_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # 3x3 Tastenmatrix Initialisierung (Rows im Ruhezustand Hi-Z / Input!)
        for r_pin in MATRIX_ROWS:
            GPIO.setup(r_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        for c_pin in MATRIX_COLS:
            GPIO.setup(c_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Persistenter Zustand laden
        self.state = self._load_state()
        self._state_mtime = os.path.getmtime(STATE_FILE) if os.path.exists(STATE_FILE) else 0.0
        self.brightness = self.state.get("brightness", 85)
        self.favorites = self.state.get("favorites", {"1": None, "2": None, "3": None})

        self.pwm_brightness.set_percent(self.brightness)

        # Laufzeit-Zustand (initiale Senderliste aus state oder Fallback)
        self.power_on = True
        self.has_scanned_stations = bool(self.state.get("stations_scanned", False))
        if isinstance(self.state.get("stations"), list):
            self.stations = self.state.get("stations")
        else:
            self.stations = [
                {"id": "0xd210", "sid": "0xd210", "scids": 0, "name": "Deutschlandfunk", "channel": "5C"},
                {"id": "0xd220", "sid": "0xd220", "scids": 0, "name": "Dlf Kultur", "channel": "5C"},
                {"id": "0xd319", "sid": "0xd319", "scids": 0, "name": "sunshine live", "channel": "5C"},
                {"id": "0xd418", "sid": "0xd418", "scids": 0, "name": "RADIO BOB!", "channel": "5C"},
            ]
        self.current_station_idx = 0
        self.live_station_name = ""
        self._tuning_hold_until = 0.0
        self._ens_stable_sig = None
        self._ens_stable_since = 0.0
        self.browse_idx = 0
        self.current_dls = "dnt IPdio mini - S/PDIF 100% Bit-Perfekt"
        self.show_dab_time = False
        self.dab_time_str = ""
        self.standby_station = self.state.get("standby_station")
        self._audio_lock = threading.Lock()
        self.snr_db = 18.4
        self.station_info = {
            "snr": 18.4,
            "rf_level": -63.2,
            "fic_quality": 99.8,
            "tii_code": "--",
            "transmitter": "Kein TII",
            "ensemble": "DR Deutschland",
            "eid": "10BC",
            "channel": "5C",
            "freq_mhz": 178.352,
            "bitrate": 104,
            "codec": "DAB+ (AAC-LC)",
            "sample_rate": "48kHz Stereo",
            "sid": "D210",
            "scid": 1,
            "protection": "EEP-3A",
            "genre": "Information",
        }

        # UI Modus: 'normal' | 'browse' | 'menu' | 'station_info' | 'edit_brightness' | 'scanning'
        #           | 'wifi_menu' | 'wifi_status' | 'wifi_scanning' | 'wifi_list' | 'wifi_key' | 'wifi_connecting'
        self.ui_mode = "normal"
        self.menu_items = ["Senderinfo", "Sendersuchlauf", "Helligkeit", "WLAN / WiFi"]
        self.wifi_menu_items = ["WLAN Status", "Netzwerk suchen", "WLAN trennen"]
        self.menu_idx = 0
        self.info_page_idx = 0
        self.browse_timeout = 0
        self.toast_until = 0
        self.toast_line1 = ""
        self.toast_line2 = ""
        self.scan_progress = 0

        # WLAN Status & Setup Zustand
        self.wifi_menu_idx = 0
        self.wifi_status_page_idx = 0
        self.wifi_list_idx = 0
        self.wifi_networks = []
        self.wifi_target_net = {"ssid": "", "signal": 0, "secured": True}
        self.wifi_passphrase = ""
        self.wifi_char_idx = WIFI_KEY_TOKENS.index("a")
        self.wifi_status = {
            "connected": False,
            "ssid": "Nicht verbunden",
            "signal": 0,
            "ip": "--",
            "gateway": "--",
        }
        self._refresh_wifi_status()

        self.dls_scroll_pos = 0
        self.last_scroll_time = time.time()
        self.last_encoded = (GPIO.input(ENC_A) << 1) | GPIO.input(ENC_B)
        self.enc_lock = threading.Lock()
        self.enc_accum = 0
        self.last_enc_time = time.monotonic()

        # Tasten-Zustände für Kurz-/Lang-Druck Entprellung
        self.btn_press_start = {}
        self.btn_long_handled = {}

        # Encoder Interrupts (mit automatischem 1-kHz-Polling-Fallback für Bookworm/Kernel 6.6+)
        self._stop_event = threading.Event()
        try:
            GPIO.add_event_detect(ENC_A, GPIO.BOTH, callback=self._handle_encoder)
            GPIO.add_event_detect(ENC_B, GPIO.BOTH, callback=self._handle_encoder)
            logging.info("Drehregler nutzt GPIO Hardware-Interrupts.")
        except RuntimeError as e:
            logging.info(f"GPIO Edge-Detect nicht verfuegbar ({e}) - nutze 1-kHz-Drehregler-Polling.")
            for pin in (ENC_A, ENC_B):
                try:
                    GPIO.remove_event_detect(pin)
                except Exception:
                    pass
            threading.Thread(target=self._encoder_poll_loop, daemon=True).start()

        logging.info("dnt IPdio mini Hardware-Bridge gestartet (S/PDIF 100% Bit-Perfekt).")

    def _load_state(self):
        try:
            if os.path.exists(STATE_FILE):
                with open(STATE_FILE, "r") as f:
                    return json.load(f)
        except Exception as e:
            logging.warning(f"Statusdatei konnte nicht geladen werden: {e}")
        return {}

    def _sync_state_from_disk(self):
        if self.ui_mode in ("scanning", "scan_confirm"):
            return
        try:
            if not os.path.exists(STATE_FILE):
                return
            mtime = os.path.getmtime(STATE_FILE)
            if mtime == self._state_mtime:
                return
            self._state_mtime = mtime
            with open(STATE_FILE, "r") as f:
                data = json.load(f)
            if isinstance(data, dict) and isinstance(data.get("stations"), list):
                self.stations = data["stations"]
                self.has_scanned_stations = bool(data.get("stations_scanned", self.has_scanned_stations))
                if not self.stations:
                    self.current_station_idx = 0
                    self.browse_idx = 0
                elif self.current_station_idx >= len(self.stations):
                    self.current_station_idx = 0
                logging.info(f"Senderliste aus {STATE_FILE} aktualisiert ({len(self.stations)} Sender).")
        except Exception as e:
            logging.debug(f"Sync von {STATE_FILE}: {e}")

    def _save_state(self):
        try:
            os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
            data = {
                "brightness": self.brightness,
                "favorites": self.favorites,
                "stations": self.stations,
                "stations_scanned": self.has_scanned_stations,
                "standby_station": self.standby_station,
            }
            tmp_path = STATE_FILE + ".tmp"
            with open(tmp_path, "w") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp_path, STATE_FILE)
            self._state_mtime = os.path.getmtime(STATE_FILE)
        except Exception as e:
            logging.warning(f"Fehler beim Speichern von {STATE_FILE}: {e}")

    @staticmethod
    def _norm_sid(val):
        s = str(val or "").strip().lower()
        if s.isdigit():
            try:
                return hex(int(s)).lower()
            except Exception:
                return s
        return s

    @classmethod
    def _svc_key(cls, ch, sid, scids=0):
        return f"{str(ch or '').strip().upper()}:{cls._norm_sid(sid)}:{int(scids or 0)}"

    def _show_toast(self, line1, line2, duration=1.8):
        self.toast_line1 = line1
        self.toast_line2 = line2
        self.toast_until = time.time() + duration

    # ------------------------------------------
    # DREH-DRÜCK-RAD (SENDERAUSWAHL & MENÜ)
    # ------------------------------------------
    def _encoder_poll_loop(self):
        while not self._stop_event.is_set():
            self._handle_encoder(ENC_A)
            time.sleep(0.001)

    def _handle_encoder(self, channel):
        msb = GPIO.input(ENC_A)
        lsb = GPIO.input(ENC_B)
        encoded = (msb << 1) | lsb
        if encoded == self.last_encoded:
            return

        emit_dirs = []
        with self.enc_lock:
            sum_val = (self.last_encoded << 2) | encoded
            self.last_encoded = encoded

            step = 0
            if sum_val in (0b1101, 0b0100, 0b0010, 0b1011):
                step = 1
            elif sum_val in (0b1110, 0b0111, 0b0001, 0b1000):
                step = -1

            if step != 0:
                now = time.monotonic()
                if (now - self.last_enc_time) > ENC_IDLE_RESET_SEC:
                    self.enc_accum = 0
                self.last_enc_time = now
                self.enc_accum += step

                while self.enc_accum >= ENC_STEPS_PER_DETENT:
                    self.enc_accum -= ENC_STEPS_PER_DETENT
                    emit_dirs.append(1)
                while self.enc_accum <= -ENC_STEPS_PER_DETENT:
                    self.enc_accum += ENC_STEPS_PER_DETENT
                    emit_dirs.append(-1)

        for d in emit_dirs:
            self._on_rotate(d)

    def _on_rotate(self, direction):
        if not self.power_on:
            return

        if self.ui_mode in ("normal", "browse"):
            if self.ui_mode == "normal":
                self.browse_idx = self.current_station_idx
                self.ui_mode = "browse"
            if self.stations:
                self.browse_idx = (self.browse_idx + direction) % len(self.stations)
            self.browse_timeout = time.time() + 4.5

        elif self.ui_mode == "menu":
            self.menu_idx = (self.menu_idx + direction) % len(self.menu_items)

        elif self.ui_mode == "station_info":
            self.info_page_idx = (self.info_page_idx + direction) % 6
            self.dls_scroll_pos = 0

        elif self.ui_mode == "edit_brightness":
            self.brightness = max(5, min(100, self.brightness + direction * 5))
            self.pwm_brightness.set_percent(self.brightness)

        elif self.ui_mode == "wifi_menu":
            self.wifi_menu_idx = (self.wifi_menu_idx + direction) % len(self.wifi_menu_items)

        elif self.ui_mode == "wifi_status":
            self.wifi_status_page_idx = (self.wifi_status_page_idx + direction) % 3
            self.dls_scroll_pos = 0

        elif self.ui_mode == "wifi_list" and self.wifi_networks:
            self.wifi_list_idx = (self.wifi_list_idx + direction) % len(self.wifi_networks)
            self.dls_scroll_pos = 0

        elif self.ui_mode == "wifi_key":
            self.wifi_char_idx = (self.wifi_char_idx + direction) % len(WIFI_KEY_TOKENS)

    def _on_encoder_push(self):
        if not self.power_on:
            self._toggle_power()
            return

        if self.ui_mode == "normal":
            self.show_dab_time = not self.show_dab_time
            if self.show_dab_time:
                threading.Thread(target=self._fetch_status, daemon=True).start()
            return

        if self.ui_mode == "browse":
            # Gewählten Sender aktivieren
            if self.stations and 0 <= self.browse_idx < len(self.stations):
                self.current_station_idx = self.browse_idx
                st = self.stations[self.current_station_idx]
                self._tune_station(st)
            self.ui_mode = "normal"

        elif self.ui_mode == "menu":
            if self.menu_idx == 0:
                self.info_page_idx = 0
                self.dls_scroll_pos = 0
                self.ui_mode = "station_info"
            elif self.menu_idx == 1:
                self.dls_scroll_pos = 0
                self.ui_mode = "scan_confirm"
            elif self.menu_idx == 2:
                self.ui_mode = "edit_brightness"
            elif self.menu_idx == 3:
                self._refresh_wifi_status()
                self.wifi_menu_idx = 0
                self.ui_mode = "wifi_menu"

        elif self.ui_mode == "scan_confirm":
            self.stations = []
            self.has_scanned_stations = False
            self.current_station_idx = 0
            self.browse_idx = 0
            self.live_station_name = ""
            self._ens_stable_sig = None
            self._ens_stable_since = 0.0
            self._save_state()
            self._start_dab_scan()

        elif self.ui_mode == "station_info":
            self.ui_mode = "menu"

        elif self.ui_mode == "edit_brightness":
            self._save_state()
            self._show_toast("GESPEICHERT", "Wert uebernommen", 1.0)
            self.ui_mode = "menu"

        elif self.ui_mode == "wifi_menu":
            if self.wifi_menu_idx == 0:
                self._refresh_wifi_status()
                self.wifi_status_page_idx = 0
                self.dls_scroll_pos = 0
                self.ui_mode = "wifi_status"
            elif self.wifi_menu_idx == 1:
                self._start_wifi_scan_async()
            elif self.wifi_menu_idx == 2:
                self._disconnect_wifi()

        elif self.ui_mode == "wifi_status":
            self.ui_mode = "wifi_menu"

        elif self.ui_mode == "wifi_list" and self.wifi_networks:
            net = self.wifi_networks[self.wifi_list_idx % len(self.wifi_networks)]
            self.wifi_target_net = net
            self.wifi_passphrase = ""
            if not net["secured"]:
                self._connect_wifi_async(net["ssid"], "", False)
            else:
                self.wifi_char_idx = WIFI_KEY_TOKENS.index("a")
                self.ui_mode = "wifi_key"

        elif self.ui_mode == "wifi_key":
            tok = WIFI_KEY_TOKENS[self.wifi_char_idx]
            if tok == "[OK]":
                self._connect_wifi_async(
                    self.wifi_target_net["ssid"],
                    self.wifi_passphrase,
                    self.wifi_target_net.get("secured", True),
                )
            elif tok == "[<-]":
                self.wifi_passphrase = self.wifi_passphrase[:-1]
            elif tok == "[CL]":
                self.wifi_passphrase = ""
            else:
                self.wifi_passphrase += tok

    # ------------------------------------------
    # 3x3 MATRIX SCAN & TASTEN POLLING
    # ------------------------------------------
    def _scan_matrix(self):
        pressed_keys = set()
        for r_pin in MATRIX_ROWS:
            # Nur aktive Zeile kurz als Output LOW treiben (verhindert Kurzschluss!)
            GPIO.setup(r_pin, GPIO.OUT)
            GPIO.output(r_pin, GPIO.LOW)
            time.sleep(0.0001)
            for c_pin in MATRIX_COLS:
                if GPIO.input(c_pin) == GPIO.LOW:
                    key_name = MATRIX_KEYMAP.get((r_pin, c_pin))
                    if key_name:
                        pressed_keys.add(key_name)
            # Zeile sofort wieder hochohmig (Hi-Z Input) schalten
            GPIO.setup(r_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        return pressed_keys

    def _poll_buttons(self):
        now = time.time()
        active_keys = self._scan_matrix()

        for name in ALL_LOGICAL_KEYS:
            pressed = (name in active_keys)
            if pressed:
                if name not in self.btn_press_start:
                    self.btn_press_start[name] = now
                    self.btn_long_handled[name] = False
                else:
                    # Prüfe Langdruck für Favorit I, II, III (>= 1.2s) - außer bei WLAN-Key-Eingabe
                    if (
                        self.ui_mode != "wifi_key"
                        and name in ("FAV1", "FAV2", "FAV3")
                        and not self.btn_long_handled[name]
                    ):
                        if (now - self.btn_press_start[name]) >= LONG_PRESS_SEC:
                            self.btn_long_handled[name] = True
                            slot = name[-1]
                            self._save_favorite(slot)
            else:
                if name in self.btn_press_start:
                    # Taste wurde losgelassen
                    if not self.btn_long_handled.get(name, False):
                        if (now - self.btn_press_start[name]) >= 0.04:
                            self._on_short_press(name)
                    del self.btn_press_start[name]

    def _on_short_press(self, name):
        if name == "POWER":
            self._toggle_power()
            return
        if not self.power_on:
            return

        if name == "ENC_BTN":
            self._on_encoder_push()
        elif name == "REPLY":
            if self.ui_mode in (
                "menu",
                "station_info",
                "edit_brightness",
                "scan_confirm",
                "wifi_menu",
                "wifi_status",
                "wifi_scanning",
                "wifi_list",
                "wifi_key",
                "wifi_connecting",
            ):
                self.ui_mode = "normal"
            else:
                self.ui_mode = "menu"
                self.menu_idx = 0
        elif name == "BACK":
            if self.ui_mode == "edit_brightness":
                self._save_state()
                self.ui_mode = "menu"
            elif self.ui_mode in ("station_info", "wifi_menu", "scan_confirm"):
                self.ui_mode = "menu"
            elif self.ui_mode in ("wifi_status", "wifi_list", "wifi_scanning"):
                self.ui_mode = "wifi_menu"
            elif self.ui_mode == "wifi_key":
                if len(self.wifi_passphrase) > 0:
                    self.wifi_passphrase = self.wifi_passphrase[:-1]
                else:
                    self.ui_mode = "wifi_list"
            else:
                self.show_dab_time = False
                self.ui_mode = "normal"
        elif name in ("FAV1", "FAV2", "FAV3"):
            if self.ui_mode == "wifi_key":
                idx_a = WIFI_KEY_TOKENS.index("a")
                idx_upper = WIFI_KEY_TOKENS.index("A")
                idx_0 = WIFI_KEY_TOKENS.index("0")
                idx_sym = WIFI_KEY_TOKENS.index("!")
                if name == "FAV1":
                    self.wifi_char_idx = idx_upper if idx_a <= self.wifi_char_idx < idx_upper else idx_a
                elif name == "FAV2":
                    self.wifi_char_idx = idx_sym if idx_0 <= self.wifi_char_idx < idx_sym else idx_0
                elif name == "FAV3":
                    self.wifi_char_idx = 0  # [OK] Verbinden
            else:
                self._load_favorite(name[-1])

    def _ensure_independent_bridge_unit(self):
        """Stellt sicher, dass ipdio-bridge.service beim Stoppen von AbracaDABra im Standby weiterlaeuft."""
        unit_path = "/etc/systemd/system/ipdio-bridge.service"
        try:
            if os.path.exists(unit_path):
                with open(unit_path, "r", encoding="utf-8") as f:
                    content = f.read()
                if "Requires=abracadabra.service" in content:
                    updated = content.replace("Requires=abracadabra.service", "Wants=abracadabra.service")
                    with open(unit_path, "w", encoding="utf-8") as f:
                        f.write(updated)
                    self._run_cmd(["systemctl", "daemon-reload"], timeout=5)
                    logging.info("ipdio-bridge.service auf Wants=abracadabra.service umgestellt (Standby-Schutz).")
        except Exception as e:
            logging.warning(f"Konnte ipdio-bridge.service nicht pruefen: {e}")

    def _set_audio_standby(self, standby):
        def worker():
            with self._audio_lock:
                if standby:
                    if self.power_on:
                        return
                    self._ensure_independent_bridge_unit()
                    logging.info(f"Standby: Stoppe {ABRACADABRA_SERVICE} (Audio & SDR aus)")
                    self._run_cmd(["systemctl", "stop", ABRACADABRA_SERVICE], timeout=10)
                    # Falls noch ein alter Stream-Dienst aktiv sein sollte, sauber mitbeenden
                    for legacy in ("dabstream.service", "abracadabra-spdif.service"):
                        _, out, _ = self._run_cmd(["systemctl", "is-active", legacy], timeout=2)
                        if out.strip() in ("active", "activating"):
                            logging.info(f"Standby: Stoppe veralteten Audio-Dienst '{legacy}'")
                            self._run_cmd(["systemctl", "stop", legacy], timeout=8)
                else:
                    if not self.power_on:
                        return
                    logging.info(f"Power ON: Starte {ABRACADABRA_SERVICE} (lokales S/PDIF Audio & WebIF aktiv)")
                    self._run_cmd(["systemctl", "start", ABRACADABRA_SERVICE], timeout=10)
                    # Warte bis AbracaDABra WebIF erreichbar ist, und synchronisiere Sender
                    ready_status = None
                    for _ in range(24):
                        if not self.power_on:
                            return
                        st = self._send_api("status", method="GET", timeout=1.0)
                        if isinstance(st, dict) and st.get("deviceReady"):
                            ready_status = st
                            if st.get("services"):
                                break
                        time.sleep(0.5)

                    if not self.power_on:
                        return

                    saved = self.standby_station
                    if isinstance(saved, dict) and saved.get("sid") and isinstance(ready_status, dict):
                        cur = ready_status.get("current") or {}
                        cur_sid = str(cur.get("sid") or "").lower().strip()
                        cur_label = (cur.get("label") or "").strip().lower()
                        saved_sid = str(saved.get("sid") or "").lower().strip()
                        saved_name = (saved.get("name") or "").strip().lower()
                        if (saved_sid and cur_sid and cur_sid != saved_sid) or (
                            saved_name and cur_label and cur_label != saved_name
                        ):
                            self._tune_station(saved)
                    self._fetch_status()

        threading.Thread(target=worker, daemon=True).start()

    def _toggle_power(self):
        self.power_on = not self.power_on
        if self.power_on:
            self.pwm_brightness.set_percent(self.brightness)
            self.ui_mode = "normal"
            self._set_audio_standby(False)
            self._show_toast("dnt IPdio mini", "S/PDIF 100% AKTIV", 1.8)
            threading.Thread(target=self._fetch_status, daemon=True).start()
        else:
            if self.stations and 0 <= self.current_station_idx < len(self.stations):
                self.standby_station = dict(self.stations[self.current_station_idx])
                self._save_state()
            elif self.live_station_name:
                self.standby_station = {
                    "name": self.live_station_name,
                    "channel": self.station_info.get("channel"),
                    "sid": self.station_info.get("sid"),
                    "scids": 0,
                }
                self._save_state()
            self._set_audio_standby(True)
            self.show_dab_time = False
            self.dab_time_str = ""
            self.lcd.write_line("   STANDBY...   ", 0)
            self.lcd.write_line("                ", 1)
            time.sleep(0.6)
            self.pwm_brightness.set_percent(0)
            self.lcd.clear()

    def _save_favorite(self, slot):
        if not self.power_on or not self.stations:
            return
        st = self.stations[self.current_station_idx]
        self.favorites[slot] = st
        self._save_state()
        roman = {"1": "I", "2": "II", "3": "III"}[slot]
        logging.info(f"Favorit {roman} gespeichert: {st['name']}")
        self._show_toast(f"FAV {roman} GESPEICHERT", st["name"], 2.0)

    def _load_favorite(self, slot):
        fav = self.favorites.get(slot)
        roman = {"1": "I", "2": "II", "3": "III"}[slot]
        if not fav:
            self._show_toast(f"FAVORIT {roman} LEER", "Lang druecken=Sp.", 1.8)
            return
        # Finde Index in Senderliste oder tune direkt
        for idx, st in enumerate(self.stations):
            if st["id"] == fav.get("id") or st["name"] == fav.get("name"):
                self.current_station_idx = idx
                break
        self._tune_station(fav)
        self.ui_mode = "normal"
        self._show_toast(f"FAVORIT {roman}", fav["name"], 1.4)

    # ------------------------------------------
    # ABRACADABRA WEBIF REST API (KejPi CLI)
    # ------------------------------------------
    def _send_api(self, endpoint, payload=None, method="POST", timeout=2.0):
        try:
            data = json.dumps(payload or {}).encode("utf-8") if method == "POST" else None
            headers = {"Content-Type": "application/json"} if method == "POST" else {}
            req = urllib.request.Request(
                f"{ABRACADABRA_API_URL}/{endpoint}",
                data=data,
                headers=headers,
                method=method,
            )
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:
            logging.debug(f"AbracaDABra API ({endpoint}): {e}")
            return None

    def _tune_station(self, station):
        logging.info(f"Wechsle Sender: {station['name']}")
        self.live_station_name = station["name"]
        self._tuning_hold_until = time.time() + 4.5
        self.current_dls = f"{station['name']} - Lade Radiotext..."
        self.dls_scroll_pos = 0

        def worker():
            target_ch = station.get("channel")
            sid = station.get("sid") or station.get("id")
            scids = int(station.get("scids", 0))
            if target_ch and target_ch != self.station_info.get("channel"):
                self.dab_time_str = ""
                self._send_api("tune", {"channel": target_ch}, timeout=3.0)
                time.sleep(1.8)
            for _ in range(4):
                res = self._send_api("service", {"sid": sid, "scids": scids}, timeout=2.0)
                if res and res.get("ok"):
                    break
                time.sleep(0.7)

        threading.Thread(target=worker, daemon=True).start()

    def _start_dab_scan(self):
        logging.info("Starte DAB+ Sendersuchlauf ueber Band III...")
        self.ui_mode = "scanning"
        self.scan_progress = 0

        def worker():
            ch_data = self._send_api("channels", method="GET", timeout=2.0)
            channels = []
            if ch_data and isinstance(ch_data.get("channels"), list):
                for item in ch_data["channels"]:
                    if isinstance(item, dict) and item.get("channel"):
                        channels.append(item["channel"])
                    elif isinstance(item, str):
                        channels.append(item)
            if not channels:
                channels = ["5A", "5B", "5C", "5D", "6A", "6B", "6C", "6D", "7A", "7B", "7C", "7D", "8A", "8B", "8C", "8D", "9A", "9B", "9C", "9D", "10A", "10B", "10C", "10D", "11A", "11B", "11C", "11D", "12A", "12B", "12C", "12D"]

            found = {}
            total = max(1, len(channels))
            for idx, ch in enumerate(channels):
                if self.ui_mode != "scanning":
                    return
                self.scan_progress = int(((idx + 1) / total) * 100)
                self._send_api("tune", {"channel": ch}, timeout=2.0)
                time.sleep(1.3)
                status = self._send_api("status", method="GET", timeout=1.5)
                if status and int(status.get("syncLevel", 0)) > 0:
                    time.sleep(1.2)
                    status = self._send_api("status", method="GET", timeout=1.5) or status
                    for svc in status.get("services", []):
                        sid = svc.get("sid")
                        scids = int(svc.get("scids", 0))
                        label = (svc.get("label") or "").strip()
                        if sid and label:
                            key = f"{ch}:{sid}:{scids}"
                            found[key] = {
                                "id": sid,
                                "sid": sid,
                                "scids": scids,
                                "name": label,
                                "channel": ch,
                            }

            if found:
                self.stations = sorted(found.values(), key=lambda x: x["name"].lower())
                self.has_scanned_stations = True
                self.current_station_idx = 0
                self._save_state()
                self._tune_station(self.stations[0])
                self._show_toast("SUCHLAUF FERTIG", f"{len(self.stations)} Sender", 2.2)
            else:
                self.stations = []
                self.has_scanned_stations = False
                self.current_station_idx = 0
                self._save_state()
                self._show_toast("SUCHLAUF LEER", "0 Sender gef.", 2.2)
            self.scan_progress = 100
            self.ui_mode = "normal"

        threading.Thread(target=worker, daemon=True).start()

    def _fetch_status(self):
        self._sync_state_from_disk()
        try:
            with urllib.request.urlopen(f"{ABRACADABRA_API_URL}/status", timeout=0.8) as resp:
                data = json.loads(resp.read().decode())

                if "snr" in data and data["snr"] is not None:
                    self.snr_db = round(float(data["snr"]), 1)
                    self.station_info["snr"] = self.snr_db
                if data.get("channel"):
                    self.station_info["channel"] = data["channel"]
                if data.get("frequencyKHz"):
                    self.station_info["freq_mhz"] = round(float(data["frequencyKHz"]) / 1000.0, 3)

                dab_t = data.get("dabTime")
                if isinstance(dab_t, str) and dab_t.strip():
                    self.dab_time_str = dab_t.strip()[:16]
                elif isinstance(data.get("dabTimeIso"), str) and data["dabTimeIso"].strip():
                    iso = data["dabTimeIso"].strip()
                    if len(iso) >= 16 and iso[4] == "-" and iso[7] == "-" and iso[10] in ("T", " "):
                        self.dab_time_str = f"{iso[8:10]}.{iso[5:7]}.{iso[0:4]} {iso[11:16]}"
                    else:
                        self.dab_time_str = ""
                else:
                    self.dab_time_str = ""

                # KejPi AbracaDABra nested format
                ens = data.get("ensemble")
                if isinstance(ens, dict):
                    if ens.get("label"):
                        self.station_info["ensemble"] = ens["label"]
                    if ens.get("eid"):
                        self.station_info["eid"] = ens["eid"]
                elif isinstance(ens, str) and ens:
                    self.station_info["ensemble"] = ens

                cur = data.get("current")
                if isinstance(cur, dict):
                    if cur.get("dls"):
                        self.current_dls = cur["dls"]
                    elif cur.get("dlPlusTitle"):
                        artist = cur.get("dlPlusArtist", "")
                        self.current_dls = f"{artist} - {cur['dlPlusTitle']}" if artist else cur["dlPlusTitle"]
                    elif cur.get("playing") and time.time() >= self._tuning_hold_until:
                        self.current_dls = cur.get("label") or "DAB+ Radio"
                    if cur.get("sid"):
                        self.station_info["sid"] = str(cur["sid"]).replace("0x", "").upper()
                    audio = cur.get("audio")
                    if isinstance(audio, dict):
                        sr = audio.get("sampleRate")
                        st_lbl = "Stereo" if audio.get("stereo", True) else "Mono"
                        if sr:
                            self.station_info["sample_rate"] = f"{int(sr)//1000}kHz {st_lbl}"
                        if audio.get("sbr") and audio.get("parametricStereo"):
                            self.station_info["codec"] = "HE-AAC v2"
                        elif audio.get("sbr"):
                            self.station_info["codec"] = "HE-AAC v1"
                        elif audio.get("coding") == 1:
                            self.station_info["codec"] = "DAB+ (AAC-LC)"

                adv = data.get("advanced")
                tii_db_info = data.get("tiiDb") if isinstance(data.get("tiiDb"), dict) else {}
                if isinstance(adv, dict):
                    if adv.get("rfLevel") is not None:
                        self.station_info["rf_level"] = round(float(adv["rfLevel"]), 1)
                    if adv.get("subChId") is not None:
                        self.station_info["scid"] = adv["subChId"]
                    if adv.get("bitRate") is not None:
                        self.station_info["bitrate"] = adv["bitRate"]
                    if adv.get("protection"):
                        self.station_info["protection"] = str(adv["protection"])
                    if adv.get("tii"):
                        self.station_info["tii_code"] = str(adv["tii"])
                        if adv.get("transmitter"):
                            self.station_info["transmitter"] = str(adv["transmitter"])
                        elif tii_db_info.get("loaded"):
                            self.station_info["transmitter"] = "Ort unbekannt"
                        else:
                            self.station_info["transmitter"] = "TII DB laden"
                    else:
                        self.station_info["tii_code"] = "--"
                        self.station_info["transmitter"] = "Kein TII"
                    stats = adv.get("decodingStats")
                    if isinstance(stats, dict) and stats.get("fibCntr", 0) > 0:
                        ok_pct = 100.0 * (1.0 - (stats.get("fibErrorCntr", 0) / float(stats["fibCntr"])))
                        self.station_info["fic_quality"] = round(max(0.0, min(100.0, ok_pct)), 1)

                # Services aus aktuellem Ensemble synchronisieren (wenn nicht im Suchlauf)
                svcs = data.get("services")
                ch = str(data.get("channel") or self.station_info.get("channel", "5C")).strip()
                sync_level = int(data.get("syncLevel") or 0)
                ens_valid = isinstance(ens, dict) and bool(ens.get("valid", False))
                if isinstance(svcs, list) and len(svcs) > 0 and self.ui_mode not in ("scanning", "scan_confirm"):
                    live_services = []
                    for svc in svcs:
                        sid = svc.get("sid") or svc.get("id")
                        label = (svc.get("label") or svc.get("name") or "").strip()
                        if sid and label:
                            live_services.append({
                                "id": sid,
                                "sid": sid,
                                "scids": int(svc.get("scids", 0)),
                                "name": label,
                                "channel": ch,
                            })
                    if live_services:
                        if not self.has_scanned_stations:
                            if not os.path.exists(STATE_FILE):
                                self.stations = live_services
                        else:
                            cur_sid_before = None
                            if 0 <= self.current_station_idx < len(self.stations):
                                cur_st = self.stations[self.current_station_idx]
                                cur_sid_before = self._svc_key(cur_st.get("channel"), cur_st.get("sid") or cur_st.get("id"), cur_st.get("scids", 0))

                            by_key = {}
                            for idx_s, existing in enumerate(self.stations):
                                k = self._svc_key(existing.get("channel"), existing.get("sid") or existing.get("id"), existing.get("scids", 0))
                                by_key[k] = idx_s

                            changed = False
                            live_keys = set()
                            for item in live_services:
                                k = self._svc_key(item["channel"], item["sid"], item["scids"])
                                live_keys.add(k)
                                if k not in by_key:
                                    self.stations.append(item)
                                    changed = True
                                else:
                                    ex = self.stations[by_key[k]]
                                    if (ex.get("name") or "").strip() != item["name"]:
                                        ex["name"] = item["name"]
                                        changed = True

                            if sync_level >= 2 and ens_valid:
                                live_sig = (ch.upper(), tuple(sorted(live_keys)))
                                if live_sig == self._ens_stable_sig:
                                    if (time.time() - self._ens_stable_since) >= 6.0:
                                        kept = []
                                        for st_item in self.stations:
                                            st_ch = str(st_item.get("channel") or "").strip().upper()
                                            st_k = self._svc_key(st_item.get("channel"), st_item.get("sid") or st_item.get("id"), st_item.get("scids", 0))
                                            if st_ch == ch.upper() and st_k not in live_keys:
                                                changed = True
                                                continue
                                            kept.append(st_item)
                                        if changed:
                                            self.stations = kept
                                else:
                                    self._ens_stable_sig = live_sig
                                    self._ens_stable_since = time.time()
                            else:
                                self._ens_stable_sig = None

                            if changed:
                                self.stations.sort(key=lambda x: str(x.get("name") or "").lower())
                                if cur_sid_before:
                                    for idx_s, st_item in enumerate(self.stations):
                                        if self._svc_key(st_item.get("channel"), st_item.get("sid") or st_item.get("id"), st_item.get("scids", 0)) == cur_sid_before:
                                            self.current_station_idx = idx_s
                                            break
                                if self.current_station_idx >= len(self.stations):
                                    self.current_station_idx = 0
                                self._save_state()
                elif sync_level < 2:
                    self._ens_stable_sig = None

                # Aktuell laufenden Sendernamen (z.B. nach Reboot mit Dlf Nova) auf Zeile 1 synchronisieren
                if isinstance(cur, dict) and time.time() >= self._tuning_hold_until:
                    cur_label = (cur.get("label") or cur.get("name") or "").strip()
                    cur_sid = str(cur.get("sid") or "").lower().strip()
                    cur_scids = int(cur.get("scids", 0))
                    if cur_label:
                        self.live_station_name = cur_label
                    if cur_sid or cur_label:
                        matched_idx = None
                        for i, st in enumerate(self.stations):
                            st_sid = str(st.get("sid") or st.get("id") or "").lower().strip()
                            st_name = (st.get("name") or "").strip()
                            if (cur_sid and st_sid == cur_sid and int(st.get("scids", 0)) == cur_scids) or (
                                cur_label and st_name.lower() == cur_label.lower()
                            ):
                                matched_idx = i
                                break
                        if matched_idx is not None:
                            self.current_station_idx = matched_idx
                        elif cur_label and (self.has_scanned_stations or not os.path.exists(STATE_FILE)):
                            self.stations.append({
                                "id": cur_sid or cur_label,
                                "sid": cur.get("sid") or cur_label,
                                "scids": cur_scids,
                                "name": cur_label,
                                "channel": ch,
                            })
                            if self.has_scanned_stations:
                                self.stations.sort(key=lambda x: str(x.get("name") or "").lower())
                                target_k = self._svc_key(ch, cur.get("sid") or cur_label, cur_scids)
                                for idx_s, st_item in enumerate(self.stations):
                                    if self._svc_key(st_item.get("channel"), st_item.get("sid") or st_item.get("id"), st_item.get("scids", 0)) == target_k:
                                        self.current_station_idx = idx_s
                                        break
                                self._save_state()
                            else:
                                self.current_station_idx = len(self.stations) - 1

                # Optionale flache Felder als Fallback
                if "dls" in data and data["dls"]:
                    self.current_dls = data["dls"]
                field_map = {
                    "rfLevel": "rf_level",
                    "rfLevelDbm": "rf_level",
                    "ficQuality": "fic_quality",
                    "fic": "fic_quality",
                    "tii": "tii_code",
                    "transmitter": "transmitter",
                    "location": "transmitter",
                    "eid": "eid",
                    "frequencyMhz": "freq_mhz",
                    "bitrate": "bitrate",
                    "bitrateKbps": "bitrate",
                    "codec": "codec",
                    "serviceId": "sid",
                    "subchannelId": "scid",
                    "protectionLevel": "protection",
                    "genre": "genre",
                }
                for api_key, local_key in field_map.items():
                    if api_key in data and data[api_key] not in (None, ""):
                        self.station_info[local_key] = data[api_key]
        except Exception:
            pass

    def _get_info_pages(self):
        info = self.station_info
        return [
            (
                "Info 1/6 Signal ",
                f"SNR {info['snr']}dB · Pegel {info['rf_level']}dBm · FIC {info['fic_quality']}%",
            ),
            (
                "Info 2/6 TII-Ort",
                f"{info['transmitter']} · TII {info['tii_code']}",
            ),
            (
                "Info 3/6 Ensembl",
                f"{info['channel']} {info['ensemble']} · EID {info['eid']}",
            ),
            (
                "Info 4/6 Audio  ",
                f"{info['bitrate']} kbps {info['codec']} · {info['sample_rate']} S/PDIF",
            ),
            (
                "Info 5/6 Service",
                f"SID {info['sid']} · SCID {info['scid']} · {info['protection']} · {info['genre']}",
            ),
            (
                "Info 6/6 SDR/RF ",
                f"{info['freq_mhz']} MHz Band III · S/PDIF 100% Bit-Perfekt",
            ),
        ]

    # ------------------------------------------
    # WLAN / WIFI STEUERUNG (NMCLI / RPi OS)
    # ------------------------------------------
    def _run_cmd(self, cmd, timeout=6):
        try:
            res = subprocess.run(
                cmd, capture_output=True, text=True, timeout=timeout, check=False
            )
            return res.returncode, res.stdout.strip(), res.stderr.strip()
        except Exception as e:
            logging.debug(f"Cmd {cmd}: {e}")
            return -1, "", str(e)

    def _refresh_wifi_status(self):
        connected = False
        ssid = "Nicht verbunden"
        signal = 0
        ip_addr = "--"
        gateway = "--"

        rc, out, _ = self._run_cmd(["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL", "dev", "wifi"])
        if rc == 0 and out:
            for line in out.splitlines():
                parts = line.split(":")
                if len(parts) >= 3 and parts[0] == "yes":
                    connected = True
                    ssid = parts[1] or "Verbunden"
                    try:
                        signal = int(parts[2])
                    except ValueError:
                        signal = 0
                    break
        else:
            rc_ssid, out_ssid, _ = self._run_cmd(["iwgetid", "-r"])
            if rc_ssid == 0 and out_ssid:
                connected = True
                ssid = out_ssid

        rc_ip, out_ip, _ = self._run_cmd(["ip", "-4", "-o", "addr", "show", WIFI_IFACE])
        if rc_ip == 0 and out_ip:
            for token in out_ip.split():
                if "/" in token and "." in token:
                    ip_addr = token.split("/")[0]
                    connected = True
                    break

        rc_gw, out_gw, _ = self._run_cmd(["ip", "route", "show", "default", "dev", WIFI_IFACE])
        if rc_gw == 0 and "via " in out_gw:
            parts = out_gw.split()
            if "via" in parts:
                idx = parts.index("via")
                if idx + 1 < len(parts):
                    gateway = parts[idx + 1]

        self.wifi_status = {
            "connected": connected,
            "ssid": ssid,
            "signal": signal,
            "ip": ip_addr,
            "gateway": gateway,
        }

    def _get_wifi_status_pages(self):
        st = self.wifi_status
        if not st["connected"]:
            return [
                ("WLAN 1/3 Verb.  ", "Nicht verbunden - Netzwerk suchen"),
                ("WLAN 2/3 IP/Web ", "Keine IP-Adresse - Offline"),
                ("WLAN 3/3 Ger.   ", f"{WIFI_IFACE} BEREIT - DHCP aktiv"),
            ]
        return [
            ("WLAN 1/3 Verb.  ", f"{st['ssid']} - Signal {st['signal']}%"),
            ("WLAN 2/3 IP/Web ", f"IP {st['ip']} - WebIF http://{st['ip']}:8080"),
            ("WLAN 3/3 Ger.   ", f"{WIFI_IFACE} AKTIV - GW {st['gateway']}"),
        ]

    def _start_wifi_scan_async(self):
        self.ui_mode = "wifi_scanning"

        def worker():
            nets = {}
            rc, out, _ = self._run_cmd(
                ["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY", "dev", "wifi", "list", "--rescan", "yes"],
                timeout=12,
            )
            if rc == 0 and out:
                for line in out.splitlines():
                    parts = line.split(":")
                    if len(parts) >= 3:
                        ssid = parts[0].strip()
                        if not ssid:
                            continue
                        try:
                            sig = int(parts[1])
                        except ValueError:
                            sig = 0
                        sec = parts[2].strip() not in ("", "--", "none")
                        if ssid not in nets or sig > nets[ssid]["signal"]:
                            nets[ssid] = {"ssid": ssid, "signal": sig, "secured": sec}
            sorted_nets = sorted(nets.values(), key=lambda x: x["signal"], reverse=True)
            self.wifi_networks = sorted_nets
            self.wifi_list_idx = 0
            self.dls_scroll_pos = 0
            if self.ui_mode == "wifi_scanning":
                if sorted_nets:
                    self.ui_mode = "wifi_list"
                else:
                    self._show_toast("KEIN WLAN", "Nichts gefunden", 1.8)
                    self.ui_mode = "wifi_menu"

        threading.Thread(target=worker, daemon=True).start()

    def _connect_wifi_async(self, ssid, passphrase, secured):
        self.ui_mode = "wifi_connecting"

        def worker():
            cmd = ["nmcli", "dev", "wifi", "connect", ssid]
            if secured:
                cmd += ["password", passphrase]
            rc, out, err = self._run_cmd(cmd, timeout=22)
            if rc != 0:
                # Falls altes Profil mit falschem Key existiert: Profil bereinigen und neu versuchen
                self._run_cmd(["nmcli", "connection", "delete", ssid], timeout=5)
                rc, out, err = self._run_cmd(cmd, timeout=22)

            if rc == 0:
                self._refresh_wifi_status()
                self.wifi_status_page_idx = 0
                self.dls_scroll_pos = 0
                self.ui_mode = "wifi_status"
                self._show_toast("WLAN VERBUNDEN", self.wifi_status["ip"], 2.2)
                logging.info(f"WLAN verbunden mit {ssid} ({self.wifi_status['ip']})")
            else:
                logging.warning(f"WLAN Verbindung fehlgeschlagen ({ssid}): {err or out}")
                self._show_toast("WLAN FEHLER", "Key pruefen!", 2.0)
                self.ui_mode = "wifi_key" if secured else "wifi_list"

        threading.Thread(target=worker, daemon=True).start()

    def _disconnect_wifi(self):
        self._run_cmd(["nmcli", "dev", "disconnect", WIFI_IFACE], timeout=6)
        self._refresh_wifi_status()
        self._show_toast("WLAN GETRENNT", "Verbindung aus", 1.5)

    def _format_wifi_key_wheel(self, selected_idx):
        def tok_disp(tok, is_sel):
            if tok == "[OK]":
                return "[OK]" if is_sel else "OK"
            if tok == "[<-]":
                return "[<-]" if is_sel else "<-"
            if tok == "[CL]":
                return "[CL]" if is_sel else "CL"
            ch = "_" if tok == " " else tok
            return f"[{ch}]" if is_sel else ch

        total = len(WIFI_KEY_TOKENS)
        center = tok_disp(WIFI_KEY_TOKENS[selected_idx], True)
        left_str = ""
        for step in range(1, 5):
            lbl = tok_disp(WIFI_KEY_TOKENS[(selected_idx - step) % total], False)
            cand = f"{lbl} {left_str}"
            if len(cand) <= 6:
                left_str = cand
            else:
                break
        right_str = ""
        for step in range(1, 5):
            lbl = tok_disp(WIFI_KEY_TOKENS[(selected_idx + step) % total], False)
            cand = f"{right_str} {lbl}"
            if len(cand) <= 6:
                right_str = cand
            else:
                break
        return f"{left_str.rjust(6)}{center}{right_str}".ljust(16)[:16]

    # ------------------------------------------
    # LCD DARSTELLUNG (2x16)
    # ------------------------------------------
    def _render_lcd(self):
        if not self.power_on:
            return

        now = time.time()
        if now < self.toast_until:
            self.lcd.write_line(self.toast_line1, 0)
            self.lcd.write_line(self.toast_line2, 1)
            return

        if self.ui_mode == "browse":
            if now > self.browse_timeout:
                self.ui_mode = "normal"
            else:
                st = self.stations[self.browse_idx] if self.stations else {"name": "Kein Sender"}
                line1 = f"Sender [{self.browse_idx+1}/{len(self.stations)}]"
                line2 = f"> {st['name']}"
                self.lcd.write_line(line1, 0)
                self.lcd.write_line(line2, 1)
                return

        if self.ui_mode == "menu":
            self.lcd.write_line("MENU (Reply)    ", 0)
            self.lcd.write_line(f"> {self.menu_items[self.menu_idx]}", 1)
            return

        if self.ui_mode == "station_info":
            pages = self._get_info_pages()
            header, detail = pages[self.info_page_idx % len(pages)]
            self.lcd.write_line(header, 0)
            if len(detail) <= 16:
                self.lcd.write_line(detail.ljust(16), 1)
            else:
                padded = detail + "   +++   "
                idx = self.dls_scroll_pos % len(padded)
                self.lcd.write_line((padded[idx:] + padded[:idx])[:16], 1)
            return

        if self.ui_mode == "edit_brightness":
            bars = int((self.brightness / 100.0) * 10)
            bar_str = "=" * bars + "-" * (10 - bars)
            self.lcd.write_line("Helligkeit LCD  ", 0)
            self.lcd.write_line(f"{self.brightness:3d}% [{bar_str}]", 1)
            return

        if self.ui_mode == "scan_confirm":
            self.lcd.write_line("Sendersuchlauf  ", 0)
            detail = "Aktuelle Liste wird gelöscht! Weiter?"
            padded = detail + "   +++   "
            idx = self.dls_scroll_pos % len(padded)
            self.lcd.write_line((padded[idx:] + padded[:idx])[:16], 1)
            return

        if self.ui_mode == "scanning":
            bars = int((self.scan_progress / 100.0) * 10)
            bar_str = "=" * bars + "-" * (10 - bars)
            self.lcd.write_line("DAB+ Suchlauf...", 0)
            self.lcd.write_line(f"{self.scan_progress:3d}% [{bar_str}]", 1)
            return

        if self.ui_mode == "wifi_menu":
            badge = "ON " if self.wifi_status["connected"] else "OFF"
            self.lcd.write_line(f"WLAN MENU  [{badge}]", 0)
            self.lcd.write_line(f"> {self.wifi_menu_items[self.wifi_menu_idx]}", 1)
            return

        if self.ui_mode == "wifi_status":
            pages = self._get_wifi_status_pages()
            header, detail = pages[self.wifi_status_page_idx % len(pages)]
            self.lcd.write_line(header, 0)
            if len(detail) <= 16:
                self.lcd.write_line(detail.ljust(16), 1)
            else:
                padded = detail + "   +++   "
                idx = self.dls_scroll_pos % len(padded)
                self.lcd.write_line((padded[idx:] + padded[:idx])[:16], 1)
            return

        if self.ui_mode == "wifi_scanning":
            self.lcd.write_line("WLAN SUCHLAUF   ", 0)
            self.lcd.write_line("Suche Netze...  ", 1)
            return

        if self.ui_mode == "wifi_list" and self.wifi_networks:
            net = self.wifi_networks[self.wifi_list_idx % len(self.wifi_networks)]
            lock_mark = "*" if net["secured"] else "O"
            header = f"WLAN {self.wifi_list_idx+1}/{len(self.wifi_networks)} {net['signal']}% {lock_mark}"
            self.lcd.write_line(header, 0)
            detail = f"> {net['ssid']}"
            if len(detail) <= 16:
                self.lcd.write_line(detail.ljust(16), 1)
            else:
                padded = detail + "   +++   "
                idx = self.dls_scroll_pos % len(padded)
                self.lcd.write_line((padded[idx:] + padded[:idx])[:16], 1)
            return

        if self.ui_mode == "wifi_key":
            pw_len = len(self.wifi_passphrase)
            tail = "(Dreh+OK)" if pw_len == 0 else f"{self.wifi_passphrase[-8:]}_"
            self.lcd.write_line(f"PW({pw_len:02d}): {tail}", 0)
            self.lcd.write_line(self._format_wifi_key_wheel(self.wifi_char_idx), 1)
            return

        if self.ui_mode == "wifi_connecting":
            self.lcd.write_line("WLAN Verbinde...", 0)
            self.lcd.write_line(f"> {self.wifi_target_net.get('ssid', '')}", 1)
            return

        # Normaler Wiedergabe-Modus
        if self.live_station_name:
            st_name = self.live_station_name
        elif self.stations and 0 <= self.current_station_idx < len(self.stations):
            st_name = self.stations[self.current_station_idx]["name"]
        else:
            st_name = "DAB+ Radio"
        sig_lvl = 5 if self.snr_db >= 18 else (4 if self.snr_db >= 14 else (3 if self.snr_db >= 11 else (2 if self.snr_db >= 8 else 1)))
        self.lcd.set_signal_level(sig_lvl)
        line1 = f"{st_name[:14].ljust(14)}|{chr(1)}"
        self.lcd.write_line(line1, 0)

        if self.show_dab_time:
            line2 = (self.dab_time_str if self.dab_time_str else "DAB Zeit: --:-- ").ljust(16)[:16]
        else:
            padded_dls = self.current_dls + "  +++  "
            if len(padded_dls) <= 16:
                line2 = padded_dls.ljust(16)
            else:
                idx = self.dls_scroll_pos % len(padded_dls)
                line2 = (padded_dls[idx:] + padded_dls[:idx])[:16]
        self.lcd.write_line(line2, 1)

    def run(self):
        poll_timer = 0.0  # Sofort beim Start Sendername & Status von AbracaDABra abrufen
        self._ensure_independent_bridge_unit()
        try:
            while True:
                now = time.time()
                self._poll_buttons()

                if now - poll_timer > 1.5:
                    poll_timer = now
                    if self.power_on and self.ui_mode != "scanning":
                        self._fetch_status()

                if self.power_on and (now - self.last_scroll_time > 0.35):
                    self.last_scroll_time = now
                    self.dls_scroll_pos += 1

                self._render_lcd()
                time.sleep(0.03)
        except KeyboardInterrupt:
            logging.info("Beende dnt IPdio mini Bridge...")
        finally:
            self._stop_event.set()
            self.lcd.clear()
            self.pwm_brightness.stop()
            GPIO.cleanup()


if __name__ == "__main__":
    controller = IpdioMiniController()
    controller.run()
