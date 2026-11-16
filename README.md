# dnt IPdio mini Frontpanel & S/PDIF Bridge für AbracaDABra auf Raspberry Pi 4

Python 3 basiert, benötigt  https://github.com/MHdotM/AbracaDABra_IPdio

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

<p align="center" width="100%">
  <img width="4032" height="3024" alt="IMG_2241" src="https://github.com/user-attachments/assets/c3c35d67-6145-49d3-85fe-bb567cc5a4fa" />
</p>
