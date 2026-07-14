# Device controls

- `D`: open protected schedule settings.
- `B` while idle: advance exactly one chamber (10 full steps).
- `C` while idle: run one full motor revolution (200 full steps).
- `A` while idle: request the next pending dose immediately.
- `#` while idle: run the buzzer and RGB LED self-test.

# Keypad schedule setup

The default access PIN is `1234`. The PIN is stored in NVS and remains changed
after reboot. PIN entry is masked on the LCD.

1. Press `D`, enter the four-digit PIN, then press `#`.
2. Press `1` for a new dose plan.
3. Enter how many times medicine is taken per day (`1` to `4`), then press `#`.
4. Enter each requested time as `HHMM` in 24-hour format. For example, `0830`
   means 08:30. Press `#` after every time.
5. Choose the treatment quantity method:
   - Press `1` when the total number of prepared dose occasions is known.
   - Press `2` when only total pills and pills taken per dose are known.
6. Review the generated plan, press `#` to see the calculated final dose date
   and time, then press `A` to save.

The device generates names such as `Local Dose 01`; the limited keypad is not
used for medicine-name typing. Times are sorted automatically, duplicate times
are rejected, and no chamber number is requested. A plan may contain at most 20
dispensing occasions because the physical carousel holds 20 prepared doses.

When total pills are used, dispensing occasions are calculated as:

`ceil(total pills / pills per dose)`

If the values are not evenly divisible, the LCD explicitly shows the smaller
quantity expected in the final dose.

# Protected settings

After PIN login:

- Press `1` to create a new dose plan. It replaces an existing plan only after
  the replacement warning is accepted.
- Press `2` to choose and confirm a new four-digit PIN.
- Press `3` to set local date and time manually for offline operation.
- Press `*` to exit without saving.

During entry, `B` goes back, `#` accepts the current value, and `*` cancels.
Setup returns home after 45 seconds without input. A due dose always takes LCD
priority and closes the setup wizard so hand detection is not hidden.

At dose time, the hand must remain within 7 cm continuously for one second.
Moving away during that confirmation resets the timer, preventing medicine from
being released before the hand is properly positioned below the outlet.

The manual clock supports scheduling without internet while the dispenser is
powered. After a complete power loss, set the clock again if NTP is unavailable;
the current hardware does not have an integrated battery-backed RTC in use.
