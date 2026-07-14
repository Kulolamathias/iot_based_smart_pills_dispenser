    // 1. Discovery (device sends this once)
smartpill/discovery/announce  ->  {"id":"e8f60a8b259c","type":"pill_dispenser"}

// 2. Set schedule (backend sends)
smartpill/dispenser/e8f60a8b259c/schedule  ->  [{"name":"Aspirin","times":["08:00","20:00"],"duration_days":3,"total_pills":6}]

Optional quantity fields supported by the device:

- `total_doses`: maximum number of dispensing occasions to generate.
- `pills_per_dose`: number of pills consumed at each dispensing occasion.

For example, 12 pills taken 2 at a time creates 6 dispensing occasions:

[{"name":"Aspirin","times":["08:00","20:00"],"duration_days":4,"total_pills":12,"pills_per_dose":2,"total_doses":6}]

Existing mobile payloads remain compatible. If `pills_per_dose` is omitted, it
defaults to one pill per dispensing occasion.

// 3. Command (backend sends)
smartpill/dispenser/e8f60a8b259c/command  ->  {"action":"dispense_now"}

// 4. Status (device publishes)
smartpill/dispenser/e8f60a8b259c/status  ->  {"state":"online"}

// 5. Log (device publishes)
smartpill/dispenser/e8f60a8b259c/log  ->  {"event":"dispensed","medicine":"Aspirin","time":"2026-05-22T08:00:00Z","remaining_pills":5}




INTERPRETATION OF DOSE SCHEDULE
[{"name":"Aspirin","times":["08:00","20:00"],"duration_days":3,"total_pills":6}]
means: starting from today, dispense Aspirin twice per day, at 08:00 and 20:00, for 3 calendar days, total 6 doses.
If the schedule is registered before 08:00 on day 1:
Day 1: 08:00, 20:00
Day 2: 08:00, 20:00
Day 3: 08:00, 20:00
So after each dose, the next scheduled dose is normally 12 hours later.
Important detail: the system does not rotate immediately just because time arrived. At 08:00 or 20:00 it enters “dose due” mode, turns red, buzzes, and waits for the hand sensor. The actual dispensing happens only when ultrasonic detects a hand at 7 cm or below. If the user takes the 08:00 dose at 08:10, the next schedule is still 20:00, not 12 hours after 08:10.
Also, if the command is registered after a time has already passed, that past time is skipped. For example, if registered today at 10:00, today’s 08:00 dose is not generated; the first dose becomes today at 20:00.
