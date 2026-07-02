Press keypad B: move exactly one configured chamber, currently 10 full steps.
Press keypad C: move one full motor revolution, 200 full steps.
MQTT command {"action":"advance_chamber"}: same as B.
MQTT command {"action":"motor_rev_test"}: same as C.


the offline keypad dose-registration flow and verified it builds.
How it works now:
Press D to open the local schedule wizard.
Enter medicine slot 01 to 20, then press #.
Enter time as HHMM, for example 0830, then press #.
Enter number of daily doses/chambers to schedule, 01 to 20, then press #.
Confirm with A or #.
Cancel anytime with *.
Go back with B during input screens.

