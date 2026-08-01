# MicroTensile Stage for Gu Group
Same code, same functionality.

Recent update: 07/15/2026: "C++ --> Python"
Connected MS1 and MS2 to power --> 1/16
VM power supply missing --> linear actuator coils cannot move --> created 18V power source
adjusted potentiometer on tms2208 such that multimeter reads 0.35V
tie en to gpio22, code changes 
remove battery gnd lead from gnd rail. now goes directly into vm-gnd pin (break shared gnd)
1N5245B (15V Zener) and  BZX79C4V7 (4.7V Zener) in one unit. band side in VM, unbanded side in Gnd next to VM
330 ohm resistors installed in-line STEP and DIR (i.e. GPIO pins --> 330 resistor --> 1n4148 #1 to vdd --> 1n4148 #2 to gnd --> STEP/DIR) (performed continuity checks and resistance checks (gnd rail to vdd rail = high resistance, respective resistors to step/dir pins = low resistance))
key issue: short seems to exist on rail --> 2208 shorted