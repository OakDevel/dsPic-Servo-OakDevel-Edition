EESchema Schematic File Version 2
LIBS:dsPic Servo SALVADO Edition-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:dsPic Servo SALVADO Edition-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title "dsPIC Servo"
Date "11 oct 2013"
Rev "2"
Comp "P.O.S"
Comment1 "Salvado Edition"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L L6203 IC4
U 1 1 52503A46
P 5750 3100
F 0 "IC4" H 5250 3450 50  0000 L BNN
F 1 "L6203" H 5250 2600 50  0000 L BNN
F 2 "PKG-ZIP:ZIP-11" H 5750 3250 50  0001 C CNN
F 3 "" H 5750 3100 60  0000 C CNN
	1    5750 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3100 6350 3100
Wire Wire Line
	6350 3200 6700 3200
Wire Wire Line
	4800 3100 5150 3100
Wire Wire Line
	4800 3200 5150 3200
$Comp
L GND-RESCUE-dsPic_Servo_SALVADO_Edition #PWR051
U 1 1 52505CED
P 5950 4300
AR Path="/52505CED" Ref="#PWR051"  Part="1" 
AR Path="/525133B0/52505CED" Ref="#PWR051"  Part="1" 
F 0 "#PWR051" H 5950 4300 30  0001 C CNN
F 1 "GND" H 5950 4230 30  0001 C CNN
F 2 "" H 5950 4300 60  0000 C CNN
F 3 "" H 5950 4300 60  0000 C CNN
	1    5950 4300
	1    0    0    -1  
$EndComp
$Comp
L RES RSense1
U 1 1 525068F1
P 4950 3600
F 0 "RSense1" H 4950 3650 30  0000 C CNN
F 1 ".47R" H 4950 3600 30  0000 C CNN
F 2 "Resistors-SMD:M1206" H 4950 3750 60  0000 C CNN
F 3 "" H 4950 3600 60  0000 C CNN
	1    4950 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4950 3450 4950 3300
Wire Wire Line
	4700 3300 5150 3300
Wire Wire Line
	4950 3750 4950 3900
Wire Wire Line
	4950 3900 7550 3900
Connection ~ 4950 3300
$Comp
L +24V #PWR050
U 1 1 5250BFB8
P 5950 2600
F 0 "#PWR050" H 5950 2550 20  0001 C CNN
F 1 "+24V" H 5950 2700 30  0000 C CNN
F 2 "" H 5950 2600 60  0000 C CNN
F 3 "" H 5950 2600 60  0000 C CNN
	1    5950 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 2600 5950 2700
$Comp
L +5V #PWR048
U 1 1 5250BFD5
P 4800 2600
F 0 "#PWR048" H 4800 2690 20  0001 C CNN
F 1 "+5V" H 4800 2690 30  0000 C CNN
F 2 "" H 4800 2600 60  0000 C CNN
F 3 "" H 4800 2600 60  0000 C CNN
	1    4800 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 2600 4800 3000
Wire Wire Line
	4800 3000 5150 3000
$Comp
L CAP C14
U 1 1 5250BFFD
P 6600 3000
F 0 "C14" H 6675 3050 30  0000 C CNN
F 1 "15nF" H 6700 2950 30  0000 C CNN
F 2 "Cap-SMD:CAP-SMD-0805" H 6600 3000 60  0001 C CNN
F 3 "" H 6600 3000 60  0000 C CNN
	1    6600 3000
	0    -1   -1   0   
$EndComp
$Comp
L CAP C15
U 1 1 5250C00C
P 6600 3300
F 0 "C15" H 6675 3350 30  0000 C CNN
F 1 "15nF" H 6700 3250 30  0000 C CNN
F 2 "Cap-SMD:CAP-SMD-0805" H 6600 3300 60  0001 C CNN
F 3 "" H 6600 3300 60  0000 C CNN
	1    6600 3300
	0    -1   -1   0   
$EndComp
$Comp
L CAP C13
U 1 1 5250C01B
P 4950 2800
F 0 "C13" H 5025 2850 30  0000 C CNN
F 1 "22nF" H 5050 2750 30  0000 C CNN
F 2 "Cap-SMD:CAP-SMD-0805" H 4950 2800 60  0001 C CNN
F 3 "" H 4950 2800 60  0000 C CNN
	1    4950 2800
	1    0    0    -1  
$EndComp
$Comp
L CAP C17
U 1 1 5250C02A
P 7550 3200
F 0 "C17" H 7625 3250 30  0000 C CNN
F 1 "220nF" H 7650 3150 30  0000 C CNN
F 2 "Cap-SMD:CAP-SMD-0805" H 7550 3200 60  0001 C CNN
F 3 "" H 7550 3200 60  0000 C CNN
	1    7550 3200
	1    0    0    -1  
$EndComp
$Comp
L CAP C16
U 1 1 5250C039
P 7300 3400
F 0 "C16" H 7375 3450 30  0000 C CNN
F 1 "15nF" H 7400 3350 30  0000 C CNN
F 2 "Cap-SMD:CAP-SMD-0805" H 7300 3400 60  0001 C CNN
F 3 "" H 7300 3400 60  0000 C CNN
	1    7300 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 3000 6500 3000
Wire Wire Line
	6350 3300 6500 3300
Wire Wire Line
	6700 2850 6700 3100
Wire Wire Line
	6700 3200 6700 3500
Wire Wire Line
	4950 2900 5150 2900
$Comp
L GND-RESCUE-dsPic_Servo_SALVADO_Edition #PWR049
U 1 1 5250C0D5
P 4950 2550
AR Path="/5250C0D5" Ref="#PWR049"  Part="1" 
AR Path="/525133B0/5250C0D5" Ref="#PWR049"  Part="1" 
F 0 "#PWR049" H 4950 2550 30  0001 C CNN
F 1 "GND" H 4950 2480 30  0001 C CNN
F 2 "" H 4950 2550 60  0000 C CNN
F 3 "" H 4950 2550 60  0000 C CNN
	1    4950 2550
	-1   0    0    1   
$EndComp
Connection ~ 5950 2650
$Comp
L RES R16
U 1 1 5250C17B
P 7300 3000
F 0 "R16" H 7300 3050 30  0000 C CNN
F 1 "7R5" H 7300 3000 30  0000 C CNN
F 2 "Resistors-SMD:M0805" H 7300 3000 60  0000 C CNN
F 3 "" H 7300 3000 60  0000 C CNN
	1    7300 3000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7550 3900 7550 3300
$Comp
L CONN_2 Con7
U 1 1 5250C285
P 7100 3150
F 0 "Con7" V 7050 3150 40  0000 C CNN
F 1 "MOTOR" V 7150 3150 40  0000 C CNN
F 2 "Con-PTR500:CON-PTR500-AK300_2" V 7700 3000 60  0000 C CNN
F 3 "" H 7100 3150 60  0000 C CNN
	1    7100 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3100 7550 2650
Wire Wire Line
	7550 2650 5950 2650
Wire Wire Line
	7300 3150 7300 3300
Wire Wire Line
	6750 3050 6700 3050
Connection ~ 6700 3050
Wire Wire Line
	6750 3250 6700 3250
Connection ~ 6700 3250
Wire Wire Line
	6700 2850 7300 2850
Connection ~ 6700 3000
Wire Wire Line
	6700 3500 7300 3500
Connection ~ 6700 3300
Wire Wire Line
	5950 3600 5950 4300
Wire Wire Line
	4950 2550 4950 2700
Text GLabel 4800 3100 0    60   Input ~ 0
PWM_H_A
Text GLabel 4800 3200 0    60   Input ~ 0
PWM_H_B
Text GLabel 4700 3300 0    60   Output ~ 0
LSENSE
$EndSCHEMATC
