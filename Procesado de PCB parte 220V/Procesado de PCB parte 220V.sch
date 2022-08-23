EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Screw_Terminal_01x03 J1
U 1 1 5EDBBEA1
P 850 900
F 0 "J1" H 768 575 50  0000 C CNN
F 1 "Entrada" H 768 666 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-3_P5.00mm" H 850 900 50  0001 C CNN
F 3 "~" H 850 900 50  0001 C CNN
	1    850  900 
	-1   0    0    1   
$EndComp
$Comp
L Device:Transformer_1P_1S T1
U 1 1 5EDBE5B4
P 2550 950
F 0 "T1" H 2550 1331 50  0000 C CNN
F 1 "Transformer_1P_1S" H 2550 1240 50  0000 C CNN
F 2 "Transformer_THT:Transformer_Toroid_Horizontal_D14.0mm_Amidon-T50" H 2550 950 50  0001 C CNN
F 3 "~" H 2550 950 50  0001 C CNN
	1    2550 950 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 5EDBF9B4
P 1550 2300
F 0 "J2" V 1422 2380 50  0000 L CNN
F 1 "Resistencia calor" V 1513 2380 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 1550 2300 50  0001 C CNN
F 3 "~" H 1550 2300 50  0001 C CNN
	1    1550 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 2000 1450 2100
Wire Wire Line
	1550 2000 1550 2100
Wire Wire Line
	1850 900  1850 750 
Wire Wire Line
	1850 750  2150 750 
Wire Wire Line
	2150 1150 1650 1150
$Comp
L Regulator_Linear:L7812 U1
U 1 1 5EDC4AFA
P 5700 1000
F 0 "U1" H 5700 1242 50  0000 C CNN
F 1 "L7812" H 5700 1151 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 5725 850 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 5700 950 50  0001 C CNN
	1    5700 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Bridge_+-AA D2
U 1 1 5EDC7A82
P 3650 1000
F 0 "D2" H 4000 1250 50  0000 L CNN
F 1 "D_Bridge_+-AA" H 3950 1150 50  0000 L CNN
F 2 "Diode_THT:Diode_Bridge_19.0x3.5x10.0mm_P5.0mm" H 3650 1000 50  0001 C CNN
F 3 "~" H 3650 1000 50  0001 C CNN
	1    3650 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 750  3450 750 
Wire Wire Line
	3450 750  3450 600 
Wire Wire Line
	3450 600  3650 600 
Wire Wire Line
	3650 600  3650 700 
Wire Wire Line
	3650 1300 3650 1350
Wire Wire Line
	3650 1350 3100 1350
Wire Wire Line
	3100 1350 3100 1150
Wire Wire Line
	3100 1150 2950 1150
$Comp
L Device:CP C1
U 1 1 5EDCACC8
P 4150 1250
F 0 "C1" H 4268 1296 50  0000 L CNN
F 1 "4700uF 50V" H 3650 1100 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P2.50mm" H 4188 1100 50  0001 C CNN
F 3 "~" H 4150 1250 50  0001 C CNN
	1    4150 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5EDCB46B
P 4500 1250
F 0 "C2" H 4618 1296 50  0000 L CNN
F 1 "2200uF 50V" H 4550 1100 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P2.50mm" H 4538 1100 50  0001 C CNN
F 3 "~" H 4500 1250 50  0001 C CNN
	1    4500 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 1000 4150 1000
Wire Wire Line
	4150 1000 4150 1100
Wire Wire Line
	4500 1000 4500 1100
Wire Wire Line
	4150 1000 4500 1000
Connection ~ 4150 1000
Wire Wire Line
	4150 1400 4150 1500
Wire Wire Line
	4150 1500 3250 1500
Wire Wire Line
	3250 1500 3250 1000
Wire Wire Line
	3250 1000 3350 1000
Wire Wire Line
	4500 1400 4500 1500
Wire Wire Line
	4500 1500 4150 1500
Connection ~ 4150 1500
$Comp
L Device:C C3
U 1 1 5EDD0ECF
P 5150 1250
F 0 "C3" H 5265 1296 50  0000 L CNN
F 1 "100nF" H 5265 1205 50  0000 L CNN
F 2 "Capacitor_THT:C_Axial_L3.8mm_D2.6mm_P7.50mm_Horizontal" H 5188 1100 50  0001 C CNN
F 3 "~" H 5150 1250 50  0001 C CNN
	1    5150 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5EDD1FF1
P 6200 1250
F 0 "C4" H 6315 1296 50  0000 L CNN
F 1 "100nF" H 6315 1205 50  0000 L CNN
F 2 "Capacitor_THT:C_Axial_L3.8mm_D2.6mm_P7.50mm_Horizontal" H 6238 1100 50  0001 C CNN
F 3 "~" H 6200 1250 50  0001 C CNN
	1    6200 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1000 5150 1000
Connection ~ 4500 1000
Wire Wire Line
	5150 1100 5150 1000
Connection ~ 5150 1000
Wire Wire Line
	5150 1000 4500 1000
Wire Wire Line
	6000 1000 6200 1000
Wire Wire Line
	6200 1000 6200 1100
Wire Wire Line
	5150 1400 5150 1500
Wire Wire Line
	5150 1500 4750 1500
Connection ~ 4500 1500
Wire Wire Line
	5150 1500 5700 1500
Wire Wire Line
	6200 1500 6200 1400
Connection ~ 5150 1500
Wire Wire Line
	5700 1300 5700 1500
Connection ~ 5700 1500
Wire Wire Line
	5700 1500 6200 1500
$Comp
L Device:C C5
U 1 1 5EDDB8D6
P 6850 1250
F 0 "C5" H 6965 1296 50  0000 L CNN
F 1 "100nF" H 6965 1205 50  0000 L CNN
F 2 "Capacitor_THT:C_Axial_L3.8mm_D2.6mm_P7.50mm_Horizontal" H 6888 1100 50  0001 C CNN
F 3 "~" H 6850 1250 50  0001 C CNN
	1    6850 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5EDDB8DC
P 7900 1250
F 0 "C6" H 8015 1296 50  0000 L CNN
F 1 "100nF" H 8015 1205 50  0000 L CNN
F 2 "Capacitor_THT:C_Axial_L3.8mm_D2.6mm_P7.50mm_Horizontal" H 7938 1100 50  0001 C CNN
F 3 "~" H 7900 1250 50  0001 C CNN
	1    7900 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1000 6850 1000
Wire Wire Line
	6850 1100 6850 1000
Connection ~ 6850 1000
Wire Wire Line
	6850 1000 6500 1000
Wire Wire Line
	7700 1000 7900 1000
Wire Wire Line
	7900 1000 7900 1100
Wire Wire Line
	6850 1400 6850 1500
Wire Wire Line
	6850 1500 6400 1500
Wire Wire Line
	6850 1500 7400 1500
Wire Wire Line
	7900 1500 7900 1400
Connection ~ 6850 1500
Wire Wire Line
	7400 1300 7400 1500
Connection ~ 7400 1500
Wire Wire Line
	7400 1500 7900 1500
$Comp
L Connector:Screw_Terminal_01x02 J4
U 1 1 5EDDC272
P 6500 1900
F 0 "J4" V 6372 1980 50  0000 L CNN
F 1 "Screw_Terminal_01x02" V 6463 1980 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 6500 1900 50  0001 C CNN
F 3 "~" H 6500 1900 50  0001 C CNN
	1    6500 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	6500 1000 6500 1700
Connection ~ 6500 1000
Wire Wire Line
	6500 1000 6200 1000
Wire Wire Line
	6400 1500 6400 1700
Connection ~ 6400 1500
Wire Wire Line
	6400 1500 6200 1500
$Comp
L Regulator_Linear:L7805 U2
U 1 1 5EDDF45E
P 7400 1000
F 0 "U2" H 7400 1242 50  0000 C CNN
F 1 "L7805" H 7400 1151 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 7425 850 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 7400 950 50  0001 C CNN
	1    7400 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NPN_BCE Q1
U 1 1 5EDDFBB5
P 2750 2300
F 0 "Q1" H 2941 2346 50  0000 L CNN
F 1 "Q_NPN_BCE" H 2941 2255 50  0000 L CNN
F 2 "Package_TO_SOT_THT:SIPAK_Vertical" H 2950 2400 50  0001 C CNN
F 3 "~" H 2750 2300 50  0001 C CNN
	1    2750 2300
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5EDE0D21
P 2650 2700
F 0 "#PWR0101" H 2650 2450 50  0001 C CNN
F 1 "GND" H 2655 2527 50  0000 C CNN
F 2 "" H 2650 2700 50  0001 C CNN
F 3 "" H 2650 2700 50  0001 C CNN
	1    2650 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2500 2650 2700
Wire Wire Line
	2650 2100 2650 2050
Wire Wire Line
	2650 2050 2050 2050
Wire Wire Line
	2050 2050 2050 2000
$Comp
L Device:R R1
U 1 1 5EDE2E25
P 3250 2300
F 0 "R1" V 3043 2300 50  0000 C CNN
F 1 "330" V 3134 2300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3180 2300 50  0001 C CNN
F 3 "~" H 3250 2300 50  0001 C CNN
	1    3250 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 2300 3100 2300
$Comp
L Device:LED D1
U 1 1 5EDE4D19
P 3500 2550
F 0 "D1" V 3539 2432 50  0000 R CNN
F 1 "LED" V 3448 2432 50  0000 R CNN
F 2 "LED_THT:LED_D1.8mm_W1.8mm_H2.4mm_Horizontal_O6.35mm_Z1.6mm" H 3500 2550 50  0001 C CNN
F 3 "~" H 3500 2550 50  0001 C CNN
	1    3500 2550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3400 2300 3500 2300
Wire Wire Line
	3500 2300 3500 2400
$Comp
L power:GND #PWR0102
U 1 1 5EDE688A
P 3500 3100
F 0 "#PWR0102" H 3500 2850 50  0001 C CNN
F 1 "GND" H 3505 2927 50  0000 C CNN
F 2 "" H 3500 3100 50  0001 C CNN
F 3 "" H 3500 3100 50  0001 C CNN
	1    3500 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5EDE7E27
P 4750 1700
F 0 "#PWR0103" H 4750 1450 50  0001 C CNN
F 1 "GND" H 4755 1527 50  0000 C CNN
F 2 "" H 4750 1700 50  0001 C CNN
F 3 "" H 4750 1700 50  0001 C CNN
	1    4750 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 1500 4750 1700
Connection ~ 4750 1500
Wire Wire Line
	4750 1500 4500 1500
$Comp
L Connector:Screw_Terminal_01x02 J5
U 1 1 5EDEB997
P 8750 1200
F 0 "J5" H 8830 1192 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 8830 1101 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 8750 1200 50  0001 C CNN
F 3 "~" H 8750 1200 50  0001 C CNN
	1    8750 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1000 8100 1000
Wire Wire Line
	8250 1000 8250 1200
Wire Wire Line
	8250 1200 8550 1200
Connection ~ 7900 1000
Wire Wire Line
	8550 1300 8250 1300
Wire Wire Line
	8250 1300 8250 1500
Wire Wire Line
	8250 1500 7900 1500
Connection ~ 7900 1500
Wire Wire Line
	8100 1000 8100 2050
Wire Wire Line
	8100 2050 3100 2050
Wire Wire Line
	3100 2050 3100 1450
Wire Wire Line
	3100 1450 2750 1450
Wire Wire Line
	2750 1450 2750 1300
Wire Wire Line
	2750 1300 2050 1300
Wire Wire Line
	2050 1300 2050 1400
Connection ~ 8100 1000
Wire Wire Line
	8100 1000 8250 1000
$Comp
L Connector:Conn_01x01_Female J3
U 1 1 5EDF1A95
P 3700 2300
F 0 "J3" H 3728 2326 50  0000 L CNN
F 1 "Conn_01x01_Female" H 3728 2235 50  0000 L CNN
F 2 "Connector_Pin:Pin_D0.7mm_L6.5mm_W1.8mm_FlatFork" H 3700 2300 50  0001 C CNN
F 3 "~" H 3700 2300 50  0001 C CNN
	1    3700 2300
	1    0    0    -1  
$EndComp
Connection ~ 3500 2300
$Comp
L Device:R R2
U 1 1 5EDF223B
P 3500 2900
F 0 "R2" H 3430 2854 50  0000 R CNN
F 1 "330" H 3430 2945 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3430 2900 50  0001 C CNN
F 3 "~" H 3500 2900 50  0001 C CNN
	1    3500 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	3500 2700 3500 2750
Wire Wire Line
	3500 3050 3500 3100
$Comp
L Relay:FINDER-36.11 K1
U 1 1 5EE00520
P 1850 1700
F 0 "K1" H 1420 1654 50  0000 R CNN
F 1 "FINDER-36.11" H 1420 1745 50  0000 R CNN
F 2 "Relay_THT:Relay_SPDT_Finder_36.11" H 3120 1670 50  0001 C CNN
F 3 "https://gfinder.findernet.com/public/attachments/36/EN/S36EN.pdf" H 1850 1700 50  0001 C CNN
	1    1850 1700
	-1   0    0    1   
$EndComp
Wire Wire Line
	1350 1000 1350 1150
Wire Wire Line
	1050 1000 1350 1000
Wire Wire Line
	1050 900  1400 900 
Wire Wire Line
	1650 1400 1650 1150
Connection ~ 1650 1150
Wire Wire Line
	1650 1150 1350 1150
Wire Wire Line
	1400 900  1400 2000
Wire Wire Line
	1400 2000 1450 2000
Connection ~ 1400 900 
Wire Wire Line
	1400 900  1850 900 
$EndSCHEMATC
