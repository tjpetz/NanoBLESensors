EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "IoTCentral"
Date "2020-09-09"
Rev "1.0"
Comp "www.tjpetz.com"
Comment1 ""
Comment2 "https://creativecommons.org/licenses/by/4.0/"
Comment3 "License CC BY 4.0"
Comment4 "Author: Thomas J. Petz, Jr."
$EndDescr
Wire Wire Line
	5250 4800 5250 4850
Wire Wire Line
	5350 4800 5250 4800
Wire Wire Line
	5350 4650 5350 4800
Connection ~ 5250 4800
Wire Wire Line
	5250 4650 5250 4800
$Comp
L power:GND #PWR02
U 1 1 5F5A33ED
P 5250 4850
F 0 "#PWR02" H 5250 4600 50  0001 C CNN
F 1 "GND" H 5255 4677 50  0000 C CNN
F 2 "" H 5250 4850 50  0001 C CNN
F 3 "" H 5250 4850 50  0001 C CNN
	1    5250 4850
	1    0    0    -1  
$EndComp
NoConn ~ 5450 2650
NoConn ~ 4750 3050
NoConn ~ 4750 3150
NoConn ~ 4750 3450
NoConn ~ 4750 3550
NoConn ~ 4750 3650
NoConn ~ 4750 3750
NoConn ~ 4750 3850
NoConn ~ 4750 3950
NoConn ~ 4750 4050
NoConn ~ 4750 4150
NoConn ~ 4750 4250
NoConn ~ 4750 4350
NoConn ~ 5750 4350
NoConn ~ 5750 4250
NoConn ~ 5750 3950
NoConn ~ 5750 3850
NoConn ~ 5750 3750
NoConn ~ 5750 3450
NoConn ~ 5750 3150
NoConn ~ 5750 3050
$Comp
L MCU_Module:Arduino_Nano_v3.x A3
U 1 1 5F59300D
P 5250 3650
F 0 "A3" H 4850 4650 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 5800 2650 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 5250 3650 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 5250 3650 50  0001 C CNN
	1    5250 3650
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5F7B99B8
P 3350 3250
F 0 "SW1" H 3350 3535 50  0000 C CNN
F 1 "SW_SelectPage" H 3350 3444 50  0000 C CNN
F 2 "" H 3350 3450 50  0001 C CNN
F 3 "~" H 3350 3450 50  0001 C CNN
	1    3350 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5F7BD70B
P 3000 3350
F 0 "#PWR04" H 3000 3100 50  0001 C CNN
F 1 "GND" H 3005 3177 50  0000 C CNN
F 2 "" H 3000 3350 50  0001 C CNN
F 3 "" H 3000 3350 50  0001 C CNN
	1    3000 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3250 3000 3250
Wire Wire Line
	3000 3250 3000 3350
$Comp
L power:GND #PWR07
U 1 1 5F7BF12A
P 7000 4700
F 0 "#PWR07" H 7000 4450 50  0001 C CNN
F 1 "GND" H 7005 4527 50  0000 C CNN
F 2 "" H 7000 4700 50  0001 C CNN
F 3 "" H 7000 4700 50  0001 C CNN
	1    7000 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 4550 7000 4700
$Comp
L power:VCC #PWR06
U 1 1 5F7C1988
P 7000 3700
F 0 "#PWR06" H 7000 3550 50  0001 C CNN
F 1 "VCC" H 7017 3873 50  0000 C CNN
F 2 "" H 7000 3700 50  0001 C CNN
F 3 "" H 7000 3700 50  0001 C CNN
	1    7000 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3700 7000 3850
Wire Wire Line
	5750 4050 6250 4050
Wire Wire Line
	6250 4050 6250 4300
Wire Wire Line
	6250 4300 6650 4300
Wire Wire Line
	5750 4150 6500 4150
Wire Wire Line
	6500 4150 6500 4100
Wire Wire Line
	6500 4100 6650 4100
$Comp
L MyParts2:SSD1306_128x64_OLED A4
U 1 1 5F7B8CCC
P 6850 4150
F 0 "A4" H 7278 4146 50  0000 L CNN
F 1 "SSD1306_128x64_OLED" H 7278 4055 50  0000 L CNN
F 2 "" H 6850 4150 50  0001 C CNN
F 3 "" H 6850 4150 50  0001 C CNN
	1    6850 4150
	1    0    0    -1  
$EndComp
NoConn ~ 5750 3650
$Comp
L power:VCC #PWR01
U 1 1 5F7A98AD
P 5350 2400
F 0 "#PWR01" H 5350 2250 50  0001 C CNN
F 1 "VCC" H 5367 2573 50  0000 C CNN
F 2 "" H 5350 2400 50  0001 C CNN
F 3 "" H 5350 2400 50  0001 C CNN
	1    5350 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 2400 5350 2650
NoConn ~ 5150 2650
Wire Wire Line
	3550 3250 4750 3250
NoConn ~ 4750 3350
$EndSCHEMATC
