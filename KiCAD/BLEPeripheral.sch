EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "BLEPeripheral"
Date "2020-09-09"
Rev "1.0"
Comp "www.tjpetz.com"
Comment1 ""
Comment2 "https://creativecommons.org/licenses/by/4.0/"
Comment3 "License CC BY 4.0"
Comment4 "Author: Thomas J. Petz, Jr."
$EndDescr
Wire Wire Line
	6800 3700 6800 3350
Wire Wire Line
	4550 3700 6800 3700
Wire Wire Line
	4550 4450 4550 3700
Wire Wire Line
	4800 4450 4550 4450
Wire Wire Line
	7100 3750 7100 3550
$Comp
L Transistor_FET:2N7000 Q1
U 1 1 5F5B9E37
P 7000 3350
F 0 "Q1" H 7204 3396 50  0000 L CNN
F 1 "2N7000" H 7204 3305 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7200 3275 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 7000 3350 50  0001 L CNN
	1    7000 3350
	1    0    0    -1  
$EndComp
Text Notes 6300 2100 0    50   ~ 0
We use a 2::1 voltage divider to bring the battery\nvoltage down to below the 2.4v range of the ADC\non the Nano.  Note, at 20k we have a continual\n210 uA draw, therefore we control this via a \nMOSFET.
Wire Wire Line
	5300 6000 5300 6050
Wire Wire Line
	5400 6000 5300 6000
Wire Wire Line
	5400 5850 5400 6000
Connection ~ 5300 6000
Wire Wire Line
	5300 5850 5300 6000
$Comp
L power:GND #PWR02
U 1 1 5F5A33ED
P 5300 6050
F 0 "#PWR02" H 5300 5800 50  0001 C CNN
F 1 "GND" H 5305 5877 50  0000 C CNN
F 2 "" H 5300 6050 50  0001 C CNN
F 3 "" H 5300 6050 50  0001 C CNN
	1    5300 6050
	1    0    0    -1  
$EndComp
NoConn ~ 5500 3850
NoConn ~ 4800 4250
NoConn ~ 4800 4350
NoConn ~ 4800 4650
NoConn ~ 4800 4750
NoConn ~ 4800 4850
NoConn ~ 4800 4950
NoConn ~ 4800 5050
NoConn ~ 4800 5150
NoConn ~ 4800 5250
NoConn ~ 4800 5350
NoConn ~ 4800 5450
NoConn ~ 4800 5550
NoConn ~ 5800 5550
NoConn ~ 5800 5450
NoConn ~ 5800 5150
NoConn ~ 5800 5050
NoConn ~ 5800 4950
NoConn ~ 5800 4650
NoConn ~ 5800 4350
NoConn ~ 5800 4250
Wire Wire Line
	3000 3100 3850 3100
Wire Wire Line
	3000 2900 3850 2900
Wire Wire Line
	4600 3100 4600 3450
Wire Wire Line
	7100 2700 7100 2750
Wire Wire Line
	6600 2700 7100 2700
Wire Wire Line
	6600 4850 6600 2700
Wire Wire Line
	5800 4850 6600 4850
Wire Wire Line
	7100 2950 7100 3150
$Comp
L power:GND #PWR03
U 1 1 5F59D9CA
P 7100 3750
F 0 "#PWR03" H 7100 3500 50  0001 C CNN
F 1 "GND" H 7105 3577 50  0000 C CNN
F 2 "" H 7100 3750 50  0001 C CNN
F 3 "" H 7100 3750 50  0001 C CNN
	1    7100 3750
	1    0    0    -1  
$EndComp
Connection ~ 7100 2700
Wire Wire Line
	7100 2650 7100 2700
Wire Wire Line
	2800 2250 2800 2400
Wire Wire Line
	7100 2250 7100 2450
Wire Wire Line
	2800 2250 7100 2250
Wire Wire Line
	5200 2900 4600 2900
Wire Wire Line
	5200 3850 5200 2900
$Comp
L power:GND #PWR01
U 1 1 5F59B20A
P 4600 3450
F 0 "#PWR01" H 4600 3200 50  0001 C CNN
F 1 "GND" H 4605 3277 50  0000 C CNN
F 2 "" H 4600 3450 50  0001 C CNN
F 3 "" H 4600 3450 50  0001 C CNN
	1    4600 3450
	1    0    0    -1  
$EndComp
Connection ~ 2800 2250
Wire Wire Line
	2800 2000 2800 2250
Wire Wire Line
	2450 2300 2450 2400
Wire Wire Line
	2400 2300 2450 2300
Wire Wire Line
	2400 2000 2400 2300
$Comp
L Device:R_Small_US R2
U 1 1 5F597E5F
P 7100 2850
F 0 "R2" H 7168 2896 50  0000 L CNN
F 1 "10k" H 7168 2805 50  0000 L CNN
F 2 "" H 7100 2850 50  0001 C CNN
F 3 "~" H 7100 2850 50  0001 C CNN
	1    7100 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 5F597A82
P 7100 2550
F 0 "R1" H 7168 2596 50  0000 L CNN
F 1 "10k" H 7168 2505 50  0000 L CNN
F 2 "" H 7100 2550 50  0001 C CNN
F 3 "~" H 7100 2550 50  0001 C CNN
	1    7100 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:Battery BT1
U 1 1 5F59650B
P 2600 2000
F 0 "BT1" V 2355 2000 50  0000 C CNN
F 1 "Battery" V 2446 2000 50  0000 C CNN
F 2 "" V 2600 2060 50  0001 C CNN
F 3 "~" V 2600 2060 50  0001 C CNN
	1    2600 2000
	0    1    1    0   
$EndComp
$Comp
L MyParts:MT3608-Boost-Converter-Module A2
U 1 1 5F5939BB
P 4250 3000
F 0 "A2" H 4000 3250 50  0000 C CNN
F 1 "MT3608-Boost-Converter-Module" H 4250 2700 50  0000 C CNN
F 2 "" H 4100 3150 50  0001 C CNN
F 3 "https://www.amazon.com/gp/product/B08979VKVL/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1" H 4100 3150 50  0001 C CNN
	1    4250 3000
	1    0    0    -1  
$EndComp
$Comp
L MyParts:TP4056-Li-Battery-Charger-Module A1
U 1 1 5F5932F9
P 2650 3000
F 0 "A1" H 2300 3450 50  0000 R CNN
F 1 "TP4056-Li-Battery-Charger-Module" H 3300 2650 50  0000 R CNN
F 2 "" H 2500 3150 50  0001 C CNN
F 3 "" H 2500 3150 50  0001 C CNN
	1    2650 3000
	1    0    0    -1  
$EndComp
$Comp
L MCU_Module:Arduino_Nano_v3.x A3
U 1 1 5F59300D
P 5300 4850
F 0 "A3" H 4900 5850 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 5850 3850 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 5300 4850 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 5300 4850 50  0001 C CNN
	1    5300 4850
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5F7B99B8
P 3400 4550
F 0 "SW1" H 3400 4835 50  0000 C CNN
F 1 "SW_SelectPage" H 3400 4744 50  0000 C CNN
F 2 "" H 3400 4750 50  0001 C CNN
F 3 "~" H 3400 4750 50  0001 C CNN
	1    3400 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4550 4800 4550
$Comp
L power:GND #PWR04
U 1 1 5F7BD70B
P 3050 4650
F 0 "#PWR04" H 3050 4400 50  0001 C CNN
F 1 "GND" H 3055 4477 50  0000 C CNN
F 2 "" H 3050 4650 50  0001 C CNN
F 3 "" H 3050 4650 50  0001 C CNN
	1    3050 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4550 3050 4550
Wire Wire Line
	3050 4550 3050 4650
$Comp
L power:GND #PWR07
U 1 1 5F7BF12A
P 7050 5900
F 0 "#PWR07" H 7050 5650 50  0001 C CNN
F 1 "GND" H 7055 5727 50  0000 C CNN
F 2 "" H 7050 5900 50  0001 C CNN
F 3 "" H 7050 5900 50  0001 C CNN
	1    7050 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 5750 7050 5900
$Comp
L power:VCC #PWR05
U 1 1 5F7C0DBE
P 5400 3650
F 0 "#PWR05" H 5400 3500 50  0001 C CNN
F 1 "VCC" H 5417 3823 50  0000 C CNN
F 2 "" H 5400 3650 50  0001 C CNN
F 3 "" H 5400 3650 50  0001 C CNN
	1    5400 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 3650 5400 3850
$Comp
L power:VCC #PWR06
U 1 1 5F7C1988
P 7050 4900
F 0 "#PWR06" H 7050 4750 50  0001 C CNN
F 1 "VCC" H 7067 5073 50  0000 C CNN
F 2 "" H 7050 4900 50  0001 C CNN
F 3 "" H 7050 4900 50  0001 C CNN
	1    7050 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4900 7050 5050
Wire Wire Line
	5800 5250 6300 5250
Wire Wire Line
	6300 5250 6300 5500
Wire Wire Line
	6300 5500 6700 5500
Wire Wire Line
	5800 5350 6550 5350
Wire Wire Line
	6550 5350 6550 5300
Wire Wire Line
	6550 5300 6700 5300
$Comp
L MyParts2:SSD1306_128x64_OLED A4
U 1 1 5F7B8CCC
P 6900 5350
F 0 "A4" H 7328 5346 50  0000 L CNN
F 1 "SSD1306_128x64_OLED" H 7328 5255 50  0000 L CNN
F 2 "" H 6900 5350 50  0001 C CNN
F 3 "" H 6900 5350 50  0001 C CNN
	1    6900 5350
	1    0    0    -1  
$EndComp
$EndSCHEMATC
