* ELE v9.0.0 Benchmark: First-Order RC Lowpass (fc = 159.155 Hz)
V1 vin 0 DC 0 AC 1 PULSE(0 1 0 1n 1n 1 2)
R1 vin vout 1k
C1 vout 0 1u
.AC DEC 125 1 100k VOUT=vout VIN=V1
.TRAN 10u 1m PROBE=vout
.END
