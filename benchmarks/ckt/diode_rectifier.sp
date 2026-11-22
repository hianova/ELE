* ELE v9.0.0 Benchmark: Nonlinear Shockley Diode Clamp
V1 vin 0 DC 5.0 AC 0.1
R1 vin vd 100
D1 vd 0 IS=1e-12 N=1.0 RS=0.05
.OP
.TRAN 1u 50u PROBE=vd
.END
