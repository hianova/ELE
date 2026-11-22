* ELE v9.0.0 Benchmark: 800V SiC MOSFET Switching Stage + Snubber + PVT Corners
Vdc vdc 0 DC 800
R_bus vdc vbus 0.08 TOL=5
L_bus vbus sw 15n TOL=10
R_load vbus sw 16
C_oss sw 0 1.2n TOL=10
R_snub sw snub 4.7 TOL=5
C_snub snub 0 4.7n TOL=10
Vgate gate 0 PULSE(0 15 20n 8n 8n 80n 200n)
M_ls sw gate 0 VTH=3.5 RON=12 ROFF=1e6
D_fw 0 sw IS=1e-11 N=1.2 RS=0.05
.TRAN 1n 220n PROBE=sw
.SOA NODE=sw VMIN=-20 VMAX=1150
.END
