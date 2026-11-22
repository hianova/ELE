* ELE v9.0.0 Benchmark: Package-Die Power Distribution Network (Z_max < 0.25 Ohm)
Rvrm vrm 0 0.01
Lpkg vrm die 2n
Cdie die 0 200n
Resr die 0 0.08
Iload die 0 DC 0 AC 1
.AC DEC 125 10k 100MEG VOUT=die
.PDN NODE=die ISRC=Iload ZMAX=0.25
.END
