#!/usr/bin/env python3
"""
ELE v11.0.0 Python SDK (Pure Standard Library)
Clean wrapper around `build/ele_cli` + persistent `ctypes` `EleFastSession`
over `build/libele.{dylib,so}`, plus Touchstone .sNp / .grid helpers.
"""
import cmath
import ctypes
import json
import math
import os
import struct
import subprocess
import sys
import tempfile


def read_snp(path, nports_hint=0):
    """Parse Touchstone v1.1 .sNp file into (freqs_hz, matrices, z0)."""
    if nports_hint <= 0:
        ext = os.path.splitext(path)[1].lower()
        if len(ext) == 4 and ext.startswith(".s") and ext.endswith("p") and ext[2].isdigit():
            nports = int(ext[2])
        else:
            nports = 2
    else:
        nports = nports_hint

    fmult = 1e9
    fmt = "MA"
    z0 = 50.0
    tokens = []
    with open(path, "r") as f:
        for raw in f:
            line = raw.split("!")[0].strip()
            if not line or line.startswith("["):
                continue
            if line.startswith("#"):
                parts = line[1:].upper().split()
                i = 0
                while i < len(parts):
                    t = parts[i]
                    if t == "HZ":
                        fmult = 1.0
                    elif t == "KHZ":
                        fmult = 1e3
                    elif t == "MHZ":
                        fmult = 1e6
                    elif t == "GHZ":
                        fmult = 1e9
                    elif t in ("RI", "MA", "DB"):
                        fmt = t
                    elif t == "R" and i + 1 < len(parts):
                        z0 = float(parts[i + 1])
                        i += 1
                    i += 1
                continue
            tokens.extend(float(x) for x in line.split())

    rec_len = 1 + 2 * nports * nports
    freqs = []
    mats = []
    for k in range(0, len(tokens), rec_len):
        if k + rec_len > len(tokens):
            break
        freqs.append(tokens[k] * fmult)
        pairs = []
        for p in range(nports * nports):
            a = tokens[k + 1 + 2 * p]
            b = tokens[k + 2 + 2 * p]
            if fmt == "RI":
                pairs.append(complex(a, b))
            elif fmt == "MA":
                pairs.append(cmath.rect(a, math.radians(b)))
            else:
                pairs.append(cmath.rect(10.0 ** (a / 20.0), math.radians(b)))
        if nports == 2:
            # Touchstone 2-port order: S11, S21, S12, S22
            mat = [[pairs[0], pairs[2]], [pairs[1], pairs[3]]]
        else:
            mat = [[pairs[i * nports + j] for j in range(nports)] for i in range(nports)]
        mats.append(mat)
    return freqs, mats, z0


class EleEngine:
    """High-level interface to ELE v10.0.0 certified multiphysics CLI."""

    def __init__(self, cli_path="build/ele_cli"):
        self.cli_path = cli_path

    def _run(self, args):
        res = subprocess.run([self.cli_path] + args, capture_output=True, text=True)
        if res.returncode != 0:
            raise RuntimeError(f"ele_cli failed ({res.returncode}): {' '.join(args)}\n{res.stderr}\n{res.stdout}")
        return res.stdout

    def selftest(self):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            self._run(["selftest", "--json", jp])
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def em_passivity(self, snp_path, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["em", "passivity", snp_path, "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def em_peec(self, peec_path, s2p_path=None, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["em", "peec", peec_path, "--json", jp]
            if s2p_path:
                args += ["--s2p", s2p_path]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def em_fdtd(self, er=4.0, sigma=0.04, s2p_path=None, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["em", "fdtd", "--er", str(er), "--sigma", str(sigma), "--json", jp]
            if s2p_path:
                args += ["--s2p", s2p_path]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def ckt_run(self, sp_path, corners=True, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["ckt", "run", sp_path, "--json", jp]
            if corners:
                args.append("--corners")
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def ckt_si_eye(self, sp_path, rate_gbps=4.0, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["ckt", "si-eye", sp_path, "--rate-gbps", str(rate_gbps), "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def motor_run(self, motor_path, rpm=3000.0, torque=260.0, sweep=True, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["motor", "run", motor_path, "--rpm", str(rpm), "--torque", str(torque), "--json", jp]
            if sweep:
                args.append("--sweep")
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def grid_pf(self, grid_path, n1=True):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["grid", "pf", grid_path, "--json", jp]
            if n1:
                args.append("--n1")
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def grid_opf(self, grid_path, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["grid", "opf", grid_path, "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def grid_emt(self, grid_path, fault_bus=7, t_clear=0.12, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["grid", "emt", grid_path, "--fault-bus", str(fault_bus), "--t-clear", str(t_clear), "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def field_to_circuit(self, peec_path, sp_path, motor_path, out_dir="build"):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            self._run([
                "field-to-circuit",
                "--peec", peec_path,
                "--sp", sp_path,
                "--motor", motor_path,
                "--out-dir", out_dir,
                "--json", jp,
            ])
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def cert_verify(self, cert_path):
        self._run(["cert-verify", cert_path])
        return True

    def cert_pack(self, json_in_path, bin_out_path):
        self._run(["cert-pack", json_in_path, bin_out_path])
        return True

    def cert_unpack(self, bin_in_path, json_out_path):
        self._run(["cert-unpack", bin_in_path, json_out_path])
        return True

    def lab_validate_thermal(self, csv_path, sp_path=None, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["lab", "validate-thermal", csv_path, "--json", jp]
            if sp_path:
                args += ["--sp", sp_path]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def lab_validate_csi(self, csv_path, s2p_path=None, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["lab", "validate-csi", csv_path, "--json", jp]
            if s2p_path:
                args += ["--s2p", s2p_path]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def lab_validate_power(self, csv_path, sp_path=None, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["lab", "validate-power", csv_path, "--json", jp]
            if sp_path:
                args += ["--sp", sp_path]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def grid_chordal(self, grid_path, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["grid", "chordal", grid_path, "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def grid_sos2(self, grid_path, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["grid", "sos2", grid_path, "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def grid_observable(self, grid_path, obs, eps=5.0, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["grid", "observable", grid_path, "--obs", obs, "--eps", str(eps), "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def em_bound(self, kl=2.0, zeta=10.0, xi0=0.0, cells=16, bw_frac=0.0, fdtd_check=False, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["em", "bound", "--kl", str(kl), "--zeta", str(zeta),
                    "--xi0", str(xi0), "--cells", str(cells), "--json", jp]
            if bw_frac > 0.0:
                args += ["--bw-frac", str(bw_frac)]
            if fdtd_check:
                args += ["--fdtd-check"]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def em_bode_fano(self, z0=50.0, rl=50.0, cl_pf=2.0, flo_ghz=2.0, fhi_ghz=3.0,
                     seed=20261122, cert_path=None, lean_path=None, mod_name=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["em", "bode-fano", "--z0", str(z0), "--rl", str(rl),
                    "--cl-pf", str(cl_pf), "--flo-ghz", str(flo_ghz), "--fhi-ghz", str(fhi_ghz),
                    "--seed", str(seed), "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            if lean_path:
                args += ["--emit-lean", lean_path]
            if mod_name:
                args += ["--name", mod_name]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def em_enforce_passivity(self, snp_path, out_snp_path=None, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["em", "enforce-passivity", snp_path, "--json", jp]
            if out_snp_path:
                args += ["--out", out_snp_path]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def em_kyp_sim(self, snp_path, rho=0.985, steps=400, cert_path=None):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            jp = tf.name
        try:
            args = ["em", "kyp-sim", snp_path, "--rho", str(rho), "--steps", str(steps), "--json", jp]
            if cert_path:
                args += ["--cert", cert_path]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if os.path.exists(jp):
                os.remove(jp)

    def atlas(self, bench_dir="benchmarks", out_json=None, cert_dir="build/certs"):
        created = False
        jp = out_json
        if jp is None:
            with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
                jp = tf.name
            created = True
        try:
            args = ["atlas", "--dir", bench_dir, "--out", jp, "--cert-dir", cert_dir]
            self._run(args)
            with open(jp, "r") as f:
                return json.load(f)
        finally:
            if created and os.path.exists(jp):
                os.remove(jp)


class EleFastSession:
    """In-process persistent ctypes grid session over libele.{dylib,so} (no file reload/spawn)."""

    _LIB_CACHE = {}

    @classmethod
    def _load_lib(cls, lib_path=None):
        if lib_path is None:
            ext = "dylib" if sys.platform == "darwin" else "so"
            lib_path = os.path.join("build", f"libele.{ext}")
        key = os.path.abspath(lib_path)
        if key not in cls._LIB_CACHE:
            lib = ctypes.CDLL(key)
            lib.ele_grid_session_create.argtypes = [ctypes.c_char_p]
            lib.ele_grid_session_create.restype = ctypes.c_void_p
            lib.ele_grid_session_destroy.argtypes = [ctypes.c_void_p]
            lib.ele_grid_session_destroy.restype = None
            lib.ele_grid_session_set_bus_limits.argtypes = [
                ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_double
            ]
            lib.ele_grid_session_set_bus_limits.restype = ctypes.c_int
            lib.ele_grid_session_set_branch_limits.argtypes = [
                ctypes.c_void_p, ctypes.c_int, ctypes.c_double,
                ctypes.c_double, ctypes.c_double, ctypes.c_double
            ]
            lib.ele_grid_session_set_branch_limits.restype = ctypes.c_int
            lib.ele_grid_session_solve_opf.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
            lib.ele_grid_session_solve_opf.restype = ctypes.c_int
            lib.ele_grid_session_solve_sos2.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
            lib.ele_grid_session_solve_sos2.restype = ctypes.c_int
            lib.ele_grid_session_solve_obs.argtypes = [
                ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_void_p
            ]
            lib.ele_grid_session_solve_obs.restype = ctypes.c_int
            cls._LIB_CACHE[key] = lib
        return cls._LIB_CACHE[key]

    def __init__(self, grid_path, lib_path=None):
        self._lib = self._load_lib(lib_path)
        self._handle = self._lib.ele_grid_session_create(os.fsencode(grid_path))
        if not self._handle:
            raise RuntimeError(f"ele_grid_session_create failed for {grid_path}")

    def close(self):
        if getattr(self, "_handle", None):
            self._lib.ele_grid_session_destroy(self._handle)
            self._handle = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def set_bus_limits(self, bus_id, vmin, vmax):
        rc = self._lib.ele_grid_session_set_bus_limits(self._handle, int(bus_id), float(vmin), float(vmax))
        if rc != 0:
            raise RuntimeError(f"ele_grid_session_set_bus_limits failed ({rc}) for bus {bus_id}")

    def set_branch_limits(self, branch_idx, smax_mva, shift_deg=0.0, angmin_deg=-60.0, angmax_deg=60.0):
        rc = self._lib.ele_grid_session_set_branch_limits(
            self._handle, int(branch_idx), float(smax_mva),
            float(shift_deg), float(angmin_deg), float(angmax_deg)
        )
        if rc != 0:
            raise RuntimeError(f"ele_grid_session_set_branch_limits failed ({rc}) for branch {branch_idx}")

    def solve_opf(self):
        buf = ctypes.create_string_buffer(82064)
        rc = self._lib.ele_grid_session_solve_opf(self._handle, ctypes.cast(buf, ctypes.c_void_p))
        if rc != 0:
            raise RuntimeError(f"ele_grid_session_solve_opf failed ({rc})")
        conv, raw, lb, ub, gabs, gpct, mu, drnd = struct.unpack_from("<i4x5d2q", buf.raw, 0)
        return {
            "converged": conv, "c_lb_dual_raw": raw, "c_lb_cert": lb,
            "c_ub_primal": ub, "gap_abs": gabs, "gap_pct": gpct,
            "mu_psd_int": mu, "delta_round_int": drnd,
        }

    def solve_sos2(self):
        buf = ctypes.create_string_buffer(67664)
        rc = self._lib.ele_grid_session_solve_sos2(self._handle, ctypes.cast(buf, ctypes.c_void_p))
        if rc != 0:
            raise RuntimeError(f"ele_grid_session_solve_sos2 failed ({rc})")
        nb, d2, nm4, conv, s1, raw, lb, ub, g1, g2, imp, mu = struct.unpack_from("<4i7dq", buf.raw, 0)
        return {
            "n_bus": nb, "d2_dim": d2, "n_monos4": nm4, "converged": conv,
            "shor1_lb_cert": s1, "sos2_lb_raw": raw, "sos2_lb_cert": lb,
            "c_ub_primal": ub, "shor1_gap_pct": g1, "sos2_gap_pct": g2,
            "improvement_abs": imp, "q0_mu_psd": mu,
        }

    def solve_obs(self, kind_or_obs, target_id=None, eps=5.0):
        if isinstance(kind_or_obs, str) and ":" in kind_or_obs:
            kstr, tstr = kind_or_obs.split(":", 1)
            kind = 0 if kstr == "vm2" else 1
            tid = int(tstr)
        elif isinstance(kind_or_obs, str):
            kind = 0 if kind_or_obs == "vm2" else 1
            tid = int(target_id)
        else:
            kind = int(kind_or_obs)
            tid = int(target_id)
        buf = ctypes.create_string_buffer(67656)
        rc = self._lib.ele_grid_session_solve_obs(
            self._handle, kind, tid, float(eps), ctypes.cast(buf, ctypes.c_void_p)
        )
        if rc != 0:
            raise RuntimeError(f"ele_grid_session_solve_obs failed ({rc})")
        rk, rtid, cap, opr, olb, oub, wid, cub, clb, cgap = struct.unpack_from("<2i8d", buf.raw, 0)
        return {
            "kind": "vm2" if rk == 0 else "pflow",
            "target_id": rtid, "cost_cap": cap,
            "o_primal": opr, "o_lb_cert": olb, "o_ub_cert": oub,
            "width": wid, "c_ub_primal": cub, "c_lb_cert": clb, "cost_gap_abs": cgap,
        }
