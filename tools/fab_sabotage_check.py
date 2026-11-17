#!/usr/bin/env python3
"""
fab_sabotage_check.py — Univalent Fab/DRC/LVS/OPC/GDSII Single-Fault Sabotage Gate for ELE v8.0.0
==================================================================================================
Empirically proves physical, topological, optical, and binary GDSII sign-off completeness of
`tools/ele_fab.h` by injecting 28 stratified single-fault mutations across ALL FIVE physical layers:
  1. `drc_pitch_width_grid`       (6 mutants: off-grid cell, sub-width M3, Abacus cell overlap,
                                   power-rail VDD/VSS clash, Via enclosure loss, HV space breach),
  2. `lvs_open_short_pad`         (6 mutants: missing VIA12, missing VIA23, M3 track short,
                                   disconnected M1 stub, severed pad feeder, shared pad short),
  3. `univalent_coevolution`      (6 mutants: disabled repeater cloning, zeroed wire RC, disabled
                                   Abacus cursor push, corrupted .flow r_sheet decode, disabled
                                   Pass-2 monotonic tunneling, repeater net double-short),
  4. `litho_2jet_opc`             (5 mutants: zeroed left OPC extension, inverted right OPC serif,
                                   unclamped MRC bridging serif, zeroed Chart-3 bias, wrong layer),
  5. `gdsii_binary_framing`       (5 mutants: HEADER version mismatch, IBM Real8 UNITS corruption,
                                   unclosed BOUNDARY 5th vertex, inverted box winding, bad ENDLIB).

Every mutant is required to:
  1. Match uniquely (`count == 1`) in `tools/ele_fab.h` (0 silent no-op mutants).
  2. Compile cleanly with strict `-std=c17 -mgeneral-regs-only -fno-jump-tables -Werror`
     (`compile_rc == 0`, proving genuine semantic fault injection rather than syntax errors).
  3. Be caught dynamically at runtime by `--fab-verify` (`run_rc != 0`, 100.0% kill rate).
"""

from __future__ import annotations

import concurrent.futures
import os
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class FabMutant:
    name: str
    category: str
    target: str
    replacement: str


def build_fab_mutants() -> list[FabMutant]:
    return [
        # --- Category 1: Constructive Kan DRC, Grid, Width, Enclosure & HV Clearance (6 mutants) ---
        FabMutant(
            "DRC_GRID_OFFBY1_CELL",
            "drc_pitch_width_grid",
            "ws->nodes[id].x0_nm = ws->core_x0 + col * P;",
            "ws->nodes[id].x0_nm = ws->core_x0 + col * P + 1;",
        ),
        FabMutant(
            "DRC_WIDTH_SHRINK_M3",
            "drc_pitch_width_grid",
            "ele_fab_emit_poly(ws, ELE_LAYER_M3, 0,\n"
            "                              net->trunk_x0_nm, net->trunk_y_nm,\n"
            "                              net->trunk_x1_nm, net->trunk_y_nm + W, n);",
            "ele_fab_emit_poly(ws, ELE_LAYER_M3, 0,\n"
            "                              net->trunk_x0_nm, net->trunk_y_nm,\n"
            "                              net->trunk_x1_nm, net->trunk_y_nm + W - pdk->grid_nm, n);",
        ),
        FabMutant(
            "DRC_CELL_ABACUS_COLLIDE",
            "drc_pitch_width_grid",
            "cursor_col = col + c_width + (ws->nodes[id].is_hv ? 2 : 1);",
            "cursor_col = col;",
        ),
        FabMutant(
            "DRC_RAIL_PARITY_SHORT",
            "drc_pitch_width_grid",
            "int32_t ry0 = (r < ELE_FAB_NUM_ROWS) ? ws->row_y0[r] : ws->row_y1[ELE_FAB_NUM_ROWS - 1];",
            "int32_t ry0 = ws->row_y0[0];",
        ),
        FabMutant(
            "DRC_VIA12_ENC_SHRINK",
            "drc_pitch_width_grid",
            "ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, x_col - E, y_stub - E,\n"
            "                                      x_col + V + E, y_stub + V + E, n);",
            "ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, x_col - E + pdk->grid_nm, y_stub - E,\n"
            "                                      x_col + V + E, y_stub + V + E, n);",
        ),
        FabMutant(
            "DRC_HV_EDGE_PAD_SPACING_BREACH",
            "drc_pitch_width_grid",
            "ele_fab_emit_poly(ws, ELE_LAYER_PAD, 0, nd->x0_nm, nd->y0_nm, nd->x1_nm, nd->y1_nm, -1);\n"
            "                ele_fab_emit_poly(ws, ELE_LAYER_M3, 0, nd->x0_nm, nd->y0_nm, nd->x1_nm, nd->y1_nm, -1);",
            "ele_fab_emit_poly(ws, ELE_LAYER_PAD, 0, nd->x0_nm, nd->y0_nm, nd->x1_nm, nd->y1_nm, -1);\n"
            "                ele_fab_emit_poly(ws, ELE_LAYER_M3, 0, nd->x0_nm, nd->y0_nm, nd->x1_nm,\n"
            "                                  (i == ELE_FAB_BASE_CELLS + 38) ? (ws->nodes[i + 1].y0_nm - pdk->min_space_nm) : nd->y1_nm, -1);",
        ),
        # --- Category 2: 3D Union-Find LVS Opens, Shorts & Perimeter Pad Connectivity (6 mutants) ---
        FabMutant(
            "LVS_OMIT_CORE_VIA12",
            "lvs_open_short_pad",
            "ele_fab_emit_poly(ws, ELE_LAYER_VIA12, 0, x_col, y_stub, x_col + V, y_stub + V, n);",
            "/* mutant: omitted core VIA12 cut */",
        ),
        FabMutant(
            "LVS_OMIT_CORE_VIA23",
            "lvs_open_short_pad",
            "ele_fab_emit_poly(ws, ELE_LAYER_VIA23, 0, x_col, net->trunk_y_nm,\n"
            "                                      x_col + V, net->trunk_y_nm + V, n);\n"
            "                    ele_fab_emit_poly(ws, ELE_LAYER_M3, 0, x_col - E, net->trunk_y_nm - E,\n"
            "                                      x_col + V + E, net->trunk_y_nm + V + E, n);\n"
            "                }\n"
            "            }\n"
            "        }",
            "/* mutant: omitted core VIA23 cut */\n"
            "                    ele_fab_emit_poly(ws, ELE_LAYER_M3, 0, x_col - E, net->trunk_y_nm - E,\n"
            "                                      x_col + V + E, net->trunk_y_nm + V + E, n);\n"
            "                }\n"
            "            }\n"
            "        }",
        ),
        FabMutant(
            "LVS_M3_TRUNK_SHORT",
            "lvs_open_short_pad",
            "net->trunk_y_track = best_trk;\n        net->trunk_y_nm    = global_y_tracks[best_trk];",
            "net->trunk_y_track = 0;\n        net->trunk_y_nm    = global_y_tracks[0];",
        ),
        FabMutant(
            "LVS_M1_PIN_STUB_DISCONNECT",
            "lvs_open_short_pad",
            "ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, stub_x0, y_stub, stub_x1, y_stub + W, n);",
            "ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, stub_x0, y_stub + 2 * P, stub_x1, y_stub + 2 * P + W, n);",
        ),
        FabMutant(
            "LVS_PAD_BRIDGE_OMIT_VIA12",
            "lvs_open_short_pad",
            "ele_fab_emit_poly(ws, ELE_LAYER_VIA12, 0, px, y_jog, px + V, y_jog + V, n);",
            "/* mutant: omitted pad VIA12 bridge */",
        ),
        FabMutant(
            "LVS_PAD_SHARED_SHORT",
            "lvs_open_short_pad",
            "ele_fab_add_pin_to_net(ws, n, ELE_FAB_BASE_CELLS + n, 0);\n        } else if (n < 46) {",
            "ele_fab_add_pin_to_net(ws, n, ELE_FAB_BASE_CELLS, 0);\n        } else if (n < 46) {",
        ),
        # --- Category 3: Univalent Co-Evolution, Abacus Legalization & .flow Coupling (6 mutants) ---
        FabMutant(
            "UNIV_DISABLE_SPLIT_PASS2",
            "univalent_coevolution",
            "uint8_t sm = ele_fab_decode_genome(cand, true, perm_row, target_x);",
            "uint8_t sm = ele_fab_decode_genome(cand, false, perm_row, target_x);",
        ),
        FabMutant(
            "UNIV_ZERO_WIRE_RC_ELMORE",
            "univalent_coevolution",
            "int64_t rc_prod = (int64_t)pdk->r_sheet_mohm * (int64_t)pdk->c_unit_af;",
            "int64_t rc_prod = 0;",
        ),
        FabMutant(
            "UNIV_ABACUS_DROP_CURSOR_PUSH",
            "univalent_coevolution",
            "int32_t col = (pref_col > cursor_col) ? pref_col : cursor_col;",
            "int32_t col = (cursor_col < 0) ? cursor_col : pref_col;",
        ),
        FabMutant(
            "UNIV_PDK_CORRUPT_RSHEET_DECODE",
            "univalent_coevolution",
            "out->r_sheet_mohm = (int32_t)abi.r_sheet_mohm;",
            "out->r_sheet_mohm = (int32_t)(abi.r_sheet_mohm / 10000);",
        ),
        FabMutant(
            "UNIV_MONOTONIC_SWAP_DROP",
            "univalent_coevolution",
            "if (sres2.best_energy < sres.best_energy) {\n            sres = sres2;\n        }",
            "if (sres2.best_energy > sres.best_energy) {\n            sres = sres2;\n        }",
        ),
        FabMutant(
            "UNIV_REPEATER_FANOUT_ALIAS_SHORT",
            "univalent_coevolution",
            "orig->split_active      = 1U;\n"
            "        orig->pin_node[far_idx] = rep_id;\n"
            "        orig->pin_slot[far_idx] = 0;",
            "orig->split_active      = 1U;\n"
            "        orig->pin_node[far_idx] = far_node;\n"
            "        orig->pin_slot[far_idx] = far_slot;",
        ),
        # --- Category 4: 2-Jet Optical Diffraction & Inverse Lithography OPC/ILT (5 mutants) ---
        FabMutant(
            "OPC_ZERO_LEFT_EXTENSION",
            "litho_2jet_opc",
            "le->x0 - ext_len, sy0, le->x0 + pdk->grid_nm, sy1, le->net_id);",
            "le->x0, sy0, le->x0 + pdk->grid_nm, sy1, le->net_id);",
        ),
        FabMutant(
            "OPC_INVERT_RIGHT_SERIF",
            "litho_2jet_opc",
            "le->x1 - pdk->grid_nm, sy0, le->x1 + ext_len, sy1, le->net_id);",
            "le->x1 - ext_len, sy0, le->x1, sy1, le->net_id);",
        ),
        FabMutant(
            "OPC_MRC_BRIDGING_VIOLATION",
            "litho_2jet_opc",
            "if (ext_len > pdk->min_space_nm) {\n            ext_len = ele_fab_snap_down(pdk->min_space_nm, pdk->grid_nm);\n        }",
            "ext_len = 2 * pdk->pitch_nm;",
        ),
        FabMutant(
            "OPC_ZERO_CHART3_FLOW_BIAS",
            "litho_2jet_opc",
            "int32_t bias_raw = (int32_t)(((int64_t)st[3] * (int64_t)sigma_nm) >> 16);",
            "int32_t bias_raw = 0;",
        ),
        FabMutant(
            "OPC_WRONG_SERIF_LAYER",
            "litho_2jet_opc",
            "ele_fab_emit_poly(ws, ELE_LAYER_OPC_SERIF, 1,\n                              le->x0 - ext_len,",
            "ele_fab_emit_poly(ws, ELE_LAYER_HV_GUARD, 1,\n                              le->x0 - ext_len,",
        ),
        # --- Category 5: Binary GDSII Stream Format & Framing Integrity (5 mutants) ---
        FabMutant(
            "GDS_BAD_HEADER_VERSION",
            "gdsii_binary_framing",
            "ele_fab_write_be16(b, &pos, 0x0002U);\n    ele_fab_write_be16(b, &pos, 0x0258U);",
            "ele_fab_write_be16(b, &pos, 0x0002U);\n    ele_fab_write_be16(b, &pos, 0x0257U);",
        ),
        FabMutant(
            "GDS_CORRUPT_IBM_REAL8_UNITS",
            "gdsii_binary_framing",
            "ele_fab_write_be64(b, &pos, UINT64_C(0x3E4189374BC6A7F0));",
            "ele_fab_write_be64(b, &pos, UINT64_C(0x3F4189374BC6A7F0));",
        ),
        FabMutant(
            "GDS_UNCLOSED_BOUNDARY_LOOP",
            "gdsii_binary_framing",
            "ele_fab_write_be32(b, &pos, p->y0);\n        ele_fab_write_be16(b, &pos, 4U);\n        ele_fab_write_be16(b, &pos, 0x1100U); /* ENDEL */",
            "ele_fab_write_be32(b, &pos, p->y1);\n        ele_fab_write_be16(b, &pos, 4U);\n        ele_fab_write_be16(b, &pos, 0x1100U); /* ENDEL */",
        ),
        FabMutant(
            "GDS_REVERSED_BOUNDING_BOX",
            "gdsii_binary_framing",
            "ele_fab_write_be16(b, &pos, 0x1003U); /* XY (5 closed vertices) */\n"
            "        ele_fab_write_be32(b, &pos, p->x0);",
            "ele_fab_write_be16(b, &pos, 0x1003U); /* XY (5 closed vertices) */\n"
            "        ele_fab_write_be32(b, &pos, p->x1);",
        ),
        FabMutant(
            "GDS_BAD_ENDLIB_OPCODE",
            "gdsii_binary_framing",
            "ele_fab_write_be16(b, &pos, 0x0400U); /* ENDLIB */",
            "ele_fab_write_be16(b, &pos, 0x0401U); /* ENDLIB */",
        ),
    ]


def eval_fab_mutant(
    m: FabMutant,
    golden_hdr: str,
    root: Path,
    flow_dir: Path,
    cc: str,
    env: dict[str, str],
    tmpdir: Path,
    idx: int,
) -> tuple[FabMutant, bool, bool, str]:
    workdir = tmpdir / f"mut_{idx:02d}"
    workdir.mkdir(parents=True, exist_ok=True)
    inc_dir = workdir / "tools"
    inc_dir.mkdir(exist_ok=True)
    pdk_dir = workdir / "pdk"
    pdk_dir.mkdir(exist_ok=True)

    mut_hdr = golden_hdr.replace(m.target, m.replacement, 1)
    (inc_dir / "ele_fab.h").write_text(mut_hdr, encoding="utf-8")
    shutil.copy2(root / "build" / "ele_pdk_abi.h", inc_dir / "ele_pdk_abi.h")
    boot_copy = inc_dir / "ele_bootstrap.c"
    shutil.copy2(root / "tools" / "ele_bootstrap.c", boot_copy)
    for pdk_file in (root / "build").glob("pdk_*.flow"):
        shutil.copy2(pdk_file, pdk_dir / pdk_file.name)

    bin_path = workdir / "ele_bootstrap_mut"
    compile_cmd = [
        cc,
        "-std=c17",
        "-mgeneral-regs-only",
        "-fno-jump-tables",
        "-fsanitize=signed-integer-overflow,shift-exponent,bounds,null,return,unreachable",
        "-fsanitize-trap=all",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-Werror",
        f"-I{flow_dir}",
        f"-I{inc_dir}",
        str(boot_copy),
        "-o",
        str(bin_path),
    ]
    c_proc = subprocess.run(compile_cmd, capture_output=True, text=True, env=env)
    if c_proc.returncode != 0:
        return (m, False, False, (c_proc.stderr or c_proc.stdout)[-600:])

    r_proc = subprocess.run([str(bin_path), "--fab-verify", str(pdk_dir)], capture_output=True, text=True, env=env)
    killed = r_proc.returncode != 0
    detail = f"rc={r_proc.returncode} out={(r_proc.stdout + r_proc.stderr)[-300:]}"
    return (m, True, killed, detail)


def main() -> int:
    t0 = time.monotonic()
    root = Path(__file__).resolve().parent.parent
    flow_dir = (root / ".." / "FLOW").resolve()
    env = os.environ.copy()
    for mk in ("MAKEFLAGS", "MFLAGS", "MAKELEVEL"):
        env.pop(mk, None)
    env["PATH"] = f"/opt/homebrew/opt/llvm/bin:/opt/homebrew/bin:/usr/local/bin:{env.get('PATH', '')}"

    cc = shutil.which("clang", path=env["PATH"]) or "clang"
    boot_bin = root / "build" / "ele_bootstrap"
    build_dir = root / "build"
    if not boot_bin.exists() or not (build_dir / "pdk_euv7.flow").exists():
        print("ERROR: Missing build/ele_bootstrap or PDK .flow files; run `make bootstrap-gate` first.", file=sys.stderr)
        return 1

    base = subprocess.run([str(boot_bin), "--fab-verify", str(build_dir)], capture_output=True, text=True, env=env)
    if base.returncode != 0:
        print("ERROR: Unmutated golden --fab-verify failed!", file=sys.stderr)
        print(base.stdout + base.stderr, file=sys.stderr)
        return 1

    hdr_path = root / "tools" / "ele_fab.h"
    golden_hdr = hdr_path.read_text(encoding="utf-8")
    mutants = build_fab_mutants()

    seen: set[str] = set()
    for m in mutants:
        if m.name in seen:
            raise RuntimeError(f"Duplicate mutant name: {m.name}")
        seen.add(m.name)
        cnt = golden_hdr.count(m.target)
        if cnt != 1:
            raise RuntimeError(f"Mutant {m.name} target matched {cnt} times (expected 1): {m.target!r}")
        if m.target == m.replacement:
            raise RuntimeError(f"Mutant {m.name} replacement matches target!")

    print("================================================================================")
    print("  ELE v8.0.0 — Adversarial HoTT/Fab/DRC/LVS/OPC/GDSII Single-Fault Sabotage")
    print("================================================================================")
    print("  golden_fab_status      : PASS (sky130 + euv7 + sic800v verified cleanly)")
    print(f"  adversarial_fab_mutants: {len(mutants)} single-fault C17/Fab mutants across 5 categories", flush=True)

    tmpdir = Path(tempfile.mkdtemp(prefix="ele_fab_sabotage_"))
    compile_errors: list[tuple[FabMutant, str]] = []
    survivors: list[FabMutant] = []
    by_cat: dict[str, list[bool]] = {}

    try:
        workers = min(8, os.cpu_count() or 4)
        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
            futs = [
                pool.submit(eval_fab_mutant, m, golden_hdr, root, flow_dir, cc, env, tmpdir, idx)
                for idx, m in enumerate(mutants)
            ]
            done = 0
            for fut in concurrent.futures.as_completed(futs):
                m, compiled_ok, killed, detail = fut.result()
                done += 1
                by_cat.setdefault(m.category, []).append(compiled_ok and killed)
                if done % 14 == 0 or done == len(mutants):
                    print(f"  [progress] {done}/{len(mutants)} fab mutants audited...", flush=True)
                if not compiled_ok:
                    compile_errors.append((m, detail))
                elif not killed:
                    survivors.append(m)
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    for cat in sorted(by_cat):
        res = by_cat[cat]
        print(f"  cat_{cat:<25}: {sum(res):2d} / {len(res):2d} killed dynamically")

    killed_total = len(mutants) - len(compile_errors) - len(survivors)
    pct = (100.0 * killed_total / len(mutants)) if mutants else 0.0
    elapsed = time.monotonic() - t0
    print(f"\n  fab_mutant_kill_rate   : {killed_total} / {len(mutants)} ({pct:.1f}%) in {elapsed:.1f}s")

    if compile_errors:
        print("================================================================================")
        print("  [FAB MUTATION AUDIT ERROR] MUTANTS FAILED TO COMPILE:")
        for m, det in compile_errors:
            print(f"    - {m.name}: {det}")
        print("================================================================================")
        return 1

    if survivors:
        print("================================================================================")
        print("  [FAB MUTATION AUDIT FAIL] SURVIVING FAB MUTANTS DETECTED:")
        for s in survivors:
            print(f"    - {s.name} [{s.category}]: {s.target!r} -> {s.replacement!r}")
        print("================================================================================")
        return 1

    print("================================================================================")
    print("  [FAB MUTATION AUDIT PASS] 100.0% DYNAMIC KILL RATE — ZERO SURVIVING MUTANTS")
    print("================================================================================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
