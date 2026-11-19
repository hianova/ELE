/-
  ELE v8.0.0 — Pure Silicon Parameter Config (`Flow.Core`/`Schema`/`Morphism`/`Canonical`, 0 Custom Theorems)
  ===============================================================================
  SSOT compliance: all formal fiber-bundle/lattice/convex/ABI/Morphism theorems
  and compilers live in domain-free `../FLOW/{core,sdk}/lean/Flow/*.lean`.
  This file configures foundry `PdkSpec` values, `ele_pdk_abi`, and ELE circuits.
-/
import Flow.Core
import Flow.Schema
import Flow.Morphism
import Flow.Canonical

set_option maxRecDepth 8192

namespace Ele

structure PdkSpec where
  name       : String
  gridNm     : Int
  pitchNm    : Int
  minWidthNm : Int
  minSpaceNm : Int
  viaSizeNm  : Int
  viaEncNm   : Int
  hvSpaceNm  : Int
  rSheetMohm : Int
  cUnitAf    : Int
  lambdaNm   : Int
  opcGainQ16 : Int
  kanLock    : UInt16
  deriving Repr, DecidableEq

def PdkSpec.padSpan (m : PdkSpec) : Int := m.viaSizeNm + 2 * m.viaEncNm

def PdkSpec.wf (m : PdkSpec) : Bool :=
  (0 < m.gridNm) && (m.gridNm ≤ m.minWidthNm) && (0 < m.minSpaceNm) &&
  (0 < m.viaSizeNm) && (0 ≤ m.viaEncNm) && (m.minWidthNm ≤ m.padSpan) &&
  (m.padSpan + m.minSpaceNm ≤ m.pitchNm) && (m.padSpan + m.hvSpaceNm ≤ 2 * m.pitchNm) &&
  (m.pitchNm % m.gridNm == 0) && (m.minWidthNm % m.gridNm == 0) &&
  (m.minSpaceNm % m.gridNm == 0) && (m.viaSizeNm % m.gridNm == 0) &&
  (m.viaEncNm % m.gridNm == 0) && (0 < m.rSheetMohm) && (0 < m.cUnitAf) &&
  (0 < m.lambdaNm) && (0 ≤ m.opcGainQ16 && m.opcGainQ16 ≤ Flow.maxQ16) &&
  (m.pitchNm * Flow.q16One ≤ Flow.maxQ16)

def pdkSchema : Flow.Schema.Spec where
  cPrefix     := "ele_pdk_abi"
  headerGuard := "ELE_PDK_ABI_H"
  kExp        := 3
  tagChart    := 4
  fields      := [
    { name := "grid_nm",      chartSlot := 4, lane := 0, minVal := 1, maxVal := 1000 },
    { name := "pitch_nm",     chartSlot := 4, lane := 1, minVal := 1, maxVal := 100000 },
    { name := "min_width_nm", chartSlot := 4, lane := 2, minVal := 1, maxVal := 100000 },
    { name := "min_space_nm", chartSlot := 5, lane := 0, minVal := 1, maxVal := 100000 },
    { name := "via_size_nm",  chartSlot := 5, lane := 1, minVal := 1, maxVal := 100000 },
    { name := "via_enc_nm",   chartSlot := 5, lane := 2, minVal := 0, maxVal := 100000 },
    { name := "hv_space_nm",  chartSlot := 6, lane := 0, minVal := 0, maxVal := 100000 },
    { name := "r_sheet_mohm", chartSlot := 6, lane := 1, minVal := 1, maxVal := 1000000 },
    { name := "c_unit_af",    chartSlot := 6, lane := 2, minVal := 1, maxVal := 1000000 },
    { name := "lambda_nm",    chartSlot := 7, lane := 0, minVal := 1, maxVal := 100000 },
    { name := "opc_gain_q16", chartSlot := 7, lane := 1, minVal := 0, maxVal := Flow.maxQ16 },
    { name := "kan_lock",     chartSlot := 7, lane := 2, minVal := 0, maxVal := 65535 }
  ]

def pdkSky130 : PdkSpec :=
  { name := "sky130", gridNm := 10, pitchNm := 460, minWidthNm := 140, minSpaceNm := 140,
    viaSizeNm := 150, viaEncNm := 60, hvSpaceNm := 140, rSheetMohm := 125, cUnitAf := 180,
    lambdaNm := 193, opcGainQ16 := 32768, kanLock := 0x0130 }

def pdkEuv7 : PdkSpec :=
  { name := "euv7", gridNm := 2, pitchNm := 48, minWidthNm := 18, minSpaceNm := 18,
    viaSizeNm := 18, viaEncNm := 6, hvSpaceNm := 18, rSheetMohm := 1850, cUnitAf := 240,
    lambdaNm := 14, opcGainQ16 := 49152, kanLock := 0x0007 }

def pdkSic800v : PdkSpec :=
  { name := "sic800v", gridNm := 20, pitchNm := 1600, minWidthNm := 600, minSpaceNm := 400,
    viaSizeNm := 600, viaEncNm := 200, hvSpaceNm := 1200, rSheetMohm := 40, cUnitAf := 120,
    lambdaNm := 248, opcGainQ16 := 28672, kanLock := 0x0800 }

def emitPdkAtlas (path : System.FilePath) (m : PdkSpec) : IO Bool := do
  if !m.wf then return false
  let q1 := Flow.q16One
  let lDt := if m.minSpaceNm / 2 > 0 then (m.minSpaceNm / 2) * q1 else q1
  let c0 : Flow.Rule := { b0 := m.gridNm * q1, b1 := q1, b2 := m.hvSpaceNm * 16, bExit := -5000 * q1, bEnter := 5000 * q1, dtMax := 500 * q1, uGain := q1, nextLo := 0, nextHi := 1, kanLock := m.kanLock }
  let c1 : Flow.Rule := { b0 := 0, b1 := m.rSheetMohm * 32, b2 := m.cUnitAf * 64, bExit := -5000 * q1, bEnter := 5000 * q1, dtMax := 500 * q1, uGain := q1, nextLo := 0, nextHi := 2, kanLock := m.kanLock }
  let c2 : Flow.Rule := { b0 := 0, b1 := q1, b2 := m.rSheetMohm * 16, bExit := -2000 * q1, bEnter := 2000 * q1, dtMax := 250 * q1, uGain := q1, nextLo := 1, nextHi := 3, kanLock := m.kanLock }
  let c3 : Flow.Rule := { b0 := 0, b1 := m.opcGainQ16, b2 := m.lambdaNm * 128, bExit := -4000 * q1, bEnter := 4000 * q1, dtMax := lDt, uGain := m.opcGainQ16, nextLo := 2, nextHi := 3, kanLock := m.kanLock }
  let fb : Flow.Rule := { b0 := 0, b1 := q1, b2 := 0, bExit := -5000 * q1, bEnter := 5000 * q1, dtMax := 500 * q1, uGain := q1, nextLo := 0, nextHi := 0, kanLock := m.kanLock }
  let st := Flow.mintState 3 (m.pitchNm * q1) (initChart := 0) (initMode := m.kanLock)
    (j0Q16 := m.minWidthNm * q1) (j1Q16 := m.minSpaceNm * q1)
  let vals : List (String × Int) := [
    ("grid_nm", m.gridNm), ("pitch_nm", m.pitchNm), ("min_width_nm", m.minWidthNm),
    ("min_space_nm", m.minSpaceNm), ("via_size_nm", m.viaSizeNm), ("via_enc_nm", m.viaEncNm),
    ("hv_space_nm", m.hvSpaceNm), ("r_sheet_mohm", m.rSheetMohm), ("c_unit_af", m.cUnitAf),
    ("lambda_nm", m.lambdaNm), ("opc_gain_q16", m.opcGainQ16), ("kan_lock", Int.ofNat m.kanLock.toNat)]
  match pdkSchema.packBundle st [c0, c1, c2, c3] fb vals 1 with
  | none => return false
  | some bytes => IO.FS.writeBinFile path bytes; return true

def flowCoreMorphism : Flow.Morphism.Graph :=
  let specs : List (String × Nat × Nat × Nat × Nat × Nat × Nat × List Nat) := [
    ("a0_ug_u", 0, 0, 1, 0, 17, 64, []), ("a1_b1_x", 0, 1, 1, 1, 17, 64, []),
    ("a2_x_sq", 0, 2, 1, 2, 17, 64, []), ("a3_b2_x", 0, 3, 1, 3, 17, 64, []),
    ("a4_b0_eff", 0, 4, 2, 4, 10, 128, [1]), ("a5_b2xsq_lo", 0, 5, 1, 5, 17, 66, [3]),
    ("a6_b2xsq_hi", 0, 6, 1, 6, 17, 66, [3]), ("a7_g0_op", 0, 7, 2, 7, 13, 128, [2, 5, 6, 7]),
    ("a8_g1_op", 0, 8, 2, 8, 8, 64, [4]), ("a9_jet_au", 0, 9, 2, 9, 11, 192, [5]),
    ("a10_jet_mux", 0, 10, 2, 10, 5, 0, [5, 8, 9, 10]),
    ("b0_g1_ll", 1, 0, 1, 11, 17, 66, []), ("b1_g1_hl", 1, 1, 1, 12, 17, 66, []),
    ("b2_g1_hh", 1, 2, 1, 13, 17, 66, []), ("b3_g0r2_0", 1, 3, 1, 14, 17, 66, []),
    ("b4_g0r2_1", 1, 4, 1, 15, 17, 66, []), ("b5_g0r2_2", 1, 5, 1, 16, 17, 66, []),
    ("b6_g1_sq", 1, 6, 2, 17, 12, 128, [12, 13, 14]), ("b7_g0_r2", 1, 7, 2, 18, 12, 128, [15, 16, 17]),
    ("b8_disc_sub", 1, 8, 2, 19, 11, 0, [18, 19]),
    ("d0_cit_sel", 2, 0, 2, 20, 14, 194, []), ("d1_div_norm", 2, 1, 2, 21, 14, 0, [21]),
    ("c0_delta_cand", 3, 0, 0, 22, 13, 33, []), ("c1_d_sq", 3, 1, 1, 23, 17, 64, [23]),
    ("c2_g1dc_lo", 3, 2, 1, 24, 17, 66, [23]), ("c3_g1dc_hi", 3, 3, 1, 25, 17, 66, [23]),
    ("c4_r2_dc", 3, 4, 1, 26, 17, 64, [23]), ("c5_r2dsq_lo", 3, 5, 1, 27, 17, 66, [24]),
    ("c6_r2dsq_hi", 3, 6, 1, 28, 17, 66, [24]), ("c7_g0_cand", 3, 7, 2, 29, 13, 128, [25, 26, 28, 29]),
    ("c8_lyap_guard", 3, 8, 2, 30, 14, 288, [23, 27, 30]), ("c9_post_round", 3, 9, 2, 31, 13, 192, [31]),
    ("c10_kan_commit", 3, 10, 2, 32, 7, 0, [31, 32])
  ]
  let b0 := (Flow.Morphism.Builder.init "ele_flow_core_ssa" .cartesian false).input "state_in" 512 |>.1
  let bFin := specs.foldl (fun b sp =>
    let w := if sp.2.2.2.2.2.2.1 == 0 then 64 else sp.2.2.2.2.2.2.1
    let p0 := UInt64.ofNat (sp.2.1 ||| (sp.2.2.1 <<< 8) ||| (sp.2.2.2.1 <<< 16) ||| (sp.2.2.2.2.1 <<< 24))
    (b.stagePipe sp.1 w sp.2.2.2.2.2.2.2 p0 (UInt64.ofNat sp.2.2.2.2.2.1) (UInt64.ofNat sp.2.2.2.2.2.2.1)).1) b0
  bFin.build [11, 20, 22, 33]

def main (args : List String) : IO UInt32 := do
  let outDir : System.FilePath := match args with
    | d :: _ => System.FilePath.mk d
    | []     => System.FilePath.mk "build"
  IO.FS.createDirAll outDir
  if !(← pdkSchema.emitAbiHeaderChecked (outDir / "ele_pdk_abi.h")) then return 1
  for m in [pdkSky130, pdkEuv7, pdkSic800v] do
    if !(← emitPdkAtlas (outDir / s!"pdk_{m.name}.flow") m) then
      IO.eprintln s!"FATAL: PDK {m.name} failed PdkSpec.wf"
      return 1
  if !(← Flow.Morphism.emitFirChecked (outDir / "ele_core_ir.fir") flowCoreMorphism) then return 1
  if !(← Flow.Morphism.emitFirChecked (outDir / "gf256_sbox_mac.fir") Flow.Canonical.gf256SboxMac) then return 1
  if !(← Flow.Morphism.emitFirChecked (outDir / "gf256_tower_spec.fir") Flow.Canonical.gf256TowerSpec) then return 1
  if !(← Flow.Morphism.emitFirChecked (outDir / "lut5_bijection_spec.fir") Flow.Canonical.lut5BijectionSpec) then return 1
  IO.println s!"  [ELE-LEAN] Pure config emitted 3x .flow PDKs, ele_pdk_abi.h & 4x .fir graphs into {outDir} (0 local theorems)"
  return 0

end Ele

def main (args : List String) : IO UInt32 :=
  Ele.main args
