import Ele

open Ele
open Ele.Electron
open Ele.Power
open Ele.MagWave
open Ele.Silicon

/-! ## 1. Compile-Time Kernel Checks (`0 sorryAx`) across Electron / Power / MagWave -/

example : fllScaledStretchNum 200 400 = 320000 := by decide
example : canaryCheck 950 80 1000 = (true, false) := by decide
example : levelCrossingFires 14 10 5 = false ∧ levelCrossingFires 15 10 5 = true := by decide

example : sscbJet2FaultMatch 180 120 45 30 (-15) (-5) = false := by decide
example : sscbJet2FaultMatch 180 120 45 30 2 (-5) = true := by decide
example : coldplateIsDryout 1200 10 200 700 = false := by decide

example : regRmw8 0xA5 0x0F 0x3C = 0xAC := by decide
example : busInvertWireFlips 64 = 1 ∧ busInvertWireFlips 33 = 32 ∧ busInvertWireFlips 32 = 32 := by decide
example : cpoModuloFsrDetune 1850 600 = 50 := by decide

private def assertCond (cond : Bool) (msg : String) : IO Unit := do
  if !cond then
    throw (IO.userError s!"[ASSERT FAIL] {msg}")

def main : IO UInt32 := do
  let workDir : System.FilePath := "build/capsules"
  IO.FS.createDirAll workDir

  -- ==========================================================================
  -- 1. Emit Sealed `.flow` Manifold Atlases & Hot-Swap Policy Cells via `Unit`
  -- ==========================================================================
  let efllBundle := workDir / "efll_atlas.flow"
  let sscbBundle := workDir / "sscb_atlas.flow"
  let egfmBundle := workDir / "egfm_atlas.flow"
  let ecpoBundle := workDir / "ecpo_atlas.flow"

  fllAtlas.emitBundle efllBundle 1
  sscbAtlas.emitBundle sscbBundle 1
  gfmAtlas.emitBundle egfmBundle 1
  cpoAtlas.emitBundle ecpoBundle 1

  let _ ← fllAtlas.verifyBundle efllBundle
  let _ ← sscbAtlas.verifyBundle sscbBundle
  let _ ← gfmAtlas.verifyBundle egfmBundle
  let _ ← cpoAtlas.verifyBundle ecpoBundle

  -- Mint tightening AD-FLL stretch cell (`quadCurv = 0.04`) & foreign SSCB cell
  let efllTightCell := workDir / "efll_stretch_v2.flow"
  let tightStretch :=
    Unit.throttleQuad FllRegime.stretch (q16! "1.0") (q16! "0.04") (qty! "35.0" "mV")
      |> Unit.Regime.deadband (qty! "15.0" "mV") (qty! "55.0" "mV") .nominal .emergency
      |> Unit.Regime.lockFlags [FllFlag.canaryArmed]
  fllAtlas.emitPolicy efllTightCell tightStretch 1

  let foreignSscbCell := workDir / "foreign_sscb.flow"
  sscbAtlas.emitPolicy foreignSscbCell (sscbAtlas.law .millerClamp) 1

  IO.println "  [PASS] Minted & FLOW-verified 4 EEE Manifold Atlases (EFLL, SSCB, EGFM, ECPO)"

  -- ==========================================================================
  -- 2. Untrusted UTF-8 Wire Ingress + Domain Lifecycle Tests (`Unit` -> `FLOW`)
  -- ==========================================================================
  let goodEfllWire :=
    s!"# AD-FLL droop telemetry (mV) + mode flags\n" ++
    s!"u=24.0mV flags=resonant|canary\n" ++
    s!"u=34.0mV flags=resonant|canary|sscg\n" ++
    s!"u=45.0mV flags=resonant|canary\n" ++
    s!"@swap stretch 2 {efllTightCell}\n" ++
    s!"@swap stretch 2 {efllTightCell}\n" ++
    s!"@swap stretch 3 {foreignSscbCell}\n" ++
    s!"u=45.0mV flags=resonant|canary\n" ++
    s!"u=120.0mV flags=resonant|canary\n" ++
    s!"u=200.0mV flags=resonant|canary\n" ++
    s!"u=200.0mV flags=resonant|canary\n"
  let badOverlong : ByteArray := ByteArray.mk #[0xC0, 0xAF, 0x0A]
  let wrongUnit   : ByteArray := "25.0A, canary\n".toUTF8
  let efllWireRaw := goodEfllWire.toUTF8 ++ badOverlong ++ wrongUnit

  let efllTrace ← fllAtlas.runWireBytes efllBundle efllWireRaw .auto
  assertCond (efllTrace.rejectedIngress == 2)
    s!"expected 2 rejected wire frames, got {efllTrace.rejectedIngress}"
  assertCond (efllTrace.swaps.size == 3) s!"expected 3 swaps, got {efllTrace.swaps.size}"
  assertCond (efllTrace.swaps[0]!.accepted == true)  "EFLL swap epoch=2 should be accepted"
  assertCond (efllTrace.swaps[1]!.accepted == false) "EFLL replay epoch=2 should be rejected"
  assertCond (efllTrace.swaps[2]!.accepted == false) "Foreign SSCB swap into EFLL must be rejected"
  assertCond (efllTrace.steps[1]!.regime == .stretch) "EFLL step 2 should escalate to .stretch"
  -- Tighter quadratic curvature (`0.04` vs `0.02`) yields strictly smaller `x` at `u=45mV`
  assertCond (efllTrace.steps[3]!.x < efllTrace.steps[2]!.x)
    "Tightened AD-FLL stretch policy must clamp droop budget tighter"
  assertCond (efllTrace.steps[5]!.regime == .emergency)
    "Sustained droop surge must transition AD-FLL into .emergency"

  -- 800V SiC SSCB short-circuit trip & `0.0000A` isolation test
  let sscbTrace ← sscbAtlas.runEvents sscbBundle [
    .obs (qty! "125.0" "A") none,
    .obs (qty! "240.0" "A") none,
    .obs (qty! "600.0" "A") none,
    .obs (qty! "900.0" "A") none,
    .obs (qty! "900.0" "A") none,
    .obs (qty! "900.0" "A") none
  ]
  assertCond (sscbTrace.steps[0]!.regime == .millerClamp) "SSCB must enter Miller clamp > 120A"
  assertCond (sscbTrace.steps[2]!.regime == .tripped)     "SSCB must trip > 165A"
  assertCond (sscbTrace.steps[5]!.x == qty! "0.0" "A")
    s!"Tripped SiC SSCB must isolate 900A fault to 0.0000A, got {sscbTrace.steps[5]!.x.format true}"

  IO.println "  [PASS] Verified UTF-8 wire sieve, Kan locks & EEE Atlases on FLOW/build/flow"

  -- ==========================================================================
  -- 3. Pure-SV Hardware EM PHY Self-Test (`ele_em_phy.sv` via Verilator)
  -- ==========================================================================
  let phySummary ← verifyEmPhyRtl
  IO.println s!"  [PASS] Synthesizable EM PHY RTL verified: {phySummary}"

  -- ==========================================================================
  -- 4. 3-Way `0-ULP` 512-Bit (`w0..w7`) Silicon Co-Verification (`ele_flow_core.sv`)
  -- ==========================================================================
  let nEfll ← verifyAtlasCosim "efll" efllBundle [
    .step (q16! "24.0") (some 0x0003),
    .step (q16! "34.0") (some 0x0007),
    .step (q16! "45.0") (some 0x0003),
    -- Attempt illegal unlock of `canaryArmed` (`bit 1`) while in `.stretch`
    .step (q16! "45.0") (some 0x0001),
    .swap 1 2 efllTightCell,
    .swap 1 2 efllTightCell,
    .swap 1 3 foreignSscbCell,
    .step (q16! "45.0") none,
    .step (q16! "120.0") none,
    .step (q16! "200.0") none,
    -- Attempt illegal Kan flip inside `.emergency` (`requireHomotopic` blocks commit!)
    .step (q16! "200.0") (some 0x0000),
    .step (q16! "200.0") none
  ]

  let nSscb ← verifyAtlasCosim "sscb" sscbBundle [
    .step (q16! "80.0")  none,
    .step (q16! "125.0") none,
    .step (q16! "240.0") none,
    .step (q16! "600.0") none,
    .step (q16! "900.0") none,
    .step (q16! "900.0") none,
    .step (q16! "900.0") none
  ]

  let nEgfm ← verifyAtlasCosim "egfm" egfmBundle [
    .step (q16! "60.0")  none,
    .step (q16! "160.0") none,
    .step (q16! "290.0") none,
    .step (q16! "480.0") none,
    .step (q16! "480.0") none,
    .step (q16! "480.0") none,
    .step (q16! "40.0")  none
  ]

  -- Includes `.fsrTunnel` (`Delta < 0` curvature tunneling in silicon!) -> `.laserSafe`
  let nEcpo ← verifyAtlasCosim "ecpo" ecpoBundle [
    .step (q16! "32.0")  none,
    .step (q16! "45.0")  none,
    .step (q16! "260.0") none,
    .step (q16! "300.0") none,
    .step (q16! "0.0")   none,
    .step (q16! "0.0")   none,
    .step (q16! "0.0")   none,
    .step (q16! "10.0")  none
  ]

  -- Upstream FLOW canonical 4-Chart fixture + 2-Chart autonomous fiber/instanton bundle
  Flow.emitCoreTestFixtures (workDir / "flow_core").toString
  let fiberBundle := workDir / "flow_core" / "fiber_atlas2.flow"
  let sFiber := Flow.mintState Flow.chartZeroTarget 0 (initChart := 0) (initMode := 0)
    (2 * Flow.q16One) (-3 * Flow.q16One) (2 * Flow.q16One)
  Flow.emitBundle fiberBundle 2 sFiber
    [Flow.chartZeroTarget, Flow.chartInstanton] Flow.chartZeroTarget 1

  let sInst := Flow.mintState Flow.chartInstanton 0 (initChart := 1) (initMode := 0)
  let instBundle := workDir / "flow_core" / "instanton_atlas2.flow"
  Flow.emitBundle instBundle 2 sInst
    [Flow.chartZeroTarget, Flow.chartInstanton] Flow.chartInstanton 1

  let flowCoreBundle := workDir / "flow_core" / "core_atlas4.flow"
  let flowHarborCell := workDir / "flow_core" / "swap_harbor.flow"
  let flowZeroCell   := workDir / "flow_core" / "zero_target.flow"
  let flowForeign    := workDir / "flow_core" / "foreign_chart.flow"
  let nCore ← verifyAtlasCosim "flow_core4" flowCoreBundle [
    .step (q16! "8.0")  none,
    .step (q16! "16.0") none,
    .step (q16! "21.0") (some 0x0007),
    .step (q16! "200.0") none,
    .step (q16! "200.0") (some 0x0000),
    .step (q16! "200.0") none,
    .swap 1 2 flowHarborCell,
    .swap 0 2 flowZeroCell,
    .swap 1 3 flowForeign,
    .step (q16! "50.0") none
  ]
  let nFiber ← verifyAtlasCosim "flow_fiber2" fiberBundle [
    .step (q16! "0.0") none,
    .step (q16! "0.0") none,
    .step (q16! "0.0") none
  ]
  let nInst ← verifyAtlasCosim "flow_instanton2" instBundle [
    .step (q16! "0.0") none,
    .step (q16! "0.0") none,
    .step (q16! "0.0") none
  ]

  let totalSteps := nEfll + nSscb + nEgfm + nEcpo + nCore + nFiber + nInst
  IO.println s!"  [PASS] 3-Way 0-ULP 512-bit Co-Sim (Lean == FLOW C99 == Verilator SV): {totalSteps} steps, 0 ULP error!"
  return 0
