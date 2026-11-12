import Unit

/-
  Ele.lean — Verified Electronics, Electrical Power & Electromagnetic Silicon Layer
  =================================================================================
  Pure Lean 4 (`0` C files, `0` headers, `0 sorryAx`) sitting cleanly atop
  untouched `FLOW v5.0.0` and `UNIT v2.0.0`, paired with two reusable IEEE 1800
  SystemVerilog hardware IPs (`rtl/ele_flow_core.sv` & `rtl/ele_em_phy.sv`).

  Domain Pillars (`ELE` = 電子 · 電機 · 電磁 + Silicon Co-Verification):
    1. `Ele.Electron` (電子): <1ns AD-FLL quadratic zero-SDC clock stretching,
       CMOS `C * V_dd^2` guardband elimination, LC `Q > pi` resonant clock tank,
       asynchronous level-crossing hysteresis & sub-gate Vernier TDC, NBTI aging
       canary, Kelvin/BGA micro-ohm contact sensing, and `fllAtlas`.
    2. `Ele.Power` (電機): 800V SiC `SSCB` 2-Jet RC-inrush vs short-circuit
       discrimination, Miller soft turn-off `V_ds` clamp, 48V multiphase VRM
       phase-shedding / ZVS / active pre-charge, 180-deg rack phase-interleaving,
       direct 800V HVDC vs triple AC-DC, GFM synthetic rotational inertia,
       electro-thermal cold-plate CHF & `Q^3` pump laws, `sscbAtlas` & `gfmAtlas`.
    3. `Ele.MagWave` (電磁): Datasheet `regRmw8` bitmask safety, stateful 65-wire
       Bus-Invert (`<= 32/65` flips), Single-Edge PPM vs UART, waveguide `Z_0`
       drive bounds, 2-Jet Zero-Bit transceiver lock, dual-rail `1.58-bit`
       ternary MAC `0-ULP` equivalence, BSPDN resonance damping, symplectic
       2-cycle zero-drift spread-spectrum EMI, CPO Modulo-FSR & `cpoAtlas`.
    4. `Ele.Silicon` (矽核): Zero-script `.flow` -> `.memh` Silicon ROM emitter
       and 3-way `0-ULP` 512-bit (`w0..w7`) differential co-verifier across
       `Flow.lean`, `FLOW/build/flow`, and Verilator `rtl/ele_flow_core.sv`.
-/

namespace Ele

def version : String := "2.0.0"

/-! ## 1. `Ele.Electron` (電子): Semiconductor Physics, Clocking & Mixed-Signal -/
namespace Electron

/-- Division-free `<1ns` AD-FLL quadratic stretch numerator: `V_od^2 + V_od * dV + 2 * dV^2`. -/
def fllScaledStretchNum (droop vod : Nat) : Nat :=
  vod * vod + vod * droop + 2 * droop * droop

/-- Per-cycle CMOS dynamic switching energy proportional to `C_eff * V_dd^2`. -/
def chipDynamicCycleEnergy (cEff vDd : Nat) : Nat :=
  cEff * vDd * vDd

/-- LC resonant clock tank vs hard-switched CMOS energy scaled by `qMilli`. -/
def resonantClockScaledEnergy (eCmos factorMilli : Nat) : Nat :=
  eCmos * factorMilli

/-- Hysteresis-guarded asynchronous level-crossing ADC comparator fire predicate. -/
def levelCrossingFires (absDiff lsb hyst : Nat) : Bool :=
  decide (absDiff >= lsb + hyst)

/-- Differential Vernier delay-line sub-gate time resolution (`tSlow - tFast`). -/
def vernierResolutionPs (tSlow tFast : Nat) : Nat :=
  tSlow - tFast

/-- Silicon NBTI aging canary & main-datapath timing violation pair `(canaryWarn, mainSdc)`. -/
def canaryCheck (tMainAged canaryGuard tClk : Nat) : Bool × Bool :=
  (decide (tMainAged + canaryGuard > tClk), decide (tMainAged > tClk))

/-- Sensed Kelvin/BGA rail voltage under regulator reference `vRef` and joint resistance `rJoint`. -/
def contactSenseVoltage (vRef iLoad rJoint : Int) : Int :=
  vRef - iLoad * rJoint

/-- 2-Jet junction temperature lookahead `T_pred = T + dT + ddT/2`. -/
def jet2ThermalLookahead (temp dTemp ddTempHalf : Int) : Int :=
  temp + dTemp + ddTempHalf

/-- 3D HBM stack steady-state self-heating power `pIo + nRefBanks * mult * pBank`. -/
def hbmStackThermalLoad (pIo nRefBanks mult pBank : Nat) : Nat :=
  pIo + nRefBanks * mult * pBank

/--
  AD-FLL Zero-SDC Envelope Theorem:
  Division-free quadratic stretch `(1 + x + 2x^2)` strictly upper-bounds `1 / (1 - x)`
  for all supply droops `droop <= vod / 2` (`0` Silent Data Corruption).
-/
theorem fll_quadratic_envelope_guarantees_zero_sdc
    (droop vod : Nat)
    (hVodPos : vod > 0) (hHalf : 2 * droop <= vod) :
    0 < vod * vod * vod ∧
    vod * vod * vod <= (vod - droop) * fllScaledStretchNum droop vod := by
  have hSqPos : vod * vod > 0 := Nat.mul_pos hVodPos hVodPos
  have hCubePos : vod * vod * vod > 0 := Nat.mul_pos hSqPos hVodPos
  refine ⟨hCubePos, ?_⟩
  have ⟨r, hVod⟩ : ∃ r, vod = 2 * droop + r := ⟨vod - 2 * droop, by omega⟩
  have hSub : vod - droop = droop + r := by omega
  rw [hSub]
  subst hVod
  unfold fllScaledStretchNum
  have hTwo : ∀ n : Nat, 2 * n = n + n := fun n => Nat.two_mul n
  simp only [hTwo, Nat.mul_add, Nat.mul_comm, Nat.mul_left_comm]
  omega

/-- Eliminating static `V_dd` voltage guardband strictly cuts per-cycle CMOS `C * V^2` energy. -/
theorem fll_guardband_elimination_strictly_saves_dynamic_energy
    (cEff vInst vNom vGuard : Nat)
    (hCap : cEff > 0) (hGuard : vGuard > 0) (hInst : vInst <= vNom) :
    chipDynamicCycleEnergy cEff vInst < chipDynamicCycleEnergy cEff (vNom + vGuard) := by
  unfold chipDynamicCycleEnergy
  have hLt : vInst < vNom + vGuard := by omega
  have hPosR : vNom + vGuard > 0 := by omega
  have hSq1 : vInst * vInst <= vInst * (vNom + vGuard) :=
    Nat.mul_le_mul_left vInst (Nat.le_of_lt hLt)
  have hSq2 : vInst * (vNom + vGuard) < (vNom + vGuard) * (vNom + vGuard) :=
    Nat.mul_lt_mul_of_pos_right hLt hPosR
  have hSq : vInst * vInst < (vNom + vGuard) * (vNom + vGuard) :=
    Nat.lt_of_le_of_lt hSq1 hSq2
  have hScaled : cEff * (vInst * vInst) < cEff * ((vNom + vGuard) * (vNom + vGuard)) :=
    Nat.mul_lt_mul_of_pos_left hSq hCap
  simp only [Nat.mul_assoc]
  exact hScaled

/-- LC resonant clock tank strictly recovers dynamic clock-tree charge whenever `Q > pi`. -/
theorem resonant_clock_recovers_charge_above_pi_q
    (eCmos piMilli qMilli : Nat)
    (hPos : eCmos > 0) (hHighQ : piMilli < qMilli) :
    resonantClockScaledEnergy eCmos piMilli < resonantClockScaledEnergy eCmos qMilli := by
  unfold resonantClockScaledEnergy
  exact Nat.mul_lt_mul_of_pos_left hHighQ hPos

/-- Hysteresis band suppresses sub-threshold noise chatter while passing genuine edges. -/
theorem level_crossing_hysteresis_blocks_sub_lsb_chatter
    (noiseDiff sigDiff lsb hyst : Nat)
    (hNoise : noiseDiff < lsb + hyst) (hSig : sigDiff >= lsb + hyst) :
    levelCrossingFires noiseDiff lsb hyst = false ∧
    levelCrossingFires sigDiff lsb hyst = true := by
  unfold levelCrossingFires
  simp only [decide_eq_false_iff_not, decide_eq_true_eq]
  constructor <;> omega

/-- Differential Vernier delay line achieves sub-gate picosecond resolution (`< tFast`). -/
theorem vernier_tdc_resolves_sub_gate_picoseconds
    (tSlow tFast : Nat)
    (hOrder : tFast < tSlow) (hVernier : tSlow < 2 * tFast) :
    0 < vernierResolutionPs tSlow tFast ∧ vernierResolutionPs tSlow tFast < tFast := by
  unfold vernierResolutionPs
  constructor <;> omega

/-- Shadow NBTI aging canary fires strictly before main-datapath timing margin collapses. -/
theorem aging_canary_fires_strictly_before_main_sdc
    (tMainAged canaryGuard tClk : Nat)
    (hEroded : tClk < tMainAged + canaryGuard)
    (hSafeMain : tMainAged <= tClk) :
    canaryCheck tMainAged canaryGuard tClk = (true, false) := by
  unfold canaryCheck
  have h1 : decide (tMainAged + canaryGuard > tClk) = true := by
    simp only [decide_eq_true_eq]; omega
  have h2 : decide (tMainAged > tClk) = false := by
    simp only [decide_eq_false_iff_not]; omega
  rw [h1, h2]

/-- Two-load Kelvin differential contact probe cancels regulator `V_ref` offset. -/
theorem differential_contact_probe_cancels_dc_offset
    (vRef iLow iHigh rHealthy rDegraded : Int)
    (hStep : iLow < iHigh) (hCrack : rHealthy < rDegraded) :
    contactSenseVoltage vRef iLow rHealthy - contactSenseVoltage vRef iHigh rHealthy =
      (iHigh - iLow) * rHealthy ∧
    contactSenseVoltage vRef iLow rHealthy - contactSenseVoltage vRef iHigh rHealthy <
      contactSenseVoltage vRef iLow rDegraded - contactSenseVoltage vRef iHigh rDegraded := by
  unfold contactSenseVoltage
  have hSubH : (iHigh - iLow) * rHealthy = iHigh * rHealthy - iLow * rHealthy :=
    Int.sub_mul iHigh iLow rHealthy
  have hSubD : (iHigh - iLow) * rDegraded = iHigh * rDegraded - iLow * rDegraded :=
    Int.sub_mul iHigh iLow rDegraded
  have hDiPos : 0 < iHigh - iLow := by omega
  have hMono : (iHigh - iLow) * rHealthy < (iHigh - iLow) * rDegraded :=
    Int.mul_lt_mul_of_pos_left hCrack hDiPos
  constructor <;> omega

/-- 2-Jet thermal lookahead strictly anticipates junction heating before reactive threshold. -/
theorem jet2_thermal_preempts_reactive_throttling
    (temp dTemp ddTempHalf : Int)
    (hVel : dTemp > 0) (hAcc : ddTempHalf >= 0) :
    jet2ThermalLookahead temp dTemp ddTempHalf > temp := by
  unfold jet2ThermalLookahead
  omega

/-- Eliding DRAM refresh on transient scratchpad banks strictly lowers 3D HBM self-heating. -/
theorem hbm_transient_refresh_elision_lowers_self_heating
    (pIo liveBanks totalBanks mult pBank : Nat)
    (hElide : liveBanks < totalBanks) (hMult : mult > 0) (hBank : pBank > 0) :
    hbmStackThermalLoad pIo liveBanks mult pBank <
      hbmStackThermalLoad pIo totalBanks mult pBank := by
  unfold hbmStackThermalLoad
  have h1 : liveBanks * mult < totalBanks * mult :=
    Nat.mul_lt_mul_of_pos_right hElide hMult
  have h2 : (liveBanks * mult) * pBank < (totalBanks * mult) * pBank :=
    Nat.mul_lt_mul_of_pos_right h1 hBank
  omega

/-! ### Executable `.flow` Manifold Atlas: All-Digital FLL Droop Stretcher (`EFLL`) -/

inductive FllRegime where
  | nominal
  | stretch
  | emergency
  deriving DecidableEq, Repr, Inhabited

inductive FllFlag where
  | resonantTank
  | canaryArmed
  | spreadSpec
  deriving DecidableEq, Repr, Inhabited

def fllCharts : Unit.ChartSpace FllRegime where
  entries  := [(.nominal, "nominal"), (.stretch, "stretch"), (.emergency, "emergency")]
  fallback := .emergency

def fllFlags : Unit.FlagSpace FllFlag where
  entries := [(.resonantTank, "resonant"), (.canaryArmed, "canary"), (.spreadSpec, "sscg")]

def fllAtlas : Unit.Atlas "mV" FllRegime FllFlag where
  domainTag  := dom! "EFLL"
  charts     := fllCharts
  flags      := fllFlags
  initRegime := .nominal
  initX      := qty! "10.0" "mV"
  initFlags  := [.resonantTank, .canaryArmed]
  law
    | .nominal =>
      Unit.track .nominal (qty! "25.0" "mV")
        |> Unit.Regime.escalateAbove (qty! "30.0" "mV") .stretch
    | .stretch =>
      Unit.throttleQuad .stretch (q16! "1.0") (q16! "0.02") (qty! "35.0" "mV")
        |> Unit.Regime.deadband (qty! "15.0" "mV") (qty! "55.0" "mV") .nominal .emergency
        |> Unit.Regime.lockFlags [.canaryArmed]
    | .emergency =>
      Unit.isolateTo .emergency (qty! "12.0" "mV") (qty! "30.0" "mV")
        |> Unit.Regime.recoverBelow (qty! "15.0" "mV") .nominal
        |> Unit.Regime.lockFlags [.canaryArmed]
        |> Unit.Regime.requireHomotopic

theorem fllEmergencyIsolatesDroopSurge (x s1 s2 : Unit.Qty "mV") :
    (fllAtlas.law .emergency).evalResidual x s1 =
    (fllAtlas.law .emergency).evalResidual x s2 :=
  Unit.zeroGain_ignores_external_shocks (fllAtlas.law .emergency) rfl x s1 s2

end Electron

/-! ## 2. `Ele.Power` (電機): 800V SiC Protection, Multiphase VRM & Grid Dynamics -/
namespace Power

/-- 2-Jet 800V SiC SSCB short-circuit vs capacitive RC hot-plug inrush discriminator. -/
def sscbJet2FaultMatch (iAmp iArm di diCrit ddi negMargin : Int) : Bool :=
  decide (iAmp >= iArm ∧ di >= diCrit ∧ ddi >= negMargin)

/-- Peak SiC drain-source turn-off voltage `V_ds = V_bus + L_bus * |dI/dt|`. -/
def sscbPeakSwitchVoltage (vBus lBus slewRate : Nat) : Nat :=
  vBus + lBus * slewRate

/-- Multiphase VRM 1-phase light-load loss scaled by `N_max`. -/
def vrmSinglePhaseScaledLoss (iSqR pSw nMax : Nat) : Nat :=
  nMax * iSqR + nMax * pSw

/-- Multiphase VRM fixed `N_max`-phase loss scaled by `N_max`. -/
def vrmFullPhaseScaledLoss (iSqR pSw nMax : Nat) : Nat :=
  iSqR + nMax * (nMax * pSw)

/-- Multiphase VRM total converter loss (`conduction + N * perPhaseSwitching`). -/
def vrmTotalLoss (condLoss nPhases pSwPhase : Nat) : Nat :=
  condLoss + nPhases * pSwPhase

/-- Active pre-charged net capacitor droop charge deficit (`stepQ - prechargeQ`). -/
def prechargedDroopDeficit (stepQ prechargeQ : Nat) : Nat :=
  stepQ - prechargeQ

/-- Synchronous 2-socket rack busbar harmonic `I^2` sum over 2 half-waves. -/
def syncRackBusI2Sum (iHigh iLow : Nat) : Nat :=
  (2 * iHigh) * (2 * iHigh) + (2 * iLow) * (2 * iLow)

/-- Symplectic 180-deg phase-interleaved 2-socket rack busbar `I^2` sum. -/
def interleavedRackBusI2Sum (iHigh iLow : Nat) : Nat :=
  (iHigh + iLow) * (iHigh + iLow) + (iLow + iHigh) * (iLow + iHigh)

/-- Legacy `PV(DC)->AC->UPS(AC-DC-AC)->PFC(DC)` triple conversion waste (`143` permille). -/
def tripleAcDcPermilleLoss (pKw : Nat) : Nat :=
  pKw * 143

/-- Direct 800V HVDC single-stage ZVS DC-DC conversion waste (`25` permille). -/
def hvdcDirectPermilleLoss (pKw : Nat) : Nat :=
  pKw * 25

/-- Grid-Following (`GFL`, `H = 0`) island frequency nadir under generation loss. -/
def gflFreqNadirMhz (fNom dropMhz : Nat) : Nat :=
  fNom - dropMhz

/-- Grid-Forming (`GFM`) synthetic-inertia frequency nadir under generation loss. -/
def gfmFreqNadirMhz (fNom dropMhz inertiaMitigationMhz : Nat) : Nat :=
  fNom - (dropMhz - inertiaMitigationMhz)

/-- Unabsorbed midday PV curtailment / reverse feeder export (`kW`). -/
def solarCurtailmentKw (pvKw loadKw : Nat) : Nat :=
  pvKw - loadKw

/-- Symplectic High-Z coast dissipated power (`0 mW` when `EN = 0`). -/
def coastWastePowerMw : Nat := 0

/-- Continuous PWM holding parasitic power (`I * V_sat + I^2 * R_coil`). -/
def continuousPwmWastePowerMw (currentMa vSatMv coilR : Nat) : Nat :=
  currentMa * vSatMv + currentMa * currentMa * coilR

/-- Two-phase microchannel cold-plate Critical Heat Flux (CHF) dryout predicate. -/
def coldplateIsDryout (qWatt flowMlS hFg xCritPermille : Nat) : Bool :=
  decide (qWatt * 1000 >= flowMlS * hFg * xCritPermille)

/-- Pump Affinity Cube-Law hydraulic motor shaft power scaled by `flowMax^3`. -/
def pumpCubeScaledPower (pIdle pShaft flow flowMax : Nat) : Nat :=
  pIdle * (flowMax * flowMax * flowMax) + pShaft * (flow * flow * flow)

/-- 2-Jet SiC SSCB rejects concave RC hot-plug inrush (`d2I < -margin`) while tripping shorts. -/
theorem sscb_jet2_separates_rc_inrush_from_short_circuit
    (iAmp iArm di diCrit ddiInrush ddiShort negMargin : Int)
    (hInrush : ddiInrush < negMargin)
    (hArm : iAmp >= iArm) (hSlope : di >= diCrit) (hShort : ddiShort >= negMargin) :
    sscbJet2FaultMatch iAmp iArm di diCrit ddiInrush negMargin = false ∧
    sscbJet2FaultMatch iAmp iArm di diCrit ddiShort negMargin = true := by
  unfold sscbJet2FaultMatch
  simp only [decide_eq_false_iff_not, decide_eq_true_eq]
  constructor <;> omega

/-- Miller-plateau soft turn-off clamps inductive kickback `V_ds` below SiC avalanche breakdown. -/
theorem sscb_soft_turnoff_clamps_inductive_kickback_below_breakdown
    (vBus lBus slewSoft slewHard vBreakdown : Nat)
    (hInd : lBus > 0) (hSlew : slewSoft < slewHard)
    (hSafe : vBus + lBus * slewSoft < vBreakdown) :
    sscbPeakSwitchVoltage vBus lBus slewSoft < sscbPeakSwitchVoltage vBus lBus slewHard ∧
    sscbPeakSwitchVoltage vBus lBus slewSoft < vBreakdown := by
  unfold sscbPeakSwitchVoltage
  have hMul : lBus * slewSoft < lBus * slewHard :=
    Nat.mul_lt_mul_of_pos_left hSlew hInd
  constructor <;> omega

/-- Dynamic VRM phase-shedding reduces converter loss below crossover `I^2 * R < N_max * P_sw`. -/
theorem vrm_phase_shedding_strictly_reduces_light_load_loss
    (iSqR pSw nMax : Nat)
    (hPhases : nMax >= 2) (hLight : iSqR < nMax * pSw) :
    vrmSinglePhaseScaledLoss iSqR pSw nMax < vrmFullPhaseScaledLoss iSqR pSw nMax := by
  match nMax, hPhases with
  | k + 2, _ =>
    unfold vrmSinglePhaseScaledLoss vrmFullPhaseScaledLoss
    have hkPos : k + 1 > 0 := by omega
    have hMul : (k + 1) * iSqR < (k + 1) * ((k + 2) * pSw) :=
      Nat.mul_lt_mul_of_pos_left hLight hkPos
    have hL : (k + 2) * iSqR = (k + 1) * iSqR + iSqR :=
      Nat.succ_mul (k + 1) iSqR
    have hR : (k + 2) * ((k + 2) * pSw) = (k + 1) * ((k + 2) * pSw) + (k + 2) * pSw :=
      Nat.succ_mul (k + 1) ((k + 2) * pSw)
    omega

/-- Resonant Zero-Voltage Switching (ZVS) strictly lowers total multiphase VRM loss. -/
theorem zvs_soft_switching_strictly_reduces_vrm_loss
    (condLoss nPhases pSwZvs pSwHard : Nat)
    (hN : nPhases > 0) (hZvs : pSwZvs < pSwHard) :
    vrmTotalLoss condLoss nPhases pSwZvs < vrmTotalLoss condLoss nPhases pSwHard := by
  unfold vrmTotalLoss
  have hMul : nPhases * pSwZvs < nPhases * pSwHard :=
    Nat.mul_lt_mul_of_pos_left hZvs hN
  omega

/-- Command-lookahead Active Pre-Charge cuts unipolar `V_droop` deficit by `>= 50%`. -/
theorem active_precharge_cuts_unipolar_vdroop
    (stepQ prechargeQ : Nat)
    (hPrePos : prechargeQ > 0) (hHalf : prechargeQ * 2 >= stepQ) (hBound : prechargeQ <= stepQ) :
    prechargedDroopDeficit stepQ prechargeQ * 2 <= stepQ ∧
    prechargedDroopDeficit stepQ prechargeQ < stepQ := by
  unfold prechargedDroopDeficit
  constructor <;> omega

/-- 180-deg rack phase-interleaving conserves 100% work while cutting peak current & `I^2 * R`. -/
theorem phase_interleaving_conserves_work_cuts_peak_and_i2r
    (iHigh iLow : Nat)
    (hCrest : iLow < iHigh) :
    ((iHigh + iLow) + (iLow + iHigh) = 2 * iHigh + 2 * iLow) ∧
    (iHigh + iLow < 2 * iHigh) ∧
    (interleavedRackBusI2Sum iHigh iLow < syncRackBusI2Sum iHigh iLow) := by
  refine ⟨by omega, by omega, ?_⟩
  unfold interleavedRackBusI2Sum syncRackBusI2Sum
  have ⟨d, hdPos, hEq⟩ : ∃ d, d > 0 ∧ iHigh = iLow + d :=
    ⟨iHigh - iLow, by omega, by omega⟩
  subst hEq
  have hdd : d * d > 0 := Nat.mul_pos hdPos hdPos
  have hComm : d * iLow = iLow * d := Nat.mul_comm d iLow
  have hTwo : ∀ x : Nat, 2 * x = x + x := fun x => Nat.two_mul x
  simp only [hTwo, Nat.add_mul, Nat.mul_add, hComm]
  omega

/-- Direct 800V HVDC bus cuts conversion waste by `> 5x` vs triple AC-DC-AC-DC conversion. -/
theorem hvdc_direct_bus_strictly_beats_triple_ac_dc_conversion
    (pKw : Nat) (hPos : pKw > 0) :
    hvdcDirectPermilleLoss pKw * 5 < tripleAcDcPermilleLoss pKw := by
  unfold hvdcDirectPermilleLoss tripleAcDcPermilleLoss
  omega

/-- Grid-Forming (GFM) synthetic rotational inertia strictly raises island frequency nadir. -/
theorem gfm_synthetic_inertia_raises_island_frequency_nadir
    (fNom dropMhz inertiaMitigationMhz : Nat)
    (hMitPos : inertiaMitigationMhz > 0)
    (hMitBound : inertiaMitigationMhz <= dropMhz)
    (hDropBound : dropMhz <= fNom) :
    gflFreqNadirMhz fNom dropMhz < gfmFreqNadirMhz fNom dropMhz inertiaMitigationMhz := by
  unfold gflFreqNadirMhz gfmFreqNadirMhz
  omega

/-- Physical PV-BESS-load balancing conserves daily work while cutting curtailment & peak draw. -/
theorem diurnal_sponge_conserves_work_cuts_curtailment_and_evening_peak
    (pvMidday cMidday cEvening eShift pStorage : Nat)
    (hShiftPos : eShift > 0)
    (hShiftBound : eShift <= cEvening)
    (hStorPos : pStorage > 0)
    (hStorBound : pStorage <= cEvening - eShift)
    (hPvSurplus : cMidday + eShift <= pvMidday) :
    ((cMidday + eShift) + (cEvening - eShift) = cMidday + cEvening) ∧
    (solarCurtailmentKw pvMidday (cMidday + eShift) < solarCurtailmentKw pvMidday cMidday) ∧
    ((cEvening - eShift) - pStorage < cEvening - eShift) ∧
    ((cEvening - eShift) - pStorage < cEvening) := by
  unfold solarCurtailmentKw
  refine ⟨by omega, by omega, by omega, by omega⟩

/-- High-Z coasting strictly eliminates continuous PWM MOSFET saturation & copper heat. -/
theorem coast_eliminates_pwm_waste (currentMa vSatMv coilR : Nat)
    (hI : currentMa > 0) (hV : vSatMv > 0) :
    coastWastePowerMw < continuousPwmWastePowerMw currentMa vSatMv coilR := by
  unfold coastWastePowerMw continuousPwmWastePowerMw
  have h1 : currentMa * vSatMv > 0 := Nat.mul_pos hI hV
  omega

/-- 2-Jet pre-spinning cold-plate coolant flow prevents Critical Heat Flux (CHF) dryout. -/
theorem coldplate_pre_spin_prevents_chf_vapor_dryout
    (qBurst flowReact flowPred hFg xTarget xCrit : Nat)
    (hFlow : flowPred > 0) (hLatent : hFg > 0) (hMargin : xTarget < xCrit)
    (hLagDryout : flowReact * hFg * xCrit <= qBurst * 1000)
    (hPreSpinSized : qBurst * 1000 <= flowPred * hFg * xTarget) :
    coldplateIsDryout qBurst flowReact hFg xCrit = true ∧
    coldplateIsDryout qBurst flowPred hFg xCrit = false := by
  unfold coldplateIsDryout
  have hCapPos : flowPred * hFg > 0 := Nat.mul_pos hFlow hLatent
  have hStrict : (flowPred * hFg) * xTarget < (flowPred * hFg) * xCrit :=
    Nat.mul_lt_mul_of_pos_left hMargin hCapPos
  simp only [decide_eq_true_eq, decide_eq_false_iff_not]
  constructor <;> omega

/-- Pump Affinity Cube Law cuts shaft power by `8x` at 50% flow and lowers total pump power. -/
theorem pump_affinity_cube_law_strictly_cuts_hydraulic_power
    (pIdle pShaft flow flowMax : Nat)
    (hShaft : pShaft > 0) (hSubMax : flow < flowMax) :
    pumpCubeScaledPower pIdle pShaft flow flowMax <
      pumpCubeScaledPower pIdle pShaft flowMax flowMax ∧
    8 * (pShaft * (flow * flow * flow)) =
      pShaft * ((2 * flow) * (2 * flow) * (2 * flow)) := by
  unfold pumpCubeScaledPower
  have hMaxPos : flowMax > 0 := by omega
  have h1 : flow * flow <= flow * flowMax :=
    Nat.mul_le_mul_left flow (Nat.le_of_lt hSubMax)
  have h2 : flow * flowMax < flowMax * flowMax :=
    Nat.mul_lt_mul_of_pos_right hSubMax hMaxPos
  have hSq : flow * flow < flowMax * flowMax := Nat.lt_of_le_of_lt h1 h2
  have h3 : (flow * flow) * flow <= (flow * flow) * flowMax :=
    Nat.mul_le_mul_left (flow * flow) (Nat.le_of_lt hSubMax)
  have h4 : (flow * flow) * flowMax < (flowMax * flowMax) * flowMax :=
    Nat.mul_lt_mul_of_pos_right hSq hMaxPos
  have hCube : flow * flow * flow < flowMax * flowMax * flowMax :=
    Nat.lt_of_le_of_lt h3 h4
  have hScaled : pShaft * (flow * flow * flow) < pShaft * (flowMax * flowMax * flowMax) :=
    Nat.mul_lt_mul_of_pos_left hCube hShaft
  constructor
  · omega
  · have hTwo : 2 * flow = flow + flow := Nat.two_mul flow
    rw [hTwo]
    have hEight : ∀ x : Nat, 8 * x = x + x + x + x + x + x + x + x := by omega
    rw [hEight]
    simp only [Nat.add_mul, Nat.mul_add]
    omega

/-! ### Executable `.flow` Manifold Atlases: 800V SiC `SSCB` & Grid-Forming Inverter (`EGFM`) -/

inductive SscbRegime where
  | conducting
  | millerClamp
  | tripped
  deriving DecidableEq, Repr, Inhabited

inductive SscbFlag where
  | gateEnabled
  | prechargeReady
  deriving DecidableEq, Repr, Inhabited

def sscbCharts : Unit.ChartSpace SscbRegime where
  entries  := [(.conducting, "conducting"), (.millerClamp, "miller_clamp"), (.tripped, "tripped")]
  fallback := .tripped

def sscbFlags : Unit.FlagSpace SscbFlag where
  entries := [(.gateEnabled, "gate_en"), (.prechargeReady, "precharge")]

def sscbAtlas : Unit.Atlas "A" SscbRegime SscbFlag where
  domainTag  := dom! "SSCB"
  charts     := sscbCharts
  flags      := sscbFlags
  initRegime := .conducting
  initX      := qty! "40.0" "A"
  initFlags  := [.gateEnabled, .prechargeReady]
  law
    | .conducting =>
      Unit.track .conducting (qty! "90.0" "A")
        |> Unit.Regime.escalateAbove (qty! "120.0" "A") .millerClamp
    | .millerClamp =>
      Unit.throttleQuad .millerClamp (q16! "1.0") (q16! "0.01") (qty! "70.0" "A")
        |> Unit.Regime.deadband (qty! "80.0" "A") (qty! "165.0" "A") .conducting .tripped
        |> Unit.Regime.lockFlags [.prechargeReady]
    | .tripped =>
      Unit.isolateTo .tripped (qty! "0.0" "A") (qty! "90.0" "A")
        |> Unit.Regime.recoverBelow (qty! "-1.0" "A") .tripped
        |> Unit.Regime.lockFlags [.gateEnabled, .prechargeReady]
        |> Unit.Regime.requireHomotopic

theorem sscbTrippedIsolatesShortCurrent (x i1 i2 : Unit.Qty "A") :
    (sscbAtlas.law .tripped).evalResidual x i1 =
    (sscbAtlas.law .tripped).evalResidual x i2 :=
  Unit.zeroGain_ignores_external_shocks (sscbAtlas.law .tripped) rfl x i1 i2

inductive GfmRegime where
  | stiffGrid
  | syntheticInertia
  | islandSafe
  deriving DecidableEq, Repr, Inhabited

inductive GfmFlag where
  | bessForming
  | rocofGuard
  deriving DecidableEq, Repr, Inhabited

def gfmCharts : Unit.ChartSpace GfmRegime where
  entries  := [(.stiffGrid, "stiff_grid"), (.syntheticInertia, "synth_inertia"), (.islandSafe, "island_safe")]
  fallback := .islandSafe

def gfmFlags : Unit.FlagSpace GfmFlag where
  entries := [(.bessForming, "bess"), (.rocofGuard, "rocof")]

/-- Grid-Forming frequency deviation Atlas (`mHz` droop magnitude away from `60.000 Hz`). -/
def gfmAtlas : Unit.Atlas "mHz" GfmRegime GfmFlag where
  domainTag  := dom! "EGFM"
  charts     := gfmCharts
  flags      := gfmFlags
  initRegime := .stiffGrid
  initX      := qty! "20.0" "mHz"
  initFlags  := [.bessForming, .rocofGuard]
  law
    | .stiffGrid =>
      Unit.track .stiffGrid (qty! "120.0" "mHz")
        |> Unit.Regime.escalateAbove (qty! "150.0" "mHz") .syntheticInertia
    | .syntheticInertia =>
      Unit.throttleQuad .syntheticInertia (q16! "1.0") (q16! "0.005") (qty! "150.0" "mHz")
        |> Unit.Regime.deadband (qty! "80.0" "mHz") (qty! "320.0" "mHz") .stiffGrid .islandSafe
        |> Unit.Regime.lockFlags [.bessForming, .rocofGuard]
    | .islandSafe =>
      Unit.isolateTo .islandSafe (qty! "50.0" "mHz") (qty! "150.0" "mHz")
        |> Unit.Regime.recoverBelow (qty! "70.0" "mHz") .stiffGrid
        |> Unit.Regime.lockFlags [.bessForming, .rocofGuard]
        |> Unit.Regime.requireHomotopic

end Power

/-! ## 3. `Ele.MagWave` (電磁): Wire Coding, Waveguide/PDN/EMI & Silicon Photonics -/
namespace MagWave

/-- Declarative hardware register Read-Modify-Write (`RMW`) bitmask operator. -/
def regRmw8 (oldReg mask newBits : UInt8) : UInt8 :=
  (oldReg &&& (~~~mask)) ||| (newBits &&& mask)

/-- Stateful 65-wire Bus-Invert (`64` data + `1` polarity wire) physical flip count. -/
def busInvertWireFlips (flipsP0 : Nat) : Nat :=
  if flipsP0 > 32 then 65 - flipsP0 else flipsP0

/-- Single-Edge Pulse-Position Modulation transitions per byte (`1` flip). -/
def ppmTransitionsPerByte : Nat := 1

/-- Minimum transitions of standard 8N1 UART frame (`start + stop >= 2`). -/
def uartMinTransitionsPerByte (dataFlips : Nat) : Nat := 2 + dataFlips

/-- Waveguide impedance `Z_0` pad-drive selector (`0=5mA, 1=10mA, 2=20mA, 3=40mA`). -/
def selectOptimalDrive (z0Ohms cFf : Nat) : Nat :=
  if z0Ohms >= 200 || cFf <= 4000 then 0
  else if z0Ohms >= 95 && cFf <= 16000 then 1
  else if cFf <= 35000 then 2
  else 3

/-- 2-Jet Zero-Bit predictive transceiver step (`txNextState, wireDelta`). -/
def predictJet2Step (x v aHalf : Int) : Int :=
  x + v + aHalf

def syncTxStep (x v aHalf xMeas eps : Int) : Int × Int :=
  let pred := predictJet2Step x v aHalf
  let res := xMeas - pred
  if -eps <= res && res <= eps then (pred, 0) else (xMeas, res)

def syncRxStep (x v aHalf wireDelta : Int) : Int :=
  predictJet2Step x v aHalf + wireDelta

def syncGateDecision (absResidual epsilon : Nat) : Nat :=
  if absResidual <= epsilon then 0 else 1

/-- Tickless 2-Jet energy-proportional bus polling period governor & 9-clock bus recovery. -/
def ticklessGovernorPeriod (activity eps curPeriod minPeriod maxPeriod : Nat) : Nat :=
  if activity > eps then minPeriod
  else
    let doubled := if curPeriod * 2 <= maxPeriod then curPeriod * 2 else maxPeriod
    if doubled < minPeriod then minPeriod else doubled

def driverHotSwapSelectScale (absDiff maxSlew newScale shadowScale : Nat) : Nat :=
  if absDiff > maxSlew then shadowScale else newScale

def busHealPulseStep (prevPulses : Nat) : Nat :=
  prevPulses + 9

/-- Dual monotonic magnitude-rail `1.58-bit` ternary MAC terminal merge (`posRail - negRail`). -/
def dualRailTernaryDiff (posRail negRail : Int) : Int :=
  posRail - negRail

/-- Inductive PDN supply guardband `V_DD = V_nom + L_droop * dI_step` & BSPDN active damping. -/
def pdnGuardbandVdd (vNom lDroop diStep : Nat) : Nat :=
  vNom + lDroop * diStep

def bspdnDampedExcursion (vUndamped dampDelta : Int) : Int :=
  vUndamped - dampDelta

/-- Symplectic rail-clamped paired `(+delta, -delta)` spread-spectrum 2-cycle period sum. -/
def spreadSpectrumPairSum (tNom delta lo hi : Int) : Int :=
  (if tNom + delta < lo then lo else if tNom + delta > hi then hi else tNom + delta) +
  (if tNom - delta < lo then lo else if tNom - delta > hi then hi else tNom - delta)

def spreadTotalSpectralPower (pBin nBins : Nat) : Nat :=
  pBin * nBins

/-- Co-Packaged Optics (CPO) microring Modulo-FSR detuning & thermal phase residual. -/
def cpoModuloFsrDetune (unwrappedDetune fsr : Nat) : Nat :=
  unwrappedDetune % fsr

def cpoCompensatedResidual (target thermal wave heater : Int) : Int :=
  target - (thermal + wave + heater)

/-- Datasheet RMW preserves all unmasked reserved hardware bits (`bv_decide`). -/
theorem rmw_preserves_unmasked_datasheet_bits (oldReg mask newBits : UInt8) :
    (regRmw8 oldReg mask newBits) &&& (~~~mask) = oldReg &&& (~~~mask) := by
  unfold regRmw8; bv_decide

/-- Datasheet RMW writes exact target bits inside mask (`bv_decide`). -/
theorem rmw_writes_masked_target_bits (oldReg mask newBits : UInt8) :
    (regRmw8 oldReg mask newBits) &&& mask = newBits &&& mask := by
  unfold regRmw8; bv_decide

/--
  65-Wire Charge-Minimal Bus-Invert Theorem (matches `rtl/ele_em_phy.sv:ele_bus_invert65`):
  Physical wire flips never exceed `32 / 65` (`< 50%` Maxwell displacement current `C * dV/dt`).
-/
theorem bus_invert_65wire_strictly_caps_flips_at_32 (flipsP0 : Nat)
    (hValid : flipsP0 <= 65) :
    busInvertWireFlips flipsP0 <= 32 := by
  unfold busInvertWireFlips
  split <;> omega

/-- Single-Edge PPM strictly uses fewer surface-charge transitions than 8N1 UART. -/
theorem ppm_strictly_fewer_flips_than_uart (dataFlips : Nat) :
    ppmTransitionsPerByte < uartMinTransitionsPerByte dataFlips := by
  unfold ppmTransitionsPerByte uartMinTransitionsPerByte; omega

/-- Waveguide `Z_0` pad-drive selector stays strictly within `[0, 3]`. -/
theorem optimal_drive_bounded (z0Ohms cFf : Nat) :
    selectOptimalDrive z0Ohms cFf <= 3 := by
  unfold selectOptimalDrive
  split <;> try omega
  split <;> try omega
  split <;> omega

/-- 2-Jet Zero-Bit Tx/Rx maintains `0-ULP` state lock and bounds error `<= eps`. -/
theorem jet2_tx_rx_zero_ulp_equivalence
    (xTx vTx aTx xRx vRx aRx xMeas eps : Int)
    (hx : xTx = xRx) (hv : vTx = vRx) (ha : aTx = aRx) (hEps : eps >= 0) :
    let (txNext, wireDelta) := syncTxStep xTx vTx aTx xMeas eps
    let rxNext := syncRxStep xRx vRx aRx wireDelta
    txNext = rxNext ∧ -eps <= xMeas - rxNext ∧ xMeas - rxNext <= eps := by
  subst hx; subst hv; subst ha
  unfold syncTxStep syncRxStep predictJet2Step
  dsimp
  split
  · next hSleep =>
    simp only [Bool.and_eq_true, decide_eq_true_eq] at hSleep
    omega
  · omega

theorem sync_gate_zero_bit_within_deadband (absResidual epsilon : Nat)
    (h : absResidual <= epsilon) :
    syncGateDecision absResidual epsilon = 0 := by
  unfold syncGateDecision; simp [h]

theorem tickless_governor_snaps_to_min_on_surprise
    (activity eps curPeriod minPeriod maxPeriod : Nat) (hSurprise : activity > eps) :
    ticklessGovernorPeriod activity eps curPeriod minPeriod maxPeriod = minPeriod := by
  unfold ticklessGovernorPeriod; simp [hSurprise]

theorem tickless_governor_bounded
    (activity eps curPeriod minPeriod maxPeriod : Nat) (hOrder : minPeriod <= maxPeriod) :
    minPeriod <= ticklessGovernorPeriod activity eps curPeriod minPeriod maxPeriod ∧
    ticklessGovernorPeriod activity eps curPeriod minPeriod maxPeriod <= maxPeriod := by
  unfold ticklessGovernorPeriod
  split
  · exact ⟨Nat.le_refl minPeriod, hOrder⟩
  · dsimp; split <;> split <;> omega

theorem driver_hotswap_auto_rollback_on_slew_fault (absDiff maxSlew newScale shadowScale : Nat)
    (hFault : absDiff > maxSlew) :
    driverHotSwapSelectScale absDiff maxSlew newScale shadowScale = shadowScale := by
  unfold driverHotSwapSelectScale; simp [hFault]

theorem bus_heal_strictly_advances_9_pulses (prevPulses : Nat) :
    busHealPulseStep prevPulses - prevPulses = 9 := by
  unfold busHealPulseStep; omega

/--
  Dual Monotonic Magnitude-Rail Ternary Theorem (matches `rtl/ele_em_phy.sv:ele_ternary_mac_dualrail`):
  Accumulating magnitudes onto positive/negative monotonic rails is `0-ULP` identical
  to signed two's-complement addition/subtraction while eliminating sign-bit toggle storms.
-/
theorem dual_rail_ternary_zero_ulp_equivalence (posRail negRail mag : Int) :
    (dualRailTernaryDiff (posRail + mag) negRail = dualRailTernaryDiff posRail negRail + mag) ∧
    (dualRailTernaryDiff posRail (negRail + mag) = dualRailTernaryDiff posRail negRail - mag) := by
  unfold dualRailTernaryDiff
  constructor <;> omega

/-- Symplectic `dI/dt` slew reduction strictly lowers inductive PDN `L * dI/dt` guardband. -/
theorem symplectic_slew_strictly_reduces_vdroop_vdd
    (vNom lDroop diSymp diSync : Nat)
    (hInd : lDroop > 0) (hSlew : diSymp < diSync) :
    pdnGuardbandVdd vNom lDroop diSymp < pdnGuardbandVdd vNom lDroop diSync := by
  unfold pdnGuardbandVdd
  have hMul : lDroop * diSymp < lDroop * diSync := Nat.mul_lt_mul_of_pos_left hSlew hInd
  omega

/-- Active velocity damping strictly attenuates high-Q BSPDN standing-wave excursions. -/
theorem bspdn_active_damping_attenuates_resonance_step
    (vUndamped dampDelta : Int)
    (hDamp : 0 < dampDelta) (hBound : dampDelta <= vUndamped) :
    0 <= bspdnDampedExcursion vUndamped dampDelta ∧
    bspdnDampedExcursion vUndamped dampDelta < vUndamped := by
  unfold bspdnDampedExcursion
  constructor <;> omega

/-- Paired `(+delta, -delta)` spread-spectrum conserves exact 2-cycle period (`0-ULP` drift). -/
theorem symplectic_spread_spectrum_conserves_two_cycle_period
    (tNom delta dMax lo hi : Int)
    (hBoundLo : -dMax <= delta) (hBoundHi : delta <= dMax)
    (hHeadLo : lo <= tNom - dMax) (hHeadHi : tNom + dMax <= hi) :
    spreadSpectrumPairSum tNom delta lo hi = 2 * tNom := by
  unfold spreadSpectrumPairSum
  omega

/-- Spreading harmonic energy across `nBins >= 2` strictly lowers peak narrowband EMI bin power. -/
theorem spread_spectrum_flattens_peak_emi_bin
    (pBin nBins pTotal : Nat)
    (hPos : pTotal > 0) (hBins : nBins >= 2)
    (hSpread : spreadTotalSpectralPower pBin nBins <= pTotal) :
    pBin < pTotal := by
  unfold spreadTotalSpectralPower at hSpread
  have hTwo : pBin * 2 <= pBin * nBins := Nat.mul_le_mul_left pBin hBins
  omega

/-- CPO Modulo-FSR hopping strictly bounds micro-heater detuning below `1 FSR`. -/
theorem cpo_modulo_fsr_bounds_heater_detuning
    (unwrappedDetune fsr : Nat)
    (hFsr : fsr > 0) (hMultiFsr : unwrappedDetune >= fsr) :
    cpoModuloFsrDetune unwrappedDetune fsr < fsr ∧
    cpoModuloFsrDetune unwrappedDetune fsr < unwrappedDetune := by
  unfold cpoModuloFsrDetune
  have hMod : unwrappedDetune % fsr < fsr := Nat.mod_lt unwrappedDetune hFsr
  constructor <;> omega

/-- 2-Jet XPU die heat-wave feedforward cancels CPO optical phase error to `0`. -/
theorem cpo_thermal_feedforward_cancels_die_heat_wave
    (target thermal wave : Int) (hWave : wave > 0) :
    cpoCompensatedResidual target thermal wave (target - (thermal + wave)) = 0 ∧
    cpoCompensatedResidual target thermal wave (target - thermal) ≠ 0 := by
  unfold cpoCompensatedResidual
  constructor <;> omega

/-! ### Executable `.flow` Manifold Atlas: Co-Packaged Optics Microring Stabilizer (`ECPO`) -/

inductive CpoRegime where
  | locked
  | thermalTrack
  | fsrTunnel
  | laserSafe
  deriving DecidableEq, Repr, Inhabited

inductive CpoFlag where
  | laserOn
  | heaterActive
  deriving DecidableEq, Repr, Inhabited

def cpoCharts : Unit.ChartSpace CpoRegime where
  entries  := [(.locked, "locked"), (.thermalTrack, "thermal_track"),
               (.fsrTunnel, "fsr_tunnel"), (.laserSafe, "laser_safe")]
  fallback := .laserSafe

def cpoFlags : Unit.FlagSpace CpoFlag where
  entries := [(.laserOn, "laser"), (.heaterActive, "heater")]

def cpoAtlas : Unit.Atlas "pm" CpoRegime CpoFlag where
  domainTag  := dom! "ECPO"
  charts     := cpoCharts
  flags      := cpoFlags
  initRegime := .locked
  initX      := qty! "5.0" "pm"
  initFlags  := [.laserOn, .heaterActive]
  law
    | .locked =>
      Unit.track .locked (qty! "30.0" "pm")
        |> Unit.Regime.escalateAbove (qty! "40.0" "pm") .thermalTrack
    | .thermalTrack =>
      Unit.throttleQuad .thermalTrack (q16! "1.0") (q16! "0.015") (qty! "45.0" "pm")
        |> Unit.Regime.deadband (qty! "20.0" "pm") (qty! "90.0" "pm") .locked .fsrTunnel
        |> Unit.Regime.lockFlags [.laserOn]
    | .fsrTunnel =>
      -- Barrier/curvature hop (`Delta < 0` when `u_ext <= 0`) testing 2-Jet tunneling in RTL
      Unit.customJet2 .fsrTunnel (q16! "4.0") 0 (q16! "2.0") (qty! "50.0" "pm") 0
        |> Unit.Regime.deadband (qty! "15.0" "pm") (qty! "140.0" "pm") .locked .laserSafe
        |> Unit.Regime.lockFlags [.laserOn]
    | .laserSafe =>
      Unit.isolateTo .laserSafe (qty! "10.0" "pm") (qty! "40.0" "pm")
        |> Unit.Regime.recoverBelow (qty! "15.0" "pm") .locked
        |> Unit.Regime.lockFlags [.laserOn, .heaterActive]
        |> Unit.Regime.requireHomotopic

end MagWave

/-! ## 4. `Ele.Silicon` (矽核): `.memh` Silicon ROM Emitter & 3-Way `0-ULP` Co-Verifier -/
namespace Silicon

instance : Inhabited Flow.Cell64 where
  default := { w0 := 0, w1 := 0, w2 := 0, w3 := 0, w4 := 0, w5 := 0, w6 := 0, w7 := 0 }

private def i64Mod  : Int := 18446744073709551616 -- 2^64
private def i64Half : Int := 9223372036854775808  -- 2^63
private def i64Min  : Int := -9223372036854775808
private def i64Max  : Int := 9223372036854775807
private def i32Mod  : Int := 4294967296           -- 2^32
private def i32Half : Int := 2147483648           -- 2^31

def u64ToI64 (u : UInt64) : Int :=
  let n : Int := Int.ofNat u.toNat
  if n >= i64Half then n - i64Mod else n

def i64ToU64 (x : Int) : UInt64 :=
  UInt64.ofNat (Int.emod x i64Mod).toNat

def wrapI64 (x : Int) : Int :=
  u64ToI64 (i64ToU64 x)

def sextI32 (u32 : UInt64) : Int :=
  let n : Int := Int.ofNat (u32 &&& 0xFFFFFFFF).toNat
  if n >= i32Half then n - i32Mod else n

def absI64ToU64 (x : Int) : UInt64 :=
  let ux := i64ToU64 x
  if x < 0 then 0 - ux else ux

def qaddI64 (a b : Int) : Int × Bool :=
  let s := a + b
  (wrapI64 s, decide (s < i64Min ∨ s > i64Max))

def qsubI64 (a b : Int) : Int × Bool :=
  let d := a - b
  (wrapI64 d, decide (d < i64Min ∨ d > i64Max))

def qmulI64 (a b : Int) : Int × Bool :=
  let ua : Nat := (i64ToU64 a).toNat
  let ub : Nat := (i64ToU64 b).toNat
  let prod : Nat := ua * ub
  let ulo : UInt64 := UInt64.ofNat (prod % (2 ^ 64))
  let uhi : UInt64 := UInt64.ofNat (prod / (2 ^ 64))
  let subA : UInt64 := if a < 0 then i64ToU64 b else 0
  let subB : UInt64 := if b < 0 then i64ToU64 a else 0
  let hi : UInt64 := uhi - subA - subB
  let shifted : UInt64 := (ulo >>> 16) ||| (hi <<< 48)
  let res : Int := u64ToI64 shifted
  let signFill : UInt64 := if res < 0 then 0xFFFFFFFFFFFFFFFF else 0
  let ov : Bool := ((hi ^^^ signFill) >>> 15) != 0
  (res, ov)

def qshr1I64 (x : Int) : Int :=
  let ux := i64ToU64 x
  let signBit := (ux >>> 63) <<< 63
  u64ToI64 ((ux >>> 1) ||| signBit)

def isqrtU64 (x : UInt64) : UInt64 := Id.run do
  let mut rem : UInt64 := x
  let mut root : UInt64 := 0
  for step in [0 : 32] do
    let sh : UInt64 := UInt64.ofNat (62 - 2 * step)
    let b : UInt64 := (1 : UInt64) <<< sh
    let cand : UInt64 := root + b
    if rem >= cand then
      rem := rem - cand
      root := (root >>> 1) + b
    else
      root := root >>> 1
  return root

def qsqrtU64 (xQ16 : UInt64) : UInt64 :=
  let hiRange := (xQ16 >>> 48) != 0
  let arg := if hiRange then xQ16 else xQ16 <<< 16
  let r := isqrtU64 arg
  if hiRange then r <<< 8 else r

def qdivI64 (numQ16 denQ16 : Int) : Int :=
  let anum := absI64ToU64 numQ16
  let aden := absI64ToU64 denQ16
  let denZero := aden == 0
  let safeDen := if denZero then 1 else aden
  let hiNum := (anum >>> 47) != 0
  let qLo := UInt64.ofNat (((anum <<< 16).toNat) / safeDen.toNat)
  let safeDenHi := (safeDen >>> 16) ||| 1
  let qHi := UInt64.ofNat (anum.toNat / safeDenHi.toNat)
  let qMag := if hiNum then qHi else qLo
  let clamped : UInt64 := if qMag > 0x7FFFFFFFFFFFFFFF then 0x7FFFFFFFFFFFFFFF else qMag
  let neg := decide (numQ16 < 0) != decide (denQ16 < 0)
  let raw := if neg then (0 : UInt64) - clamped else clamped
  u64ToI64 raw

/-- Pure Lean bit-exact mirror of `flow_step_chart` (`flow.h` lines 377-547). -/
def stepChartExact (state bound : Flow.Cell64) (capMask : UInt64)
    (uExt : Int) (modeReq : UInt16) : Flow.Cell64 :=
  let stateSealOk := (state.w7 != 0) && Flow.verifyStateCell state
  let boundSealOk := (bound.w7 != 0) && Flow.verifyBoundCell bound
  let sealOk := stateSealOk && boundSealOk
  let domOk := (state.w5 >>> 32) == (bound.w5 >>> 32)

  let b0 := u64ToI64 bound.w0
  let b1 := u64ToI64 bound.w1
  let b2 := u64ToI64 bound.w2
  let bExit := sextI32 (bound.w3 >>> 32)
  let bEnter := sextI32 bound.w3
  let dtMax := sextI32 (bound.w4 >>> 32)
  let uGain := sextI32 bound.w4

  let nextLo : UInt16 := ((bound.w5 >>> 24) &&& 0xFF).toUInt16
  let nextHi : UInt16 := ((bound.w5 >>> 16) &&& 0xFF).toUInt16
  let kanLock : UInt16 := (bound.w5 &&& 0xFFFF).toUInt16
  let reqSieve : UInt32 := (bound.w6 &&& 0xFFFFFFFF).toUInt32

  let horizonOk := decide (dtMax > 0) && decide (bExit <= bEnter)
  let safeDtMax : Int := if dtMax > 0 then dtMax else 1

  let (uScaled, ovU) := qmulI64 uGain uExt
  let (b0Eff, ovB0) := qsubI64 b0 uScaled

  let x := u64ToI64 state.w3
  let opEn := (bound.w1 != 0) || (bound.w2 != 0)

  let (xSq, opOv1) := qmulI64 x x
  let (b2XSq, opOv2) := qmulI64 b2 xSq
  let quadTerm := qshr1I64 b2XSq
  let (b1X, opOv3) := qmulI64 b1 x
  let (linPlusQuad, opOv4) := qaddI64 b1X quadTerm
  let (g0, opOv5) := qaddI64 b0Eff linPlusQuad
  let (b2X, opOv6) := qmulI64 b2 x
  let (g1, opOv7) := qaddI64 b1 b2X
  let opOvf := opOv1 || opOv2 || opOv3 || opOv4 || opOv5 || opOv6 || opOv7

  let (f0, autoOvf) := qsubI64 (u64ToI64 state.w0) b0Eff
  let ovf1 := ovU || ovB0 || (if opEn then opOvf else autoOvf)

  let r0 := if opEn then g0 else f0
  let r1 := if opEn then g1 else u64ToI64 state.w1
  let r2 := if opEn then b2 else u64ToI64 state.w2

  let (r1Sq, _) := qmulI64 r1 r1
  let (r0R2, _) := qmulI64 r0 r2
  let (twoR0R2, _) := qaddI64 r0R2 r0R2
  let (disc, _) := qsubI64 r1Sq twoR0R2

  let dMag := qsqrtU64 (absI64ToU64 disc)
  let signedD := if r1 < 0 then (0 : UInt64) - dMag else dMag
  let qCit := u64ToI64 (i64ToU64 r1 + signedD)
  let (twoR0, _) := qaddI64 r0 r0
  let (negTwoR0, _) := qsubI64 0 twoR0
  let (negQ, _) := qsubI64 0 qCit

  let rootA := qdivI64 negTwoR0 qCit
  let rootB := qdivI64 negQ r2
  let pickA := absI64ToU64 rootA <= absI64ToU64 rootB
  let stepQuad := if pickA then rootA else rootB

  let (negR0, _) := qsubI64 0 r0
  let stepLin := qdivI64 negR0 r1

  let (negR1, _) := qsubI64 0 r1
  let gradNeg := decide (r1 > 0)
  let tunD := u64ToI64 (if gradNeg then (0 : UInt64) - dMag else dMag)
  let (tunNum, _) := qaddI64 negR1 tunD
  let absR2 := absI64ToU64 r2
  let absR2I64 := u64ToI64 (if absR2 > 0x7FFFFFFFFFFFFFFF then 0x7FFFFFFFFFFFFFFF else absR2)
  let stepTunnel := qdivI64 tunNum absR2I64

  let hasR2 := r2 != 0
  let realRoots := decide (disc >= 0)
  let rawStep := if hasR2 then (if realRoots then stepQuad else stepTunnel) else stepLin

  let active := (r0 != 0) && ((r1 != 0) || (r2 != 0))
  let gradDir : Int := if gradNeg then -1 else 1
  let zeroQuant := active && (rawStep == 0)
  let nzStep := if !active then 0 else if zeroQuant then gradDir else rawStep

  let rawAbs := absI64ToU64 nzStep
  let clampHi := rawAbs > i64ToU64 safeDtMax
  let (negDtMax, ovNegDt) := qsubI64 0 safeDtMax
  let signedCap := if nzStep < 0 then negDtMax else safeDtMax
  let delta := if clampHi then signedCap else nzStep
  let dtRes := absI64ToU64 delta
  let (xp, ovXp) := qaddI64 x delta

  let (xpSq, po1) := qmulI64 xp xp
  let (b2XpSq, po2) := qmulI64 b2 xpSq
  let postQuad := qshr1I64 b2XpSq
  let (b1Xp, po3) := qmulI64 b1 xp
  let (postS1, po4) := qaddI64 b1Xp postQuad
  let (j0Op, po5) := qaddI64 b0Eff postS1
  let (b2Xp, po6) := qmulI64 b2 xp
  let (j1Op, po7) := qaddI64 b1 b2Xp
  let postOpOvf := po1 || po2 || po3 || po4 || po5 || po6 || po7

  let (dSq, pu1) := qmulI64 delta delta
  let (r2DSq, pu2) := qmulI64 r2 dSq
  let pushQuad := qshr1I64 r2DSq
  let (r1D, pu3) := qmulI64 r1 delta
  let (pushS1, pu4) := qaddI64 r1D pushQuad
  let (j0Au, pu5) := qaddI64 (u64ToI64 state.w0) pushS1
  let (r2D, pu6) := qmulI64 r2 delta
  let (j1Au, pu7) := qaddI64 (u64ToI64 state.w1) r2D
  let pushOvf := pu1 || pu2 || pu3 || pu4 || pu5 || pu6 || pu7

  let totOvf := ovf1 || ovNegDt || ovXp || (if opEn then postOpOvf else pushOvf)
  let j0Next := if opEn then j0Op else j0Au
  let j1Next := if opEn then j1Op else j1Au
  let j2Next := r2

  let (convDiff, convOvf) := qsubI64 j0Next b0Eff
  let resAfter := if opEn then j0Next else convDiff
  let converged := !(totOvf || convOvf) && (absI64ToU64 resAfter <= 4)

  let chartOrig : UInt16 := ((state.w5 >>> 16) &&& 0xFFFF).toUInt16
  let modeOrig : UInt16 := (state.w5 &&& 0xFFFF).toUInt16
  let kanOk := ((modeOrig ^^^ modeReq) &&& kanLock) == 0
  let modeProj : UInt16 := (modeReq &&& (~~~kanLock)) ||| (modeOrig &&& kanLock)

  let rawChart1 := if xp < bExit then nextLo else chartOrig
  let rawChart2 := if xp > bEnter then nextHi else rawChart1
  let chartNext : UInt16 := (rawChart2.toUInt64 &&& capMask &&& 0xFFFF).toUInt16

  let liveSieve : UInt32 :=
    (if sealOk    then Flow.sieveSealValid    else 0) |||
    (if domOk     then Flow.sieveDomainMatch  else 0) |||
    (if !totOvf   then Flow.sieveNoOverflow   else 0) |||
    (if kanOk     then Flow.sieveKanHomotopic else 0) |||
    (if horizonOk then Flow.sieveHorizonValid else 0) |||
    (if converged then Flow.sieveConverged    else 0)

  let effReq := reqSieve ||| Flow.sieveBaseGate
  let commit := (liveSieve &&& effReq) == effReq

  let auxNext : UInt64 :=
    (state.w5 &&& (0xFFFFFFFF00000000 : UInt64)) ||| (chartNext.toUInt64 <<< 16) ||| modeProj.toUInt64
  let epochNext : UInt32 := (state.w6 >>> 32).toUInt32 + 1
  let omegaNext : UInt64 := (epochNext.toUInt64 <<< 32) ||| liveSieve.toUInt64

  let w0Out := if commit then i64ToU64 j0Next else state.w0
  let w1Out := if commit then i64ToU64 j1Next else state.w1
  let w2Out := if commit then i64ToU64 j2Next else state.w2
  let w3Out := if commit then i64ToU64 xp     else state.w3
  let w4Out := if commit then dtRes           else 0
  let w5Out := if commit then auxNext         else state.w5
  let w6Out := omegaNext
  let w7Out := if stateSealOk then Flow.sealStateLanes w0Out w1Out w2Out w3Out w4Out w5Out w6Out else 0
  { w0 := w0Out, w1 := w1Out, w2 := w2Out, w3 := w3Out,
    w4 := w4Out, w5 := w5Out, w6 := w6Out, w7 := w7Out }

/-- Pure Lean bit-exact mirror of `flow_arena_try_swap` (`flow.h` lines 347-373). -/
def trySwapExact (arena : Array Flow.Cell64) (slot : Nat) (capMask : UInt64)
    (expectedDom newEpoch : UInt32) (cand : Flow.Cell64) : Array Flow.Cell64 × Bool :=
  if slot > capMask.toNat || slot >= arena.size then
    (arena, false)
  else
    let cur := arena[slot]!
    let curEp : UInt32 := (cur.w6 >>> 32).toUInt32
    let candDom : UInt32 := (cand.w5 >>> 32).toUInt32
    let nLo : UInt64 := (cand.w5 >>> 24) &&& 0xFF
    let nHi : UInt64 := (cand.w5 >>> 16) &&& 0xFF
    let bExit := sextI32 (cand.w3 >>> 32)
    let bEnter := sextI32 cand.w3
    let dtMax := sextI32 (cand.w4 >>> 32)
    let reqSv : UInt32 := (cand.w6 &&& 0xFFFFFFFF).toUInt32
    let ok :=
      (cand.w7 != 0) && Flow.verifyBoundCell cand &&
      (candDom == expectedDom) && (newEpoch > curEp) &&
      (nLo <= capMask) && (nHi <= capMask) &&
      decide (dtMax > 0) && decide (bExit <= bEnter) &&
      ((reqSv &&& Flow.sieveBaseGate) == Flow.sieveBaseGate)
    if ok then
      let w6New : UInt64 := (newEpoch.toUInt64 <<< 32) ||| reqSv.toUInt64
      (arena.set! slot { cand with w6 := w6New }, true)
    else
      (arena, false)

private def readLE64 (ba : ByteArray) (off : Nat) : UInt64 := Id.run do
  let mut acc : UInt64 := 0
  for b in [0 : 8] do
    let byteVal : UInt64 := (ba.get! (off + b)).toUInt64
    acc := acc ||| (byteVal <<< (UInt64.ofNat (8 * b)))
  return acc

def cellFromBytes (ba : ByteArray) (off : Nat := 0) : Flow.Cell64 :=
  { w0 := readLE64 ba (off + 0)
    w1 := readLE64 ba (off + 8)
    w2 := readLE64 ba (off + 16)
    w3 := readLE64 ba (off + 24)
    w4 := readLE64 ba (off + 32)
    w5 := readLE64 ba (off + 40)
    w6 := readLE64 ba (off + 48)
    w7 := readLE64 ba (off + 56) }

def loadBundleCells (bundlePath : System.FilePath) : IO (Flow.Cell64 × Array Flow.Cell64) := do
  let ba ← IO.FS.readBinFile bundlePath
  if ba.size < 128 || ba.size % 64 != 0 then
    throw (IO.userError s!"Ele.Silicon: invalid bundle size {ba.size} for {bundlePath}")
  let totalCells := ba.size / 64
  let st := cellFromBytes ba 0
  let mut charts : Array Flow.Cell64 := #[]
  for i in [1 : totalCells] do
    charts := charts.push (cellFromBytes ba (i * 64))
  return (st, charts)

private def hexDigit (n : Nat) : Char :=
  let d := n % 16
  if d < 10 then Char.ofNat ('0'.toNat + d) else Char.ofNat ('a'.toNat + (d - 10))

def hexU8 (v : UInt8) : String :=
  String.ofList [hexDigit (v.toNat / 16), hexDigit (v.toNat % 16)]

def hexU16 (v : UInt16) : String := Id.run do
  let mut cs : List Char := []
  for i in [0 : 4] do
    let shift := (3 - i) * 4
    cs := cs ++ [hexDigit ((v.toNat >>> shift) % 16)]
  return String.ofList cs

def hexU32 (v : UInt32) : String := Id.run do
  let mut cs : List Char := []
  for i in [0 : 8] do
    let shift := (7 - i) * 4
    cs := cs ++ [hexDigit ((v.toNat >>> shift) % 16)]
  return String.ofList cs

def hexU64 (v : UInt64) : String := Id.run do
  let mut cs : List Char := []
  for i in [0 : 16] do
    let shift := (15 - i) * 4
    cs := cs ++ [hexDigit ((v.toNat >>> shift) % 16)]
  return String.ofList cs

/-- Formats a 512-bit `Flow.Cell64` as a 128-hex-character `$readmemh` ROM line (`w7..w0`). -/
def cellToMemhLine (c : Flow.Cell64) : String :=
  hexU64 c.w7 ++ hexU64 c.w6 ++ hexU64 c.w5 ++ hexU64 c.w4 ++
  hexU64 c.w3 ++ hexU64 c.w2 ++ hexU64 c.w1 ++ hexU64 c.w0

/-- Emits a synthesizable FPGA/ASIC `$readmemh` ROM image directly from a `.flow` bundle. -/
def emitBundleMemh (bundlePath memhPath : System.FilePath) : IO Nat := do
  let (st, charts) ← loadBundleCells bundlePath
  let mut lines : Array String := #[cellToMemhLine st]
  for c in charts do
    lines := lines.push (cellToMemhLine c)
  IO.FS.writeFile memhPath (String.intercalate "\n" lines.toList ++ "\n")
  return charts.size

/-- Hardware / CLI co-simulation stimulus event. -/
inductive CosimCmd where
  | step (uExtQ16 : Int) (modeReq? : Option UInt16 := none)
  | swap (slot : UInt8) (epoch : UInt32) (cellPath : System.FilePath)
  deriving Inhabited

private def cmdToMemhLine (cmd : CosimCmd) : IO String := do
  match cmd with
  | .step uExt mode? =>
    let w0 := i64ToU64 uExt
    let w1 : UInt64 :=
      match mode? with
      | some m => m.toUInt64
      | none   => (1 : UInt64) <<< 16 -- `cmd[80] = 1` => use active `state.mode`
    let cell0 : Flow.Cell64 := { w0 := w0, w1 := w1, w2 := 0, w3 := 0, w4 := 0, w5 := 0, w6 := 0, w7 := 0 }
    return hexU8 0x01 ++ hexU8 0x00 ++ hexU32 0 ++ cellToMemhLine cell0
  | .swap slot ep path =>
    let ba ← IO.FS.readBinFile path
    if ba.size != 64 then
      throw (IO.userError s!"Ele.Silicon: swap cell {path} is {ba.size}B (expected 64B)")
    let cand := cellFromBytes ba 0
    return hexU8 0x02 ++ hexU8 slot ++ hexU32 ep ++ cellToMemhLine cand

private def cmdToFlowStdinLine (cmd : CosimCmd) : String :=
  match cmd with
  | .step uExt none     => s!"q16:{uExt}"
  | .step uExt (some m) => s!"q16:{uExt} 0x{hexU16 m}"
  | .swap slot ep path  => s!"@swap {slot.toNat} {ep.toNat} {path}"

private def fmtFlowStepLine (idx : Nat) (st : Flow.Cell64) : String :=
  let chart := ((st.w5 >>> 16) &&& 0xFFFF).toNat
  let mode  := (st.w5 &&& 0xFFFF).toUInt16
  let ep    := (st.w6 >>> 32).toNat
  let sv    := (st.w6 &&& 0xFF).toUInt8
  let xStr  := Unit.formatQ16 (u64ToI64 st.w3) 4
  let dtStr := Unit.formatQ16 (u64ToI64 st.w4) 4
  let r0Str := Unit.formatQ16 (u64ToI64 st.w0) 4
  s!"step={idx} chart={chart} x={xStr} dt={dtStr} r0={r0Str} mode=0x{hexU16 mode} epoch={ep} sieve=0x{hexU8 sv}"

private def parseHexU64? (s : String) : Option UInt64 := do
  let cs := s.trimAscii.toString.toList
  if cs.isEmpty || cs.length > 16 then none
  else
    cs.foldlM (fun acc c =>
      if '0' ≤ c && c ≤ '9' then
        some ((acc <<< 4) ||| UInt64.ofNat (c.toNat - '0'.toNat))
      else if 'a' ≤ c && c ≤ 'f' then
        some ((acc <<< 4) ||| UInt64.ofNat (c.toNat - 'a'.toNat + 10))
      else if 'A' ≤ c && c ≤ 'F' then
        some ((acc <<< 4) ||| UInt64.ofNat (c.toNat - 'A'.toNat + 10))
      else none) (0 : UInt64)

private def runSubproc (bin : System.FilePath) (args : Array String) (stdinStr : String := "") :
    IO String := do
  let child ← IO.Process.spawn {
    cmd    := bin.toString
    args   := args
    stdin  := .piped
    stdout := .piped
    stderr := .piped
  }
  let (stdinH, child) ← child.takeStdin
  if stdinStr != "" then
    stdinH.putStr stdinStr
  stdinH.flush
  let outTask ← IO.asTask child.stdout.readToEnd Task.Priority.dedicated
  let errStr ← child.stderr.readToEnd
  let code ← child.wait
  let outStr ← IO.ofExcept outTask.get
  if code != 0 then
    throw (IO.userError s!"subprocess '{bin}' failed (exit {code}):\n{errStr}\n{outStr}")
  return outStr

/-- Runs the hardware EM PHY self-test (`ele_bus_invert65` & `ele_ternary_mac_dualrail`). -/
def verifyEmPhyRtl (cosimBin : System.FilePath := "build/verilator/Vele_cosim_tb") :
    IO String := do
  let out ← runSubproc cosimBin #["+PHY_TEST"]
  let passLine? := (out.splitOn "\n").find? (fun l => l.startsWith "PHY_PASS ")
  match passLine? with
  | some ln => return ln.trimAscii.toString
  | none    => throw (IO.userError s!"EM PHY RTL test did not emit PHY_PASS:\n{out}")

/--
  Executes 3-way `0-ULP` differential co-verification on any `.flow` Manifold Atlas:
    1. `Flow.lean` + `Ele.Silicon.stepChartExact` (pure Lean 512-bit formal model)
    2. `FLOW/build/flow run` (frozen C99 reference engine)
    3. `rtl/ele_flow_core.sv` (`ele_flow_atlas_top` simulated by Verilator `--binary`)
  Asserts exact agreement across all `SWAP` outcomes, `FLOW/build/flow` stdout lines,
  and all 512 bits (`w0..w7`, including Goldilocks `F_p` seals) on every step.
-/
def verifyAtlasCosim (label : String) (bundlePath : System.FilePath)
    (cmds : List CosimCmd)
    (workDir : System.FilePath := "build/cosim")
    (cosimBin : System.FilePath := "build/verilator/Vele_cosim_tb") : IO Nat := do
  IO.FS.createDirAll workDir
  let bundleMemh := workDir / s!"{label}_bundle.memh"
  let stimMemh   := workDir / s!"{label}_stim.memh"
  let chartCount ← emitBundleMemh bundlePath bundleMemh

  let mut stimLines : Array String := #[]
  for cmd in cmds do
    stimLines := stimLines.push (← cmdToMemhLine cmd)
  IO.FS.writeFile stimMemh (String.intercalate "\n" stimLines.toList ++ "\n")

  -- 1. Run frozen upstream `FLOW/build/flow` C99 binary
  let flowBin ← Unit.resolveFlowBin
  let flowStdin := String.intercalate "\n" (cmds.map cmdToFlowStdinLine) ++ "\n"
  let flowStdout ← runSubproc flowBin #["run", bundlePath.toString] flowStdin
  let flowLines := (flowStdout.splitOn "\n").map (fun s => s.trimAscii.toString) |>.filter (· != "")

  -- 2. Run synthesizable SystemVerilog RTL (`ele_flow_atlas_top`) via Verilator
  let rtlStdout ← runSubproc cosimBin #[
    s!"+BUNDLE={bundleMemh}",
    s!"+CHARTS={chartCount}",
    s!"+CMDS={cmds.length}",
    s!"+STIM={stimMemh}"
  ]
  let rtlLines := (rtlStdout.splitOn "\n")
    |>.map (fun s => s.trimAscii.toString)
    |>.filter (fun l => l.startsWith "STEP " || l.startsWith "SWAP ")

  if rtlLines.length != cmds.length || flowLines.length != cmds.length then
    throw (IO.userError
      s!"[{label}] line count mismatch: cmds={cmds.length}, flow={flowLines.length}, rtl={rtlLines.length}")

  -- 3. Step pure-Lean 512-bit reference & assert 3-way 0-ULP bit-identity
  let (stInit, chartsInit) ← loadBundleCells bundlePath
  let capMask : UInt64 := UInt64.ofNat (chartCount - 1)
  let domExpected : UInt32 := (stInit.w5 >>> 32).toUInt32
  let mut stLean := stInit
  let mut arenaLean := chartsInit
  let mut stepIdx : Nat := 0

  for i in [0 : cmds.length] do
    let cmd := cmds[i]!
    let cLine := flowLines[i]!
    let vLine := rtlLines[i]!
    match cmd with
    | .swap slot ep path =>
      let ba ← IO.FS.readBinFile path
      let cand := cellFromBytes ba 0
      let (arenaNext, leanOk) := trySwapExact arenaLean slot.toNat capMask domExpected ep cand
      arenaLean := arenaNext
      let expFlow :=
        if leanOk then s!"# swap slot={slot.toNat} epoch={ep.toNat} ok"
        else s!"# swap slot={slot.toNat} epoch={ep.toNat} rejected"
      let expRtl := s!"SWAP slot={slot.toNat} epoch={ep.toNat} ok={if leanOk then 1 else 0}"
      if cLine != expFlow || vLine != expRtl then
        throw (IO.userError
          s!"[{label}] SWAP mismatch at cmd {i}:\n  lean={expRtl}\n  flow={cLine}\n  rtl ={vLine}")
    | .step uExt mode? =>
      let curMode : UInt16 := (stLean.w5 &&& 0xFFFF).toUInt16
      let modeReq := mode?.getD curMode
      let chartIdx := (((stLean.w5 >>> 16) &&& 0xFFFF) &&& capMask).toNat
      let bound := arenaLean[chartIdx]!
      stLean := stepChartExact stLean bound capMask uExt modeReq
      stepIdx := stepIdx + 1

      -- Parse 512-bit `w7..w0` from Verilator RTL line
      let kvs := (vLine.splitOn " ").filterMap fun tok =>
        match tok.splitOn "=" with
        | [k, v] => some (k, v)
        | _      => none
      let getW (k : String) : IO UInt64 := do
        match kvs.lookup k >>= parseHexU64? with
        | some w => return w
        | none   => throw (IO.userError s!"[{label}] malformed RTL word '{k}' in: {vLine}")
      let rtlCell : Flow.Cell64 := {
        w0 := ← getW "w0", w1 := ← getW "w1", w2 := ← getW "w2", w3 := ← getW "w3",
        w4 := ← getW "w4", w5 := ← getW "w5", w6 := ← getW "w6", w7 := ← getW "w7"
      }

      if !Flow.verifyStateCell rtlCell then
        throw (IO.userError s!"[{label}] RTL Goldilocks state seal w7 invalid at step {stepIdx}: {vLine}")
      if rtlCell != stLean then
        throw (IO.userError
          s!"[{label}] 512-bit Lean-vs-RTL mismatch at step {stepIdx}:\n  lean={cellToMemhLine stLean}\n  rtl ={cellToMemhLine rtlCell}")
      let rtlFormatted := fmtFlowStepLine stepIdx rtlCell
      if rtlFormatted != cLine then
        throw (IO.userError
          s!"[{label}] C-vs-RTL stdout mismatch at step {stepIdx}:\n  flow={cLine}\n  rtl ={rtlFormatted}")

  return stepIdx

end Silicon
end Ele
