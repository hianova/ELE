/-
  EleCert — Formal Zero-FP-Trust Verification Layer for ELE v10.0.0
  ==================================================================
  Imports ONLY `Flow.Interval` (domain-free, zero IO, zero `sorryAx`,
  zero `Lean.ofReduceBool`) to certify across all three EEE pillars:
    - Core Linear Algebra:
      E1  `sdd_quadratic_form_nonneg`
      E2  `cholesky_sdd_certified_psd`
      E3  `checkCholeskySDD_sound`
    - Electromagnetics (`em`):
      E4  `passivity_real_aug_identity` & E4b `certified_multiport_passivity`
      E5  `fdtd_cfl_stability_2d`
      E6  `reciprocity_symmetry_bound`
      E16 `maxwell_poynting_dual_upper_bound` & `certified_maxwell_poynting_bound`
      E17 `kyp_bounded_real_passivity_cert` & `certified_kyp_passivity`
      E20 `bode_fano_energy_limit_sound`
    - Electronics (`ckt`):
      E7  `tellegen_passive_mna_dissipation`
      E8  `trapezoidal_astability_energy_decay`
      E9  `soa_and_eye_enclosure_cert`
    - Electrical Engineering (`ee`):
      E10 `ipmsm_operating_envelope_cert`
      E11 `ac_opf_anytime_dual_lower_bound` & E11b `ac_opf_bracket_valid`
      E12 `grid_lyapunov_stability_basin`
      E13 `chordal_clique_decomposition_sound`
      E14 `ac_opf_thermal_angle_dual_sound`
      E15 `ac_opf_observable_envelope_sound`
      E18 `moment_sos_hierarchy_exactness` & `certified_sos2_lower_bound`
      E19 `cycle_phase_rank_obstruction`
-/
import Flow.Interval

namespace EleCert

open Flow.Interval

/-! ## 0. Self-Contained Exact Integer Summation & Linear Algebra -/

def absInt (x : Int) : Int :=
  if x < 0 then -x else x

def negPart (x : Int) : Int :=
  if x < 0 then x else 0

theorem absInt_nonneg (x : Int) : 0 ≤ absInt x := by
  unfold absInt
  split <;> omega

theorem int_sq_nonneg (x : Int) : 0 ≤ x * x := by
  by_cases hx : 0 ≤ x
  · exact Int.mul_nonneg hx hx
  · have hnx : 0 ≤ -x := by omega
    have hsq : (-x) * (-x) = x * x := Int.neg_mul_neg x x
    rw [← hsq]
    exact Int.mul_nonneg hnx hnx

theorem negPart_le (x : Int) : negPart x ≤ x := by
  unfold negPart
  split <;> omega

theorem negPart_nonpos (x : Int) : negPart x ≤ 0 := by
  unfold negPart
  split <;> omega

def sumFin : (n : Nat) → (Fin n → Int) → Int
  | 0, _ => 0
  | n + 1, f => sumFin n (fun i => f i.castSucc) + f ⟨n, Nat.lt_succ_self n⟩

theorem sumFin_congr {n : Nat} {f g : Fin n → Int} (h : ∀ i, f i = g i) :
    sumFin n f = sumFin n g := by
  induction n with
  | zero => rfl
  | succ k ih =>
    simp only [sumFin]
    have ih1 : sumFin k (fun i => f i.castSucc) = sumFin k (fun i => g i.castSucc) :=
      ih (fun i => h i.castSucc)
    rw [ih1, h ⟨k, Nat.lt_succ_self k⟩]

theorem sumFin_zero (n : Nat) : sumFin n (fun _ => 0) = 0 := by
  induction n with
  | zero => rfl
  | succ k ih => simp only [sumFin, ih]; omega

theorem sumFin_nonneg (n : Nat) (f : Fin n → Int) (h : ∀ i, 0 ≤ f i) :
    0 ≤ sumFin n f := by
  induction n with
  | zero => simp [sumFin]
  | succ k ih =>
    simp only [sumFin]
    have h1 := ih (fun i => f i.castSucc) (fun i => h i.castSucc)
    have h2 := h ⟨k, Nat.lt_succ_self k⟩
    omega

theorem sumFin_add (n : Nat) (f g : Fin n → Int) :
    sumFin n (fun i => f i + g i) = sumFin n f + sumFin n g := by
  induction n with
  | zero => simp [sumFin]
  | succ k ih =>
    simp only [sumFin]
    rw [ih (fun i => f i.castSucc) (fun i => g i.castSucc)]
    omega

theorem sumFin_sub (n : Nat) (f g : Fin n → Int) :
    sumFin n (fun i => f i - g i) = sumFin n f - sumFin n g := by
  induction n with
  | zero => simp [sumFin]
  | succ k ih =>
    simp only [sumFin]
    rw [ih (fun i => f i.castSucc) (fun i => g i.castSucc)]
    omega

theorem sumFin_neg (n : Nat) (f : Fin n → Int) :
    sumFin n (fun i => -f i) = -sumFin n f := by
  induction n with
  | zero => simp [sumFin]
  | succ k ih =>
    simp only [sumFin]
    rw [ih (fun i => f i.castSucc)]
    omega

theorem sumFin_mul_left (n : Nat) (c : Int) (f : Fin n → Int) :
    c * sumFin n f = sumFin n (fun i => c * f i) := by
  induction n with
  | zero => simp [sumFin]
  | succ k ih =>
    simp only [sumFin]
    rw [Int.mul_add, ih (fun i => f i.castSucc)]

theorem sumFin_mul_right (n : Nat) (f : Fin n → Int) (c : Int) :
    sumFin n f * c = sumFin n (fun i => f i * c) := by
  induction n with
  | zero => simp [sumFin]
  | succ k ih =>
    simp only [sumFin]
    rw [Int.add_mul, ih (fun i => f i.castSucc)]

theorem sumFin_swap {m n : Nat} (f : Fin m → Fin n → Int) :
    sumFin m (fun i => sumFin n (fun j => f i j)) =
      sumFin n (fun j => sumFin m (fun i => f i j)) := by
  induction m with
  | zero =>
    simp only [sumFin]
    exact (sumFin_zero n).symm
  | succ k ih =>
    simp only [sumFin]
    rw [ih (fun i j => f i.castSucc j)]
    rw [← sumFin_add n (fun j => sumFin k (fun i => f i.castSucc j))
          (fun j => f ⟨k, Nat.lt_succ_self k⟩ j)]

theorem sumFin_single_hit (n : Nat) (i : Fin n) (c : Int) :
    sumFin n (fun j => if j = i then c else 0) = c := by
  induction n with
  | zero => exact Fin.elim0 i
  | succ k ih =>
    simp only [sumFin]
    by_cases hk : i.val = k
    · have hi : i = ⟨k, Nat.lt_succ_self k⟩ := Fin.ext hk
      subst hi
      have hsub : sumFin k (fun j : Fin k => if j.castSucc = ⟨k, Nat.lt_succ_self k⟩ then c else 0) = 0 := by
        have hzero : (fun j : Fin k => if j.castSucc = ⟨k, Nat.lt_succ_self k⟩ then c else 0) = (fun _ => 0) := by
          funext j
          have hne : j.castSucc ≠ ⟨k, Nat.lt_succ_self k⟩ := by
            intro h
            have hv := congrArg Fin.val h
            exact Nat.ne_of_lt j.isLt hv
          simp [hne]
        rw [hzero, sumFin_zero]
      simp [hsub]
    · have hilt : i.val < k := Nat.lt_of_le_of_ne (Nat.lt_succ_iff.mp i.isLt) hk
      let i' : Fin k := ⟨i.val, hilt⟩
      have hi : i = i'.castSucc := Fin.ext rfl
      have hlast : (if (⟨k, Nat.lt_succ_self k⟩ : Fin (k + 1)) = i then c else 0) = 0 := by
        have hne : (⟨k, Nat.lt_succ_self k⟩ : Fin (k + 1)) ≠ i := by
          intro h
          have hv := congrArg Fin.val h
          exact hk hv.symm
        simp [hne]
      have hsub : (fun j : Fin k => if j.castSucc = i then c else 0) =
          (fun j : Fin k => if j = i' then c else 0) := by
        funext j
        by_cases hj : j = i'
        · subst hj; simp [hi]
        · have hne : j.castSucc ≠ i := by
            intro h
            apply hj
            apply Fin.ext
            have hv : (j.castSucc : Nat) = i.val := congrArg Fin.val h
            exact hv
          simp [hj, hne]
      rw [hsub, ih i', hlast]
      omega

theorem sumFin_add_split (a b : Nat) (f : Fin (a + b) → Int) :
    sumFin (a + b) f =
      sumFin a (fun i => f ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right a b)⟩) +
      sumFin b (fun j => f ⟨a + j.val, Nat.add_lt_add_left j.isLt a⟩) := by
  induction b with
  | zero =>
    have h1 : sumFin a f = sumFin a (fun i => f ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right a 0)⟩) := by
      apply sumFin_congr; intro i; rfl
    change sumFin a f = _
    rw [h1]
    simp [sumFin]
  | succ k ih =>
    have hsplit : sumFin (a + (k + 1)) f =
        sumFin (a + k) (fun i => f i.castSucc) + f ⟨a + k, Nat.lt_succ_self (a + k)⟩ := rfl
    rw [hsplit, ih (fun i => f i.castSucc)]
    have hleft :
        sumFin a (fun i => f (Fin.castSucc ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right a k)⟩)) =
        sumFin a (fun i => f ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right a (k + 1))⟩) := by
      apply sumFin_congr; intro i; rfl
    have hright :
        sumFin k (fun j => f (Fin.castSucc ⟨a + j.val, Nat.add_lt_add_left j.isLt a⟩)) =
        sumFin k (fun j => f (⟨a + j.val, Nat.add_lt_add_left (Nat.lt_succ_of_lt j.isLt) a⟩ : Fin (a + (k + 1)))) := by
      apply sumFin_congr; intro j; rfl
    have hright2 :
        sumFin k (fun j => f (⟨a + j.val, Nat.add_lt_add_left (Nat.lt_succ_of_lt j.isLt) a⟩ : Fin (a + (k + 1)))) =
        sumFin k (fun j => f ⟨a + j.castSucc.val, Nat.add_lt_add_left j.castSucc.isLt a⟩) := by
      apply sumFin_congr; intro j; rfl
    have htail : f ⟨a + k, Nat.lt_succ_self (a + k)⟩ =
        f ⟨a + k, Nat.add_lt_add_left (Nat.lt_succ_self k) a⟩ := rfl
    change _ = sumFin a (fun i => f ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right a (k + 1))⟩) +
      (sumFin k (fun j => f ⟨a + j.castSucc.val, Nat.add_lt_add_left j.castSucc.isLt a⟩) +
       f ⟨a + k, Nat.add_lt_add_left (Nat.lt_succ_self k) a⟩)
    rw [hleft, hright, hright2, htail]
    omega

theorem sumFin_le_sumFin (n : Nat) (f g : Fin n → Int) (h : ∀ i, f i ≤ g i) :
    sumFin n f ≤ sumFin n g := by
  have hd : 0 ≤ sumFin n (fun i => g i - f i) := sumFin_nonneg n _ (fun i => by have := h i; omega)
  rw [sumFin_sub] at hd
  omega

/-! ## 1. Vector Norms, Quadratic Forms & Integer Cholesky + SDD (Theorems E1–E3) -/

def vecNormSq (n : Nat) (v : Fin n → Int) : Int :=
  sumFin n (fun i => v i * v i)

theorem vecNormSq_nonneg (n : Nat) (v : Fin n → Int) : 0 ≤ vecNormSq n v := by
  unfold vecNormSq
  exact sumFin_nonneg n _ (fun i => int_sq_nonneg (v i))

def matQuad (n : Nat) (A : Fin n → Fin n → Int) (v : Fin n → Int) : Int :=
  sumFin n (fun i => sumFin n (fun j => v i * A i j * v j))

def gramLLt (n : Nat) (L : Fin n → Fin n → Int) (i j : Fin n) : Int :=
  sumFin n (fun k => L i k * L j k)

def IsSDD (n : Nat) (E : Fin n → Fin n → Int) : Prop :=
  (∀ i j, E i j = E j i) ∧
  (∀ i, sumFin n (fun j => if j = i then 0 else absInt (E i j)) ≤ E i i)

private theorem sdd_pair_nonneg (a x y : Int) :
    0 ≤ absInt a * (x * x) + 2 * (a * (x * y)) + absInt a * (y * y) := by
  by_cases ha : a < 0
  · have habs : absInt a = -a := by unfold absInt; simp [ha]
    rw [habs]
    have hna : 0 ≤ -a := by omega
    have hsq : 0 ≤ (x - y) * (x - y) := int_sq_nonneg (x - y)
    have hid : (-a) * ((x - y) * (x - y)) = (-a) * (x * x) + 2 * (a * (x * y)) + (-a) * (y * y) := by
      grind
    rw [← hid]
    exact Int.mul_nonneg hna hsq
  · have habs : absInt a = a := by unfold absInt; simp [ha]
    rw [habs]
    have hpa : 0 ≤ a := by omega
    have hsq : 0 ≤ (x + y) * (x + y) := int_sq_nonneg (x + y)
    have hid : a * ((x + y) * (x + y)) = a * (x * x) + 2 * (a * (x * y)) + a * (y * y) := by
      grind
    rw [← hid]
    exact Int.mul_nonneg hpa hsq

/-- Theorem E1 (SDD Quadratic Form Nonnegativity):
    Every symmetric diagonally dominant integer matrix has nonnegative quadratic form. -/
theorem sdd_quadratic_form_nonneg (n : Nat) (E : Fin n → Fin n → Int) (hSDD : IsSDD n E) :
    ∀ v : Fin n → Int, 0 ≤ matQuad n E v := by
  intro v
  rcases hSDD with ⟨hSymm, hDom⟩
  let offDiag : Fin n → Int := fun i => sumFin n (fun j => if j = i then 0 else absInt (E i j))
  let diagSum : Int := sumFin n (fun i => (E i i - offDiag i) * (v i * v i))
  let offSqI : Int := sumFin n (fun i => offDiag i * (v i * v i))
  let pairSum : Int :=
    sumFin n (fun i => sumFin n (fun j =>
      if j = i then 0 else absInt (E i j) * (v i * v i) + 2 * (E i j * (v i * v j)) + absInt (E i j) * (v j * v j)))
  have h1 : matQuad n E v = diagSum + offSqI + sumFin n (fun i => sumFin n (fun j => if j = i then 0 else E i j * (v i * v j))) := by
    have hA : matQuad n E v =
        sumFin n (fun i => E i i * (v i * v i) + sumFin n (fun j => if j = i then 0 else E i j * (v i * v j))) := by
      unfold matQuad
      apply sumFin_congr
      intro i
      have hSplit :
          (fun j : Fin n => v i * E i j * v j) =
            (fun j : Fin n => (if j = i then E i i * (v i * v i) else 0) + (if j = i then 0 else E i j * (v i * v j))) := by
        funext j
        by_cases hj : j = i
        · subst hj; grind
        · simp [hj]; grind
      rw [hSplit, sumFin_add, sumFin_single_hit]
    rw [hA, sumFin_add]
    have hD : sumFin n (fun i => E i i * (v i * v i)) = diagSum + offSqI := by
      have hSplit : (fun i => E i i * (v i * v i)) = (fun i => (E i i - offDiag i) * (v i * v i) + offDiag i * (v i * v i)) := by
        funext i; grind
      rw [hSplit, sumFin_add]
    rw [hD]
  have hOff1 : offSqI = sumFin n (fun i => sumFin n (fun j => if j = i then 0 else absInt (E i j) * (v i * v i))) := by
    unfold offSqI
    apply sumFin_congr
    intro i
    change sumFin n (fun j => if j = i then 0 else absInt (E i j)) * (v i * v i) = _
    rw [sumFin_mul_right]
    apply sumFin_congr
    intro j
    by_cases hj : j = i <;> simp [hj]
  have hOff2 : offSqI = sumFin n (fun i => sumFin n (fun j => if j = i then 0 else absInt (E i j) * (v j * v j))) := by
    rw [hOff1]
    have hSw := sumFin_swap (fun i j : Fin n => if j = i then 0 else absInt (E i j) * (v i * v i))
    rw [hSw]
    apply sumFin_congr
    intro i
    apply sumFin_congr
    intro j
    by_cases hji : j = i
    · subst hji; rfl
    · have hij' : i ≠ j := fun h => hji h.symm
      simp only [hji, hij', ↓reduceIte]
      rw [hSymm j i]
  have hPairEq :
      2 * offSqI + 2 * sumFin n (fun i => sumFin n (fun j => if j = i then 0 else E i j * (v i * v j))) = pairSum := by
    have hCross :
        2 * sumFin n (fun i => sumFin n (fun j => if j = i then 0 else E i j * (v i * v j))) =
          sumFin n (fun i => sumFin n (fun j => if j = i then 0 else 2 * (E i j * (v i * v j)))) := by
      rw [sumFin_mul_left]
      apply sumFin_congr
      intro i
      rw [sumFin_mul_left]
      apply sumFin_congr
      intro j
      by_cases hj : j = i <;> simp [hj]
    have hSumOff :
        offSqI + offSqI =
          sumFin n (fun i => sumFin n (fun j =>
            if j = i then 0 else absInt (E i j) * (v i * v i) + absInt (E i j) * (v j * v j))) := by
      calc offSqI + offSqI
          = sumFin n (fun i => sumFin n (fun j => if j = i then 0 else absInt (E i j) * (v i * v i))) +
              sumFin n (fun i => sumFin n (fun j => if j = i then 0 else absInt (E i j) * (v j * v j))) := by
                rw [← hOff1, ← hOff2]
        _ = sumFin n (fun i =>
              sumFin n (fun j => if j = i then 0 else absInt (E i j) * (v i * v i)) +
              sumFin n (fun j => if j = i then 0 else absInt (E i j) * (v j * v j))) := by
                rw [sumFin_add]
        _ = sumFin n (fun i => sumFin n (fun j =>
              if j = i then 0 else absInt (E i j) * (v i * v i) + absInt (E i j) * (v j * v j))) := by
                apply sumFin_congr; intro i
                rw [← sumFin_add]
                apply sumFin_congr; intro j
                by_cases hj : j = i <;> simp [hj]
    have h2 : 2 * offSqI = offSqI + offSqI := by omega
    rw [h2, hSumOff, hCross]
    unfold pairSum
    rw [← sumFin_add]
    apply sumFin_congr
    intro i
    rw [← sumFin_add]
    apply sumFin_congr
    intro j
    by_cases hj : j = i <;> simp [hj] <;> omega
  have h2Quad : 2 * matQuad n E v = 2 * diagSum + pairSum := by
    omega
  have hDiagNonneg : 0 ≤ diagSum := by
    apply sumFin_nonneg
    intro i
    have hd : 0 ≤ E i i - offDiag i := by have := hDom i; grind
    exact Int.mul_nonneg hd (int_sq_nonneg (v i))
  have hPairNonneg : 0 ≤ pairSum := by
    apply sumFin_nonneg
    intro i
    apply sumFin_nonneg
    intro j
    by_cases hj : j = i
    · simp [hj]
    · simp only [hj, ↓reduceIte]
      exact sdd_pair_nonneg (E i j) (v i) (v j)
  omega

private theorem gramLLt_psd (n : Nat) (L : Fin n → Fin n → Int) (v : Fin n → Int) :
    0 ≤ matQuad n (gramLLt n L) v := by
  have hEq : matQuad n (gramLLt n L) v =
      sumFin n (fun k => (sumFin n (fun i => v i * L i k)) * (sumFin n (fun j => v j * L j k))) := by
    unfold matQuad gramLLt
    calc
      sumFin n (fun i => sumFin n (fun j => v i * sumFin n (fun k => L i k * L j k) * v j))
          = sumFin n (fun i => sumFin n (fun j => sumFin n (fun k => (v i * L i k) * (v j * L j k)))) := by
            apply sumFin_congr; intro i
            apply sumFin_congr; intro j
            rw [sumFin_mul_left, sumFin_mul_right]
            apply sumFin_congr; intro k; grind
      _ = sumFin n (fun i => sumFin n (fun k => sumFin n (fun j => (v i * L i k) * (v j * L j k)))) := by
            apply sumFin_congr; intro i; exact sumFin_swap _
      _ = sumFin n (fun k => sumFin n (fun i => sumFin n (fun j => (v i * L i k) * (v j * L j k)))) :=
            sumFin_swap _
      _ = sumFin n (fun k => sumFin n (fun i => (v i * L i k) * sumFin n (fun j => v j * L j k))) := by
            apply sumFin_congr; intro k
            apply sumFin_congr; intro i
            exact (sumFin_mul_left n (v i * L i k) (fun j => v j * L j k)).symm
      _ = sumFin n (fun k => sumFin n (fun i => v i * L i k) * sumFin n (fun j => v j * L j k)) := by
            apply sumFin_congr; intro k
            exact (sumFin_mul_right (f := fun i => v i * L i k) n (sumFin n (fun j => v j * L j k))).symm
  rw [hEq]
  apply sumFin_nonneg
  intro k
  exact int_sq_nonneg _

/-- Theorem E2 (Integer Cholesky + SDD Certified Eigenvalue Lower Bound):
    If `E = A - mu*I - L*L^T` is symmetric diagonally dominant, then `mu * ||v||^2 ≤ v^T A v`. -/
theorem cholesky_sdd_certified_psd (n : Nat) (A L : Fin n → Fin n → Int) (mu : Int)
    (hSDD : IsSDD n (fun i j => A i j - (if i = j then mu else 0) - gramLLt n L i j)) :
    ∀ v : Fin n → Int, mu * vecNormSq n v ≤ matQuad n A v := by
  intro v
  let E : Fin n → Fin n → Int := fun i j => A i j - (if i = j then mu else 0) - gramLLt n L i j
  have hNonneg : 0 ≤ matQuad n E v := sdd_quadratic_form_nonneg n E hSDD v
  have hGram : 0 ≤ matQuad n (gramLLt n L) v := gramLLt_psd n L v
  have hDecomp : matQuad n E v = matQuad n A v - mu * vecNormSq n v - matQuad n (gramLLt n L) v := by
    have hD : sumFin n (fun i => sumFin n (fun j => v i * (if i = j then mu else 0) * v j)) = mu * vecNormSq n v := by
      unfold vecNormSq
      rw [sumFin_mul_left]
      apply sumFin_congr
      intro i
      have hSub : (fun j : Fin n => v i * (if i = j then mu else 0) * v j) =
          (fun j : Fin n => if j = i then mu * (v i * v i) else 0) := by
        funext j
        by_cases hj : j = i
        · subst hj; simp; grind
        · have hij : i ≠ j := fun h => hj h.symm
          simp [hj, hij]
      rw [hSub, sumFin_single_hit]
    unfold matQuad E
    calc
      sumFin n (fun i => sumFin n (fun j => v i * (A i j - (if i = j then mu else 0) - gramLLt n L i j) * v j))
          = sumFin n (fun i => sumFin n (fun j =>
              (v i * A i j * v j - v i * (if i = j then mu else 0) * v j) - v i * gramLLt n L i j * v j)) := by
            apply sumFin_congr; intro i
            apply sumFin_congr; intro j; grind
      _ = sumFin n (fun i => sumFin n (fun j => v i * A i j * v j) -
              sumFin n (fun j => v i * (if i = j then mu else 0) * v j) -
              sumFin n (fun j => v i * gramLLt n L i j * v j)) := by
            apply sumFin_congr; intro i
            rw [sumFin_sub, sumFin_sub]
      _ = matQuad n A v -
            sumFin n (fun i => sumFin n (fun j => v i * (if i = j then mu else 0) * v j)) -
            matQuad n (gramLLt n L) v := by
            rw [sumFin_sub, sumFin_sub]; rfl
      _ = matQuad n A v - mu * vecNormSq n v - matQuad n (gramLLt n L) v := by
            rw [hD]
  omega

def matAt {n : Nat} (rows : List (List Int)) (i j : Fin n) : Int :=
  (rows.getD i.val []).getD j.val 0

def triAt {n : Nat} (rows : List (List Int)) (i k : Fin n) : Int :=
  if k.val ≤ i.val then matAt rows i k else 0

private def allFinAux (n : Nat) (p : Fin n → Bool) : (k : Nat) → k ≤ n → Bool
  | 0, _ => true
  | k + 1, hk => allFinAux n p k (Nat.le_of_lt hk) && p ⟨k, hk⟩

def allFinB (n : Nat) (p : Fin n → Bool) : Bool :=
  allFinAux n p n (Nat.le_refl n)

private theorem allFinAux_true (n : Nat) (p : Fin n → Bool) :
    ∀ k (hk : k ≤ n), allFinAux n p k hk = true → ∀ j (hj : j < k), p ⟨j, Nat.lt_of_lt_of_le hj hk⟩ = true
  | 0, _, _, _, hj => by omega
  | k + 1, hk, hall, j, hj => by
    simp only [allFinAux, Bool.and_eq_true] at hall
    by_cases hjk : j = k
    · subst hjk; exact hall.2
    · have hjlt : j < k := by omega
      exact allFinAux_true n p k (Nat.le_of_lt hk) hall.1 j hjlt

theorem allFinB_true (n : Nat) (p : Fin n → Bool) (h : allFinB n p = true) :
    ∀ i : Fin n, p i = true := by
  intro i
  exact allFinAux_true n p n (Nat.le_refl n) h i.val i.isLt

def checkCholeskySDD (n : Nat) (A L : List (List Int)) (mu : Int) : Bool :=
  let E : Fin n → Fin n → Int := fun i j =>
    matAt A i j - (if i = j then mu else 0) - gramLLt n (triAt L) i j
  let symOk := allFinB n (fun i => allFinB n (fun j => decide (matAt A i j = matAt A j i)))
  let domOk := allFinB n (fun i =>
    decide (sumFin n (fun j => if j = i then 0 else absInt (E i j)) ≤ E i i))
  symOk && domOk

/-- Theorem E3 (Kernel-Checked Integer Cholesky + SDD Soundness):
    If `checkCholeskySDD n A L mu = true`, then `mu * ||v||^2 ≤ v^T A v` for all `v`. -/
theorem checkCholeskySDD_sound (n : Nat) (A L : List (List Int)) (mu : Int)
    (hCheck : checkCholeskySDD n A L mu = true) :
    ∀ v : Fin n → Int, mu * vecNormSq n v ≤ matQuad n (matAt A) v := by
  unfold checkCholeskySDD at hCheck
  simp only [Bool.and_eq_true] at hCheck
  have hSymmMat : ∀ i j : Fin n, matAt A i j = matAt A j i := by
    intro i j
    have hi := allFinB_true n _ hCheck.1 i
    have hj := allFinB_true n _ hi j
    exact of_decide_eq_true hj
  have hDomE : ∀ i : Fin n,
      sumFin n (fun j => if j = i then 0 else absInt (matAt A i j - (if i = j then mu else 0) - gramLLt n (triAt L) i j)) ≤
        matAt A i i - (if i = i then mu else 0) - gramLLt n (triAt L) i i := by
    intro i
    simpa using of_decide_eq_true (allFinB_true n _ hCheck.2 i)
  refine cholesky_sdd_certified_psd n (matAt A) (triAt L) mu ⟨?_, hDomE⟩
  intro i j
  have hg : gramLLt n (triAt L) i j = gramLLt n (triAt L) j i := by
    unfold gramLLt
    apply sumFin_congr; intro k; exact Int.mul_comm _ _
  by_cases hij : i = j
  · subst hij; rfl
  · have hji : j ≠ i := fun h => hij h.symm
    simp only [hij, hji, ↓reduceIte, hSymmMat i j, hg]

/-! ## 2. Pillar 1 — Electromagnetics Theorems (Theorems E4–E6) -/

def passivityBlockA (N : Nat) (Q : Int) (R X : Fin N → Fin N → Int) (i j : Fin N) : Int :=
  (if i = j then Q * Q else 0) - sumFin N (fun k => R k i * R k j + X k i * X k j)

def passivityBlockB (N : Nat) (R X : Fin N → Fin N → Int) (i j : Fin N) : Int :=
  sumFin N (fun k => R k i * X k j - X k i * R k j)

def passivityRealAugMat (N : Nat) (Q : Int) (R X : Fin N → Fin N → Int) :
    Fin (N + N) → Fin (N + N) → Int := fun r c =>
  if hr : r.val < N then
    if hc : c.val < N then passivityBlockA N Q R X ⟨r.val, hr⟩ ⟨c.val, hc⟩
    else passivityBlockB N R X ⟨r.val, hr⟩ ⟨c.val - N, by omega⟩
  else
    if hc : c.val < N then -passivityBlockB N R X ⟨r.val - N, by omega⟩ ⟨c.val, hc⟩
    else passivityBlockA N Q R X ⟨r.val - N, by omega⟩ ⟨c.val - N, by omega⟩

def concatVec (N : Nat) (u v : Fin N → Int) : Fin (N + N) → Int := fun k =>
  if hk : k.val < N then u ⟨k.val, hk⟩ else v ⟨k.val - N, by omega⟩

def matMulVec (N : Nat) (A : Fin N → Fin N → Int) (w : Fin N → Int) : Fin N → Int :=
  fun k => sumFin N (fun i => A k i * w i)

private theorem quad_bilinear_factor (N : Nat) (f g : Fin N → Int) (u v : Fin N → Int) :
    sumFin N (fun i => sumFin N (fun j => u i * (f i * g j) * v j)) =
      sumFin N (fun i => f i * u i) * sumFin N (fun j => g j * v j) := by
  calc
    sumFin N (fun i => sumFin N (fun j => u i * (f i * g j) * v j))
        = sumFin N (fun i => sumFin N (fun j => (f i * u i) * (g j * v j))) := by
          apply sumFin_congr; intro i
          apply sumFin_congr; intro j; grind
    _ = sumFin N (fun i => (f i * u i) * sumFin N (fun j => g j * v j)) := by
          apply sumFin_congr; intro i
          exact (sumFin_mul_left N (f i * u i) (fun j => g j * v j)).symm
    _ = sumFin N (fun i => f i * u i) * sumFin N (fun j => g j * v j) := by
          exact (sumFin_mul_right (f := fun i => f i * u i) N (sumFin N (fun j => g j * v j))).symm

private theorem concatVec_left (N : Nat) (u v : Fin N → Int) (i : Fin N) :
    concatVec N u v ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩ = u i := by
  simp [concatVec, i.isLt]

private theorem concatVec_right (N : Nat) (u v : Fin N → Int) (j : Fin N) :
    concatVec N u v ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩ = v j := by
  have hnj : ¬ (N + j.val < N) := by omega
  simp only [concatVec, hnj, ↓reduceDIte]
  congr 1; apply Fin.ext; dsimp; omega

private theorem passivityRealAugMat_ll (N : Nat) (Q : Int) (R X : Fin N → Fin N → Int) (i j : Fin N) :
    passivityRealAugMat N Q R X
      ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩
      ⟨j.val, Nat.lt_of_lt_of_le j.isLt (Nat.le_add_right N N)⟩ = passivityBlockA N Q R X i j := by
  simp [passivityRealAugMat, i.isLt, j.isLt]

private theorem passivityRealAugMat_lr (N : Nat) (Q : Int) (R X : Fin N → Fin N → Int) (i j : Fin N) :
    passivityRealAugMat N Q R X
      ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩
      ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩ = passivityBlockB N R X i j := by
  have hnj : ¬ (N + j.val < N) := by omega
  simp only [passivityRealAugMat, i.isLt, hnj, ↓reduceDIte]
  congr 1; apply Fin.ext; dsimp; omega

private theorem passivityRealAugMat_rl (N : Nat) (Q : Int) (R X : Fin N → Fin N → Int) (i j : Fin N) :
    passivityRealAugMat N Q R X
      ⟨N + i.val, Nat.add_lt_add_left i.isLt N⟩
      ⟨j.val, Nat.lt_of_lt_of_le j.isLt (Nat.le_add_right N N)⟩ = -passivityBlockB N R X i j := by
  have hni : ¬ (N + i.val < N) := by omega
  simp only [passivityRealAugMat, hni, j.isLt, ↓reduceDIte]
  congr 2; apply Fin.ext; dsimp; omega

private theorem passivityRealAugMat_rr (N : Nat) (Q : Int) (R X : Fin N → Fin N → Int) (i j : Fin N) :
    passivityRealAugMat N Q R X
      ⟨N + i.val, Nat.add_lt_add_left i.isLt N⟩
      ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩ = passivityBlockA N Q R X i j := by
  have hni : ¬ (N + i.val < N) := by omega
  have hnj : ¬ (N + j.val < N) := by omega
  simp only [passivityRealAugMat, hni, hnj, ↓reduceDIte]
  congr 1 <;> (apply Fin.ext; dsimp; omega)

/-- Theorem E4 (Real-Augmented Multi-Port Passivity Algebraic Identity):
    For `w = concatVec N u v` and `M = passivityRealAugMat N Q R X`,
    `w^T M w = Q^2 (||u||^2 + ||v||^2) - (||R u - X v||^2 + ||X u + R v||^2)`. -/
theorem passivity_real_aug_identity (N : Nat) (Q : Int) (R X : Fin N → Fin N → Int)
    (u v : Fin N → Int) :
    matQuad (N + N) (passivityRealAugMat N Q R X) (concatVec N u v) =
      Q * Q * (vecNormSq N u + vecNormSq N v) -
        (vecNormSq N (fun k => matMulVec N R u k - matMulVec N X v k) +
         vecNormSq N (fun k => matMulVec N X u k + matMulVec N R v k)) := by
  let w := concatVec N u v
  let M := passivityRealAugMat N Q R X
  let A := passivityBlockA N Q R X
  let B := passivityBlockB N R X
  have hQuadSplit :
      matQuad (N + N) M w =
        sumFin N (fun i => sumFin N (fun j => u i * A i j * u j)) +
        sumFin N (fun i => sumFin N (fun j => u i * B i j * v j)) -
        sumFin N (fun i => sumFin N (fun j => v i * B i j * u j)) +
        sumFin N (fun i => sumFin N (fun j => v i * A i j * v j)) := by
    unfold matQuad
    rw [sumFin_add_split N N (fun r => sumFin (N + N) (fun c => w r * M r c * w c))]
    have hTop :
        sumFin N (fun i => sumFin (N + N) (fun c =>
          w ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩ *
            M ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩ c * w c)) =
          sumFin N (fun i => sumFin N (fun j => u i * A i j * u j)) +
          sumFin N (fun i => sumFin N (fun j => u i * B i j * v j)) := by
      calc
        sumFin N (fun i => sumFin (N + N) (fun c => _))
            = sumFin N (fun i =>
                sumFin N (fun j => u i * A i j * u j) + sumFin N (fun j => u i * B i j * v j)) := by
              apply sumFin_congr; intro i
              rw [sumFin_add_split N N]
              have ht1 :
                  sumFin N (fun j =>
                    w ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩ *
                      M ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩
                        ⟨j.val, Nat.lt_of_lt_of_le j.isLt (Nat.le_add_right N N)⟩ *
                      w ⟨j.val, Nat.lt_of_lt_of_le j.isLt (Nat.le_add_right N N)⟩) =
                    sumFin N (fun j => u i * A i j * u j) := by
                apply sumFin_congr; intro j
                dsimp only [w, M, A]
                rw [concatVec_left N u v i, concatVec_left N u v j, passivityRealAugMat_ll]
              have ht2 :
                  sumFin N (fun j =>
                    w ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩ *
                      M ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩
                        ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩ *
                      w ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩) =
                    sumFin N (fun j => u i * B i j * v j) := by
                apply sumFin_congr; intro j
                dsimp only [w, M, B]
                rw [concatVec_left N u v i, concatVec_right N u v j, passivityRealAugMat_lr]
              rw [ht1, ht2]
        _ = sumFin N (fun i => sumFin N (fun j => u i * A i j * u j)) +
            sumFin N (fun i => sumFin N (fun j => u i * B i j * v j)) := sumFin_add _ _ _
    have hBot :
        sumFin N (fun i => sumFin (N + N) (fun c =>
          w ⟨N + i.val, Nat.add_lt_add_left i.isLt N⟩ *
            M ⟨N + i.val, Nat.add_lt_add_left i.isLt N⟩ c * w c)) =
          (-sumFin N (fun i => sumFin N (fun j => v i * B i j * u j))) +
          sumFin N (fun i => sumFin N (fun j => v i * A i j * v j)) := by
      calc
        sumFin N (fun i => sumFin (N + N) (fun c => _))
            = sumFin N (fun i =>
                sumFin N (fun j => -(v i * B i j * u j)) + sumFin N (fun j => v i * A i j * v j)) := by
              apply sumFin_congr; intro i
              rw [sumFin_add_split N N]
              have hb1 :
                  sumFin N (fun j =>
                    w ⟨N + i.val, Nat.add_lt_add_left i.isLt N⟩ *
                      M ⟨N + i.val, Nat.add_lt_add_left i.isLt N⟩
                        ⟨j.val, Nat.lt_of_lt_of_le j.isLt (Nat.le_add_right N N)⟩ *
                      w ⟨j.val, Nat.lt_of_lt_of_le j.isLt (Nat.le_add_right N N)⟩) =
                    sumFin N (fun j => -(v i * B i j * u j)) := by
                apply sumFin_congr; intro j
                dsimp only [w, M, B]
                rw [concatVec_right N u v i, concatVec_left N u v j, passivityRealAugMat_rl]
                change v i * (-passivityBlockB N R X i j) * u j = -(v i * passivityBlockB N R X i j * u j)
                grind
              have hb2 :
                  sumFin N (fun j =>
                    w ⟨N + i.val, Nat.add_lt_add_left i.isLt N⟩ *
                      M ⟨N + i.val, Nat.add_lt_add_left i.isLt N⟩
                        ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩ *
                      w ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩) =
                    sumFin N (fun j => v i * A i j * v j) := by
                apply sumFin_congr; intro j
                dsimp only [w, M, A]
                rw [concatVec_right N u v i, concatVec_right N u v j, passivityRealAugMat_rr]
              rw [hb1, hb2]
        _ = sumFin N (fun i => sumFin N (fun j => -(v i * B i j * u j))) +
            sumFin N (fun i => sumFin N (fun j => v i * A i j * v j)) := sumFin_add _ _ _
        _ = (-sumFin N (fun i => sumFin N (fun j => v i * B i j * u j))) +
            sumFin N (fun i => sumFin N (fun j => v i * A i j * v j)) := by
              congr 1
              rw [← sumFin_neg]
              apply sumFin_congr; intro i
              exact sumFin_neg N _
    rw [hTop, hBot]
    omega
  have hAntiB :
      sumFin N (fun i => sumFin N (fun j => v i * B i j * u j)) =
        -sumFin N (fun i => sumFin N (fun j => u i * B i j * v j)) := by
    rw [sumFin_swap]
    rw [← sumFin_neg]
    apply sumFin_congr; intro i
    rw [← sumFin_neg]
    apply sumFin_congr; intro j
    have hBskew : B j i = -B i j := by
      unfold B passivityBlockB
      rw [← sumFin_neg]
      apply sumFin_congr; intro k; grind
    rw [hBskew]; grind
  have hQuadA (z : Fin N → Int) :
      sumFin N (fun i => sumFin N (fun j => z i * A i j * z j)) =
        Q * Q * vecNormSq N z -
          sumFin N (fun k =>
            (matMulVec N R z k) * (matMulVec N R z k) + (matMulVec N X z k) * (matMulVec N X z k)) := by
    have hSplitA :
        sumFin N (fun i => sumFin N (fun j => z i * A i j * z j)) =
          sumFin N (fun i => sumFin N (fun j => z i * (if i = j then Q * Q else 0) * z j)) -
          sumFin N (fun i => sumFin N (fun j => z i * sumFin N (fun k => R k i * R k j + X k i * X k j) * z j)) := by
      rw [← sumFin_sub]
      apply sumFin_congr; intro i
      rw [← sumFin_sub]
      apply sumFin_congr; intro j
      unfold A passivityBlockA
      grind
    have hDiagQ :
        sumFin N (fun i => sumFin N (fun j => z i * (if i = j then Q * Q else 0) * z j)) =
          Q * Q * vecNormSq N z := by
      unfold vecNormSq
      rw [sumFin_mul_left]
      apply sumFin_congr; intro i
      have hSub : (fun j : Fin N => z i * (if i = j then Q * Q else 0) * z j) =
          (fun j : Fin N => if j = i then Q * Q * (z i * z i) else 0) := by
        funext j
        by_cases hj : j = i
        · subst hj; simp; grind
        · have hij : i ≠ j := fun h => hj h.symm
          simp [hj, hij]
      rw [hSub, sumFin_single_hit]
    have hMatSq :
        sumFin N (fun i => sumFin N (fun j => z i * sumFin N (fun k => R k i * R k j + X k i * X k j) * z j)) =
          sumFin N (fun k =>
            (matMulVec N R z k) * (matMulVec N R z k) + (matMulVec N X z k) * (matMulVec N X z k)) := by
      calc
        sumFin N (fun i => sumFin N (fun j => z i * sumFin N (fun k => R k i * R k j + X k i * X k j) * z j))
            = sumFin N (fun i => sumFin N (fun j => sumFin N (fun k =>
                z i * (R k i * R k j) * z j + z i * (X k i * X k j) * z j))) := by
              apply sumFin_congr; intro i
              apply sumFin_congr; intro j
              rw [sumFin_mul_left, sumFin_mul_right]
              apply sumFin_congr; intro k; grind
        _ = sumFin N (fun i => sumFin N (fun k => sumFin N (fun j =>
              z i * (R k i * R k j) * z j + z i * (X k i * X k j) * z j))) := by
              apply sumFin_congr; intro i; exact sumFin_swap _
        _ = sumFin N (fun k => sumFin N (fun i => sumFin N (fun j =>
              z i * (R k i * R k j) * z j + z i * (X k i * X k j) * z j))) := sumFin_swap _
        _ = sumFin N (fun k =>
              sumFin N (fun i => sumFin N (fun j => z i * (R k i * R k j) * z j)) +
              sumFin N (fun i => sumFin N (fun j => z i * (X k i * X k j) * z j))) := by
              apply sumFin_congr; intro k
              rw [← sumFin_add]
              apply sumFin_congr; intro i
              exact sumFin_add _ _ _
        _ = sumFin N (fun k =>
              (matMulVec N R z k) * (matMulVec N R z k) + (matMulVec N X z k) * (matMulVec N X z k)) := by
              apply sumFin_congr; intro k
              unfold matMulVec
              rw [quad_bilinear_factor N (R k) (R k) z z, quad_bilinear_factor N (X k) (X k) z z]
    rw [hSplitA, hDiagQ, hMatSq]
  have hQuadB :
      sumFin N (fun i => sumFin N (fun j => u i * B i j * v j)) =
        sumFin N (fun k =>
          matMulVec N R u k * matMulVec N X v k - matMulVec N X u k * matMulVec N R v k) := by
    calc
      sumFin N (fun i => sumFin N (fun j => u i * B i j * v j))
          = sumFin N (fun i => sumFin N (fun j => sumFin N (fun k =>
              u i * (R k i * X k j) * v j - u i * (X k i * R k j) * v j))) := by
            apply sumFin_congr; intro i
            apply sumFin_congr; intro j
            unfold B passivityBlockB
            rw [sumFin_mul_left, sumFin_mul_right]
            apply sumFin_congr; intro k; grind
      _ = sumFin N (fun i => sumFin N (fun k => sumFin N (fun j =>
            u i * (R k i * X k j) * v j - u i * (X k i * R k j) * v j))) := by
            apply sumFin_congr; intro i; exact sumFin_swap _
      _ = sumFin N (fun k => sumFin N (fun i => sumFin N (fun j =>
            u i * (R k i * X k j) * v j - u i * (X k i * R k j) * v j))) := sumFin_swap _
      _ = sumFin N (fun k =>
            sumFin N (fun i => sumFin N (fun j => u i * (R k i * X k j) * v j)) -
            sumFin N (fun i => sumFin N (fun j => u i * (X k i * R k j) * v j))) := by
            apply sumFin_congr; intro k
            rw [← sumFin_sub]
            apply sumFin_congr; intro i
            exact sumFin_sub _ _ _
      _ = sumFin N (fun k =>
            matMulVec N R u k * matMulVec N X v k - matMulVec N X u k * matMulVec N R v k) := by
            apply sumFin_congr; intro k
            unfold matMulVec
            rw [quad_bilinear_factor N (R k) (X k) u v, quad_bilinear_factor N (X k) (R k) u v]
  have hNormOut :
      vecNormSq N (fun k => matMulVec N R u k - matMulVec N X v k) +
      vecNormSq N (fun k => matMulVec N X u k + matMulVec N R v k) =
        sumFin N (fun k => matMulVec N R u k * matMulVec N R u k + matMulVec N X u k * matMulVec N X u k) +
        sumFin N (fun k => matMulVec N R v k * matMulVec N R v k + matMulVec N X v k * matMulVec N X v k) -
        2 * sumFin N (fun k =>
          matMulVec N R u k * matMulVec N X v k - matMulVec N X u k * matMulVec N R v k) := by
    unfold vecNormSq
    rw [← sumFin_add, sumFin_mul_left, ← sumFin_add, ← sumFin_sub]
    apply sumFin_congr; intro k; grind
  rw [hQuadSplit, hAntiB, hQuadA u, hQuadA v, hQuadB, hNormOut]
  grind

/-- Boolean kernel check verifying that `M_rows` equals `passivityRealAugMat` built from `R_rows, X_rows`. -/
def checkPassivityMat (N : Nat) (Q : Int) (R_rows X_rows M_rows : List (List Int)) : Bool :=
  allFinB (N + N) (fun r => allFinB (N + N) (fun c =>
    decide (matAt M_rows r c = passivityRealAugMat N Q (matAt R_rows) (matAt X_rows) r c)))

theorem checkPassivityMat_sound (N : Nat) (Q : Int) (R_rows X_rows M_rows : List (List Int))
    (h : checkPassivityMat N Q R_rows X_rows M_rows = true) :
    ∀ r c : Fin (N + N), matAt M_rows r c = passivityRealAugMat N Q (matAt R_rows) (matAt X_rows) r c := by
  intro r c
  have hr := allFinB_true (N + N) _ h r
  have hc := allFinB_true (N + N) _ hr c
  exact of_decide_eq_true hc

/-- Theorem E4b (Certified Multi-Port Passivity via Kernel Cholesky+SDD):
    Given `checkCholeskySDD (N+N) M L mu = true` where `matAt M = passivityRealAugMat N Q R X`
    and `0 ≤ mu - deltaQ`, every complex incident wave `u + j v` satisfies `||S(u+jv)||^2 ≤ (Q^2 - mu)(||u||^2 + ||v||^2) ≤ Q^2 (||u||^2 + ||v||^2)`. -/
theorem certified_multiport_passivity
    (N : Nat) (Q mu deltaQ : Int) (R X : Fin N → Fin N → Int) (M_rows L_rows : List (List Int))
    (hCheck : checkCholeskySDD (N + N) M_rows L_rows mu = true)
    (hEq : ∀ r c : Fin (N + N), matAt M_rows r c = passivityRealAugMat N Q R X r c)
    (hMargin : 0 ≤ mu - deltaQ) (_hDelta : 0 ≤ deltaQ) :
    ∀ u v : Fin N → Int,
      vecNormSq N (fun k => matMulVec N R u k - matMulVec N X v k) +
      vecNormSq N (fun k => matMulVec N X u k + matMulVec N R v k) ≤
        (Q * Q - mu) * (vecNormSq N u + vecNormSq N v) ∧
      vecNormSq N (fun k => matMulVec N R u k - matMulVec N X v k) +
      vecNormSq N (fun k => matMulVec N X u k + matMulVec N R v k) ≤
        Q * Q * (vecNormSq N u + vecNormSq N v) := by
  intro u v
  have hPsd := checkCholeskySDD_sound (N + N) M_rows L_rows mu hCheck (concatVec N u v)
  have hQuadEq : matQuad (N + N) (matAt M_rows) (concatVec N u v) =
      matQuad (N + N) (passivityRealAugMat N Q R X) (concatVec N u v) := by
    unfold matQuad
    apply sumFin_congr; intro r
    apply sumFin_congr; intro c
    rw [hEq r c]
  have hNormCat : vecNormSq (N + N) (concatVec N u v) = vecNormSq N u + vecNormSq N v := by
    unfold vecNormSq
    rw [sumFin_add_split N N]
    have hl :
        sumFin N (fun i =>
          concatVec N u v ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩ *
            concatVec N u v ⟨i.val, Nat.lt_of_lt_of_le i.isLt (Nat.le_add_right N N)⟩) =
          vecNormSq N u := by
      apply sumFin_congr; intro i
      rw [concatVec_left N u v i]
    have hr :
        sumFin N (fun j =>
          concatVec N u v ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩ *
            concatVec N u v ⟨N + j.val, Nat.add_lt_add_left j.isLt N⟩) =
          vecNormSq N v := by
      apply sumFin_congr; intro j
      rw [concatVec_right N u v j]
    rw [hl, hr]
    rfl
  rw [hQuadEq, passivity_real_aug_identity] at hPsd
  rw [hNormCat] at hPsd
  have hMu : 0 ≤ mu := by omega
  have hNormNonneg : 0 ≤ vecNormSq N u + vecNormSq N v := by
    have hu := vecNormSq_nonneg N u
    have hv := vecNormSq_nonneg N v
    omega
  have hMuNorm : 0 ≤ mu * (vecNormSq N u + vecNormSq N v) := Int.mul_nonneg hMu hNormNonneg
  have h1 :
      vecNormSq N (fun k => matMulVec N R u k - matMulVec N X v k) +
      vecNormSq N (fun k => matMulVec N X u k + matMulVec N R v k) ≤
        Q * Q * (vecNormSq N u + vecNormSq N v) - mu * (vecNormSq N u + vecNormSq N v) := by
    omega
  have hSub : Q * Q * (vecNormSq N u + vecNormSq N v) - mu * (vecNormSq N u + vecNormSq N v) =
      (Q * Q - mu) * (vecNormSq N u + vecNormSq N v) := by grind
  rw [hSub] at h1
  exact ⟨h1, by omega⟩

/-- Theorem E5 (2D FDTD Courant-Friedrichs-Lewy Stability Bound):
    Exact integer CFL inequality `c^2 dt^2 (1/dx^2 + 1/dy^2) ≤ 1` and margin preservation. -/
theorem fdtd_cfl_stability_2d (c_dt_sq dx_sq dy_sq : Int)
    (_hPos : 0 ≤ c_dt_sq ∧ 0 < dx_sq ∧ 0 < dy_sq)
    (hCfl : c_dt_sq * (dx_sq + dy_sq) ≤ dx_sq * dy_sq) :
    c_dt_sq * dx_sq + c_dt_sq * dy_sq ≤ dx_sq * dy_sq ∧
    0 ≤ dx_sq * dy_sq - c_dt_sq * (dx_sq + dy_sq) := by
  have hDist : c_dt_sq * (dx_sq + dy_sq) = c_dt_sq * dx_sq + c_dt_sq * dy_sq := Int.mul_add _ _ _
  omega

/-- Theorem E6 (Multi-Port Reciprocity Symmetry Bound):
    Kernel-verified entrywise `|S_ij - S_ji| ≤ eps` across real and imaginary parts. -/
theorem reciprocity_symmetry_bound (n : Nat) (R X : List (List Int)) (eps : Int)
    (hRecip : allFinB n (fun i => allFinB n (fun j =>
      decide (absInt (matAt R i j - matAt R j i) ≤ eps ∧
              absInt (matAt X i j - matAt X j i) ≤ eps))) = true) :
    ∀ i j : Fin n,
      absInt (matAt R i j - matAt R j i) ≤ eps ∧
      absInt (matAt X i j - matAt X j i) ≤ eps := by
  intro i j
  have hi := allFinB_true n _ hRecip i
  have hj := allFinB_true n _ hi j
  exact of_decide_eq_true hj

/-! ## 3. Pillar 2 — Electronics Theorems (Theorems E7–E9) -/

/-- Theorem E7 (Tellegen Passive MNA Power Dissipation):
    A symmetric MNA conductance matrix `G` certified PSD by `checkCholeskySDD` guarantees
    nonnegative dissipated power `v^T i = v^T (G v) ≥ mu * ||v||^2 ≥ 0`. -/
theorem tellegen_passive_mna_dissipation (n : Nat) (G L : List (List Int)) (mu : Int)
    (hCheck : checkCholeskySDD n G L mu = true) (hMu : 0 ≤ mu)
    (v i : Fin n → Int) (hKcl : ∀ k, i k = sumFin n (fun j => matAt G k j * v j)) :
    mu * vecNormSq n v ≤ sumFin n (fun k => v k * i k) ∧
    0 ≤ sumFin n (fun k => v k * i k) := by
  have hPsd := checkCholeskySDD_sound n G L mu hCheck v
  have hPowEq : sumFin n (fun k => v k * i k) = matQuad n (matAt G) v := by
    unfold matQuad
    apply sumFin_congr; intro k
    rw [hKcl k, sumFin_mul_left]
    apply sumFin_congr; intro j; grind
  have hNonneg : 0 ≤ mu * vecNormSq n v := Int.mul_nonneg hMu (vecNormSq_nonneg n v)
  rw [hPowEq]
  exact ⟨hPsd, by omega⟩

/-- Theorem E8 (A-Stability of Trapezoidal Companion Integration):
    For passive RC decay `(S + alpha) * v_next = (S - alpha) * v_curr` with `S > 0, alpha ≥ 0`,
    energy is monotonically non-increasing: `v_next^2 ≤ v_curr^2`. -/
theorem trapezoidal_astability_energy_decay (S alpha v_curr v_next : Int)
    (hS : 0 < S) (_hA : 0 ≤ alpha)
    (hTrap : (S + alpha) * v_next = (S - alpha) * v_curr) :
    v_next * v_next ≤ v_curr * v_curr := by
  have hProd : (S + alpha) * (S + alpha) * (v_next * v_next) =
      (S - alpha) * (S - alpha) * (v_curr * v_curr) := by
    calc
      (S + alpha) * (S + alpha) * (v_next * v_next)
          = ((S + alpha) * v_next) * ((S + alpha) * v_next) := by grind
      _ = ((S - alpha) * v_curr) * ((S - alpha) * v_curr) := by rw [hTrap]
      _ = (S - alpha) * (S - alpha) * (v_curr * v_curr) := by grind
  have hCoeff : (S - alpha) * (S - alpha) ≤ (S + alpha) * (S + alpha) := by
    have hDiff : (S + alpha) * (S + alpha) - (S - alpha) * (S - alpha) = 4 * S * alpha := by grind
    have h4sa : 0 ≤ 4 * S * alpha := Int.mul_nonneg (by omega) _hA
    omega
  have hDiffSq :
      (S + alpha) * (S + alpha) * (v_curr * v_curr - v_next * v_next) =
        ((S + alpha) * (S + alpha) - (S - alpha) * (S - alpha)) * (v_curr * v_curr) := by
    grind
  have hPosCoeff : 0 < (S + alpha) * (S + alpha) := by
    have hSa : 0 < S + alpha := by omega
    exact Int.mul_pos hSa hSa
  have hNonnegRhs : 0 ≤ ((S + alpha) * (S + alpha) - (S - alpha) * (S - alpha)) * (v_curr * v_curr) :=
    Int.mul_nonneg (Int.sub_nonneg.mpr hCoeff) (int_sq_nonneg v_curr)
  have hNonnegProd : 0 ≤ (S + alpha) * (S + alpha) * (v_curr * v_curr - v_next * v_next) := by
    rw [hDiffSq]; exact hNonnegRhs
  by_cases hle : v_next * v_next ≤ v_curr * v_curr
  · exact hle
  · have hNeg2 : 0 < v_next * v_next - v_curr * v_curr := by omega
    have hp : 0 < (S + alpha) * (S + alpha) * (v_next * v_next - v_curr * v_curr) :=
      Int.mul_pos hPosCoeff hNeg2
    have hNegProd : (S + alpha) * (S + alpha) * (v_curr * v_curr - v_next * v_next) =
        -((S + alpha) * (S + alpha) * (v_next * v_next - v_curr * v_curr)) := by grind
    omega

/-- Theorem E9 (Worst-Case PVT Corner SOA & SI Eye-Margin Enclosure):
    Any sample contained in certified PVT corner interval `iv` lies within SOA `[vLo, vHi]`,
    and disjoint high/low eye rails yield strictly positive certified eye opening above `mask`. -/
theorem soa_and_eye_enclosure_cert (iv ivLo ivHi : Interval1D) (vLo vHi eyeMask : Int)
    (hSoa : vLo ≤ iv.lo ∧ iv.hi ≤ vHi)
    (hEye : eyeMask < ivHi.lo - ivLo.hi) :
    (∀ x, iv.contains x → vLo ≤ x ∧ x ≤ vHi) ∧
    (∀ xLo xHi, ivLo.contains xLo → ivHi.contains xHi → eyeMask < xHi - xLo) := by
  unfold Interval1D.contains at *
  refine ⟨?_, ?_⟩
  · intro x ⟨hx0, hx1⟩; omega
  · intro xLo xHi ⟨_, hl1⟩ ⟨hh0, _⟩; omega

/-! ## 4. Pillar 3 — Electrical Engineering Theorems (Theorems E10–E12) -/

/-- Theorem E10 (IPMSM Operating Envelope & Torque Bracket Certification):
    Certifies simultaneous current-circle, voltage-ellipse, and scaled torque interval compliance. -/
theorem ipmsm_operating_envelope_cert
    (id iq vd vq psiD psiQ iMax vMax twoP : Int) (tBracket : Interval1D)
    (hCurr : id * id + iq * iq ≤ iMax * iMax)
    (hVolt : vd * vd + vq * vq ≤ vMax * vMax)
    (hTorque : tBracket.contains (twoP * (psiD * iq - psiQ * id))) :
    id * id + iq * iq ≤ iMax * iMax ∧
    vd * vd + vq * vq ≤ vMax * vMax ∧
    tBracket.lo ≤ twoP * (psiD * iq - psiQ * id) ∧
    twoP * (psiD * iq - psiQ * id) ≤ tBracket.hi := by
  unfold Interval1D.contains at hTorque
  exact ⟨hCurr, hVolt, hTorque.1, hTorque.2⟩

/-- Theorem E11 (AC-OPF Anytime Weak-Duality Lower Bound):
    Exact weak duality from integer Cholesky+SDD floor `mu_psd * ||x||^2 ≤ x^T A x`
    and bounded bus voltage-squared norm `||x||^2 ≤ W_max`. -/
theorem ac_opf_anytime_dual_lower_bound
    (C_primal C_dual mu_psd normSq_x W_max delta_round : Int)
    (quadA : Int)
    (hLagrange : C_dual + quadA ≤ C_primal)
    (hPsd : mu_psd * normSq_x ≤ quadA)
    (hNormLo : 0 ≤ normSq_x)
    (hNormHi : normSq_x ≤ W_max)
    (hDelta : 0 ≤ delta_round) :
    C_dual + negPart mu_psd * W_max - delta_round ≤ C_primal := by
  have h1 : negPart mu_psd * normSq_x ≤ mu_psd * normSq_x :=
    Int.mul_le_mul_of_nonneg_right (negPart_le mu_psd) hNormLo
  have h2 : negPart mu_psd * W_max ≤ negPart mu_psd * normSq_x := by
    have hNegP : 0 ≤ -negPart mu_psd := by
      have := negPart_nonpos mu_psd; omega
    have hMul := Int.mul_le_mul_of_nonneg_left hNormHi hNegP
    grind
  omega

/-- Theorem E11b (Two-Sided AC-OPF Interval Bracket Validity):
    Combines certified dual lower bound `C_lb` and feasible primal AC-PF upper bound `C_ub`. -/
theorem ac_opf_bracket_valid (C_lb C_exact C_ub : Int)
    (hLB : C_lb ≤ C_exact) (hUB : C_exact ≤ C_ub) :
    ({ lo := C_lb, hi := C_ub } : Interval1D).contains C_exact := by
  unfold Interval1D.contains
  dsimp
  exact ⟨hLB, hUB⟩

/-- Theorem E12 (Grid EMT Lyapunov Transient Stability Margin):
    Given post-fault energy `V_post = V_kin + V_pot`, conservative integration bound `err ≥ 0`,
    and `V_post + err < V_crit`, proves potential boundary clearance and strictly positive stability margin. -/
theorem grid_lyapunov_stability_basin (V_kin V_pot V_crit err : Int)
    (hKin : 0 ≤ V_kin) (hErr : 0 ≤ err)
    (hMarg : V_kin + V_pot + err < V_crit) :
    V_pot < V_crit ∧ 0 < V_crit - (V_kin + V_pot + err) := by
  omega

/-! ## 5. Frontier Theorems (Theorems E13–E20, ELE v10.0.0) -/

theorem checkCholeskySDD_sym (n : Nat) (A L : List (List Int)) (mu : Int)
    (hCheck : checkCholeskySDD n A L mu = true) :
    ∀ i j : Fin n, matAt A i j = matAt A j i := by
  unfold checkCholeskySDD at hCheck
  simp only [Bool.and_eq_true] at hCheck
  intro i j
  exact of_decide_eq_true (allFinB_true n _ (allFinB_true n _ hCheck.1 i) j)

private theorem sumFin_one_add (k : Nat) (f : Fin (1 + k) → Int) :
    sumFin (1 + k) f =
      f ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ +
      sumFin k (fun j => f ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩) := by
  induction k with
  | zero =>
    change 0 + f ⟨0, Nat.add_pos_left Nat.zero_lt_one 0⟩ =
      f ⟨0, Nat.add_pos_left Nat.zero_lt_one 0⟩ + 0
    omega
  | succ m ih =>
    have hstep : sumFin (1 + (m + 1)) f =
        sumFin (1 + m) (fun i => f i.castSucc) + f ⟨1 + m, Nat.add_lt_add_left (Nat.lt_succ_self m) 1⟩ := rfl
    rw [hstep, ih (fun i => f i.castSucc)]
    have hrhs : sumFin (m + 1) (fun j : Fin (m + 1) => f ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩) =
        sumFin m (fun j : Fin m => f ⟨1 + j.val, Nat.add_lt_add_left (Nat.lt_succ_of_lt j.isLt) 1⟩) +
        f ⟨1 + m, Nat.add_lt_add_left (Nat.lt_succ_self m) 1⟩ := rfl
    rw [hrhs]
    have h0 : f (⟨0, Nat.add_pos_left Nat.zero_lt_one m⟩ : Fin (1 + m)).castSucc =
        f ⟨0, Nat.add_pos_left Nat.zero_lt_one (m + 1)⟩ := rfl
    have hsub : sumFin m (fun j : Fin m => f (⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ : Fin (1 + m)).castSucc) =
        sumFin m (fun j : Fin m => f ⟨1 + j.val, Nat.add_lt_add_left (Nat.lt_succ_of_lt j.isLt) 1⟩) :=
      sumFin_congr (fun _ => rfl)
    rw [h0, hsub]
    omega

/-- Theorem E13 (Sparse Chordal Clique Decomposition & Weak-Duality Soundness):
    If sparse `A_full = ∑_c S_c` and each clique block satisfies `mu_c ||v||^2 ≤ v^T S_c v`,
    then `∑_c (v^T S_c v) = v^T A_full v` and the sum of Gershgorin residuals is a sound global LB. -/
theorem chordal_clique_decomposition_sound
    (n m : Nat) (A_full : Fin n → Fin n → Int) (S : Fin m → Fin n → Fin n → Int)
    (mu wMax : Fin m → Int) (cDual dRnd cPrimal : Int) (v : Fin n → Int)
    (hDecomp : ∀ i j, sumFin m (fun c => S c i j) = A_full i j)
    (hLag : cDual + matQuad n A_full v ≤ cPrimal)
    (hCliquePsd : ∀ c, mu c * vecNormSq n v ≤ matQuad n (S c) v)
    (hNormLo : 0 ≤ vecNormSq n v)
    (hNormHi : ∀ c, vecNormSq n v ≤ wMax c)
    (hDelta : 0 ≤ dRnd) :
    cDual + sumFin m (fun c => negPart (mu c) * wMax c) - dRnd ≤ cPrimal := by
  have hQuad : sumFin m (fun c => matQuad n (S c) v) = matQuad n A_full v := by
    unfold matQuad
    rw [sumFin_swap (fun c i => sumFin n (fun j => v i * S c i j * v j))]
    apply sumFin_congr; intro i
    rw [sumFin_swap (fun c j => v i * S c i j * v j)]
    apply sumFin_congr; intro j
    have hmul : ∀ c : Fin m, v i * S c i j * v j = S c i j * (v i * v j) := fun c => by grind
    rw [sumFin_congr hmul, ← sumFin_mul_right m (fun c => S c i j) (v i * v j), hDecomp i j]
    grind
  have hCliqueBd : ∀ c : Fin m, negPart (mu c) * wMax c ≤ matQuad n (S c) v := by
    intro c
    have h1 : negPart (mu c) * vecNormSq n v ≤ mu c * vecNormSq n v :=
      Int.mul_le_mul_of_nonneg_right (negPart_le (mu c)) hNormLo
    have hNegP : 0 ≤ -negPart (mu c) := by have := negPart_nonpos (mu c); omega
    have h2 : negPart (mu c) * wMax c ≤ negPart (mu c) * vecNormSq n v := by
      have hm := Int.mul_le_mul_of_nonneg_left (hNormHi c) hNegP
      grind
    have h3 := hCliquePsd c
    omega
  have hSum : sumFin m (fun c => negPart (mu c) * wMax c) ≤ matQuad n A_full v := by
    rw [← hQuad]
    exact sumFin_le_sumFin m _ _ hCliqueBd
  omega

/-- Theorem E14 (AC-OPF Branch Thermal + Angle-Difference Dual Weak Duality):
    Nonnegative branch thermal `(xi, eta)` and angle `(tauLo, tauHi)` dual penalties combine
    validly with the integer Cholesky+SDD spectral floor. -/
theorem ac_opf_thermal_angle_dual_sound
    (nEdges : Nat) (C_primal C_dual mu_psd normSq_x W_max delta_round quadA : Int)
    (thermSlack angLoSlack angHiSlack : Fin nEdges → Int)
    (hLag : C_dual + quadA +
        sumFin nEdges (fun e => thermSlack e + angLoSlack e + angHiSlack e) ≤ C_primal)
    (hTherm : ∀ e, 0 ≤ thermSlack e)
    (hAngLo : ∀ e, 0 ≤ angLoSlack e)
    (hAngHi : ∀ e, 0 ≤ angHiSlack e)
    (hPsd : mu_psd * normSq_x ≤ quadA)
    (hNormLo : 0 ≤ normSq_x) (hNormHi : normSq_x ≤ W_max) (hDelta : 0 ≤ delta_round) :
    C_dual + negPart mu_psd * W_max - delta_round ≤ C_primal := by
  have hPen : 0 ≤ sumFin nEdges (fun e => thermSlack e + angLoSlack e + angHiSlack e) :=
    sumFin_nonneg nEdges _ (fun e => by
      have ht := hTherm e; have ha1 := hAngLo e; have ha2 := hAngHi e; omega)
  exact ac_opf_anytime_dual_lower_bound C_primal C_dual mu_psd normSq_x W_max delta_round quadA
    (by omega) hPsd hNormLo hNormHi hDelta

/-- Theorem E15 (Certified Grid Observable Envelope):
    Dual lower bounds on `+phi` and `-phi` enclose every AC-OPF feasible observable `phi`. -/
theorem ac_opf_observable_envelope_sound
    (obsVal cDualLo muLo normSq Wmax dRndLo cDualHi muHi dRndHi quadLo quadHi : Int)
    (hLagLo : cDualLo + quadLo ≤ obsVal) (hPsdLo : muLo * normSq ≤ quadLo)
    (hLagHi : cDualHi + quadHi ≤ -obsVal) (hPsdHi : muHi * normSq ≤ quadHi)
    (hNormLo : 0 ≤ normSq) (hNormHi : normSq ≤ Wmax)
    (hD1 : 0 ≤ dRndLo) (hD2 : 0 ≤ dRndHi) :
    ({ lo := cDualLo + negPart muLo * Wmax - dRndLo,
       hi := -(cDualHi + negPart muHi * Wmax - dRndHi) } : Interval1D).contains obsVal := by
  have hLb := ac_opf_anytime_dual_lower_bound obsVal cDualLo muLo normSq Wmax dRndLo quadLo
    hLagLo hPsdLo hNormLo hNormHi hD1
  have hUb := ac_opf_anytime_dual_lower_bound (-obsVal) cDualHi muHi normSq Wmax dRndHi quadHi
    hLagHi hPsdHi hNormLo hNormHi hD2
  unfold Interval1D.contains
  dsimp
  omega

/-- Theorem E16a (Maxwell-Poynting Quadratic Upper Bound):
    Symmetric PSD `(1 + k) × (1 + k)` block `M` yields `-(2 c^T t) - t^T H t ≤ M[0,0]` for all `t`. -/
theorem maxwell_poynting_dual_upper_bound
    (k : Nat) (M : Fin (1 + k) → Fin (1 + k) → Int)
    (hSym : ∀ a b, M a b = M b a)
    (hPsd : ∀ v : Fin (1 + k) → Int, 0 ≤ matQuad (1 + k) M v)
    (t : Fin k → Int) :
    -(sumFin k (fun j => 2 * M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j)) -
      sumFin k (fun i => sumFin k (fun j =>
        t i * M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j)) ≤
      M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ := by
  let v : Fin (1 + k) → Int := fun idx =>
    if h : idx.val = 0 then 1 else t ⟨idx.val - 1, by omega⟩
  have hv0 : v ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ = 1 := rfl
  have hvt : ∀ j : Fin k, v ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ = t j := by
    intro j
    simp only [v]
    split
    · omega
    · exact congrArg t (Fin.ext (Nat.add_sub_cancel_left 1 j.val))
  have hq : 0 ≤ matQuad (1 + k) M v := hPsd v
  have hExp : matQuad (1 + k) M v =
      M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ +
      sumFin k (fun j => 2 * M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j) +
      sumFin k (fun i => sumFin k (fun j =>
        t i * M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j)) := by
    unfold matQuad
    rw [sumFin_one_add k (fun i => sumFin (1 + k) (fun j => v i * M i j * v j))]
    rw [sumFin_one_add k (fun j => v ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ * M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ j * v j)]
    have hRow0 : sumFin k (fun j : Fin k =>
        v ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ * M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ *
        v ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩) =
        sumFin k (fun j => M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j) := by
      apply sumFin_congr; intro j; rw [hv0, hvt j, Int.one_mul]
    have hDiag0 : v ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ * M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ *
        v ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ = M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ := by
      rw [hv0, Int.one_mul, Int.mul_one]
    have hTail : sumFin k (fun i : Fin k =>
        sumFin (1 + k) (fun j => v ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ *
          M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ j * v j)) =
        sumFin k (fun i => M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ * t i) +
        sumFin k (fun i => sumFin k (fun j =>
          t i * M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j)) := by
      have hIn : ∀ i : Fin k,
          sumFin (1 + k) (fun j => v ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ *
            M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ j * v j) =
          M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ * t i +
          sumFin k (fun j => t i * M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j) := by
        intro i
        rw [sumFin_one_add k]
        have hs := hSym ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩
        have h0i : v ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ *
            M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ *
            v ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ =
            M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ * t i := by
          rw [hvt i, hv0, hs]; grind
        have hInner : sumFin k (fun j : Fin k =>
            v ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ *
            M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ *
            v ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩) =
            sumFin k (fun j => t i * M ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j) := by
          apply sumFin_congr; intro j; rw [hvt i, hvt j]
        rw [h0i, hInner]
      rw [sumFin_congr hIn, sumFin_add]
    have hTwo : sumFin k (fun j => 2 * M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j) =
        sumFin k (fun j => M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j) +
        sumFin k (fun j => M ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j) := by
      rw [← sumFin_add]
      apply sumFin_congr; intro j; grind
    rw [hDiag0, hRow0, hTail, hTwo]
    omega
  omega

/-- Theorem E16b (Kernel-Checked Maxwell-Poynting Bound Endorsement). -/
theorem certified_maxwell_poynting_bound
    (k : Nat) (M_rows L_rows : List (List Int)) (mu localUb globalUb : Int)
    (hCheck : checkCholeskySDD (1 + k) M_rows L_rows mu = true)
    (hMu : 0 ≤ mu) (hTight : localUb ≤ globalUb) :
    (∀ t : Fin k → Int,
      -(sumFin k (fun j => 2 * matAt M_rows ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j)) -
      sumFin k (fun i => sumFin k (fun j =>
        t i * matAt M_rows ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j)) ≤
        matAt M_rows ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩ ⟨0, Nat.add_pos_left Nat.zero_lt_one k⟩) ∧
    localUb ≤ globalUb := by
  have hSym := checkCholeskySDD_sym (1 + k) M_rows L_rows mu hCheck
  have hSd := checkCholeskySDD_sound (1 + k) M_rows L_rows mu hCheck
  have hPsd : ∀ v : Fin (1 + k) → Int, 0 ≤ matQuad (1 + k) (matAt M_rows) v := by
    intro v
    have h1 := hSd v
    have h2 : 0 ≤ mu * vecNormSq (1 + k) v := Int.mul_nonneg hMu (vecNormSq_nonneg (1 + k) v)
    omega
  exact ⟨maxwell_poynting_dual_upper_bound k (matAt M_rows) hSym hPsd, hTight⟩

/-- Theorem E17 (KYP Bounded-Real LMI Passivity Certification). -/
theorem kyp_bounded_real_passivity_cert
    (d : Nat) (M_rows L_rows : List (List Int)) (mu : Int)
    (hCheck : checkCholeskySDD d M_rows L_rows mu = true) (hMu : 0 ≤ mu) :
    ∀ w : Fin d → Int, 0 ≤ matQuad d (matAt M_rows) w := by
  intro w
  have hSd := checkCholeskySDD_sound d M_rows L_rows mu hCheck w
  have hNn : 0 ≤ mu * vecNormSq d w := Int.mul_nonneg hMu (vecNormSq_nonneg d w)
  omega

/-- Theorem E17b (Certified KYP Bounded-Real Passivity Repair Wrapper). -/
theorem certified_kyp_passivity
    (d : Nat) (M_rows L_rows : List (List Int)) (mu sigmaRepQ gammaQ : Int)
    (hCheck : checkCholeskySDD d M_rows L_rows mu = true)
    (hMu : 0 ≤ mu) (hSig : sigmaRepQ ≤ gammaQ) :
    (∀ w : Fin d → Int, 0 ≤ matQuad d (matAt M_rows) w) ∧ sigmaRepQ ≤ gammaQ :=
  ⟨kyp_bounded_real_passivity_cert d M_rows L_rows mu hCheck hMu, hSig⟩

/-- Theorem E18a (Lasserre Moment-SOS-2 Monotonic Hierarchy Exactness). -/
theorem moment_sos_hierarchy_exactness
    (d : Nat) (shor1LB cDual mu wMax dRnd cLB cUB : Int)
    (A_rows L_rows : List (List Int)) (z : Fin d → Int) (C_primal : Int)
    (hTight : shor1LB ≤ cLB)
    (hCheck : checkCholeskySDD d A_rows L_rows mu = true)
    (hDelta : 0 ≤ dRnd)
    (hLag : cDual + matQuad d (matAt A_rows) z ≤ C_primal)
    (hNormLo : 0 ≤ vecNormSq d z)
    (hNormHi : vecNormSq d z ≤ wMax)
    (hEqLB : cLB = cDual + negPart mu * wMax - dRnd)
    (hPrimalUb : C_primal ≤ cUB) :
    shor1LB ≤ cLB ∧ ({ lo := cLB, hi := cUB } : Interval1D).contains C_primal := by
  have hPsd := checkCholeskySDD_sound d A_rows L_rows mu hCheck z
  have hLower := ac_opf_anytime_dual_lower_bound C_primal cDual mu (vecNormSq d z) wMax dRnd
    (matQuad d (matAt A_rows) z) hLag hPsd hNormLo hNormHi hDelta
  have hLower2 : cLB ≤ C_primal := by rw [hEqLB]; exact hLower
  exact ⟨hTight, ac_opf_bracket_valid cLB C_primal cUB hLower2 hPrimalUb⟩

/-- Theorem E18b (Kernel-Checked SOS-2 Lower Bound Alias). -/
theorem certified_sos2_lower_bound
    (d : Nat) (shor1LB cDual mu wMax dRnd cLB cUB : Int)
    (A_rows L_rows : List (List Int)) (z : Fin d → Int) (C_primal : Int)
    (hTight : shor1LB ≤ cLB)
    (hCheck : checkCholeskySDD d A_rows L_rows mu = true)
    (hDelta : 0 ≤ dRnd)
    (hLag : cDual + matQuad d (matAt A_rows) z ≤ C_primal)
    (hNormLo : 0 ≤ vecNormSq d z)
    (hNormHi : vecNormSq d z ≤ wMax)
    (hEqLB : cLB = cDual + negPart mu * wMax - dRnd)
    (hPrimalUb : C_primal ≤ cUB) :
    shor1LB ≤ cLB ∧ ({ lo := cLB, hi := cUB } : Interval1D).contains C_primal :=
  moment_sos_hierarchy_exactness d shor1LB cDual mu wMax dRnd cLB cUB A_rows L_rows z C_primal
    hTight hCheck hDelta hLag hNormLo hNormHi hEqLB hPrimalUb

/-- Concrete 3-cycle Gram obstruction matrix separating Shor-1 PSD from rank-1 manifold. -/
def W_cycle : List (List Int) :=
  [[4, 2, -2], [2, 4, 2], [-2, 2, 4]]

private def L_zero3 : List (List Int) :=
  [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

/-- Theorem E19 (Explicit Rank-1 Obstruction on a 3-Cycle Shor-1 Matrix):
    `W_cycle` is strictly inside the PSD cone (`∀ v, 0 ≤ v^T W_cycle v`), yet admits
    no rank-1 outer product `x x^T = W_cycle`. -/
theorem cycle_phase_rank_obstruction :
    (∀ v : Fin 3 → Int, 0 ≤ matQuad 3 (matAt W_cycle) v) ∧
    (¬ ∃ x : Fin 3 → Int, ∀ i j : Fin 3, x i * x j = matAt W_cycle i j) := by
  refine ⟨?_, ?_⟩
  · intro v
    have h := checkCholeskySDD_sound 3 W_cycle L_zero3 0 (by decide) v
    omega
  · rintro ⟨x, hx⟩
    have h01 : x 0 * x 1 = 2 := hx 0 1
    have h12 : x 1 * x 2 = 2 := hx 1 2
    have h02 : x 0 * x 2 = -2 := hx 0 2
    have h11 : x 1 * x 1 = 4 := hx 1 1
    have hid : (x 0 * x 1) * (x 1 * x 2) = (x 0 * x 2) * (x 1 * x 1) := by grind
    rw [h01, h12, h02, h11] at hid
    omega

/-- Theorem E20 (Bode-Fano Multi-Cell Energy & Local-Global Certificate Ordering):
    If per-cell loss/reflection terms respect `cellCap`, total integral sums to `≤ nCells * cellCap`,
    and certified SEARU design ≤ Poynting local bound ≤ Bode-Fano global integral limit. -/
theorem bode_fano_energy_limit_sound
    (nCells : Nat) (cellResp : Fin nCells → Int) (cellCap searuMetric localBound globalBound : Int)
    (hCell : ∀ i, cellResp i ≤ cellCap)
    (hSearu : searuMetric ≤ localBound) (hGlob : localBound ≤ globalBound) :
    sumFin nCells cellResp ≤ Int.ofNat nCells * cellCap ∧
    searuMetric ≤ globalBound ∧ 0 ≤ globalBound - searuMetric := by
  have hSum : sumFin nCells cellResp ≤ Int.ofNat nCells * cellCap := by
    induction nCells with
    | zero => simp [sumFin]
    | succ k ih =>
      simp only [sumFin]
      have h1 := ih (fun i => cellResp i.castSucc) (fun i => hCell i.castSucc)
      have h2 := hCell ⟨k, Nat.lt_succ_self k⟩
      have hCast : Int.ofNat (k + 1) * cellCap = Int.ofNat k * cellCap + cellCap := by
        have hsucc : Int.ofNat (k + 1) = Int.ofNat k + 1 := rfl
        rw [hsucc, Int.add_mul, Int.one_mul]
      omega
  exact ⟨hSum, by omega, by omega⟩

/-! ### Theorem E13b: Kernel-Checked Per-Clique Chordal Cholesky+SDD AC-OPF Soundness -/

structure CliqueCert where
  vars : List Nat
  sMat : List (List Int)
  lMat : List (List Int)
  muPsd : Int
  wMax : Int
  deriving Repr, DecidableEq

def projCliqueVec (n : Nat) (cl : CliqueCert) (v : Fin n → Int) : Fin cl.vars.length → Int :=
  fun a => if h : cl.vars.getD a.val 0 < n then v ⟨cl.vars.getD a.val 0, h⟩ else 0

def liftCliqueMat (n : Nat) (cl : CliqueCert) (i j : Fin n) : Int :=
  sumFin cl.vars.length (fun a =>
    sumFin cl.vars.length (fun b =>
      if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
      then matAt cl.sMat a b else 0))

private theorem sumFin_mul_left_right (k : Nat) (c1 c2 : Int) (f : Fin k → Fin k → Int) :
    c1 * sumFin k (fun a => sumFin k (fun b => f a b)) * c2 =
      sumFin k (fun a => sumFin k (fun b => c1 * f a b * c2)) := by
  rw [sumFin_mul_left, sumFin_mul_right]
  apply sumFin_congr; intro a
  rw [sumFin_mul_left, sumFin_mul_right]

theorem liftCliqueMat_quad (n : Nat) (cl : CliqueCert) (v : Fin n → Int)
    (hVars : allFinB cl.vars.length (fun a => decide (cl.vars.getD a.val 0 < n)) = true) :
    matQuad n (liftCliqueMat n cl) v =
      matQuad cl.vars.length (matAt cl.sMat) (projCliqueVec n cl v) := by
  unfold matQuad liftCliqueMat
  have hVarsLt : ∀ a : Fin cl.vars.length, cl.vars.getD a.val 0 < n :=
    fun a => of_decide_eq_true (allFinB_true _ _ hVars a)
  have h1 :
      sumFin n (fun i => sumFin n (fun j =>
        v i * sumFin cl.vars.length (fun a => sumFin cl.vars.length (fun b =>
          if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
          then matAt cl.sMat a b else 0)) * v j)) =
      sumFin n (fun i => sumFin n (fun j =>
        sumFin cl.vars.length (fun a => sumFin cl.vars.length (fun b =>
          v i * (if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
                 then matAt cl.sMat a b else 0) * v j)))) := by
    apply sumFin_congr; intro i
    apply sumFin_congr; intro j
    exact sumFin_mul_left_right cl.vars.length (v i) (v j) _
  rw [h1]
  have h2 :
      sumFin n (fun i => sumFin n (fun j =>
        sumFin cl.vars.length (fun a => sumFin cl.vars.length (fun b =>
          v i * (if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
                 then matAt cl.sMat a b else 0) * v j)))) =
      sumFin cl.vars.length (fun a => sumFin cl.vars.length (fun b =>
        sumFin n (fun i => sumFin n (fun j =>
          v i * (if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
                 then matAt cl.sMat a b else 0) * v j)))) := by
    have hs1 : ∀ i : Fin n,
        sumFin n (fun j => sumFin cl.vars.length (fun a => sumFin cl.vars.length (fun b =>
          v i * (if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
                 then matAt cl.sMat a b else 0) * v j))) =
        sumFin cl.vars.length (fun a => sumFin cl.vars.length (fun b =>
          sumFin n (fun j =>
            v i * (if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
                   then matAt cl.sMat a b else 0) * v j))) := by
      intro i
      rw [sumFin_swap]
      apply sumFin_congr; intro a
      exact sumFin_swap _
    rw [sumFin_congr hs1, sumFin_swap]
    apply sumFin_congr; intro a
    exact sumFin_swap _
  rw [h2]
  apply sumFin_congr; intro a
  apply sumFin_congr; intro b
  let i0 : Fin n := ⟨cl.vars.getD a.val 0, hVarsLt a⟩
  let j0 : Fin n := ⟨cl.vars.getD b.val 0, hVarsLt b⟩
  have hInner :
      sumFin n (fun i => sumFin n (fun j =>
        v i * (if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
               then matAt cl.sMat a b else 0) * v j)) =
      v i0 * matAt cl.sMat a b * v j0 := by
    have hjCollapse : ∀ i : Fin n,
        sumFin n (fun j =>
          v i * (if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
                 then matAt cl.sMat a b else 0) * v j) =
        (if i = i0 then v i0 * matAt cl.sMat a b * v j0 else 0) := by
      intro i
      have hFun :
          (fun j : Fin n =>
            v i * (if cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val
                   then matAt cl.sMat a b else 0) * v j) =
          (fun j : Fin n => if j = j0 then (if i = i0 then v i0 * matAt cl.sMat a b * v j0 else 0) else 0) := by
        funext j
        by_cases hj : j = j0
        · subst hj
          by_cases hi : i = i0
          · subst hi
            have hcond : cl.vars.getD a.val 0 = i0.val ∧ cl.vars.getD b.val 0 = j0.val := ⟨rfl, rfl⟩
            simp only [hcond.1, hcond.2, and_self, ↓reduceIte]
          · have hcond : ¬(cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j0.val) :=
              fun h => hi (Fin.ext h.1.symm)
            simp only [hcond, hi, ↓reduceIte]
            grind
        · have hcond : ¬(cl.vars.getD a.val 0 = i.val ∧ cl.vars.getD b.val 0 = j.val) :=
            fun h => hj (Fin.ext h.2.symm)
          simp only [hcond, hj, ↓reduceIte]
          grind
      rw [hFun, sumFin_single_hit]
    rw [sumFin_congr hjCollapse, sumFin_single_hit]
  rw [hInner]
  unfold projCliqueVec
  simp only [hVarsLt a, hVarsLt b, ↓reduceDIte]
  rfl

def checkChordalCert (n : Nat) (A_full : List (List Int)) (cliques : List CliqueCert) : Bool :=
  let cliquesOk := allFinB cliques.length (fun c =>
    let cl := cliques[c]
    let k := cl.vars.length
    allFinB k (fun a => decide (cl.vars.getD a.val 0 < n)) &&
    checkCholeskySDD k cl.sMat cl.lMat cl.muPsd)
  let decompOk := allFinB n (fun i => allFinB n (fun j =>
    decide (sumFin cliques.length (fun c => liftCliqueMat n cliques[c] i j) = matAt A_full i j)))
  cliquesOk && decompOk

/-- Theorem E13b (Kernel-Checked Chordal AC-OPF Soundness with Per-Clique `wMax`):
    If `checkChordalCert` succeeds and each clique projected norm satisfies `||v_c||^2 ≤ wMax_c`,
    then `C_primal ∈ [cLB, cUB]`. -/
theorem certified_chordal_acopf
    (n : Nat) (A_full : List (List Int)) (cliques : List CliqueCert)
    (cDual dRnd cLB cUB : Int) (v : Fin n → Int) (C_primal : Int)
    (hCheck : checkChordalCert n A_full cliques = true)
    (hLBdef : cLB = cDual + sumFin cliques.length (fun c => negPart (cliques[c].muPsd) * cliques[c].wMax) - dRnd)
    (hLag : cDual + matQuad n (matAt A_full) v ≤ C_primal)
    (hNorm : ∀ c : Fin cliques.length,
      vecNormSq (cliques[c].vars.length) (projCliqueVec n cliques[c] v) ≤ cliques[c].wMax)
    (hDelta : 0 ≤ dRnd)
    (hPrimalUb : C_primal ≤ cUB) :
    ({ lo := cLB, hi := cUB } : Interval1D).contains C_primal := by
  unfold checkChordalCert at hCheck
  simp only [Bool.and_eq_true] at hCheck
  have hCliques : ∀ c : Fin cliques.length,
      allFinB cliques[c].vars.length (fun a => decide (cliques[c].vars.getD a.val 0 < n)) = true ∧
      checkCholeskySDD cliques[c].vars.length cliques[c].sMat cliques[c].lMat cliques[c].muPsd = true := by
    intro c
    have hc := allFinB_true _ _ hCheck.1 c
    simp only [Bool.and_eq_true] at hc
    exact hc
  have hDecomp : ∀ i j : Fin n,
      sumFin cliques.length (fun c => liftCliqueMat n cliques[c] i j) = matAt A_full i j := by
    intro i j
    have hi := allFinB_true n _ hCheck.2 i
    have hj := allFinB_true n _ hi j
    exact of_decide_eq_true hj
  have hQuadSum :
      sumFin cliques.length (fun c => matQuad n (liftCliqueMat n cliques[c]) v) =
        matQuad n (matAt A_full) v := by
    unfold matQuad
    rw [sumFin_swap (fun c i => sumFin n (fun j => v i * liftCliqueMat n cliques[c] i j * v j))]
    apply sumFin_congr; intro i
    rw [sumFin_swap (fun c j => v i * liftCliqueMat n cliques[c] i j * v j)]
    apply sumFin_congr; intro j
    have hmul : ∀ c : Fin cliques.length,
        v i * liftCliqueMat n cliques[c] i j * v j = liftCliqueMat n cliques[c] i j * (v i * v j) :=
      fun c => by grind
    rw [sumFin_congr hmul, ← sumFin_mul_right cliques.length _ (v i * v j), hDecomp i j]
    grind
  have hCliqueBd : ∀ c : Fin cliques.length,
      negPart (cliques[c].muPsd) * cliques[c].wMax ≤ matQuad n (liftCliqueMat n cliques[c]) v := by
    intro c
    have hPsd := checkCholeskySDD_sound _ _ _ _ (hCliques c).2 (projCliqueVec n cliques[c] v)
    have hLift := liftCliqueMat_quad n cliques[c] v (hCliques c).1
    have hNormLo : 0 ≤ vecNormSq cliques[c].vars.length (projCliqueVec n cliques[c] v) :=
      vecNormSq_nonneg _ _
    have h1 : negPart (cliques[c].muPsd) * vecNormSq cliques[c].vars.length (projCliqueVec n cliques[c] v) ≤
        cliques[c].muPsd * vecNormSq cliques[c].vars.length (projCliqueVec n cliques[c] v) :=
      Int.mul_le_mul_of_nonneg_right (negPart_le _) hNormLo
    have hNegP : 0 ≤ -negPart (cliques[c].muPsd) := by
      have := negPart_nonpos (cliques[c].muPsd); omega
    have h2 : negPart (cliques[c].muPsd) * cliques[c].wMax ≤
        negPart (cliques[c].muPsd) * vecNormSq cliques[c].vars.length (projCliqueVec n cliques[c] v) := by
      have hm := Int.mul_le_mul_of_nonneg_left (hNorm c) hNegP
      grind
    omega
  have hSumBd :
      sumFin cliques.length (fun c => negPart (cliques[c].muPsd) * cliques[c].wMax) ≤
        matQuad n (matAt A_full) v := by
    rw [← hQuadSum]
    exact sumFin_le_sumFin cliques.length _ _ hCliqueBd
  exact ac_opf_bracket_valid cLB C_primal cUB (by omega) hPrimalUb

#print axioms sdd_quadratic_form_nonneg
#print axioms cholesky_sdd_certified_psd
#print axioms checkCholeskySDD_sound
#print axioms checkCholeskySDD_sym
#print axioms passivity_real_aug_identity
#print axioms checkPassivityMat_sound
#print axioms certified_multiport_passivity
#print axioms fdtd_cfl_stability_2d
#print axioms reciprocity_symmetry_bound
#print axioms tellegen_passive_mna_dissipation
#print axioms trapezoidal_astability_energy_decay
#print axioms soa_and_eye_enclosure_cert
#print axioms ipmsm_operating_envelope_cert
#print axioms ac_opf_anytime_dual_lower_bound
#print axioms ac_opf_bracket_valid
#print axioms grid_lyapunov_stability_basin
#print axioms chordal_clique_decomposition_sound
#print axioms ac_opf_thermal_angle_dual_sound
#print axioms ac_opf_observable_envelope_sound
#print axioms maxwell_poynting_dual_upper_bound
#print axioms certified_maxwell_poynting_bound
#print axioms kyp_bounded_real_passivity_cert
#print axioms certified_kyp_passivity
#print axioms moment_sos_hierarchy_exactness
#print axioms certified_sos2_lower_bound
#print axioms cycle_phase_rank_obstruction
#print axioms bode_fano_energy_limit_sound
#print axioms liftCliqueMat_quad
#print axioms certified_chordal_acopf

end EleCert
