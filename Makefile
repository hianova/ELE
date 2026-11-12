export PATH := $(HOME)/.elan/bin:/opt/homebrew/opt/llvm/bin:/opt/homebrew/bin:/usr/local/bin:$(PATH)

LEAN      ?= lean
VERILATOR ?= verilator
YOSYS     ?= yosys
PREFIX    ?= /usr/local

FLOW_DIR   ?= $(abspath ../FLOW)
UNIT_DIR   ?= $(abspath ../UNIT)
FLOW_BUILD := $(FLOW_DIR)/build
UNIT_BUILD := $(UNIT_DIR)/build
FLOW_BIN   ?= $(FLOW_BUILD)/flow

BUILD_DIR     := build
ELE_OLEAN     := $(BUILD_DIR)/Ele.olean
VERILATOR_DIR := $(BUILD_DIR)/verilator
COSIM_BIN     := $(VERILATOR_DIR)/Vele_cosim_tb

RTL_IPS := rtl/ele_em_phy.sv rtl/ele_flow_core.sv
RTL_TB  := rtl/ele_cosim_tb.sv

.DEFAULT_GOAL := all
.PHONY: all test clean install uninstall upstream-deps synth-gate

all: $(ELE_OLEAN) $(COSIM_BIN)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

upstream-deps:
	@test -f "$(FLOW_BUILD)/Flow.olean" && test -x "$(FLOW_BIN)" || $(MAKE) -C "$(FLOW_DIR)" all
	@test -f "$(UNIT_BUILD)/Unit.olean" || $(MAKE) -C "$(UNIT_DIR)" all

$(ELE_OLEAN): Ele.lean | $(BUILD_DIR) upstream-deps
	@echo "=== [Gate 1] ELE Pure Lean 4 Formal Proof Check (0 sorryAx, 0 C/C++ files) ==="
	LEAN_PATH="$(FLOW_BUILD):$(UNIT_BUILD)" $(LEAN) -DwarningAsError=true -o $@ Ele.lean
	@ax="$$(printf 'import Ele\n#print axioms Ele.Electron.fll_quadratic_envelope_guarantees_zero_sdc\n#print axioms Ele.Electron.fll_guardband_elimination_strictly_saves_dynamic_energy\n#print axioms Ele.Electron.fllEmergencyIsolatesDroopSurge\n#print axioms Ele.Power.sscb_jet2_separates_rc_inrush_from_short_circuit\n#print axioms Ele.Power.phase_interleaving_conserves_work_cuts_peak_and_i2r\n#print axioms Ele.Power.sscbTrippedIsolatesShortCurrent\n#print axioms Ele.MagWave.rmw_preserves_unmasked_datasheet_bits\n#print axioms Ele.MagWave.bus_invert_65wire_strictly_caps_flips_at_32\n#print axioms Ele.MagWave.dual_rail_ternary_zero_ulp_equivalence\n#print axioms Ele.MagWave.symplectic_spread_spectrum_conserves_two_cycle_period\n' | LEAN_PATH="$(FLOW_BUILD):$(UNIT_BUILD):$(BUILD_DIR)" $(LEAN) --stdin)" && \
		! printf '%s\n' "$$ax" | grep -q 'sorryAx'
	@test -z "$$(find . -maxdepth 3 \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.cc' \) -not -path './build/*')"
	@echo "  [PASS] $(ELE_OLEAN) compiled (0 sorryAx, 0 .c/.h/.cpp files)"

$(COSIM_BIN): $(RTL_IPS) $(RTL_TB) | $(BUILD_DIR)
	@echo "=== [Gate 2] Verilator -Wall Strict RTL Lint & Pure-SV Binary Build ==="
	@mkdir -p $(VERILATOR_DIR)
	$(VERILATOR) --lint-only -Wall --top-module ele_flow_core rtl/ele_flow_core.sv
	$(VERILATOR) --lint-only -Wall --top-module ele_flow_atlas_top rtl/ele_flow_core.sv
	$(VERILATOR) --lint-only -Wall --top-module ele_bus_invert65 rtl/ele_em_phy.sv
	$(VERILATOR) --lint-only -Wall --top-module ele_ternary_mac_dualrail rtl/ele_em_phy.sv
	$(VERILATOR) --binary -Wall -Mdir $(VERILATOR_DIR) --top-module ele_cosim_tb \
		$(RTL_IPS) $(RTL_TB) -o Vele_cosim_tb
	@echo "  [PASS] $(COSIM_BIN) built cleanly from pure IEEE 1800 SystemVerilog (0 warnings)"

synth-gate: $(RTL_IPS)
	@echo "=== [Gate 4] Yosys CMOS Gate-Level Synthesis & Zero-Latch DRC ==="
	$(YOSYS) -q -p "read_verilog -nolatches -sv rtl/ele_flow_core.sv; synth -top ele_flow_core; check -assert; select -assert-none t:\$$_DLATCH* t:\$$dlatch*"
	$(YOSYS) -q -p "read_verilog -nolatches -sv rtl/ele_em_phy.sv; synth -top ele_bus_invert65; check -assert; select -assert-none t:\$$_DLATCH* t:\$$dlatch*"
	$(YOSYS) -q -p "read_verilog -nolatches -sv rtl/ele_em_phy.sv; synth -top ele_ternary_mac_dualrail; check -assert; select -assert-none t:\$$_DLATCH* t:\$$dlatch*"
	@echo "  [PASS] Yosys synthesized ele_flow_core, ele_bus_invert65 & ele_ternary_mac_dualrail (0 latches, 0 DRC errors)"

test: $(ELE_OLEAN) $(COSIM_BIN) TestEle.lean
	@echo "=== [Gate 3] 3-Way 0-ULP 512-Bit Co-Verification (Lean == FLOW C99 == Verilator SV) ==="
	FLOW_BIN="$(FLOW_BIN)" LEAN_PATH="$(FLOW_BUILD):$(UNIT_BUILD):$(BUILD_DIR)" \
		$(LEAN) -DwarningAsError=true --run TestEle.lean
	@$(MAKE) synth-gate
	@echo "================================================================================"
	@echo "   ALL 4 ELE VERIFICATION GATES PASSED (0 .c/.h/.cpp, 0 sorryAx, 0-ULP, 0 LATCHES)"
	@echo "================================================================================"

install: $(ELE_OLEAN)
	mkdir -p $(DESTDIR)$(PREFIX)/lib/flow $(DESTDIR)$(PREFIX)/share/ele/rtl
	install -m 644 Ele.lean $(DESTDIR)$(PREFIX)/lib/flow/Ele.lean
	install -m 644 $(ELE_OLEAN) $(DESTDIR)$(PREFIX)/lib/flow/Ele.olean
	install -m 644 rtl/ele_flow_core.sv $(DESTDIR)$(PREFIX)/share/ele/rtl/ele_flow_core.sv
	install -m 644 rtl/ele_em_phy.sv $(DESTDIR)$(PREFIX)/share/ele/rtl/ele_em_phy.sv

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/lib/flow/Ele.lean $(DESTDIR)$(PREFIX)/lib/flow/Ele.olean
	rm -rf $(DESTDIR)$(PREFIX)/share/ele

clean:
	rm -rf $(BUILD_DIR)
