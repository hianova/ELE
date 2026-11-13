export PATH := /opt/homebrew/opt/llvm/bin:/opt/homebrew/bin:/usr/local/bin:$(PATH)

CC        ?= clang
VERILATOR ?= verilator
YOSYS     ?= yosys
PREFIX    ?= /usr/local

FLOW_DIR := $(abspath ../FLOW)
FLOW_H   := $(FLOW_DIR)/flow.h

BUILD_DIR     := build
FLOW_OBJ      := $(BUILD_DIR)/flow_kernel.o
VERILATOR_DIR := $(BUILD_DIR)/verilator
COSIM_BIN     := $(VERILATOR_DIR)/Vele_cosim_tb

RTL_IPS := rtl/ele_em_phy.sv rtl/ele_flow_core.sv
RTL_TB  := rtl/ele_cosim_tb.sv

.DEFAULT_GOAL := all
.PHONY: all test clean install uninstall hygiene-gate cosim-gate synth-gate

all: hygiene-gate $(COSIM_BIN)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

hygiene-gate:
	@echo "=== [Gate 1] ELE Pure-SV Source Hygiene Gate (0 .lean, 0 .c/.h/.cpp) ==="
	@stray="$$(find . -maxdepth 3 \( -name '*.lean' -o -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.cc' \) -not -path './build/*')" && \
		if [ -n "$$stray" ]; then echo "ERROR: Non-SV source files found in ELE: $$stray" >&2; exit 1; fi
	@test -f "$(FLOW_H)" || (echo "ERROR: Missing upstream $(FLOW_H)" >&2 && exit 1)
	@echo "  [PASS] Pure IEEE 1800 SystemVerilog repository verified (0 .lean, 0 .c/.h/.cpp)"

$(FLOW_OBJ): $(FLOW_H) | $(BUILD_DIR)
	$(CC) -x c -std=c99 -O3 -Wall -Wextra -Werror -DFLOW_EXPORT_KERNEL -c "$(FLOW_H)" -o $@

$(COSIM_BIN): $(RTL_IPS) $(RTL_TB) $(FLOW_OBJ) | $(BUILD_DIR)
	@echo "=== [Gate 2] Verilator -Wall Strict RTL Lint & Direct FLOW v6.0.0 DPI-C Build ==="
	@mkdir -p $(VERILATOR_DIR)
	$(VERILATOR) --lint-only -Wall --top-module ele_flow_core rtl/ele_flow_core.sv
	$(VERILATOR) --lint-only -Wall --top-module ele_flow_atlas_top rtl/ele_flow_core.sv
	$(VERILATOR) --lint-only -Wall --top-module ele_bus_invert65 rtl/ele_em_phy.sv
	$(VERILATOR) --lint-only -Wall --top-module ele_ternary_mac_dualrail rtl/ele_em_phy.sv
	$(VERILATOR) --binary -Wall -Mdir $(VERILATOR_DIR) --top-module ele_cosim_tb \
		$(RTL_IPS) $(RTL_TB) "$(abspath $(FLOW_OBJ))" -o Vele_cosim_tb
	@echo "  [PASS] $(COSIM_BIN) built cleanly with 0 warnings"

cosim-gate: hygiene-gate $(COSIM_BIN)
	@echo "=== [Gate 3] 0-ULP 512-Bit Silicon Wind-Tunnel & CMOS Switching-Energy Telemetry ==="
	./$(COSIM_BIN)

synth-gate: $(RTL_IPS)
	@echo "=== [Gate 4] Yosys CMOS Gate-Level Synthesis & Zero-Latch DRC ==="
	$(YOSYS) -q -p "read_verilog -nolatches -sv rtl/ele_flow_core.sv; synth -top ele_flow_core; check -assert; select -assert-none t:\$$_DLATCH* t:\$$dlatch*"
	$(YOSYS) -q -p "read_verilog -nolatches -sv rtl/ele_flow_core.sv; chparam -set MAX_CHARTS 4 ele_flow_atlas_top; synth -top ele_flow_atlas_top; check -assert; select -assert-none t:\$$_DLATCH* t:\$$dlatch*"
	$(YOSYS) -q -p "read_verilog -nolatches -sv rtl/ele_em_phy.sv; synth -top ele_bus_invert65; check -assert; select -assert-none t:\$$_DLATCH* t:\$$dlatch*"
	$(YOSYS) -q -p "read_verilog -nolatches -sv rtl/ele_em_phy.sv; synth -top ele_ternary_mac_dualrail; check -assert; select -assert-none t:\$$_DLATCH* t:\$$dlatch*"
	@echo "  [PASS] Yosys synthesized ele_flow_core, ele_flow_atlas_top, ele_bus_invert65 & ele_ternary_mac_dualrail (0 latches, 0 DRC errors)"

test: cosim-gate synth-gate
	@echo "================================================================================"
	@echo "   ALL 4 ELE v6.0.0 SILICON GATES PASSED (PURE SV, 0-ULP 512-BIT, 0 LATCHES)"
	@echo "================================================================================"

install: hygiene-gate
	mkdir -p $(DESTDIR)$(PREFIX)/share/ele/rtl
	install -m 644 rtl/ele_flow_core.sv $(DESTDIR)$(PREFIX)/share/ele/rtl/ele_flow_core.sv
	install -m 644 rtl/ele_em_phy.sv $(DESTDIR)$(PREFIX)/share/ele/rtl/ele_em_phy.sv

uninstall:
	rm -rf $(DESTDIR)$(PREFIX)/share/ele

clean:
	rm -rf $(BUILD_DIR)
