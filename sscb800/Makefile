export PATH := $(HOME)/.elan/bin:/opt/homebrew/opt/llvm/bin:/opt/homebrew/bin:/usr/local/bin:$(PATH)

YOSYS     ?= yosys
VERILATOR ?= verilator
CXX       ?= clang++
PYTHON    ?= python3

RTL_SRCS  := ../rtl/ele_primordial.sv ../rtl/ele_power_phy.sv ele_sscb800_ip.sv
BUILD     := build
SIM_DIR   := $(BUILD)/obj_dir
SIM_BIN   := $(SIM_DIR)/Vele_sscb800_ip
BENCH_JS  := $(BUILD)/sscb800_bench.json
FPGA_TXT  := $(BUILD)/fpga_stats.txt

.PHONY: all lint formal synth-fpga sim datasheet clean

all: lint formal synth-fpga sim datasheet

$(BUILD):
	mkdir -p $(BUILD)

lint:
	$(VERILATOR) --lint-only -Wall --top-module ele_sscb800_ip $(RTL_SRCS)
	@echo "  [PASS] Verilator -Wall lint clean on ele_sscb800_ip"

formal:
	@echo "=== [SSCB800 Formal Gate] Unbounded Yosys SAT k-Induction Proofs ==="
	$(YOSYS) -q -p "read_verilog -sv -formal ele_sscb800_ip.sv; prep -top ele_huber_median3_atom -flatten; async2sync; dffunmap; opt -fast -purge; sat -verify -tempinduct -set-init-zero -prove-asserts"
	@echo "  [QED] ele_huber_median3_atom : unbounded k-induction PASS (Theorem 12 1-sample EMI ceiling)"
	$(YOSYS) -q -p "read_verilog -sv -formal $(RTL_SRCS); prep -top ele_sscb800_ip -flatten; async2sync; dffunmap; opt -fast -purge; sat -verify -tempinduct -set-init-zero -prove-asserts"
	@echo "  [QED] ele_sscb800_ip         : unbounded k-induction PASS (0-cyc DESAT, EMI spike immunity, Blackbox freeze, sticky rearm)"

synth-fpga: | $(BUILD)
	@echo "=== [SSCB800 FPGA Synth] Yosys Target Synthesis (iCE40 + Xilinx 7-Series) ==="
	@$(YOSYS) -p "read_verilog -sv $(RTL_SRCS); synth_ice40 -top ele_sscb800_ip; stat" > $(BUILD)/ice40_synth.log 2>&1
	@$(YOSYS) -p "read_verilog -sv $(RTL_SRCS); synth_xilinx -top ele_sscb800_ip; flatten; stat" > $(BUILD)/xilinx_synth.log 2>&1
	@$(PYTHON) gen_datasheet.py --parse-fpga $(BUILD)/ice40_synth.log $(BUILD)/xilinx_synth.log --fpga-out $(FPGA_TXT)

$(SIM_BIN): $(RTL_SRCS) tb_sscb800_windtunnel.cpp | $(BUILD)
	$(VERILATOR) --cc --exe --build -j 0 -Wall --x-assign unique --x-initial unique \
		--top-module ele_sscb800_ip -Mdir $(SIM_DIR) \
		-CFLAGS "-std=c++20 -O2" \
		$(RTL_SRCS) tb_sscb800_windtunnel.cpp

sim: $(SIM_BIN)
	./$(SIM_BIN) --json $(BENCH_JS)

datasheet: $(BENCH_JS) $(FPGA_TXT)
	$(PYTHON) gen_datasheet.py --bench $(BENCH_JS) --fpga $(FPGA_TXT) --out IP_EVAL_DATASHEET.md

clean:
	rm -rf $(BUILD)
