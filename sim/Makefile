# Targets: run (default), build, lint, golden, clean. Run from sim/.
.DEFAULT_GOAL := run

TOP    = tb_soc
VVP    = sim.vvp
PYTHON ?= python3

build:
	iverilog -g2012 -Wall -s $(TOP) -o $(VVP) -f files.f

run: build
	@out=$$(vvp $(VVP)); \
	echo "$$out"; \
	echo "$$out" | grep -q "TEST PASSED"

lint:
	verilator --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSED \
	  --top-module soc_top \
	  ../rtl/coulomb.v ../rtl/ocv_lut.v ../rtl/rest_det.v \
	  ../rtl/sanity.v ../rtl/soc_top.v

# regenerate rtl/ocv_lut.mem and test/bms_vectors.mem (stdlib only)
golden:
	cd .. && $(PYTHON) python/bms_model.py

clean:
	rm -f $(VVP) *.vcd
