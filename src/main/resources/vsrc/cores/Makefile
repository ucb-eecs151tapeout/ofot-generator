#########################################################################################
# Pre-process EECS151 core Verilog into a single blackbox file
#########################################################################################

## Overridable params
vsrc_dir =$(abspath ./)
PROJ_NAME = EECS151-Fa24-ASIC-Project# or your own
MACHINE_NAME = eda# or "bwrc"
RUN_TYPE = pd# or "sim"

#########################################################################################
# Setup
#########################################################################################

# Path to Verilog folder
PREPROC_VERILOG = $(vsrc_dir)/$(PROJ_NAME).preprocessed.v
.PHONY: default $(PREPROC_VERILOG)
.SILENT: $(PREPROC_VERILOG)
default: $(PREPROC_VERILOG)

# Collateral path
sram_dir = "/scratch/ee198-20-tab/sram22_sky130_macros"
ifeq ($(MACHINE_NAME),bwrc)
$(info "############ switched to bwrc settings ############")
	sram_dir = "/home/ff/eecs251b/sky130/sram22_sky130_macros"
endif

# Fake SRAM Macros for sims
sram_include_str = ""
ifeq ($(RUN_TYPE),sim)
$(info  "############ switched to sim settings ############")
	sram_include_str = "\`include \"$(sram_dir)/sram22_256x32m4w8/sram22_256x32m4w8.v\"\n\`include \"$(sram_dir)/sram22_64x32m4w8/sram22_64x32m4w8.v\"\n\`include \"$(sram_dir)/sram22_512x64m4w8/sram22_512x64m4w8.v\"\n\`include \"$(sram_dir)/sram22_64x24m4w24/sram22_64x24m4w24.v\"\n"
endif
## Notes from elam:
# This is tuning wmask to a wire rather than a 1 width bus in sram22_64x24m4w24 bc it is otherwise incompatible with the lib
# This only works for sim - it causes the sram macros to not be placed if you try to run synthesis 
# (bc it maps the sram modules to some trivial verilog)

#########################################################################################
# Includes and vsrcs
#########################################################################################

lookup_srcs = $(shell find -L $(1)/ -name target -prune -o -iname "*.$(2)" -print 2> /dev/null)

VLOG_DIR = $(vsrc_dir)/$(PROJ_NAME)/src

ALL_VSRCS = $(VLOG_DIR)/*.v

INC_DIRS = $(VLOG_DIR)/

#########################################################################################
# Pre-process using custom script to replace the includes (but leave rest unaffected)
#########################################################################################

# Note: Comments have to be untabbed to not get printed out and flood log

PREPROC_SCRIPT = $(vsrc_dir)/../../../../../../../scripts/insert-includes.py

$(PREPROC_VERILOG): $(ALL_VSRCS)
	rm -rf combined.v
	mkdir -p $(dir $(PREPROC_VERILOG))

# elam: "Don't include testbenches in the verilog because I'm lazy [fake news] and don't want to 
# fix the msb/lsb thing in them and we won't use them anyways" - 
	touch combined.v
	find $(ALL_VSRCS) | grep -iv test | xargs -I{} sh -c "cat {}; echo ''" >> combined.v

# Unique-ify ALU module name which colides with rocketchip
	sed -i 's/ALU ALU/custom_ALU custom_ALU/g' combined.v
	sed -i 's/ALU(/custom_ALU(/g' combined.v

# The riscv_top module name needs to match the Scala blackbox name
	sed -i 's/module riscv_top/module OneFiftyOneCoreBlackBox/g' combined.v

### Notes: How may we get around this?
## Changing the modules is easy:
# sed -i 's/module riscv_top/module OneFiftyOneCore_$(PROJ_NAME)_BlackBox/g' combined.v
## But to make multiple black boxes...
# Either we use 
#/scalastyle:off
#//turn off linter: blackbox name must match verilog module
#ElaborationArtefacts.add(...
#....and then hardcode the TCL >_< )
# //scalastyle:on
# (That's what SiFive does..?)
## An interesting exception is by using addPath and some other processing?
# Aka: chipyard/generators/caliptra-aes-acc/src/main/scala/aes/AesCipherCore.scala
# But I think they have the same issue..
## Or we use some metaprogramming macros to manipulate the scala at compile time?
## Or create a scala factory that chooses between black boxes? 
# `def createClass(className: String): MyClass = className match {` type deal?
## More realistically, since we're only doing a few cores at most, 
# could just go back to making multiple classes manually?
# I think that's most reasonable right now.
## Hence I ignore this for now.. >_< but leaving it as easter hunt for anyone who reads this ig.

	$(PREPROC_SCRIPT) combined.v $@ $(INC_DIRS)

# Disable the cache for easier initial tilelink interface setup/debuggin
# TODO this breaks sim, see README (but the README is long gone and i forgor oops - ask elam)
# I think can leave it disabled for both sim and syn
# printf "\`define no_cache_mem\n" | cat - $@ > temp && mv temp $@

# Include SRAM Verilog file, if any, at the top of the preprocessed file
	printf $(sram_include_str) | cat - $@ > temp && mv temp $@

# "FIXME FIXME FIXME getting into sketch hack territory here" - elam
# This causes a width mismatch
	sed -i 's/`define CACHE_DEPTH_BITS_IMEM 7/`define CACHE_DEPTH_BITS_IMEM 6/g' $@

	rm -rf combined.v

	@echo "############ pre-processing complete! ############"

clean:
	rm -rf $(vsrc_dir)/*.preprocessed.v
