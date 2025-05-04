package ofo

import ofo.OneFiftyOneCtrlRegs._
import chisel3._
import chisel3.util._
import freechips.rocketchip.tile.CoreParams
import freechips.rocketchip.rocket.MulDivParams
import freechips.rocketchip.tile.FPUParams

case class OneFiftyOneCoreParams(
    //val projectName: String = "kevin-kore",
    val projectName: String = "E151LA",
    val MEM_ADDR_BITS: Int = REG_WIDTH(MEM_REQ_ADDR),
    val MEM_TAG_BITS: Int = REG_WIDTH(MEM_REQ_TAG),
    val MEM_DATA_BITS: Int = MEM_REQ_DATA_BITS_WIDTH,
    val bootFreqHz: BigInt = 50000000, // Frequency
    useVM: Boolean = false, // Support virtual memory
    useUser: Boolean = false, // Support user mode
    useSupervisor: Boolean = false, // Support supervisor mode
    useDebug: Boolean = false, // Support RISC-V debug specs
    useAtomics: Boolean = false, // Support A extension
    useAtomicsOnlyForIO: Boolean =
      false, // Support A extension for memory-mapped IO (may be true even if useAtomics is false)
    useCompressed: Boolean = false, // Support C extension
    useSCIE: Boolean =
      false, // Support custom instructions (in custom-0 and custom-1)
    useRVE: Boolean = false, // Use E base ISA
    mulDiv: Option[MulDivParams] =
      None, // *Rocket specific: M extension related setting (Use Some(MulDivParams()) to indicate M extension supported)
    fpu: Option[FPUParams] =
      None, // F and D extensions and related setting (see below)
    // NOTE: superscalars should override this
    fetchWidth: Int = 1, // Max # of insts fetched every cycle
    decodeWidth: Int = 1, // Max # of insts decoded every cycle
    retireWidth: Int = 1, // Max # of insts retired every cycle
    instBits: Int =
      32, // Instruction bits (if 32 bit and 64 bit are both supported, use 64)
    nLocalInterrupts: Int =
      0, // # of local interrupts (see SiFive interrupt cookbook)
    nPMPs: Int = 0, // # of Physical Memory Protection units
    pmpGranularity: Int =
      0, // Size of the smallest unit of region for PMP unit (must be power of 2)
    nBreakpoints: Int =
      0, // # of hardware breakpoints supported (in RISC-V debug specs)
    useBPWatch: Boolean = false, // Support hardware breakpoints
    nPerfCounters: Int = 0, // # of supported performance counters
    haveBasicCounters: Boolean =
      false, // Support basic counters defined in the RISC-V counter extension
    haveFSDirty: Boolean =
      false, // If true, the core will set FS field in mstatus CSR to dirty when appropriate
    misaWritable: Boolean =
      false, // Support writable misa CSR (like variable instruction bits)
    haveCFlush: Boolean =
      false, // Rocket specific: enables Rocket's custom instruction extension to flush the cache
    nL2TLBEntries: Int = 0, // # of L2 TLB entries
    mtvecInit: Option[BigInt] =
      None, // mtvec CSR (of V extension) initial value
    mtvecWritable: Boolean = false, // If mtvec CSR is writable
    // TODO[fix]: add explanations for this
    lrscCycles: Int = 1,
    mcontextWidth: Int = 0,
    nL2TLBWays: Int = 0,
    nPTECacheEntries: Int = 0,
    scontextWidth: Int = 0,
    traceHasWdata: Boolean = false,
    useConditionalZero: Boolean = false,
    useHypervisor: Boolean = false,
    useNMI: Boolean = false,

  // The # of instruction bits. Use maximum # of bits if your core supports both 32 and 64 bits.
  xLen: Int = 32,
  // no idea what these do.. ~copied from rocket
  pgLevels: Int = 4,
  useZba :Boolean = false,
  useZbb :Boolean = false,
  useZbs:Boolean = false
) extends CoreParams

class OneFiftyOneCoreBlackBox(
    p: OneFiftyOneCoreParams
) extends BlackBox
    with HasBlackBoxResource {

  // Define the 151 core IO
  val io = IO(new Bundle {
    val clk = Input(Clock()) // TODO: this is required for the TL impl but probably needs to be hacked back to a bool for the mmio impl?
    val reset = Input(Bool())
    val mem_req_valid = Output(Bool())
    val mem_req_ready = Input(Bool())
    val mem_req_rw = Output(Bool())
    val mem_req_tag = Output(UInt(p.MEM_TAG_BITS.W))
    val mem_req_addr = Output(UInt(p.MEM_ADDR_BITS.W))

    val mem_req_data_valid = Output(Bool())
    val mem_req_data_ready = Input(Bool())
    val mem_req_data_bits = Output(UInt(p.MEM_DATA_BITS.W))
    val mem_req_data_mask =
      Output(UInt((p.MEM_DATA_BITS / 8).W)) // TODO is this correct
    val mem_resp_valid = Input(Bool())
    val mem_resp_tag = Input(UInt(p.MEM_TAG_BITS.W))
    val mem_resp_data = Input(UInt(p.MEM_DATA_BITS.W))
    val csr = Output(UInt(32.W))
  })
  addResource(s"vsrc/cores/${p.projectName}.preprocessed.v");
}
