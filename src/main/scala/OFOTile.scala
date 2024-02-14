package ofo

import ofo._

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.prci.ClockSinkParameters
import chisel3.util.log2Ceil
import freechips.rocketchip.rocket.LoadGen

case class OFOTileParams(
    val baseName: String = "shadow_council_core",
    val clockSinkParams: ClockSinkParameters = ClockSinkParameters(),
    val uniqueName: String,
    val memStart: UInt,
    val tileId: Int,
    val core: OneFiftyOneCoreParams,
    regNodeBase: BigInt = 0x4000,
    val blockerCtrlAddr: Option[BigInt] = None,
    val btb: Option[freechips.rocketchip.rocket.BTBParams] = None,
    val dcache: Option[freechips.rocketchip.rocket.DCacheParams] = None,
    val icache: Option[freechips.rocketchip.rocket.ICacheParams] = None
) extends InstantiableTileParams[OFOTile] {
  def instantiate(
      crossing: freechips.rocketchip.subsystem.HierarchicalElementCrossingParamsLike,
      lookup: LookupByHartIdImpl
  )(implicit p: Parameters): OFOTile = {
    new OFOTile(this, crossing, lookup, p)
  }
}

class OFOTile(
    val params: OFOTileParams, // TODO: do we need params? prob should be a seq of core/project names, unless we want them to each have their own tile
    crossing: freechips.rocketchip.subsystem.HierarchicalElementCrossingParamsLike,
    lookup: LookupByHartIdImpl,
    q: Parameters
) extends BaseTile(params, crossing.crossingType, lookup, q)
    with SinksExternalInterrupts // TODO how do we disable this, still need to define haltNode, ceasesNode etc and can't make them None
    with SourcesExternalNotifications {
  // define a register node so the rocket can enable/disable the core and probe state
  val REG_NODE_MASK = 0xff;



  //val slaveNode = visibilityNode


  // Implementation class (See below)
  override lazy val module = new OFOTileImp(this)

  // TODO elam fix, we might need custom rtl to handle interrupts
  val cpuDevice: SimpleDevice =
    new SimpleDevice(
      params.core.projectName,
      Seq(s"shadow_council,${params.core.projectName}", "riscv")
    ) {
      override def parent = Some(ResourceAnchors.cpus)
      override def describe(resources: ResourceBindings): Description = {
        val Description(name, mapping) = super.describe(resources)
        Description(
          name,
          mapping ++
            cpuProperties ++
            // TODO: figure out if we want this or not
            //nextLevelCacheProperty ++
            tileProperties
        )
      }
    }



  // # of bits used in TileLink ID for master node. 4 bits can support 16 master nodes, but you can have a longer ID if you need more.
  val idBits = 4

  // Define a TL client node for our core
  val clientNode = TLClientNode(
    Seq(
      TLMasterPortParameters.v2(
        Seq(
          TLMasterParameters.v2(
            name = "test-ofo-core-client-node",
            sourceId = IdRange(0, idBits),
            requestFifo = true,
          ))
        ),
      )
    )

   val regNode = TLRegisterNode(
    address = Seq(AddressSet(params.regNodeBase, REG_NODE_MASK)),
    device = 
     new SimpleDevice(s"${params.core.projectName}_regs",  
      Seq(s"shadow_council,${params.core.projectName}_regs", "regs/control")),
    beatBytes = 8,
    concurrency = 1)

   // Require TileLink nodes
  val intOutwardNode = None
  val masterNode = visibilityNode
  val slaveNode = TLIdentityNode()


  // referencing sodor_tile.scala
  tlOtherMastersNode := tlMasterXbar.node
  masterNode :=* tlOtherMastersNode
  tlSlaveXbar.node :*= regNode := slaveNode
  DisableMonitors { implicit p => tlSlaveXbar.node :*= slaveNode }

  // TODO add memory taps
  // play nice with the default TL bus size
  tlMasterXbar.node  := 
   //   memoryTapCPU :=
    TLWidthWidget(128/8) := 
      clientNode


  ResourceBinding {
    Resource(cpuDevice, "reg").bind(ResourceAddress(tileId))
  }
}

// referencing the 'adding a custom core' chipyard tutorial
class OFOTileImp(outer: OFOTile) extends BaseTileModuleImp(outer) {

  // TODO: figure out clock and reset signals
  val core = Module(
    new OneFiftyOneCoreBlackBox(outer.params.core)
  )

  val io = core.io
  val NUM_DATA_CYCLES = 4 // TODO: make consts file
  val MEM_REQ_SIZE_BITS = 512
  val MEM_BUS_SIZE_BITS = 128
  // making PC_RESET = 0 so all memory addresses (incl. below 0x2000) from the core are inbound for DRAM
  val MEM_OFFSET = 0x0.U // 0x80000000L.U // - 0x2000.U // map PC_RESET to the start of the DRAM space in tilelink

  // TODO: multiple cores
  // referencing `sodor_tile`
  require(outer.clientNode.out.size == 1)
  require(outer.clientNode.in.size == 0)
  val (tl_out, edge) = outer.clientNode.out(0)

  // Register
  // State
  val s_ready :: s_active :: s_inflight :: s_responding :: Nil = Enum(4)
  val state = RegInit(s_ready)
  // Address and signedness of the request to be used by LoadGen
  val a_address_reg = Reg(UInt(io.mem_req_addr.getWidth.W))

  // Request register - store the request when the input port fires to avoid value changes when sending the TileLink request
  val req_address_reg = Reg(UInt(32.W)) // TODO: don't hardcode this
  val req_tag_reg = Reg(UInt(io.mem_req_tag.getWidth.W))
  val req_data_reg = Reg(UInt(io.mem_req_data_bits.getWidth.W))
  val req_type = Reg(Bool())

  val resp_cntr = Reg(UInt(log2Ceil(NUM_DATA_CYCLES).W))
  val resp_data_reg = Reg(Vec(4, UInt(MEM_BUS_SIZE_BITS.W)))

  val read_req_size = log2Ceil(MEM_REQ_SIZE_BITS/8).U
  
  
  val write_req_size_lg = Wire(UInt(16.W)) //TODO: Stop hardcoding this
  val write_req_size = PopCount(io.mem_req_data_mask)
  
  write_req_size_lg := 0.U
  for (i <- 0 until 5) {
    when(write_req_size(i)) {
      write_req_size_lg := i.U
    }
  }


  // debuggin
  // dontTouch(read_req_size)
  // dontTouch(write_req_size)

  // Sign and size
  // we have a constant request size

  // State logic
  when(state === s_ready && io.mem_req_valid) {
    state := s_active
    req_address_reg := Mux(io.mem_req_rw, Cat(io.mem_req_addr, io.mem_req_tag), Cat(io.mem_req_addr >> 2.U, 0.U(6.W)))
    req_tag_reg := io.mem_req_tag
    req_data_reg := io.mem_req_data_bits
    req_type := io.mem_req_rw
  }
  when(state === s_active && tl_out.a.fire) {
    state := s_inflight
    resp_cntr := 0.U
  }
  when(state === s_inflight && tl_out.d.fire) {
    when (req_type === true.B) {
      state := s_ready
    }.otherwise {
      resp_data_reg(resp_cntr) := tl_out.d.bits.data
      resp_cntr := resp_cntr + 1.U
      when (resp_cntr === 3.U) {
        state := s_responding
        resp_cntr := 0.U
      }
    }
  }
  when(state === s_responding) {
    when (resp_cntr === 3.U) {
      state := s_ready
    }.otherwise {
      resp_cntr := resp_cntr + 1.U
    }
  }

  tl_out.a.valid := state === s_active
  tl_out.d.ready := true.B
  io.mem_req_ready := state === s_ready
  io.mem_req_data_ready := state === s_ready
  
  io.mem_resp_valid := state === s_responding

  // Bookkeeping
  when(tl_out.a.fire) {
    a_address_reg := io.mem_req_addr
  }

  // Build "Get" message
  val (legal_get, get_bundle) = edge.Get(0.U, req_address_reg, read_req_size)
  // Build "Put" message
  val (legal_put, put_bundle) =
    edge.Put(0.U, req_address_reg, write_req_size_lg, req_data_reg)

  // Connect Channel A bundle
  tl_out.a.bits := Mux(io.mem_req_rw, put_bundle, get_bundle)

  // Connect Channel D bundle (read result)
  io.mem_resp_data := resp_data_reg(resp_cntr)
  io.mem_resp_tag := req_tag_reg

  // Handle error
  val legal_op = Mux(io.mem_req_rw, legal_put, legal_get)
  val resp_xp = tl_out.d.bits.corrupt | tl_out.d.bits.denied
  // Since the core doesn't have an external exception port, we have to kill it
  assert(legal_op | !tl_out.a.valid, "Illegal operation")
  assert(!resp_xp | !tl_out.d.valid, "Responds exception")

  dontTouch(legal_get)
  // Tie off unused channels
  tl_out.b.valid := false.B
  tl_out.c.ready := true.B
  tl_out.e.ready := true.B

  // connect clock and reset
  core.io.clk := clock

  // connect reset to mmio 
  val reset_reg = RegInit(true.B)

  core.io.reset := reset_reg 

  outer.regNode.regmap(
     0x0 -> Seq(RegField.w(32, reset_reg)),
     0x4 -> Seq(RegField.r(32, core.io.csr ))
   )

}

case class OFOTileAttachParams(
    tileParams: OFOTileParams,
    crossingParams: freechips.rocketchip.subsystem.HierarchicalElementCrossingParamsLike
) extends CanAttachTile {
  type TileType = OFOTile
  val lookup = PriorityMuxHartIdFromSeq(Seq(tileParams))
}
