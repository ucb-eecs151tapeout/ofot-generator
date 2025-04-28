// package ofo

// import chisel3._
// import chisel3.util._
// // Some more imports here

// //////////////////////////////////////////////////////////////////
// // the Config class
// //////////////////////////////////////////////////////////////////

// class WithPlaceboAccel extends Config((site, here, up) => {
//   case TilesLocated(InSubsystem) => {
//     // Copy pasta (with minor edit)!
//     val prev = up(TilesLocated(InSubsystem))
//     val idOffset = up(NumTiles)
//     PlaceboAccelTileAttachParams(
//       tileParams = PlaceboAccelTileParams(
//         core = PlaceboAccelCoreParams(),
//         icache = Some(ICacheParams()),
//         dcache = Some(DCacheParams())
//       ),
//       crossingParams = RocketCrossingParams()
//     ) ++ prev
//   }
//   case NumTiles => up(NumTiles) + n
// })

// //////////////////////////////////////////////////////////////////
// // the CanAttachTile class
// //////////////////////////////////////////////////////////////////

// case class PlaceboAccelTileAttachParams(
//   // Copy pasta!
//   tileParams: PlaceboAccelTileParams,
//   crossingParams: RocketCrossingParams
// ) extends CanAttachTile {
//   // Copy pasta (with minor edit)!
//   type TileType = PlaceboAccelTile
//   val lookup = PriorityMuxHartIdFromSeq(Seq(tileParams))
  
// }

// //////////////////////////////////////////////////////////////////
// // CoreParams class
// //////////////////////////////////////////////////////////////////

// case class PlaceboAccelCoreParams(
//   // Some custom params 
    
// ) extends CoreParams {
//   // A whole bunch of default params
//   // Copy pasta (with minor edit)!
  
// }

// //////////////////////////////////////////////////////////////////
// // the InstantiableTileParams class
// //////////////////////////////////////////////////////////////////

// case class PlaceboAccelTileParams(
//   // Copy pasta (with minor edit)!
//   val core: CoreParams,                  // Core parameters (see below)
//   val icache: Option[ICacheParams],      // Rocket specific: I1 cache option
//   val dcache: Option[DCacheParams]      // Rocket specific: D1 cache option

// ) extends InstantiableTileParams[PlaceboAccelTile] {
//   // Copy pasta (with minor edit)!
//     def instantiate(crossing: TileCrossingParamsLike, lookup: LookupByHartIdImpl)
//                     (implicit p: Parameters): TileType

// }

// //////////////////////////////////////////////////////////////////
// // the actual Tile class
// //////////////////////////////////////////////////////////////////

// class PlaceboAccelTile(
//   // Copy pasta (with minor edit)!
//   val params: PlaceboAccelTileParams,
//   crossing: TileCrossingParamsLike,
//   lookup: LookupByHartIdImpl,
//   q: Parameters
// ) extends BaseTile(params, crossing.crossingType, lookup, q) {

//   //////////////////////////////////////////////////////////////////
//   // instantiate Implementation class
//   //////////////////////////////////////////////////////////////////
//   override lazy val module = new PlaceboAccelTileImp(this)

//   //////////////////////////////////////////////////////////////////
//   // required entry of CPU device in the device tree for interrupt purpose
//   //////////////////////////////////////////////////////////////////
//   val cpuDevice: SimpleDevice = new SimpleDevice("cpu", Seq("ucb-bar,placeboaccel", "riscv")) {
//     // Copy pasta!
//   }
//   ResourceBinding {
//     Resource(cpuDevice, "reg").bind(ResourceAddress(tileId))
//   }

//   //////////////////////////////////////////////////////////////////
//   // define a TileLink client node
//   //////////////////////////////////////////////////////////////////

//   // # of bits used in TileLink ID for master node. 
//   // 4 bits can support 16 master nodes, but you can have a longer ID if you need more.
//   val idBits = 4
//   val clientNode = TLClientNode(
//     // Copy pasta (with minor edit)!
//   )

//   //////////////////////////////////////////////////////////////////
//   // define a TileLink register node
//   //////////////////////////////////////////////////////////////////

//   val regNode = TLRegisterNode(
//     // Copy pasta (with minor edit)!
//     address = Seq(AddressSet(params.regNodeBase, REG_NODE_MASK)),
//     device = 
//       new SimpleDevice(s"${params.core.projectName}_regs",  
//       Seq(s"shadow_council,${params.core.projectName}_regs", "regs/control")),
//     beatBytes = 8,
//     concurrency = 1
//   )

//   //////////////////////////////////////////////////////////////////
//   // connect TileLink nodes
//   //////////////////////////////////////////////////////////////////

//   // Required TileLink nodes
//   val intOutwardNode = None
//   val masterNode = visibilityNode
//   val slaveNode = TLIdentityNode()

//   // "slave" (legacy term) side - add register node
//   tlSlaveXbar.node :*= regNode := slaveNode

//   // "master" (legacy term) side - add client node
//   tlOtherMastersNode := tlMasterXbar.node
//   masterNode :=* tlOtherMastersNode
//   tlMasterXbar.node  := clientNode
// }

// //////////////////////////////////////////////////////////////////
// // the Implementation Tile class
// //////////////////////////////////////////////////////////////////

// class PlaceboAccelTileImp(outer: PlaceboAccelTile) extends BaseTileModuleImp(outer) {
  
//   //////////////////////////////////////////////////////////////////
//   // instantiate BlackBox class
//   //////////////////////////////////////////////////////////////////
//   val myTotallyLegitAccel = Module(
//    // Copy pasta (with minor edit)!
//     new PlaceboAccelBlackBox(outer.params)
//   )

//   //////////////////////////////////////////////////////////////////
//   // connect IO to register node map (or other logic)
//   //////////////////////////////////////////////////////////////////

//   val io = myTotallyLegitAccel.io
  
//   // MMIO everything
//   val A_reg = Reg(UInt(32.W))
//   val B_reg = Reg(UInt(32.W))
//   val ALUop_reg = Reg(UInt(4.W))

//   myTotallyLegitAccel.io.A := A_reg
//   myTotallyLegitAccel.io.B := B_reg
//   myTotallyLegitAccel.io.ALUop := ALUop_reg

//   outer.regNode.regmap(
//      // Create your regmap!
//     0x0 -> Seq(RegField.r(32, A_reg)),
//     0x4 -> Seq(RegField.r(32, B_reg)),
//     0x8 -> Seq(RegField.r(4, ALUop_reg)),
//     0xC -> Seq(RegField.w(32, myTotallyLegitAccel.io.Out))
//    )
// }

// //////////////////////////////////////////////////////////////////
// // the BlackBox class
// //////////////////////////////////////////////////////////////////

// class PlaceboAccelBlackBox(implicit p: Parameters) 
//     extends BlackBox with HasBlackBoxResource {

//   // Define the PlaceboAccel IO
//   val io = IO(new Bundle {
//     // Define your io!
//     val A = Input(UInt(32.W))
//     val B = Input(UInt(32.W))
//     val ALUop = Input(UInt(4.W))
//     val Out = Output(UInt(32.W))
//   })
//   // Copy pasta (with minor edit)! - Hint: addResource
//   addResource(s"vsrc/SueprFastPlaceboAccelerator.v");
// }