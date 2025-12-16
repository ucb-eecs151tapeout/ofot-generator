// package ofo

// // LJ's NOTE:
// // we will not complete this Tile 100% and you will likely need to comment or hide the file later to avoid compilation issues

// import chisel3._
// import chisel3.util._
// // Some more imports here

// //////////////////////////////////////////////////////////////////
// // the Config class
// //////////////////////////////////////////////////////////////////

// class WithPlaceboAccel(core: PlaceboAccelCoreParams) extends Config((site, here, up) => {
//   case TilesLocated(InSubsystem) => {
//     val prev = up(TilesLocated(InSubsystem))
//     val idOffset = up(NumTiles)
    
//     // Copy pasta (with minor edit)!
//     Seq(PlaceboAccelTileAttachParams(
//         tileParams = PlaceboAccelTileParams(
//             core = core
//             hartId = idOffset,
//             name = "NameOfPlaceboAccelTile"
//         ),
//         crossingParams = RocketCrossingParams()
//     )) ++ prev
//   }
//   case NumTiles => up(NumTiles) + 1
// })

// //////////////////////////////////////////////////////////////////
// // the CanAttachTile class
// //////////////////////////////////////////////////////////////////


// case class PlaceboAccelTileAttachParams(
//   tileParams: PlaceboAccelTileParams,
// //   crossingParams: RocketCrossingParams
//   crossingParams: freechips.rocketchip.subsystem.HierarchicalElementCrossingParamsLike  // The given OFO uses this for some reason 
// ) extends CanAttachTile {
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
//   val core: CoreParams                  // Core parameters (see below)
// //   val icache: Option[ICacheParams]      // Rocket specific: I1 cache option
// //   val dcache: Option[DCacheParams]      // Rocket specific: D1 cache option
// //   val btb: Option[BTBParams]            // Rocket specific: BTB / branch predictor option
//   val hartId: Int                       // Hart ID: Must be unique within a design config (This MUST be a case class parameter)
// //   val beuAddr: Option[BigInt]           // Rocket specific: Bus Error Unit for Rocket Core
// //   val blockerCtrlAddr: Option[BigInt]   // Rocket specific: Bus Blocker for Rocket Core
//   val name: Option[String]              // Name of the core
//   regNodeBase: BigInt = 0x4000,
// ) extends InstantiableTileParams[PlaceboAccelTile] {
//   def instantiate(
//       crossing: freechips.rocketchip.subsystem.HierarchicalElementCrossingParamsLike,
//       lookup: LookupByHartIdImpl
//   )(implicit p: Parameters): PlaceboAccelTile = {
//     new PlaceboAccelTile(this, crossing, lookup, p)
//   }
// }

// //////////////////////////////////////////////////////////////////
// // the actual Tile class
// //////////////////////////////////////////////////////////////////

// class PlaceboAccelTile(
//     val params: PlaceboAccelTileParams, 
//     crossing: freechips.rocketchip.subsystem.HierarchicalElementCrossingParamsLike,
//     lookup: LookupByHartIdImpl,
//     q: Parameters
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
//     address = Seq(AddressSet(params.regNodeBase, 4096-1)),
//     device = cpuDevice,
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
//     new PlaceboAccelBlackBox(outer.params.core)
//   )

//   //////////////////////////////////////////////////////////////////
//   // connect IO to register node map (or other logic)
//   //////////////////////////////////////////////////////////////////

//   val io = myTotallyLegitAccel.io
  
//   // MMIO everything
//   val A_reg = Reg(UInt(32.W))
//   val B_reg = Reg(UInt(32.W))
//   val ALUop_reg = Reg(UInt(4.W))
//   val Out_reg = Reg(UInt(32.W)) // is this needed???? - LJ

//   myTotallyLegitAccel.io.A := A_reg
//   myTotallyLegitAccel.io.B := B_reg
//   myTotallyLegitAccel.io.ALUop := ALUop_reg
//   myTotallyLegitAccel.io.Out := Out_reg // this too? - LJ

//   // input [31:0] A,B,
//   // input [3:0] ALUop,
//   // output reg [31:0] Out

//   outer.regNode.regmap(
//      {0x00} -> RegField.w(32, A_reg)
//      {0x04} -> RegField.w(32, B_reg)
//      {0x08} -> RegField.w(4, ALUop_reg)
//      {0x0C} -> RegField.r(32, Out_reg) // this way or below? - LJ
//      {0x0C} -> RegField.r(32, myTotallyLegitAccel.io.Out)
//    )
// }

// //////////////////////////////////////////////////////////////////
// // the BlackBox class
// //////////////////////////////////////////////////////////////////

// class PlaceboAccelBlackBox(implicit p: Parameters) 
//     extends BlackBox with HasBlackBoxResource {

//     // module SuperFastPlaceboAccelerator(
//     //     input [31:0] A,B,
//     //     input [3:0] ALUop,
//     //     output reg [31:0] Out
//     // );

//     // Define the PlaceboAccel IO
//     val io = IO(new Bundle {
//         val A = Input(UInt(32.W))
//         val B = Input(UInt(32.W))
//         val ALUop = Input(UInt(4.W))
//         val Out = Output(UInt(32.W))
//     })
    
//     // Copy pasta (with minor edit)! - Hint: addResource
//     addResource("/vsrc/SuperFastPlaceboAccelerator.v")
// }