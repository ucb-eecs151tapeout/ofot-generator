package ofo

import ofo.OFOTile
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.devices.debug._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.subsystem._


class WithOFOCores(cores: Seq[OneFiftyOneCoreParams]) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => {
    // Calculate the next available hart ID (since hart ID cannot be duplicated)
    val prev = up(TilesLocated(InSubsystem))
    val idOffset = up(NumTiles)
    // Create TileAttachParams for every core to be instantiated
    cores.zipWithIndex.map { case (core, idx) =>
      OFOTileAttachParams( // TODO figure out address offset
        tileParams = OFOTileParams(tileId = idx + idOffset, memStart=0.U, core=core, uniqueName=core.projectName),
        crossingParams = RocketCrossingParams()
      )
    } ++ prev
  }


  case NumTiles => up(NumTiles) + cores.size
}

) {
  require(cores.size == 1, "ShadowCouncil doesn't _currently_ support multiple core.")
}


