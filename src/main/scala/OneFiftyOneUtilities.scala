package ofo

import scala.collection.mutable.LinkedHashMap

object OneFiftyOneCtrlRegs extends Enumeration {

  type Type = Value
  val CLK, RESET, MEM_REQ_VALID, MEM_REQ_READY, MEM_REQ_RW, MEM_REQ_TAG,
      MEM_REQ_ADDR, MEM_REQ_DATA_VALID, MEM_REQ_DATA_READY, MEM_REQ_DATA_BITS_MSB, MEM_REQ_DATA_BITS_LSB,
      MEM_REQ_DATA_MASK, MEM_RESP_VALID, MEM_RESP_TAG, MEM_RESP_DATA_MSB, MEM_RESP_DATA_LSB, CSR, EX =
    Value

  val REG_WIDTH = LinkedHashMap(
    CLK -> 1,
    RESET -> 1,
    MEM_REQ_VALID -> 1,
    MEM_REQ_READY -> 1,
    MEM_REQ_RW -> 1,
    MEM_REQ_DATA_VALID -> 1,
    MEM_REQ_DATA_READY -> 1,
    MEM_RESP_VALID -> 1,
    MEM_REQ_ADDR -> 28,
    MEM_REQ_TAG -> 5,
    MEM_REQ_DATA_BITS_MSB -> 64,
    MEM_REQ_DATA_BITS_LSB -> 64,
    MEM_REQ_DATA_MASK -> 128 / 8,
    MEM_RESP_TAG -> 5,
    MEM_RESP_DATA_MSB -> 64,
    MEM_RESP_DATA_LSB -> 64,
    CSR -> 32
  )
  val TOTAL_REG_WIDTH = REG_WIDTH.values.sum

  val SCAN_CHAIN_OFFSET =
    REG_WIDTH.keys.zip(REG_WIDTH.values.scanLeft(0)(_ + _).dropRight(1)).toMap

  val SCAN_OUT_OFFSET =
    REG_WIDTH.keys.zip(REG_WIDTH.values.scanRight(0)(_ + _).drop(1)).toMap

  val REGMAP_OFFSET =
    (REG_WIDTH.keys ++ Iterator(EX))
      .zip(
        REG_WIDTH.values.scanLeft(0)((acc, n) => acc + ((n - 1) / 64 + 1) * 8)
      )
      .toMap
  val MEM_REQ_DATA_BITS_WIDTH = REG_WIDTH(MEM_REQ_DATA_BITS_MSB) + REG_WIDTH(MEM_REQ_DATA_BITS_LSB)
}
