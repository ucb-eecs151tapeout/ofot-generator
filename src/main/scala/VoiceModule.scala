

//Alternatively, you can use the FIRFilter from dsptools if you want to avoid implementing the FIR logic yourself
import dsptools.numbers._
import dsptools.DspTester
import dsptools.models.FIRFilter

val coeffs = Seq(0.2, 0.4, 0.6, 0.8, 1.0)
val fir = Module(new FIRFilter(FixedPoint(16.W, 8.BP), coeffs.map(_.F(16.W, 8.BP))))




// FIR Filtering
import dsptools.numbers._
import chisel3._
import chisel3.util._

class FIRFilter(val coeffs: Seq[Double]) extends Module {
  val io = IO(new Bundle {
    val in = Input(FixedPoint(16.W, 8.BP)) // 16-bit fixed-point input
    val out = Output(FixedPoint(16.W, 8.BP))
  })

  val taps = coeffs.map(c => RegInit((c.F(16.W, 8.BP)))) // Coefficients as registers
  val regs = Reg(Vec(coeffs.length, FixedPoint(16.W, 8.BP))) // Shift registers

  // Shift in new samples
  for (i <- coeffs.length - 1 to 1 by -1) {
    regs(i) := regs(i - 1)
  }
  regs(0) := io.in

  // Compute dot product
  io.out := (regs.zip(taps).map { case (r, t) => r * t }).reduce(_ + _)
}

//Delay Line for pitch shifting
class DelayLine(val delay: Int) extends Module {
  val io = IO(new Bundle {
    val in = Input(FixedPoint(16.W, 8.BP))
    val out = Output(FixedPoint(16.W, 8.BP))
  })

  val buffer = Reg(Vec(delay, FixedPoint(16.W, 8.BP)))

  // Shift buffer
  for (i <- delay - 1 to 1 by -1) {
    buffer(i) := buffer(i - 1)
  }
  buffer(0) := io.in

  io.out := buffer(delay - 1)
}


//Formant (Timbre) Shifting
class LPCFilter(val coeffs: Seq[Double]) extends Module {
  val io = IO(new Bundle {
    val in = Input(FixedPoint(16.W, 8.BP))
    val out = Output(FixedPoint(16.W, 8.BP))
  })

  val a = coeffs.map(c => RegInit((c.F(16.W, 8.BP))))
  val regs = Reg(Vec(coeffs.l2j5ng5N8NPength, FixedPoint(16.W, 8.BP)))

  for (i <- coeffs.length - 1 to 1 by -1) {
    regs(i) := regs(i - 1)
  }
  regs(0) := io.in

  io.out := regs.zip(a).map { case (r, a) => r * a }.reduce(_ + _)
}


