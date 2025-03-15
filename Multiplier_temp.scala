// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import chisel3._
import chisel3.util.{Cat, log2Up, log2Ceil, log2Floor, Log2, Decoupled, Enum, Fill, Valid, Pipe}
import Chisel.ImplicitConversions._
import freechips.rocketchip.util._
import ALU._

// 单元的输入接口
class MultiplierReq(dataBits: Int, tagBits: Int) extends Bundle {
  val fn = Bits(SZ_ALU_FN.W)
  val dw = Bits(SZ_DW.W)
  val in1 = Bits(dataBits.W)
  val in2 = Bits(dataBits.W)
  val tag = UInt(tagBits.W)
//***********************************************在单元的输入接口处添加安全标签信号**********************************************************
  val in1_secutrity_tag = UInt(2.W)
  val in2_secutrity_tag = UInt(2.W)
//***********************************************在单元的输入接口处添加安全标签信号*********************************************************
}

// 单元的输出接口
class MultiplierResp(dataBits: Int, tagBits: Int) extends Bundle {
  val data = Bits(dataBits.W)
  val tag = UInt(tagBits.W)
//***********************************************在单元的输出接口处添加安全标签信号**********************************************************
  val out_secutrity_tag = UInt(2.W)
//***********************************************在单元的输出接口处添加安全标签信号********************************************************
}

// 单元的IO接口定义:引用MultiplierResp和MultiplierReq两个类 
class MultiplierIO(val dataBits: Int, val tagBits: Int) extends Bundle {
  val req = Flipped(Decoupled(new MultiplierReq(dataBits, tagBits)))
  val kill = Input(Bool())
  val resp = Decoupled(new MultiplierResp(dataBits, tagBits))
}

// 单元的参数定义
case class MulDivParams(
  mulUnroll: Int = 1,
  divUnroll: Int = 1,
  mulEarlyOut: Boolean = false,
  divEarlyOut: Boolean = false,
  divEarlyOutGranularity: Int = 1
)

class MulDiv(cfg: MulDivParams, width: Int, nXpr: Int = 32) extends Module {
  private def minDivLatency = (cfg.divUnroll > 0).option(if (cfg.divEarlyOut) 3 else 1 + w/cfg.divUnroll)
  private def minMulLatency = (cfg.mulUnroll > 0).option(if (cfg.mulEarlyOut) 2 else w/cfg.mulUnroll)
  def minLatency: Int = (minDivLatency ++ minMulLatency).min

  val io = IO(new MultiplierIO(width, log2Up(nXpr)))
  val w = io.req.bits.in1.getWidth        // w为scala数据类型（UInt），表示操作数的位宽，非硬件资源
  val mulw = if (cfg.mulUnroll == 0) w else (w + cfg.mulUnroll - 1) / cfg.mulUnroll * cfg.mulUnroll        // mulw同上，非硬件资源
  val fastMulW = if (cfg.mulUnroll == 0) false else w/2 > cfg.mulUnroll && w % (2*cfg.mulUnroll) == 0      // fastMulW为scala数据类型（Boolean），非硬件资源
 
  val s_ready :: s_neg_inputs :: s_mul :: s_div :: s_dummy :: s_neg_output :: s_done_mul :: s_done_div :: Nil = Enum(8)       // 状态机状态定义
  val state = RegInit(s_ready)      // 状态机状态寄存器，状态初始化为s_ready
 
  val req = Reg(chiselTypeOf(io.req.bits))        // 输入寄存器，存储输入数据
  val count = Reg(UInt(log2Ceil(
    ((cfg.divUnroll != 0).option(w/cfg.divUnroll + 1).toSeq ++
     (cfg.mulUnroll != 0).option(mulw/cfg.mulUnroll)).reduce(_ max _)).W))        // 寄存器类型，位宽由以上定义的常量决定（取最大值）
  val neg_out = Reg(Bool())       // 寄存器类型，
  val isHi = Reg(Bool())          // 寄存器类型，
  val resHi = Reg(Bool())         // 寄存器类型，
  val divisor = Reg(Bits((w+1).W))      // 寄存器类型，位宽为（w+1）位？  div only needs w bits
  val remainder = Reg(Bits((2*mulw+2).W))         // 寄存器类型，位宽为（2*mulw+2）位？  div only needs 2*w+1 bits

  val mulDecode = List(
    FN_MUL    -> List(Y, N, X, X),
    FN_MULH   -> List(Y, Y, Y, Y),
    FN_MULHU  -> List(Y, Y, N, N),
    FN_MULHSU -> List(Y, Y, Y, N))
  val divDecode = List(
    FN_DIV    -> List(N, N, Y, Y),
    FN_REM    -> List(N, Y, Y, Y),
    FN_DIVU   -> List(N, N, N, N),
    FN_REMU   -> List(N, Y, N, N))
// DecodeLogic函数的作用是根据输入的fn值（操作码，对应上述表格中的左半部分），确定对应的操作类型，返回一个List列表赋值于cmdMul :: cmdHi :: lhsSigned :: rhsSigned等四个布尔型硬件
  val cmdMul :: cmdHi :: lhsSigned :: rhsSigned :: Nil =
    DecodeLogic(io.req.bits.fn, List(X, X, X, X),
      (if (cfg.divUnroll != 0) divDecode else Nil) ++ (if (cfg.mulUnroll != 0) mulDecode else Nil)).map(_.asBool)


  require(w == 32 || w == 64)       // 断言函数，判断w是否等于32或64，否则报错
  def halfWidth(req: MultiplierReq) = (w > 32).B && req.dw === DW_32        // 定义函数，判断该数据是否为半宽数据

  def sext(x: Bits, halfW: Bool, signed: Bool) = {
    val sign = signed && Mux(halfW, x(w/2-1), x(w-1))
    val hi = Mux(halfW, Fill(w/2, sign), x(w-1,w/2))
    (Cat(hi, x(w/2-1,0)), sign)                            
  }       // halfW = true.B , 则返回原值和符号位   ；  halfW = false.B , 将原数据的低w/2位为有效数据，将其拓展为w位，高w/2位均为原值的符号位        
  val (lhs_in, lhs_sign) = sext(io.req.bits.in1, halfWidth(io.req.bits), lhsSigned)
  val (rhs_in, rhs_sign) = sext(io.req.bits.in2, halfWidth(io.req.bits), rhsSigned)
  
  // 下面三行纯硬件构造，不涉及数据，那构造的变量全部为reg类型？？？
  val subtractor = remainder(2*w,w) - divisor       // 减法器，remainder为被减数，divisor为除数，subtractor为remainder（2w，w）减去divisor的结果
  val result = Mux(resHi, remainder(2*w, w+1), remainder(w-1, 0))       // result为remainder的高w位或低w位，取决于resHi的值
  val negated_remainder = -result       // negated_remainder为result的负数

  //状态：s_neg_inputs，完成负数的处理和状态的转换，下一个状态进入s_div
  if (cfg.divUnroll != 0) when (state === s_neg_inputs) {
    when (remainder(w-1)) {
      remainder := negated_remainder
    }
    when (divisor(w-1)) {
      divisor := subtractor
    }
    state := s_div
  }       

  //状态：s_neg_output，该状态下处理remainder，并将resHi信号拉高，下一个状态进入s_done_div
  if (cfg.divUnroll != 0) when (state === s_neg_output) {
    remainder := negated_remainder
    state := s_done_div
    resHi := false
  }

  //状态：s_mul，完成乘法运算，下一个状态进入s_done_mul
  if (cfg.mulUnroll != 0) when (state === s_mul) {
    val mulReg = Cat(remainder(2*mulw+1,w+1),remainder(w-1,0))
    val mplierSign = remainder(w)
    val mplier = mulReg(mulw-1,0)
    val accum = mulReg(2*mulw,mulw).asSInt
    val mpcand = divisor.asSInt
    val prod = Cat(mplierSign, mplier(cfg.mulUnroll-1, 0)).asSInt * mpcand + accum
    val nextMulReg = Cat(prod, mplier(mulw-1, cfg.mulUnroll))
    val nextMplierSign = count === mulw/cfg.mulUnroll-2 && neg_out        //定义硬件，寄存器类型

    val eOutMask = ((BigInt(-1) << mulw).S >> (count * cfg.mulUnroll)(log2Up(mulw)-1,0))(mulw-1,0)
    val eOut = (cfg.mulEarlyOut).B && count =/= mulw/cfg.mulUnroll-1 && count =/= 0 &&
      !isHi && (mplier & ~eOutMask) === 0.U
    val eOutRes = (mulReg >> (mulw - count * cfg.mulUnroll)(log2Up(mulw)-1,0))
    val nextMulReg1 = Cat(nextMulReg(2*mulw,mulw), Mux(eOut, eOutRes, nextMulReg)(mulw-1,0))
    remainder := Cat(nextMulReg1 >> w, nextMplierSign, nextMulReg1(w-1,0))

    count := count + 1        //该状态下计数器 + 1
    when (eOut || count === mulw/cfg.mulUnroll-1) {
      state := s_done_mul
      resHi := isHi
    }
  }

  //状态：s_div，完成除法运算，下一个状态进入s_done_div
  if (cfg.divUnroll != 0) when (state === s_div) {
    val unrolls = ((0 until cfg.divUnroll) scanLeft remainder) { case (rem, i) =>
      // the special case for iteration 0 is to save HW, not for correctness
      val difference = if (i == 0) subtractor else rem(2*w,w) - divisor(w-1,0)
      val less = difference(w)
      Cat(Mux(less, rem(2*w-1,w), difference(w-1,0)), rem(w-1,0), !less)
    } tail

    remainder := unrolls.last
    when (count === w/cfg.divUnroll) {
      state := Mux(neg_out, s_neg_output, s_done_div)
      resHi := isHi
      if (w % cfg.divUnroll < cfg.divUnroll - 1)
        remainder := unrolls(w % cfg.divUnroll)
    }
    count := count + 1

    val divby0 = count === 0 && !subtractor(w)
    if (cfg.divEarlyOut) {
      val align = 1 << log2Floor(cfg.divUnroll max cfg.divEarlyOutGranularity)
      val alignMask = ~((align-1).U(log2Ceil(w).W))
      val divisorMSB = Log2(divisor(w-1,0), w) & alignMask
      val dividendMSB = Log2(remainder(w-1,0), w) | ~alignMask
      val eOutPos = ~(dividendMSB - divisorMSB)
      val eOut = count === 0 && !divby0 && eOutPos >= align
      when (eOut) {
        remainder := remainder(w-1,0) << eOutPos
        count := eOutPos >> log2Floor(cfg.divUnroll)
      }
    }
    when (divby0 && !isHi) { neg_out := false }
  }

  //状态：s_done_mul，完成乘法运算，下一个状态进入s_ready
  when (io.resp.fire() || io.kill) {
    state := s_ready
  }

  
  when (io.req.fire()) {
    state := Mux(cmdMul, s_mul, Mux(lhs_sign || rhs_sign, s_neg_inputs, s_div))
    isHi := cmdHi
    resHi := false
    count := (if (fastMulW) Mux[UInt](cmdMul && halfWidth(io.req.bits), w/cfg.mulUnroll/2, 0) else 0)
    neg_out := Mux(cmdHi, lhs_sign, lhs_sign =/= rhs_sign)
    divisor := Cat(rhs_sign, rhs_in)
    remainder := lhs_in
    req := io.req.bits
  }

  val outMul = (state & (s_done_mul ^ s_done_div)) === (s_done_mul & ~s_done_div)
  val loOut = Mux(fastMulW.B && halfWidth(req) && outMul, result(w-1,w/2), result(w/2-1,0))
  val hiOut = Mux(halfWidth(req), Fill(w/2, loOut(w/2-1)), result(w-1,w/2))
  io.resp.bits.tag := req.tag

  io.resp.bits.data := Cat(hiOut, loOut)
  io.resp.valid := (state === s_done_mul || state === s_done_div)
//***********************************************输出安全标签的结果**********************************************************
  val min_security_tag = Mux (  (req.in1_secutrity_tag < req.in2_secutrity_tag), 
                                  req.in1_secutrity_tag, req.in2_secutrity_tag)
  io.resp.bits.out_secutrity_tag := Mux(state === s_done_mul || state === s_done_div, min_security_tag, 0.U(2.W))
//***********************************************输出安全标签的结果**********************************************************
  io.req.ready := state === s_ready
}

class PipelinedMultiplier(width: Int, latency: Int, nXpr: Int = 32) extends Module with ShouldBeRetimed {
  val io = IO(new Bundle {
    val req = Flipped(Valid(new MultiplierReq(width, log2Ceil(nXpr))))
    val resp = Valid(new MultiplierResp(width, log2Ceil(nXpr)))
  })

  val in = Pipe(io.req)

  val decode = List(
    FN_MUL    -> List(N, X, X),
    FN_MULH   -> List(Y, Y, Y),
    FN_MULHU  -> List(Y, N, N),
    FN_MULHSU -> List(Y, Y, N))
  val cmdHi :: lhsSigned :: rhsSigned :: Nil =
    DecodeLogic(in.bits.fn, List(X, X, X), decode).map(_.asBool)
  val cmdHalf = (width > 32).B && in.bits.dw === DW_32

  val lhs = Cat(lhsSigned && in.bits.in1(width-1), in.bits.in1).asSInt
  val rhs = Cat(rhsSigned && in.bits.in2(width-1), in.bits.in2).asSInt
  val prod = lhs * rhs
  val muxed = Mux(cmdHi, prod(2*width-1, width), Mux(cmdHalf, prod(width/2-1, 0).sextTo(width), prod(width-1, 0)))
//***********************************************计算最小标签***************************************************************
  val min_security_tag = Mux (  (in.bits.in1_secutrity_tag < in.bits.in2_secutrity_tag), 
                                  in.bits.in1_secutrity_tag, in.bits.in2_secutrity_tag)
//***********************************************计算最小标签***************************************************************
  val resp = Pipe(in, latency-1)
  io.resp.valid := resp.valid
  io.resp.bits.tag := resp.bits.tag
  io.resp.bits.data := Pipe(in.valid, muxed, latency-1).bits
//***********************************************输出安全标签的结果**********************************************************
  io.resp.bits.out_secutrity_tag := Pipe(in.valid, min_security_tag, latency-1).bits
//***********************************************输出安全标签的结果**********************************************************
}
