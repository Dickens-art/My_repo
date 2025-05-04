//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Functional Units
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// If regfile bypassing is disabled, then the functional unit must do its own
// bypassing in here on the WB stage (i.e., bypassing the io.resp.data)
//
// TODO: explore possibility of conditional IO fields? if a branch unit... how to add extra to IO in subclass?

package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental.chiselName

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.ALU._
import freechips.rocketchip.util._
import freechips.rocketchip.tile
import freechips.rocketchip.rocket.{PipelinedMultiplier,BP,BreakpointUnit,Causes,CSR}

import boom.common._
import boom.ifu._
import boom.util._

/**t
 * Functional unit constants
 */
object FUConstants                                  //功能单元所涉及的常数
{
  // bit mask, since a given execution pipeline may support multiple functional units
  //一个流水线执行单元可能包括多种功能单元
  val FUC_SZ = 10                                                  //10 bits 独热码，作为功能单元的功能选择信号
  val FU_X   = BitPat.dontCare(FUC_SZ)
  val FU_ALU =   1.U(FUC_SZ.W)
  val FU_JMP =   2.U(FUC_SZ.W)
  val FU_MEM =   4.U(FUC_SZ.W)
  val FU_MUL =   8.U(FUC_SZ.W)
  val FU_DIV =  16.U(FUC_SZ.W)
  val FU_CSR =  32.U(FUC_SZ.W)
  val FU_FPU =  64.U(FUC_SZ.W)
  val FU_FDV = 128.U(FUC_SZ.W)
  val FU_I2F = 256.U(FUC_SZ.W)
  val FU_F2I = 512.U(FUC_SZ.W)

  // FP stores generate data through FP F2I, and generate address through MemAddrCalc
  val FU_F2IMEM = 516.U(FUC_SZ.W)        
}
import FUConstants._

/**
 * Class to tell the FUDecoders what units it needs to support
 *
 * @param alu support alu unit?
 * @param bru support br unit?
 * @param mem support mem unit?
 * @param muld support multiple div unit?
 * @param fpu support FP unit?
 * @param csr support csr writing unit?
 * @param fdiv support FP div unit?
 * @param ifpu support int to FP unit?
 */
class SupportedFuncUnits(        //读寄存器模块的输入，即支持什么单元，需要有对应的读寄存器控制信号
  val alu: Boolean  = false,
  val jmp: Boolean  = false,
  val mem: Boolean  = false,
  val muld: Boolean = false,
  val fpu: Boolean  = false,
  val csr: Boolean  = false,
  val fdiv: Boolean = false,
  val ifpu: Boolean = false)
{
}


/**
 * Bundle for signals sent to the functional unit
 *
 * @param dataWidth width of the data sent to the functional unit
 */
class FuncUnitReq(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle                 //定义funcunitreq的数据类型（功能单元需要什么数据，即输入的一部分）
  with HasBoomUOP                                                 //with部分有uop
{
  
  val uop_securitytag = UInt(2.W)    //tag

  val numOperands = 3                                                                                                                                               //3个给定位宽的操作数，预测标志，kill标志

  val rs1_data = UInt(dataWidth.W)
  val rs2_data = UInt(dataWidth.W)
  val rs3_data = UInt(dataWidth.W) // only used for FMA units
  val pred_data = Bool()                                                                                                                                              //这是啥？这个功能是否是被预测执行的？

  val rs1_data_securitytag = UInt(2.W)
  val rs2_data_securitytag = UInt(2.W)
  val rs3_data_securitytag = UInt(2.W)

  val kill = Bool()                                                                                                                                                              // kill everything
}

/**
 * Bundle for the signals sent out of the function unit
 *
 * @param dataWidth data sent from the functional unit
 */
class FuncUnitResp(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle                  //功能单元的响应（输出）
  with HasBoomUOP
{
  val predicated = Bool() // Was this response from a predicated-off instruction                               //是否该响应来自一个错误预测的指令                   
  val data = UInt(dataWidth.W)                                                                                                                                  //数据

  val data_securitytag = UInt(2.W)                                                                                                                        //数据的securitytag

  //val  data_check_unpass = UInt(1.W)                                                                                                                 //数据安全检测结果

  val fflags = new ValidIO(new FFlagsResp)                                                                                                         //浮点单元的输出
  val addr = UInt((vaddrBits+1).W)                                                                                                                          // only for maddr -> LSU 访存地址，输出给lsu

  val addr_securitytag = UInt(2.W)                                                                                                                      //访存地址的tag（即tag[p_addr]）

  val mxcpt = new ValidIO(UInt((freechips.rocketchip.rocket.Causes.all.max+2).W))                     //only for maddr->LSU
  val sfence = Valid(new freechips.rocketchip.rocket.SFenceReq)                                                          // only for mcalc
}

/**
 * Branch resolution information given from the branch unit   来自分支单元的相关信号
 */
class BrResolutionInfo(implicit p: Parameters) extends BoomBundle          //分支预测单元给出的预测信息
{
  val uop        = new MicroOp                                                                                               //指令微码

  //val uop_securitytag = UInt(2.W)                      //tag

  val valid      = Bool()                                                                                                              //有效位
  val mispredict = Bool()                                                                                                       //预测错误位
  val taken      = Bool()                     // which direction did the branch go?           //预测
  val cfi_type   = UInt(CFI_SZ.W)                                                                                        //cfi是什么？calling frame info？

  // Info for recalculating the pc for this branch
  val pc_sel     = UInt(2.W)                                                                                                     //pc选择信号

  val jalr_target = UInt(vaddrBitsExtended.W)                                                            //跳转地址
  val target_offset = SInt()                                                                                                    //地质偏移
}

class BrUpdateInfo(implicit p: Parameters) extends BoomBundle                  //分支更新信息
{
  // On the first cycle we get masks to kill registers                     //这个mask是什么？
  //在BOOM中，每个处于推测状态的指令都有一个分支mask，该mask是独热码
  //（使用N位状态寄存器来对N个状态进行编码，每个状态都由他独立的寄存器位，并且在任意时候，其中只有一位有效），每个bit对应着一条分支。
  val b1 = new BrUpdateMasks
  // On the second cycle we get indices to reset pointers
  val b2 = new BrResolutionInfo
}
class BrUpdateMasks(implicit p: Parameters) extends BoomBundle
{
  val resolve_mask = UInt(maxBrCount.W)
  val mispredict_mask = UInt(maxBrCount.W)
}


/**
 * Abstract top level functional unit class that wraps a lower level hand made functional unit
 *
 * @param isPipelined is the functional unit pipelined?
 * @param numStages how many pipeline stages does the functional unit have
 * @param numBypassStages how many bypass stages does the function unit have
 * @param dataWidth width of the data being operated on in the functional unit
 * @param hasBranchUnit does this functional unit have a branch unit?
 */
abstract class FunctionalUnit(                                      //抽象一个功能单元模型
  val isPipelined: Boolean,                                               //是否为流水线功能单元
  val numStages: Int,                                                          //流水级数
  val numBypassStages: Int,                                            //可以bypass的级数
  val dataWidth: Int,                                 
  val isJmpUnit: Boolean = false,                                   //是否为跳转单元
  val isAluUnit: Boolean = false,                                      //是否为alu单元
  val isMemAddrCalcUnit: Boolean = false,               //是否为i内存地址计算单元
  val needsFcsr: Boolean = false)                                   //是否需要fcsr，fcsr是啥？（浮点csr）
  (implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {                                                                                                    //定义io借口
    val req    = Flipped(new DecoupledIO(new FuncUnitReq(dataWidth)))     //功能单元的输入（包含数据，数据tag，kill，pred）
    val resp   = (new DecoupledIO(new FuncUnitResp(dataWidth)))

    val data_check_unpass = Output(Bool())                                                               //data_check_unpass

    val brupdate = Input(new BrUpdateInfo())                                                             //分支更新信息

    val bypass = Output(Vec(numBypassStages, Valid(new ExeUnitResp(dataWidth))))        //输出bypass信息（包含需要bypass的全部信息）

    // only used by the fpu unit
    val fcsr_rm = if (needsFcsr) Input(UInt(tile.FPConstants.RM_SZ.W)) else null

    // only used by branch unit
    val brinfo     = if (isAluUnit) Output(new BrResolutionInfo()) else null
    val get_ftq_pc = if (isJmpUnit) Flipped(new GetPCFromFtqIO()) else null
    val status     = if (isMemAddrCalcUnit) Input(new freechips.rocketchip.rocket.MStatus()) else null

    // only used by memaddr calc unit
    val bp = if (isMemAddrCalcUnit) Input(Vec(nBreakpoints, new BP)) else null
    val mcontext = if (isMemAddrCalcUnit) Input(UInt(coreParams.mcontextWidth.W)) else null
    val scontext = if (isMemAddrCalcUnit) Input(UInt(coreParams.scontextWidth.W)) else null

  })
}

/**
 * Abstract top level pipelined functional unit
 *
 * Note: this helps track which uops get killed while in intermediate stages,
 * but it is the job of the consumer to check for kills on the same cycle as consumption!!!
 *
 * @param numStages how many pipeline stages does the functional unit have
 * @param numBypassStages how many bypass stages does the function unit have
 * @param earliestBypassStage first stage that you can start bypassing from
 * @param dataWidth width of the data being operated on in the functional unit
 * @param hasBranchUnit does this functional unit have a branch unit?
 */
abstract class PipelinedFunctionalUnit(                       //抽象一个流水线功能单元
  numStages: Int,                                                                      //流水级数
  numBypassStages: Int,                                                        //bypass级数
  earliestBypassStage: Int,                                                    //最前边的bypass级
  dataWidth: Int,                                                                        //数据宽度
  isJmpUnit: Boolean = false,                                              //是否为跳转单元
  isAluUnit: Boolean = false,                                                //是否为alu单元
  isMemAddrCalcUnit: Boolean = false,                          //是否为内存地址计算单元
  needsFcsr: Boolean = false                                               //是否需要浮点csr
  )(implicit p: Parameters) extends FunctionalUnit(
    isPipelined = true,                                                              //是流水线单元，其他全部继承
    numStages = numStages,
    numBypassStages = numBypassStages,
    dataWidth = dataWidth,
    isJmpUnit = isJmpUnit,
    isAluUnit = isAluUnit,
    isMemAddrCalcUnit = isMemAddrCalcUnit,
    needsFcsr = needsFcsr)
{
  // Pipelined functional unit is always ready.
  io.req.ready := true.B                                       //由于流水线，故一定一直ready（一直可以接受信号）

  if (numStages > 0) {                                           
    val r_valids = RegInit(VecInit(Seq.fill(numStages) { false.B }))                 //流水线寄存器，每一级de1valid信号
    val r_uops   = Reg(Vec(numStages, new MicroOp()))                                   //流水线寄存器，每一级的微码（包括securitytag）

    // handle incoming request                                                                                    //输入连接
    r_valids(0) := io.req.valid && !IsKilledByBranch(io.brupdate, io.req.bits.uop) && !io.req.bits.kill
    r_uops(0)   := io.req.bits.uop
    r_uops(0).br_mask := GetNewBrMask(io.brupdate, io.req.bits.uop)

    // handle middle of the pipeline                                                                          //流水线连接
    for (i <- 1 until numStages) {
      r_valids(i) := r_valids(i-1) && !IsKilledByBranch(io.brupdate, r_uops(i-1)) && !io.req.bits.kill
      r_uops(i)   := r_uops(i-1)
      r_uops(i).br_mask := GetNewBrMask(io.brupdate, r_uops(i-1))

      if (numBypassStages > 0) {
        io.bypass(i-1).bits.uop := r_uops(i-1)
      }
    }

    // handle outgoing (branch could still kill it)                                             //输出连接
    // consumer must also check for pipeline flushes (kills)
    io.resp.valid    := r_valids(numStages-1) && !IsKilledByBranch(io.brupdate, r_uops(numStages-1))
    io.resp.bits.predicated := false.B
    io.resp.bits.uop := r_uops(numStages-1)
    io.resp.bits.uop.br_mask := GetNewBrMask(io.brupdate, r_uops(numStages-1))

    // bypassing (TODO allow bypass vector to have a different size from numStages) bypass应该包含需要bypass的全部信息
    if (numBypassStages > 0 && earliestBypassStage == 0) {
      io.bypass(0).bits.uop := io.req.bits.uop

      for (i <- 1 until numBypassStages) {
        io.bypass(i).bits.uop := r_uops(i-1)
      }
    }
  } else {
    require (numStages == 0)
    // pass req straight through to response

    // valid doesn't check kill signals, let consumer deal with it.
    // The LSU already handles it and this hurts critical path.
    io.resp.valid    := io.req.valid && !IsKilledByBranch(io.brupdate, io.req.bits.uop)
    io.resp.bits.predicated := false.B
    io.resp.bits.uop := io.req.bits.uop
    io.resp.bits.uop.br_mask := GetNewBrMask(io.brupdate, io.req.bits.uop)
  }
}

/**
 * Functional unit that wraps RocketChips ALU
 *
 * @param isBranchUnit is this a branch unit?
 * @param numStages how many pipeline stages does the functional unit have
 * @param dataWidth width of the data being operated on in the functional unit
 */

  // alu单元（继承流水线单元）

@chiselName
class ALUUnit(isJmpUnit: Boolean = false, numStages: Int = 1, dataWidth: Int)(implicit p: Parameters)        
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = numStages,
    isAluUnit = true,                                   //是alu
    earliestBypassStage = 0,                  //第一级就能bypass
    dataWidth = dataWidth,
    isJmpUnit = isJmpUnit)
  with boom.ifu.HasBoomFrontendParameters
{
  val uop = io.req.bits.uop

  val uop_securitytag = io.req.bits.uop_securitytag                 //指令tag

  dontTouch(uop_securitytag)

  // immediate generation
  val imm_xprlen = ImmGen(uop.imm_packed, uop.ctrl.imm_sel)        //生成 20 bits imme，其tag为pc_tag

  val imm_xprlen_securitytag = uop_securitytag


  // operand 1 select               op1和op1的securitutag的选择
  var op1_data: UInt = null

  var op1_data_securitytag: UInt = null

  if (isJmpUnit) {                      //如果是跳转指令，(即这个alu被例化为jmp_unit)
    // Get the uop PC for jumps
    val block_pc = AlignPCToBoundary(io.get_ftq_pc.pc, icBlockBytes)  //实现按照icblockbytes = 64对齐，即屏蔽pc低6位
    val uop_pc = (block_pc | uop.pc_lob) - Mux(uop.edge_inst, 2.U, 0.U)  //当前指令pc（前边或完是完整的pc）

    val uop_pc_securitytag = uop_securitytag                                                   //当前指令的pc_securitytag

    op1_data = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , io.req.bits.rs1_data,     //op1选择是rs1还是pc还是0
               Mux(uop.ctrl.op1_sel.asUInt === OP1_PC  , Sext(uop_pc, xLen),
                                                         0.U))

    op1_data_securitytag = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , io.req.bits.rs1_data_securitytag,     //op1_securitytag选择是rs1还是pc
               Mux(uop.ctrl.op1_sel.asUInt === OP1_PC  , uop_securitytag,
                                                        uop_securitytag))

  } else {                                    //不是跳转指令，只需要在rs1和0之间选择即可
    op1_data = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , io.req.bits.rs1_data,
                                                         0.U)

    op1_data_securitytag = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , io.req.bits.rs1_data_securitytag,
                                                     uop_securitytag)

  }


  

  // operand 2 select                      op2选择，imme、immc、rs2、2/4、0
  val op2_data = Mux(uop.ctrl.op2_sel === OP2_IMM,  Sext(imm_xprlen.asUInt, xLen),
                 Mux(uop.ctrl.op2_sel === OP2_IMMC, io.req.bits.uop.prs1(4,0),
                 Mux(uop.ctrl.op2_sel === OP2_RS2 , io.req.bits.rs2_data,
                 Mux(uop.ctrl.op2_sel === OP2_NEXT, Mux(uop.is_rvc, 2.U, 4.U),
                                                    0.U))))

  val op2_data_securitytag = Mux(uop.ctrl.op2_sel === OP2_IMM, uop_securitytag,
                 Mux(uop.ctrl.op2_sel === OP2_IMMC, uop_securitytag,                                                            //csr的立即数？
                 Mux(uop.ctrl.op2_sel === OP2_RS2 , io.req.bits.rs2_data_securitytag,                              //读寄存器得到的
                 Mux(uop.ctrl.op2_sel === OP2_NEXT, Mux(uop.is_rvc, 0.U(2.W) , 0.U(2.W)),                     //JAL/JALR_tag 
                                                    uop_securitytag))))

 //val op1_data_securitytag_in = Mux((op1_data === 0.U(2.W)) & (op1_data_securitytag === 3.U(2.W)) , 0.U(2.W) , op1_data_securitytag)   
  //tag[zero]  = uop_tag（但是这里只能检查操作数是不是0，可能会有漏洞）,该功能实现在了read_reg部分，并消除了漏洞
 
 //val op2_data_securitytag_in = Mux((op2_data === 0.U(2.W)) & (op2_data_securitytag === 3.U(2.W)) , 0.U(2.W) , op2_data_securitytag)   
                            
  val data_check_unpass   =  ( ((~(uop_securitytag(1))) & ( op1_data_securitytag(1)))   |
                                                           ((~(uop_securitytag (1))) & (~(uop_securitytag(0)))&(op1_data_securitytag(0)))  |
                                                           ((~(uop_securitytag (0))) &  op1_data_securitytag(1))&(op1_data_securitytag(0))   )         |         //安全检查,uop_securitytag  >= op1_securitytag
                                                           ( ((~(uop_securitytag(1))) & ( op2_data_securitytag(1)))   |
                                                           ((~(uop_securitytag (1))) & (~(uop_securitytag(0)))&(op2_data_securitytag(0)))  |
                                                           ((~(uop_securitytag (0))) &  op2_data_securitytag(1))&(op2_data_securitytag(0))   )             //安全检查,uop_securitytag  >= op2_securitytag

  dontTouch(data_check_unpass)                         //会出现在使用到alu的exu里边，例如（alu_exe_unit、csr_exe_unit、jmp_unit）

  val alu = Module(new freechips.rocketchip.rocket.ALU())          //alu模块例化

  alu.io.in1 := op1_data.asUInt
  alu.io.in2 := op2_data.asUInt

  alu.io.in1_securitytag := op1_data_securitytag                  //传入合适的securitytag
  alu.io.in2_securitytag := op2_data_securitytag

  alu.io.fn  := uop.ctrl.op_fcn                                                                     //alu类型
  alu.io.dw  := uop.ctrl.fcn_dw                                                                  //数据为64/32


  // Did I just get killed by the previous cycle's branch,
  // or by a flush pipeline?
  val killed = WireInit(false.B)
  when (io.req.bits.kill || IsKilledByBranch(io.brupdate, uop)) {
    killed := true.B
  }

  val rs1 = io.req.bits.rs1_data
  val rs2 = io.req.bits.rs2_data

  val rs1_securitytag = io.req.bits.rs1_data_securitytag
  val rs2_securitytag = io.req.bits.rs2_data_securitytag

  val br_eq  = (rs1 === rs2)
  val br_ltu = (rs1.asUInt < rs2.asUInt)
  val br_lt  = (~(rs1(xLen-1) ^ rs2(xLen-1)) & br_ltu |
                rs1(xLen-1) & ~rs2(xLen-1)).asBool

 //pc_securitytag不需要改变，这是因为需要使用当前的pc_securitytag去检查isnt_n的securitytag，安全检查通过后
 //更新pc_securitytag（即pc的securitytag在前端取指令时候更新的，后续都是传递）

  val pc_sel = MuxLookup(uop.ctrl.br_type, PC_PLUS4,              //pc选选择信号生成，根据输入的br_type，生成pc选择信号
                 Seq(   BR_N   -> PC_PLUS4,
                        BR_NE  -> Mux(!br_eq,  PC_BRJMP, PC_PLUS4),
                        BR_EQ  -> Mux( br_eq,  PC_BRJMP, PC_PLUS4),
                        BR_GE  -> Mux(!br_lt,  PC_BRJMP, PC_PLUS4),
                        BR_GEU -> Mux(!br_ltu, PC_BRJMP, PC_PLUS4),
                        BR_LT  -> Mux( br_lt,  PC_BRJMP, PC_PLUS4),
                        BR_LTU -> Mux( br_ltu, PC_BRJMP, PC_PLUS4),
                        BR_J   -> PC_BRJMP,
                        BR_JR  -> PC_JALR                                                               
                        ))

  val is_taken = io.req.valid &&                                                  //确认预测正确
                   !killed &&
                   (uop.is_br || uop.is_jalr || uop.is_jal) &&
                   (pc_sel =/= PC_PLUS4)

  // "mispredict" means that a branch has been resolved and it must be killed
  val mispredict = WireInit(false.B)                                       //预测错误

  val is_br          = io.req.valid && !killed && uop.is_br && !uop.is_sfb         //分支类型
  val is_jal         = io.req.valid && !killed && uop.is_jal
  val is_jalr        = io.req.valid && !killed && uop.is_jalr

  when (is_br || is_jalr) {                                            //pc_sel信号的断言和补充
    if (!isJmpUnit) {
      assert (pc_sel =/= PC_JALR)
    }
    when (pc_sel === PC_PLUS4) {
      mispredict := uop.taken
    }
    when (pc_sel === PC_BRJMP) {
      mispredict := !uop.taken
    }
  }

  val brinfo = Wire(new BrResolutionInfo)                       

  // note: jal doesn't allocate a branch-mask, so don't clear a br-mask bit   
  brinfo.valid          := is_br || is_jalr
  brinfo.mispredict     := mispredict
  brinfo.uop            := uop
  brinfo.cfi_type       := Mux(is_jalr, CFI_JALR,
                           Mux(is_br  , CFI_BR, CFI_X))
  brinfo.taken          := is_taken
  brinfo.pc_sel         := pc_sel

  brinfo.jalr_target    := DontCare


  // Branch/Jump Target Calculation
  // For jumps we read the FTQ, and can calculate the target
  // For branches we emit the offset for the core to redirect if necessary
  val target_offset = imm_xprlen(20,0).asSInt
  brinfo.jalr_target := DontCare
  if (isJmpUnit) {
    def encodeVirtualAddress(a0: UInt, ea: UInt) = if (vaddrBitsExtended == vaddrBits) {
      ea
    } else {
      // Efficient means to compress 64-bit VA into vaddrBits+1 bits.
      // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1)).
      val a = a0.asSInt >> vaddrBits
      val msb = Mux(a === 0.S || a === -1.S, ea(vaddrBits), !ea(vaddrBits-1))
      Cat(msb, ea(vaddrBits-1,0))
    }


    val jalr_target_base = io.req.bits.rs1_data.asSInt                               //返回目标计算
    val jalr_target_xlen = Wire(UInt(xLen.W))
    jalr_target_xlen := (jalr_target_base + target_offset).asUInt
    val jalr_target = (encodeVirtualAddress(jalr_target_xlen, jalr_target_xlen).asSInt & -2.S).asUInt

    brinfo.jalr_target := jalr_target
    val cfi_idx = ((uop.pc_lob ^ Mux(io.get_ftq_pc.entry.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U)))(log2Ceil(fetchWidth),1)

    when (pc_sel === PC_JALR) {
      mispredict := !io.get_ftq_pc.next_val ||
                    (io.get_ftq_pc.next_pc =/= jalr_target) ||
                    !io.get_ftq_pc.entry.cfi_idx.valid ||
                    (io.get_ftq_pc.entry.cfi_idx.bits =/= cfi_idx)
    }
  }

  brinfo.target_offset := target_offset


  io.brinfo := brinfo



 // Response
 //功能单元响应
 // TODO add clock gate on resp bits from functional units
 //   io.resp.bits.data := RegEnable(alu.io.out, io.req.valid)
 //   val reg_data = Reg(outType = Bits(width = xLen))
 //   reg_data := alu.io.out
 //   io.resp.bits.data := reg_data

  val r_val  = RegInit(VecInit(Seq.fill(numStages) { false.B }))                                             //流水线寄存器的val信号
  val r_data = Reg(Vec(numStages, UInt(xLen.W)))                                                                 //流水线寄存器的data

  val r_data_securitytag = Reg(Vec(numStages, UInt(2.W)))                                            //流水线寄存器的securitytag

  val r_data_check_unpass =  Reg(Vec(numStages, Bool()))                                                //data_check_result

  val r_pred = Reg(Vec(numStages, Bool()))                                                                              //流水线寄存器的预测信号
  val alu_out = Mux(io.req.bits.uop.is_sfb_shadow && io.req.bits.pred_data,           //是否为被预测的
    Mux(io.req.bits.uop.ldst_is_rs1, io.req.bits.rs1_data, io.req.bits.rs2_data),          //如果预测错误，选择rs1，否则选择rs2
    Mux(io.req.bits.uop.uopc === uopMOV, io.req.bits.rs2_data, alu.io.out))               //

 val alu_out_securitytag = Mux(io.req.bits.uop.is_sfb_shadow && io.req.bits.pred_data,           
    Mux(io.req.bits.uop.ldst_is_rs1, io.req.bits.rs1_data_securitytag, io.req.bits.rs2_data_securitytag),          
    Mux(io.req.bits.uop.uopc === uopMOV, io.req.bits.rs2_data_securitytag, alu.io.out_securitytag))

  r_val (0) := io.req.valid                                                                                                                      //第0级流水线寄存器的输入
  r_data(0) := Mux(io.req.bits.uop.is_sfb_br, pc_sel === PC_BRJMP, alu_out)           //这是干啥的

  r_data_securitytag(0) :=  Mux(io.req.bits.uop.is_sfb_br,  uop_securitytag , alu_out_securitytag)       

  r_data_check_unpass(0)  :=  data_check_unpass                                                                   //data_check_unpass

  r_pred(0) := io.req.bits.uop.is_sfb_shadow && io.req.bits.pred_data
  for (i <- 1 until numStages) {                                                                                                         //流水线寄存器的段间连接
    r_val(i)  := r_val(i-1)
    r_data(i) := r_data(i-1)

    r_data_securitytag(i) := r_data_securitytag(i-1)                                                                 //tag

    r_data_check_unpass(i) := r_data_check_unpass(i-1)                                                    //data_check_unpass

    r_pred(i) := r_pred(i-1)
  }
  io.resp.bits.data := r_data(numStages-1)

  io.resp.bits.data_securitytag := r_data_securitytag(numStages-1)

  io.data_check_unpass := r_data_check_unpass(numStages-1)                                    //输出的data_check_unpass

  io.resp.bits.predicated := r_pred(numStages-1)
  // Bypass
  // for the ALU, we can bypass same cycle as compute 
  require (numStages >= 1)
  require (numBypassStages >= 1)                //只有满足这些才能bypass
  io.bypass(0).valid := io.req.valid
  io.bypass(0).bits.data := Mux(io.req.bits.uop.is_sfb_br, pc_sel === PC_BRJMP, alu_out)

  io.bypass(0).bits.data_securitytag := Mux(io.req.bits.uop.is_sfb_br, uop_securitytag, alu_out_securitytag)       //同上的疑问

  for (i <- 1 until numStages) {
    io.bypass(i).valid := r_val(i-1)
    io.bypass(i).bits.data := r_data(i-1)

   io.bypass(i).bits.data_securitytag := r_data_securitytag(i-1)

  }

  // Exceptions
  io.resp.bits.fflags.valid := false.B
}

/**
 * Functional unit that passes in base+imm to calculate addresses, and passes store data
 * to the LSU.
 * For floating point, 65bit FP store-data needs to be decoded into 64bit FP form
 */
class MemAddrCalcUnit(implicit p: Parameters)                                                   //访存地址计算单元,已经加入addr_securitytag的计算
  extends PipelinedFunctionalUnit(
    numStages = 0,
    numBypassStages = 0,
    earliestBypassStage = 0,                        
    dataWidth = 65, // TODO enable this only if FP is enabled?
    isMemAddrCalcUnit = true)
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
  with freechips.rocketchip.rocket.constants.ScalarOpConstants
{
  // perform address calculation
  val sum = (io.req.bits.rs1_data.asSInt + io.req.bits.uop.imm_packed(19,8).asSInt).asUInt

  val sum_securitytag = min_2 (io.req.bits.rs1_data_securitytag , io.req.bits.uop.imm_packed_securitytag)   
 //计算出来的地址的securitytag，其中立即数的securitytag其实就是当前执行的包含imm的指令的securitytag，连接见decode
 //计算出来的地址最后生成为p_addr

  val ea_sign = Mux(sum(vaddrBits-1), ~sum(63,vaddrBits) === 0.U,
                                       sum(63,vaddrBits) =/= 0.U)
  val effective_address = Cat(ea_sign, sum(vaddrBits-1,0)).asUInt

  val effective_address_securitytag = sum_securitytag

  val store_data = io.req.bits.rs2_data                                                               //存的数据

  val store_data_securitytag = io.req.bits.rs2_data_securitytag        //存的数据的securitytag

  io.resp.bits.addr := effective_address                                                             //访存地址  （p_addr）

  io.resp.bits.addr_securitytag :=  effective_address_securitytag                                //生成的访存地址的securitytag

  io.resp.bits.data := store_data

  io.resp.bits.data_securitytag := store_data_securitytag

  if (dataWidth > 63) {                                                                                   //如果存储浮点数据，tag不支持
    assert (!(io.req.valid && io.req.bits.uop.ctrl.is_std &&
      io.resp.bits.data(64).asBool === true.B), "65th bit set in MemAddrCalcUnit.")

    assert (!(io.req.valid && io.req.bits.uop.ctrl.is_std && io.req.bits.uop.fp_val),
      "FP store-data should now be going through a different unit.")
  }

  assert (!(io.req.bits.uop.fp_val && io.req.valid && io.req.bits.uop.uopc =/=    //是不是可以用assert的方式来抛出异常？
          uopLD && io.req.bits.uop.uopc =/= uopSTA),
          "[maddrcalc] assert we never get store data in here.")

  // Handle misaligned exceptions       未对齐
  val size = io.req.bits.uop.mem_size
  val misaligned =
    (size === 1.U && (effective_address(0) =/= 0.U)) ||        //16 bit 对齐
    (size === 2.U && (effective_address(1,0) =/= 0.U)) ||    //32
    (size === 3.U && (effective_address(2,0) =/= 0.U))       //64

  val bkptu = Module(new BreakpointUnit(nBreakpoints))           //例化一个断点单元，保存信息
  bkptu.io.status   := io.status
  bkptu.io.bp       := io.bp
  bkptu.io.pc       := DontCare
  bkptu.io.ea       := effective_address                                                      //断点地址（所存数据的地址）

  //bkptu.io.ea_securitytag       := effective_address_securitytag          //暂时不管

  bkptu.io.mcontext := io.mcontext
  bkptu.io.scontext := io.scontext

  val ma_ld  = io.req.valid && io.req.bits.uop.uopc === uopLD && misaligned
  val ma_st  = io.req.valid && (io.req.bits.uop.uopc === uopSTA || io.req.bits.uop.uopc === uopAMO_AG) && misaligned
  val dbg_bp = io.req.valid && ((io.req.bits.uop.uopc === uopLD  && bkptu.io.debug_ld) ||
                                (io.req.bits.uop.uopc === uopSTA && bkptu.io.debug_st))
  val bp     = io.req.valid && ((io.req.bits.uop.uopc === uopLD  && bkptu.io.xcpt_ld) ||
                                (io.req.bits.uop.uopc === uopSTA && bkptu.io.xcpt_st))

  def checkExceptions(x: Seq[(Bool, UInt)]) =                        
    (x.map(_._1).reduce(_||_), PriorityMux(x))
  val (xcpt_val, xcpt_cause) = checkExceptions(List(
    (ma_ld,  (Causes.misaligned_load).U),
    (ma_st,  (Causes.misaligned_store).U),
    (dbg_bp, (CSR.debugTriggerCause).U),
    (bp,     (Causes.breakpoint).U)))

  io.resp.bits.mxcpt.valid := xcpt_val
  io.resp.bits.mxcpt.bits  := xcpt_cause
  assert (!(ma_ld && ma_st), "Mutually-exclusive exceptions are firing.")

  io.resp.bits.sfence.valid := io.req.valid && io.req.bits.uop.mem_cmd === M_SFENCE
  io.resp.bits.sfence.bits.rs1 := io.req.bits.uop.mem_size(0)                //rs1寄存器号

  //io.resp.bits.sfence.bits.rs1_securitytag    

  io.resp.bits.sfence.bits.rs2 := io.req.bits.uop.mem_size(1)               

  //同上

  io.resp.bits.sfence.bits.addr := io.req.bits.rs1_data

  //io.resp.bits.sfence.bits.addr_securitytag := io.req.bits.rs1_data_securitytag                //tag

  io.resp.bits.sfence.bits.asid := io.req.bits.rs2_data

  //io.resp.bits.sfence.bits.asid_securitytag := io.req.bits.rs2_data_securitytag

}


/**
 * Functional unit to wrap lower level FPU
 *fpu，tag不支持
 * Currently, bypassing is unsupported!
 * All FP instructions are padded out to the max latency unit for easy
 * write-port scheduling.
 */
class FPUUnit(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = p(tile.TileKey).core.fpu.get.dfmaLatency,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = 65,
    needsFcsr = true)
{
  val fpu = Module(new FPU())
  fpu.io.req.valid         := io.req.valid
  fpu.io.req.bits.uop      := io.req.bits.uop
  fpu.io.req.bits.rs1_data := io.req.bits.rs1_data
  fpu.io.req.bits.rs2_data := io.req.bits.rs2_data
  fpu.io.req.bits.rs3_data := io.req.bits.rs3_data



  fpu.io.req.bits.fcsr_rm  := io.fcsr_rm

  io.resp.bits.data              := fpu.io.resp.bits.data
  io.resp.bits.fflags.valid      := fpu.io.resp.bits.fflags.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  io.resp.bits.fflags.bits.flags := fpu.io.resp.bits.fflags.bits.flags // kill me now
}

/**
 * Int to FP conversion functional unit
 *
 * @param latency the amount of stages to delay by
 */
class IntToFPUnit(latency: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = latency,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = 65,
    needsFcsr = true)
  with tile.HasFPUParameters
{
  val fp_decoder = Module(new UOPCodeFPUDecoder) // TODO use a simpler decoder
  val io_req = io.req.bits
  fp_decoder.io.uopc := io_req.uop.uopc
  val fp_ctrl = fp_decoder.io.sigs
  val fp_rm = Mux(ImmGenRm(io_req.uop.imm_packed) === 7.U, io.fcsr_rm, ImmGenRm(io_req.uop.imm_packed))
  val req = Wire(new tile.FPInput)
  val tag = fp_ctrl.typeTagIn

  req <> fp_ctrl

  req.rm := fp_rm
  req.in1 := unbox(io_req.rs1_data, tag, None)
  req.in2 := unbox(io_req.rs2_data, tag, None)
  req.in3 := DontCare
  req.typ := ImmGenTyp(io_req.uop.imm_packed)
  req.fmt := DontCare // FIXME: this may not be the right thing to do here
  req.fmaCmd := DontCare

  assert (!(io.req.valid && fp_ctrl.fromint && req.in1(xLen).asBool),
    "[func] IntToFP integer input has 65th high-order bit set!")

  assert (!(io.req.valid && !fp_ctrl.fromint),
    "[func] Only support fromInt micro-ops.")

  val ifpu = Module(new tile.IntToFP(intToFpLatency))
  ifpu.io.in.valid := io.req.valid
  ifpu.io.in.bits := req
  ifpu.io.in.bits.in1 := io_req.rs1_data
  val out_double = Pipe(io.req.valid, fp_ctrl.typeTagOut === D, intToFpLatency).bits

//io.resp.bits.data              := box(ifpu.io.out.bits.data, !io.resp.bits.uop.fp_single)
  io.resp.bits.data              := box(ifpu.io.out.bits.data, out_double)
  io.resp.bits.fflags.valid      := ifpu.io.out.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  io.resp.bits.fflags.bits.flags := ifpu.io.out.bits.exc
}

/**
 * Iterative/unpipelined functional unit, can only hold a single MicroOp at a time
 * assumes at least one register between request and response
 *
 * 非流水线单元，同一时间整个单元正能执行一个指令（只能接受一个指令的微码）
 * 
 * TODO allow up to N micro-ops simultaneously.
 *
 * @param dataWidth width of the data to be passed into the functional unit
 */
abstract class IterativeFunctionalUnit(dataWidth: Int)(implicit p: Parameters)
  extends FunctionalUnit(
    isPipelined = false,                       //非流水线单元
    numStages = 1,
    numBypassStages = 0,
    dataWidth = dataWidth)
{
  val r_uop = Reg(new MicroOp())

  val r_uop_securitytag = Reg(UInt(2.W))                  //tag

  val do_kill = Wire(Bool())
  do_kill := io.req.bits.kill // irrelevant default

  when (io.req.fire) {
    // update incoming uop
    do_kill := IsKilledByBranch(io.brupdate, io.req.bits.uop) || io.req.bits.kill
    r_uop := io.req.bits.uop

   r_uop_securitytag := io.req.bits.uop_securitytag                               //tag

    r_uop.br_mask := GetNewBrMask(io.brupdate, io.req.bits.uop)
  } .otherwise {
    do_kill := IsKilledByBranch(io.brupdate, r_uop) || io.req.bits.kill
    r_uop.br_mask := GetNewBrMask(io.brupdate, r_uop)
  }

  // assumes at least one pipeline register between request and response
  io.resp.bits.uop := r_uop

  //io.resp.bits.uop_securitytag := r_uop_securitytag                               //tag，好像没什么必要输出这个

}

/**
 * Divide functional unit.        除法单元
 * tag不支持
 * @param dataWidth data to be passed into the functional unit
 */
class DivUnit(dataWidth: Int)(implicit p: Parameters)
  extends IterativeFunctionalUnit(dataWidth)
{

  // We don't use the iterative multiply functionality here.
  // Instead we use the PipelinedMultiplier
  val div = Module(new freechips.rocketchip.rocket.MulDiv(mulDivParams, width = dataWidth))

  // request
  div.io.req.valid    := io.req.valid && !this.do_kill
  div.io.req.bits.dw  := io.req.bits.uop.ctrl.fcn_dw
  div.io.req.bits.fn  := io.req.bits.uop.ctrl.op_fcn
  div.io.req.bits.in1 := io.req.bits.rs1_data
  div.io.req.bits.in2 := io.req.bits.rs2_data
  div.io.req.bits.tag := DontCare
  io.req.ready        := div.io.req.ready
  //***********************************************输入接口传递**********************************************************
  div.io.req.bits.in1_security_tag := io.req.bits.rs1_data_securitytag
  div.io.req.bits.in2_security_tag := io.req.bits.rs2_data_securitytag
  div.io.req.bits.uop_security_tag  := io.req.bits.uop_securitytag      //   bits æ¯ä»ä¹ ï¼
  //*************************************************输入接口传递**********************************************************

  // handle pipeline kills and branch misspeculations
  div.io.kill         := this.do_kill

  // response
  io.resp.valid       := div.io.resp.valid && !this.do_kill
  div.io.resp.ready   := io.resp.ready
  io.resp.bits.data   := div.io.resp.bits.data
    //***********************************************输出接口传递**********************************************************
  io.resp.bits.data_securitytag := div.io.resp.bits.out_security_tag
  io.data_check_unpass          := div.io.resp.bits.data_check_unpass
//*************************************************输出接口传递**********************************************************
}

/**
 * Pipelined multiplier functional unit that wraps around the RocketChip pipelined multiplier
 *
 * 流水线乘法单元
 * tag 不支持
 * @param numStages number of pipeline stages
 * @param dataWidth size of the data being passed into the functional unit
 */
class PipelinedMulUnit(numStages: Int, dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth)
{
  val imul = Module(new PipelinedMultiplier(xLen, numStages))
  // request
  imul.io.req.valid    := io.req.valid
  imul.io.req.bits.fn  := io.req.bits.uop.ctrl.op_fcn
  imul.io.req.bits.dw  := io.req.bits.uop.ctrl.fcn_dw
  imul.io.req.bits.in1 := io.req.bits.rs1_data
  imul.io.req.bits.in2 := io.req.bits.rs2_data
  imul.io.req.bits.tag := DontCare
  //***********************************************输入接口传递**********************************************************
  imul.io.req.bits.in1_security_tag := io.req.bits.rs1_data_securitytag
  imul.io.req.bits.in2_security_tag := io.req.bits.rs2_data_securitytag
  imul.io.req.bits.uop_security_tag  := io.req.bits.uop_securitytag
//***********************************************输入接口传递**********************************************************
  // response
  io.resp.bits.data    := imul.io.resp.bits.data
  //***********************************************输出接口传递**********************************************************
  io.resp.bits.data_securitytag := imul.io.resp.bits.out_security_tag
  io.data_check_unpass              := imul.io.resp.bits.data_check_unpass
//************************************************输出接口传递**********************************************************
}
