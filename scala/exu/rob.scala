//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Re-order Buffer
//重命名buffer
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Bank the ROB, such that each "dispatch" group gets its own row of the ROB,
// and each instruction in the dispatch group goes to a different bank.
// We can compress out the PC by only saving the high-order bits!
//
// ASSUMPTIONS:
//    - dispatch groups are aligned to the PC.
//
// NOTES:
//    - Currently we do not compress out bubbles in the ROB.
//    - Exceptions are only taken when at the head of the commit bundle --
//      this helps deal with loads, stores, and refetch instructions.

package boom.exu

import scala.math.ceil

import chisel3._
import chisel3.util._
import chisel3.experimental.chiselName

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.Str

import boom.common._
import boom.util._
import freechips.rocketchip.subsystem.CrossesToOnlyOneResetDomain

/**
 * IO bundle to interact with the ROB
 * 
 * 定义rob的io
 *
 * @param numWakeupPorts number of wakeup ports to the rob         
 * @param numFpuPorts number of fpu ports that will write back fflags
 */
class RobIo(
  val numWakeupPorts: Int,        //bank数量？
  val numFpuPorts: Int
  )(implicit p: Parameters)  extends BoomBundle
{


  //为方便超标量dispatch和commit，ROB由具有corewidth个bank的循环缓冲区实现，每个bank的同一层组成一个rob行，最多包括corewidth条指令

  // Decode Stage 解码级
  // (Allocate, write instruction to ROB).
  val enq_valids       = Input(Vec(coreWidth, Bool()))
  val enq_uops         = Input(Vec(coreWidth, new MicroOp()))          //指令信息，连接dispatch级，一次并行输入多条指令
  
  val enq_uops_securitytag  = Input(Vec(coreWidth,UInt(2.W)))            //指令的securitytag, 用以进行指令安全检查

  val enq_partial_stall= Input(Bool()) // we're dispatching only a partial packet,
                                       // and stalling on the rest of it (don't
                                       // advance the tail ptr)
                                       //只发送一部分数据包，而非全部

  val xcpt_fetch_pc = Input(UInt(vaddrBitsExtended.W))                       //该rob行的0号bank指令pc，可以据此计算出该rob行其他指令pc

  //val xcpt_fetch_pc_securitytag = Input(UInt(2.W))                                  
  //这个地方应该是这个fetch packet的tag（即这个rob行的tag），
  //如果修改了预取器，那么应该只需要一个就能表示packet里所有tag，甚至只需要1位
  //先通过enq_uops_securitytag传进来再归一吧           

  //在dispatch时，多达w条指令从Fetch Packet写入ROB行，每条指令被写入该行的不同bank。
  //由于Fetch Packet中的指令在内存中连续（且对齐），这允许整个Fetch Packet由一个PC表示
  //（并且指令在Fetch Packet中的位置向它们自身的PC提供低位）

  //robaddrsz为rob的所能容纳的指令数量，其值为 bank数*每个bank的行数（64）的 二进制位数

  val rob_tail_idx = Output(UInt(robAddrSz.W))             //rob的尾index（派遣的指令信息被写入，最晚的指令）
  val rob_pnr_idx  = Output(UInt(robAddrSz.W))           


//PNR(Point of No Return)的指针，它指向着位于最前面的有可能是错误预测，或有可能发生异常的指令。 
//意味着在PNR前面的指令一定是安全的，不会发生回滚的。

  val rob_head_idx = Output(UInt(robAddrSz.W))         //rob的头index（交付，最早的指令）

  // Handle Branch Misspeculations
  val brupdate = Input(new BrUpdateInfo())                    //分支信息

  // Write-back Stage    写回级
  // (Update of ROB)     更新rob
  // Instruction is no longer busy and can be committed
  val wb_resps = Flipped(Vec(numWakeupPorts, Valid(new ExeUnitResp(xLen max fLen+1))))             
  //exu写回的resps，用以告知指令执行情况

  // Unbusying ports for stores.
  //来自lsu的清除busy和unsafe的resp
  // +1 for fpstdata
  val lsu_clr_bsy      = Input(Vec(memWidth + 1, Valid(UInt(robAddrSz.W))))     

  // Port for unmarking loads/stores as speculation hazards..
  val lsu_clr_unsafe   = Input(Vec(memWidth, Valid(UInt(robAddrSz.W))))

  //val lsu_data_check_unpass  = Input(Vec(memWidth,Bool()))


  // Track side-effects for debug purposes.
  // Also need to know when loads write back, whereas we don't need loads to unbusy.
  val debug_wb_valids = Input(Vec(numWakeupPorts, Bool()))
  val debug_wb_wdata  = Input(Vec(numWakeupPorts, Bits(xLen.W)))                              //alu传入的wdata

  val debug_wb_wdata_securitytag  = Input(Vec(numWakeupPorts, Bits(2.W)))  
  val data_check_unpasses = Input(Vec(numWakeupPorts,Bool() ))                                    //alu传入的data_check_unpass

  val fflags = Flipped(Vec(numFpuPorts, new ValidIO(new FFlagsResp())))                        //浮点异常
  val lxcpt = Flipped(new ValidIO(new Exception()))                                                                     // LSU异常

  //val lxcpt_data_check_unpass = Input(Bool())                                                                                                 //lsu_data_check

  // Commit stage (free resources; also used for rollback).   提交级
  val commit = Output(new CommitSignals())

  // tell the LSU that the head of the ROB is a load
  // (some loads can only execute once they are at the head of the ROB).

  //告知lsu当前rob头部指令为load指令（有些load只有在头部才能执行（比如需要exu运算地址））

  val com_load_is_at_rob_head = Output(Bool())

  // Communicate exceptions to the CSRFile 和csr的交互
  val com_xcpt = Valid(new CommitExceptionSignals())       //提交的异常

  // Let the CSRFile stall us (e.g., wfi).
  val csr_stall = Input(Bool())

  // Flush signals (including exceptions, pipeline replays, and memory ordering failures)
  // to send to the frontend for redirection.
  //冲刷信号，给前端以重定向pc
  val flush = Valid(new CommitExceptionSignals)

  // Stall Decode as appropriate
  //暂停解码（握手信号）
  val empty = Output(Bool())
  val ready = Output(Bool()) // ROB is busy unrolling rename state...

  // Stall the frontend if we know we will redirect the PC
  //前端冲刷信号（为什么有两个？）
  val flush_frontend = Output(Bool())


  val debug_tsrc = Input(UInt(xLen.W))
}

/**
 * Bundle to send commit signals across processor
 * 
 * 提交的信号
 * retirewidth为可同时退休指令的数量
 */
class CommitSignals(implicit p: Parameters) extends BoomBundle
{
  val valids      = Vec(retireWidth, Bool()) // These instructions may not correspond to an architecturally executed insn
  val arch_valids = Vec(retireWidth, Bool())       //可以修改架构级
  val uops        = Vec(retireWidth, new MicroOp())    //提交指令的信息

  val uops_securitytag        = Vec(retireWidth, UInt(2.W))      //提交指令的tag（应该没啥用）

  val fflags      = Valid(UInt(5.W))                    //5 bits 浮点异常标志

  // These come a cycle later
  val debug_insts = Vec(retireWidth, UInt(32.W))

  val debug_insts_securitytag = Vec(retireWidth, UInt(2.W))

  // Perform rollback of rename state (in conjuction with commit.uops).
  //重命名状态的回滚
  val rbk_valids = Vec(retireWidth, Bool())
  val rollback   = Bool()

  val debug_wdata = Vec(retireWidth, UInt(xLen.W))

  val debug_wdata_securitytag = Vec(retireWidth, UInt(2.W))

}

/**
 * Bundle to communicate exceptions to CSRFile
 *和csr的异常交互
 *提交异常信号，只用于记录最早的异常
 * TODO combine FlushSignals and ExceptionSignals (currently timed to different cycles).
 */
class CommitExceptionSignals(implicit p: Parameters) extends BoomBundle
{
  val ftq_idx    = UInt(log2Ceil(ftqSz).W)                              //在fetch packet里的位置
  val edge_inst  = Bool()
  val is_rvc     = Bool()
  val pc_lob     = UInt(log2Ceil(icBlockBytes).W)               //pc低位
  val cause      = UInt(xLen.W)                                                     //异常信息

  val data_check_unpass = Bool()                                          //提交的两个检查
  val inst_check_unpass =Bool()       

  val badvaddr   = UInt(xLen.W)                                               //异常发生的packet pc

  val badvaddr_securitytag = UInt(2.W)                              //异常指令tag

// The ROB needs to tell the FTQ if there's a pipeline flush (and what type)
// so the FTQ can drive the frontend with the correct redirected PC.
  val flush_typ  = FlushTypes()                                                 //流水线冲刷类型
}

/**
 * Tell the frontend the type of flush so it can set up the next PC properly.
 * 
 * 和前端的交互，告诉前端冲刷类型，以便前端可以更新 pc
 * 
 */
object FlushTypes   
{
  def SZ = 3
  def apply() = UInt(SZ.W)
  def none = 0.U
  def xcpt = 1.U            // An exception occurred.
  def eret = (2+1).U    // Execute an environment return instruction.
  def refetch = 2.U      // Flush and refetch the head instruction.
  def next = 4.U           // Flush and fetch the next instruction.

//通过输入的typ判断操作类型

  def useCsrEvec(typ: UInt): Bool = typ(0)                        // typ的最低位为1，即typ === xcpt.U || typ === eret.U（001或011），tag异常应该都需要csr处理
  def useSamePC(typ: UInt): Bool = typ === refetch
  def usePCplus4(typ: UInt): Bool = typ === next

//type生成，指令和数据安全检测不通过应该是xcpt类型

  def getType(valid: Bool, i_xcpt: Bool, i_eret: Bool, i_refetch: Bool): UInt = {
    val ret =
      Mux(!valid, none,
      Mux(i_eret, eret,
      Mux(i_xcpt, xcpt,
      Mux(i_refetch, refetch,
        next))))
    ret
  }
}

/**
 * Bundle of signals indicating that an exception occurred
 * 异常
 */
class Exception(implicit p: Parameters) extends BoomBundle              
{
  val uop = new MicroOp()                   //异常指令信息，里边包含了pc_lob等

  //val uop_securitytag = UInt(2.W)   //异常指令的securitytag

  val cause = Bits(log2Ceil(freechips.rocketchip.rocket.Causes.all.max+2).W)             //异常信息
  val badvaddr = UInt(coreMaxAddrBits.W)          //异常地址，packet的

  val badvaddr_securitytag = UInt(2.W)                //异常地址的securitytag（其实就是本packet的securitytag），和上边那个保留一个就行

}

/**
 * Bundle for debug ROB signals
 * These should not be synthesized!
 * 
 * 好像只是作测试用的
 * 
 */
class DebugRobSignals(implicit p: Parameters) extends BoomBundle
{
  val state = UInt()
  val rob_head = UInt(robAddrSz.W)
  val rob_pnr = UInt(robAddrSz.W)
  val xcpt_val = Bool()
  val xcpt_uop = new MicroOp()

  //val xcpt_uop_securitytag = UInt(2.W)

  val xcpt_badvaddr = UInt(xLen.W)
}

/**
 * Reorder Buffer to keep track of dependencies and inflight instructions
 *
 * ROB
 * 
 * @param numWakeupPorts number of wakeup ports to the ROB
 * @param numFpuPorts number of FPU units that will write back fflags
 */
@chiselName
class Rob(
  val numWakeupPorts: Int,
  val numFpuPorts: Int
  )(implicit p: Parameters) extends BoomModule
{
  val io = IO(new RobIo(numWakeupPorts, numFpuPorts))

  // ROB Finite State Machine      rob的有限状态机，rollback为rob反转
  val s_reset :: s_normal :: s_rollback :: s_wait_till_empty :: Nil = Enum(4)
  val rob_state = RegInit(s_reset)

  //commit entries at the head, and unwind exceptions from the tail        
  //从头部提交指令，   从尾部展开异常，展开是什么意思
  val rob_head     = RegInit(0.U(log2Ceil(numRobRows).W))              
  //rob_head的高位，下边那个是lsb（最低有效位），用以在超标量状态下精确区分指令？
  val rob_head_lsb = RegInit(0.U((1 max log2Ceil(coreWidth)).W)) // TODO: Accurately track head LSB (currently always 0)
  val rob_head_idx = if (coreWidth == 1) rob_head else Cat(rob_head, rob_head_lsb)       //真正的rob_head指针

  val rob_tail     = RegInit(0.U(log2Ceil(numRobRows).W))
  val rob_tail_lsb = RegInit(0.U((1 max log2Ceil(coreWidth)).W))
  val rob_tail_idx = if (coreWidth == 1) rob_tail else Cat(rob_tail, rob_tail_lsb)                     //同上

  val rob_pnr      = RegInit(0.U(log2Ceil(numRobRows).W))                 //pnr指向最早的异常
  val rob_pnr_lsb  = RegInit(0.U((1 max log2Ceil(coreWidth)).W))
  val rob_pnr_idx  = if (coreWidth == 1) rob_pnr  else Cat(rob_pnr , rob_pnr_lsb)               //同上

  val com_idx = Mux(rob_state === s_rollback, rob_tail, rob_head)      //提交索引（根据rob处在正序/倒序，来决定是头指针还是尾指针）

  val rob_tail_securitytag = RegInit(0.U(8.W))                                                  //即设计中的ins_n_all

  val rob_tail_pctag = RegInit(0.U(2.W))                                                        //pc_tag

  val rob_com_securitytag = RegInit(0.U(8.W))                                                  //即设计中的ins_n_all

  val rob_row_securitytag       = RegInit(VecInit(Seq.fill(numRobRows)(0.U(8.W))))         //all_tag

  val rob_commit_pctag = RegInit(0.U(2.W))                                              //pc_tag

  val rob_inst0_tag = WireDefault(0.U(2.W))
  val rob_inst1_tag = WireDefault(0.U(2.W))
  val rob_inst2_tag = WireDefault(0.U(2.W))
  val rob_inst3_tag = WireDefault(0.U(2.W))

  val rob_inst_tag_max01 = WireDefault(0.U(2.W))
  val rob_inst_tag_max23 = WireDefault(0.U(2.W))

  val rob_com_inst0_tag = WireDefault(0.U(2.W))
  val rob_com_inst1_tag = WireDefault(0.U(2.W))
  val rob_com_inst2_tag = WireDefault(0.U(2.W))
  val rob_com_inst3_tag = WireDefault(0.U(2.W))

  val rob_com_inst_tag_max01 = WireDefault(0.U(2.W))
  val rob_com_inst_tag_max23 = WireDefault(0.U(2.W))

  //val rob_tail_securitytag_bool = Wire(Vec(8,Bool()))                                           //tag_bool


  val maybe_full   = RegInit(false.B)             //rob容量指示
  val full         = Wire(Bool())
  val empty        = Wire(Bool())

  //指令信息


  val will_commit         = Wire(Vec(coreWidth, Bool()))                      //头部指令确定提交（已确定无异常）
  val can_commit          = Wire(Vec(coreWidth, Bool()))                    //头部指令可以提交（即计算完毕，但是不确定有无异常）

  //val data_check_unpass    =  Wire(Vec(coreWidth, Bool()))          //data check位

  val can_throw_exception = Wire(Vec(coreWidth, Bool()))           //可以抛出异常

  val rob_pnr_unsafe      = Wire(Vec(coreWidth, Bool())) // are the instructions at the pnr unsafe?，pnr指的指令是否安全
  val rob_head_vals       = Wire(Vec(coreWidth, Bool())) // are the instructions at the head valid?       head的指令是否有效
  val rob_tail_vals       = Wire(Vec(coreWidth, Bool())) // are the instructions at the tail valid? (to track partial row dispatches)
  val rob_head_uses_stq   = Wire(Vec(coreWidth, Bool()))     //head指令是否使用store queue
  val rob_head_uses_ldq   = Wire(Vec(coreWidth, Bool()))    //head指令是否使用load queue   这两个是判断是否为ls指令
  val rob_head_fflags     = Wire(Vec(coreWidth, UInt(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W)))        //浮点标记

  val exception_thrown = Wire(Bool())             //异常抛出

  // exception info   异常信息
  // TODO compress xcpt cause size. Most bits in the middle are zero.
  val r_xcpt_val       = RegInit(false.B)
  val r_xcpt_uop       = Reg(new MicroOp())

  //val r_xcpt_uop_securitytag       = Reg(UInt(2.W))         //异常的tag

  val r_xcpt_badvaddr  = Reg(UInt(coreMaxAddrBits.W))

  io.flush_frontend := r_xcpt_val            //出现异常直接冲刷前端

  //--------------------------------------------------
  // Utility
  //行索引为高位，bank索引为低位
  //二者一起才能索引到单个指令

  def GetRowIdx(rob_idx: UInt): UInt = {                          //行索引函数，当超标量时，索引右移对应位（即一行共用一个索引）
    if (coreWidth == 1) return rob_idx
    else return rob_idx >> log2Ceil(coreWidth).U
  }
  def GetBankIdx(rob_idx: UInt): UInt = {                        //bank索引函数，超标量时，返回行内偏移量
    if(coreWidth == 1) { return 0.U }
    else           { return rob_idx(log2Ceil(coreWidth)-1, 0).asUInt }
  }

  // **************************************************************************
  // Debug

  class DebugRobBundle extends BoomBundle
  {
    val valid      = Bool()
    val busy       = Bool()
    val unsafe     = Bool()
    val uop        = new MicroOp()

    val uop_securitytag        =  UInt(2.W)

    val exception  = Bool()
  }
  val debug_entry = Wire(Vec(numRobEntries, new DebugRobBundle))
  debug_entry := DontCare // override in statements below

  // **************************************************************************
  // --------------------------------------------------------------------------
  // **************************************************************************

  // Contains all information the PNR needs to find the oldest instruction which can't be safely speculated past.
  //包含PNR需要查找的无法安全推测的最旧指令所需的所有信息。
  val rob_unsafe_masked = WireInit(VecInit(Seq.fill(numRobRows << log2Ceil(coreWidth)){false.B}))
  //rob行数左移对应并行宽度位，即可以代表rob内所有指令（rob内每个指令位置对应一位）

  // Used for trace port, for debug purposes only
  val rob_debug_inst_mem   = SyncReadMem(numRobRows, Vec(coreWidth, UInt(32.W)))

  val rob_debug_inst_mem_securitytag   = SyncReadMem(numRobRows, Vec(coreWidth, UInt(2.W)))

  val rob_debug_inst_wmask = WireInit(VecInit(0.U(coreWidth.W).asBools))
  val rob_debug_inst_wdata = Wire(Vec(coreWidth, UInt(32.W)))

  val rob_debug_inst_wdata_securitytag = Wire(Vec(coreWidth, UInt(2.W)))

  rob_debug_inst_mem.write(rob_tail, rob_debug_inst_wdata, rob_debug_inst_wmask)

  rob_debug_inst_mem_securitytag.write(rob_tail, rob_debug_inst_wdata_securitytag, rob_debug_inst_wmask)                     
  //这部分好像是记录和mem传递的数据用于debug？

  val rob_debug_inst_rdata = rob_debug_inst_mem.read(rob_head, will_commit.reduce(_||_)) 

  val rob_debug_inst_rdata_securitytag = rob_debug_inst_mem_securitytag.read(rob_head, will_commit.reduce(_||_))           //tag


  for (w <- 0 until coreWidth) {                  //对于每个bank（竖着看）
    def MatchBank(bank_idx: UInt): Bool = (bank_idx === w.U)      //bank匹配函数（即确定bank_idx是不是对应的这个bank）

    // one bank，每个bank
    val rob_val       = RegInit(VecInit(Seq.fill(numRobRows){false.B}))   //该行内部的信息是否有效，每个bank的每一行对应1位
    val rob_bsy       = Reg(Vec(numRobRows, Bool()))                                   //是否执行完毕，每个bank的每一行对应一位
    val rob_unsafe    = Reg(Vec(numRobRows, Bool()))                               //指示是否确定安全（非预测/异常），同上
    val rob_uop       = Reg(Vec(numRobRows, new MicroOp()))                //同上

    val rob_uop_securitytag       = RegInit(VecInit(Seq.fill(numRobRows)(0.U(2.W))))   //如果使用统一的rob_tail_securitytag的话，不需要这个

    val rob_exception = Reg(Vec(numRobRows, Bool()))                            //是否有异常

    val rob_inst_check_unpass = Reg(Vec(numRobRows, Bool()))          //指令tag安全检查,在指令进入rob时即设置

    val rob_inst_check_unpass_commit = WireDefault(VecInit(Seq.fill(numRobRows){false.B}))               //   WireDefault(VecInit(Seq.fill(numRobRows){false.B}))

    val rob_data_check_unpass = Reg(Vec(numRobRows, Bool()))        //数据tag安全检查，依据exu的resp设置

    val rob_predicated = Reg(Vec(numRobRows, Bool())) // Was this instruction predicated out?   是否是被预测
    val rob_fflags    = Mem(numRobRows, Bits(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W))   //浮点？

    val rob_debug_wdata = Mem(numRobRows, UInt(xLen.W))            //写回的数据

    val rob_debug_wdata_securitytag = Mem(numRobRows, UInt(2.W))   //写回数据的tag


    //-----------------------------------------------
    // Dispatch: Add Entry to ROB
    //派遣（向rob中添加行）,主要就是连线

    rob_debug_inst_wmask(w) := io.enq_valids(w)                                                                //decode对rob的写入（即派遣）mask
    rob_debug_inst_wdata(w) := io.enq_uops(w).debug_inst                                           //写入的内容

    rob_debug_inst_wdata_securitytag(w) := io.enq_uops_securitytag(w)                 //写入的指令tag

    when (io.enq_valids(w)) {                                                            //从尾部添加指令
      rob_val(rob_tail)       := true.B                                                  //标记该行有效
      rob_bsy(rob_tail)       := !(io.enq_uops(w).is_fence ||     //busy位设置，只要不是fence类指令，统统先设置为 未完成执行状态
                                   io.enq_uops(w).is_fencei)

      rob_data_check_unpass(rob_tail)          :=    false.B          //初始设为通
      rob_inst_check_unpass(rob_tail)            :=    ((!rob_tail_pctag(1))&&(!rob_tail_pctag(0))&&(io.enq_uops_securitytag(w)(1)))           || 
                                                       ((!rob_tail_pctag(0))&&(io.enq_uops_securitytag(w)(0))&&(io.enq_uops_securitytag(w)(1))) ||
                                                       ((!rob_tail_pctag(1))&&(rob_tail_pctag(0))&&(!io.enq_uops_securitytag(w)(1))&&(!io.enq_uops_securitytag(w)(0)))
                                                                                                        //直接进行判断

      //rob_tail_securitytag_bool(w+w+1) := io.enq_uops_securitytag(w)(1)
      //rob_tail_securitytag_bool(w+w) := io.enq_uops_securitytag(w)(0)

      rob_tail_securitytag  := Cat((io.enq_uops_securitytag(0)&Cat(io.enq_valids(0),io.enq_valids(0))),
                                   (io.enq_uops_securitytag(1)&Cat(io.enq_valids(1),io.enq_valids(1))),
                                   (io.enq_uops_securitytag(2)&Cat(io.enq_valids(2),io.enq_valids(2))),
                                   (io.enq_uops_securitytag(3)&Cat(io.enq_valids(3),io.enq_valids(3)))
                                  )

      rob_inst0_tag := Mux(io.enq_valids(0),io.enq_uops_securitytag(0),0.U)
      rob_inst1_tag := Mux(io.enq_valids(1),io.enq_uops_securitytag(1),0.U)
      rob_inst2_tag := Mux(io.enq_valids(2),io.enq_uops_securitytag(2),0.U)
      rob_inst3_tag := Mux(io.enq_valids(3),io.enq_uops_securitytag(3),0.U)

      rob_inst_tag_max01 := Mux(rob_inst0_tag > rob_inst1_tag,rob_inst0_tag,rob_inst1_tag)
      rob_inst_tag_max23 := Mux(rob_inst2_tag > rob_inst3_tag,rob_inst2_tag,rob_inst3_tag)

      rob_tail_pctag := Mux(rob_inst_tag_max01 > rob_inst_tag_max23,rob_inst_tag_max01,rob_inst_tag_max23)


      

      //直接更新即可，因为可以修改prefetcher使得每个packet要么是同一个tag，要么以tc-tag结尾

      rob_unsafe(rob_tail)    := io.enq_uops(w).unsafe          //表示该指令是否处于不安全状态
      rob_uop(rob_tail)       := io.enq_uops(w)                           //指令信息

      rob_uop_securitytag(rob_tail)       := io.enq_uops_securitytag(w)          //指令tag

      rob_row_securitytag(rob_tail)       := Cat(io.enq_uops_securitytag(3),io.enq_uops_securitytag(2),io.enq_uops_securitytag(1),io.enq_uops_securitytag(0))           //all_tag 

      

      rob_exception(rob_tail) := io.enq_uops(w).exception            //指令异常

      rob_predicated(rob_tail)   := false.B                                               //
      rob_fflags(rob_tail)    := 0.U

      assert (rob_val(rob_tail) === false.B, "[rob] overwriting a valid entry.")    //如果尾部已经是有效的，则覆盖了一个有效的rob条目
      assert ((io.enq_uops(w).rob_idx >> log2Ceil(coreWidth)) === rob_tail)
    } .elsewhen (io.enq_valids.reduce(_|_) && !rob_val(rob_tail)) {                       //如果尾部已有效
      rob_uop(rob_tail).debug_inst := BUBBLE // just for debug purposes

      //rob_uop(rob_tail).debug_inst_securitytag := io.enq_uops_securitytag                                            //这是啥？

    }

    //-----------------------------------------------
    // Writeback

    //写回

    for (i <- 0 until numWakeupPorts) {                                                                                                            //3个发射路径都能写回
      val wb_resp = io.wb_resps(i)
      val wb_uop = wb_resp.bits.uop

      //val wb_uop_securitytag = wb_resp.bits.uop_securitytag

      //rob_idx 7bit，用于指示指令在rob中具体位置（4bank+32行，2+5）
      val row_idx = GetRowIdx(wb_uop.rob_idx)                                                                                          //
      when (wb_resp.valid && MatchBank(GetBankIdx(wb_uop.rob_idx))) {                                   //找到对应指令
        rob_bsy(row_idx)      := false.B                                                                                                                   //更改指令状态，指示该指令执行完毕
        rob_unsafe(row_idx)   := false.B                                                                                                              //不处于推测状态                            

        rob_data_check_unpass(row_idx)  := io.data_check_unpasses(i)                            //将alu计算后的data结果写入

        rob_predicated(row_idx)  := wb_resp.bits.predicated                                                                  //预测执行与否
      }
      // TODO check that fflags aren't overwritten
      // TODO check that the wb is to a valid ROB entry, give it a time stamp
//        assert (!(wb_resp.valid && MatchBank(GetBankIdx(wb_uop.rob_idx)) &&
//                  wb_uop.fp_val && !(wb_uop.is_load || wb_uop.is_store) &&
//                  rob_exc_cause(row_idx) =/= 0.U),
//                  "FP instruction writing back exc bits is overriding an existing exception.")
    }

    // Stores have a separate method to clear busy bits   
    //存储指令清除busy
    //应该把dcahce_check_exception传入（未实现）       
    for (clr_rob_idx <- io.lsu_clr_bsy) {
      when (clr_rob_idx.valid && MatchBank(GetBankIdx(clr_rob_idx.bits))) {
        val cidx = GetRowIdx(clr_rob_idx.bits)
        rob_bsy(cidx)    := false.B
        rob_unsafe(cidx) := false.B

        //rob_data_check_unpass(cidx)  := io.lsu_data_check_unpass                             //lsu应该负责检查数据tag，这里只是传入（默认为false）

        assert (rob_val(cidx) === true.B, "[rob] store writing back to invalid entry.")
        assert (rob_bsy(cidx) === true.B, "[rob] store writing back to a not-busy entry.")
      }
    }
    for (clr <- io.lsu_clr_unsafe) {
      when (clr.valid && MatchBank(GetBankIdx(clr.bits))) {
        val cidx = GetRowIdx(clr.bits)
        rob_unsafe(cidx) := false.B
      }
    }


    //-----------------------------------------------
    // Accruing fflags
    for (i <- 0 until numFpuPorts) {
      val fflag_uop = io.fflags(i).bits.uop
      when (io.fflags(i).valid && MatchBank(GetBankIdx(fflag_uop.rob_idx))) {
        rob_fflags(GetRowIdx(fflag_uop.rob_idx)) := io.fflags(i).bits.flags
      }
    }

    //-----------------------------------------------------
    // Exceptions     异常处理
    // (the cause bits are compressed and stored elsewhere)

    when (io.lxcpt.valid && MatchBank(GetBankIdx(io.lxcpt.bits.uop.rob_idx))) {
      rob_exception(GetRowIdx(io.lxcpt.bits.uop.rob_idx)) := true.B                                 //lsu原始的异常

      //rob_data_check_unpass(GetRowIdx(io.lxcpt.bits.uop.rob_idx)) := io.lxcpt_data_check_unpass            //lsu data_check异常

      when (io.lxcpt.bits.cause =/= MINI_EXCEPTION_MEM_ORDERING) {
        // In the case of a mem-ordering failure, the failing load will have been marked safe already.
        assert(rob_unsafe(GetRowIdx(io.lxcpt.bits.uop.rob_idx)),
          "An instruction marked as safe is causing an exception")
      }
    }
    can_throw_exception(w) := rob_val(rob_head) && ( rob_exception(rob_head) )                                // ||
                                                                                                              // rob_data_check_unpass(rob_head) ||
                                                                                                              // rob_inst_check_unpass(rob_head) )           
    //第w bank的头部指令（异常从最早进来的开始）有异常，此时汇总异常，后续其实没必要传出异常，目前将这两个异常信号传出rob是因为方便后续fpga调试，暂时不汇总也是因为这样


       
                                                                                                     

    //-----------------------------------------------
    // Commit or Rollback  提交/回滚

    // Can this instruction commit? (the check for exceptions/rob_state happens later).
    can_commit(w) := rob_val(rob_head) && !(rob_bsy(rob_head)) && !io.csr_stall   


       
    //这个bank的头部指令可以提交（即计算完毕，但是不确定有无异常）


    // use the same "com_uop" for both rollback AND commit
    // Perform Commit
    io.commit.valids(w) := will_commit(w)                                                                               //will_commit为这个bank的头部指令将要提交（已检查了异常）
    io.commit.arch_valids(w) := will_commit(w) && !rob_predicated(com_idx)     //可以反映到架构级上（非推测执行）
    io.commit.uops(w)   := rob_uop(com_idx)                                                                        //提交信息（包含异常信息）

    io.commit.uops(w).data_check_unpass  := rob_data_check_unpass(com_idx) && will_commit(w) && !rob_predicated(com_idx)                  //提交的两个检查结果(其实是没必要的，只是方便调试，且目前还是错位的，)
    io.commit.uops(w).inst_check_unpass  := /*rob_inst_check_unpass(com_idx) &&*/ will_commit(w) && !rob_predicated(com_idx) && rob_inst_check_unpass_commit(com_idx)       //      

    rob_inst_check_unpass_commit (rob_head)          :=    ((!rob_commit_pctag(1))&&(!rob_commit_pctag(0))&&(rob_uop_securitytag(rob_head)(1)))           || 
                                                       ((!rob_commit_pctag(0))&&(rob_uop_securitytag(rob_head)(0))&&(rob_uop_securitytag(rob_head)(1))) ||
                                                       ((!rob_commit_pctag(1))&&(rob_commit_pctag(0))&&(!rob_uop_securitytag(rob_head)(1))&&(!rob_uop_securitytag(rob_head)(0)))
                                                                                                        //直接进行判断


    rob_com_securitytag  := Cat((rob_row_securitytag(com_idx)(1,0)&Cat(io.commit.valids(0),io.commit.valids(0))),             //确认提交时候才进行pc_tag维护
                                   (rob_row_securitytag(com_idx)(3,2)&Cat(io.commit.valids(1),io.commit.valids(1))),
                                   (rob_row_securitytag(com_idx)(5,4)&Cat(io.commit.valids(2),io.commit.valids(2))),
                                   (rob_row_securitytag(com_idx)(7,6)&Cat(io.commit.valids(3),io.commit.valids(3)))
                                  )

      rob_com_inst0_tag := Mux(io.commit.valids(0),rob_row_securitytag(com_idx)(1,0),0.U)
      rob_com_inst1_tag := Mux(io.commit.valids(1),rob_row_securitytag(com_idx)(3,2),0.U)
      rob_com_inst2_tag := Mux(io.commit.valids(2),rob_row_securitytag(com_idx)(5,4),0.U)
      rob_com_inst3_tag := Mux(io.commit.valids(3),rob_row_securitytag(com_idx)(7,6),0.U)        

      rob_com_inst_tag_max01 := Mux(rob_com_inst0_tag > rob_com_inst1_tag,rob_com_inst0_tag,rob_com_inst1_tag)
      rob_com_inst_tag_max23 := Mux(rob_com_inst2_tag > rob_com_inst3_tag,rob_com_inst2_tag,rob_com_inst3_tag)

      rob_commit_pctag := Mux((io.commit.valids(0) || io.commit.valids(1) || io.commit.valids(2) || io.commit.valids(3) ), 
                                                                   Mux(rob_com_inst_tag_max01 > rob_com_inst_tag_max23,rob_com_inst_tag_max01,rob_com_inst_tag_max23),
                                                                   rob_commit_pctag)         //加入如果4个都无效，则保持不变

    when(io.commit.uops(w).data_check_unpass){                                                            //打印错误信息
      printf("data_functional_units exception :\n")
      printf(p"exception_rob_idx = ${com_idx}\n")
    }

    when(io.commit.uops(w).inst_check_unpass){
      printf("inst_tag transition exception :\n")
      printf(p"exception_rob_idx = ${com_idx}\n")
    }


    dontTouch(io.commit.uops(w).data_check_unpass)     
    dontTouch(io.commit.uops(w).inst_check_unpass)


    io.commit.debug_insts(w) := rob_debug_inst_rdata(w)




    // We unbusy branches in b1, but its easier to mark the taken/provider src in b2,
    // when the branch might be committing
    //分支预测
    when (io.brupdate.b2.mispredict &&
      MatchBank(GetBankIdx(io.brupdate.b2.uop.rob_idx)) &&
      GetRowIdx(io.brupdate.b2.uop.rob_idx) === com_idx) {
      io.commit.uops(w).debug_fsrc := BSRC_C
      io.commit.uops(w).taken      := io.brupdate.b2.taken
    }


    // Don't attempt to rollback the tail's row when the rob is full.
    //当rob已满时，不要rollback
    val rbk_row = rob_state === s_rollback && !full

    io.commit.rbk_valids(w) := rbk_row && rob_val(com_idx) && !(enableCommitMapTable.B)
    io.commit.rollback := (rob_state === s_rollback)

    assert (!(io.commit.valids.reduce(_||_) && io.commit.rbk_valids.reduce(_||_)),
      "com_valids and rbk_valids are mutually exclusive")

    when (rbk_row) {
      rob_val(com_idx)       := false.B
      rob_exception(com_idx) := false.B
    }

    if (enableCommitMapTable) {
      when (RegNext(exception_thrown)) {             //异常抛出时，rob所有val无效，即暂时不发射新指令
        for (i <- 0 until numRobRows) {
          rob_val(i) := false.B
          rob_bsy(i) := false.B
          rob_uop(i).debug_inst := BUBBLE

          //rob_uop(i).debug_inst_securitytag := 0.U                       //tag的设置应该不重要

        }
      }
    }

    // -----------------------------------------------
    // Kill speculated entries on branch mispredict
    //错误推测时候，清除被推测的条目
    for (i <- 0 until numRobRows) {
      val br_mask = rob_uop(i).br_mask

      //kill instruction if mispredict & br mask match
      when (IsKilledByBranch(io.brupdate, br_mask))
      {
        rob_val(i) := false.B
        rob_uop(i.U).debug_inst := BUBBLE

        //rob_uop(i.U).debug_inst_securitytag := 0.U                //tag

      } .elsewhen (rob_val(i)) {
        // clear speculation bit even on correct speculation
        rob_uop(i).br_mask := GetNewBrMask(io.brupdate, br_mask)
      }
    }


    // Debug signal to figure out which prediction structure
    // or core resolved a branch correctly
    when (io.brupdate.b2.mispredict &&
      MatchBank(GetBankIdx(io.brupdate.b2.uop.rob_idx))) {
      rob_uop(GetRowIdx(io.brupdate.b2.uop.rob_idx)).debug_fsrc := BSRC_C
      rob_uop(GetRowIdx(io.brupdate.b2.uop.rob_idx)).taken      := io.brupdate.b2.taken
    }

    // -----------------------------------------------
    // Commit 提交后，将条目设为无效，即可写入新的条目
    when (will_commit(w)) {
      rob_val(rob_head) := false.B
    }

    // -----------------------------------------------
    // Outputs
    //rob的输出
    rob_head_vals(w)     := rob_val(rob_head)                         //rob头有效（负责提交）
    rob_tail_vals(w)     := rob_val(rob_tail)                                //rob尾有效（负责dis）
    rob_head_fflags(w)   := rob_fflags(rob_head)                  
    rob_head_uses_stq(w) := rob_uop(rob_head).uses_stq         //st和ld的标志
    rob_head_uses_ldq(w) := rob_uop(rob_head).uses_ldq

    //------------------------------------------------
    // Invalid entries are safe; thrown exceptions are unsafe.
    //标记不安全的条目（用以计算pnr位值）
    for (i <- 0 until numRobRows) {
      rob_unsafe_masked((i << log2Ceil(coreWidth)) + w) := rob_val(i) && (rob_unsafe(i) || rob_exception(i))
    }
    // Read unsafe status of PNR row.
    //pnr行是否安全
    rob_pnr_unsafe(w) := rob_val(rob_pnr) && (rob_unsafe(rob_pnr) || rob_exception(rob_pnr))

    // -----------------------------------------------
    // debugging write ports that should not be synthesized
    when (will_commit(w)) {
      rob_uop(rob_head).debug_inst := BUBBLE
 
      //rob_uop(rob_head).debug_inst_securitytag := 0.U      //tag

    } .elsewhen (rbk_row)
    {
      rob_uop(rob_tail).debug_inst := BUBBLE

      //rob_uop(rob_tail).debug_inst_securitytag := 0.U      //tag

    }

    //--------------------------------------------------
    // Debug: for debug purposes, track side-effects to all register destinations

    for (i <- 0 until numWakeupPorts) {
      val rob_idx = io.wb_resps(i).bits.uop.rob_idx
      when (io.debug_wb_valids(i) && MatchBank(GetBankIdx(rob_idx))) {
        rob_debug_wdata(GetRowIdx(rob_idx)) := io.debug_wb_wdata(i)

        rob_debug_wdata_securitytag(GetRowIdx(rob_idx)) := io.debug_wb_wdata_securitytag(i)

      }
      val temp_uop = rob_uop(GetRowIdx(rob_idx))

      val temp_uop_securitytag = rob_uop_securitytag(GetRowIdx(rob_idx))

      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               !rob_val(GetRowIdx(rob_idx))),
               "[rob] writeback (" + i + ") occurred to an invalid ROB entry.")
      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               !rob_bsy(GetRowIdx(rob_idx))),
               "[rob] writeback (" + i + ") occurred to a not-busy ROB entry.")
      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               temp_uop.ldst_val && temp_uop.pdst =/= io.wb_resps(i).bits.uop.pdst),
               "[rob] writeback (" + i + ") occurred to the wrong pdst.")
    }
    io.commit.debug_wdata(w) := rob_debug_wdata(rob_head)

    io.commit.debug_wdata_securitytag(w) := rob_debug_wdata_securitytag(rob_head)

  } //for (w <- 0 until coreWidth)

  // **************************************************************************
  // --------------------------------------------------------------------------
  // **************************************************************************

  // -----------------------------------------------
  // Commit Logic
  // need to take a "can_commit" array, and let the first can_commits commit
  //检索can_commit有效的行，并让最靠前的提交
  //靠前的可能会阻止靠后的提交
  // previous instructions may block the commit of younger instructions in the commit bundle
  // e.g., exception, or (valid && busy).
  //只有头部才能异常
  // Finally, don't throw an exception if there are instructions in front of
  // it that want to commit (only throw exception when head of the bundle).

  var block_commit = (rob_state =/= s_normal) && (rob_state =/= s_wait_till_empty) || 
                                                    RegNext(exception_thrown) || RegNext(RegNext(exception_thrown))
  var will_throw_exception = false.B
  var block_xcpt   = false.B

  for (w <- 0 until coreWidth) {
    will_throw_exception = (can_throw_exception(w) && !block_commit && !block_xcpt) || will_throw_exception 
                                                   //can_throw_exception已加入data/inst_check_unpass                                  
            

    will_commit(w)       := can_commit(w) && !can_throw_exception(w) && !block_commit
    block_commit         = (rob_head_vals(w) &&
                           (!can_commit(w) || can_throw_exception(w))) || block_commit
    block_xcpt           = will_commit(w)
  }

  // Note: exception must be in the commit bundle.
  // Note: exception must be the first valid instruction in the commit bundle.
  exception_thrown := will_throw_exception
  val is_mini_exception = io.com_xcpt.bits.cause === MINI_EXCEPTION_MEM_ORDERING         //小型异常
  io.com_xcpt.valid := exception_thrown && !is_mini_exception                                                             //非小型异常
  io.com_xcpt.bits.cause := r_xcpt_uop.exc_cause                                                                                        //cause（似乎前端异常是没有cause的）

  io.com_xcpt.bits.badvaddr := Sext(r_xcpt_badvaddr, xLen)

  //以下的部分都是重新取指的吧？应该会送往前端？（得看一下连线）
  val insn_sys_pc2epc =
    rob_head_vals.reduce(_|_) && PriorityMux(rob_head_vals, io.commit.uops.map{u => u.is_sys_pc2epc})

  val refetch_inst = exception_thrown || insn_sys_pc2epc
  val com_xcpt_uop = PriorityMux(rob_head_vals, io.commit.uops)                     //commit里包括异常信息

  io.com_xcpt.bits.ftq_idx   := com_xcpt_uop.ftq_idx
  io.com_xcpt.bits.edge_inst := com_xcpt_uop.edge_inst
  io.com_xcpt.bits.is_rvc    := com_xcpt_uop.is_rvc
  io.com_xcpt.bits.pc_lob    := com_xcpt_uop.pc_lob

  io.com_xcpt.bits.data_check_unpass  := com_xcpt_uop.data_check_unpass      //两个异常发送到csr
  io.com_xcpt.bits.inst_check_unpass  := com_xcpt_uop.inst_check_unpass  

  val flush_commit_mask = Range(0,coreWidth).map{i => io.commit.valids(i) && io.commit.uops(i).flush_on_commit}
  val flush_commit = flush_commit_mask.reduce(_|_)
  val flush_val = exception_thrown || flush_commit

  assert(!(PopCount(flush_commit_mask) > 1.U),
    "[rob] Can't commit multiple flush_on_commit instructions on one cycle")

  val flush_uop = Mux(exception_thrown, com_xcpt_uop, Mux1H(flush_commit_mask, io.commit.uops))


  // delay a cycle for critical path considerations
  io.flush.valid          := flush_val
  io.flush.bits.ftq_idx   := flush_uop.ftq_idx
  io.flush.bits.pc_lob    := flush_uop.pc_lob
  io.flush.bits.edge_inst := flush_uop.edge_inst
  io.flush.bits.is_rvc    := flush_uop.is_rvc
  io.flush.bits.flush_typ := FlushTypes.getType(flush_val,                                             //冲刷类型选择，直接连到前端的pc选择部分
                                                exception_thrown && !is_mini_exception,
                                                flush_commit && flush_uop.uopc === uopERET,
                                                refetch_inst)


  // -----------------------------------------------
  // FP Exceptions
  // send fflags bits to the CSRFile to accrue

  val fflags_val = Wire(Vec(coreWidth, Bool()))
  val fflags     = Wire(Vec(coreWidth, UInt(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W)))

  for (w <- 0 until coreWidth) {
    fflags_val(w) :=
      io.commit.valids(w) &&
      io.commit.uops(w).fp_val &&
      !io.commit.uops(w).uses_stq

    fflags(w) := Mux(fflags_val(w), rob_head_fflags(w), 0.U)

    assert (!(io.commit.valids(w) &&
             !io.commit.uops(w).fp_val &&
             rob_head_fflags(w) =/= 0.U),
             "Committed non-FP instruction has non-zero fflag bits.")
    assert (!(io.commit.valids(w) &&
             io.commit.uops(w).fp_val &&
             (io.commit.uops(w).uses_ldq || io.commit.uops(w).uses_stq) &&
             rob_head_fflags(w) =/= 0.U),
             "Committed FP load or store has non-zero fflag bits.")
  }
  io.commit.fflags.valid := fflags_val.reduce(_|_)
  io.commit.fflags.bits  := fflags.reduce(_|_)

  // -----------------------------------------------
  // Exception Tracking Logic，异常追踪逻辑，只存储最早发生的异常
  // only store the oldest exception, since only one can happen!

  val next_xcpt_uop = Wire(new MicroOp())
  next_xcpt_uop := r_xcpt_uop                              
        
  val enq_xcpts = Wire(Vec(coreWidth, Bool()))                                               //每个bank的异常指示位
  for (i <- 0 until coreWidth) {
    enq_xcpts(i) := io.enq_valids(i) && io.enq_uops(i).exception               //dispatch的异常异常指示
  }

  when (!(io.flush.valid || exception_thrown) && rob_state =/= s_rollback) {
    when (io.lxcpt.valid) {                                                                                             //lsu发来异常
      val new_xcpt_uop = io.lxcpt.bits.uop                                                            //记录lsu异常输入

  
      when (!r_xcpt_val || IsOlder(new_xcpt_uop.rob_idx, r_xcpt_uop.rob_idx, rob_head_idx)) {         
        //isolder(i0,i1,head),用于判断i0是否是最小的，即new_xcpt_uop所记录的异常是最早的
        r_xcpt_val              := true.B                                                      //此时已经记录了一个异常，这个标志只有在异常被处理后才会重置
        next_xcpt_uop           := new_xcpt_uop                                       

        next_xcpt_uop.exc_cause := io.lxcpt.bits.cause           //这种异常的 cause 要单独放一个地方
        r_xcpt_badvaddr         := io.lxcpt.bits.badvaddr

      }
    } .elsewhen (!r_xcpt_val && enq_xcpts.reduce(_|_)) {                             //lsu没有异常                      
      val idx = enq_xcpts.indexWhere{i: Bool => i}

      // if no exception yet, dispatch exception wins
      //如果没有lsu异常，则记录dis异常（即前端异常）
      r_xcpt_val      := true.B                                                               
      next_xcpt_uop   := io.enq_uops(idx)                 //dispatch异常信息输入，包含cause

      r_xcpt_badvaddr := AlignPCToBoundary(io.xcpt_fetch_pc, icBlockBytes) | io.enq_uops(idx).pc_lob
    }
  }

  r_xcpt_uop         := next_xcpt_uop                          //记录的异常                                                        

  r_xcpt_uop.br_mask := GetNewBrMask(io.brupdate, next_xcpt_uop)
  when (io.flush.valid || IsKilledByBranch(io.brupdate, next_xcpt_uop)) {            //冲刷时则重置记录的异常
    r_xcpt_val := false.B
  }

  assert (!(exception_thrown && !r_xcpt_val),
    "ROB trying to throw an exception, but it doesn't have a valid xcpt_cause")

  assert (!(empty && r_xcpt_val),
    "ROB is empty, but believes it has an outstanding exception.")

  assert (!(will_throw_exception && (GetRowIdx(r_xcpt_uop.rob_idx) =/= rob_head)),
    "ROB is throwing an exception, but the stored exception information's " +
    "rob_idx does not match the rob_head")

  // -----------------------------------------------
  // ROB Head Logic
 //rob_head指针维护
  // remember if we're still waiting on the rest of the dispatch packet, and prevent
  // the rob_head from advancing if it commits a partial parket before we
  // dispatch the rest of it.
  // update when committed ALL valid instructions in commit_bundle

  val rob_deq = WireInit(false.B)
  val r_partial_row = RegInit(false.B)

  when (io.enq_valids.reduce(_|_)) {
    r_partial_row := io.enq_partial_stall
  }

  val finished_committing_row =
    (io.commit.valids.asUInt =/= 0.U) &&
    ((will_commit.asUInt ^ rob_head_vals.asUInt) === 0.U) &&
    !(r_partial_row && rob_head === rob_tail && !maybe_full)

  when (finished_committing_row) {                                             //提交完成后
    rob_head     := WrapInc(rob_head, numRobRows)             //wrapinc为自增，增大到上限（numrobrows）会归0
    rob_head_lsb := 0.U
    rob_deq      := true.B
  } .otherwise {
    rob_head_lsb := OHToUInt(PriorityEncoderOH(rob_head_vals.asUInt))   
  }

  // -----------------------------------------------
  // ROB Point-of-No-Return (PNR) Logic
  //pnr指针维护
  // Acts as a second head, but only waits on busy instructions which might cause misspeculation.
  // TODO is it worth it to add an extra 'parity' bit to all rob pointer logic?
  // Makes 'older than' comparisons ~3x cheaper, in case we're going to use the PNR to do a large number of those.
  // Also doesn't require the rob tail (or head) to be exported to whatever we want to compare with the PNR.

  if (enableFastPNR) {
    val unsafe_entry_in_rob = rob_unsafe_masked.reduce(_||_)
    val next_rob_pnr_idx = Mux(unsafe_entry_in_rob,
                               AgePriorityEncoder(rob_unsafe_masked, rob_head_idx),
                               rob_tail << log2Ceil(coreWidth) | PriorityEncoder(~rob_tail_vals.asUInt))
    rob_pnr := next_rob_pnr_idx >> log2Ceil(coreWidth)
    if (coreWidth > 1)
      rob_pnr_lsb := next_rob_pnr_idx(log2Ceil(coreWidth)-1, 0)
  } else {
    // Distinguish between PNR being at head/tail when ROB is full.
    // Works the same as maybe_full tracking for the ROB tail.
    val pnr_maybe_at_tail = RegInit(false.B)

    val safe_to_inc = rob_state === s_normal || rob_state === s_wait_till_empty
    val do_inc_row  = !rob_pnr_unsafe.reduce(_||_) && (rob_pnr =/= rob_tail || (full && !pnr_maybe_at_tail))
    when (empty && io.enq_valids.asUInt =/= 0.U) {
      // Unforunately for us, the ROB does not use its entries in monotonically
      //  increasing order, even in the case of no exceptions. The edge case
      //  arises when partial rows are enqueued and committed, leaving an empty
      //  ROB.
      rob_pnr     := rob_head
      rob_pnr_lsb := PriorityEncoder(io.enq_valids)
    } .elsewhen (safe_to_inc && do_inc_row) {
      rob_pnr     := WrapInc(rob_pnr, numRobRows)
      rob_pnr_lsb := 0.U
    } .elsewhen (safe_to_inc && (rob_pnr =/= rob_tail || (full && !pnr_maybe_at_tail))) {
      rob_pnr_lsb := PriorityEncoder(rob_pnr_unsafe)
    } .elsewhen (safe_to_inc && !full && !empty) {
      rob_pnr_lsb := PriorityEncoder(rob_pnr_unsafe.asUInt | ~MaskLower(rob_tail_vals.asUInt))
    } .elsewhen (full && pnr_maybe_at_tail) {
      rob_pnr_lsb := 0.U
    }

    pnr_maybe_at_tail := !rob_deq && (do_inc_row || pnr_maybe_at_tail)
  }

  // Head overrunning PNR likely means an entry hasn't been marked as safe when it should have been.
  assert(!IsOlder(rob_pnr_idx, rob_head_idx, rob_tail_idx) || rob_pnr_idx === rob_tail_idx)

  // PNR overrunning tail likely means an entry has been marked as safe when it shouldn't have been.
  assert(!IsOlder(rob_tail_idx, rob_pnr_idx, rob_head_idx) || full)

  // -----------------------------------------------
  // ROB Tail Logic
  //rob_tail指针维护
  //也应该承担rob_tail_tag的维护？

  val rob_enq = WireInit(false.B)

  when (rob_state === s_rollback && (rob_tail =/= rob_head || maybe_full)) {
    // Rollback a row
    rob_tail     := WrapDec(rob_tail, numRobRows)       //wrapdec为自减函数，减到0会返回上限
    rob_tail_lsb := (coreWidth-1).U
    rob_deq := true.B
  } .elsewhen (rob_state === s_rollback && (rob_tail === rob_head) && !maybe_full) {
    // Rollback an entry
    rob_tail_lsb := rob_head_lsb
  } .elsewhen (io.brupdate.b2.mispredict) {
    rob_tail     := WrapInc(GetRowIdx(io.brupdate.b2.uop.rob_idx), numRobRows)
    rob_tail_lsb := 0.U
  } .elsewhen (io.enq_valids.asUInt =/= 0.U && !io.enq_partial_stall) {                  //tag updata
    rob_tail     := WrapInc(rob_tail, numRobRows)
    rob_tail_lsb := 0.U
    rob_enq      := true.B
  } .elsewhen (io.enq_valids.asUInt =/= 0.U && io.enq_partial_stall) {
    rob_tail_lsb := PriorityEncoder(~MaskLower(io.enq_valids.asUInt))
  }


  if (enableCommitMapTable) {
    when (RegNext(exception_thrown)) {   //抛出异常时，需要冲刷流水线，故直接冲刷rob
      rob_tail     := 0.U
      rob_tail_lsb := 0.U
      rob_head     := 0.U
      rob_pnr      := 0.U
      rob_pnr_lsb  := 0.U
    }
  }

  // -----------------------------------------------
  // Full/Empty Logic
  // The ROB can be completely full, but only if it did not dispatch a row in the prior cycle.
  // I.E. at least one entry will be empty when in a steady state of dispatching and committing a row each cycle.
  // TODO should we add an extra 'parity bit' onto the ROB pointers to simplify this logic?

  maybe_full := !rob_deq && (rob_enq || maybe_full) || io.brupdate.b1.mispredict_mask =/= 0.U
  full       := rob_tail === rob_head && maybe_full
  empty      := (rob_head === rob_tail) && (rob_head_vals.asUInt === 0.U)

  io.rob_head_idx := rob_head_idx
  io.rob_tail_idx := rob_tail_idx
  io.rob_pnr_idx  := rob_pnr_idx
  io.empty        := empty
  io.ready        := (rob_state === s_normal) && !full && !r_xcpt_val

  //-----------------------------------------------
  //-----------------------------------------------
  //-----------------------------------------------

  // ROB FSM
  //rob的状态机
  if (!enableCommitMapTable) {
    switch (rob_state) {
      is (s_reset) {
        rob_state := s_normal
      }
      is (s_normal) {
        // Delay rollback 2 cycles so branch mispredictions can drain
        when (RegNext(RegNext(exception_thrown))) {
          rob_state := s_rollback
        } .otherwise {
          for (w <- 0 until coreWidth) {
            when (io.enq_valids(w) && io.enq_uops(w).is_unique) {
              rob_state := s_wait_till_empty
            }
          }
        }
      }
      is (s_rollback) {
        when (empty) {
          rob_state := s_normal
        }
      }
      is (s_wait_till_empty) {
        when (RegNext(exception_thrown)) {
          rob_state := s_rollback
        } .elsewhen (empty) {
          rob_state := s_normal
        }
      }
    }
  } else {
    switch (rob_state) {
      is (s_reset) {
        rob_state := s_normal
      }
      is (s_normal) {
        when (exception_thrown) {
          ; //rob_state := s_rollback
        } .otherwise {
          for (w <- 0 until coreWidth) {
            when (io.enq_valids(w) && io.enq_uops(w).is_unique) {
              rob_state := s_wait_till_empty
            }
          }
        }
      }
      is (s_rollback) {
        when (rob_tail_idx  === rob_head_idx) {
          rob_state := s_normal
        }
      }
      is (s_wait_till_empty) {
        when (exception_thrown) {
          ; //rob_state := s_rollback
        } .elsewhen (rob_tail === rob_head) {
          rob_state := s_normal
        }
      }
    }
  }

  // -----------------------------------------------
  // Outputs

  io.com_load_is_at_rob_head := RegNext(rob_head_uses_ldq(PriorityEncoder(rob_head_vals.asUInt)) &&
                                        !will_commit.reduce(_||_))                         



  override def toString: String = BoomCoreStringPrefix(
    "==ROB==",
    "Machine Width      : " + coreWidth,
    "Rob Entries        : " + numRobEntries,
    "Rob Rows           : " + numRobRows,
    "Rob Row size       : " + log2Ceil(numRobRows),
    "log2Ceil(coreWidth): " + log2Ceil(coreWidth),
    "FPU FFlag Ports    : " + numFpuPorts)
}
