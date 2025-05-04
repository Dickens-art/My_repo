//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Frontend
//boom的前端（取指令和分支预测)
//securitytag主要增加pc和指令的securitytag部分，并增加取指的安全检查
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.ifu

import chisel3._
import chisel3.util._
import chisel3.internal.sourceinfo.{SourceInfo}

import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._

import boom.common._
import boom.exu.{CommitExceptionSignals, BranchDecode, BrUpdateInfo, BranchDecodeSignals}
import boom.util._


class FrontendResp(implicit p: Parameters) extends BoomBundle()(p) {           //前端响应，看最后的resp_security就行，加入了securitytag
  val pc = UInt(vaddrBitsExtended.W)                                                                                  // ID stage PC

  val data = UInt((fetchWidth * coreInstBits).W)    

  //取到的数据（其实就是取到的指令机器码，coreinstbits为单个指令的位宽，fetchwidth为一次性取几条）

  val mask = UInt(fetchWidth.W)                                                                                            //独热掩码                                                                                             
  val xcpt = new FrontendExceptions                                                                                   //异常  
  val ghist = new GlobalHistory                                                                                              //全局历史

  // fsrc provides the prediction FROM a branch in this packet                                 
  // tsrc provides the prediction TO this packet
  val fsrc = UInt(BSRC_SZ.W)
  val tsrc = UInt(BSRC_SZ.W)
}

class GlobalHistory(implicit p: Parameters) extends BoomBundle()(p)                //全局历史追踪
  with HasBoomFrontendParameters
{
  // For the dual banked case, each bank ignores the contribution of the
  // last bank to the history. Thus we have to track the most recent update to the
  // history in that case
  val old_history = UInt(globalHistoryLength.W)                                                               
  //旧的历史，记录了之前N条分支指令的结果，globalhistorylength = 64

  val current_saw_branch_not_taken = Bool()                                                                   //当前该分支预测信息

  val new_saw_branch_not_taken = Bool()                                                                          //新的分支预测信息
  val new_saw_branch_taken     = Bool()                                                                               
  //之所以设置taken/not taken两个信号，是因为并非所有指令都是分支指令
  //当两个信号都无效时候，历史表不做更新

  //值得注意的是，在BOOM中GHR采用的是推测式更新法，当某条分支指令被预测后，就会更新在GHR中。当后端发现某条分支指令预测错误时，
  //就需要重置并更新正确的GHR，因此对于流水线中正在执行的每条分支指令，均保留了它当时的GHR快照，以便后面需要时进行恢复。


  val ras_idx = UInt(log2Ceil(nRasEntries).W)                                                                      //返回地址栈(RAS)的索引

  def histories(bank: Int) = {                                                                                                          //历史表
    if (nBanks == 1) {                                                                                                                           //如果仅有一个bank，则历史表就为旧历史
      old_history
    } else {
      require(nBanks == 2)                            //如果有两个bank，则需要根据新的分支预测信息选择历史表（旧历史左移1位代表更新上本条指令的预测结果）
      if (bank == 0) {                                          //第一个bank保存原始历史表
        old_history
      } else {                                                         //第二个bank保存预测更新后的历史表
        Mux(new_saw_branch_taken                            , old_history << 1 | 1.U,          //左移动一位并或上1，其实就是更新入taken
        Mux(new_saw_branch_not_taken                        , old_history << 1,              //左移动一位（补0），其实就是更新入not taken
                                                              old_history))                                                                 //历史表不变
      }
    }
  }

  def ===(other: GlobalHistory): Bool = {                                                                               //定义===函数？
    ((old_history === other.old_history) &&
     (new_saw_branch_not_taken === other.new_saw_branch_not_taken) &&
     (new_saw_branch_taken === other.new_saw_branch_taken)
    )
  }
  def =/=(other: GlobalHistory): Bool = !(this === other)                                               //定义=/=函数

  def update(branches: UInt, cfi_taken: Bool, cfi_is_br: Bool, cfi_idx: UInt,        //定义updata函数
    cfi_valid: Bool, addr: UInt,
    cfi_is_call: Bool, cfi_is_ret: Bool): GlobalHistory = {
    val cfi_idx_fixed = cfi_idx(log2Ceil(fetchWidth)-1,0)                                                 //索引
    val cfi_idx_oh = UIntToOH(cfi_idx_fixed)
    val new_history = Wire(new GlobalHistory)                                                                  //新历史表

    val not_taken_branches = branches & Mux(cfi_valid,
                                            MaskLower(cfi_idx_oh) & ~Mux(cfi_is_br && cfi_taken, cfi_idx_oh, 0.U(fetchWidth.W)),
                                            ~(0.U(fetchWidth.W)))

    if (nBanks == 1) {
      // In the single bank case every bank sees the history including the previous bank
      new_history := DontCare
      new_history.current_saw_branch_not_taken := false.B
      val saw_not_taken_branch = not_taken_branches =/= 0.U || current_saw_branch_not_taken
      new_history.old_history := Mux(cfi_is_br && cfi_taken && cfi_valid   , histories(0) << 1 | 1.U,
                                 Mux(saw_not_taken_branch                  , histories(0) << 1,
                                                                             histories(0)))
    } else {
      // In the two bank case every bank ignore the history added by the previous bank
      val base = histories(1)
      val cfi_in_bank_0 = cfi_valid && cfi_taken && cfi_idx_fixed < bankWidth.U
      val ignore_second_bank = cfi_in_bank_0 || mayNotBeDualBanked(addr)

      val first_bank_saw_not_taken = not_taken_branches(bankWidth-1,0) =/= 0.U || current_saw_branch_not_taken
      new_history.current_saw_branch_not_taken := false.B
      when (ignore_second_bank) {
        new_history.old_history := histories(1)
        new_history.new_saw_branch_not_taken := first_bank_saw_not_taken
        new_history.new_saw_branch_taken     := cfi_is_br && cfi_in_bank_0
      } .otherwise {
        new_history.old_history := Mux(cfi_is_br && cfi_in_bank_0                             , histories(1) << 1 | 1.U,
                                   Mux(first_bank_saw_not_taken                               , histories(1) << 1,
                                                                                                histories(1)))

        new_history.new_saw_branch_not_taken := not_taken_branches(fetchWidth-1,bankWidth) =/= 0.U
        new_history.new_saw_branch_taken     := cfi_valid && cfi_taken && cfi_is_br && !cfi_in_bank_0

      }
    }
    new_history.ras_idx := Mux(cfi_valid && cfi_is_call, WrapInc(ras_idx, nRasEntries),
                           Mux(cfi_valid && cfi_is_ret , WrapDec(ras_idx, nRasEntries), ras_idx))
    new_history
  }

}

/**
 * Parameters to manage a L1 Banked ICache
 */
trait HasBoomFrontendParameters extends HasL1ICacheParameters
{
  // How many banks does the ICache use?
  //Icache中Bank的数量
  //bank是能并行访问的阵列单位，一个bank有自己的读写外围电路，可以单独访问，把cache分成多个bank是为了提高访问并行度。
  //超过8字节就需要第二个bank，即一个bank最多一次取出64 bits
  //没看懂为什么要这样
  val nBanks = if (cacheParams.fetchBytes <= 8) 1 else 2           //fetcjbytes = 16, nbank = 2  //注意nbank不能等于0
  // How many bytes wide is a bank?
  val bankBytes = fetchBytes/nBanks  // = 8，每个bank访问时取出的字节数

  val bankWidth = fetchWidth/nBanks// 每个bank承担的指令数，bankwidth = 4,fetchwidth = 8

  require(nBanks == 1 || nBanks == 2) 



  // How many "chunks"/interleavings make up a cache line?
  val numChunks = cacheParams.blockBytes / bankBytes  //一次取出的block数据由几个bank承担，blockbytes = 64

  // Which bank is the address pointing to?
  def bank(addr: UInt) = if (nBanks == 2) addr(log2Ceil(bankBytes)) else 0.U         //判断数据在哪一个bank里，返回addr的第log2 bankbytes 位，妙啊
  def isLastBankInBlock(addr: UInt) = {                                                                                    //返回addr的（块偏移量-1，）
    (nBanks == 2).B && addr(blockOffBits-1, log2Ceil(bankBytes)) === (numChunks-1).U
  }
  def mayNotBeDualBanked(addr: UInt) = {
    require(nBanks == 2)
    isLastBankInBlock(addr)
  }

  def blockAlign(addr: UInt) = ~(~addr | (cacheParams.blockBytes-1).U)                //block对齐，即去掉addr的低 blockbytes 位
  def bankAlign(addr: UInt) = ~(~addr | (bankBytes-1).U)          //得到Bank中指令的地址对齐，即除去addr的低 bankbytes 位

  def fetchIdx(addr: UInt) = addr >> log2Ceil(fetchBytes)          //定义indix，addr去掉低 fetchbytes 位（即只索引第几个fetch）

  def nextBank(addr: UInt) = bankAlign(addr) + bankBytes.U    
  //下一个bank的首指令地址（考虑间隙指令，则nextbank应该从前一个bank的最后一条指令取起）
  def nextFetch(addr: UInt) = {                                                                  //下一次 fetch packet的地址
    if (nBanks == 1) {
      bankAlign(addr) + bankBytes.U                                                        //如果只有一个bank，那就是nextbank
    } else {
      require(nBanks == 2)                                                   //如果有两个bank，
      bankAlign(addr) + Mux(mayNotBeDualBanked(addr), bankBytes.U, fetchBytes.U)
    }
  }

  def fetchMask(addr: UInt) = {                          //返回指令的idx                                                                                                               
    val idx = addr.extract(log2Ceil(fetchWidth)+log2Ceil(coreInstBytes)-1, log2Ceil(coreInstBytes))   //idx作为区分fetchpacket内指令的索引
    if (nBanks == 1) {
      ((1 << fetchWidth)-1).U << idx             //乘2再减1（乘2是因为idx以字节区分，乘2后便为以半字区分）
    } else {
      val shamt = idx.extract(log2Ceil(fetchWidth)-2, 0)
      val end_mask = Mux(mayNotBeDualBanked(addr), Fill(fetchWidth/2, 1.U), Fill(fetchWidth, 1.U))
      ((1 << fetchWidth)-1).U << shamt & end_mask
    }
  }

  def bankMask(addr: UInt) = {
    val idx = addr.extract(log2Ceil(fetchWidth)+log2Ceil(coreInstBytes)-1, log2Ceil(coreInstBytes))
    if (nBanks == 1) {
      1.U(1.W)
    } else {
      Mux(mayNotBeDualBanked(addr), 1.U(2.W), 3.U(2.W))
    }
  }
}



/**
 * Bundle passed into the FetchBuffer and used to combine multiple
 * relevant signals together.
 */

//一条指令从指令缓存中取出来，到写入Fetch Buffer之前，要经历如下的过程： 
//1. Fetch Packet的大小为fetchWidth*16位，在F2周期每次从I-Cache中取出一个Fetch Packet大小的指令存放到IMem Response Queue中， 
//2. 在F3周期中维护取指“状态”，包括上一个Fetch Packet的最后16位
//（有可能上一个Fetch Packet的最后16位和本Fetch Packet的前16位共同组成一条指令）、PC等，用于解决RVC指令的若干问题。
//F3中还包括了预译码单元，用于确定每条指令的起始位置。 
//3. 在存到Fetch Buffer前将无效指令去掉


class FetchBundle(implicit p: Parameters) extends BoomBundle         
  with HasBoomFrontendParameters
{
  val pc            = UInt(vaddrBitsExtended.W)    //该packet的第一条指令的pc（作为一致化的基准）

  //val pc_securitytag            = UInt(fetchWidth*2.W)        //每条指令对应pc的securitytag（这个要依次检查后更新）

  val next_pc       = UInt(vaddrBitsExtended.W)   //下一个packet的第一条指令的pc

  val edge_inst     = Vec(nBanks, Bool())   // True if 1st instruction in this bundle is pc - 2    该packet中第一条指令是否为边缘指令
  val insts         = Vec(fetchWidth, Bits(32.W))

  val insts_securitytag         = Vec(fetchWidth, Bits(2.W))

  val exp_insts     = Vec(fetchWidth, Bits(32.W))

  //val exp_insts_securitytag     = Vec(fetchWidth, Bits(2.W))

  // Information for sfb folding
  // NOTE: This IS NOT equivalent to uop.pc_lob, that gets calculated in the FB
  val sfbs                 = Vec(fetchWidth, Bool())
  val sfb_masks            = Vec(fetchWidth, UInt((2*fetchWidth).W))
  val sfb_dests            = Vec(fetchWidth, UInt((1+log2Ceil(fetchBytes)).W))
  val shadowable_mask      = Vec(fetchWidth, Bool())
  val shadowed_mask        = Vec(fetchWidth, Bool())

  val cfi_idx       = Valid(UInt(log2Ceil(fetchWidth).W))
  val cfi_type      = UInt(CFI_SZ.W)
  val cfi_is_call   = Bool()
  val cfi_is_ret    = Bool()
  val cfi_npc_plus4 = Bool()

  val ras_top       = UInt(vaddrBitsExtended.W)   

  //val ras_top_securitytag       = UInt(2.W)          //tag

  val ftq_idx       = UInt(log2Ceil(ftqSz).W)
  val mask          = UInt(fetchWidth.W) // mark which words are valid instructions

  val br_mask       = UInt(fetchWidth.W)

  val ghist         = new GlobalHistory
  val lhist         = Vec(nBanks, UInt(localHistoryLength.W))

  val xcpt_pf_if    = Bool() // I-TLB miss (instruction fetch fault).
  val xcpt_ae_if    = Bool() // Access exception.

  val bp_debug_if_oh= Vec(fetchWidth, Bool())
  val bp_xcpt_if_oh = Vec(fetchWidth, Bool())

  val end_half      = Valid(UInt(16.W))


  val bpd_meta      = Vec(nBanks, UInt())

  // Source of the prediction from this bundle
  val fsrc    = UInt(BSRC_SZ.W)
  // Source of the prediction to this bundle
  val tsrc    = UInt(BSRC_SZ.W)
}



/**
 * IO for the BOOM Frontend to/from the CPU
 */
class BoomFrontendIO(implicit p: Parameters) extends BoomBundle           //看后边带securitytag的
{
  // Give the backend a packet of instructions.
  val fetchpacket       = Flipped(new DecoupledIO(new FetchBufferResp))

  // 1 for xcpt/jalr/auipc/flush
  val get_pc            = Flipped(Vec(2, new GetPCFromFtqIO()))
  val debug_ftq_idx     = Output(Vec(coreWidth, UInt(log2Ceil(ftqSz).W)))
  val debug_fetch_pc    = Input(Vec(coreWidth, UInt(vaddrBitsExtended.W)))

  // Breakpoint info
  val status            = Output(new MStatus)
  val bp                = Output(Vec(nBreakpoints, new BP))
  val mcontext          = Output(UInt(coreParams.mcontextWidth.W))
  val scontext          = Output(UInt(coreParams.scontextWidth.W))

  val sfence = Valid(new SFenceReq)

  val brupdate          = Output(new BrUpdateInfo)

  // Redirects change the PC
  val redirect_flush   = Output(Bool()) // Flush and hang the frontend?
  val redirect_val     = Output(Bool()) // Redirect the frontend?
  val redirect_pc      = Output(UInt()) // Where do we redirect to?
  val redirect_ftq_idx = Output(UInt()) // Which ftq entry should we reset to?
  val redirect_ghist   = Output(new GlobalHistory) // What are we setting as the global history?

  val commit = Valid(UInt(ftqSz.W))

  val flush_icache = Output(Bool())

  val perf = Input(new FrontendPerfEvents)
}

/**
 * Top level Frontend class
 * 前端
 * @param icacheParams parameters for the icache
 * @param hartid id for the hardware thread of the core
 */
class BoomFrontend(val icacheParams: ICacheParams, staticIdForMetadataUseOnly: Int)(implicit p: Parameters) extends LazyModule
{
  lazy val module = new BoomFrontendModule(this)
  val icache = LazyModule(new boom.ifu.ICache(icacheParams, staticIdForMetadataUseOnly))    //icache
  val masterNode = icache.masterNode                                                                                                                     //
  val resetVectorSinkNode = BundleBridgeSink[UInt](Some(() =>
    UInt(masterNode.edges.out.head.bundle.addressBits.W)))
}

/**
 * Bundle wrapping the IO for the Frontend as a whole
 *
 * @param outer top level Frontend class
 */
class BoomFrontendBundle(val outer: BoomFrontend) extends CoreBundle()(outer.p)         //看后边带securitytag的
{
  val cpu = Flipped(new BoomFrontendIO())
  val ptw = new TLBPTWIO()
  val errors = new ICacheErrors
}

//Security_Tag
/**
 * Main Frontend module that connects the icache, TLB, fetch controller,
 * and branch prediction pipeline together.
 *
 * @param outer top level Frontend class
 */
class BoomFrontendModule(outer: BoomFrontend) extends LazyModuleImp(outer)     
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
{
  val io = IO(new BoomFrontendBundle_Security(outer))                     //已包含securitytag
  val io_reset_vector = outer.resetVectorSinkNode.bundle                  //reset后的初始值?
  implicit val edge = outer.masterNode.edges.out(0)
  require(fetchWidth*coreInstBytes == outer.icacheParams.fetchBytes)

  val bpd = Module(new BranchPredictor)
  bpd.io.f3_fire := false.B
  val ras = Module(new BoomRAS)

  val icache = outer.icache.module
  icache.io.invalidate := io.cpu.flush_icache
  val tlb = Module(new TLB(true, log2Ceil(fetchBytes), TLBConfig(nTLBSets, nTLBWays)))           //fetchbytes=16，则igmaxsize=4
  io.ptw <> tlb.io.ptw
  io.cpu.perf.tlbMiss := io.ptw.req.fire
  io.cpu.perf.acquire := icache.io.perf.acquire

  // --------------------------------------------------------
  // **** NextPC Select (F0) ****         F0阶段，选择next_pc，并向icache发送请求
  //      Send request to ICache
  //该阶段的主要任务是选取PC值，然后将此PC值向Icache发送请求，并且将PC送到分支预测器中进行分支预测。
  //当系统复位时，PC会等于系统的初始PC值，全局历史寄存器会被初始化。
  //
  // --------------------------------------------------------

  val s0_vpc       = WireInit(0.U(vaddrBitsExtended.W))                           //取指pc（发送给icache）（vaddr）

  //val s0_vpc_securitytag       = Vec(fetchWidth, 0.U(2.W))                   //reset后的securitytag有待商榷 , 得看icache对应复位后第一条指令的securitytag

  val s0_ghist     = WireInit((0.U).asTypeOf(new GlobalHistory))         //全局历史
  //全局历史跟随着前端的每一个周期，记录了当前周期之前的全局历史，一旦发生重定向，需要依据这些信息进行全局历史的重定向
  val s0_tsrc      = WireInit(0.U(BSRC_SZ.W))                                                //分支类型
  val s0_valid     = WireInit(false.B)                                                                  //s0_valid控制是否访问icache，是否进入s1，是否访问bpd等
  val s0_is_replay = WireInit(false.B)                                                              //s0阶段重置？
  val s0_is_sfence = WireInit(false.B)                                                              //隔离？
  val s0_replay_resp = Wire(new TLBResp)                                                 
  //s0重置后的resp（即如果icache miss，需要用pc（addr）通过tlb转换成paddr，交给icache去访问内存）
  val s0_replay_bpd_resp = Wire(new BranchPredictionBundle)     //s0重置后bpd的resp
  val s0_replay_ppc  = Wire(UInt())                                                                 //paddr
  val s0_s1_use_f3_bpd_resp = WireInit(false.B)                                     //是否使用s3_bpd_resp




  when (RegNext(reset.asBool) && !reset.asBool) {      //reset信号上升沿，即复位
    s0_valid   := true.B                                                                  //
    s0_vpc     := io_reset_vector                                               //pc初始化（其实s0_vpc就是发送给icache的）

   //  for (i <- 0 until fetchWidth) {
   //     s0_vpc_securitytag(i) := 0.U
   //}          

    //securitytag的初始化，初始化后pc_securitytag是什么有待商榷（不能比初始化后指令的securitytag低1以上）

    //s0_vpc_securitytag   作为pc_tag更新 （是否应该是fetchwidth个securitytag？
    //我觉得是的，需要依次更新，且pc_tag的 更新 应该比 依据取的指令生成next_pc 早）
    //这个东西目前加入到fb里边了，在写入fb之前判断，然后更新pc_tag，等待下一次判断

    s0_ghist   := (0.U).asTypeOf(new GlobalHistory)      //初始化全局历史表
    s0_tsrc    := BSRC_C                                                              //分支类型
  }

  icache.io.req.valid     := s0_valid
  icache.io.req.bits.addr := s0_vpc                                      //将pc作为addr发送给icache            
  //因为icache不需要接受securitytag，只负责将存储的securitytag发给core，故不需要给icache发送addr_securitytag

  bpd.io.f0_req.valid      := s0_valid
  bpd.io.f0_req.bits.pc    := s0_vpc                 
  //bpd只是预测下一个pc，因此不需要pc_securitytag（pc_securitytag由下一条指令的inst_securitytag生成，
  //且在指令取到后才会更新pc_securitytag）
  bpd.io.f0_req.bits.ghist := s0_ghist

  // --------------------------------------------------------
  // **** ICache Access (F1) ****               F1阶段，
  //      Translate VPC
  //由于程序中的PC是虚拟地址，而实际访问存储器时需要使用物理地址，所以F1阶段的主要任务是访问TLB，
  //TLB缓存虚拟地址和其映射的物理地址，将虚拟的PC值翻译成物理地址，
  //并将该物理地址传给Icache。当TLB发生了miss或者F1阶段需要被冲刷时，需要中止对Icache的访问。
  //F1阶段可以得到单周期分支预测器的预测结果，如果预测结果为跳转，则将预测的目标跳转地址给到PC，反之，则正常指向下一个PC值。
  // --------------------------------------------------------
  val s1_vpc       = RegNext(s0_vpc)                          //沿用s0的vpc（vaddr）
  val s1_valid     = RegNext(s0_valid, false.B)      //s1有效信号
  val s1_ghist     = RegNext(s0_ghist)                      //s1全局历史
  val s1_is_replay = RegNext(s0_is_replay)         //s1重置
  val s1_is_sfence = RegNext(s0_is_sfence)
  val f1_clear     = WireInit(false.B)                           //清除f1
  val s1_tsrc      = RegNext(s0_tsrc)                          //分支类型

//tlb的输入
  tlb.io.req.valid      := (s1_valid && !s1_is_replay && !f1_clear) || s1_is_sfence            //s1有效，且s1没有重置，f1没有清除
  tlb.io.req.bits.cmd   := DontCare                
  tlb.io.req.bits.vaddr := s1_vpc                                                                      //送往tlb的vaddr，送往tlb也不需要securitytag，tlb只是找地址
  tlb.io.req.bits.passthrough := false.B                                                        //默认不透传，即使用虚拟存储                                         
  tlb.io.req.bits.size  := log2Ceil(coreInstBytes * fetchWidth).U       //size=4
  tlb.io.req.bits.v     := io.ptw.status.v
  tlb.io.req.bits.prv   := io.ptw.status.prv
  tlb.io.sfence         := RegNext(io.cpu.sfence)
  tlb.io.kill           := false.B

  val s1_tlb_miss = !s1_is_replay && tlb.io.resp.miss
  val s1_tlb_resp = Mux(s1_is_replay, RegNext(s0_replay_resp), tlb.io.resp)      //tlb响应（如果s1重置，则选择s0的重置响应）
  val s1_ppc  = Mux(s1_is_replay, RegNext(s0_replay_ppc), tlb.io.resp.paddr)  //tlb的响应给出paddr
  val s1_bpd_resp = bpd.io.resp.f1

  icache.io.s1_paddr := s1_ppc                                                                                                //将paddr送往icache
  icache.io.s1_kill  := tlb.io.resp.miss || f1_clear

  val f1_mask = fetchMask(s1_vpc)
  val f1_redirects = (0 until fetchWidth) map { i =>
    s1_valid && f1_mask(i) && s1_bpd_resp.preds(i).predicted_pc.valid &&
    (s1_bpd_resp.preds(i).is_jal ||
      (s1_bpd_resp.preds(i).is_br && s1_bpd_resp.preds(i).taken))
  }
  val f1_redirect_idx = PriorityEncoder(f1_redirects)
  val f1_do_redirect = f1_redirects.reduce(_||_) && useBPD.B
  val f1_targs = s1_bpd_resp.preds.map(_.predicted_pc.bits)
  val f1_predicted_target = Mux(f1_do_redirect,                                                             //f1预测目标地址（根据f1是否预测）
                                f1_targs(f1_redirect_idx),
                                nextFetch(s1_vpc))

  val f1_predicted_ghist = s1_ghist.update(
    s1_bpd_resp.preds.map(p => p.is_br && p.predicted_pc.valid).asUInt & f1_mask,
    s1_bpd_resp.preds(f1_redirect_idx).taken && f1_do_redirect,
    s1_bpd_resp.preds(f1_redirect_idx).is_br,
    f1_redirect_idx,
    f1_do_redirect,
    s1_vpc,
    false.B,
    false.B)

  when (s1_valid && !s1_tlb_miss) {               //当s1有效且s1的tlb访问没有miss，则直接更新pc去取指
    // Stop fetching on fault     
    s0_valid     := !(s1_tlb_resp.ae.inst || s1_tlb_resp.pf.inst)  //
    s0_tsrc      := BSRC_1                                           //分支类型
    s0_vpc       := f1_predicted_target                 //更新的pc
    s0_ghist     := f1_predicted_ghist                  //更新的历史表
    s0_is_replay := false.B
  }

  // --------------------------------------------------------
  // **** ICache Response (F2) ****
  //该阶段可以得到Icache的响应结果，当Icache的响应无效或者F3阶段传来的握手信号没有准备就绪时，
  //需要将该阶段的PC值重定向回F0阶段，重新访问Icache，并且需要清除此时F1阶段中的内容。
  //该阶段还会得到两周期分支预测器的预测结果，需要先判断F2阶段分支预测的目标跳转地址与此时F1阶段的PC值是否一样，
  //预测跳转方向与之前F1阶段的预测是否一样，如果是则证明两个阶段的分支预测器的预测结果相同，不需要进行PC的重定向，
  //只需要更新F2阶段的全局历史寄存器即可；反之，则需要清除F1阶段的内容，将F2阶段预测的目标地址给到F0阶段重定向PC。
  // --------------------------------------------------------

  val s2_valid = RegNext(s1_valid && !f1_clear, false.B)
  val s2_vpc   = RegNext(s1_vpc)
  val s2_ghist = Reg(new GlobalHistory)
  s2_ghist := s1_ghist
  val s2_ppc  = RegNext(s1_ppc)
  val s2_tsrc = RegNext(s1_tsrc) // tsrc provides the predictor component which provided the prediction TO this instruction
  //tsrc指出是哪类分支预测器预测到这个指令
  val s2_fsrc = WireInit(BSRC_1) // fsrc provides the predictor component which provided the prediction FROM this instruction
  //fsrc指出这个指令用的哪类分支预测器
  val f2_clear = WireInit(false.B)
  val s2_tlb_resp = RegNext(s1_tlb_resp)
  val s2_tlb_miss = RegNext(s1_tlb_miss)
  val s2_is_replay = RegNext(s1_is_replay) && s2_valid
  val s2_xcpt = s2_valid && (s2_tlb_resp.ae.inst || s2_tlb_resp.pf.inst) && !s2_is_replay
  val f3_ready = Wire(Bool())

  icache.io.s2_kill := s2_xcpt                //kill信号

  val f2_bpd_resp = bpd.io.resp.f2
  val f2_mask = fetchMask(s2_vpc)
  val f2_redirects = (0 until fetchWidth) map { i =>
    s2_valid && f2_mask(i) && f2_bpd_resp.preds(i).predicted_pc.valid &&
    (f2_bpd_resp.preds(i).is_jal ||
      (f2_bpd_resp.preds(i).is_br && f2_bpd_resp.preds(i).taken))
  }
  val f2_redirect_idx = PriorityEncoder(f2_redirects)
  val f2_targs = f2_bpd_resp.preds.map(_.predicted_pc.bits)
  val f2_do_redirect = f2_redirects.reduce(_||_) && useBPD.B
  val f2_predicted_target = Mux(f2_do_redirect,
                                f2_targs(f2_redirect_idx),
                                nextFetch(s2_vpc))
  val f2_predicted_ghist = s2_ghist.update(
    f2_bpd_resp.preds.map(p => p.is_br && p.predicted_pc.valid).asUInt & f2_mask,
    f2_bpd_resp.preds(f2_redirect_idx).taken && f2_do_redirect,
    f2_bpd_resp.preds(f2_redirect_idx).is_br,
    f2_redirect_idx,
    f2_do_redirect,
    s2_vpc,
    false.B,
    false.B)

  val f2_correct_f1_ghist = s1_ghist =/= f2_predicted_ghist && enableGHistStallRepair.B

  //当Icache的响应无效或者F3阶段传来的握手信号没有准备就绪时，
  //需要将该阶段的PC值重定向回F0阶段，重新访问Icache，并且需要清除此时F1阶段中的内容。
  when ((s2_valid && !icache.io.resp.valid) ||
        (s2_valid && icache.io.resp.valid && !f3_ready)) {                   
    s0_valid := (!s2_tlb_resp.ae.inst && !s2_tlb_resp.pf.inst) || s2_is_replay || s2_tlb_miss     
    s0_vpc   := s2_vpc
    s0_is_replay := s2_valid && icache.io.resp.valid
    // When this is not a replay (it queried the BPDs, we should use f3 resp in the replaying s1)
    s0_s1_use_f3_bpd_resp := !s2_is_replay
    s0_ghist := s2_ghist
    s0_tsrc  := s2_tsrc
    f1_clear := true.B
  } .elsewhen (s2_valid && f3_ready) {
    when (s1_valid && s1_vpc === f2_predicted_target && !f2_correct_f1_ghist) {
      // We trust our prediction of what the global history for the next branch should be
      //该阶段还会得到两周期分支预测器的预测结果，需要先判断F2阶段分支预测的目标跳转地址与此时F1阶段的PC值是否一样，
      //预测跳转方向与之前F1阶段的预测是否一样，如果是则证明两个阶段的分支预测器的预测结果相同，不需要进行PC的重定向，
      //只需要更新F2阶段的全局历史寄存器即可；反之，则需要清除F1阶段的内容，将F2阶段预测的目标地址给到F0阶段重定向PC。
      s2_ghist := f2_predicted_ghist
    }
    when ((s1_valid && (s1_vpc =/= f2_predicted_target || f2_correct_f1_ghist)) || !s1_valid) {
      f1_clear := true.B

      s0_valid     := !((s2_tlb_resp.ae.inst || s2_tlb_resp.pf.inst) && !s2_is_replay)
      s0_vpc       := f2_predicted_target
      s0_is_replay := false.B
      s0_ghist     := f2_predicted_ghist
      s2_fsrc      := BSRC_2
      s0_tsrc      := BSRC_2
    }
  }
  s0_replay_bpd_resp := f2_bpd_resp
  s0_replay_resp := s2_tlb_resp
  s0_replay_ppc  := s2_ppc

  // --------------------------------------------------------
  // **** F3 ****
  //在F3阶段出队，主要传递Icache响应的指令、PC、全局历史等信息
  // --------------------------------------------------------
  val f3_clear = WireInit(false.B)
  val f3 = withReset(reset.asBool || f3_clear) {                     //实际上时imem resp queue
    Module(new Queue(new FrontendResp_Security, 1, pipe=true, flow=false)) }                  
    //FrontendResp_Security.bits.data字段包含取到的指令和对应的securitytag

  // Queue up the bpd resp as well, incase f4 backpressures f3
  // This is "flow" because the response (enq) arrives in f3, not f2
  val f3_bpd_resp = withReset(reset.asBool || f3_clear) {
    Module(new Queue(new BranchPredictionBundle, 1, pipe=true, flow=true)) }




  val f4_ready = Wire(Bool())
  f3_ready := f3.io.enq.ready                                                    //由于f3_bpd_resp队列为wire，所以f3的ready为准
  f3.io.enq.valid   := (s2_valid && !f2_clear &&
    (icache.io.resp.valid || ((s2_tlb_resp.ae.inst || s2_tlb_resp.pf.inst) && !s2_tlb_miss))       
    //即s2传过来的东西（不管是icache resp，还是tlb的resp（包括异常，只要不miss））有效
  )
  f3.io.enq.bits.pc := s2_vpc                                                                                                                              //指令的pc
  f3.io.enq.bits.data  := Mux(s2_xcpt, 0.U, icache.io.resp.bits.data)                                                //icache传入的data(包含securitytag)
  f3.io.enq.bits.ghist := s2_ghist                                                                                                                      //全局历史追踪
  f3.io.enq.bits.mask := fetchMask(s2_vpc)                                                                                               //返回该地址的idx（）
  f3.io.enq.bits.xcpt := s2_tlb_resp                                                                                                                //异常传递（只用到了pf和ae）
  f3.io.enq.bits.fsrc := s2_fsrc                                                                                                                           //分支预测器记录
  f3.io.enq.bits.tsrc := s2_tsrc

  // RAS takes a cycle to read
  val ras_read_idx = RegInit(0.U(log2Ceil(nRasEntries).W))
  ras.io.read_idx := ras_read_idx
  when (f3.io.enq.fire) {                            //fire为交火，即valid、ready都为高
    ras_read_idx := f3.io.enq.bits.ghist.ras_idx
    ras.io.read_idx := f3.io.enq.bits.ghist.ras_idx
  }


  // The BPD resp comes in f3
  f3_bpd_resp.io.enq.valid := f3.io.deq.valid && RegNext(f3.io.enq.ready)
  f3_bpd_resp.io.enq.bits  := bpd.io.resp.f3
  when (f3_bpd_resp.io.enq.fire) {
    bpd.io.f3_fire := true.B
  }

  f3.io.deq.ready := f4_ready
  f3_bpd_resp.io.deq.ready := f4_ready


  val f3_imemresp     = f3.io.deq.bits                                                                                                 //f3的输出
  val f3_bank_mask    = bankMask(f3_imemresp.pc)
  // val f3_data         = f3_imemresp.data
  val f3_data         = Cat(f3_imemresp.data(131,68), f3_imemresp.data(63,0))              //指令机器码部分
  val f3_data_tag     = Cat(f3_imemresp.data(135,132), f3_imemresp.data(67,64))    //指令对应的tag部分
  val f3_aligned_pc   = bankAlign(f3_imemresp.pc)                                                                 //pc
  val f3_is_last_bank_in_block = isLastBankInBlock(f3_aligned_pc)                               
  val f3_is_rvc       = Wire(Vec(fetchWidth, Bool()))
  val f3_redirects    = Wire(Vec(fetchWidth, Bool()))
  val f3_targs        = Wire(Vec(fetchWidth, UInt(vaddrBitsExtended.W)))                           //f3的目标地址
  val f3_cfi_types    = Wire(Vec(fetchWidth, UInt(CFI_SZ.W)))
  val f3_shadowed_mask = Wire(Vec(fetchWidth, Bool()))
  val f3_fetch_bundle = Wire(new FetchBundle_Security)
  val f3_mask         = Wire(Vec(fetchWidth, Bool()))
  val f3_br_mask      = Wire(Vec(fetchWidth, Bool()))
  val f3_call_mask    = Wire(Vec(fetchWidth, Bool()))
  val f3_ret_mask     = Wire(Vec(fetchWidth, Bool()))
  val f3_npc_plus4_mask = Wire(Vec(fetchWidth, Bool()))
  val f3_btb_mispredicts = Wire(Vec(fetchWidth, Bool()))
  f3_fetch_bundle.mask := f3_mask.asUInt
  f3_fetch_bundle.br_mask := f3_br_mask.asUInt
  f3_fetch_bundle.pc := f3_imemresp.pc
  f3_fetch_bundle.ftq_idx := 0.U // This gets assigned later
  f3_fetch_bundle.xcpt_pf_if := f3_imemresp.xcpt.pf.inst                          //异常传递
  f3_fetch_bundle.xcpt_ae_if := f3_imemresp.xcpt.ae.inst
  f3_fetch_bundle.fsrc := f3_imemresp.fsrc
  f3_fetch_bundle.tsrc := f3_imemresp.tsrc
  f3_fetch_bundle.shadowed_mask := f3_shadowed_mask

  // Tracks trailing 16b of previous fetch packet
  val f3_prev_half    = Reg(UInt(16.W))                             //记录上一个packet的最后 16 bits
  val f3_prev_half_tag = Reg(UInt(2.W))                         //记录上一个tag_packet的最后2 bits
  // Tracks if last fetchpacket contained a half-inst
  val f3_prev_is_half = RegInit(false.B)                           //记录是否这16 bits 为边缘指令

  require(fetchWidth >= 4) // Logic gets kind of annoying with fetchWidth = 2
  //（至少每次取出64 bit 的指令（对于rvi来说是2条，每个bank各一条））时，注意fetchwidth是对于rvc来说，即fetchwidth = 16
  def isRVC(inst: UInt) = (inst(1,0) =/= 3.U)              //rvc指令判断（低2位不是11）
  var redirect_found = false.B
  var bank_prev_is_half = f3_prev_is_half  //边缘判断（对齐）
  var bank_prev_half    = f3_prev_half          //记录边缘指令
  var bank_prev_half_tag = f3_prev_half_tag   //记录边缘指令的tag
  var last_inst = 0.U(16.W)                                  //记录边缘指令
  var last_inst_tag = 0.U(2.W)                            //记录边缘指令的tag
  for (b <- 0 until nBanks) {                                 //对于每个bank（实际为两个bank，即b=0或1）
    val bank_data  = f3_data((b+1)*bankWidth*16-1, b*bankWidth*16)                //bankwidth = 4，bank_data 64 bit，有两个bank_data(无后缀和_1)
    val bank_data_tag = f3_data_tag((b+1)*bankWidth-1, b*bankWidth)             //securitytag
    val bank_mask  = Wire(Vec(bankWidth, Bool()))      
    val bank_insts = Wire(Vec(bankWidth, UInt(32.W)))                                                 //指令
    val bank_insts_tag = Wire(Vec(bankWidth, UInt(2.W)))                                          //tag

    for (w <- 0 until bankWidth) {                        //对于每个bank中所有指令，w = 0-3（rvc）
      val i = (b * bankWidth) + w                           //i为在整个packet下的第几个指令（i = 0-7（rvc））

      val valid = Wire(Bool())
      val bpu = Module(new BreakpointUnit(nBreakpoints))
      bpu.io.status   := io.cpu.status
      bpu.io.bp       := io.cpu.bp
      bpu.io.ea       := DontCare
      bpu.io.mcontext := io.cpu.mcontext
      bpu.io.scontext := io.cpu.scontext
      
      //快速译码单元，通过快速译码逻辑可以判断指令是否为分支跳转指令，若是分支跳转指令则：
      //对于jal指令，该指令为无条件跳转指令，即一定会发生跳转，且可以通过译码得到直接跳转的地址，
      //将译码得到的地址与之前预测的地址进行比较，若不一致或者之前的预测的跳转方向为不跳则需要重定向PC，
      //刷掉前两个阶段取到的错误指令。对于jalr指令，该指令也为无条件跳转指令，但跳转地址不能通过译码获得，
      //所以只判断跳转方向是否预测正确，跳转地址仍使用预测的跳转地址，若之前的跳转方向预测错误，还是需要重定向PC，
      //刷掉前两个阶段取到的错误指令，最终的跳转地址还需要考虑该指令类型是否是return指令，若是则需要从ras中取出跳转地址，
      //不是return指令才从预测结果中取出。对于branch指令，该类指令为有条件跳转指令，无法通过译码得到跳转方向，
      //因此跳转方向采用分支预测器预测的结果，但可以通过译码得到跳转地址，所以需要判断跳转地址是否预测正确，
      //若跳转地址预测不正确则需要重定向PC，刷掉前两个阶段取到的错误指令。
      //对于jal指令和branch指令，由于它们的跳转地址是以立即数的形式存在指令中，通过译码再加上偏移量即可获得跳转地址，
      //所以在该阶段可以得到正确的跳转地址，与之前的预测结果进行比较，如果不相同则需要更新分支预测器中对应的项。
      
      val brsigs = Wire(new BranchDecodeSignals)                    //分支解码信号（包括对立即数地址的计算，分支方向预测等）
      if (w == 0) {                                                                                         //第一条指令（由于可能为边缘指令，故特殊处理）

        //inst0为边缘32位，inst1为非边缘32位，exp inst0为边缘扩展后rvc，exp inst1为非边缘扩展后rvc

        val inst0 = Cat(bank_data(15,0), f3_prev_half)               //可能存在的边缘指令记录

        val inst0_tag = f3_prev_half_tag                                           //可能存在的边缘指令的tag

        val inst1 = bank_data(31,0)                                                      //第一条指令
        val inst1_tag = bank_data_tag(1, 0)                                     //第一条指令的tag

        val exp_inst0 = ExpandRVC(inst0)                                         //rvc指令扩展，该函数输入最大32bit，对低16位判断是否为rvc并扩展（扩展f3_prev_half）
        val exp_inst1 = ExpandRVC(inst1)                                         //expandrvc函数定义在rvc.scala。(扩展bank_data(15,0))

        val exp_inst0_tag = inst0_tag                                        //tag对应
        val exp_inst1_tag = inst1_tag        

        val pc0 = (f3_aligned_pc + (i << log2Ceil(coreInstBytes)).U - 2.U)    //指令对应的pc（基址+偏移量），-2是为了应对边缘指令
        val pc1 = (f3_aligned_pc + (i << log2Ceil(coreInstBytes)).U)              //

        val bpd_decoder0 = Module(new BranchDecode)                                 //针对两种情况（是否存在边缘指令）的两种可能
        bpd_decoder0.io.inst := exp_inst0
        bpd_decoder0.io.pc   := pc0
        val bpd_decoder1 = Module(new BranchDecode)
        bpd_decoder1.io.inst := exp_inst1
        bpd_decoder1.io.pc   := pc1

        when (bank_prev_is_half) {                                                          //边缘指令（选择inst0相关的）                                       
          bank_insts(w)                := inst0                                                     //w = 0，即bank_insts(0) = inst0

          bank_insts_tag(w)       := inst0_tag
                                                                                                                                              //f3_fetch_bundle为传递给f4的信号bundle
          f3_fetch_bundle.insts(i)     := inst0                                                               //i = 4 * b，即对于packet中第0或4（对应bank0和bank1）
          f3_fetch_bundle.exp_insts(i) := exp_inst0
          f3_fetch_bundle.insts_securitytag(i) := inst0_tag                               //securitytag
          f3_fetch_bundle.exp_insts_securitytag(i) := exp_inst0_tag           //rvc tag
          bpu.io.pc                    := pc0
          brsigs                       := bpd_decoder0.io.out
          f3_fetch_bundle.edge_inst(b) := true.B
          if (b > 0) {                                                                                             //对于bank1
            val inst0b     = Cat(bank_data(15,0), last_inst)                  //last_inst为bank0的可能存在的半条指令（63，48）

            val inst0b_tag = last_inst_tag

            val exp_inst0b = ExpandRVC(inst0b)

            val exp_inst0b_tag = inst0b_tag

            val bpd_decoder0b = Module(new BranchDecode)
            bpd_decoder0b.io.inst := exp_inst0b
            bpd_decoder0b.io.pc   := pc0

            when (f3_bank_mask(b-1)) {                                                   //bank0填充第零项（b = 0）（w = 0）
              bank_insts(w)                := inst0b

              bank_insts_tag(w)       := inst0b_tag

              f3_fetch_bundle.insts(i)     := inst0b
              f3_fetch_bundle.exp_insts(i) := exp_inst0b
              f3_fetch_bundle.insts_securitytag(i) := inst0b_tag                            //securitytag

              f3_fetch_bundle.exp_insts_securitytag(i) := exp_inst0b_tag

              brsigs                       := bpd_decoder0b.io.out
            }
          }
        } .otherwise {                                                                                         //非边缘指令（选择inst1相关的）
          bank_insts(w)                := inst1
 
          bank_insts_tag(w)       := inst1_tag

          f3_fetch_bundle.insts(i)     := inst1
          f3_fetch_bundle.exp_insts(i) := exp_inst1
          f3_fetch_bundle.insts_securitytag(i) := inst1_tag                               //采用此地址，securitytag更新

          f3_fetch_bundle.exp_insts_securitytag(i) := exp_inst1_tag

          bpu.io.pc                    := pc1
          brsigs                       := bpd_decoder1.io.out
          f3_fetch_bundle.edge_inst(b) := false.B
        }
        valid := true.B
      } else {                                                                                                            // w > 0, 除了第一条指令外的指令
        val inst = Wire(UInt(32.W))
        val exp_inst = ExpandRVC(inst)                                                       //rvc

        //val exp_inst_tag = inst_tag                                                              //rvc tag

        val inst_tag = Wire(UInt(2.W))                                                          //securitytag
        val pc = f3_aligned_pc + (i << log2Ceil(coreInstBytes)).U    //指令对应的pc
        val bpd_decoder = Module(new BranchDecode)
        bpd_decoder.io.inst := exp_inst
        bpd_decoder.io.pc   := pc

        bank_insts(w)                := inst                                                             //bank

        bank_insts_tag(w)       := inst_tag              

        f3_fetch_bundle.insts(i)     := inst                                                              //i = 1/2/3                                                
        f3_fetch_bundle.exp_insts(i) := exp_inst
        f3_fetch_bundle.insts_securitytag(i) := inst_tag

        f3_fetch_bundle.exp_insts_securitytag(i) := inst_tag

        bpu.io.pc                    := pc
        brsigs                       := bpd_decoder.io.out
        if (w == 1) {                               //第二条指令
          // Need special case since 0th instruction may carry over the wrap around
          //w为bankwidth（0-3）
          //rvc的tag应该始终与低位的rvc指令对齐
          inst  := bank_data(47,16)       //w=1 
          inst_tag := bank_data_tag(1,0)            //考虑带tag的rvc必定连续偶数个，则带tag的（47，16）若有效则必定只有低16位有效
          valid := bank_prev_is_half || !(bank_mask(0) && !isRVC(bank_insts(0)))                //第一条指令是rvc、边缘指令、无效指令才有效
        } else if (w == bankWidth - 1) {           //w = 3
          inst  := Cat(0.U(16.W), bank_data(bankWidth*16-1,(bankWidth-1)*16))                 //（63，48）
          inst_tag := bank_data_tag (bankWidth-1,bankWidth-2)                                               //rvc tag对应 (3,2)
          valid := !((bank_mask(w-1) && !isRVC(bank_insts(w-1))) ||                                           //本条指令和上一条指令都是rvc才有效
            !isRVC(inst))
        } else {                                                                              //其实就是w=2，记录32bit，但是间隔是16bit
          inst  := bank_data(w*16+32-1,w*16)                //指令，其实就是（63，32）
          inst_tag := bank_data_tag(w+2-1,w)               //securitytag   (3,2)
          valid := !(bank_mask(w-1) && !isRVC(bank_insts(w-1)))                                                //
        }
      }

      f3_is_rvc(i) := isRVC(bank_insts(w))                      //对每个指令都做一个是否为rvc的标记信号


      bank_mask(w) := f3.io.deq.valid && f3_imemresp.mask(i) && valid && !redirect_found     //mask(i)为啥能这样用（mask第i位就能区分第几个指令吗？）
      f3_mask  (i) := f3.io.deq.valid && f3_imemresp.mask(i) && valid && !redirect_found            //
      f3_targs (i) := Mux(brsigs.cfi_type === CFI_JALR,                                                                                   
        f3_bpd_resp.io.deq.bits.preds(i).predicted_pc.bits,
        brsigs.target)                   

        //f3的目标（如果时jalr，由于地址存在ras里，故采用bpd返回的，如果不是（即为branch或jal，则直接根据计算得到））

      // Flush BTB entries for JALs if we mispredict the target                                 
      //该指令为无条件跳转指令，即一定会发生跳转，且可以通过译码得到直接跳转的地址，将译码得到的地址与之前预测的地址进行比较，
      //若不一致或者之前的预测的跳转方向为不跳则需要重定向PC，刷掉前两个阶段取到的错误指令。
      //如果jal预测错误，则冲刷btb
      f3_btb_mispredicts(i) := (brsigs.cfi_type === CFI_JAL && valid &&             
        f3_bpd_resp.io.deq.bits.preds(i).predicted_pc.valid &&
        (f3_bpd_resp.io.deq.bits.preds(i).predicted_pc.bits =/= brsigs.target)              //判断译码后的地址（后者）和预测地址是否一致
      )

       //pc+4的mask（若该指令位rvc，则对应mask（i）=0，计算下一条指令的地址为该rvc指令的pc+2）
      f3_npc_plus4_mask(i) := (if (w == 0) {        //加4标志信号                                                                            
        !f3_is_rvc(i) && !bank_prev_is_half
      } else {
        !f3_is_rvc(i)
      })
      val offset_from_aligned_pc = (                                                     //偏移量
        (i << 1).U((log2Ceil(icBlockBytes)+1).W) +
        brsigs.sfb_offset.bits -
        Mux(bank_prev_is_half && (w == 0).B, 2.U, 0.U)
      )
      val lower_mask = Wire(UInt((2*fetchWidth).W))                  //
      val upper_mask = Wire(UInt((2*fetchWidth).W))
      lower_mask := UIntToOH(i.U)
      upper_mask := UIntToOH(offset_from_aligned_pc(log2Ceil(fetchBytes)+1,1)) << Mux(f3_is_last_bank_in_block, bankWidth.U, 0.U)

      f3_fetch_bundle.sfbs(i) := (
        f3_mask(i) &&
        brsigs.sfb_offset.valid &&
        (offset_from_aligned_pc <= Mux(f3_is_last_bank_in_block, (fetchBytes+bankBytes).U,(2*fetchBytes).U))
      )
      f3_fetch_bundle.sfb_masks(i)       := ~MaskLower(lower_mask) & ~MaskUpper(upper_mask)
      f3_fetch_bundle.shadowable_mask(i) := (!(f3_fetch_bundle.xcpt_pf_if || f3_fetch_bundle.xcpt_ae_if || bpu.io.debug_if || bpu.io.xcpt_if) &&
                                             f3_bank_mask(b) &&
                                             (brsigs.shadowable || !f3_mask(i)))
      f3_fetch_bundle.sfb_dests(i)       := offset_from_aligned_pc

      // Redirect if
      //重定向
      //  1) its a JAL/JALR (unconditional)无条件转移指令
      //  2) the BPD believes this is a branch and says we should take it ，bpd认为应该跳转
      f3_redirects(i)    := f3_mask(i) && (         //重定向mask（即该位对应的指令是否为上述）
        brsigs.cfi_type === CFI_JAL || brsigs.cfi_type === CFI_JALR ||
        (brsigs.cfi_type === CFI_BR && f3_bpd_resp.io.deq.bits.preds(i).taken && useBPD.B)
      )

      f3_br_mask(i)   := f3_mask(i) && brsigs.cfi_type === CFI_BR
      f3_cfi_types(i) := brsigs.cfi_type
      f3_call_mask(i) := brsigs.is_call
      f3_ret_mask(i)  := brsigs.is_ret

      f3_fetch_bundle.bp_debug_if_oh(i) := bpu.io.debug_if
      f3_fetch_bundle.bp_xcpt_if_oh (i) := bpu.io.xcpt_if

      redirect_found = redirect_found || f3_redirects(i)
    }
    last_inst = bank_insts(bankWidth-1)(15,0)                             //记录该bank的可能存在的最后半条指令（63，48）(可能要和另一个bank拼，但都是这一个packet的)

    last_inst_tag = bank_insts_tag(bankWidth-1)                      //(3,2)

    bank_prev_is_half = Mux(f3_bank_mask(b),                        //判断边缘指令（可能是bank之间拼接（bank_mask （1）= 1），也可能是packet之间拼接（bank_mask （0）= 1））
      (!(bank_mask(bankWidth-2) && !isRVC(bank_insts(bankWidth-2))) && !isRVC(last_inst)),
      bank_prev_is_half)
    bank_prev_half    = Mux(f3_bank_mask(b),                          //
      last_inst(15,0),
      bank_prev_half)

    bank_prev_half_tag = Mux(f3_bank_mask(b),                    //
      last_inst_tag,
      bank_prev_half_tag)

  }

  f3_fetch_bundle.cfi_type      := f3_cfi_types(f3_fetch_bundle.cfi_idx.bits)
  f3_fetch_bundle.cfi_is_call   := f3_call_mask(f3_fetch_bundle.cfi_idx.bits)
  f3_fetch_bundle.cfi_is_ret    := f3_ret_mask (f3_fetch_bundle.cfi_idx.bits)
  f3_fetch_bundle.cfi_npc_plus4 := f3_npc_plus4_mask(f3_fetch_bundle.cfi_idx.bits)

  f3_fetch_bundle.ghist    := f3.io.deq.bits.ghist
  f3_fetch_bundle.lhist    := f3_bpd_resp.io.deq.bits.lhist
  f3_fetch_bundle.bpd_meta := f3_bpd_resp.io.deq.bits.meta

  f3_fetch_bundle.end_half.valid := bank_prev_is_half
  f3_fetch_bundle.end_half.bits  := bank_prev_half

  when (f3.io.deq.fire) {
    f3_prev_is_half := bank_prev_is_half
    f3_prev_half    := bank_prev_half

    f3_prev_half_tag    := bank_prev_half_tag           //边缘指令和tag的记录

    assert(f3_bpd_resp.io.deq.bits.pc === f3_fetch_bundle.pc)
  }

  when (f3_clear) {
    f3_prev_is_half := false.B
  }

 //PriorityEncoder返回输入位向量的最低有效高位的位位置，例如，4b'0110，返回1.U

  f3_fetch_bundle.cfi_idx.valid := f3_redirects.reduce(_||_)
  f3_fetch_bundle.cfi_idx.bits  := PriorityEncoder(f3_redirects)   

  f3_fetch_bundle.ras_top := ras.io.read_addr
  // Redirect earlier stages only if the later stage
  // can consume this packet

  val f3_predicted_target = Mux(f3_redirects.reduce(_||_),           //如果f3这packet指令中有导致重定向的
    Mux(f3_fetch_bundle.cfi_is_ret && useBPD.B && useRAS.B,  //
      ras.io.read_addr,                                                                                      //如果使用ras（jalr）
      f3_targs(PriorityEncoder(f3_redirects))                                         //如果jal和branch
    ),
    nextFetch(f3_fetch_bundle.pc)
  )

  f3_fetch_bundle.next_pc       := f3_predicted_target
  val f3_predicted_ghist = f3_fetch_bundle.ghist.update(
    f3_fetch_bundle.br_mask,
    f3_fetch_bundle.cfi_idx.valid,
    f3_fetch_bundle.br_mask(f3_fetch_bundle.cfi_idx.bits),
    f3_fetch_bundle.cfi_idx.bits,
    f3_fetch_bundle.cfi_idx.valid,
    f3_fetch_bundle.pc,
    f3_fetch_bundle.cfi_is_call,
    f3_fetch_bundle.cfi_is_ret
  )


  ras.io.write_valid := false.B
  ras.io.write_addr  := f3_aligned_pc + (f3_fetch_bundle.cfi_idx.bits << 1) + Mux(
    f3_fetch_bundle.cfi_npc_plus4, 4.U, 2.U)
  ras.io.write_idx   := WrapInc(f3_fetch_bundle.ghist.ras_idx, nRasEntries)


  val f3_correct_f1_ghist = s1_ghist =/= f3_predicted_ghist && enableGHistStallRepair.B
  val f3_correct_f2_ghist = s2_ghist =/= f3_predicted_ghist && enableGHistStallRepair.B

  when (f3.io.deq.valid && f4_ready) {
    when (f3_fetch_bundle.cfi_is_call && f3_fetch_bundle.cfi_idx.valid) {
      ras.io.write_valid := true.B
    }
    when (f3_redirects.reduce(_||_)) {        //f3某一条指令出现重定向情况
      f3_prev_is_half := false.B
    }
    when (s2_valid && s2_vpc === f3_predicted_target && !f3_correct_f2_ghist) {
      f3.io.enq.bits.ghist := f3_predicted_ghist
    } .elsewhen (!s2_valid && s1_valid && s1_vpc === f3_predicted_target && !f3_correct_f1_ghist) {
      s2_ghist := f3_predicted_ghist                     //s2全局历史重置
    } .elsewhen (( s2_valid &&  (s2_vpc =/= f3_predicted_target || f3_correct_f2_ghist)) ||
          (!s2_valid &&  s1_valid && (s1_vpc =/= f3_predicted_target || f3_correct_f1_ghist)) ||
          (!s2_valid && !s1_valid)) {
      f2_clear := true.B             //f2重置
      f1_clear := true.B             //f1重置

      s0_valid     := !(f3_fetch_bundle.xcpt_pf_if || f3_fetch_bundle.xcpt_ae_if)     //出现异常
      s0_vpc       := f3_predicted_target                                                                                      //重定向
      s0_is_replay := false.B
      s0_ghist     := f3_predicted_ghist                                                                                        //全局历史重置
      s0_tsrc      := BSRC_3                                                                                                                //该指令的分支预测来自于bpd

      f3_fetch_bundle.fsrc := BSRC_3                                                                                         //同上
    }
  }

  // When f3 finds a btb mispredict, queue up a bpd correction update
  //如果f3发现btb预测错误，则更新btb
  val f4_btb_corrections = Module(new Queue(new BranchPredictionUpdate, 2))
  f4_btb_corrections.io.enq.valid := f3.io.deq.fire && f3_btb_mispredicts.reduce(_||_) && enableBTBFastRepair.B
  f4_btb_corrections.io.enq.bits  := DontCare
  f4_btb_corrections.io.enq.bits.is_mispredict_update := false.B
  f4_btb_corrections.io.enq.bits.is_repair_update     := false.B
  f4_btb_corrections.io.enq.bits.btb_mispredicts      := f3_btb_mispredicts.asUInt
  f4_btb_corrections.io.enq.bits.pc                   := f3_fetch_bundle.pc
  f4_btb_corrections.io.enq.bits.ghist                := f3_fetch_bundle.ghist
  f4_btb_corrections.io.enq.bits.lhist                := f3_fetch_bundle.lhist
  f4_btb_corrections.io.enq.bits.meta                 := f3_fetch_bundle.bpd_meta


  // -------------------------------------------------------
  // **** F4 ****
  // -------------------------------------------------------
  val f4_clear = WireInit(false.B)
  val f4 = withReset(reset.asBool || f4_clear) {
    Module(new Queue(new FetchBundle_Security, 1, pipe=true, flow=false))}

  val fb  = Module(new FetchBuffer_Security)                     //fetch buffer
  val ftq = Module(new FetchTargetQueue)

  // When we mispredict, we need to repair

  // Deal with sfbs
  val f4_shadowable_masks = VecInit((0 until fetchWidth) map { i =>
     f4.io.deq.bits.shadowable_mask.asUInt |
    ~f4.io.deq.bits.sfb_masks(i)(fetchWidth-1,0)
  })
  val f3_shadowable_masks = VecInit((0 until fetchWidth) map { i =>
    Mux(f4.io.enq.valid, f4.io.enq.bits.shadowable_mask.asUInt, 0.U) |
    ~f4.io.deq.bits.sfb_masks(i)(2*fetchWidth-1,fetchWidth)
  })
  val f4_sfbs = VecInit((0 until fetchWidth) map { i =>
    enableSFBOpt.B &&
    ((~f4_shadowable_masks(i) === 0.U) &&
     (~f3_shadowable_masks(i) === 0.U) &&
     f4.io.deq.bits.sfbs(i) &&
     !(f4.io.deq.bits.cfi_idx.valid && f4.io.deq.bits.cfi_idx.bits === i.U) &&
      Mux(f4.io.deq.bits.sfb_dests(i) === 0.U,
        !bank_prev_is_half,
      Mux(f4.io.deq.bits.sfb_dests(i) === fetchBytes.U,
        !f4.io.deq.bits.end_half.valid,
        true.B)
      )

     )
  })
  val f4_sfb_valid = f4_sfbs.reduce(_||_) && f4.io.deq.valid
  val f4_sfb_idx   = PriorityEncoder(f4_sfbs)
  val f4_sfb_mask  = f4.io.deq.bits.sfb_masks(f4_sfb_idx)
  // If we have a SFB, wait for next fetch to be available in f3
  val f4_delay     = (                                           
    f4.io.deq.bits.sfbs.reduce(_||_) &&
    !f4.io.deq.bits.cfi_idx.valid &&
    !f4.io.enq.valid &&
    !f4.io.deq.bits.xcpt_pf_if &&                                           //
    !f4.io.deq.bits.xcpt_ae_if
  )
  when (f4_sfb_valid) {
    f3_shadowed_mask := f4_sfb_mask(2*fetchWidth-1,fetchWidth).asBools
  } .otherwise {
    f3_shadowed_mask := VecInit(0.U(fetchWidth.W).asBools)
  }

  f4_ready := f4.io.enq.ready
  f4.io.enq.valid := f3.io.deq.valid && !f3_clear
  f4.io.enq.bits  := f3_fetch_bundle
  f4.io.deq.ready := fb.io.enq.ready && ftq.io.enq.ready && !f4_delay              //

  fb.io.enq.valid := f4.io.deq.valid && ftq.io.enq.ready && !f4_delay
  fb.io.enq.bits  := f4.io.deq.bits
  fb.io.enq.bits.ftq_idx := ftq.io.enq_idx
  fb.io.enq.bits.sfbs    := Mux(f4_sfb_valid, UIntToOH(f4_sfb_idx), 0.U(fetchWidth.W)).asBools
  fb.io.enq.bits.shadowed_mask := (
    Mux(f4_sfb_valid, f4_sfb_mask(fetchWidth-1,0), 0.U(fetchWidth.W)) |
    f4.io.deq.bits.shadowed_mask.asUInt
  ).asBools


  ftq.io.enq.valid          := f4.io.deq.valid && fb.io.enq.ready && !f4_delay
  ftq.io.enq.bits           := f4.io.deq.bits

  val bpd_update_arbiter = Module(new Arbiter(new BranchPredictionUpdate, 2))
  bpd_update_arbiter.io.in(0).valid := ftq.io.bpdupdate.valid
  bpd_update_arbiter.io.in(0).bits  := ftq.io.bpdupdate.bits
  assert(bpd_update_arbiter.io.in(0).ready)
  bpd_update_arbiter.io.in(1) <> f4_btb_corrections.io.deq
  bpd.io.update := bpd_update_arbiter.io.out
  bpd_update_arbiter.io.out.ready := true.B

  when (ftq.io.ras_update && enableRasTopRepair.B) {
    ras.io.write_valid := true.B
    ras.io.write_idx   := ftq.io.ras_update_idx
    ras.io.write_addr  := ftq.io.ras_update_pc
  }


  // -------------------------------------------------------
  // **** To Core (F5) ****
  // -------------------------------------------------------

  io.cpu.fetchpacket <> fb.io.deq                     //fetch buffer的输出作为fetch packet
  io.cpu.get_pc <> ftq.io.get_ftq_pc               //
  ftq.io.deq := io.cpu.commit
  ftq.io.brupdate := io.cpu.brupdate

  ftq.io.redirect.valid   := io.cpu.redirect_val
  ftq.io.redirect.bits    := io.cpu.redirect_ftq_idx
  fb.io.clear := false.B

  when (io.cpu.sfence.valid) {
    fb.io.clear := true.B
    f4_clear    := true.B
    f3_clear    := true.B
    f2_clear    := true.B
    f1_clear    := true.B

    s0_valid     := false.B
    s0_vpc       := io.cpu.sfence.bits.addr
    s0_is_replay := false.B
    s0_is_sfence := true.B

  }.elsewhen (io.cpu.redirect_flush) {
    fb.io.clear := true.B
    f4_clear    := true.B
    f3_clear    := true.B
    f2_clear    := true.B
    f1_clear    := true.B

    f3_prev_is_half := false.B

    s0_valid     := io.cpu.redirect_val
    s0_vpc       := io.cpu.redirect_pc
    s0_ghist     := io.cpu.redirect_ghist
    s0_tsrc      := BSRC_C
    s0_is_replay := false.B

    ftq.io.redirect.valid := io.cpu.redirect_val
    ftq.io.redirect.bits  := io.cpu.redirect_ftq_idx
  }

  ftq.io.debug_ftq_idx := io.cpu.debug_ftq_idx
  io.cpu.debug_fetch_pc := ftq.io.debug_fetch_pc


  override def toString: String =
    (BoomCoreStringPrefix("====Overall Frontend Params====") + "\n"
    + icache.toString + bpd.toString)
}

//Security_Tag
/**
 * Bundle passed into the FetchBuffer and used to combine multiple
 * relevant signals together.
 * 
 * fb's input
 * 
 */
class FetchBundle_Security(implicit p: Parameters) extends BoomBundle
  with HasBoomFrontendParameters
{
  val pc            = UInt(vaddrBitsExtended.W)

  //val pc_securitytag = UInt(2.W)

  val next_pc       = UInt(vaddrBitsExtended.W)
  val edge_inst     = Vec(nBanks, Bool()) // True if 1st instruction in this bundle is pc - 2
  val insts         = Vec(fetchWidth, Bits(32.W))
  val exp_insts     = Vec(fetchWidth, Bits(32.W))

  val exp_insts_securitytag  = Vec(fetchWidth, Bits(2.W))    //rvc的tag

  val insts_securitytag     = Vec(fetchWidth, Bits(2.W))          //指令的securitytag

  // Information for sfb folding
  // NOTE: This IS NOT equivalent to uop.pc_lob, that gets calculated in the FB
  val sfbs                 = Vec(fetchWidth, Bool())
  val sfb_masks            = Vec(fetchWidth, UInt((2*fetchWidth).W))
  val sfb_dests            = Vec(fetchWidth, UInt((1+log2Ceil(fetchBytes)).W))
  val shadowable_mask      = Vec(fetchWidth, Bool())
  val shadowed_mask        = Vec(fetchWidth, Bool())

  val cfi_idx       = Valid(UInt(log2Ceil(fetchWidth).W))
  val cfi_type      = UInt(CFI_SZ.W)
  val cfi_is_call   = Bool()
  val cfi_is_ret    = Bool()
  val cfi_npc_plus4 = Bool()

  val ras_top       = UInt(vaddrBitsExtended.W)

  val ftq_idx       = UInt(log2Ceil(ftqSz).W)
  val mask          = UInt(fetchWidth.W) // mark which words are valid instructions

  val br_mask       = UInt(fetchWidth.W)

  val ghist         = new GlobalHistory
  val lhist         = Vec(nBanks, UInt(localHistoryLength.W))

  val xcpt_pf_if    = Bool() // I-TLB miss (instruction fetch fault).
  val xcpt_ae_if    = Bool() // Access exception.

  val bp_debug_if_oh= Vec(fetchWidth, Bool())
  val bp_xcpt_if_oh = Vec(fetchWidth, Bool())

  val end_half      = Valid(UInt(16.W))


  val bpd_meta      = Vec(nBanks, UInt())

  // Source of the prediction from this bundle
  val fsrc    = UInt(BSRC_SZ.W)
  // Source of the prediction to this bundle
  val tsrc    = UInt(BSRC_SZ.W)
}

/**
 * IO for the BOOM Frontend to/from the CPU
 */
class BoomFrontendIO_Security(implicit p: Parameters) extends BoomBundle        //io定义
{
  // Give the backend a packet of instructions. 给后端一个packet的指令
  val fetchpacket       = Flipped(new DecoupledIO(new FetchBufferResp_Security))    //每个指令的microop和securitytag

  // 1 for xcpt/jalr/auipc/flush
  val get_pc            = Flipped(Vec(2, new GetPCFromFtqIO()))                                                 //
  val debug_ftq_idx     = Output(Vec(coreWidth, UInt(log2Ceil(ftqSz).W)))                       //ftq的debug索引
  val debug_fetch_pc    = Input(Vec(coreWidth, UInt(vaddrBitsExtended.W)))

  //val debug_fetch_pc_securitytag    = Input(Vec(coreWidth, UInt(2.W)))

  // Breakpoint info
  val status            = Output(new MStatus)
  val bp                = Output(Vec(nBreakpoints, new BP))
  val mcontext          = Output(UInt(coreParams.mcontextWidth.W))
  val scontext          = Output(UInt(coreParams.scontextWidth.W))

  val sfence = Valid(new SFenceReq)

  val brupdate          = Output(new BrUpdateInfo)

  // Redirects change the PC
  val redirect_flush   = Output(Bool()) // Flush and hang the frontend?   前端冲刷
  val redirect_val     = Output(Bool()) // Redirect the frontend?
  val redirect_pc      = Output(UInt()) // Where do we redirect to?
  val redirect_ftq_idx = Output(UInt()) // Which ftq entry should we reset to?
  val redirect_ghist   = Output(new GlobalHistory) // What are we setting as the global history?

  val commit = Valid(UInt(ftqSz.W))

  val flush_icache = Output(Bool())       //icache冲刷？

  val perf = Input(new FrontendPerfEvents)
}

/**
 * Bundle wrapping the IO for the Frontend as a whole
 *
 * @param outer top level Frontend class
 */
class BoomFrontendBundle_Security(val outer: BoomFrontend) extends CoreBundle()(outer.p)
{
  val cpu = Flipped(new BoomFrontendIO_Security())    //包含了securitytag
  val ptw = new TLBPTWIO()                                                        //查找TLB
  val errors = new ICacheErrors
}

class FrontendResp_Security(implicit p: Parameters) extends BoomBundle()(p) {             //加了securitytag的前端输出
  val pc = UInt(vaddrBitsExtended.W)  // ID stage PC                                                                          //给解码级的pc

//val pc_securitytag = UInt(2.W) 

  val data = UInt(((fetchWidth * coreInstBits + fetchWidth * coreInstBits / 16)).W)                //取到的指令和指令的securitytag
  //取到的数据（其实就是取到的指令机器码，coreinstbits为单个指令的位宽，fetchwidth为一次性取几条）
  val mask = UInt(fetchWidth.W)                                                                                                                  //独热掩码，每一位对应一条指令
  val xcpt = new FrontendExceptions                                                                                                         //异常,位于rocket中
  val ghist = new GlobalHistory                                                                                                                     //全局历史

  // fsrc provides the prediction FROM a branch in this packet
  // tsrc provides the prediction TO this packet
  val fsrc = UInt(BSRC_SZ.W)                                                                                                                          //分支预测的周期数
  val tsrc = UInt(BSRC_SZ.W)
}
