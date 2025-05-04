//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// MicroOp
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.common

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters

import boom.exu.FUConstants

/**
 * Extension to BoomBundle to add a MicroOp
 */
abstract trait HasBoomUOP extends BoomBundle
{
  val uop = new MicroOp() 

  //val uop_securitytag = UInt(2.W) 

}

/**
 * MicroOp passing through the pipeline                              //定义了通过流水线的微码（我理解为传递控制信息的信号，不包括数据）
 */
class MicroOp(implicit p: Parameters) extends BoomBundle                        //微码的定义
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
  with freechips.rocketchip.rocket.constants.ScalarOpConstants
{
  val uopc             = UInt(UOPC_SZ.W)       // micro-op code，指令机器码的低7位

  val uopc_securitytag = UInt(2.W)         //微码对应的指令的securitytag

  val inst             = UInt(32.W)                         //32位指令码

  val inst_securitytag = UInt(2.W)              //指令securitytag

  val debug_inst       = UInt(32.W)                //inst和debuginst有什么区别？

  //val debug_inst_securitytag = UInt(2.W)

  val is_rvc           = Bool()                                //是否为rvc（16位压缩指令）
  val debug_pc         = UInt(coreMaxAddrBits.W)       //和pc有什么区别（）

  val debug_pc_securitytag = UInt(2.W)

  val iq_type          = UInt(IQT_SZ.W)        // which issue unit do we use?
  val fu_code          = UInt(FUConstants.FUC_SZ.W) // which functional unit do we use?
  val ctrl             = new CtrlSignals

  // What is the next state of this uop in the issue window? useful
  // for the compacting queue.
  val iw_state         = UInt(2.W)
  // Has operand 1 or 2 been waken speculatively by a load?
  // Only integer operands are speculaively woken up,
  // so we can ignore p3.
  val iw_p1_poisoned   = Bool()
  val iw_p2_poisoned   = Bool()

  val is_br            = Bool()                      // is this micro-op a (branch) vs a regular PC+4 inst?
  val is_jalr          = Bool()                      // is this a jump? (jal or jalr)
  val is_jal           = Bool()                      // is this a JAL (doesn't include JR)? used for branch unit
  val is_sfb           = Bool()                      // is this a sfb or in the shadow of a sfb

  val br_mask          = UInt(maxBrCount.W)  // which branches are we being speculated under?
  val br_tag           = UInt(brTagSz.W)

  // Index into FTQ to figure out our fetch PC.
  val ftq_idx          = UInt(log2Ceil(ftqSz).W)                              //
  // This inst straddles two fetch packets
  val edge_inst        = Bool()          //该指令是否为边缘指令（横跨两个取到的packet）
  // Low-order bits of our own PC. Combine with ftq[ftq_idx] to get PC.
  // Aligned to a cache-line size, as that is the greater fetch granularity.
  // TODO: Shouldn't this be aligned to fetch-width size?
  val pc_lob           = UInt(log2Ceil(icBlockBytes).W)              //pc的低6位(用于索引？)

  // Was this a branch that was predicted taken?   //分支预测方向
  val taken            = Bool()

  val imm_packed       = UInt(LONGEST_IMM_SZ.W) // densely pack the imm in decode...
                                              // then translate and sign-extend in execute

  val imm_packed_securitytag       = UInt(2.W)  //立即数的securitytag，即当前pc的securitytag

  val csr_addr         = UInt(CSR_ADDR_SZ.W)    // only used for critical path reasons in Exe

  val csr_addr_securitytag         = UInt(2.W) 
 
  val rob_idx          = UInt(robAddrSz.W)
  val ldq_idx          = UInt(ldqAddrSz.W)
  val stq_idx          = UInt(stqAddrSz.W)
  val rxq_idx          = UInt(log2Ceil(numRxqEntries).W)
  val pdst             = UInt(maxPregSz.W)               //物理目的寄存器
  val prs1             = UInt(maxPregSz.W)               //物理源寄存器123
  val prs2             = UInt(maxPregSz.W)
  val prs3             = UInt(maxPregSz.W)
  val ppred            = UInt(log2Ceil(ftqSz).W)

  val prs1_busy        = Bool()
  val prs2_busy        = Bool()
  val prs3_busy        = Bool()
  val ppred_busy       = Bool()
  val stale_pdst       = UInt(maxPregSz.W)
  val exception        = Bool()
  val exc_cause        = UInt(xLen.W)          // TODO compress this down, xlen is insanity
  val bypassable       = Bool()                      // can we bypass ALU results? (doesn't include loads, csr, etc...)
  val mem_cmd          = UInt(M_SZ.W)          // sync primitives/cache flushes
  val mem_size         = UInt(2.W)
  val mem_signed       = Bool()
  val is_fence         = Bool()
  val is_fencei        = Bool()
  val is_amo           = Bool()
  val uses_ldq         = Bool()
  val uses_stq         = Bool()
  val is_sys_pc2epc    = Bool()                      // Is a ECall or Breakpoint -- both set EPC to PC.
  val is_unique        = Bool()                      // only allow this instruction in the pipeline, wait for STQ to
                                                     // drain, clear fetcha fter it (tell ROB to un-ready until empty)
  val flush_on_commit  = Bool()                      // some instructions need to flush the pipeline behind them

  // Preditation
  def is_sfb_br        = is_br && is_sfb && enableSFBOpt.B // Does this write a predicate
  def is_sfb_shadow    = !is_br && is_sfb && enableSFBOpt.B // Is this predicated
  val ldst_is_rs1      = Bool() // If this is set and we are predicated off, copy rs1 to dst,
                                // else copy rs2 to dst

  // logical specifiers (only used in Decode->Rename), except rollback (ldst)
  val ldst             = UInt(lregSz.W)
  val lrs1             = UInt(lregSz.W)
  val lrs2             = UInt(lregSz.W)
  val lrs3             = UInt(lregSz.W)

  val ldst_val         = Bool()              // is there a destination? invalid for stores, rd==x0, etc.
  val dst_rtype        = UInt(2.W)
  val lrs1_rtype       = UInt(2.W)
  val lrs2_rtype       = UInt(2.W)    //逻辑操作数类型？
  val frs3_en          = Bool()

  // floating point information
  val fp_val           = Bool()             // is a floating-point instruction (F- or D-extension)?
                                            // If it's non-ld/st it will write back exception bits to the fcsr.
  val fp_single        = Bool()             // single-precision floating point instruction (F-extension)

  // frontend exception information   前端异常信息（好像不需要cause就行）
  val xcpt_pf_if       = Bool()             // I-TLB page fault.     指令TLB页表异常
  val xcpt_ae_if       = Bool()             // I$ access exception.      icache存取异常
  val xcpt_ma_if       = Bool()             // Misaligned fetch (jal/brjumping to misaligned addr).       地址未对齐
  val bp_debug_if      = Bool()             // Breakpoint           断点（用来debug的）
  val bp_xcpt_if       = Bool()             // Breakpoint

  val inst_check_unpass          =  Bool()          //指令tag安全检查未通过

  val data_check_unpass          =  Bool()         //数据tag安全检查未通过




  // What prediction structure provides the prediction FROM this op
  val debug_fsrc       = UInt(BSRC_SZ.W)
  // What prediction structure provides the prediction TO this op
  val  debug_tsrc      = UInt(BSRC_SZ.W)

  // Do we allocate a branch tag for this?
  // SFB branches don't get a mask, they get a predicate bit
  def allocate_brtag   = (is_br && !is_sfb) || is_jalr

  // Does this register write-back
  def rf_wen           = dst_rtype =/= RT_X

  // Is it possible for this uop to misspeculate, preventing the commit of subsequent uops?
  //不安全指处于推测执行且能对外显示的，load store br jal指令
  def unsafe           = uses_ldq || (uses_stq && !is_fence) || is_br || is_jalr

  def fu_code_is(_fu: UInt) = (fu_code & _fu) =/= 0.U
}

/**
 * Control signals within a MicroOp
 *
 * TODO REFACTOR this, as this should no longer be true, as bypass occurs in stage before branch resolution
 */
class CtrlSignals extends Bundle()
{
  val br_type     = UInt(BR_N.getWidth.W)
  val op1_sel     = UInt(OP1_X.getWidth.W)
  val op2_sel     = UInt(OP2_X.getWidth.W)
  val imm_sel     = UInt(IS_X.getWidth.W)
  val op_fcn      = UInt(freechips.rocketchip.rocket.ALU.SZ_ALU_FN.W)
  val fcn_dw      = Bool()
  val csr_cmd     = UInt(freechips.rocketchip.rocket.CSR.SZ.W)
  val is_load     = Bool()   // will invoke TLB address lookup
  val is_sta      = Bool()   // will invoke TLB address lookup
  val is_std      = Bool()
}



