//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Issue Slot Logic
//发射槽
//--------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Note: stores (and AMOs) are "broken down" into 2 uops, but stored within a single issue-slot.
// TODO XXX make a separate issueSlot for MemoryIssueSlots, and only they break apart stores.
// TODO Disable ldspec for FP queue.

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters

import boom.common._
import boom.util._
import FUConstants._

/**
 * IO bundle to interact with Issue slot
 *
 * @param numWakeupPorts number of wakeup ports for the slot
 */
class IssueSlotIO(val numWakeupPorts: Int)(implicit p: Parameters) extends BoomBundle
{
  val valid         = Output(Bool())
  val will_be_valid = Output(Bool()) // TODO code review, do we need this signal so explicitely?
  val request       = Output(Bool())
  val request_hp    = Output(Bool())
  val grant         = Input(Bool())

  val brupdate        = Input(new BrUpdateInfo())
  val kill          = Input(Bool()) // pipeline flush
  val clear         = Input(Bool()) // entry being moved elsewhere (not mutually exclusive with grant)
  val ldspec_miss   = Input(Bool()) // Previous cycle's speculative load wakeup was mispredicted.

  val wakeup_ports  = Flipped(Vec(numWakeupPorts, Valid(new IqWakeup(maxPregSz))))
  val pred_wakeup_port = Flipped(Valid(UInt(log2Ceil(ftqSz).W)))
  val spec_ld_wakeup = Flipped(Vec(memWidth, Valid(UInt(width=maxPregSz.W))))
  val in_uop        = Flipped(Valid(new MicroOp())) // if valid, this WILL overwrite an entry!写入发射槽的uop

 val in_uop_securitytag        = Input(UInt(2.W)) //写入发射槽的securitytag

  val out_uop   = Output(new MicroOp()) // the updated slot uop; will be shifted upwards in a collasping queue.更新的uop，会向上移动

 val out_uop_securitytag   = Output(UInt(2.W)) //更新的securitytag 

  val uop           = Output(new MicroOp()) // the current Slot's uop. Sent down the pipeline when issued. 当前发射槽的uop，发射时会被发送到流水线

  val uop_securitytag           = Output(UInt(2.W))

  val debug = {
    val result = new Bundle {
      val p1 = Bool()
      val p2 = Bool()
      val p3 = Bool()
      val ppred = Bool()
      val state = UInt(width=2.W)
    }
    Output(result)
  }
}

/**
 * Single issue slot. Holds a uop within the issue queue
 *
 * @param numWakeupPorts number of wakeup ports
 */
class IssueSlot(val numWakeupPorts: Int)(implicit p: Parameters)
  extends BoomModule
  with IssueUnitConstants
{
  val io = IO(new IssueSlotIO(numWakeupPorts))

  // slot invalid?
  // slot is valid, holding 1 uop
  // slot is valid, holds 2 uops (like a store)
  def is_invalid = state === s_invalid
  def is_valid = state =/= s_invalid

  val next_state      = Wire(UInt()) // the next state of this slot (which might then get moved to a new slot)
  val next_uopc       = Wire(UInt()) // the next uopc of this slot (which might then get moved to a new slot)

  val next_uopc_securitytag       = Wire(UInt(2.W))             //uopc为inst低七位

  val next_lrs1_rtype = Wire(UInt()) // the next reg type of this slot (which might then get moved to a new slot)
  val next_lrs2_rtype = Wire(UInt()) // the next reg type of this slot (which might then get moved to a new slot)

  val state = RegInit(s_invalid)
  val p1    = RegInit(false.B)              //指示指令中源操作数是否已经准备好，如果都准备好则可以进入发射
  val p2    = RegInit(false.B)
  val p3    = RegInit(false.B)
  val ppred = RegInit(false.B)

  // Poison if woken up by speculative load.
  // Poison lasts 1 cycle (as ldMiss will come on the next cycle).
  // SO if poisoned is true, set it to false!
  //如果是推测load导致的唤醒，那么会进入1个cycle的中毒状态
  val p1_poisoned = RegInit(false.B)
  val p2_poisoned = RegInit(false.B)
  p1_poisoned := false.B
  p2_poisoned := false.B
  val next_p1_poisoned = Mux(io.in_uop.valid, io.in_uop.bits.iw_p1_poisoned, p1_poisoned)
  val next_p2_poisoned = Mux(io.in_uop.valid, io.in_uop.bits.iw_p2_poisoned, p2_poisoned)

  val slot_uop = RegInit(NullMicroOp)             //nullmicroop为nop指令，此处为设置默认值

  val slot_uop_securitytag = RegInit(0.U(2.W))            //默认值为0

  val next_uop = Mux(io.in_uop.valid, io.in_uop.bits, slot_uop)         //根据输入有效无效来决定是延续之前的uop还是使用输入的uop

 val next_uop_securitytag = Mux(io.in_uop.valid, io.in_uop_securitytag, slot_uop_securitytag)

  //-----------------------------------------------------------------------------
  // next slot state computation
  // compute the next state for THIS entry slot (in a collasping queue, the
  // current uop may get moved elsewhere, and a new uop can enter

  when (io.kill) {                            //slot的状态机
    state := s_invalid                                    //初始为无效状态
  } .elsewhen (io.in_uop.valid) {           //有新的微指令进入发射槽时
    state := io.in_uop.bits.iw_state       //状态转为输入给予的发射窗（issue window）状态
  } .elsewhen (io.clear) {                           //清除
    state := s_invalid                                    //转为无效状态
  } .otherwise {                                             //默认为进入下一状态
    state := next_state
  }

  //-----------------------------------------------------------------------------
  // "update" state
  // compute the next state for the micro-op in this slot. This micro-op may
  // be moved elsewhere, so the "next_state" travels with it.

  // defaults        初始条件（全部不变）
  next_state := state                       
  next_uopc := slot_uop.uopc

  next_uopc_securitytag := slot_uop_securitytag

  next_lrs1_rtype := slot_uop.lrs1_rtype
  next_lrs2_rtype := slot_uop.lrs2_rtype

  when (io.kill) {                                 //如果需要冲刷流水线
    next_state := s_invalid               //下一个状态应为无效状态      
  } .elsewhen ((io.grant && (state === s_valid_1)) ||           //当发射槽里有有效的微指令时，尝试发射该指令（s1，s2两种情况都有）
    (io.grant && (state === s_valid_2) && p1 && p2 && ppred)) {      //s2状态且操作数准备好
    // try to issue this uop.
    when (!(io.ldspec_miss && (p1_poisoned || p2_poisoned))) {      //
      next_state := s_invalid                                                           //切换为无效状态
    }
  } .elsewhen (io.grant && (state === s_valid_2)) {            //进入s2状态
    when (!(io.ldspec_miss && (p1_poisoned || p2_poisoned))) {        //且未出现推测错误
      next_state := s_valid_1                      //转为s1状态，下个时钟发射
      when (p1) {                                              //如果p1已经准备好（源操作数准备好）
        slot_uop.uopc := uopSTD               //

        slot_uop_securitytag :=      io.in_uop_securitytag                            //tag
        
        next_uopc := uopSTD

        next_uopc_securitytag :=       io.in_uop_securitytag                       //tag

        slot_uop.lrs1_rtype := RT_X
        next_lrs1_rtype := RT_X
      } .otherwise {
        slot_uop.lrs2_rtype := RT_X
        next_lrs2_rtype := RT_X
      }
    }
  }

  when (io.in_uop.valid) {
    slot_uop := io.in_uop.bits

    slot_uop_securitytag := io.in_uop_securitytag

    assert (is_invalid || io.clear || io.kill, "trying to overwrite a valid issue slot.")
  }

  // Wakeup Compare Logic

  // these signals are the "next_p*" for the current slot's micro-op.
  // they are important for shifting the current slot_uop up to an other entry.
  val next_p1 = WireInit(p1)
  val next_p2 = WireInit(p2)
  val next_p3 = WireInit(p3)
  val next_ppred = WireInit(ppred)

  when (io.in_uop.valid) {
    p1 := !(io.in_uop.bits.prs1_busy)
    p2 := !(io.in_uop.bits.prs2_busy)
    p3 := !(io.in_uop.bits.prs3_busy)
    ppred := !(io.in_uop.bits.ppred_busy)
  }

  when (io.ldspec_miss && next_p1_poisoned) {
    assert(next_uop.prs1 =/= 0.U, "Poison bit can't be set for prs1=x0!")
    p1 := false.B
  }
  when (io.ldspec_miss && next_p2_poisoned) {
    assert(next_uop.prs2 =/= 0.U, "Poison bit can't be set for prs2=x0!")
    p2 := false.B
  }

  for (i <- 0 until numWakeupPorts) {
    when (io.wakeup_ports(i).valid &&
         (io.wakeup_ports(i).bits.pdst === next_uop.prs1)) {
      p1 := true.B
    }
    when (io.wakeup_ports(i).valid &&
         (io.wakeup_ports(i).bits.pdst === next_uop.prs2)) {
      p2 := true.B
    }
    when (io.wakeup_ports(i).valid &&
         (io.wakeup_ports(i).bits.pdst === next_uop.prs3)) {
      p3 := true.B
    }
  }
  when (io.pred_wakeup_port.valid && io.pred_wakeup_port.bits === next_uop.ppred) {
    ppred := true.B
  }

  for (w <- 0 until memWidth) {
    assert (!(io.spec_ld_wakeup(w).valid && io.spec_ld_wakeup(w).bits === 0.U),
      "Loads to x0 should never speculatively wakeup other instructions")
  }

  // TODO disable if FP IQ.
  for (w <- 0 until memWidth) {
    when (io.spec_ld_wakeup(w).valid &&
      io.spec_ld_wakeup(w).bits === next_uop.prs1 &&
      next_uop.lrs1_rtype === RT_FIX) {
      p1 := true.B
      p1_poisoned := true.B
      assert (!next_p1_poisoned)
    }
    when (io.spec_ld_wakeup(w).valid &&
      io.spec_ld_wakeup(w).bits === next_uop.prs2 &&
      next_uop.lrs2_rtype === RT_FIX) {
      p2 := true.B
      p2_poisoned := true.B
      assert (!next_p2_poisoned)
    }
  }


  // Handle branch misspeculations
  val next_br_mask = GetNewBrMask(io.brupdate, slot_uop)

  // was this micro-op killed by a branch? if yes, we can't let it be valid if
  // we compact it into an other entry
  when (IsKilledByBranch(io.brupdate, slot_uop)) {
    next_state := s_invalid
  }

  when (!io.in_uop.valid) {
    slot_uop.br_mask := next_br_mask
  }

  //-------------------------------------------------------------
  // Request Logic
  io.request := is_valid && p1 && p2 && p3 && ppred && !io.kill
  val high_priority = slot_uop.is_br || slot_uop.is_jal || slot_uop.is_jalr
  io.request_hp := io.request && high_priority

  when (state === s_valid_1) {
    io.request := p1 && p2 && p3 && ppred && !io.kill
  } .elsewhen (state === s_valid_2) {
    io.request := (p1 || p2) && ppred && !io.kill
  } .otherwise {
    io.request := false.B
  }

  //assign outputs
  io.valid := is_valid
  io.uop := slot_uop

  io.uop_securitytag := slot_uop_securitytag           //tag

  io.uop.iw_p1_poisoned := p1_poisoned
  io.uop.iw_p2_poisoned := p2_poisoned

  // micro-op will vacate due to grant.
  val may_vacate = io.grant && ((state === s_valid_1) || (state === s_valid_2) && p1 && p2 && ppred)
  val squash_grant = io.ldspec_miss && (p1_poisoned || p2_poisoned)
  io.will_be_valid := is_valid && !(may_vacate && !squash_grant)

  io.out_uop            := slot_uop

  io.out_uop_securitytag            := slot_uop_securitytag      //tag

  io.out_uop.iw_state   := next_state
  io.out_uop.uopc       := next_uopc

  io.out_uop_securitytag       := next_uopc_securitytag                                 //tag

  io.out_uop.lrs1_rtype := next_lrs1_rtype
  io.out_uop.lrs2_rtype := next_lrs2_rtype
  io.out_uop.br_mask    := next_br_mask
  io.out_uop.prs1_busy  := !p1
  io.out_uop.prs2_busy  := !p2
  io.out_uop.prs3_busy  := !p3
  io.out_uop.ppred_busy := !ppred
  io.out_uop.iw_p1_poisoned := p1_poisoned
  io.out_uop.iw_p2_poisoned := p2_poisoned

  when (state === s_valid_2) {
    when (p1 && p2 && ppred) {
      ; // send out the entire instruction as one uop
    } .elsewhen (p1 && ppred) {
      io.uop.uopc := slot_uop.uopc

      io.uop_securitytag := slot_uop_securitytag             //tag

      io.uop.lrs2_rtype := RT_X
    } .elsewhen (p2 && ppred) {
      io.uop.uopc := uopSTD

      io.uop_securitytag :=   slot_uop_securitytag           //tag(这里不太确定对不对，但是应该此时slot里的tag就是该指令的tag)

      io.uop.lrs1_rtype := RT_X
    }
  }

  // debug outputs
  io.debug.p1 := p1
  io.debug.p2 := p2
  io.debug.p3 := p3
  io.debug.ppred := ppred
  io.debug.state := state
}
