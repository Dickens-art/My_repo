//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename FreeList

//重命名 freelist ，用于记录哪些物理寄存器可用，重命名阶段查找该表并找到一个空闲状态的寄存器为它分配
//对于securitytag，应该不需要改动
//
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class RenameFreeList(
  val plWidth: Int,
  val numPregs: Int,
  val numLregs: Int)
  (implicit p: Parameters) extends BoomModule
{
  private val pregSz = log2Ceil(numPregs)
  private val n = numPregs

  val io = IO(new BoomBundle()(p) {
    // Physical register requests.
    val reqs          = Input(Vec(plWidth, Bool()))                                             //微指令的请求信息
    val alloc_pregs   = Output(Vec(plWidth, Valid(UInt(pregSz.W))))  //输出空闲的寄存器编号

    // Pregs returned by the ROB.
    val dealloc_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W))))  //需要释放的寄存器（rob中退休了的）

    // Branch info for starting new allocation lists.    分支信息
    val ren_br_tags   = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))

    // Mispredict info for recovering speculatively allocated registers.
    val brupdate        = Input(new BrUpdateInfo)                                      //错误预测信息

    val debug = new Bundle {
      val pipeline_empty = Input(Bool())
      val freelist = Output(Bits(numPregs.W))
      val isprlist = Output(Bits(numPregs.W))
    }
  })
  // The free list register array and its branch allocation lists.
  val free_list = RegInit(UInt(numPregs.W), ~(1.U(numPregs.W)))
  //free_list 保存每个物理寄存器的空闲状态，1表示空闲，初始化为111……110，宽度为numPregs。
  //x0 寄存器的值始终保持为0，表示始终不空闲，因为 x0 寄存器的值固定为 0，不可用于分配作为目的寄存器写入新的值。

  val br_alloc_lists = Reg(Vec(maxBrCount, UInt(numPregs.W)))
  //保存对于每一个分支，物理寄存器的分配状况，1表示被分配。其更新逻辑是：

  //若分支指令有效，br_alloc_lists 与 alloc_masks 保持一致
  //否则，将分支中已释放的寄存器(br_deallocs)置0，新分配的寄存器(alloc_masks[0])置1

  // Select pregs from the free list.
  val sels = SelectFirstN(free_list, plWidth)
  //使用 SelctFirstN 函数生成，从free_list中寻找第一个值为 1 的位返回其独热码，
  //并将该位的值修改为0，共执行 plWidth 次，即返回 plWidth 个独热码。
  val sel_fire  = Wire(Vec(plWidth, Bool()))
  //指示每条指令的请求是否能够得到满足，1有效。

  // Allocations seen by branches in each pipeline slot.
  val allocs = io.alloc_pregs map (a => UIntToOH(a.bits))
  val alloc_masks = (allocs zip io.reqs).scanRight(0.U(n.W)) { case ((a,r),m) => m | a & Fill(n,r) }

  // Masks that modify the freelist array.
  val sel_mask = (sels zip sel_fire) map { case (s,f) => s & Fill(n,f) } reduce(_|_)         //记录寄存器是否被分配，是由sels和sel_fire组成的掩码。
  val br_deallocs = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)
  val dealloc_mask = io.dealloc_pregs.map(d => UIntToOH(d.bits)(numPregs-1,0) & Fill(n,d.valid)).reduce(_|_) | br_deallocs
  //保存需要释放的物理寄存器独热码集合，需要释放的寄存器包括：
  //在dealloc_pregs中声明释放的寄存器
  //因分支预测失败需要释放的寄存器

  val br_slots = VecInit(io.ren_br_tags.map(tag => tag.valid)).asUInt
  // Create branch allocation lists.
  for (i <- 0 until maxBrCount) {
    val list_req = VecInit(io.ren_br_tags.map(tag => UIntToOH(tag.bits)(i))).asUInt & br_slots
    val new_list = list_req.orR
    br_alloc_lists(i) := Mux(new_list, Mux1H(list_req, alloc_masks.slice(1, plWidth+1)),
                                       br_alloc_lists(i) & ~br_deallocs | alloc_masks(0))
  }

  // Update the free list.
  free_list := (free_list & ~sel_mask | dealloc_mask) & ~(1.U(numPregs.W))
  //free_list 更新的逻辑是：

  //将被分配出去(sel_mask=1)的寄存器置0
  //将被释放(dealloc_mask=1)的寄存器置1
  //x0 寄存器始终不空闲

  // Pipeline logic | hookup outputs.
  for (w <- 0 until plWidth) {
    val can_sel = sels(w).orR
    val r_valid = RegInit(false.B)
    val r_sel   = RegEnable(OHToUInt(sels(w)), sel_fire(w))

    r_valid := r_valid && !io.reqs(w) || can_sel
    sel_fire(w) := (!r_valid || io.reqs(w)) && can_sel

    io.alloc_pregs(w).bits  := r_sel
    io.alloc_pregs(w).valid := r_valid
  }

  io.debug.freelist := free_list | io.alloc_pregs.map(p => UIntToOH(p.bits) & Fill(n,p.valid)).reduce(_|_)
  io.debug.isprlist := 0.U  // TODO track commit free list.

  assert (!(io.debug.freelist & dealloc_mask).orR, "[freelist] Returning a free physical register.")
  assert (!io.debug.pipeline_empty || PopCount(io.debug.freelist) >= (numPregs - numLregs - 1).U,
    "[freelist] Leaking physical registers.")
}
