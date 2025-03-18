//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename BusyTable  重命名busy table，用于记录每个寄存器是否处于"脏"状态，对于securitytag应该不需要作改动
//当进行重命名时，对于操作数而言，首先根据重命名映射表进行转换，
//并通过BusyTable判断该寄存器是否可读（不可读的情况则为RAW，无法解决只能等待）；
//对于目标寄存器而言，在Free List中找到一个空闲状态的寄存器为它分配，同时将BusyTable对应表项置位，并更新重命名映射表。
//寄存器被标记为busy(1有效)时，说明该寄存器中的源操作数没有准备好，寄存器不可读、当前指令不可发射。
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class BusyResp extends Bundle          //busylist输出（三个源寄存器是否准备好）
{
  val prs1_busy = Bool()
  val prs2_busy = Bool()
  val prs3_busy = Bool()
}

class RenameBusyTable(
  val plWidth: Int,
  val numPregs: Int,
  val numWbPorts: Int,
  val bypass: Boolean,
  val float: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_uops = Input(Vec(plWidth, new MicroOp))                                     
     //这里uop是寄存器的index（比如prs1，prd，lrs1），不包含指令内容，故不需要添加securitytag

    val busy_resps = Output(Vec(plWidth, new BusyResp))           
    val rebusy_reqs = Input(Vec(plWidth, Bool()))                              

    val wb_pdsts = Input(Vec(numWbPorts, UInt(pregSz.W)))
    val wb_valids = Input(Vec(numWbPorts, Bool()))

    val debug = new Bundle { val busytable = Output(Bits(numPregs.W)) }
  })

  val busy_table = RegInit(0.U(numPregs.W))
  // Unbusy written back registers.
  //完成写回的寄存器置0
  val busy_table_wb = busy_table & ~(io.wb_pdsts zip io.wb_valids)
    .map {case (pdst, valid) => UIntToOH(pdst) & Fill(numPregs, valid.asUInt)}.reduce(_|_)
  // Rebusy newly allocated registers.
  //新分配的重命名寄存器置1
  val busy_table_next = busy_table_wb | (io.ren_uops zip io.rebusy_reqs)
    .map {case (uop, req) => UIntToOH(uop.pdst) & Fill(numPregs, req.asUInt)}.reduce(_|_)

  busy_table := busy_table_next

  // Read the busy table.
  for (i <- 0 until plWidth) {
    val prs1_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs1 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
    val prs2_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs2 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
    val prs3_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs3 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)

    io.busy_resps(i).prs1_busy := busy_table(io.ren_uops(i).prs1) || prs1_was_bypassed && bypass.B
    io.busy_resps(i).prs2_busy := busy_table(io.ren_uops(i).prs2) || prs2_was_bypassed && bypass.B
    io.busy_resps(i).prs3_busy := busy_table(io.ren_uops(i).prs3) || prs3_was_bypassed && bypass.B
    if (!float) io.busy_resps(i).prs3_busy := false.B
  }

  io.debug.busytable := busy_table
}
