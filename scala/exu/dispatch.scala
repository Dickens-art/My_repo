//******************************************************************************
// Copyright (c) 2012 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// BOOM Instruction Dispatcher
//派遣
//对于securitytag，只需要让tag跟随指令一起被派遣出去即可
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters

import boom.common._
import boom.util._

class DispatchIO(implicit p: Parameters) extends BoomBundle
{
  // incoming microops from rename2
  //来自ren（ame）单元的uop
  val ren_uops = Vec(coreWidth, Flipped(DecoupledIO(new MicroOp)))

  val ren_uops_securitytag = Input(Vec(coreWidth, UInt(2.W)))

  // outgoing microops to issue queues
  //dis（patch）到发射序列的uop
  // N issues each accept up to dispatchWidth uops
  // N个发射口，每个发射口最多接受dispatchwidth个指令
  // dispatchWidth may vary between issue queues
  // dispatchwidth会因发射序列而不同
  //（在BOOM中存在着三个发射队列（整数指令、浮点指令和访存指令），不同类型的指令会放入不同的发射队列中。）
  val dis_uops = MixedVec(issueParams.map(ip=>Vec(ip.dispatchWidth, DecoupledIO(new MicroOp))))

  val dis_uops_securitytag = Output(MixedVec(issueParams.map(ip=>Vec(ip.dispatchWidth, UInt(2.W)))))

  //dontTouch(dis_uops_securitytag)

}

abstract class Dispatcher(implicit p: Parameters) extends BoomModule
{
  val io = IO(new DispatchIO)
}

/**
 * This Dispatcher assumes worst case, all dispatched uops go to 1 issue queue
 * This is equivalent to BOOMv2 behavior
 */
class BasicDispatcher(implicit p: Parameters) extends Dispatcher
{
  issueParams.map(ip=>require(ip.dispatchWidth == coreWidth))

  val ren_readys = io.dis_uops.map(d=>VecInit(d.map(_.ready)).asUInt).reduce(_&_)

  for (w <- 0 until coreWidth) {
    io.ren_uops(w).ready := ren_readys(w)
  }

  for {i <- 0 until issueParams.size
       w <- 0 until coreWidth} {
    val issueParam = issueParams(i)
    val dis        = io.dis_uops(i)

    val dis_securitytag = io.dis_uops_securitytag(i)

    dis(w).valid := io.ren_uops(w).valid && ((io.ren_uops(w).bits.iq_type & issueParam.iqType.U) =/= 0.U)
    dis(w).bits  := io.ren_uops(w).bits

    dis_securitytag(w) := io.ren_uops_securitytag(w)

  }
}

/**
 *  Tries to dispatch as many uops as it can to issue queues,
 *  which may accept fewer than coreWidth per cycle.
 *  When dispatchWidth == coreWidth, its behavior differs
 *  from the BasicDispatcher in that it will only stall dispatch when
 *  an issue queue required by a uop is full.
 */
class CompactingDispatcher(implicit p: Parameters) extends Dispatcher
{
  issueParams.map(ip => require(ip.dispatchWidth >= ip.issueWidth))

  val ren_readys = Wire(Vec(issueParams.size, Vec(coreWidth, Bool())))

  for (((ip, dis), rdy) <- issueParams zip io.dis_uops zip ren_readys) {
    val ren = Wire(Vec(coreWidth, Decoupled(new MicroOp)))

    val ren_securitytag = Wire(Vec(coreWidth, UInt(2.W)))

    ren <> io.ren_uops

    ren_securitytag <> io.ren_uops_securitytag

    val uses_iq = ren map (u => (u.bits.iq_type & ip.iqType.U).orR)

    // Only request an issue slot if the uop needs to enter that queue.
    (ren zip io.ren_uops zip uses_iq) foreach {case ((u,v),q) =>
      u.valid := v.valid && q}

    val compactor = Module(new Compactor(coreWidth, ip.dispatchWidth, new MicroOp))
    compactor.io.in  <> ren
    dis <> compactor.io.out

   // val compactor_securitytag = Module(new Compactor(coreWidth, ip.dispatchWidth, UInt(2.W)))
   // compactor.io.in  <> ren_securitytag                //securitytag
   // dis_securitytag  <> compactor.io.out              //securitytag

    // The queue is considered ready if the uop doesn't use it.
    rdy := ren zip uses_iq map {case (u,q) => u.ready || !q}
  }

  (ren_readys.reduce((r,i) =>
      VecInit(r zip i map {case (r,i) =>
        r && i})) zip io.ren_uops) foreach {case (r,u) =>
          u.ready := r}
}
