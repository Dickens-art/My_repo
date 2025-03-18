//******************************************************************************
// Copyright (c) 2018 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Fetch Buffer
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Takes a FetchBundle and converts into a vector of MicroOps.

package boom.ifu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.{MStatus, BP, BreakpointUnit}

import boom.common._
import boom.util.{BoolToChar, MaskUpper}

/**
 * Bundle that is made up of converted MicroOps from the Fetch Bundle
 * input to the Fetch Buffer. This is handed to the Decode stage.
 */
class FetchBufferResp(implicit p: Parameters) extends BoomBundle   //直接看最后的resp_security
{
  val uops = Vec(coreWidth, Valid(new MicroOp()))
}

/**
 * Buffer to hold fetched packets and convert them into a vector of MicroOps
 * to give the Decode stage
 *
 * @param num_entries effectively the number of full-sized fetch packets we can hold.
 */
class FetchBuffer(implicit p: Parameters) extends BoomModule                              //看下边带securitytag的
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
{
  val numEntries = numFetchBufferEntries
  val io = IO(new BoomBundle {
    val enq = Flipped(Decoupled(new FetchBundle()))
    val deq = new DecoupledIO(new FetchBufferResp())

    // Was the pipeline redirected? Clear/reset the fetchbuffer.
    val clear = Input(Bool())
  })

  require (numEntries > fetchWidth)              
  require (numEntries % coreWidth == 0)
  val numRows = numEntries / coreWidth

  val ram = Reg(Vec(numEntries, new MicroOp))                      //定义一个ram存储输入信息

 //val ram_securitytag = Reg(Vec(numEntries, UInt(2.W)))  

  ram.suggestName("fb_uop_ram")
  val deq_vec = Wire(Vec(numRows, Vec(coreWidth, new MicroOp)))          //输出

  //val deq_vec_securitytag = Wire(Vec(numRows, Vec(coreWidth, UInt(2.W))))

  val head = RegInit(1.U(numRows.W))
  val tail = RegInit(1.U(numEntries.W))

  val maybe_full = RegInit(false.B)

  //-------------------------------------------------------------
  // **** Enqueue Uops ****
  // 输入uops
  //-------------------------------------------------------------
  // Step 1: Convert FetchPacket into a vector of MicroOps.  将FetchPacket转换为MicroOps的向量
  // Step 2: Generate one-hot write indices.                                  生成独热索引
  // Step 3: Write MicroOps into the RAM.                                       将microops写入ram

  def rotateLeft(in: UInt, k: Int) = {
    val n = in.getWidth
    Cat(in(n-k-1,0), in(n-1, n-k))
  }

  val might_hit_head = (1 until fetchWidth).map(k => VecInit(rotateLeft(tail, k).asBools.zipWithIndex.filter
    {case (e,i) => i % coreWidth == 0}.map {case (e,i) => e}).asUInt).map(tail => head & tail).reduce(_|_).orR
  val at_head = (VecInit(tail.asBools.zipWithIndex.filter {case (e,i) => i % coreWidth == 0}
    .map {case (e,i) => e}).asUInt & head).orR
  val do_enq = !(at_head && maybe_full || might_hit_head)

  io.enq.ready := do_enq

  // Input microops.
  val in_mask = Wire(Vec(fetchWidth, Bool()))
  val in_uops = Wire(Vec(fetchWidth, new MicroOp()))

  //val in_uops = Wire(Vec(fetchWidth, new MicroOp()))

  // Step 1: Convert FetchPacket into a vector of MicroOps.
  for (b <- 0 until nBanks) {
    for (w <- 0 until bankWidth) {
      val i = (b * bankWidth) + w

      val pc = (bankAlign(io.enq.bits.pc) + (i << 1).U)

      in_uops(i)                := DontCare
      in_mask(i)                := io.enq.valid && io.enq.bits.mask(i)
      in_uops(i).edge_inst      := false.B
      in_uops(i).debug_pc       := pc
      in_uops(i).pc_lob         := pc

      in_uops(i).is_sfb         := io.enq.bits.sfbs(i) || io.enq.bits.shadowed_mask(i)

      if (w == 0) {
        when (io.enq.bits.edge_inst(b)) {
          in_uops(i).debug_pc  := bankAlign(io.enq.bits.pc) + (b * bankBytes).U - 2.U    
          in_uops(i).pc_lob    := bankAlign(io.enq.bits.pc) + (b * bankBytes).U
          in_uops(i).edge_inst := true.B
        }
      }
      in_uops(i).ftq_idx        := io.enq.bits.ftq_idx
      in_uops(i).inst           := io.enq.bits.exp_insts(i)
      in_uops(i).debug_inst     := io.enq.bits.insts(i)
      in_uops(i).is_rvc         := io.enq.bits.insts(i)(1,0) =/= 3.U
      in_uops(i).taken          := io.enq.bits.cfi_idx.bits === i.U && io.enq.bits.cfi_idx.valid

      in_uops(i).xcpt_pf_if     := io.enq.bits.xcpt_pf_if
      in_uops(i).xcpt_ae_if     := io.enq.bits.xcpt_ae_if
      in_uops(i).bp_debug_if    := io.enq.bits.bp_debug_if_oh(i)
      in_uops(i).bp_xcpt_if     := io.enq.bits.bp_xcpt_if_oh(i)

      in_uops(i).debug_fsrc     := io.enq.bits.fsrc
    }
  }

  // Step 2. Generate one-hot write indices.
  val enq_idxs = Wire(Vec(fetchWidth, UInt(numEntries.W)))

  def inc(ptr: UInt) = {
    val n = ptr.getWidth
    Cat(ptr(n-2,0), ptr(n-1))
  }

  var enq_idx = tail
  for (i <- 0 until fetchWidth) {
    enq_idxs(i) := enq_idx
    enq_idx = Mux(in_mask(i), inc(enq_idx), enq_idx)
  }

  // Step 3: Write MicroOps into the RAM.
  for (i <- 0 until fetchWidth) {
    for (j <- 0 until numEntries) {
      when (do_enq && in_mask(i) && enq_idxs(i)(j)) {
        ram(j) := in_uops(i)
      }
    }
  }

  //-------------------------------------------------------------
  // **** Dequeue Uops ****
  //-------------------------------------------------------------

  val tail_collisions = VecInit((0 until numEntries).map(i =>
                          head(i/coreWidth) && (!maybe_full || (i % coreWidth != 0).B))).asUInt & tail
  val slot_will_hit_tail = (0 until numRows).map(i => tail_collisions((i+1)*coreWidth-1, i*coreWidth)).reduce(_|_)
  val will_hit_tail = slot_will_hit_tail.orR

  val do_deq = io.deq.ready && !will_hit_tail

  val deq_valids = (~MaskUpper(slot_will_hit_tail)).asBools

  // Generate vec for dequeue read port.
  for (i <- 0 until numEntries) {
    deq_vec(i/coreWidth)(i%coreWidth) := ram(i)
  }

  io.deq.bits.uops zip deq_valids           map {case (d,v) => d.valid := v}
  io.deq.bits.uops zip Mux1H(head, deq_vec) map {case (d,q) => d.bits  := q}
  io.deq.valid := deq_valids.reduce(_||_)

  //-------------------------------------------------------------
  // **** Update State ****
  //-------------------------------------------------------------

  when (do_enq) {
    tail := enq_idx
    when (in_mask.reduce(_||_)) {
      maybe_full := true.B
    }
  }

  when (do_deq) {
    head := inc(head)
    maybe_full := false.B
  }

  when (io.clear) {
    head := 1.U
    tail := 1.U
    maybe_full := false.B
  }

  // TODO Is this necessary?
  when (reset.asBool) {
    io.deq.bits.uops map { u => u.valid := false.B }
  }

}

//Security_Tag
/**
 * Buffer to hold fetched packets and convert them into a vector of MicroOps
 * to give the Decode stage
 *
 * @param num_entries effectively the number of full-sized fetch packets we can hold.
 */
class FetchBuffer_Security(implicit p: Parameters) extends BoomModule           //带securitytag的fetch buffer
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
{
  val numEntries = numFetchBufferEntries                    //深度为16
  val io = IO(new BoomBundle {
    val enq = Flipped(Decoupled(new FetchBundle_Security()))    //inst_securitytag         
    val deq = new DecoupledIO(new FetchBufferResp_Security())   //uops_securitytag

    // Was the pipeline redirected? Clear/reset the fetchbuffer.
    val clear = Input(Bool())
  })

  require (numEntries > fetchWidth)
  require (numEntries % coreWidth == 0)
  val numRows = numEntries / coreWidth

  val ram = Reg(Vec(numEntries, new MicroOp))                                                               //ram存输入的microops
  val ram_security = Reg(Vec(numEntries, UInt(2.W)))                                                    //使用寄存器存储输入的securitytag

  //val ram_tagnotpass = Reg(Vec(numEntries,false.B))                                                        //ram存储指令对应的tag检查结果,默认为0，即没有不通过  

  ram.suggestName("fb_uop_ram")
  ram_security.suggestName("fb_uop_securitytag_ram")
  val deq_vec = Wire(Vec(numRows, Vec(coreWidth, new MicroOp)))                      //输出microops
  val deq_vec_security = Wire(Vec(numRows, Vec(coreWidth, UInt(2.W))))            //输出securitytag的线网

  //val deq_vec_tagnotpass = Wire(Vec(numRows, Vec(coreWidth, false.B)))             //异常输出

  val head = RegInit(1.U(numRows.W))                                                                                  //头部和尾部指针
  val tail = RegInit(1.U(numEntries.W))

  val maybe_full = RegInit(false.B)                                                                                          //buffer满

  //-------------------------------------------------------------
  // **** Enqueue Uops ****
  //输入uops（和securitytag）
  //1、把 fetchpacket 转换成 uops、securitytag
  //2、生成独热码
  //3、把uops、securitytag写入ram
  //-------------------------------------------------------------
  // Step 1: Convert FetchPacket into a vector of MicroOps.
  // Step 2: Generate one-hot write indices.
  // Step 3: Write MicroOps into the RAM.

  def rotateLeft(in: UInt, k: Int) = {                //回环左移函数，把in左移k位，低位用之前的高k位补充
    val n = in.getWidth
    Cat(in(n-k-1,0), in(n-1, n-k))
  }

  val might_hit_head = (1 until fetchWidth).map(k => VecInit(rotateLeft(tail, k).asBools.zipWithIndex.filter
    {case (e,i) => i % coreWidth == 0}.map {case (e,i) => e}).asUInt).map(tail => head & tail).reduce(_|_).orR
  val at_head = (VecInit(tail.asBools.zipWithIndex.filter {case (e,i) => i % coreWidth == 0}
    .map {case (e,i) => e}).asUInt & head).orR
  val do_enq = !(at_head && maybe_full || might_hit_head)                     
  //或许可以把idex判断加到enq里边？不行，必须得发出ready后，才能输入指令，进而判断

  io.enq.ready := do_enq

  // Input microops.
  val in_mask = Wire(Vec(fetchWidth, Bool()))                                   //
  val in_uops = Wire(Vec(fetchWidth, new MicroOp()))                  //将输入的packet拆解为uops
  val in_uops_security = Wire(Vec(fetchWidth, UInt(2.W)))           //输入securitytag的线网

  //val in_uops_tagpass = RegInit(Vec(fetchWidth,Bool()))                      //每条指令的安全检测结果

  // Step 1: Convert FetchPacket into a vector of MicroOps.
  for (b <- 0 until nBanks) {
    for (w <- 0 until bankWidth) {
      val i = (b * bankWidth) + w           
      //每个bank取到的microops的合并，bankwidth为每个bank承担的指令数
      //b为bank索引，b = 0，w为bank内的指令索引，bankwidth = 4，i为合并后的指令索引

      val pc = (bankAlign(io.enq.bits.pc) + (i << 1).U)      //i << 1为 2i ，即每个指令对应的pc依次+2（考虑rvc）

     

      in_uops(i)                := DontCare
      in_uops_security(i)  := DontCare
      in_mask(i)                := io.enq.valid && io.enq.bits.mask(i)             //
      in_uops(i).edge_inst      := false.B
      in_uops(i).debug_pc       := pc 

     

      in_uops(i).pc_lob         := pc                               //记录uops对应的pc

      in_uops(i).is_sfb         := io.enq.bits.sfbs(i) || io.enq.bits.shadowed_mask(i)

      if (w == 0) {
        when (io.enq.bits.edge_inst(b)) {                      //出现边缘指令
          in_uops(i).debug_pc  := bankAlign(io.enq.bits.pc) + (b * bankBytes).U - 2.U   //
          in_uops(i).pc_lob    := bankAlign(io.enq.bits.pc) + (b * bankBytes).U
          in_uops(i).edge_inst := true.B
        }
      }
      in_uops(i).ftq_idx        := io.enq.bits.ftq_idx                           //
      in_uops(i).inst           := io.enq.bits.exp_insts(i)                      //输入的指令机器码
      in_uops(i).debug_inst     := io.enq.bits.insts(i)                      //debug_inst直接就是inst（难道是为了传递下去一直记录当前该级执行的inst信息？）
      in_uops(i).is_rvc         := io.enq.bits.insts(i)(1,0) =/= 3.U    //假如inst最后两位为11，则为rvc（16bits）
      in_uops_security(i)       := io.enq.bits.insts_securitytag(i)                //输入的该指令securitytag

    //指令tag输入完，在此之后即可检查

    

  
      in_uops(i).taken          := io.enq.bits.cfi_idx.bits === i.U && io.enq.bits.cfi_idx.valid   //

      in_uops(i).xcpt_pf_if     := io.enq.bits.xcpt_pf_if                                //异常
      in_uops(i).xcpt_ae_if     := io.enq.bits.xcpt_ae_if
      in_uops(i).bp_debug_if    := io.enq.bits.bp_debug_if_oh(i)
      in_uops(i).bp_xcpt_if     := io.enq.bits.bp_xcpt_if_oh(i)

      in_uops(i).debug_fsrc     := io.enq.bits.fsrc
    }
  }

  // Step 2. Generate one-hot write indices.
  val enq_idxs = Wire(Vec(fetchWidth, UInt(numEntries.W)))                 //独热码

  def inc(ptr: UInt) = {          //回环左移1位
    val n = ptr.getWidth
    Cat(ptr(n-2,0), ptr(n-1))
  }

  var enq_idx = tail                          //从尾部开始
  for (i <- 0 until fetchWidth) {
    enq_idxs(i) := enq_idx             //按指令顺序输入索引
    enq_idx = Mux(in_mask(i), inc(enq_idx), enq_idx)            //如果mask（i）有效，则回环左移1位
  }

  // Step 3: Write MicroOps into the RAM.  

  //加入判断条件：pc_tag - inst_tag <= 1，则更新pc-tag

   //val pc_securitytag = RegInit(0.U(2.W))                                                                      //pc_tag, 默认值设为0

   //val inst_tag_check_notpass  = RegInit(false.B)                                                    //异常
   //inst_tag_check_notpass  := ram_tagnotpass.reduce(_||_)                              //发现异常即抛出
           
  
 for (i <- 0 until fetchWidth) {       //对于一个packet内的所有指令
    for (j <- 0 until numEntries) {   //对于fb的每个条目
      when (do_enq && in_mask(i) && enq_idxs(i)(j)/* && pc_securitytag_pass(i)*/) {             //全部都放进来，然后再判断安全检查

 /*       assert((pc_securitytag(1) && (!in_uops_security(i)(1))) ||                     
                                                                     ((!pc_securitytag(0)) &&(!in_uops_security(i)(1))) || 
                                                                     (pc_securitytag(1) && (!in_uops_security(i)(0))) || 
                                                                     (pc_securitytag(0) && in_uops_security(i)(1)), "指令安全检查不通过")           
                                                                      //指令安全检查，抛出的异常在下边  */

        ram(j) := in_uops(i)
        ram_security(j) := in_uops_security(i)                           //输入的securitytag进入寄存器

     //   ram_tagnotpass(j)  :=  !( (pc_securitytag(1) && (!in_uops_security(i)(1))) ||                      //当满足往fetch buffer写入指令的条件时进行安全检查
     //                                                                ((!pc_securitytag(0)) &&(!in_uops_security(i)(1))) || 
     //                                                                (pc_securitytag(1) && (!in_uops_security(i)(0))) || 
     //                                                                (pc_securitytag(0) && in_uops_security(i)(1))         )

     //   pc_securitytag  :=  in_uops_security(i)                   //更新pc_tag，这里有个问题，pc_tag不可能一个周期更新i次，怎么处理？

      
      }
    }
  }




  //-------------------------------------------------------------
  // **** Dequeue Uops ****
  //-------------------------------------------------------------

  val tail_collisions = VecInit((0 until numEntries).map(i =>
                          head(i/coreWidth) && (!maybe_full || (i % coreWidth != 0).B))).asUInt & tail
  val slot_will_hit_tail = (0 until numRows).map(i => tail_collisions((i+1)*coreWidth-1, i*coreWidth)).reduce(_|_)
  val will_hit_tail = slot_will_hit_tail.orR

  val do_deq = io.deq.ready && !will_hit_tail

  val deq_valids = (~MaskUpper(slot_will_hit_tail)).asBools

  // Generate vec for dequeue read port.
  for (i <- 0 until numEntries) {
    deq_vec(i/coreWidth)(i%coreWidth) := ram(i)
    deq_vec_security(i/coreWidth)(i%coreWidth) := ram_security(i)            //securitytag的输出

  //  deq_vec_tagnotpass(i/coreWidth)(i%coreWidth) := ram_tagnotpass(i)          //异常输出

  }

  io.deq.bits.uops zip deq_valids           map {case (d,v) => d.valid := v}

 // io.deq.bits.uops zip Mux1H(head, deq_vec_tagnotpass) map {case (d,q) => d.bits.inst_check_unpass  := q}           //异常输出


  io.deq.bits.uops zip Mux1H(head, deq_vec) map {case (d,q) => d.bits  := q}
  io.deq.bits.uops_securitytag zip Mux1H(head, deq_vec_security) map {case (d,q) => d := q}
  io.deq.valid := deq_valids.reduce(_||_)

  //-------------------------------------------------------------
  // **** Update State ****
  //-------------------------------------------------------------

  when (do_enq) {
    tail := enq_idx
    when (in_mask.reduce(_||_)) {
      maybe_full := true.B
    }
  }

  when (do_deq) {
    head := inc(head)
    maybe_full := false.B
  }

  when (io.clear) {
    head := 1.U
    tail := 1.U
    maybe_full := false.B
  }

  // TODO Is this necessary?
  when (reset.asBool) {
    io.deq.bits.uops map { u => u.valid := false.B }
  }

}

/**
 * Bundle that is made up of converted MicroOps from the Fetch Bundle
 * input to the Fetch Buffer. This is handed to the Decode stage.
 */
class FetchBufferResp_Security(implicit p: Parameters) extends BoomBundle           //定义一次取出来指令和securitytag的类型
{
  val uops = Vec(coreWidth, Valid(new MicroOp()))                                                                     //取到的微码
  val uops_securitytag = Vec(coreWidth, UInt(2.W))                                                                           //取到的securitytag（指令机器码为32位，故tag为2位）

  

}
