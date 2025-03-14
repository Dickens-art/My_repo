//******************************************************************************
// Copyright (c) 2012 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Register Read
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters

import boom.common._
import boom.util._

/**
 * Handle the register read and bypass network for the OoO backend                       //处理rf和发射单元、执行单元的交互
 * interfaces with the issue window on the enqueue side, and the execution
 * pipelines on the dequeue side.
 *
 * @param issueWidth total issue width from all issue queues                                                                                                  //发射宽度（rv64i中指一次几条指令）
 * @param supportedUnitsArray seq of SupportedFuncUnits classes indicating what the functional units do   //
 * @param numTotalReadPorts number of read ports                                                                                                                   //读端口数量
 * @param numReadPortsArray execution units read port sequence                                                                                     //
 * @param numTotalBypassPorts number of bypass ports out of the execution units                                                   //bypass端口数量
 * @param registerWidth size of register in bits                                                                                                                                   //寄存器位宽
 */
class RegisterRead(                                   //读寄存器模块
  issueWidth: Int,
  supportedUnitsArray: Seq[SupportedFuncUnits],                //功能单元支持序列
  numTotalReadPorts: Int,                                                                   //读端口数量
  numReadPortsArray: Seq[Int],                                                       //读端口序列（将数据传递给执行单元）数量
                        // each exe_unit must tell us how many max
                        // operands it can accept (the sum should equal
                        // numTotalReadPorts)
  numTotalBypassPorts: Int,                                                              //
  numTotalPredBypassPorts: Int,                                                    //预测的bypass？
  registerWidth: Int                                                                                //寄存器位宽（64）
)(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    // issued micro-ops
    val iss_valids = Input(Vec(issueWidth, Bool()))                             //发射有效
    val iss_uops   = Input(Vec(issueWidth, new MicroOp()))           //发射微码

    val iss_uops_securitytag = Input(Vec(issueWidth,UInt(2.W)))   //tag

    // interface with register file's read ports
    val rf_read_ports = Flipped(Vec(numTotalReadPorts, new RegisterFileReadPortIO(maxPregSz, registerWidth)))  //模块和rf的交互端口（注意输出addr，输入data）（加入securitytag）
    val prf_read_ports = Flipped(Vec(issueWidth, new RegisterFileReadPortIO(log2Ceil(ftqSz), 1)))                                    //发射端口和模块之间的交互

    val bypass = Input(Vec(numTotalBypassPorts, Valid(new ExeUnitResp(registerWidth))))                                                   //bypass序列
    val pred_bypass = Input(Vec(numTotalPredBypassPorts, Valid(new ExeUnitResp(1))))

    // send micro-ops to the execution pipelines
    val exe_reqs = Vec(issueWidth, (new DecoupledIO(new FuncUnitReq(registerWidth))))                                                     
    //和exu的交互端口（输出exu需要的）(加入securitytag)

    val kill   = Input(Bool())                                                                                                                                                                                       //kill信号定义
    val brupdate = Input(new BrUpdateInfo())                                                                                                                                                //分支信号定义
  })

  val rrd_valids       = Wire(Vec(issueWidth, Bool()))                                                                                                                                    //读有效（给执行单元的）
  val rrd_uops         = Wire(Vec(issueWidth, new MicroOp()))                                                                                                                  //读到的微码

  val rrd_uops_securitytag         = Wire(Vec(issueWidth, UInt(2.W)))                                                                                                  //tag

  val exe_reg_valids   = RegInit(VecInit(Seq.fill(issueWidth) { false.B }))                                                                                           //用寄存器存储读到信息
  val exe_reg_uops     = Reg(Vec(issueWidth, new MicroOp()))

  val exe_reg_uops_securitytag     = Reg(Vec(issueWidth, UInt(2.W)))                                                                                          

  val exe_reg_rs1_data = Reg(Vec(issueWidth, Bits(registerWidth.W)))                                            
  val exe_reg_rs2_data = Reg(Vec(issueWidth, Bits(registerWidth.W)))
  val exe_reg_rs3_data = Reg(Vec(issueWidth, Bits(registerWidth.W)))
  val exe_reg_pred_data = Reg(Vec(issueWidth, Bool()))                                                                                                                      //是否为预测执行

  val exe_reg_rs1_data_securitytag = Reg(Vec(issueWidth, UInt(2.W)))                                                                                         //寄存器存读到的数据的securitytag
  val exe_reg_rs2_data_securitytag = Reg(Vec(issueWidth, UInt(2.W)))
  val exe_reg_rs3_data_securitytag = Reg(Vec(issueWidth, UInt(2.W)))

  //-------------------------------------------------------------
  // hook up inputs           输入解码

  for (w <- 0 until issueWidth) {                                                                                                                                                                              //微码中包含securitytag字段
    val rrd_decode_unit = Module(new RegisterReadDecode(supportedUnitsArray(w)))
    rrd_decode_unit.io.iss_valid := io.iss_valids(w)
    rrd_decode_unit.io.iss_uop   := io.iss_uops(w)

    rrd_decode_unit.io.iss_uop_securitytag   := io.iss_uops_securitytag(w)                       //tag

    //解码后的控制信号输出

    rrd_valids(w) := RegNext(rrd_decode_unit.io.rrd_valid &&
                !IsKilledByBranch(io.brupdate, rrd_decode_unit.io.rrd_uop))
    rrd_uops(w)   := RegNext(GetNewUopAndBrMask(rrd_decode_unit.io.rrd_uop, io.brupdate))   //uop并没有变化

    rrd_uops_securitytag(w) :=RegNext(rrd_decode_unit.io.rrd_uop_securitytag)                             //tag  
  }

  //-------------------------------------------------------------
  // read ports

  require (numTotalReadPorts == numReadPortsArray.reduce(_+_))                      //读端口数量应该等于读端口序列中1的个数（独热码）？

  val rrd_rs1_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))                         //操作数的线网
  val rrd_rs2_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_rs3_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_pred_data  = Wire(Vec(issueWidth, Bool()))                                                    //判断是否是预测的标记

 val rrd_rs1_data_securitytag   = Wire(Vec(issueWidth, Bits(2.W)))                        //操作数securitytag的线网
 val rrd_rs2_data_securitytag   = Wire(Vec(issueWidth, Bits(2.W)))
 val rrd_rs3_data_securitytag   = Wire(Vec(issueWidth, Bits(2.W)))

  rrd_rs1_data := DontCare
  rrd_rs2_data := DontCare
  rrd_rs3_data := DontCare
  rrd_pred_data := DontCare
 
  rrd_rs1_data_securitytag  := DontCare
  rrd_rs2_data_securitytag  := DontCare
  rrd_rs3_data_securitytag  := DontCare

  io.prf_read_ports := DontCare

  var idx = 0 // index into flattened read_ports array
  for (w <- 0 until issueWidth) {
    val numReadPorts = numReadPortsArray(w)

    // NOTE:
    // rrdLatency==1, we need to send read address at end of ISS stage,
    //    in order to get read data back at end of RRD stage.

    val rs1_addr = io.iss_uops(w).prs1                                                                     //地址传输（需要用寄存器记录，不然对应的是下一拍的）
    val rs2_addr = io.iss_uops(w).prs2
    val rs3_addr = io.iss_uops(w).prs3
    val pred_addr = io.iss_uops(w).ppred                   

    val iss_uops_securitytag = io.iss_uops_securitytag(w)                               //同上                                   

    if (numReadPorts > 0) io.rf_read_ports(idx+0).addr := rs1_addr
    if (numReadPorts > 1) io.rf_read_ports(idx+1).addr := rs2_addr
    if (numReadPorts > 2) io.rf_read_ports(idx+2).addr := rs3_addr

    if (enableSFBOpt) io.prf_read_ports(w).addr := pred_addr

    if (numReadPorts > 0) rrd_rs1_data(w) := Mux(RegNext(rs1_addr === 0.U), 0.U, io.rf_read_ports(idx+0).data)                  //数据对应
    if (numReadPorts > 1) rrd_rs2_data(w) := Mux(RegNext(rs2_addr === 0.U), 0.U, io.rf_read_ports(idx+1).data)
    if (numReadPorts > 2) rrd_rs3_data(w) := Mux(RegNext(rs3_addr === 0.U), 0.U, io.rf_read_ports(idx+2).data)

    if (numReadPorts > 0) rrd_rs1_data_securitytag(w) := Mux(RegNext(rs1_addr === 0.U),RegNext(iss_uops_securitytag), io.rf_read_ports(idx+0).data_securitytag)                  //securitytag对应，注意zero_tag = uop_tag
    if (numReadPorts > 1) rrd_rs2_data_securitytag(w) := Mux(RegNext(rs2_addr === 0.U),RegNext(iss_uops_securitytag), io.rf_read_ports(idx+1).data_securitytag)
    if (numReadPorts > 2) rrd_rs3_data_securitytag(w) := Mux(RegNext(rs3_addr === 0.U),RegNext(iss_uops_securitytag), io.rf_read_ports(idx+2).data_securitytag)


    if (enableSFBOpt) rrd_pred_data(w) := Mux(RegNext(io.iss_uops(w).is_sfb_shadow), io.prf_read_ports(w).data, false.B)                                       //选择是否为预测

    val rrd_kill = io.kill || IsKilledByBranch(io.brupdate, rrd_uops(w))

    exe_reg_valids(w) := Mux(rrd_kill, false.B, rrd_valids(w))
    // TODO use only the valids signal, don't require us to set nullUop
    exe_reg_uops(w)   := Mux(rrd_kill, NullMicroOp, rrd_uops(w))                                                      //控制信号输出

    exe_reg_uops_securitytag(w)  := Mux(rrd_kill,0.U(2.W),rrd_uops_securitytag(w))               //指令tag

    exe_reg_uops(w).br_mask := GetNewBrMask(io.brupdate, rrd_uops(w))

    idx += numReadPorts
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // BYPASS MUXES -----------------------------------------------
  // performed at the end of the register read stage

  // NOTES: this code is fairly hard-coded. Sorry.
  // ASSUMPTIONS:
  //    - rs3 is used for FPU ops which are NOT bypassed (so don't check
  //       them!).
  //    - only bypass integer registers.

  val bypassed_rs1_data = Wire(Vec(issueWidth, Bits(registerWidth.W)))       //bypass的数据
  val bypassed_rs2_data = Wire(Vec(issueWidth, Bits(registerWidth.W)))

  val bypassed_rs1_data_securitytag = Wire(Vec(issueWidth, Bits(2.W)))       //数据的tag
  val bypassed_rs2_data_securitytag = Wire(Vec(issueWidth, Bits(2.W)))

  val bypassed_pred_data = Wire(Vec(issueWidth, Bool()))                                  //是否为预测
  bypassed_pred_data := DontCare

  for (w <- 0 until issueWidth) {
    val numReadPorts = numReadPortsArray(w)
    var rs1_cases = Array((false.B, 0.U(registerWidth.W)))
    var rs2_cases = Array((false.B, 0.U(registerWidth.W)))

    var rs1_cases_securitytag = Array((false.B, 0.U(2.W)))                      //tag
    var rs2_cases_securitytag = Array((false.B, 0.U(2.W)))

    var pred_cases = Array((false.B, 0.U(1.W)))

    val prs1       = rrd_uops(w).prs1
    val lrs1_rtype = rrd_uops(w).lrs1_rtype
    val prs2       = rrd_uops(w).prs2
    val lrs2_rtype = rrd_uops(w).lrs2_rtype
    val ppred      = rrd_uops(w).ppred

    for (b <- 0 until numTotalBypassPorts)
    {
      val bypass = io.bypass(b)
      // can't use "io.bypass.valid(b) since it would create a combinational loop on branch kills"
      rs1_cases ++= Array((bypass.valid && (prs1 === bypass.bits.uop.pdst) && bypass.bits.uop.rf_wen
        && bypass.bits.uop.dst_rtype === RT_FIX && lrs1_rtype === RT_FIX && (prs1 =/= 0.U), bypass.bits.data))
      rs2_cases ++= Array((bypass.valid && (prs2 === bypass.bits.uop.pdst) && bypass.bits.uop.rf_wen
        && bypass.bits.uop.dst_rtype === RT_FIX && lrs2_rtype === RT_FIX && (prs2 =/= 0.U), bypass.bits.data))

      rs1_cases_securitytag ++= Array((bypass.valid && (prs1 === bypass.bits.uop.pdst) && bypass.bits.uop.rf_wen
        && bypass.bits.uop.dst_rtype === RT_FIX && lrs1_rtype === RT_FIX && (prs1 =/= 0.U), bypass.bits.data_securitytag))
      rs2_cases_securitytag ++= Array((bypass.valid && (prs2 === bypass.bits.uop.pdst) && bypass.bits.uop.rf_wen
        && bypass.bits.uop.dst_rtype === RT_FIX && lrs2_rtype === RT_FIX && (prs2 =/= 0.U), bypass.bits.data_securitytag))

    }

    for (b <- 0 until numTotalPredBypassPorts)
    {
      val bypass = io.pred_bypass(b)
      pred_cases ++= Array((bypass.valid && (ppred === bypass.bits.uop.pdst) && bypass.bits.uop.is_sfb_br, bypass.bits.data))
    }

    if (numReadPorts > 0) bypassed_rs1_data(w)  := MuxCase(rrd_rs1_data(w), rs1_cases) 
    if (numReadPorts > 1) bypassed_rs2_data(w)  := MuxCase(rrd_rs2_data(w), rs2_cases)

    if (numReadPorts > 0) bypassed_rs1_data_securitytag(w)  := MuxCase(rrd_rs1_data_securitytag(w), rs1_cases_securitytag) 
    if (numReadPorts > 1) bypassed_rs2_data_securitytag(w)  := MuxCase(rrd_rs2_data_securitytag(w), rs2_cases_securitytag) 

    if (enableSFBOpt)     bypassed_pred_data(w) := MuxCase(rrd_pred_data(w), pred_cases)
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  for (w <- 0 until issueWidth) {
    val numReadPorts = numReadPortsArray(w)
    if (numReadPorts > 0) exe_reg_rs1_data(w) := bypassed_rs1_data(w)
    if (numReadPorts > 1) exe_reg_rs2_data(w) := bypassed_rs2_data(w)
    if (numReadPorts > 2) exe_reg_rs3_data(w) := rrd_rs3_data(w)

    if (numReadPorts > 0) exe_reg_rs1_data_securitytag(w) := bypassed_rs1_data_securitytag(w)
    if (numReadPorts > 1) exe_reg_rs2_data_securitytag(w) := bypassed_rs2_data_securitytag(w)
    if (numReadPorts > 2) exe_reg_rs3_data_securitytag(w) := rrd_rs3_data_securitytag(w)


    if (enableSFBOpt)     exe_reg_pred_data(w) := bypassed_pred_data(w)
    // ASSUMPTION: rs3 is FPU which is NOT bypassed
  }
  // TODO add assert to detect bypass conflicts on non-bypassable things
  // TODO add assert that checks bypassing to verify there isn't something it hits rs3

  //-------------------------------------------------------------
  // set outputs to execute pipelines
  for (w <- 0 until issueWidth) {
    val numReadPorts = numReadPortsArray(w)

    io.exe_reqs(w).valid    := exe_reg_valids(w)
    io.exe_reqs(w).bits.uop := exe_reg_uops(w)

    io.exe_reqs(w).bits.uop_securitytag := exe_reg_uops_securitytag(w)

    if (numReadPorts > 0) io.exe_reqs(w).bits.rs1_data := exe_reg_rs1_data(w)
    if (numReadPorts > 1) io.exe_reqs(w).bits.rs2_data := exe_reg_rs2_data(w)
    if (numReadPorts > 2) io.exe_reqs(w).bits.rs3_data := exe_reg_rs3_data(w)

    if (numReadPorts > 0) io.exe_reqs(w).bits.rs1_data_securitytag := exe_reg_rs1_data_securitytag(w)
    if (numReadPorts > 1) io.exe_reqs(w).bits.rs2_data_securitytag := exe_reg_rs2_data_securitytag(w)
    if (numReadPorts > 2) io.exe_reqs(w).bits.rs3_data_securitytag := exe_reg_rs3_data_securitytag(w)

    if (enableSFBOpt)     io.exe_reqs(w).bits.pred_data := exe_reg_pred_data(w)
  }
}
