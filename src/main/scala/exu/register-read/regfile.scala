//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Register File (Abstract class and Synthesizable RegFile)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import scala.collection.mutable.ArrayBuffer

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters

import boom.common._
import boom.util.{BoomCoreStringPrefix}

/**
 * IO bundle for a register read port
 *
 * @param addrWidth size of register address in bits      地址位宽，7位
 * @param dataWidth size of register in bits                        数据位宽 ，64位 
 */

//implicit为隐式传参，模块例化时不需要定义参数，所有参数只需要在顶层定义一次。Scala编译器会自动寻找用implicit关键字定义的变量或函数或对象，并传入模块。

class RegisterFileReadPortIO(val addrWidth: Int, val dataWidth: Int)(implicit p: Parameters) extends BoomBundle //定义了一组寄存器的读io类（输入一个地址，输出一个数据）
{
  val addr = Input(UInt(addrWidth.W))
  val data = Output(UInt(dataWidth.W))

   val data_securitytag = Output(UInt(2.W))               //securitytag，对于内部寄存器而言，每个寄存器一定只存一个数据，故直接写死为2bit

}

/**
 * IO bundle for the register write port    写端口信号
 *
 * @param addrWidth size of register address in bits
 * @param dataWidth size of register in bits
 */
class RegisterFileWritePort(val addrWidth: Int, val dataWidth: Int)(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(addrWidth.W)
  val data = UInt(dataWidth.W)

  val data_securitytag = UInt(2.W)                         //securitytag

}

/**
 * Utility function to turn ExeUnitResps to match the regfile's WritePort I/Os.
 */

// 定义一个工厂方法，decoupled函数作用是给接口包装valid和ready，默认接口方向为发送端的，即输入ready，输出valid和bits，若想改为接收端则可以加.flip翻转，gen通过bits字段访问 
// valid方法为产生一个valid信号和bits，当valid为1，bits有效，flipped为翻转输入输出方向，gen通过bits字段访问 

object WritePort
{
  def apply(enq: DecoupledIO[ExeUnitResp], addrWidth: Int, dataWidth: Int, rtype: UInt)   //enq里的decoiupled函数为exeunitresp信号包装上ready和valid
    (implicit p: Parameters): Valid[RegisterFileWritePort] = {                                                                //此处的valid[RegisterFileWritePort]为函数返回值类型
     val wport = Wire(Valid(new RegisterFileWritePort(addrWidth, dataWidth)))                         //生成wport

     wport.valid     := enq.valid && enq.bits.uop.dst_rtype === rtype
     wport.bits.addr := enq.bits.uop.pdst
     wport.bits.data := enq.bits.data

     wport.bits.data_securitytag := enq.bits.data_securitytag                              //securitytag字段 

     enq.ready       := true.B
     wport                                                                                                                  //返回wport
  }
}

/**
 * Register file abstract class
 *
 * @param numRegisters number of registers
 * @param numReadPorts number of read ports
 * @param numWritePorts number of write ports
 * @param registerWidth size of registers in bits
 * @param bypassableArray list of write ports from func units to the read port of the regfile
 */

//抽象一个registerfile类，输入参数为寄存器、读端口、写端口的数量和单个寄存器的位宽，及是否可以bypass
//seq函数作为传参使用，可以传输一个不定长的seq集合
//maxPregSz为最大物理寄存器数目，定义见common/parameters文件


abstract class RegisterFile(
  numRegisters: Int,
  numReadPorts: Int,
  numWritePorts: Int,
  registerWidth: Int,
  bypassableArray: Seq[Boolean]) // which write ports can be bypassed to the read ports? //boolean是scala的bool类型。注意是一个序列，因为不止一个端口可能被bypass
  (implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    val read_ports = Vec(numReadPorts, new RegisterFileReadPortIO(maxPregSz, registerWidth))              //读写端口定义
    val write_ports = Flipped(Vec(numWritePorts, Valid(new RegisterFileWritePort(maxPregSz, registerWidth))))
  })

//io定义，生成numreadport个前述的读端口，maxPregSz为最大物理寄存器数对应的位宽（即reg的索引位宽）
//生成numwriteport个前述的写端口（写端口为输入）


  private val rf_cost = (numReadPorts + numWritePorts) * (numReadPorts + 2*numWritePorts)   //寄存器堆的硬件消耗？（没懂，为什么是这样算的）
  private val type_str = if (registerWidth == fLen+1) "Floating Point" else "Integer"                              //判断存储数据类型为浮点数还是整数
  override def toString: String = BoomCoreStringPrefix(                                                                                    ////重写tostring方法，使得函数返回如下字符串（显示寄存器堆的配置）
    "==" + type_str + " Regfile==",
    "Num RF Read Ports     : " + numReadPorts,
    "Num RF Write Ports    : " + numWritePorts,
    "RF Cost (R+W)*(R+2W)  : " + rf_cost,
    "Bypassable Units      : " + bypassableArray)
}

/**
 * A synthesizable model of a Register File. You will likely want to blackbox this for more than modest port counts.
 *
 * @param numRegisters number of registers
 * @param numReadPorts number of read ports
 * @param numWritePorts number of write ports
 * @param registerWidth size of registers in bits
 * @param bypassableArray list of write ports from func units to the read port of the regfile
 */

//寄存器堆的模型

class RegisterFileSynthesizable(
   numRegisters: Int,                                             //物理寄存器数量
   numReadPorts: Int,
   numWritePorts: Int,
   registerWidth: Int,
   bypassableArray: Seq[Boolean])
   (implicit p: Parameters)
   extends RegisterFile(numRegisters, numReadPorts, numWritePorts, registerWidth, bypassableArray)
{
  // --------------------------------------------------------------

  val regfile = Mem(numRegisters, UInt(registerWidth.W))             //生成regfiles

  val regfile_securitytag = Mem(numRegisters, UInt(2.W))               //生成regfiles存储securitytag

  // --------------------------------------------------------------
  // Read ports.

  val read_data = Wire(Vec(numReadPorts, UInt(registerWidth.W)))                         //生成所有读端口对应的读数据的线网


  val read_securitytag = Wire(Vec(numReadPorts, UInt(2.W)))           //读端口对应的读securitytag的线网

  // Register the read port addresses to give a full cycle to the RegisterRead Stage (if desired).
  val read_addrs = io.read_ports.map(p => RegNext(p.addr))

  //对端口对应的读地址，map为对于read_ports集合的每个元素调用函数（寄存器生成），结果生成一个新集合，此处为用寄存器保存每一个read_ports.addr信号

  //regfile_securitytag(0) := 3.U(2.W)                          //tag[zero] = 11，实现在reg_read里边

  for (i <- 0 until numReadPorts) {                             //for循环包含until左边不包含until右边
    read_data(i) := regfile(read_addrs(i))                 //读数据和读地址的对应

    read_securitytag(i) := regfile_securitytag(read_addrs(i))         //securitytag与读地址的对应

  }

  // --------------------------------------------------------------
  // Bypass out of the ALU's write ports.
  // We are assuming we cannot bypass a writer to a reader within the regfile memory
  // for a write that occurs at the end of cycle S1 and a read that returns data on cycle S1.
  // But since these bypasses are expensive, and not all write ports need to bypass their data,
  // only perform the w->r bypass on a select number of write ports.

  require (bypassableArray.length == io.write_ports.length)
  //require需要（）内的成立才继续执行后续，常用于验证方法的前提条件，此处是bypassarray长度和写端口数目必须严格相等（即一一对应）

  if (bypassableArray.reduce(_||_)) {                       //规约或bypass序列，如果为1，则说明有端口需要bypass
    val bypassable_wports = ArrayBuffer[Valid[RegisterFileWritePort]]()
    io.write_ports zip bypassableArray map { case (wport, b) => if (b) { bypassable_wports += wport} }
    //arraybuffer是scala的一个集合类型，将写端口类的信号（data和addr（securitytag））存到该集合。初始为空

    //zip函数会合并两个序列为一个序列，序列的每个元素都是一个元组，分别包两个原始序列分别的两个元素。
   //然后调用case函数，生成 { 写端口信号（wport），是否bypass（b）} 序列
    //注意这里的+=由于类型是arraybuffer，实际上是往这个集合里加元素，元素内容为需要bypass的写端口信号（addr和data（securitytag））

    for (i <- 0 until numReadPorts) {                                                                        //对于每个读端口
      val bypass_ens = bypassable_wports.map(x => x.valid &&                 //通过地址匹配生成读端口的bypass使能信号序列，每个读端口最多只有一位是1，arraybuffer里的地址和读地址是否一样
        x.bits.addr === read_addrs(i))

      val bypass_data = Mux1H(VecInit(bypass_ens), VecInit(bypassable_wports.map(_.bits.data)))
       
      val bypass_data_securitytag = Mux1H(VecInit(bypass_ens.toSeq), VecInit(bypassable_wports.map(_.bits.data_securitytag).toSeq))          //securitytag也要bypass


       //seq序列中每个元素有固定的索引位置
       //定义每个读端口的bypass数据，toseq将map序列转换为seq序列，每个序列只有一个1（不知道为啥要转换，但不重要）。 从中筛选（map）出数据，toseq生成为序列
       //Mux1H函数是一种高级的多路选择器，它接受一个选择信号和一系列输入。选择信号可以是一个位宽为N的编码信号，其中只有一个比特位为1，其他都为0。根据选择信号的编码，Mux1H选择对应位置为1的输入作为输出。

      io.read_ports(i).data := Mux(bypass_ens.reduce(_|_), bypass_data, read_data(i))       //mux选择读到的数据是bypassdata还是存的数据（read_data）

      io.read_ports(i).data_securitytag := Mux(bypass_ens.reduce(_|_), bypass_data_securitytag, read_securitytag(i))      //选择securitytag是否bypass

    }
  } else {                                                                                   //若规约结果为0，则不需要bypass
    for (i <- 0 until numReadPorts) {
      io.read_ports(i).data := read_data(i)                  //读到的数据就是存的数据

      io.read_ports(i).data_securitytag := read_securitytag(i)                                                      //读到的数据对应的securitytag就是存的securitytag

    }
  }

  // --------------------------------------------------------------
  // Write ports.

  for (wport <- io.write_ports) {                                    //为wport赋值，wport定义见上
    when (wport.valid) {                                                     //写有效
      regfile(wport.bits.addr) := wport.bits.data      //写入对应地址

      regfile_securitytag(wport.bits.addr) := wport.bits.data_securitytag                                        //写入对应securitytag

    }
  }

  // ensure there is only 1 writer per register (unless to preg0)
  //表达式 assert(condition) 将在condition条件不成立的时候抛出 AssertionError
  //两个写操作同时有效且对应的地址一样（且该地址非0），则触发assert
  // =/= 操作符为不等操作符，和===类似
  if (numWritePorts > 1) {
    for (i <- 0 until (numWritePorts - 1)) {
      for (j <- (i + 1) until numWritePorts) {
        assert(!io.write_ports(i).valid ||
               !io.write_ports(j).valid ||
               (io.write_ports(i).bits.addr =/= io.write_ports(j).bits.addr) ||
               (io.write_ports(i).bits.addr === 0.U), // note: you only have to check one here
          "[regfile] too many writers a register")
      }
    }
  }
}
