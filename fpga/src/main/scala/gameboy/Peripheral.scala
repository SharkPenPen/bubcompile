package gameboy

import chisel3._
import chisel3.util._

class PeripheralAccess extends Bundle {
  /** Peripheral register selection -- 0xFFxx */
  val address = Input(UInt(8.W))
  /** Whether there's an access attempt */
  val enabled = Input(Bool())
  /** Whether the access is a write. Includes clock enable gate. */
  val write = Input(Bool())
  /** The data being written */
  val dataWrite = Input(UInt(8.W))
  /** The data being read */
  val dataRead = Output(UInt(8.W))
  /** Whether the read is valid */
  val valid = Output(Bool())
}

class HighRam extends Module {
  val io = IO(new PeripheralAccess)

  // But only 127 bytes are accessible
  val memory = SRAM(128, UInt(8.W), numReadPorts = 0, numWritePorts = 0, numReadwritePorts = 1)
  val port = memory.readwritePorts(0)

  port.enable := io.enabled && io.address >= 0x80.U && io.address <= 0xFE.U
  port.address := io.address(6, 0)
  port.isWrite := io.write
  port.writeData := io.dataWrite
  io.dataRead := port.readData
  io.valid := port.enable && !port.isWrite
}
