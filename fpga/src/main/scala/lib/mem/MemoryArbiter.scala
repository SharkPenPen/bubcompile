package lib.mem

import chisel3._
import chisel3.util._

/**
 * Allows multiple initiator MemoryInterfaces to map to one target MemoryInterface.
 * Priority is given to the lowest initiator.
 */
class MemoryArbiter(addressWidth: Int, dataWidth: Int, n: Int) extends Module {
  val io = IO(new Bundle {
    val target = Flipped(new MemoryInterface(addressWidth, dataWidth))
    val initiator = Vec(n, new MemoryInterface(addressWidth, dataWidth))
  })

  io.target.enable := false.B
  io.target.address := DontCare
  io.target.write := DontCare
  io.target.dataWrite := DontCare
  io.target.writeStrobe := DontCare

  /// Whether we're currently waiting for an access to complete.
  val busy = RegInit(false.B)
  /// If busy, the initiator who initiated the access.
  val busyOwner = Reg(Vec(n, Bool()))
  /// Vector of which input ports have a memory request.
  val requestsVec = Wire(Vec(n, Bool()))
  /// Which input port is chosen for the next request.
  val requestChosen = Wire(Vec(n, Bool()))

  for ((initiator, i) <- io.initiator.zipWithIndex) {
    initiator.done := false.B
    initiator.dataRead := io.target.dataRead
    requestsVec(i) := initiator.enable

    when ((busy && busyOwner(i)) || (initiator.enable && requestChosen(i))) {
      io.target.enable := initiator.enable
      io.target.address := initiator.address
      io.target.write := initiator.write
      io.target.dataWrite := initiator.dataWrite
      io.target.writeStrobe := initiator.writeStrobe
    }

    when (busy && busyOwner(i)) {
      initiator.done := io.target.done
    }
  }

  // Choose the next request.
  val isRequesting = WireDefault(requestsVec.asUInt.orR)
  requestChosen := PriorityEncoderOH(requestsVec)

  when (busy) {
    when (io.target.done) {
      busy := false.B
    }
  } .elsewhen (isRequesting) {
    busyOwner := requestChosen
    busy := true.B
  }
}