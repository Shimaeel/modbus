#pragma once
/**
 * @file snippets.hpp
 * @brief Small Modbus read functions that populate `SharedMemory`.
 *
 * @details
 * These are the "raw function snippets" the manager described — each one:
 *   1. issues a single Modbus read,
 *   2. writes the result into `SharedMemory` under a stable tag key,
 *   3. on failure, marks the tag BAD/STALE with an error message
 *      (it does **not** throw — the state machine should keep running).
 *
 * SEL-735 register layout used here is documented in CLAUDE.md
 * (Table E.26): addr 0..19 = FID, 20..39 = serial, 62 = meter form, etc.
 *
 * Tag naming convention (kept short and grep-able):
 *   identity.fid                 → std::string
 *   identity.serial              → std::string
 *   identity.meter_form          → uint16_t (raw enum)
 *   io.contact_inputs            → std::vector<bool>
 *   io.contact_outputs           → std::vector<bool>
 *   io.device_word               → std::vector<uint16_t> (50-reg bitmap block)
 *   counters.comm                → std::vector<uint16_t> (9 regs starting at 160)
 */

#include "shared_memory.hpp"

namespace Modbus {

class Master;  // forward

/** @brief FID + serial + meter form → identity.* keys. */
void readDeviceIdentity   (Master& m, SharedMemory& sm);

/** @brief 6 contact inputs (FC 02) → io.contact_inputs. */
void readContactInputs    (Master& m, SharedMemory& sm);

/** @brief 23 coil bits (FC 01) → io.contact_outputs. */
void readContactOutputs   (Master& m, SharedMemory& sm);

/** @brief Device-word bitmap block 100..149 (FC 03) → io.device_word. */
void readDeviceWordBitmap (Master& m, SharedMemory& sm);

/** @brief Communication counters 160..168 (FC 03) → counters.comm. */
void readCommCounters     (Master& m, SharedMemory& sm);

} // namespace Modbus
