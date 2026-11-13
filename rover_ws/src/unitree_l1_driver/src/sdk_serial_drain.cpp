// SPDX-License-Identifier: BSD-3-Clause
// Compatibility shim for the bundled SDK 1.0.16 static archives only.
// runParse() calls Serial::waitReadable() BEFORE draining its own byte deque.
// On coalesced serial reads this can strand complete packets until more bytes
// arrive, grow the deque beyond its cap, and discard data. Let its immediately
// following available() check choose whether to read, then parse cached bytes.
// No vendor binary is patched; GNU ld --wrap redirects this one call at link time.
// The worker sleeps briefly on NONE; a separate process still owns the deadline.
extern "C" bool __wrap__ZN6serial6Serial12waitReadableEv(void*) {
  return true;
}
