#!/usr/bin/env python3
"""Compile the MAP accounting helpers against a bounded admission budget."""
from pathlib import Path
import subprocess
import tempfile
source = (Path(__file__).resolve().parents[1] / 'mrp.c').read_text()
start = source.index('static void msrp_release_admission(')
end = source.index('\n#endif', start)
helpers = source[start:end]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define CONFIG_ESP_AVB_NUM_PORTS 2
typedef enum { AVB_SR_CLASS_A, AVB_SR_CLASS_B } avb_sr_class_e;
typedef struct {
 uint32_t admitted_bps[2]; avb_sr_class_e admitted_class[2];
} msrp_talker_entry_t;
static uint32_t total[2][2];
static unsigned admissions, releases;
static int avb_srp_admission_try_admit(int port, avb_sr_class_e cls, uint32_t bps) {
 ++admissions;
 if (total[port][cls] + bps > 112500000) return -2;
 total[port][cls] += bps; return 0;
}
static void avb_srp_admission_release(int port, avb_sr_class_e cls, uint32_t bps) {
 ++releases;
 assert(total[port][cls] >= bps);
 total[port][cls] -= bps;
}
'''
checks = r'''
int main(void) {
 msrp_talker_entry_t stream = {0}, other = {0}, refused = {0};
 for (unsigned refresh = 0; refresh < 1000; ++refresh)
  assert(msrp_update_admission(&stream, 1, AVB_SR_CLASS_B, 13632000) == 0);
 assert(admissions == 1 && releases == 0);
 assert(total[1][AVB_SR_CLASS_B] == 13632000);
 assert(msrp_update_admission(&other, 1, AVB_SR_CLASS_B, 50000000) == 0);
 assert(msrp_update_admission(&refused, 1, AVB_SR_CLASS_B, 100000000) == -2);
 msrp_release_admission(&refused, 1);
 assert(total[1][AVB_SR_CLASS_B] == 63632000);
 assert(msrp_update_admission(&stream, 1, AVB_SR_CLASS_B, 20000000) == 0);
 assert(total[1][AVB_SR_CLASS_B] == 70000000);
 assert(msrp_update_admission(&stream, 1, AVB_SR_CLASS_A, 30000000) == 0);
 assert(total[1][AVB_SR_CLASS_B] == 50000000);
 assert(total[1][AVB_SR_CLASS_A] == 30000000);
 assert(msrp_update_admission(&stream, 1, AVB_SR_CLASS_B, 100000000) == -2);
 assert(total[1][AVB_SR_CLASS_A] == 0);
 msrp_release_admission(&stream, 1);
 assert(total[1][AVB_SR_CLASS_B] == 50000000);
 msrp_release_admission(&other, 1);
 msrp_release_admission(&other, 1);
 assert(total[1][AVB_SR_CLASS_B] == 0);
 assert(msrp_update_admission(&stream, 0, AVB_SR_CLASS_B, 1000) == 0);
 assert(msrp_update_admission(&stream, 1, AVB_SR_CLASS_B, 2000) == 0);
 msrp_release_admission(&stream, 0);
 assert(total[0][AVB_SR_CLASS_B] == 0 && total[1][AVB_SR_CLASS_B] == 2000);
 msrp_release_admission(&stream, 1);
 puts("MAP accounting refresh, replacement, rejection, withdrawal and per-port cases passed");
}
'''
with tempfile.TemporaryDirectory() as work:
 path = Path(work) / 'test.c'
 binary = Path(work) / 'test'
 path.write_text(harness + helpers + checks)
 subprocess.run(['cc', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', '-g', str(path), '-o', str(binary)], check=True)
 subprocess.run([str(binary)], check=True)
