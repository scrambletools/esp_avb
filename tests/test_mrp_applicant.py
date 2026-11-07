#!/usr/bin/env python3
"""Compile the MRP Applicant state machine and check its transmit rows."""
from pathlib import Path
import re
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
source = (root / 'mrp.c').read_text()
header = (root / 'mrp.h').read_text()
start = source.index('void mrp_applicant_step(')
end = source.index('\n/* §10.7.8 Table 10-4', start)
step = source[start:end]
types = ''.join(re.search(r'typedef enum \{[^}]*\} %s;' % name, header).group(0) + '\n'
                for name in ('mrp_applicant_state_e', 'mrp_registrar_state_e',
                             'mrp_event_e', 'mrp_tx_action_e'))
types += re.search(r'typedef struct \{[^}]*\} mrp_sm_state_t;', header).group(0)
harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
''' + types + '\n'
checks = r'''
static mrp_sm_state_t at(mrp_applicant_state_e s) {
 mrp_sm_state_t sm = {0}; sm.applicant = s; sm.pending_tx = mrp_tx_none; return sm;
}
int main(void) {
 /* AP declares: sJ to QA on tx! and txLA!. */
 mrp_sm_state_t sm = at(mrp_applicant_ap);
 mrp_applicant_step(&sm, mrp_event_tx);
 assert(sm.applicant == mrp_applicant_qa && sm.pending_tx == mrp_tx_join);
 sm = at(mrp_applicant_ap);
 mrp_applicant_step(&sm, mrp_event_tx_la);
 assert(sm.applicant == mrp_applicant_qa && sm.pending_tx == mrp_tx_join);
 /* QP: quiet on tx!, sJ to QA on txLA!. */
 sm = at(mrp_applicant_qp);
 mrp_applicant_step(&sm, mrp_event_tx);
 assert(sm.applicant == mrp_applicant_qp && sm.pending_tx == mrp_tx_none);
 mrp_applicant_step(&sm, mrp_event_tx_la);
 assert(sm.applicant == mrp_applicant_qa && sm.pending_tx == mrp_tx_join);
 /* A declaration that hears the peer's JoinIn while VP still renews:
  * VP -rJoinIn-> AP -tx!-> sJ QA, then periodic! and tx! keep sending. */
 sm = at(mrp_applicant_vp);
 mrp_applicant_step(&sm, mrp_event_r_join_in);
 mrp_applicant_step(&sm, mrp_event_tx);
 assert(sm.applicant == mrp_applicant_qa && sm.pending_tx == mrp_tx_join);
 for (int period = 0; period < 3; ++period) {
  sm.pending_tx = mrp_tx_none;
  mrp_applicant_step(&sm, mrp_event_periodic);
  mrp_applicant_step(&sm, mrp_event_tx);
  assert(sm.pending_tx == mrp_tx_join);
 }
 /* Observers stay quiet. */
 for (mrp_applicant_state_e s = mrp_applicant_vo; s <= mrp_applicant_lo; ++s) {
  if (s != mrp_applicant_vo && s != mrp_applicant_ao && s != mrp_applicant_qo)
   continue;
  sm = at(s);
  mrp_applicant_step(&sm, mrp_event_tx);
  assert(sm.pending_tx == mrp_tx_none);
 }
 puts("ok");
 return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    c_file = Path(tmp) / 'applicant.c'
    c_file.write_text(harness + step + checks)
    binary = Path(tmp) / 'applicant'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Werror', '-o', str(binary),
                    str(c_file)], check=True)
    subprocess.run([str(binary)], check=True)
