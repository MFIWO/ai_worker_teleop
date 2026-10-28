"""[ADDED 2026-10-28] Offline-only SH5 ready-recovery protocol regressions."""
import json
import unittest

from hardware.ready_recovery import decode_request, gate_disarmed, recovery_event


class RecoveryContractTests(unittest.TestCase):
    def test_only_serialized_well_formed_requests_accepted(self):
        self.assertEqual(decode_request('{"request_id":"123abc","action":"ready_pose"}'),
                         '123abc')
        for text in ('{}','{"request_id":"","action":"ready_pose"}',
                     '{"request_id":"1","action":"start"}',
                     '{"request_id":4,"action":"ready_pose"}',
                     '{"request_id":"1","action":"ready_pose","enabled":true}',
                     'not json'):
            with self.subTest(text=text):
                with self.assertRaises((ValueError, TypeError)):
                    decode_request(text)

    def test_recovery_requires_fresh_disarmed_gate_and_healthy_robot(self):
        gate = {'model':'sh5','gate_instance':'new','enabled':False,
                'forwarding':False,'manual_active':False,'fault':None,
                'description_ok':True,'measured_fresh':True,'motor_healthy':True}
        self.assertTrue(gate_disarmed(gate,received_at=2.1,now=2.2))
        self.assertFalse(gate_disarmed(gate,received_at=2.1,now=3.))
        for key,value in (('enabled',True),('forwarding',True),('manual_active',True),
                          ('fault','hardware fault'),('description_ok',False),
                          ('measured_fresh',False),('motor_healthy',False),
                          ('model','sg2')):
            bad=dict(gate,**{key:value})
            with self.subTest(key=key):
                self.assertFalse(gate_disarmed(bad,received_at=2.1,now=2.2))

    def test_acknowledgement_is_request_id_matched(self):
        event=json.loads(recovery_event('abc123','READY','fresh gate'))
        self.assertEqual(event,{'request_id':'abc123','state':'READY','message':'fresh gate'})
        with self.assertRaises(ValueError):
            recovery_event('abc123','MOVING_WHILE_ACTIVE')


if __name__ == '__main__':
    unittest.main()
