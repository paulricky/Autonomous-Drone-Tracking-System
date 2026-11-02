import importlib.util
from pathlib import Path
import queue
import threading

spec=importlib.util.spec_from_file_location('hand_control',Path(__file__).resolve().parents[2]/'scripts/run_hand_control.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def client():
    c=module.Connection.__new__(module.Connection)
    c.recovery_latched=c.takeoff_key_latched=c.started=c.enabled=False
    c.takeoff_prerequisites=None;c.status={};c.sample=None
    c.emergency_pending=threading.Event();c.pending=queue.PriorityQueue()
    return c


def test_single_t_debounces_but_can_retry_after_readiness_changes():
    c=client();c.request('TAKEOFF');c.request('TAKEOFF')
    assert c.pending.qsize()==1
    c.status={'status':dict(armed=True,ready=True,ground=True,mode='POSCTL',identity_verified=True)}
    c.enabled=True;c.request('TAKEOFF');c.request('TAKEOFF')
    assert c.pending.qsize()==2
    c.takeoff_key_latched=True;c.request('TAKEOFF')
    assert c.pending.qsize()==2


def test_emergency_bypasses_normal_queue_and_inhibits_commands():
    c=client();c.request('EMERGENCY');c.request('ARM')
    assert c.started and c.emergency_pending.is_set() and c.recovery_latched
    assert c.pending.empty() and not c.enabled
