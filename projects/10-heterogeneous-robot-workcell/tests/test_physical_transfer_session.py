import pytest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from workcell.resources import ResourceTable
from workcell.transfer import TransferLedger
from workcell.physical_transfer_session import PhysicalTransferSession


def setup():
    resources=ResourceTable(epoch=1,resources=['source','receiver'])
    for name in resources.state:
        resources.mark_cleared(name,evidence=['obs/init.json'],now_s=0.)
    ledger=TransferLedger(epoch=1)
    events=[]
    session=PhysicalTransferSession(ledger=ledger,resources=resources,journal=events.append)
    pre={key:True for key in ('source_capacity_free','receiver_capacity_free',
        'height_within_tolerance','lateral_within_tolerance','yaw_within_tolerance',
        'clearance_ok','amr_at_rest','source_has_tray','receiver_empty',
        'arms_retracted','stop_chain_healthy')}
    pre.update(evidence_age_s=0.,max_evidence_age_s=.5,ttl_s=30.)
    request=dict(transfer_id='transfer',order_id='order',epoch=1,
                 source='source',receiver='receiver',tray_id='tray',preconditions=pre)
    return session,request,events


def confirmation(_):
    return dict(receiver_supported=True,source_cleared=True,stopped_confirmed=True,
                evidence=['obs/receiver.json'])


def test_reserves_and_changes_custody_before_actual_drive():
    session,request,events=setup()
    def drive():
        assert session.ledger.by_id['transfer'].custody=='TRANSFERRING'
        assert all(session.resources.snapshot(n)['owner']=='transfer' for n in ('source','receiver'))
        return {'status':'SUCCEEDED'}
    session.execute(request,now_s=0.,launch_evidence=['obs/dock.json'],drive=drive,confirm=confirmation)
    assert session.ledger.by_id['transfer'].stage=='COMMITTED'
    assert events[1]['event']=='transferring_before_motion'
    session.release('transfer',cleared={'source':True,'receiver':True},
                    evidence=['obs/cleared.json'],now_s=1.)
    assert session.ledger.by_id['transfer'].stage=='RELEASED'


def test_unknown_launch_never_calls_drive():
    session,request,_=setup();request['preconditions']['arms_retracted']=None
    moved=[]
    with pytest.raises(Exception):
        session.execute(request,now_s=0.,launch_evidence=['obs/dock.json'],
                        drive=lambda:moved.append(True),confirm=confirmation)
    assert moved==[]


@pytest.mark.parametrize('field',['receiver_supported','source_cleared','stopped_confirmed'])
def test_missing_confirmation_retains_custody_and_locks(field):
    session,request,_=setup()
    def missing(result):
        observed=confirmation(result);del observed[field];return observed
    with pytest.raises(RuntimeError):
        session.execute(request,now_s=0.,launch_evidence=['obs/dock.json'],
                        drive=lambda:{'status':'SUCCEEDED'},confirm=missing)
    record=session.ledger.by_id['transfer']
    assert record.stage=='NEEDS_ATTENTION' and record.custody=='TRANSFERRING'
    assert all(session.resources.snapshot(n)['owner']=='transfer' for n in ('source','receiver'))


def test_drive_exception_is_not_a_delivery():
    session,request,_=setup()
    with pytest.raises(TimeoutError):
        session.execute(request,now_s=0.,launch_evidence=['obs/dock.json'],
            drive=lambda:(_ for _ in ()).throw(TimeoutError()),confirm=confirmation)
    assert session.ledger.by_id['transfer'].stage=='NEEDS_ATTENTION'


def test_commit_does_not_automatically_release_occupied_resources():
    session,request,_=setup()
    session.execute(request,now_s=0.,launch_evidence=['obs/dock.json'],
                    drive=lambda:{'status':'SUCCEEDED'},confirm=confirmation)
    with pytest.raises(Exception):
        session.release('transfer',cleared={'source':True},evidence=['obs/clear.json'],now_s=1.)
    assert session.resources.snapshot('receiver')['owner']=='transfer'


def test_interrupted_locks_do_not_expire_into_false_clearance():
    session,request,_=setup()
    with pytest.raises(TimeoutError):
        session.execute(request,now_s=0.,launch_evidence=['obs/dock.json'],
            drive=lambda:(_ for _ in ()).throw(TimeoutError()),confirm=confirmation)
    assert session.resources.expire(100.)==[]
    assert all(session.resources.snapshot(n)['state']=='OCCUPIED' for n in ('source','receiver'))


def occupied_source(session):
    table=session.resources
    table.state['source']='UNKNOWN';table.grant['source']=None
    token=table.initialise_occupied('source',owner='order',ttl_s=30.,now_s=0.,
                                    evidence=['obs/loaded.json'])
    return dict(owner='order',generation=token['generation'],epoch=1,
                evidence=['obs/source_handover.json'])


def test_loaded_source_transfers_without_false_free_state():
    session,request,_=setup();token=occupied_source(session)
    session.execute(request,now_s=1.,launch_evidence=['obs/dock.json'],source_grant=token,
                    drive=lambda:{'status':'SUCCEEDED'},confirm=confirmation)
    assert session.resources.snapshot('source')['state']=='OCCUPIED'
    assert session.resources.snapshot('source')['owner']=='transfer'


def test_receiver_refusal_leaves_original_loaded_source_owner():
    session,request,_=setup();token=occupied_source(session)
    session.resources.reserve('receiver',owner='other',ttl_s=30.,now_s=0.)
    with pytest.raises(Exception):
        session.execute(request,now_s=1.,launch_evidence=['obs/dock.json'],source_grant=token,
                        drive=lambda:pytest.fail('must not move'),confirm=confirmation)
    assert session.resources.snapshot('source')['owner']=='order'


def test_stale_source_token_rolls_back_empty_receiver_reservation():
    session,request,_=setup();token=occupied_source(session);token['generation']=0
    with pytest.raises(Exception):
        session.execute(request,now_s=1.,launch_evidence=['obs/dock.json'],source_grant=token,
                        drive=lambda:pytest.fail('must not move'),confirm=confirmation)
    assert session.resources.snapshot('source')['owner']=='order'
    assert session.resources.snapshot('receiver')['state']=='FREE'
