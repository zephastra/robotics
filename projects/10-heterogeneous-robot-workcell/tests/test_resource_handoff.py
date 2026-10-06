import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from workcell.resources import ResourceTable
from workcell.schema import SchemaRefused


def occupied():
    table=ResourceTable(epoch=2,resources=['source'])
    token=table.initialise_occupied('source',owner='order',ttl_s=10.,now_s=1.,
                                    evidence=['obs/loaded.json'])
    return table,token


def test_initial_loaded_state_is_occupied_not_free():
    table,token=occupied()
    assert token['state']=='OCCUPIED' and token['owner']=='order'
    assert table.expire(100.)==[]


def test_handover_preserves_occupancy_and_revokes_old_generation():
    table,token=occupied()
    new=table.handover_occupied('source',owner='order',generation=token['generation'],
        epoch=2,new_owner='transfer',ttl_s=10.,now_s=2.,evidence=['obs/dock.json'])
    assert new['state']=='OCCUPIED' and new['owner']=='transfer'
    assert new['generation']==token['generation']+1
    with pytest.raises(SchemaRefused):
        table.release('source',owner='order',generation=token['generation'],epoch=2)


@pytest.mark.parametrize('change',[{'owner':'other'},{'generation':0},{'epoch':1},
    {'now_s':20.},{'now_s':0.},{'ttl_s':0.},{'evidence':[]},{'evidence':'obs/dock.json'},
    {'new_owner':'order'}])
def test_invalid_handover_does_not_mutate_loaded_source(change):
    table,token=occupied()
    arguments=dict(owner='order',generation=token['generation'],epoch=2,
                   new_owner='transfer',ttl_s=10.,now_s=2.,evidence=['obs/dock.json'])
    arguments.update(change)
    before=table.snapshot('source')
    with pytest.raises(SchemaRefused):table.handover_occupied('source',**arguments)
    assert table.snapshot('source')==before


def test_initial_occupied_cannot_replace_existing_owner():
    table,token=occupied()
    with pytest.raises(SchemaRefused):
        table.initialise_occupied('source',owner='other',ttl_s=10.,now_s=2.,
                                  evidence=['obs/loaded.json'])
    assert table.snapshot('source')['owner']=='order'
