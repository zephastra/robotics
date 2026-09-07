import os
import time
import numpy as np
import pytest
from humanoid004.app import Keys
from humanoid004.manual_panel import ManualPanel, TerminalDecoder


@pytest.mark.parametrize('sequence',[b'\x1b[C',b'\x1b[D',b'\x1b[1;5C',b'\x1bOC',
                                     b'\x1b[200~wcpff\x1b[201~',b'\x1b]0;CWF\x07',
                                     b'\x1bPcw\x1b\\'])
def test_terminal_escape_sequences_cannot_command_robot(sequence):
    decoder=TerminalDecoder()
    result=[]
    for byte in sequence:
        result+=decoder.feed(bytes([byte]))
    assert result==[]
    assert decoder.feed(b'wc')==[ord('W'),ord('C')]


def test_hold_heartbeat_does_not_need_repeat_but_expires_if_ui_stalls(monkeypatch):
    import humanoid004.app as app
    clock=[0.0]
    monkeypatch.setattr(app.time,'monotonic',lambda:clock[0])
    keys=Keys('t800')
    for i in range(500):
        clock[0]=i*.02
        keys.held([ord('W')])
        assert keys.read()[0][0]==.4
    clock[0]+=.51
    assert np.all(keys.read()[0]==0)
    keys.held([ord('W'),ord('A')])
    np.testing.assert_allclose(keys.read()[0],[.4,0,.6])
    keys.release()
    assert np.all(keys.read()[0]==0)


def test_pause_cancel_suppress_held_input():
    keys=Keys('t800')
    for code in ('P','C'):
        keys.callback(ord(code))
        keys.held([ord('W')])
        assert np.all(keys.read()[0]==0)


@pytest.fixture
def panel():
    if os.environ.get('H004_TEST_GUI')!='1':
        pytest.skip('Set H004_TEST_GUI=1 to create a dedicated control test window')
    with ManualPanel(Keys('t800'),'T800 input test') as ui:
        ui.root.update()
        yield ui


def press(ui, key):
    ui.root.event_generate('<KeyPress>',keysym=key)
    ui.root.update()
    ui.heartbeat()


def release(ui,key):
    ui.root.event_generate('<KeyRelease>',keysym=key)
    ui.root.update()
    ui.heartbeat()


def test_gui_single_press_is_held_past_old_timeout(panel):
    press(panel,'w')
    end=time.monotonic()+.8
    while time.monotonic()<end:
        panel.pump()
        assert panel.keys.read()[0][0]==.4
        time.sleep(.02)
    release(panel,'w')
    assert np.all(panel.keys.read()[0]==0)


def test_gui_autorepeat_coalescing_and_pause(panel):
    press(panel,'p')
    assert panel.keys.pause
    # Simulated X11 repeat pair is queued before the idle release callback.
    panel.root.event_generate('<KeyRelease>',keysym='p',when='tail')
    panel.root.event_generate('<KeyPress>',keysym='p',when='tail')
    panel.root.update()
    assert panel.keys.pause
    release(panel,'p')
    press(panel,'p')
    assert not panel.keys.pause


def test_gui_focus_loss_stops_input_and_close_cancels_while_paused(panel):
    import tkinter as tk
    press(panel,'w')
    other=tk.Toplevel(panel.root)
    other.title('004 test focus target')
    other.update()
    other.focus_force()
    panel.root.update()
    panel.check_focus()
    assert np.all(panel.keys.read()[0]==0)
    other.destroy()
    panel.keys.pause=True
    panel.close()
    assert panel.keys.cancel and not panel.keys.pause


def test_gui_mouse_hold_release_and_stop_blocks_existing_key(panel):
    press(panel,'w')
    panel.action('X')
    panel.heartbeat()
    assert np.all(panel.keys.read()[0]==0)
    release(panel,'w')
    button=panel.buttons['W']
    button.event_generate('<ButtonPress-1>',x=10,y=10)
    panel.root.update()
    panel.heartbeat()
    assert panel.keys.read()[0][0]==.4
    button.event_generate('<ButtonRelease-1>',x=10,y=10)
    panel.root.update()
    assert np.all(panel.keys.read()[0]==0)


def test_gui_ctrl_c_and_arrow_do_not_cancel(panel):
    panel.root.event_generate('<KeyPress>',keysym='c',state=0x4)
    panel.root.event_generate('<KeyPress>',keysym='Right')
    panel.root.update()
    assert not panel.keys.cancel
