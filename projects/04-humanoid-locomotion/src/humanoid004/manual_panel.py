"""Local simulation controls with real release/focus events; no global key hooks."""
import time


class TerminalDecoder:
    """Ignore ANSI keys, terminal replies, OSC/DCS and bracketed paste payloads."""
    def __init__(self):
        self.state='plain'
        self.sequence=''

    def feed(self, data):
        result=[]
        for byte in data:
            ch=chr(byte)
            if self.state=='paste':
                self.sequence=(self.sequence+ch)[-6:]
                if self.sequence=='\x1b[201~':
                    self.state='plain'
                continue
            if self.state=='string':
                if ch=='\x07': self.state='plain'
                elif ch=='\x1b': self.state='string_escape'
                continue
            if self.state=='string_escape':
                self.state='plain' if ch=='\\' else 'string'
                continue
            if self.state=='escape':
                self.sequence=''
                self.state={'[':'csi','O':'ss3',']':'string','P':'string','^':'string','_':'string'}.get(ch,'plain')
                continue
            if self.state in ('csi','ss3'):
                self.sequence=(self.sequence+ch)[-32:]
                if 0x40 <= byte <= 0x7e:
                    self.state='paste' if self.sequence=='200~' else 'plain'
                    self.sequence=''
                continue
            if ch=='\x1b': self.state='escape'
            elif 32 <= byte < 127: result.append(ord(ch.upper()))
        return result


class ManualPanel:
    """Tk lives on the simulation main thread, while MuJoCo owns its viewer thread."""
    def __init__(self, keys, title):
        self.keys=keys
        self.title=title
        self.events=[]
        self.started=time.monotonic()
        self.down=set()
        self.blocked=set()
        self.mouse=None
        self.pending={}
        self.last_pump=-1.0
        self.last_status=''
        self.closed=False
        self.sim_time=0.0

    def event(self, name, **fields):
        self.events.append(dict(event=name, wall_seconds=time.monotonic()-self.started,
                                sim_time=self.sim_time, **fields))

    def __enter__(self):
        import tkinter as tk
        from tkinter import ttk
        self.root=tk.Tk()
        self.root.title('004 Controls - '+self.title)
        self.root.geometry('610x400')
        self.root.minsize(580,380)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.info=tk.StringVar(value='Click here. Wait for 2 simulation seconds, then hold W.')
        self.state=tk.StringVar(value='WARMUP')
        ttk.Label(self.root,text=self.title+' | MANUAL CONTROL',font=('Arial',16,'bold')).pack(pady=12)
        ttk.Label(self.root,text='Focus THIS window. Hold keys or mouse buttons.\nRelease or switch away to stop. W+A combinations are supported.',justify='center').pack()
        frame=ttk.Frame(self.root)
        frame.pack(pady=12)
        self.buttons={}
        for key,label,row,col in [('Q','Q  Side left',0,0),('W','W  Forward',0,1),('E','E  Side right',0,2),
                                  ('A','A  Turn left',1,0),('S','S  Backward',1,1),('D','D  Turn right',1,2),
                                  ('R','R  Forward-left',2,0),('X','X / Space  STOP',2,1),('T','T  Forward-right',2,2)]:
            button=ttk.Button(frame,text=label,takefocus=False,width=21)
            button.grid(row=row,column=col,padx=3,pady=3)
            button.bind('<ButtonPress-1>',lambda e,k=key:self.mouse_press(k))
            button.bind('<ButtonRelease-1>',lambda e:self.mouse_release())
            button.bind('<Leave>',lambda e:self.mouse_release())
            self.buttons[key]=button
        actions=ttk.Frame(self.root)
        actions.pack()
        for key,label in [('P','P  Pause / Resume'),('F','F  Push test'),('C','C  Cancel & stop')]:
            ttk.Button(actions,text=label,takefocus=False,command=lambda k=key:self.action(k)).pack(side='left',padx=4)
        ttk.Label(self.root,textvariable=self.info,justify='center').pack(pady=8)
        ttk.Label(self.root,textvariable=self.state,justify='center').pack()
        self.root.bind('<KeyPress>',self.key_press)
        self.root.bind('<KeyRelease>',self.key_release)
        self.root.bind('<FocusOut>',lambda e:self.root.after_idle(self.check_focus))
        self.root.bind('<Unmap>',lambda e:self.stop('WINDOW_HIDDEN'))
        self.root.update()
        self.root.focus_force()
        self.event('PANEL_OPENED')
        return self

    def focused(self):
        widget=self.root.focus_displayof()
        return widget is not None and widget.winfo_toplevel()==self.root

    def stop(self, reason):
        moving=bool((self.down-self.blocked).intersection(set('WSADQERT')) or self.mouse)
        self.blocked.update(self.down)
        self.mouse=None
        self.keys.release()
        if moving: self.event(reason)

    def check_focus(self):
        if not self.closed and not self.focused():
            self.stop('FOCUS_LOST')

    def action(self, key):
        if key in ('X',' ','P','C'):
            self.stop('STOP_OR_PAUSE')
        self.keys.callback(ord(key))
        if key=='C':
            self.keys.pause=False
        self.event('ACTION',key=key)

    @staticmethod
    def code(event):
        return ' ' if event.keysym=='space' else event.keysym.upper()

    def key_press(self, event):
        code=self.code(event)
        if code in self.pending:
            self.root.after_cancel(self.pending.pop(code))
        if len(code)!=1 or not self.focused(): return 'break'
        # Ignore Ctrl/Alt shortcuts (copy/paste must not become Cancel or Forward).
        if event.state & (0x4 | 0x8 | 0x80):
            self.stop('MODIFIER_SHORTCUT')
            return 'break'
        if code in self.down or code in self.blocked: return 'break'
        self.down.add(code)
        self.event('KEY_DOWN',key=code)
        if code in ('X',' ','P','C','F'): self.action(code)
        return 'break'

    def key_release(self, event):
        code=self.code(event)
        if code in self.pending: self.root.after_cancel(self.pending.pop(code))
        # X11 may synthesize release+press pairs for autorepeat. Coalesce those.
        def release():
            self.pending.pop(code,None)
            self.down.discard(code)
            self.blocked.discard(code)
            self.event('KEY_UP',key=code)
            self.heartbeat()
        self.pending[code]=self.root.after_idle(release)
        return 'break'

    def mouse_press(self, code):
        self.root.focus_force()
        if code=='X': self.action('X')
        else:
            self.mouse=code
            self.event('MOUSE_DOWN',key=code)
        return 'break'

    def mouse_release(self):
        if self.mouse:
            self.event('MOUSE_UP',key=self.mouse)
            self.mouse=None
            self.heartbeat()

    def heartbeat(self):
        if self.closed or not self.focused():
            self.keys.release()
            return
        codes=(self.down-self.blocked).intersection(set('WSADQERT'))
        if self.mouse: codes.add(self.mouse)
        self.keys.held([ord(c) for c in codes])
        self.info.set('Held: '+(' + '.join(sorted(codes)) if codes else 'none / holding position')+
                      '\nRelease input before switching focus back; use C to finish.')

    def pump(self):
        if self.closed: return
        now=time.monotonic()
        if now-self.last_pump < .02: return
        self.last_pump=now
        self.root.update()
        if not self.closed:
            self.check_focus()
            self.heartbeat()

    def status(self, now, phase, command):
        self.sim_time=now
        if self.closed: return
        text=f'{phase} | sim {now:.1f} s\nCommand: vx {command[0]:.2f} m/s, vy {command[1]:.2f} m/s, turn {command[2]:.2f} rad/s'
        if text!=self.last_status:
            self.state.set(text)
            self.last_status=text

    def close(self):
        if self.closed: return
        self.stop('PANEL_CLOSED')
        self.keys.callback(ord('C'))
        self.keys.pause=False
        self.closed=True
        self.root.destroy()

    def __exit__(self,*exc):
        self.keys.release()
        if not self.closed:
            self.closed=True
            self.root.destroy()
