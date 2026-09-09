"""Exercise this application's Tk callbacks with a real MuJoCo viewer (WSLg)."""
from pathlib import Path
import sys
import tkinter as tk
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from humanoid006 import app


def main():
    original_tk = tk.Tk
    ticks = [0]
    checks = []
    original_step = app.Runtime.step

    def step(self,*args,**kwargs):
        ticks[0] += 1
        return original_step(self,*args,**kwargs)

    def create_panel():
        root = original_tk()
        def start_check():
            buttons = [w for w in root.winfo_children() if isinstance(w,tk.Button)]
            buttons[0].invoke()
            before = ticks[0]
            def resume_check():
                checks.append(ticks[0] == before)
                buttons[0].invoke()
                def cancel_check():
                    checks.append(ticks[0] > before)
                    buttons[1].invoke()
                root.after(400,cancel_check)
            root.after(400,resume_check)
        root.after(500,start_check)
        return root

    with patch.object(tk,'Tk',create_panel), patch.object(app.Runtime,'step',step), \
         patch.object(sys,'argv',['gui_smoke','--mode','lift']):
        result = app.main()
    assert result == 1 and checks == [True,True], (result,checks)
    print('GUI smoke passed: pause freezes physics, resume advances, cancel exits.')


if __name__ == '__main__':
    main()
