from tkinter import *
from tkinter.messagebox import *
import datetime
import builtins
import threading

def waitInputConfirm(msg):
    root = None
    try:
        root = Tk()
        root.withdraw()
        ret = askokcancel("waitInputConfirm", msg)
        return ret == True
    except Exception:
        _, e, _ = sys.exc_info()
        if "couldn't connect to display" in str(e):
            c = input(msg+' (Enter [Y/y] to proceed) ').lower()
            return c == 'y' or c == ''
        raise
    finally:
        if root:
            root.destroy()
    return True

def waitInputSelect(msg):
    root = Tk()
    root.withdraw()
    ret = askyesno("waitInputSelect", msg)
    root.destroy()
    return ret

class commandPanel:
    def __init__(self, frame, wimf, label, cmd, isFirst):
        self.wimf = wimf
        self.panel = Frame(frame)
        self.panel.pack()
        self.cmd = cmd
        self.isFirst = isFirst
        self.bCmd = Button(self.panel, text=label, command=self.processEvent)
        self.bCmd.pack(side=LEFT)
        self.args = []
        self.nstr = cmd.count("#T")
        self.nint = cmd.count("#I")
        self.ndbl = cmd.count("#D")
        if self.nstr:
            for i in range(self.nstr):
                tf = Text(self.panel, height=1, width=10)
                tf.pack(side=LEFT)
                self.args.append(tf)
                bc = Button(self.panel, text="select", command=self.selectFile)
                bc.pack(side=LEFT)
        elif self.nint:
            for i in range(self.nint):
                tf = Text(self.panel, height=1, width=10)
                tf.pack(side=LEFT)
                self.args.append(tf)
        elif self.ndbl:
            for i in range(self.ndbl):
                tf = Text(self.panel, height=1, width=10)
                tf.pack(side=LEFT)
                self.args.append(tf)
        self.continuous = BooleanVar()
        if cmd.count("#continuous"):
            #self.continuous.set(True)
            self.cbContinuous = CheckButton(self.panel, text="continuous", command=self.toggleContinuous, variable=self.continuous)
            self.cbContinuous.setSelected(True)
        else:
            self.cbContinuous = None
            self.continuous.set(False)
        return
    def toggleContinuous(self):
        if self.continuous.get() == True:
            self.continuous.set(False)
        else:
            #self.continuous.set(True)
            None
        return
    def selectFile(self):
        jfc = JFileChooser(".")
        pf = posFilter()
        jfc.addChoosableFileFilter(pf)
        ret = jfc.showOpenDialog(None)
        if ret == 0:
            path = jfc.getSelectedFile().toString()
            a = path.split("/")
            file = a[len(a)-1]
            base = file.split(".")[0]
            self.args[0].setText(base)
    def processEvent(self):
        cmd = self.cmd
        if self.nstr:
            for i in range(self.nstr):
                str = self.args[i].get("1.0",END)
                cmd = cmd.replace("#T", str[0:len(str)-1], 1)
        elif self.nint:
            for i in range(self.nint):
                str = self.args[i].get("1.0",END)
                cmd = cmd.replace("#I", str[0:len(str)-1], 1)
        elif self.ndbl:
            for i in range(self.ndbl):
                str = self.args[i].get("1.0",END)
                cmd = cmd.replace("#D", str[0:len(str)-1], 1)
        self.wimf.addHistory(cmd)
        #print "cmd=",cmd
        exec(cmd)
        if self.isFirst and self.wimf.isSequential():
            self.wimf.advancePage()
        return

class waitInputMenuFrame(Frame):
    def __init__(self, menu, master=None):
        Frame.__init__(self, master, width=400, height=600)
        self.master.title('waitInputMenu')
        
        self.lmenu = menu[1:]
        self.page = 0
        self.pageExec = 0
        # panel for tab buttons
        panel1 = LabelFrame(self, relief=GROOVE)
        panel1.pack()
        la = Button(panel1, text='Local', command = self.showLocalFrame)
        la.pack(padx = 5, pady = 5, side = LEFT)
        lb = Button(panel1, text='Global', command = self.showGlobalFrame)
        lb.pack(padx = 5, pady = 5, side = LEFT)
        lc = Button(panel1, text='History', command = self.showHistoryFrame)
        lc.pack(padx = 5, pady = 5, side = LEFT)
        # 
        panel2 = Frame(self)
        panel2.pack()
        # history panel
        self.HistoryPanel = Frame(panel2)
        self.taHistory = Text(self.HistoryPanel, height=5)
        self.taHistory.pack()
        # local menu panel
        self.localMenuPanel2 = Frame(panel2)
        self.localMenuPanel2.pack()
        self.localMenuPanel = Frame(self.localMenuPanel2)
        self.localMenuPanel.pack()
        p = LabelFrame(self.localMenuPanel2)
        p.pack()
        self.lPage = Label(p)
        self.bPrev = Button(p, text="Prev", command=self.prevPage)
        self.bPrev.pack(padx=5, pady=5, side=LEFT)
        self.bNext = Button(p, text="Next", command=self.nextPage)
        self.bNext.pack(padx=5, pady=5, side=LEFT)
        self.bSkip = Button(p, text="Skip", command=self.advancePage)
        self.bSkip.pack(padx=5, pady=5, side=LEFT)
        self.cbSequential = BooleanVar()
        cb = Checkbutton(p, text="sequential", command=self.setupPage, variable=self.cbSequential)
        cb.pack(padx=5, pady=5, side=LEFT)
        self.cbSequential.set(True)
        self.setupPage()
        # global menu panel
        gmenu = menu[0]
        self.GlobalMenuPanel = Frame(panel2)
        bQuit = Button(self.GlobalMenuPanel, text="Quit", command = exit)
        bQuit.pack(padx = 5, pady = 5)
        bRestart = Button(self.GlobalMenuPanel, text="Restart", command = self.restart)
        bRestart.pack(padx = 5, pady = 5)
        for i in range(len(gmenu)/2):
            label = gmenu[i*2]
            content = gmenu[i*2+1]
            if content == "#label":
                p = Label(self.GlobalMenuPanel, text=label, bd=0)
            else:
                cp = commandPanel(self.GlobalMenuPanel, self, label, content, False)
                p = cp.panel
            p.pack()
    def restart(self):
        self.pageExec = 0
        self.page = 0
        self.setupPage()
        self.showLocalFrame()
        return
    def isSequential(self):
        return self.cbSequential.get()
    def setupPage(self):
        for k,v in self.localMenuPanel.children.iteritems():
            v.pack_forget()
        pmenu = self.lmenu[self.page]
        for i in range(len(pmenu)/2):
            cp = commandPanel(self.localMenuPanel, self, pmenu[i*2], pmenu[i*2+1], i==0)
            if self.page != self.pageExec and self.isSequential():
                cp.bCmd.configure(state='disabled')
            else:
                cp.bCmd.configure(state='active')
            if cp.continuous.get() and self.page == self.pageExec:
                cp.processEvent()
        self.lPage.configure(text=str(self.page+1)+"/"+str(len(self.lmenu)))
        if self.page == 0:
            self.bPrev.configure(state='disabled')
        else:
            self.bPrev.configure(state='active')
        if self.page == len(self.lmenu)-1:
            self.bNext.configure(state='disabled')
            self.bSkip.configure(state='disabled')
        else:
            self.bNext.configure(state='active')
            self.bSkip.configure(state='active')
        return
    def advancePage(self):
        if self.pageExec < len(self.lmenu)-1:
            self.pageExec += 1
            self.page += 1
            self.setupPage()
        elif self.pageExec == len(self.lmenu)-1:
            self.showGlobalFrame()
        return
    def nextPage(self):
        self.page += 1
        self.setupPage()
        return
    def prevPage(self):
        self.page -= 1
        self.setupPage()
        return
    def addHistory(self,msg):
        dstr = datetime.datetime.today().strftime("%Y-%m-%d %H:%M:%S")
        str = dstr+":"+msg+"\n"
        self.taHistory.insert("1.0", str)
        return
    def showLocalFrame(self):
        self.HistoryPanel.pack_forget()
        self.GlobalMenuPanel.pack_forget()
        self.localMenuPanel2.pack()
        return True
    def showGlobalFrame(self):
        self.localMenuPanel2.pack_forget()
        self.HistoryPanel.pack_forget()
        self.GlobalMenuPanel.pack()
        return True
    def showHistoryFrame(self):
        self.localMenuPanel2.pack_forget()
        self.GlobalMenuPanel.pack_forget()
        self.HistoryPanel.pack()
        return True

def waitInputMenuMain(menu):
    f = waitInputMenuFrame(menu)
    f.pack()
    f.mainloop()
    
def waitInputMenu(menu):
    thr = threading.Thread(target=waitInputMenuMain, args=(menu,))
    thr.start()
    return thr
