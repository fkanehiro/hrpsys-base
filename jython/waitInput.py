from javax.swing import *
from javax.swing.filechooser import FileFilter
from java.lang import *
from java.awt import *
import datetime
import __builtin__

wimf = None
def waitInputConfirm(msg):
    if wimf:
        f = wimf.f
    else:
        f = None
    ret = JOptionPane.showConfirmDialog(f, msg, "waitInputConfirm",
                                        JOptionPane.OK_CANCEL_OPTION)
    if ret == 2:
        raise StandardError, "script is canceled"
    return True
    
def waitInputSelect(msg):
    if wimf:
        f = wimf.f
    else:
        f = None
    ret = JOptionPane.showConfirmDialog(f, msg, "waitInputSelect",
                                        JOptionPane.YES_NO_CANCEL_OPTION)
    if ret == 0:
        return True
    elif ret == 1:
        return False
    elif ret == 2:
        raise StandardError, "script is canceled"

class posFilter(FileFilter):
    def accept(self, f):
        if f.isDirectory():
            return True
        s = f.toString()
        ss = s.split(".")
        ext = ss[len(ss)-1]
        return ext == "pos"

class commandPanel:
    def __init__(self, frame, label, cmd, isFirst):
        self.frame = frame
        self.panel = JPanel()
        self.cmd = cmd
        self.isFirst = isFirst
        self.bCmd = JButton(label, actionPerformed=self.processEvent)
        self.panel.add(self.bCmd)
        self.args = []
        self.nstr = cmd.count("#T")
        self.nint = cmd.count("#I")
        self.ndbl = cmd.count("#D")
        if self.nstr:
            for i in range(self.nstr):
                tf = JTextField(15)
                self.args.append(tf)
                self.panel.add(tf)
                bc = JButton("select", actionPerformed=self.selectFile)
                self.panel.add(bc)
        elif self.nint:
            for i in range(self.nint):
                tf = JTextField(3)
                self.args.append(tf)
                self.panel.add(tf)
        elif self.ndbl:
            for i in range(self.ndbl):
                tf = JTextField(3)
                self.args.append(tf)
                self.panel.add(tf)
        if cmd.count("#continuous"):
            self.cbContinuous = JCheckBox("continuous", actionPerformed=self.toggleContinuous)
            self.cbContinuous.setSelected(True)
            self.panel.add(self.cbContinuous)
            self.continuous = True
        else:
            self.cbContinuous = None
            self.continuous = False
        return
    def toggleContinuous(self,event):
        self.continuous = self.cbContinuous.isSelected()
        return
    def selectFile(self, event):
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
    def processEvent(self, event):
        cmd = self.cmd
        if self.nstr:
            for i in range(self.nstr):
                cmd = cmd.replace("#T", self.args[i].getText(), 1)
        elif self.nint:
            for i in range(self.nint):
                cmd = cmd.replace("#I", self.args[i].getText(), 1)
        elif self.ndbl:
            for i in range(self.ndbl):
                cmd = cmd.replace("#D", self.args[i].getText(), 1)
        self.frame.addHistory(cmd)
        exec(cmd)
        if self.isFirst and self.frame.isSequential():
            self.frame.advancePage()
        return

class waitInputMenuFrame:
    def __init__(self, menu):
        self.f = JFrame("waitInputMenu")
        self.f.setSize(400, 600)
        self.tabPane = JTabbedPane(JTabbedPane.TOP)
        
        self.lmenu = menu[1:]
        self.page = 0
        self.pageExec = 0
        self.color = Color(238,238,238)
        panel1 = JPanel()
        panel1.setLayout(BorderLayout())
        self.localMenuPanel = JPanel()
        self.localMenuPanel.setLayout(BoxLayout(self.localMenuPanel, 
                                                BoxLayout.Y_AXIS))
        panel1.add(self.localMenuPanel, BorderLayout.NORTH)
        p = JPanel()
        p.setLayout(GridLayout(1,4))
        self.lPage = JLabel()
        p.add(self.lPage)
        self.bPrev = JButton("Prev", actionPerformed=self.prevPage)
        p.add(self.bPrev)
        self.bNext = JButton("Next", actionPerformed=self.nextPage)
        p.add(self.bNext)
        self.bSkip = JButton("Skip", actionPerformed=self.advancePage)
        p.add(self.bSkip)
        self.cbSequential = JCheckBox("sequential", actionPerformed=self.setupPage)
        self.cbSequential.setSelected(True)
        p.add(self.cbSequential)
        panel1.add(p, BorderLayout.SOUTH)
        self.setupPage()
        self.tabPane.addTab("Local", panel1)
        
        gmenu = menu[0]
        panel2 = JPanel()
        p = JPanel()
        panel2.setLayout(BoxLayout(panel2, BoxLayout.Y_AXIS))
        bQuit = JButton("Quit", actionPerformed=exit)
        p.add(bQuit)
        panel2.add(p)
        p = JPanel()
        bRestart = JButton("Restart", actionPerformed=self.restart)
        p.add(bRestart)
        panel2.add(p)
        for i in range(len(gmenu)/2):
            label = gmenu[i*2]
            content = gmenu[i*2+1]
            if content == "#label":
                p = JPanel()
                l = JLabel(label)
                p.add(l)
            else:
                cp = commandPanel(self, label, content, False)
                p = cp.panel
            panel2.add(p)
        self.tabPane.addTab("Global", panel2)
        
        panel3 = JPanel()
        self.taHistory = JTextArea()
        sp = JScrollPane(self.taHistory)
        panel3.setLayout(BorderLayout())
        panel3.add(sp, BorderLayout.CENTER)
        self.tabPane.addTab("History", panel3)
        
        self.f.add(self.tabPane)
        self.f.setVisible(True)
    def restart(self, event):
        self.pageExec = 0
        self.page = 0
        self.setupPage()
        self.tabPane.setSelectedIndex(0)
        return
    def isSequential(self):
        return self.cbSequential.isSelected()
    def setupPage(self, event=None):
        self.localMenuPanel.removeAll()
        pmenu = self.lmenu[self.page]
        for i in range(len(pmenu)/2):
            cp = commandPanel(self, pmenu[i*2], pmenu[i*2+1], i==0)
            if self.page != self.pageExec and self.isSequential():
                cp.bCmd.setEnabled(False)
            else:
                cp.bCmd.setEnabled(True)
            self.localMenuPanel.add(cp.panel)
            if cp.continuous and self.page == self.pageExec:
                cp.processEvent(event)
        self.f.setVisible(True)
        self.lPage.setText(str(self.page+1)+"/"+str(len(self.lmenu)))
        if self.page == 0:
            self.bPrev.setEnabled(False)
        else:
            self.bPrev.setEnabled(True)
        if self.page == len(self.lmenu)-1:
            self.bNext.setEnabled(False)
            self.bSkip.setEnabled(False)
        else:
            self.bNext.setEnabled(True)
            self.bSkip.setEnabled(True)
        for c in self.localMenuPanel.getComponents():
            c.setBackground(self.color)
        return
    def advancePage(self, event=None):
        if self.pageExec < len(self.lmenu)-1:
            self.pageExec += 1
            self.page += 1
            self.setupPage()
        elif self.pageExec == len(self.lmenu)-1:
            self.tabPane.setSelectedIndex(1)
        return
    def nextPage(self, event):
        self.page += 1
        self.setupPage()
        return
    def prevPage(self, event):
        self.page -= 1
        self.setupPage()
        return
    def addHistory(self,msg):
        dstr = datetime.datetime.today().strftime("%Y-%m-%d %H:%M:%S")
        str = dstr+":"+msg+"\n"
        self.taHistory.append(str)
        return

def waitInputMenu(menu):
    global wimf
    wimf = waitInputMenuFrame(menu)

def setMenuColor(c):
    wimf.color = c
    for comp in wimf.localMenuPanel.getComponents():
        comp.setBackground(c)


__builtin__.waitInputConfirm = waitInputConfirm
__builtin__.waitInputSelect  = waitInputSelect
__builtin__.waitInputMenu    = waitInputMenu
